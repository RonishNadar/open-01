#include "drv_vl53l0x.h"
#include "config/config.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "DRV_VL53L0X";

// =============================================================
//  drv_vl53l0x.cpp
//  Based on Pololu VL53L0X Arduino library (MIT license)
//  https://github.com/pololu/vl53l0x-arduino
//  Full ST init sequence, single-shot ranging with polling
// =============================================================

#define TIMEOUT_MS   500
#define T pdMS_TO_TICKS(TIMEOUT_MS)

// ── Register definitions (Pololu) ─────────────────────────────
#define SYSRANGE_START                              0x00
#define SYSTEM_THRESH_HIGH                          0x0C
#define SYSTEM_THRESH_LOW                           0x0E
#define SYSTEM_SEQUENCE_CONFIG                      0x01
#define SYSTEM_RANGE_CONFIG                         0x09
#define SYSTEM_INTERMEASUREMENT_PERIOD              0x04
#define SYSTEM_INTERRUPT_CONFIG_GPIO                0x0A
#define GPIO_HV_MUX_ACTIVE_HIGH                     0x84
#define SYSTEM_INTERRUPT_CLEAR                      0x0B
#define RESULT_INTERRUPT_STATUS                     0x13
#define RESULT_RANGE_STATUS                         0x14
#define RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       0xBC
#define RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        0xC0
#define RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       0xD0
#define RESULT_CORE_RANGING_TOTAL_EVENTS_REF        0xD4
#define RESULT_PEAK_SIGNAL_RATE_REF                 0xB6
#define ALGO_PART_TO_PART_RANGE_OFFSET_MM           0x28
#define I2C_SLAVE_DEVICE_ADDRESS                    0x8A
#define MSRC_CONFIG_CONTROL                         0x60
#define PRE_RANGE_CONFIG_MIN_SNR                    0x27
#define PRE_RANGE_CONFIG_VALID_PHASE_LOW            0x56
#define PRE_RANGE_CONFIG_VALID_PHASE_HIGH           0x57
#define PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          0x64
#define FINAL_RANGE_CONFIG_MIN_SNR                  0x67
#define FINAL_RANGE_CONFIG_VALID_PHASE_LOW          0x47
#define FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         0x48
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44
#define PRE_RANGE_CONFIG_SIGMA_THRESH_HI            0x61
#define PRE_RANGE_CONFIG_SIGMA_THRESH_LO            0x62
#define PRE_RANGE_CONFIG_VCSEL_PERIOD               0x50
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          0x51
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          0x52
#define SYSTEM_HISTOGRAM_BIN                        0x81
#define HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       0x33
#define HISTOGRAM_CONFIG_READOUT_CTRL               0x55
#define FINAL_RANGE_CONFIG_VCSEL_PERIOD             0x70
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        0x71
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        0x72
#define CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       0x20
#define MSRC_CONFIG_TIMEOUT_MACROP                  0x46
#define SOFT_RESET_GO2_SOFT_RESET_N                 0xBF
#define IDENTIFICATION_MODEL_ID                     0xC0
#define IDENTIFICATION_REVISION_ID                  0xC2
#define OSC_CALIBRATE_VAL                           0xF8
#define GLOBAL_CONFIG_VCSEL_WIDTH                   0x32
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0            0xB0
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_1            0xB1
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_2            0xB2
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_3            0xB3
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_4            0xB4
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_5            0xB5
#define GLOBAL_CONFIG_REF_EN_START_SELECT           0xB6
#define DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         0x4E
#define DYNAMIC_SPAD_REF_EN_START_OFFSET            0x4F
#define POWER_MANAGEMENT_GO1_POWER_FORCE            0x80
#define VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           0x89
#define ALGO_PHASECAL_LIM                           0x30
#define ALGO_PHASECAL_CONFIG_TIMEOUT                0x30

// ── I2C helpers ───────────────────────────────────────────────
static esp_err_t w8(i2c_port_t p, uint8_t a, uint8_t r, uint8_t v) {
    uint8_t b[2] = {r, v};
    return i2c_master_write_to_device(p, a, b, 2, T);
}
static esp_err_t w16(i2c_port_t p, uint8_t a, uint8_t r, uint16_t v) {
    uint8_t b[3] = {r, (uint8_t)(v>>8), (uint8_t)(v&0xFF)};
    return i2c_master_write_to_device(p, a, b, 3, T);
}
static esp_err_t r8(i2c_port_t p, uint8_t a, uint8_t r, uint8_t* v) {
    return i2c_master_write_read_device(p, a, &r, 1, v, 1, T);
}
static esp_err_t r16(i2c_port_t p, uint8_t a, uint8_t r, uint16_t* v) {
    uint8_t b[2];
    esp_err_t ret = i2c_master_write_read_device(p, a, &r, 1, b, 2, T);
    if (ret == ESP_OK) *v = (uint16_t)((b[0]<<8)|b[1]);
    return ret;
}
// Write multiple bytes: reg, data[0..n-1]
static esp_err_t wN(i2c_port_t p, uint8_t a, uint8_t r,
                    const uint8_t* data, size_t n) {
    uint8_t buf[64];
    if (n+1 > sizeof(buf)) return ESP_ERR_INVALID_SIZE;
    buf[0] = r;
    for (size_t i = 0; i < n; i++) buf[i+1] = data[i];
    return i2c_master_write_to_device(p, a, buf, n+1, T);
}

// ── I2C bus init ──────────────────────────────────────────────
esp_err_t drv_vl53l0x_i2c_init(void) {
    i2c_config_t cfg = {};
    cfg.mode             = I2C_MODE_MASTER;
    cfg.sda_io_num       = TOF_SDA;
    cfg.scl_io_num       = TOF_SCL;
    cfg.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    cfg.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    cfg.master.clk_speed = TOF_I2C_FREQ_HZ;
    esp_err_t ret = i2c_param_config(TOF_I2C_PORT, &cfg);
    if (ret != ESP_OK) return ret;
    ret = i2c_driver_install(TOF_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "ToF I2C initialized");
    return ESP_OK;
}

// ── Address change ────────────────────────────────────────────
esp_err_t drv_vl53l0x_set_address(vl53l0x_dev_t* dev, uint8_t new_addr) {
    esp_err_t ret = w8(dev->port, dev->addr,
                       I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Addr 0x%02X → 0x%02X", dev->addr, new_addr);
        dev->addr = new_addr;
    }
    return ret;
}

// ── Full Pololu init sequence ─────────────────────────────────
esp_err_t drv_vl53l0x_init(vl53l0x_dev_t* dev) {
    esp_err_t ret;
    uint8_t   v8;

    i2c_port_t p = dev->port;
    uint8_t    a = dev->addr;

    // Verify chip — reference register 0xC0 should be 0xEE
    ret = r8(p, a, IDENTIFICATION_MODEL_ID, &v8);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "[0x%02X] Model ID read failed: %d", a, ret);
        return ret;
    }
    ESP_LOGI(TAG, "[0x%02X] Model ID: 0x%02X %s",
             a, v8, v8 == 0xEE ? "OK" : "(unexpected)");

    // Set 2v8 mode
    ret = r8(p, a, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, &v8);
    if (ret != ESP_OK) return ret;
    w8(p, a, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, v8 | 0x01);

    // Standard init — Pololu sequence
    w8(p, a, 0x88, 0x00);
    w8(p, a, 0x80, 0x01);
    w8(p, a, 0xFF, 0x01);
    w8(p, a, 0x00, 0x00);
    r8(p, a, 0x91, &dev->stop_variable);
    w8(p, a, 0x00, 0x01);
    w8(p, a, 0xFF, 0x00);
    w8(p, a, 0x80, 0x00);

    // Disable SIGNAL_RATE_MSRC and SIGNAL_RATE_PRE_RANGE checks
    r8(p, a, MSRC_CONFIG_CONTROL, &v8);
    w8(p, a, MSRC_CONFIG_CONTROL, v8 | 0x12);

    // Set final range signal rate limit to 0.25 MCPS (2^9 = 512 → 0.25 * 65536 = 16384)
    w16(p, a, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 32);

    w8(p, a, SYSTEM_SEQUENCE_CONFIG, 0xFF);

    // SPAD init
    uint8_t spad_count;
    bool    spad_type_is_aperture;
    {
        uint8_t tmp;
        w8(p, a, 0x80, 0x01);
        w8(p, a, 0xFF, 0x01);
        w8(p, a, 0x00, 0x00);
        w8(p, a, 0xFF, 0x06);
        r8(p, a, 0x83, &tmp);
        w8(p, a, 0x83, tmp | 0x04);
        w8(p, a, 0xFF, 0x07);
        w8(p, a, 0x81, 0x01);
        w8(p, a, 0x80, 0x01);
        w8(p, a, 0x94, 0x6B);
        w8(p, a, 0x83, 0x00);

        uint32_t t2 = 1000;
        do { r8(p, a, 0x83, &tmp); vTaskDelay(1); } while (tmp == 0x00 && --t2);

        w8(p, a, 0x83, 0x01);
        r8(p, a, 0x92, &tmp);
        spad_count = tmp & 0x7F;
        spad_type_is_aperture = (tmp >> 7) & 0x01;

        w8(p, a, 0x81, 0x00);
        w8(p, a, 0xFF, 0x06);
        r8(p, a, 0x83, &tmp);
        w8(p, a, 0x83, tmp & ~0x04);
        w8(p, a, 0xFF, 0x01);
        w8(p, a, 0x00, 0x01);
        w8(p, a, 0xFF, 0x00);
        w8(p, a, 0x80, 0x00);
    }

    // Enable reference SPADs
    {
        static const uint8_t spad_enables_ref[6] = {
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
        };
        uint8_t ref_spad_map[6];
        w8(p, a, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);
        uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0;
        uint8_t spads_enabled = 0;
        for (int i = 0; i < 48; i++) {
            if (i < first_spad_to_enable || spads_enabled == spad_count) {
                ref_spad_map[i/8] &= ~(1 << (i%8));
            } else if ((spad_enables_ref[i/8] >> (i%8)) & 0x1) {
                ref_spad_map[i/8] |= (1 << (i%8));
                spads_enabled++;
            }
        }
        // Read current map
        uint8_t cur[6];
        uint8_t reg = GLOBAL_CONFIG_SPAD_ENABLES_REF_0;
        i2c_master_write_read_device(p, a, &reg, 1, cur, 6, T);
        wN(p, a, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
    }

    // Load default tuning settings (Pololu)
    w8(p,a,0xFF,0x01); w8(p,a,0x00,0x00);
    w8(p,a,0xFF,0x00); w8(p,a,0x09,0x00);
    w8(p,a,0x10,0x00); w8(p,a,0x11,0x00);
    w8(p,a,0x24,0x01); w8(p,a,0x25,0xFF);
    w8(p,a,0x75,0x00); w8(p,a,0xFF,0x01);
    w8(p,a,0x4E,0x2C); w8(p,a,0x48,0x00);
    w8(p,a,0x30,0x20); w8(p,a,0xFF,0x00);
    w8(p,a,0x30,0x09); w8(p,a,0x54,0x00);
    w8(p,a,0x31,0x04); w8(p,a,0x32,0x03);
    w8(p,a,0x40,0x83); w8(p,a,0x46,0x25);
    w8(p,a,0x60,0x00); w8(p,a,0x27,0x00);
    w8(p,a,0x50,0x06); w8(p,a,0x51,0x00);
    w8(p,a,0x52,0x96); w8(p,a,0x56,0x08);
    w8(p,a,0x57,0x30); w8(p,a,0x61,0x00);
    w8(p,a,0x62,0x00); w8(p,a,0x64,0x00);
    w8(p,a,0x65,0x00); w8(p,a,0x66,0xA0);
    w8(p,a,0xFF,0x01); w8(p,a,0x22,0x32);
    w8(p,a,0x47,0x14); w8(p,a,0x49,0xFF);
    w8(p,a,0x4A,0x00); w8(p,a,0xFF,0x00);
    w8(p,a,0x7A,0x0A); w8(p,a,0x7B,0x00);
    w8(p,a,0x78,0x21); w8(p,a,0xFF,0x01);
    w8(p,a,0x23,0x34); w8(p,a,0x42,0x00);
    w8(p,a,0x44,0xFF); w8(p,a,0x45,0x26);
    w8(p,a,0x46,0x05); w8(p,a,0x40,0x40);
    w8(p,a,0x0E,0x06); w8(p,a,0x20,0x1A);
    w8(p,a,0x43,0x40); w8(p,a,0xFF,0x00);
    w8(p,a,0x34,0x03); w8(p,a,0x35,0x44);
    w8(p,a,0xFF,0x01); w8(p,a,0x31,0x04);
    w8(p,a,0x4B,0x09); w8(p,a,0x4C,0x05);
    w8(p,a,0x4D,0x04); w8(p,a,0xFF,0x00);
    w8(p,a,0x44,0x00); w8(p,a,0x45,0x20);
    w8(p,a,0x47,0x08); w8(p,a,0x48,0x28);
    w8(p,a,0x67,0x00); w8(p,a,0x70,0x04);
    w8(p,a,0x71,0x01); w8(p,a,0x72,0xFE);
    w8(p,a,0x76,0x00); w8(p,a,0x77,0x00);
    w8(p,a,0xFF,0x01); w8(p,a,0x0D,0x01);
    w8(p,a,0xFF,0x00); w8(p,a,0x80,0x01);
    w8(p,a,0x01,0xF8); w8(p,a,0xFF,0x01);
    w8(p,a,0x8E,0x01); w8(p,a,0x00,0x01);
    w8(p,a,0xFF,0x00); w8(p,a,0x80,0x00);

    // Set interrupt config
    w8(p, a, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    uint8_t gpio_hv;
    r8(p, a, GPIO_HV_MUX_ACTIVE_HIGH, &gpio_hv);
    w8(p, a, GPIO_HV_MUX_ACTIVE_HIGH, gpio_hv & ~0x10);
    w8(p, a, SYSTEM_INTERRUPT_CLEAR, 0x01);

    // Set sequence: DSS, pre-range, final-range
    w8(p, a, SYSTEM_SEQUENCE_CONFIG, 0xE8);

    // VHV + PhaseCal
    w8(p, a, SYSTEM_SEQUENCE_CONFIG, 0x01);
    {
        // Perform VHV calibration
        w8(p, a, SYSRANGE_START, 0x01);
        uint32_t t3 = 1000;
        uint8_t  tmp;
        do { r8(p, a, RESULT_INTERRUPT_STATUS, &tmp); vTaskDelay(1); }
        while ((tmp & 0x07) == 0 && --t3);
        w8(p, a, SYSTEM_INTERRUPT_CLEAR, 0x01);
        w8(p, a, SYSRANGE_START, 0x00);
    }
    w8(p, a, SYSTEM_SEQUENCE_CONFIG, 0x02);
    {
        // Perform PhaseCal
        w8(p, a, SYSRANGE_START, 0x01);
        uint32_t t3 = 1000;
        uint8_t  tmp;
        do { r8(p, a, RESULT_INTERRUPT_STATUS, &tmp); vTaskDelay(1); }
        while ((tmp & 0x07) == 0 && --t3);
        w8(p, a, SYSTEM_INTERRUPT_CLEAR, 0x01);
        w8(p, a, SYSRANGE_START, 0x00);
    }

    // Restore sequence
    w8(p, a, SYSTEM_SEQUENCE_CONFIG, 0xE8);

    ESP_LOGI(TAG, "[0x%02X] VL53L0X ready (stop_var=0x%02X)",
             a, dev->stop_variable);
    return ESP_OK;
}

// ── Single-shot range read ────────────────────────────────────
esp_err_t drv_vl53l0x_read_mm(vl53l0x_dev_t* dev, uint16_t* mm_out) {
    i2c_port_t p = dev->port;
    uint8_t    a = dev->addr;
    uint8_t    v8;
    uint16_t   range;

    // Start single measurement
    w8(p, a, 0x80, 0x01);
    w8(p, a, 0xFF, 0x01);
    w8(p, a, 0x00, 0x00);
    w8(p, a, 0x91, dev->stop_variable);
    w8(p, a, 0x00, 0x01);
    w8(p, a, 0xFF, 0x00);
    w8(p, a, 0x80, 0x00);
    w8(p, a, SYSRANGE_START, 0x01);

    // Wait for SYSRANGE_START bit 0 to clear
    uint32_t t = 500;
    do {
        esp_err_t ret = r8(p, a, SYSRANGE_START, &v8);
        if (ret != ESP_OK) { *mm_out = VL53L0X_OUT_OF_RANGE; return ret; }
        vTaskDelay(1);
    } while ((v8 & 0x01) && --t);
    if (!t) { *mm_out = VL53L0X_OUT_OF_RANGE; return ESP_ERR_TIMEOUT; }

    // Wait for measurement ready
    t = 500;
    do {
        esp_err_t ret = r8(p, a, RESULT_INTERRUPT_STATUS, &v8);
        if (ret != ESP_OK) { *mm_out = VL53L0X_OUT_OF_RANGE; return ret; }
        vTaskDelay(1);
    } while ((v8 & 0x07) == 0 && --t);
    if (!t) { *mm_out = VL53L0X_OUT_OF_RANGE; return ESP_ERR_TIMEOUT; }

    // Read range at RESULT_RANGE_STATUS + 10
    esp_err_t ret = r16(p, a, (uint8_t)(RESULT_RANGE_STATUS + 10), &range);
    w8(p, a, SYSTEM_INTERRUPT_CLEAR, 0x01);

    if (ret != ESP_OK) { *mm_out = VL53L0X_OUT_OF_RANGE; return ret; }

    *mm_out = (range == 0 || range > VL53L0X_MAX_RANGE_MM)
              ? VL53L0X_OUT_OF_RANGE : range;
    return ESP_OK;
}