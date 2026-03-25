#include "hal_tof.h"
#include "drivers/drv_vl53l0x.h"
#include "config/config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "HAL_TOF";

// =============================================================
//  hal_tof.cpp — 3x VL53L0X HAL
//
//  XSHUT address assignment sequence:
//  1. Pull all XSHUT LOW  → all sensors in reset
//  2. Bring LEFT HIGH     → left wakes at default 0x29
//  3. Change left to 0x30
//  4. Bring BACK HIGH     → back wakes at default 0x29
//  5. Change back to 0x31
//  6. Bring RIGHT HIGH    → right wakes at default 0x29
//  7. Change right to 0x32
// =============================================================

static vl53l0x_dev_t s_left  = { .port = TOF_I2C_PORT, .addr = ADDR_LEFT,  .stop_variable = 0 };
static vl53l0x_dev_t s_back  = { .port = TOF_I2C_PORT, .addr = ADDR_BACK,  .stop_variable = 0 };
static vl53l0x_dev_t s_right = { .port = TOF_I2C_PORT, .addr = ADDR_RIGHT, .stop_variable = 0 };

static void xshut_init_pins(void) {
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << XSHUT_LEFT) |
                       (1ULL << XSHUT_BACK) |
                       (1ULL << XSHUT_RIGHT);
    cfg.mode         = GPIO_MODE_OUTPUT;
    cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&cfg);
}

esp_err_t hal_tof_init(void) {
    esp_err_t ret;

    // Init I2C bus
    ret = drv_vl53l0x_i2c_init();
    if (ret != ESP_OK) return ret;

    // Init XSHUT pins
    xshut_init_pins();

    // Step 1 — pull all XSHUT LOW (reset all)
    gpio_set_level((gpio_num_t)XSHUT_LEFT,  0);
    gpio_set_level((gpio_num_t)XSHUT_BACK,  0);
    gpio_set_level((gpio_num_t)XSHUT_RIGHT, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Step 2+3 — bring LEFT up, assign address
    gpio_set_level((gpio_num_t)XSHUT_LEFT, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    s_left.addr = VL53L0X_DEFAULT_ADDR;
    ret = drv_vl53l0x_set_address(&s_left, ADDR_LEFT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEFT address assign failed: %d", ret);
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Step 4+5 — bring BACK up, assign address
    gpio_set_level((gpio_num_t)XSHUT_BACK, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    s_back.addr = VL53L0X_DEFAULT_ADDR;
    ret = drv_vl53l0x_set_address(&s_back, ADDR_BACK);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BACK address assign failed: %d", ret);
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Step 6+7 — bring RIGHT up, assign address
    gpio_set_level((gpio_num_t)XSHUT_RIGHT, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    s_right.addr = VL53L0X_DEFAULT_ADDR;
    ret = drv_vl53l0x_set_address(&s_right, ADDR_RIGHT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RIGHT address assign failed: %d", ret);
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Init each sensor
    ret = drv_vl53l0x_init(&s_left);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "LEFT init failed"); return ret; }

    ret = drv_vl53l0x_init(&s_back);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "BACK init failed"); return ret; }

    ret = drv_vl53l0x_init(&s_right);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "RIGHT init failed"); return ret; }

    ESP_LOGI(TAG, "All 3 ToF sensors initialized");
    return ESP_OK;
}

esp_err_t hal_tof_read(tof_reading_t* out) {
    // Read all 3 — don't abort on single sensor failure
    esp_err_t ret_l = drv_vl53l0x_read_mm(&s_left,  &out->left_mm);
    esp_err_t ret_b = drv_vl53l0x_read_mm(&s_back,  &out->back_mm);
    esp_err_t ret_r = drv_vl53l0x_read_mm(&s_right, &out->right_mm);

    if (ret_l != ESP_OK) out->left_mm  = 0xFFFF;
    if (ret_b != ESP_OK) out->back_mm  = 0xFFFF;
    if (ret_r != ESP_OK) out->right_mm = 0xFFFF;

    return ESP_OK;
}