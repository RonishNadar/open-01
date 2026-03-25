#include "drv_mpu6500.h"
#include "config/config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "DRV_MPU6500";

// =============================================================
//  drv_mpu6500.cpp — MPU6500 I2C driver
//  Burst reads accel + gyro in one 12-byte transaction
// =============================================================

#define I2C_TIMEOUT_MS   50
#define I2C_TIMEOUT_TICKS pdMS_TO_TICKS(I2C_TIMEOUT_MS)

// ── Low-level I2C helpers ─────────────────────────────────────
static esp_err_t write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(
        IMU_I2C_PORT, MPU_ADDR, buf, 2, I2C_TIMEOUT_TICKS);
}

static esp_err_t read_regs(uint8_t reg, uint8_t* data, size_t len) {
    return i2c_master_write_read_device(
        IMU_I2C_PORT, MPU_ADDR,
        &reg, 1,
        data, len,
        I2C_TIMEOUT_TICKS);
}

// ── Init ─────────────────────────────────────────────────────
esp_err_t drv_mpu6500_init(mpu6500_accel_fs_t accel_fs,
                            mpu6500_gyro_fs_t  gyro_fs) {
    esp_err_t ret;

    // Configure I2C bus
    i2c_config_t cfg = {};
    cfg.mode             = I2C_MODE_MASTER;
    cfg.sda_io_num       = MPU_SDA;
    cfg.scl_io_num       = MPU_SCL;
    cfg.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    cfg.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    cfg.master.clk_speed = MPU_I2C_FREQ_HZ;

    ret = i2c_param_config(IMU_I2C_PORT, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %d", ret);
        return ret;
    }

    ret = i2c_driver_install(IMU_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %d", ret);
        return ret;
    }

    // Small delay after bus init
    vTaskDelay(pdMS_TO_TICKS(50));

    // WHO_AM_I check
    uint8_t who_am_i = 0;
    ret = read_regs(MPU6500_REG_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I read failed: %d", ret);
        return ret;
    }
    if (who_am_i != MPU6500_WHO_AM_I_VAL) {
        ESP_LOGW(TAG, "WHO_AM_I = 0x%02X (expected 0x%02X) — continuing anyway",
                 who_am_i, MPU6500_WHO_AM_I_VAL);
    } else {
        ESP_LOGI(TAG, "WHO_AM_I = 0x%02X OK", who_am_i);
    }

    // Reset I2C bus between transactions — prevents stuck bus after read
    i2c_reset_tx_fifo(IMU_I2C_PORT);
    i2c_reset_rx_fifo(IMU_I2C_PORT);
    vTaskDelay(pdMS_TO_TICKS(20));

    // Wake up — clear sleep bit, use PLL clock
    ret = write_reg(MPU6500_REG_PWR_MGMT_1, 0x01);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "PWR_MGMT_1 failed"); return ret; }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Enable all axes
    ret = write_reg(MPU6500_REG_PWR_MGMT_2, 0x00);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "PWR_MGMT_2 failed"); return ret; }

    // Sample rate divider — 0 = run at full gyro rate (1kHz)
    ret = write_reg(MPU6500_REG_SMPLRT_DIV, 0x00);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "SMPLRT_DIV failed"); return ret; }

    // DLPF config — 92Hz bandwidth (smooth but responsive)
    ret = write_reg(MPU6500_REG_CONFIG, 0x02);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "CONFIG failed"); return ret; }

    // Gyro full scale
    ret = write_reg(MPU6500_REG_GYRO_CONFIG, (uint8_t)(gyro_fs << 3));
    if (ret != ESP_OK) { ESP_LOGE(TAG, "GYRO_CONFIG failed"); return ret; }

    // Accel full scale
    ret = write_reg(MPU6500_REG_ACCEL_CONFIG, (uint8_t)(accel_fs << 3));
    if (ret != ESP_OK) { ESP_LOGE(TAG, "ACCEL_CONFIG failed"); return ret; }

    // Accel DLPF — 99Hz
    ret = write_reg(MPU6500_REG_ACCEL_CONFIG2, 0x02);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "ACCEL_CONFIG2 failed"); return ret; }

    ESP_LOGI(TAG, "MPU6500 initialized (accel=±%dg gyro=±%ddeg/s)",
             2 << accel_fs, 250 << gyro_fs);

    return ESP_OK;
}

// ── Read ─────────────────────────────────────────────────────
esp_err_t drv_mpu6500_read(mpu6500_raw_t* out) {
    // Burst read: ACCEL_XOUT_H through GYRO_ZOUT_L = 14 bytes
    // Bytes 0-5:  accel X,Y,Z (H,L pairs)
    // Bytes 6-7:  temperature (skip)
    // Bytes 8-13: gyro X,Y,Z (H,L pairs)
    uint8_t buf[14];
    esp_err_t ret = read_regs(MPU6500_REG_ACCEL_XOUT_H, buf, 14);
    if (ret != ESP_OK) return ret;

    out->accel_x = (int16_t)((buf[0]  << 8) | buf[1]);
    out->accel_y = (int16_t)((buf[2]  << 8) | buf[3]);
    out->accel_z = (int16_t)((buf[4]  << 8) | buf[5]);
    // buf[6..7] = temperature — skipped
    out->gyro_x  = (int16_t)((buf[8]  << 8) | buf[9]);
    out->gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
    out->gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);

    return ESP_OK;
}

// ── Scale factors ─────────────────────────────────────────────
float drv_mpu6500_accel_scale(mpu6500_accel_fs_t fs) {
    // Returns LSB/g — divide raw by this to get g
    const float scales[] = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
    return scales[fs];
}

float drv_mpu6500_gyro_scale(mpu6500_gyro_fs_t fs) {
    // Returns LSB/(deg/s) — divide raw by this to get deg/s
    const float scales[] = { 131.0f, 65.5f, 32.8f, 16.4f };
    return scales[fs];
}