#include "hal_imu.h"
#include "drivers/drv_mpu6500.h"
#include "esp_log.h"

static const char* TAG = "HAL_IMU";

// =============================================================
//  hal_imu.cpp — IMU HAL
//  Converts raw ADC counts → m/s² and deg/s
// =============================================================

// Using ±2g accel, ±500 deg/s gyro — good for a ground robot
#define IMU_ACCEL_FS   MPU6500_ACCEL_FS_2G
#define IMU_GYRO_FS    MPU6500_GYRO_FS_500

static float s_accel_scale = 1.0f;
static float s_gyro_scale  = 1.0f;

#define G_TO_MS2  9.80665f   // 1g in m/s²

esp_err_t hal_imu_init(void) {
    esp_err_t ret = drv_mpu6500_init(IMU_ACCEL_FS, IMU_GYRO_FS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6500 init failed: %d", ret);
        return ret;
    }

    s_accel_scale = drv_mpu6500_accel_scale(IMU_ACCEL_FS);
    s_gyro_scale  = drv_mpu6500_gyro_scale(IMU_GYRO_FS);

    ESP_LOGI(TAG, "IMU HAL initialized");
    return ESP_OK;
}

esp_err_t hal_imu_read(imu_data_t* out) {
    mpu6500_raw_t raw;
    esp_err_t ret = drv_mpu6500_read(&raw);
    if (ret != ESP_OK) return ret;

    // Convert to SI units
    out->accel_x = ((float)raw.accel_x / s_accel_scale) * G_TO_MS2;
    out->accel_y = ((float)raw.accel_y / s_accel_scale) * G_TO_MS2;
    out->accel_z = ((float)raw.accel_z / s_accel_scale) * G_TO_MS2;

    out->gyro_x  = (float)raw.gyro_x / s_gyro_scale;
    out->gyro_y  = (float)raw.gyro_y / s_gyro_scale;
    out->gyro_z  = (float)raw.gyro_z / s_gyro_scale;

    return ESP_OK;
}
