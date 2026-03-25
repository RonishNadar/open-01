#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================
//  hal_imu.h — IMU hardware abstraction layer
//  Wraps drv_mpu6500, returns calibrated SI units
// =============================================================

typedef struct {
    float accel_x;   // m/s²
    float accel_y;
    float accel_z;
    float gyro_x;    // deg/s
    float gyro_y;
    float gyro_z;
} imu_data_t;

// Initialize IMU — call once at boot
esp_err_t hal_imu_init(void);

// Read latest IMU data in SI units
// accel in m/s², gyro in deg/s
esp_err_t hal_imu_read(imu_data_t* out);

#ifdef __cplusplus
}
#endif
