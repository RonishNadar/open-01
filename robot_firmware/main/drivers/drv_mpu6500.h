#pragma once

#include "driver/i2c.h"
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================
//  drv_mpu6500.h — MPU6500 raw register driver
//  I2C Bus 1, address 0x68
//  No fusion, no filtering — raw accel/gyro reads only
// =============================================================

// ── MPU6500 Register Map ─────────────────────────────────────
#define MPU6500_REG_SMPLRT_DIV      0x19
#define MPU6500_REG_CONFIG          0x1A
#define MPU6500_REG_GYRO_CONFIG     0x1B
#define MPU6500_REG_ACCEL_CONFIG    0x1C
#define MPU6500_REG_ACCEL_CONFIG2   0x1D
#define MPU6500_REG_INT_PIN_CFG     0x37
#define MPU6500_REG_INT_ENABLE      0x38
#define MPU6500_REG_ACCEL_XOUT_H    0x3B
#define MPU6500_REG_GYRO_XOUT_H     0x43
#define MPU6500_REG_PWR_MGMT_1      0x6B
#define MPU6500_REG_PWR_MGMT_2      0x6C
#define MPU6500_REG_WHO_AM_I        0x75

#define MPU6500_WHO_AM_I_VAL        0x70   // Expected WHO_AM_I response

// ── Accel full-scale range ────────────────────────────────────
typedef enum {
    MPU6500_ACCEL_FS_2G  = 0,   // ±2g,  sensitivity 16384 LSB/g
    MPU6500_ACCEL_FS_4G  = 1,   // ±4g,  sensitivity 8192  LSB/g
    MPU6500_ACCEL_FS_8G  = 2,   // ±8g,  sensitivity 4096  LSB/g
    MPU6500_ACCEL_FS_16G = 3,   // ±16g, sensitivity 2048  LSB/g
} mpu6500_accel_fs_t;

// ── Gyro full-scale range ─────────────────────────────────────
typedef enum {
    MPU6500_GYRO_FS_250  = 0,   // ±250  deg/s, sensitivity 131   LSB/deg/s
    MPU6500_GYRO_FS_500  = 1,   // ±500  deg/s, sensitivity 65.5  LSB/deg/s
    MPU6500_GYRO_FS_1000 = 2,   // ±1000 deg/s, sensitivity 32.8  LSB/deg/s
    MPU6500_GYRO_FS_2000 = 3,   // ±2000 deg/s, sensitivity 16.4  LSB/deg/s
} mpu6500_gyro_fs_t;

// ── Raw sensor data ───────────────────────────────────────────
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu6500_raw_t;

// ── Init ─────────────────────────────────────────────────────
// Initializes I2C bus, verifies WHO_AM_I, configures ranges
esp_err_t drv_mpu6500_init(mpu6500_accel_fs_t accel_fs,
                            mpu6500_gyro_fs_t  gyro_fs);

// ── Read ─────────────────────────────────────────────────────
// Burst-reads all 6 axes in one I2C transaction (12 bytes)
esp_err_t drv_mpu6500_read(mpu6500_raw_t* out);

// ── Sensitivity scalars ───────────────────────────────────────
float drv_mpu6500_accel_scale(mpu6500_accel_fs_t fs); // returns LSB/g divisor
float drv_mpu6500_gyro_scale(mpu6500_gyro_fs_t fs);   // returns LSB/deg/s divisor

#ifdef __cplusplus
}
#endif
