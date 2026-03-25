#pragma once

#include "driver/i2c.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================
//  drv_vl53l0x.h — VL53L0X Time-of-Flight sensor driver
//  I2C Bus 0, 3 sensors with XSHUT address assignment
//  Default address: 0x29, reassigned via XSHUT sequence
// =============================================================

#define VL53L0X_DEFAULT_ADDR    0x29
#define VL53L0X_MAX_RANGE_MM    2000
#define VL53L0X_OUT_OF_RANGE    0xFFFF   // Returned when no target

typedef struct {
    i2c_port_t  port;
    uint8_t     addr;          // Assigned address
    uint8_t     stop_variable; // Read during init, needed for ranging
} vl53l0x_dev_t;

// ── Init ─────────────────────────────────────────────────────
// Initialize I2C bus 0 for ToF sensors
esp_err_t drv_vl53l0x_i2c_init(void);

// Initialize a single sensor at given address
// Call after bringing XSHUT high and assigning address
esp_err_t drv_vl53l0x_init(vl53l0x_dev_t* dev);

// ── Read ─────────────────────────────────────────────────────
// Read distance in mm. Returns VL53L0X_OUT_OF_RANGE if no target.
esp_err_t drv_vl53l0x_read_mm(vl53l0x_dev_t* dev, uint16_t* mm_out);

// ── Address assignment ────────────────────────────────────────
// Change I2C address of sensor currently at old_addr to new_addr
esp_err_t drv_vl53l0x_set_address(vl53l0x_dev_t* dev, uint8_t new_addr);

#ifdef __cplusplus
}
#endif