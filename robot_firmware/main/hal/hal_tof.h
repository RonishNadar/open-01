#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================
//  hal_tof.h — ToF sensor HAL
//  Manages 3x VL53L0X via XSHUT address assignment
// =============================================================

typedef struct {
    uint16_t left_mm;    // VL53L0X_OUT_OF_RANGE if no target
    uint16_t back_mm;
    uint16_t right_mm;
} tof_reading_t;

// Initialize all 3 ToF sensors via XSHUT sequence
esp_err_t hal_tof_init(void);

// Read all 3 sensors — returns ESP_OK even if some fail
esp_err_t hal_tof_read(tof_reading_t* out);

#ifdef __cplusplus
}
#endif
