#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================
//  hal_battery.h — Battery voltage HAL
//  Reads ADC on GPIO2, applies voltage divider + calibration
//
//  Voltage divider:
//  VBAT ── R1(10k) ── ADC ── R2(2.55k) ── GND
//  V_adc = VBAT × R2 / (R1 + R2)
//  VBAT  = V_adc × (R1 + R2) / R2
// =============================================================

esp_err_t hal_battery_init(void);
float     hal_battery_read_voltage(void);  // Returns voltage in volts

#ifdef __cplusplus
}
#endif
