#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================
//  hal_motor.h — Motor hardware abstraction layer
//  Wraps LEDC PWM + GPIO DIR + FG interrupt counting
//  for the DFRobot FIT0441 brushless motor
//
//  FIT0441 notes:
//   - PWM is INVERTED: 255 = stop, 0 = full speed
//   - FG pin outputs frequency pulses (not quadrature)
//   - 270 FG pulses per output shaft revolution
//   - Direction is software-tracked (no encoder direction)
// =============================================================

typedef enum {
    MOTOR_LEFT  = 0,
    MOTOR_RIGHT = 1,
    MOTOR_COUNT = 2
} motor_id_t;

// ── Init ─────────────────────────────────────────────────────
esp_err_t hal_motor_init(void);

void hal_motor_debug_delta(int32_t* left, int32_t* right);
// ── Control ──────────────────────────────────────────────────

// Set motor output — normalized -1.0 (full reverse) to +1.0 (full forward)
// 0.0 = stop. Sign = direction.
void hal_motor_set(motor_id_t motor, float output);

// Stop a motor immediately
void hal_motor_stop(motor_id_t motor);

// Stop both motors
void hal_motor_stop_all(void);

// ── Encoder ──────────────────────────────────────────────────
// Get raw cumulative FG count (never resets)
int32_t hal_motor_get_count(motor_id_t motor);

// ── Velocity + Distance — call ONCE per tick ─────────────────
// Reads delta, computes EMA velocity, outputs distance for odometry.
// dist_m_out: distance travelled this tick in meters (can be NULL)
// returns:    EMA-smoothed velocity in m/s, signed by direction
float hal_motor_get_velocity(motor_id_t motor, float dt, float* dist_m_out);

#ifdef __cplusplus
}
#endif