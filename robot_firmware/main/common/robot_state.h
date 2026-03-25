#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// =============================================================
//  robot_state.h — Shared state between all FreeRTOS tasks
//  Access only via robot_state_get() / robot_state_set_*()
//  Never read/write the struct directly from tasks.
// =============================================================

// ── ToF distances ────────────────────────────────────────────
typedef struct {
    uint16_t left_mm;
    uint16_t back_mm;
    uint16_t right_mm;
    bool     valid;
} tof_state_t;

// ── IMU data ─────────────────────────────────────────────────
typedef struct {
    float accel_x;    // m/s²
    float accel_y;
    float accel_z;
    float gyro_x;     // deg/s
    float gyro_y;
    float gyro_z;
    bool  valid;
} imu_state_t;

// ── Battery ──────────────────────────────────────────────────
typedef struct {
    float voltage;    // Volts
    bool  valid;
} battery_state_t;

// ── Motor / Odometry ─────────────────────────────────────────
typedef struct {
    // Commanded target velocities (set by comms task from cmd_vel)
    float target_left_ms;    // m/s, signed
    float target_right_ms;

    // Measured velocities (set by motor task from encoder)
    float measured_left_ms;
    float measured_right_ms;

    // Odometry pose (set by motor task)
    float x;           // meters
    float y;           // meters
    float theta;       // radians

    // Odometry velocity (for publishing)
    float linear_ms;   // m/s
    float angular_rs;  // rad/s
} motor_state_t;

// ── Full robot state ─────────────────────────────────────────
typedef struct {
    tof_state_t     tof;
    imu_state_t     imu;
    battery_state_t battery;
    motor_state_t   motor;
    int64_t         timestamp_us;   // esp_timer_get_time()
} robot_state_t;

// ── Public API ───────────────────────────────────────────────
#ifdef __cplusplus
extern "C" {
#endif

void robot_state_init(void);

// Setters — each locks the mutex internally
void robot_state_set_tof(const tof_state_t* tof);
void robot_state_set_imu(const imu_state_t* imu);
void robot_state_set_battery(const battery_state_t* battery);
void robot_state_set_motor(const motor_state_t* motor);
void robot_state_set_target_vel(float left_ms, float right_ms);

// Getters — each locks the mutex internally
void robot_state_get(robot_state_t* out);
void robot_state_get_tof(tof_state_t* out);
void robot_state_get_imu(imu_state_t* out);
void robot_state_get_battery(battery_state_t* out);
void robot_state_get_motor(motor_state_t* out);
void robot_state_get_target_vel(float* left_ms, float* right_ms);

#ifdef __cplusplus
}
#endif
