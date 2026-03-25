#pragma once

#include "services/pid.h"
#include "services/odometry.h"
#include "services/kinematic.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================
//  motor_controller.h — Velocity PID control + odometry
//  Called every TASK_MOTOR_MS by task_motor
// =============================================================

typedef struct {
    pid_t   pid_left;
    pid_t   pid_right;
    odom_t  odom;

    float   target_left_ms;    // m/s set by comms task
    float   target_right_ms;

    float   measured_left_ms;  // m/s from encoder
    float   measured_right_ms;
} motor_controller_t;

// Initialize — sets up PID gains and odometry
void motor_controller_init(motor_controller_t* mc);

// Set target wheel velocities (called from comms task via robot_state)
void motor_controller_set_target(motor_controller_t* mc,
                                  float left_ms, float right_ms);

// Run one control tick — call every TASK_MOTOR_MS
// Reads encoders, runs PID, drives PWM, updates odometry
void motor_controller_update(motor_controller_t* mc, float dt);

// Get current odometry
void motor_controller_get_odom(const motor_controller_t* mc, odom_t* out);

#ifdef __cplusplus
}
#endif
