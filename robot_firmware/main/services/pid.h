#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================
//  pid.h — Generic PID controller
//  Pure math — no hardware, no FreeRTOS dependencies
//  One instance per wheel
// =============================================================

typedef struct {
    // Gains
    float kp;
    float ki;
    float kd;

    // State
    float integral;
    float prev_error;
    float output;

    // Limits
    float integral_max;   // Anti-windup clamp
    float output_max;     // Output clamp (normalized 0.0–1.0)
} pid_t;

// Initialize PID with gains and limits
void pid_init(pid_t* pid,
              float kp, float ki, float kd,
              float integral_max, float output_max);

// Reset integrator and state (call when motor stops or direction changes)
void pid_reset(pid_t* pid);

// Update PID — call every fixed dt seconds
// Returns output clamped to [-output_max, +output_max]
float pid_update(pid_t* pid, float target, float measured, float dt);

#ifdef __cplusplus
}
#endif
