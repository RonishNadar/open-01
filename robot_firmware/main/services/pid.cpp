#include "pid.h"
#include <string.h>

// =============================================================
//  pid.cpp — PID controller implementation
// =============================================================

static float clamp(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void pid_init(pid_t* pid,
              float kp, float ki, float kd,
              float integral_max, float output_max) {
    memset(pid, 0, sizeof(pid_t));
    pid->kp           = kp;
    pid->ki           = ki;
    pid->kd           = kd;
    pid->integral_max = integral_max;
    pid->output_max   = output_max;
}

void pid_reset(pid_t* pid) {
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->output     = 0.0f;
}

float pid_update(pid_t* pid, float target, float measured, float dt) {
    if (dt <= 0.0f) return pid->output;

    float error = target - measured;

    // Proportional
    float p = pid->kp * error;

    // Integral with anti-windup clamp
    pid->integral += error * dt;
    pid->integral  = clamp(pid->integral,
                           -pid->integral_max,
                            pid->integral_max);
    float i = pid->ki * pid->integral;

    // Derivative
    float d = pid->kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    // Output
    pid->output = clamp(p + i + d, -pid->output_max, pid->output_max);
    return pid->output;
}
