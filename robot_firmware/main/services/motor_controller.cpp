#include "motor_controller.h"
#include "hal/hal_motor.h"
#include "common/robot_state.h"
#include "config/config.h"
#include "esp_log.h"
#include <math.h>

static const char* TAG = "MOTOR_CTRL";

// =============================================================
//  motor_controller.cpp
//  Per-tick:
//    1. Read FG encoder deltas
//    2. Compute measured velocity (m/s)
//    3. Run PID → output [-1, 1]
//    4. Drive PWM
//    5. Update odometry
//    6. Write to robot_state
// =============================================================

void motor_controller_init(motor_controller_t* mc) {
    pid_init(&mc->pid_left,
             PID_KP, PID_KI, PID_KD,
             PID_INTEGRAL_MAX, PID_OUTPUT_MAX);

    pid_init(&mc->pid_right,
             PID_KP, PID_KI, PID_KD,
             PID_INTEGRAL_MAX, PID_OUTPUT_MAX);

    odom_init(&mc->odom);

    mc->target_left_ms   = 0.0f;
    mc->target_right_ms  = 0.0f;
    mc->measured_left_ms = 0.0f;
    mc->measured_right_ms= 0.0f;

    ESP_LOGI(TAG, "Motor controller initialized (Kp=%.2f Ki=%.2f Kd=%.2f)",
             PID_KP, PID_KI, PID_KD);
}

void motor_controller_set_target(motor_controller_t* mc,
                                  float left_ms, float right_ms) {
    // Reset integrator if direction changes
    if ((left_ms  >= 0.0f) != (mc->target_left_ms  >= 0.0f)) pid_reset(&mc->pid_left);
    if ((right_ms >= 0.0f) != (mc->target_right_ms >= 0.0f)) pid_reset(&mc->pid_right);

    mc->target_left_ms  = left_ms;
    mc->target_right_ms = right_ms;
}

void motor_controller_update(motor_controller_t* mc, float dt) {
    // ── 1+2. Read velocity + distance in one atomic call ─────
    float dist_left_m  = 0.0f;
    float dist_right_m = 0.0f;
    mc->measured_left_ms  = hal_motor_get_velocity(MOTOR_LEFT,  dt, &dist_left_m);
    mc->measured_right_ms = hal_motor_get_velocity(MOTOR_RIGHT, dt, &dist_right_m);

    // ── 3. Feedforward + PID ──────────────────────────────────
    // Feedforward: normalize target velocity to PWM range
    // At max velocity (MAX_VEL_MS) we need output=1.0
    // This gives the motor a baseline PWM so PID only corrects small errors
    float out_left, out_right;

    if (fabsf(mc->target_left_ms) < MIN_VEL_MS) {
        hal_motor_stop(MOTOR_LEFT);
        pid_reset(&mc->pid_left);
        out_left = 0.0f;
    } else {
        float ff   = mc->target_left_ms / MAX_VEL_MS;
        float corr = pid_update(&mc->pid_left,
                                mc->target_left_ms,
                                mc->measured_left_ms, dt);
        out_left = ff + corr;
        if (out_left >  1.0f) out_left =  1.0f;
        if (out_left < -1.0f) out_left = -1.0f;
        hal_motor_set(MOTOR_LEFT, out_left);
    }

    if (fabsf(mc->target_right_ms) < MIN_VEL_MS) {
        hal_motor_stop(MOTOR_RIGHT);
        pid_reset(&mc->pid_right);
        out_right = 0.0f;
    } else {
        float ff   = mc->target_right_ms / MAX_VEL_MS;
        float corr = pid_update(&mc->pid_right,
                                mc->target_right_ms,
                                mc->measured_right_ms, dt);
        out_right = ff + corr;
        if (out_right >  1.0f) out_right =  1.0f;
        if (out_right < -1.0f) out_right = -1.0f;
        hal_motor_set(MOTOR_RIGHT, out_right);
    }

    // ── 4. Update odometry ────────────────────────────────────
    odom_update(&mc->odom,
                dist_left_m, dist_right_m,
                WHEEL_SEPARATION_M, dt);

    // ── 5. Write to shared robot state ───────────────────────
    motor_state_t state = {};
    state.target_left_ms   = mc->target_left_ms;
    state.target_right_ms  = mc->target_right_ms;
    state.measured_left_ms = mc->measured_left_ms;
    state.measured_right_ms= mc->measured_right_ms;
    state.x                = mc->odom.pose.x;
    state.y                = mc->odom.pose.y;
    state.theta            = mc->odom.pose.theta;
    state.linear_ms        = mc->odom.linear_ms;
    state.angular_rs       = mc->odom.angular_rs;
    robot_state_set_motor(&state);
}

void motor_controller_get_odom(const motor_controller_t* mc, odom_t* out) {
    *out = mc->odom;
}