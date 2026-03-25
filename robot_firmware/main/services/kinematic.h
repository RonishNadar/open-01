#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================
//  kinematic.h — Differential drive kinematics
//  Pure math — no hardware dependencies
// =============================================================

typedef struct {
    float linear_ms;    // m/s forward
    float angular_rs;   // rad/s rotation
} cmd_vel_t;

typedef struct {
    float left_ms;      // m/s
    float right_ms;     // m/s
} wheel_vel_t;

// cmd_vel → wheel velocities
// v_left  = linear - (angular × separation / 2)
// v_right = linear + (angular × separation / 2)
void kinematic_cmd_to_wheels(const cmd_vel_t* cmd,
                              wheel_vel_t* out,
                              float wheel_separation_m);

// wheel velocities → cmd_vel (inverse)
void kinematic_wheels_to_cmd(const wheel_vel_t* wheels,
                              cmd_vel_t* out,
                              float wheel_separation_m);

#ifdef __cplusplus
}
#endif
