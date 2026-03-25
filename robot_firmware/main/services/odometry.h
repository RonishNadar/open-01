#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================
//  odometry.h — Wheel odometry pose integration
//  Pure math — no hardware dependencies
// =============================================================

typedef struct {
    float x;        // meters
    float y;        // meters
    float theta;    // radians
} pose_t;

typedef struct {
    pose_t  pose;
    float   linear_ms;   // current linear velocity m/s
    float   angular_rs;  // current angular velocity rad/s
} odom_t;

// Initialize odometry (zero pose)
void odom_init(odom_t* odom);

// Reset pose to zero
void odom_reset(odom_t* odom);

// Update odometry from wheel encoder deltas
// delta_left_m, delta_right_m: distance each wheel travelled (meters)
// dt: time step in seconds
void odom_update(odom_t* odom,
                 float delta_left_m,
                 float delta_right_m,
                 float wheel_separation_m,
                 float dt);

#ifdef __cplusplus
}
#endif
