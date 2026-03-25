#include "odometry.h"
#include <math.h>
#include <string.h>

// =============================================================
//  odometry.cpp — Wheel odometry pose integration
//  Uses midpoint heading for better accuracy
// =============================================================

void odom_init(odom_t* odom) {
    memset(odom, 0, sizeof(odom_t));
}

void odom_reset(odom_t* odom) {
    memset(odom, 0, sizeof(odom_t));
}

void odom_update(odom_t* odom,
                 float delta_left_m,
                 float delta_right_m,
                 float wheel_separation_m,
                 float dt) {

    // Distance and heading change
    float delta_dist  = (delta_left_m + delta_right_m) / 2.0f;
    float delta_theta = (delta_right_m - delta_left_m) / wheel_separation_m;

    // Midpoint heading for better accuracy
    float mid_theta = odom->pose.theta + delta_theta / 2.0f;

    // Integrate pose
    odom->pose.x     += delta_dist * cosf(mid_theta);
    odom->pose.y     += delta_dist * sinf(mid_theta);
    odom->pose.theta += delta_theta;

    // Normalize theta to [-π, π]
    while (odom->pose.theta >  (float)M_PI) odom->pose.theta -= 2.0f * (float)M_PI;
    while (odom->pose.theta < -(float)M_PI) odom->pose.theta += 2.0f * (float)M_PI;

    // Velocities
    if (dt > 0.0f) {
        odom->linear_ms  = delta_dist  / dt;
        odom->angular_rs = delta_theta / dt;
    }
}
