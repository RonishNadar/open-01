#include "kinematic.h"
#include "config/config.h"

// =============================================================
//  kinematic.cpp — Differential drive kinematics
// =============================================================

void kinematic_cmd_to_wheels(const cmd_vel_t* cmd,
                              wheel_vel_t* out,
                              float wheel_separation_m) {
    float half_sep = wheel_separation_m / 2.0f;
    out->left_ms  = cmd->linear_ms - (cmd->angular_rs * half_sep);
    out->right_ms = cmd->linear_ms + (cmd->angular_rs * half_sep);

    // Clamp to max velocity
    if (out->left_ms  >  MAX_VEL_MS) out->left_ms  =  MAX_VEL_MS;
    if (out->left_ms  < -MAX_VEL_MS) out->left_ms  = -MAX_VEL_MS;
    if (out->right_ms >  MAX_VEL_MS) out->right_ms =  MAX_VEL_MS;
    if (out->right_ms < -MAX_VEL_MS) out->right_ms = -MAX_VEL_MS;
}

void kinematic_wheels_to_cmd(const wheel_vel_t* wheels,
                              cmd_vel_t* out,
                              float wheel_separation_m) {
    out->linear_ms  = (wheels->right_ms + wheels->left_ms) / 2.0f;
    out->angular_rs = (wheels->right_ms - wheels->left_ms)
                      / wheel_separation_m;
}
