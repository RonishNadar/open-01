#include "robot_state.h"
#include "esp_timer.h"
#include <string.h>

// =============================================================
//  robot_state.cpp — Thread-safe shared state implementation
// =============================================================

static robot_state_t  s_state;
static SemaphoreHandle_t s_mutex = NULL;

// ── Internal helpers ─────────────────────────────────────────
static inline void lock()   { xSemaphoreTake(s_mutex, portMAX_DELAY); }
static inline void unlock() { xSemaphoreGive(s_mutex); }

// ── Init ─────────────────────────────────────────────────────
void robot_state_init(void) {
    memset(&s_state, 0, sizeof(s_state));
    s_mutex = xSemaphoreCreateMutex();
    configASSERT(s_mutex != NULL);
}

// ── Setters ──────────────────────────────────────────────────
void robot_state_set_tof(const tof_state_t* tof) {
    lock();
    s_state.tof = *tof;
    s_state.timestamp_us = esp_timer_get_time();
    unlock();
}

void robot_state_set_imu(const imu_state_t* imu) {
    lock();
    s_state.imu = *imu;
    s_state.timestamp_us = esp_timer_get_time();
    unlock();
}

void robot_state_set_battery(const battery_state_t* battery) {
    lock();
    s_state.battery = *battery;
    unlock();
}

void robot_state_set_motor(const motor_state_t* motor) {
    lock();
    s_state.motor = *motor;
    s_state.timestamp_us = esp_timer_get_time();
    unlock();
}

void robot_state_set_target_vel(float left_ms, float right_ms) {
    lock();
    s_state.motor.target_left_ms  = left_ms;
    s_state.motor.target_right_ms = right_ms;
    unlock();
}

// ── Getters ──────────────────────────────────────────────────
void robot_state_get(robot_state_t* out) {
    lock();
    *out = s_state;
    unlock();
}

void robot_state_get_tof(tof_state_t* out) {
    lock();
    *out = s_state.tof;
    unlock();
}

void robot_state_get_imu(imu_state_t* out) {
    lock();
    *out = s_state.imu;
    unlock();
}

void robot_state_get_battery(battery_state_t* out) {
    lock();
    *out = s_state.battery;
    unlock();
}

void robot_state_get_motor(motor_state_t* out) {
    lock();
    *out = s_state.motor;
    unlock();
}

void robot_state_get_target_vel(float* left_ms, float* right_ms) {
    lock();
    *left_ms  = s_state.motor.target_left_ms;
    *right_ms = s_state.motor.target_right_ms;
    unlock();
}
