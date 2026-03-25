#include "task_motor.h"
#include "hal/hal_motor.h"
#include "services/motor_controller.h"
#include "common/robot_state.h"
#include "config/config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "TASK_MOTOR";

void task_motor(void* arg) {
    ESP_LOGI(TAG, "Motor task started");

    ESP_ERROR_CHECK(hal_motor_init());

    motor_controller_t mc;
    motor_controller_init(&mc);

    // TEMP: hardcoded target — replace with robot_state_get_target_vel() once comms ready
    motor_controller_set_target(&mc, 0.2f, 0.2f);
    ESP_LOGI(TAG, "Target: 0.2 m/s forward");

    TickType_t last_wake = xTaskGetTickCount();
    int64_t    last_time = esp_timer_get_time();

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_MOTOR_MS));

        int64_t now = esp_timer_get_time();
        float dt    = (float)(now - last_time) / 1e6f;
        last_time   = now;

        // Run PID
        motor_controller_update(&mc, dt);

        ESP_LOGI(TAG,
            "L: tgt=%5.0f meas=%5.0f mm/s | R: tgt=%5.0f meas=%5.0f mm/s | "
            "x=%.3f y=%.3f θ=%.2f",
            mc.target_left_ms  * 1000.0f, mc.measured_left_ms  * 1000.0f,
            mc.target_right_ms * 1000.0f, mc.measured_right_ms * 1000.0f,
            mc.odom.pose.x, mc.odom.pose.y, mc.odom.pose.theta);
    }
}