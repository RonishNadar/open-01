#include "task_battery.h"
#include "hal/hal_battery.h"
#include "common/robot_state.h"
#include "config/config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "TASK_BATTERY";

// =============================================================
//  task_battery.cpp — Battery monitoring task
//  Runs at 2Hz (500ms), writes voltage to robot_state
// =============================================================

// Low battery warning thresholds
#define BATTERY_WARN_V   10.5f   // ~3.5V per cell (3S LiPo)
#define BATTERY_CRIT_V    9.9f   // ~3.3V per cell (cutoff)

void task_battery(void* arg) {
    ESP_LOGI(TAG, "Battery task started");

    ESP_ERROR_CHECK(hal_battery_init());

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_BATTERY_MS));

        float voltage = hal_battery_read_voltage();

        // Write to shared state
        battery_state_t state = {};
        state.voltage = voltage;
        state.valid   = true;
        robot_state_set_battery(&state);

        // Log with warning levels
        if (voltage < BATTERY_CRIT_V) {
            ESP_LOGE(TAG, "CRITICAL: %.2fV — STOP ROBOT NOW!", voltage);
        } else if (voltage < BATTERY_WARN_V) {
            ESP_LOGW(TAG, "LOW BATTERY: %.2fV", voltage);
        } else {
            ESP_LOGI(TAG, "Battery: %.2fV", voltage);
        }
    }
}
