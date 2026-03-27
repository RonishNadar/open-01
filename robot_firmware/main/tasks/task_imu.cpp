#include "task_imu.h"
#include "hal/hal_imu.h"
#include "common/robot_state.h"
#include "config/config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "TASK_IMU";

// =============================================================
//  task_imu.cpp — IMU task
//  Polls MPU6500 at 100Hz, writes to robot_state
// =============================================================

void task_imu(void* arg) {
    ESP_LOGI(TAG, "IMU task started");
    vTaskDelay(pdMS_TO_TICKS(500));

    // Retry init up to 5 times — handles intermittent I2C startup issues
    esp_err_t ret = ESP_FAIL;
    int attempt = 0;
    while (ret != ESP_OK) {
        attempt++;
        ret = hal_imu_init();
        if (ret == ESP_OK) break;
        ESP_LOGW(TAG, "IMU init attempt %d failed: %d — retrying...", attempt, ret);
        vTaskDelay(pdMS_TO_TICKS(500));   // back to 200ms, was working before
    }
    ESP_LOGI(TAG, "IMU initialized after %d attempt(s)", attempt);

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_IMU_MS));

        imu_data_t data;
        esp_err_t ret = hal_imu_read(&data);

        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "IMU read failed: %d", ret);
            continue;
        }

        // Write to shared state
        imu_state_t state = {};
        state.accel_x = data.accel_x;
        state.accel_y = data.accel_y;
        state.accel_z = data.accel_z;
        state.gyro_x  = data.gyro_x;
        state.gyro_y  = data.gyro_y;
        state.gyro_z  = data.gyro_z;
        state.valid   = true;
        robot_state_set_imu(&state);

        ESP_LOGI(TAG,
            "Accel: x=%6.2f y=%6.2f z=%6.2f m/s² | "
            "Gyro:  x=%6.1f y=%6.1f z=%6.1f deg/s",
            data.accel_x, data.accel_y, data.accel_z,
            data.gyro_x,  data.gyro_y,  data.gyro_z);
    }
}