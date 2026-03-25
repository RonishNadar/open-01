#include "task_tof.h"
#include "hal/hal_tof.h"
#include "common/robot_state.h"
#include "config/config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "TASK_TOF";

void task_tof(void* arg) {
    ESP_LOGI(TAG, "ToF task started");

    // Retry init
    esp_err_t ret = ESP_FAIL;
    for (int i = 1; i <= 5; i++) {
        ret = hal_tof_init();
        if (ret == ESP_OK) break;
        ESP_LOGW(TAG, "ToF init attempt %d/5 failed: %d", i, ret);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ToF init failed after 5 attempts — task exiting");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "ToF sensors ready");
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_TOF_MS));

        tof_reading_t reading;
        hal_tof_read(&reading);

        // Write to shared state
        tof_state_t state = {};
        state.left_mm  = reading.left_mm;
        state.back_mm  = reading.back_mm;
        state.right_mm = reading.right_mm;
        state.valid    = true;
        robot_state_set_tof(&state);

        ESP_LOGI(TAG, "L: %4dmm | B: %4dmm | R: %4dmm",
                 reading.left_mm, reading.back_mm, reading.right_mm);
    }
}
