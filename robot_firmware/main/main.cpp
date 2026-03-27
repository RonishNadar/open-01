#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "config/config.h"
#include "common/robot_state.h"
#include "comms/comm_serial.h"
#include "tasks/task_motor.h"
#include "tasks/task_imu.h"
#include "tasks/task_tof.h"
#include "tasks/task_battery.h"
#include "tasks/task_comms.h"

static const char* TAG = "MAIN";

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "  Robot Firmware v%d.%d.%d",
             FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH);
    ESP_LOGI(TAG, "==============================================");

    robot_state_init();
    ESP_LOGI(TAG, "[OK] Shared state initialized");

    xTaskCreatePinnedToCore(task_motor,   "motor",   STACK_MOTOR,   NULL, PRIO_MOTOR,   NULL, CORE_MOTOR);
    ESP_LOGI(TAG, "[OK] Motor task started");

    xTaskCreatePinnedToCore(task_imu,     "imu",     STACK_IMU,     NULL, PRIO_IMU,     NULL, CORE_IMU);
    ESP_LOGI(TAG, "[OK] IMU task started");

    xTaskCreatePinnedToCore(task_tof,     "tof",     STACK_TOF,     NULL, PRIO_TOF,     NULL, CORE_TOF);
    ESP_LOGI(TAG, "[OK] ToF task started");

    xTaskCreatePinnedToCore(task_battery, "battery", STACK_BATTERY, NULL, PRIO_BATTERY, NULL, CORE_BATTERY);
    ESP_LOGI(TAG, "[OK] Battery task started");

    comm_serial_init();
    xTaskCreatePinnedToCore(task_comms,  "comms",   STACK_COMMS,   NULL, PRIO_COMMS,   NULL, CORE_COMMS);
    ESP_LOGI(TAG, "[OK] Comms task started");
}