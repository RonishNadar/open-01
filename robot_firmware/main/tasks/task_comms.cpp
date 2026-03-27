#include "task_comms.h"
#include "comms/comm_serial.h"
#include "comms/comm_handler.h"
#include "config/config.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char* TAG = "TASK_COMMS";

void task_comms(void* pvParams) {
    ESP_LOGI(TAG, "Comms task running");

    QueueHandle_t uart_queue = comm_serial_get_event_queue();
    uart_event_t  event;
    CommPacket    pkt;

    TickType_t last_telem = xTaskGetTickCount();
    const TickType_t telem_period = pdMS_TO_TICKS(TASK_COMMS_MS);

    while (true) {
        // Block until UART event OR telemetry period expires — no busy polling
        TickType_t now     = xTaskGetTickCount();
        TickType_t elapsed = now - last_telem;
        TickType_t wait    = (elapsed >= telem_period) ? 0
                           : (telem_period - elapsed);

        if (xQueueReceive(uart_queue, &event, wait) == pdTRUE) {
            if (event.type == UART_DATA) {
                while (comm_serial_receive(&pkt))
                    comm_handler_process(&pkt);
            } else if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
                ESP_LOGW(TAG, "UART overflow — flushing");
                uart_flush_input(COMM_UART_PORT);
                xQueueReset(uart_queue);
            }
        }

        // Send telemetry at fixed rate
        if ((xTaskGetTickCount() - last_telem) >= telem_period) {
            comm_builder_reset();
            comm_builder_add_imu();
            comm_builder_add_tof();
            comm_builder_add_odom();
            comm_builder_add_battery();
            comm_builder_add_timestamp();
            comm_builder_send();
            last_telem = xTaskGetTickCount();
        }
    }
}