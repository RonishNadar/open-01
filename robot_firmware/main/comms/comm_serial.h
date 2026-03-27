#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "comm_protocol.h"

void comm_serial_init(void);
bool comm_serial_send(MsgType type, const uint8_t* payload, uint8_t payload_len);
bool comm_serial_receive(CommPacket* out);
QueueHandle_t comm_serial_get_event_queue(void);