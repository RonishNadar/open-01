#include "comm_serial.h"
#include "config/config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "COMM_SERIAL";

#define UART_EVENT_QUEUE_SIZE  10

static QueueHandle_t s_uart_event_queue = nullptr;

// ── RX state machine ─────────────────────────────────────────
enum class RxState : uint8_t {
    WAIT_HEADER0, WAIT_HEADER1, WAIT_VERSION, WAIT_MSG_TYPE,
    WAIT_PAYLOAD_LEN, RECV_PAYLOAD, WAIT_CRC_HI, WAIT_CRC_LO, WAIT_END,
};

static RxState    s_state       = RxState::WAIT_HEADER0;
static CommPacket s_pkt         = {};
static uint8_t    s_payload_idx = 0;
static uint16_t   s_crc_recv    = 0;

// ── Init ─────────────────────────────────────────────────────
void comm_serial_init(void) {
    uart_config_t cfg = {};
    cfg.baud_rate  = COMM_UART_BAUD;
    cfg.data_bits  = UART_DATA_8_BITS;
    cfg.parity     = UART_PARITY_DISABLE;
    cfg.stop_bits  = UART_STOP_BITS_1;
    cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;

    ESP_ERROR_CHECK(uart_param_config(COMM_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(COMM_UART_PORT,
                                 COMM_UART_TX_PIN, COMM_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(COMM_UART_PORT,
                                        COMM_UART_BUF_SIZE,
                                        COMM_UART_BUF_SIZE,
                                        UART_EVENT_QUEUE_SIZE,
                                        &s_uart_event_queue,
                                        0));
    ESP_LOGI(TAG, "UART init done");
}

QueueHandle_t comm_serial_get_event_queue(void) {
    return s_uart_event_queue;
}

// ── TX ───────────────────────────────────────────────────────
bool comm_serial_send(MsgType type, const uint8_t* payload, uint8_t payload_len) {
    if (payload_len > COMM_MAX_PAYLOAD) return false;

    uint8_t crc_buf[3 + COMM_MAX_PAYLOAD];
    crc_buf[0] = COMM_VERSION;
    crc_buf[1] = (uint8_t)type;
    crc_buf[2] = payload_len;
    if (payload_len > 0) memcpy(&crc_buf[3], payload, payload_len);
    uint16_t crc = comm_crc16(crc_buf, 3 + payload_len);

    uint8_t frame[8 + COMM_MAX_PAYLOAD];
    uint8_t i = 0;
    frame[i++] = COMM_HEADER_0;
    frame[i++] = COMM_HEADER_1;
    frame[i++] = COMM_VERSION;
    frame[i++] = (uint8_t)type;
    frame[i++] = payload_len;
    if (payload_len > 0) { memcpy(&frame[i], payload, payload_len); i += payload_len; }
    frame[i++] = (crc >> 8) & 0xFF;
    frame[i++] = crc & 0xFF;
    frame[i++] = COMM_END_BYTE;

    return uart_write_bytes(COMM_UART_PORT, (const char*)frame, i) == i;
}

// ── RX state machine (call after data event) ─────────────────
bool comm_serial_receive(CommPacket* out) {
    uint8_t byte;
    while (uart_read_bytes(COMM_UART_PORT, &byte, 1, 0) == 1) {
        switch (s_state) {
            case RxState::WAIT_HEADER0:
                if (byte == COMM_HEADER_0) s_state = RxState::WAIT_HEADER1;
                break;
            case RxState::WAIT_HEADER1:
                s_state = (byte == COMM_HEADER_1) ? RxState::WAIT_VERSION
                                                   : RxState::WAIT_HEADER0;
                break;
            case RxState::WAIT_VERSION:
                s_pkt.version = byte;
                s_state = RxState::WAIT_MSG_TYPE;
                break;
            case RxState::WAIT_MSG_TYPE:
                s_pkt.msg_type = byte;
                s_state = RxState::WAIT_PAYLOAD_LEN;
                break;
            case RxState::WAIT_PAYLOAD_LEN:
                if (byte > COMM_MAX_PAYLOAD) { s_state = RxState::WAIT_HEADER0; break; }
                s_pkt.payload_len = byte;
                s_payload_idx = 0;
                s_state = (byte == 0) ? RxState::WAIT_CRC_HI : RxState::RECV_PAYLOAD;
                break;
            case RxState::RECV_PAYLOAD:
                s_pkt.payload[s_payload_idx++] = byte;
                if (s_payload_idx >= s_pkt.payload_len) s_state = RxState::WAIT_CRC_HI;
                break;
            case RxState::WAIT_CRC_HI:
                s_crc_recv = (uint16_t)byte << 8;
                s_state = RxState::WAIT_CRC_LO;
                break;
            case RxState::WAIT_CRC_LO:
                s_crc_recv |= byte;
                s_state = RxState::WAIT_END;
                break;
            case RxState::WAIT_END: {
                s_state = RxState::WAIT_HEADER0;
                if (byte != COMM_END_BYTE) break;

                uint8_t crc_buf[3 + COMM_MAX_PAYLOAD];
                crc_buf[0] = s_pkt.version;
                crc_buf[1] = s_pkt.msg_type;
                crc_buf[2] = s_pkt.payload_len;
                memcpy(&crc_buf[3], s_pkt.payload, s_pkt.payload_len);
                uint16_t expected = comm_crc16(crc_buf, 3 + s_pkt.payload_len);
                if (expected != s_crc_recv) {
                    ESP_LOGW(TAG, "CRC fail: got 0x%04X exp 0x%04X",
                             s_crc_recv, expected);
                    break;
                }
                s_pkt.crc = s_crc_recv;
                *out = s_pkt;
                return true;
            }
        }
    }
    return false;
}