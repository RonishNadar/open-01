#pragma once
#include <stdint.h>

// ── Frame constants ───────────────────────────────────────────────────────────
static constexpr uint8_t COMM_HEADER_0     = 0xAA;
static constexpr uint8_t COMM_HEADER_1     = 0x55;
static constexpr uint8_t COMM_VERSION      = 0x01;
static constexpr uint8_t COMM_END_BYTE     = 0xFF;
static constexpr uint8_t COMM_MAX_PAYLOAD  = 128;

// ── Message types ─────────────────────────────────────────────────────────────
enum class MsgType : uint8_t {
    CMD_VELOCITY    = 0x10,
    CMD_ESTOP       = 0x11,
    CMD_PING        = 0x12,
    TELEMETRY       = 0x20,
    PONG            = 0x21,
};

// ── Sub-block IDs (only used inside TELEMETRY payload) ────────────────────────
enum class SubID : uint8_t {
    IMU       = 0x01,   // 6x float  = 24 bytes
    TOF       = 0x02,   // 3x uint16 =  6 bytes
    ODOM      = 0x03,   // 6x float  = 24 bytes
    BATTERY   = 0x04,   // 1x float  =  4 bytes
    TIMESTAMP = 0x05,   // 1x uint32 =  4 bytes
};

// ── Sub-block payload structs (packed) ────────────────────────────────────────
struct __attribute__((packed)) SubImu {
    float accel_x, accel_y, accel_z;   // m/s²
    float gyro_x,  gyro_y,  gyro_z;    // rad/s
};

struct __attribute__((packed)) SubTof {
    uint16_t left_mm, right_mm, back_mm;
};

struct __attribute__((packed)) SubOdom {
    float pos_x, pos_y, heading;        // m, m, rad
    float vel_x, vel_y, vel_w;          // m/s, m/s, rad/s
};

struct __attribute__((packed)) SubBattery {
    float voltage;                       // V
};

struct __attribute__((packed)) SubTimestamp {
    uint32_t ms;
};

// ── CMD payload structs ───────────────────────────────────────────────────────
struct __attribute__((packed)) PayloadCmdVelocity {
    float linear_x;
    float linear_y;
    float angular_z;
};

// ── Parsed packet ─────────────────────────────────────────────────────────────
struct CommPacket {
    uint8_t  version;
    uint8_t  msg_type;
    uint8_t  payload_len;
    uint8_t  payload[COMM_MAX_PAYLOAD];
    uint16_t crc;
};

// ── CRC16-CCITT (poly=0x1021, init=0x0000) ───────────────────────────────────
inline uint16_t comm_crc16(const uint8_t* data, uint8_t len) {
    uint16_t crc = 0x0000;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}