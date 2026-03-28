#include "comm_handler.h"
#include "comm_serial.h"
#include "common/robot_state.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char* TAG = "COMM_HANDLER";

static uint8_t s_buf[COMM_MAX_PAYLOAD];
static uint8_t s_buf_len = 0;

static bool builder_append(SubID id, const void* data, uint8_t data_len) {
    uint8_t needed = 2 + data_len;
    if (s_buf_len + needed > COMM_MAX_PAYLOAD) {
        ESP_LOGW(TAG, "Builder overflow");
        return false;
    }
    s_buf[s_buf_len++] = (uint8_t)id;
    s_buf[s_buf_len++] = data_len;
    memcpy(&s_buf[s_buf_len], data, data_len);
    s_buf_len += data_len;
    return true;
}

void comm_builder_reset(void) { s_buf_len = 0; }

void comm_builder_add_imu(void) {
    imu_state_t imu;
    robot_state_get_imu(&imu);
    SubImu d = { imu.accel_x, imu.accel_y, imu.accel_z,
                 imu.gyro_x,  imu.gyro_y,  imu.gyro_z };
    builder_append(SubID::IMU, &d, sizeof(d));
}

void comm_builder_add_tof(void) {
    tof_state_t tof;
    robot_state_get_tof(&tof);
    SubTof d = { tof.left_mm, tof.right_mm, tof.back_mm };
    builder_append(SubID::TOF, &d, sizeof(d));
}

void comm_builder_add_odom(void) {
    motor_state_t motor;
    robot_state_get_motor(&motor);
    SubOdom d = { motor.x,         motor.y,         motor.theta,
                  motor.linear_ms, motor.angular_rs, 0.0f };
    builder_append(SubID::ODOM, &d, sizeof(d));
}

void comm_builder_add_battery(void) {
    battery_state_t bat;
    robot_state_get_battery(&bat);
    SubBattery d = { bat.voltage };
    builder_append(SubID::BATTERY, &d, sizeof(d));
}

void comm_builder_add_timestamp(void) {
    SubTimestamp d = { (uint32_t)(esp_timer_get_time() / 1000ULL) };
    builder_append(SubID::TIMESTAMP, &d, sizeof(d));
}

bool comm_builder_send(void) {
    return comm_serial_send(MsgType::TELEMETRY, s_buf, s_buf_len);
}

void comm_handler_process(const CommPacket* pkt) {
    switch ((MsgType)pkt->msg_type) {

        case MsgType::CMD_VELOCITY: {
            if (pkt->payload_len != sizeof(PayloadCmdVelocity)) {
                ESP_LOGW(TAG, "CMD_VELOCITY bad len %d", pkt->payload_len);
                break;
            }
            PayloadCmdVelocity cmd;
            memcpy(&cmd, pkt->payload, sizeof(cmd));
            ESP_LOGI("COMM", "CMD_VELOCITY received: vx=%.3f m/s, vy=%.3f m/s, vw=%.2f rad/s",
                     cmd.linear_x, cmd.linear_y, cmd.angular_z);
            robot_state_set_cmd_vel(cmd.linear_x, cmd.linear_y, cmd.angular_z);
            break;
        }

        case MsgType::CMD_ESTOP:
            robot_state_set_estop(true);
            robot_state_set_cmd_vel(0.0f, 0.0f, 0.0f);
            ESP_LOGW(TAG, "E-STOP received");
            break;

        case MsgType::CMD_PING:
            comm_serial_send(MsgType::PONG, nullptr, 0);
            break;

        default:
            ESP_LOGW(TAG, "Unknown MsgType 0x%02X", pkt->msg_type);
            break;
    }
}