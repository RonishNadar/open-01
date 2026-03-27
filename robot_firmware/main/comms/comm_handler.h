#pragma once
#include "comm_protocol.h"

// No robot_state_t* param needed — uses robot_state API internally
void comm_handler_process(const CommPacket* pkt);

void comm_builder_reset(void);
void comm_builder_add_imu(void);
void comm_builder_add_tof(void);
void comm_builder_add_odom(void);
void comm_builder_add_battery(void);
void comm_builder_add_timestamp(void);
bool comm_builder_send(void);