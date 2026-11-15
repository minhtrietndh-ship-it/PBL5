#ifndef ESP_NOW_PROTOCOL_H
#define ESP_NOW_PROTOCOL_H

#include <stdint.h>

// Các loại bản tin
#define MSG_TYPE_CMD  0x01
#define MSG_TYPE_DATA 0x02

// Lệnh điều khiển
#define CMD_STOP_STREAM  0x00
#define CMD_START_STREAM 0x01

// 1. Gói tin Lệnh (Node 1 -> Node 2)
typedef struct __attribute__((packed)) {
    uint8_t msg_type;   // Phải là MSG_TYPE_CMD
    uint8_t command;    // CMD_START_STREAM hoặc CMD_STOP_STREAM
} cmd_packet_t;

// 2. Gói tin Dữ liệu (Node 2 -> Node 1)
typedef struct __attribute__((packed)) {
    uint8_t msg_type;   // Phải là MSG_TYPE_DATA
    float bpm;          // Nhịp tim
} health_data_packet_t;

#endif