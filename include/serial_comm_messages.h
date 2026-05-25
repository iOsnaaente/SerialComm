/**
 * @brief Serial communication protocol, structs, types and constants
 */

#pragma once 

#include <stdint.h>
#include <stddef.h>

constexpr uint8_t HEADER_0 = 0xAA;
constexpr uint8_t HEADER_1 = 0x55;
constexpr uint8_t HEADER_2 = 0xAA;

constexpr uint8_t PROTOCOL_VERSION = 0x01;
constexpr size_t MAX_PAYLOAD_SIZE = 256;

enum class Command : uint8_t {
    READ_JOINTS   = 0x01,
    WRITE_JOINTS  = 0x02,
    PING          = 0x03,
    READ_UTILITY  = 0x04,
    WRITE_UTILITY = 0x05,
    SMOOTH_MOVE   = 0x06
};

struct Packet {
    uint8_t version;
    Command command;
    uint16_t payload_len;
    uint8_t payload[MAX_PAYLOAD_SIZE];
    uint16_t crc;
};
