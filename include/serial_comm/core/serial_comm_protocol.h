/**
 * @file    serial_comm_protocol.h
 * @brief   Protocol implementation for serial communication middleware
 * @details This file defines the protocol for encoding and decoding 
 *          packets, as well as utilities for computing and validating CRC 
 *          checksums, framming abstration and packet serializatio.
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Created on 22 of May, 2026
 */


#pragma once

#include "serial_comm/serial_comm_config.h"
#include "serial_comm/messages/serial_comm_messages.h"
#include "serial_comm/core/serial_comm_utils.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <cstring>


/**
 * @brief Protocol header bytes
 */
constexpr uint8_t SERIAL_COMM_HEADER_0          = 0xAA;
constexpr uint8_t SERIAL_COMM_HEADER_1          = 0x55;
constexpr uint8_t SERIAL_COMM_HEADER_2          = 0xAA;

/**
 * @brief Protocol field sizes in bytes
 */
constexpr uint8_t SERIAL_COMM_HEADER_SIZE       = 3;
constexpr uint8_t SERIAL_COMM_SEQ_SIZE          = 2;
constexpr uint8_t SERIAL_COMM_PROTOCOL_VER_SIZE = 1;
constexpr uint8_t SERIAL_COMM_SRC_SIZE          = 1;
constexpr uint8_t SERIAL_COMM_DST_SIZE          = 1;
constexpr uint8_t SERIAL_COMM_FLAG_SIZE         = 1;
constexpr uint8_t SERIAL_COMM_COMMAND_SIZE      = 1;
constexpr uint8_t SERIAL_COMM_LENGTH_SIZE       = 2;
constexpr uint8_t SERIAL_COMM_CRC_SIZE          = 2;

/**
 * @brief Current protocol version
 */
constexpr uint8_t SERIAL_COMM_PROTOCOL_VER1     = 0x01;
constexpr uint8_t SERIAL_COMM_PROTOCOL_VER2     = 0x02;

/**
 * @brief Maximum payload size
 */
constexpr size_t SERIAL_COMM_MAX_PAYLOAD = 
    SerialCommConfig::MAX_PAYLOAD_SIZE;


/**
 * @brief Maximum packet size for V1
 * HEADER(3)
 * SEQ_ID(2)
 * VERSION(1)
 * COMMAND(1)
 * LENGTH(2)
 * PAYLOAD(1024)
 * CRC16(2)
 */
constexpr size_t SERIAL_COMM_MAX_PACKET_SIZE_V1 =  \
    SERIAL_COMM_HEADER_SIZE + \
    SERIAL_COMM_SEQ_SIZE + \
    SERIAL_COMM_PROTOCOL_VER_SIZE + \
    SERIAL_COMM_COMMAND_SIZE + \
    SERIAL_COMM_LENGTH_SIZE + \
    SERIAL_COMM_MAX_PAYLOAD + \
    SERIAL_COMM_CRC_SIZE;


/**
 * @brief Maximum packet size for V2
 * HEADER(3)
 * SEQ_ID(2)
 * VERSION(1)
 * SRC(1)
 * DST(1)
 * FLAG(1)
 * COMMAND(1)
 * LENGTH(2)
 * PAYLOAD(1024)
 * CRC16(2)
 */
constexpr size_t SERIAL_COMM_MAX_PACKET_SIZE_V2 =  \
    SERIAL_COMM_HEADER_SIZE + \
    SERIAL_COMM_SEQ_SIZE + \
    SERIAL_COMM_PROTOCOL_VER_SIZE + \
    SERIAL_COMM_SRC_SIZE + \
    SERIAL_COMM_DST_SIZE + \
    SERIAL_COMM_FLAG_SIZE + \
    SERIAL_COMM_COMMAND_SIZE + \
    SERIAL_COMM_LENGTH_SIZE + \
    SERIAL_COMM_MAX_PAYLOAD + \
    SERIAL_COMM_CRC_SIZE;


/**
 * @brief   Protocol packet structure
 * @param   header Packet header containing metadata
 * @param   seq_id Packet sequence identifier: 
 *              - For tracking and matching requests/replies
 * @param   version Protocol version
 * @param   command Command identifier
 * @param   payload_len Packet payload length 
 */
struct SerialCommProtoHeader {
    uint8_t header[3] = {
        SERIAL_COMM_HEADER_0,
        SERIAL_COMM_HEADER_1,
        SERIAL_COMM_HEADER_2
    };
    uint16_t seq_id = 0;
    uint8_t version = SERIAL_COMM_PROTOCOL_VER1;
    SerialCommCommand command = SerialCommCommand::UNDEFINED;
    uint16_t payload_len = 0;
};


/**
 * @brief   Generic serial communication packet
 * @param   header Packet header containing metadata
 * @param   payload Packet payload data
 * @param   crc CRC16 checksum of the packet:
 *              - Excluding the Header and CRC field
 */
struct SerialCommProtoPacket {
    SerialCommProtoHeader header;
    uint8_t payload[SERIAL_COMM_MAX_PAYLOAD ] = { 0 };
    uint16_t crc = 0;
};


class SerialCommProtocol {
public:

    /**
     * @brief       Encode packet into byte stream
     * @param[in]   packet Packet to encode
     * @param[out]  out_buffer Serialized packet buffer
     * @param[in]   max_len Output buffer maximum size
     * @return      Number of bytes encoded
     */
    static size_t encode(
        const SerialCommProtoPacket& packet,
        uint8_t* out_buffer,
        size_t max_len
    );

    /**
     * @brief       Decode byte stream into packet
     * @param[in]   raw_data Raw serialized packet
     * @param[in]   raw_len Raw data size
     * @param[out]  out_packet Decoded packet
     * @return      Protocol SerialCommResult::errCode 
     */
    static int32_t decode(
        const uint8_t* raw_data,
        size_t raw_len,
        SerialCommProtoPacket& out_packet
    );


    /**
     * @brief       Compute packet CRC16
     * @param[in]   data Data buffer
     * @param[in]   len Data length
     * @return      CRC16 result
     */
    static uint16_t compute_crc16(
        const uint8_t* data,
        size_t len
    );


    /**
     * @brief       Validate packet CRC
     * @param[in]   packet Packet to validate
     * @return      true if CRC is valid
     * @return      false otherwise
     */
    static bool validate_crc(
        const SerialCommProtoPacket& packet
    );


    /**
     * @brief       Validate packet structure
     * @param[in]   packet Packet to validate
     * @return      true if packet is structurally valid
     * @return      false otherwise
     */
    static bool validate_packet(
        const SerialCommProtoPacket& packet
    );


    /**
     * @brief       Compute serialized packet size
     * @param[in]   packet Packet reference
     * @return      Total serialized packet size
     */
    static size_t packet_size(
        const SerialCommProtoPacket& packet
    );


    /**
     * @brief       Clear packet contents
     * @param[out]  packet Packet reference
     */
    static void clear_packet(
        SerialCommProtoPacket& packet
    );

};
