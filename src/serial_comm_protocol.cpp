/**
 * @file    serial_comm_protocol.cpp
 * @brief   Protocol implementation for serial communication middleware
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Created on 22 of May, 2026
**/

#include "serial_comm_protocol.h"


/* To use the errCode and err_to_str easily */
using namespace SerialCommResult_Codes;


size_t SerialCommProtocol::encode(
    const SerialCommProtoPacket& packet,
    uint8_t* out_buffer,
    size_t max_len
) {
    if (out_buffer == nullptr)
        return 0;
    size_t required_size = packet_size(packet);
    if (max_len < required_size)
        return 0;
    size_t index = 0;

     // HEADER
    out_buffer[index++] = SERIAL_COMM_HEADER_0;
    out_buffer[index++] = SERIAL_COMM_HEADER_1;
    out_buffer[index++] = SERIAL_COMM_HEADER_2;
    
    // SEQ_ID 
    out_buffer[index++] = (packet.header.seq_id & 0xFF);
    out_buffer[index++] = ((packet.header.seq_id >> 8) & 0xFF);

    // VERSION
    out_buffer[index++] = packet.header.version;
    
    // COMMAND
    out_buffer[index++] = static_cast<uint8_t>( packet.header.command );

    // PAYLOAD LENGTH
    out_buffer[index++] = (packet.header.payload_len & 0xFF);
    out_buffer[index++] = ((packet.header.payload_len >> 8) & 0xFF);
    
    // PAYLOAD
    memcpy(
        &out_buffer[index],
        packet.payload,
        packet.header.payload_len
    );
    index += packet.header.payload_len;

     // CRC16 excludes HEADER and CRC field itself
    uint16_t crc =
        compute_crc16(
            &out_buffer[SERIAL_COMM_HEADER_SIZE],
            required_size -
            SERIAL_COMM_HEADER_SIZE -
            SERIAL_COMM_CRC_SIZE
        );
    out_buffer[index++] = (crc & 0xFF);
    out_buffer[index++] = ((crc >> 8) & 0xFF);
    return index;
}


int32_t SerialCommProtocol::decode(
    const uint8_t* raw_data,
    size_t raw_len,
    SerialCommProtoPacket& out_packet
) {
    // Basic validation of packet structure pointer 
    if (raw_data == nullptr){
        return errCode::ERR_NULL_POINTER;
    }
    
    // Check de maximum acceptable packet size
    if ( raw_len > SERIAL_COMM_MAX_PACKET_SIZE_V1 ) {
        return errCode::ERR_OVERFLOW;
    }

    // Check minimum size for a valid packet 
    size_t min_packet_size =
        SERIAL_COMM_HEADER_SIZE +
        SERIAL_COMM_PROTOCOL_VER_SIZE +
        SERIAL_COMM_COMMAND_SIZE +
        SERIAL_COMM_LENGTH_SIZE +
        SERIAL_COMM_CRC_SIZE;

    if (raw_len < min_packet_size){
        return errCode::ERR_INVALID_ARG;
    }

    // HEADER VALIDATION
    if (
        raw_data[0] != SERIAL_COMM_HEADER_0 ||
        raw_data[1] != SERIAL_COMM_HEADER_1 ||
        raw_data[2] != SERIAL_COMM_HEADER_2
    ) {
        return errCode::ERR_PARSER;
    }
    size_t index = 3;

    // SEQ_ID
    out_packet.header.seq_id =
        raw_data[index] |
        (raw_data[index + 1] << 8);
    index += 2;

    // VERSION
    out_packet.header.version = raw_data[index++];

    // COMMAND
    out_packet.header.command =
        static_cast<SerialCommCommand>(
            raw_data[index++]
        );

    // PAYLOAD LENGTH
    out_packet.header.payload_len =
        raw_data[index] |
        (raw_data[index + 1] << 8);
    index += 2;

    // PAYLOAD SIZE VALIDATION
    if ( out_packet.header.payload_len > SERIAL_COMM_MAX_PAYLOAD_V1 ) {
        return errCode::ERR_OVERFLOW;
    }
    size_t expected_size =
        SERIAL_COMM_HEADER_SIZE +
        SERIAL_COMM_PROTOCOL_VER_SIZE +
        SERIAL_COMM_COMMAND_SIZE +
        SERIAL_COMM_LENGTH_SIZE +
        out_packet.header.payload_len +
        SERIAL_COMM_CRC_SIZE;
    if (raw_len < expected_size) {
        return errCode::ERR_INVALID_ARG;
    }

    // PAYLOAD
    memcpy(
        out_packet.payload,
        &raw_data[index],
        out_packet.header.payload_len
    );
    index += out_packet.header.payload_len;

    // CRC
    out_packet.crc = raw_data[index] | (raw_data[index + 1] << 8);

    // CRC VALIDATION
    if ( !validate_crc( out_packet ) ) {
        return errCode::ERR_CRC;
    }
    return errCode::OK;
}


uint16_t SerialCommProtocol::compute_crc16(
    const uint8_t* data,
    size_t len
) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

bool SerialCommProtocol::validate_crc(
    const SerialCommProtoPacket& packet
) {
    uint8_t temp_buffer[ SERIAL_COMM_MAX_PACKET_SIZE_V1 ];
    size_t index = 0;

    // VERSION
    temp_buffer[index++] = packet.header.version;

    // COMMAND
    temp_buffer[index++] =
        static_cast<uint8_t>(
            packet.header.command
        );

    // PAYLOAD LENGTH
    temp_buffer[index++] = (packet.header.payload_len & 0xFF);
    temp_buffer[index++] = ((packet.header.payload_len >> 8) & 0xFF);

    // PAYLOAD
    memcpy(
        &temp_buffer[index],
        packet.payload,
        packet.header.payload_len
    );
    index += packet.header.payload_len;

    // COMPUTE CRC16 TO COMPARE 
    uint16_t computed_crc =
        compute_crc16(
            temp_buffer,
            index
        );
    return ( computed_crc == packet.crc );
}


bool SerialCommProtocol::validate_packet(
    const SerialCommProtoPacket& packet
) {
    // Validate the version (only acepting V1 for now)
    if ( packet.header.version != SERIAL_COMM_PROTOCOL_VER1 ) {
        return false;
    }
    // Validate payload length
    if ( packet.header.payload_len > SERIAL_COMM_MAX_PAYLOAD_V1 ) {
        return false;
    }
    // Validate CRC
    if ( !validate_crc(packet)) {
        return false;
    }
    return true;
}


size_t SerialCommProtocol::packet_size(
    const SerialCommProtoPacket& packet
) {
    return
        SERIAL_COMM_HEADER_SIZE +
        SERIAL_COMM_PROTOCOL_VER_SIZE +
        SERIAL_COMM_COMMAND_SIZE +
        SERIAL_COMM_LENGTH_SIZE +
        packet.header.payload_len +
        SERIAL_COMM_CRC_SIZE;
}

void SerialCommProtocol::clear_packet( SerialCommProtoPacket& packet ) {
    memset( &packet, 0, sizeof(SerialCommProtoPacket) );
    packet.header.version = SERIAL_COMM_PROTOCOL_VER1;
    packet.header.command = SerialCommCommand::UNKNOWN;
}
