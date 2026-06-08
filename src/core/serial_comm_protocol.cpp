/**
 * @file    serial_comm_protocol.cpp
 * @brief   Protocol implementation for serial communication middleware
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Created on 22 of May, 2026
**/

#include "serial_comm/core/serial_comm_protocol.h"


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
    
    // Check de maximum acceptable packet size (accept V2 max as upper bound)
    if ( raw_len > SERIAL_COMM_MAX_PACKET_SIZE_V2 ) {
        return errCode::ERR_OVERFLOW;
    }

    // Check minimum size for a valid packet 
    size_t min_packet_size =
        SERIAL_COMM_HEADER_SIZE +
        SERIAL_COMM_SEQ_SIZE +
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
    if ( out_packet.header.payload_len > SERIAL_COMM_MAX_PAYLOAD ) {
        return errCode::ERR_OVERFLOW;
    }
    size_t expected_size =
        SERIAL_COMM_HEADER_SIZE +
        SERIAL_COMM_SEQ_SIZE +
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


namespace {

/**
 * @brief   CRC16-CCITT (poly=0x1021, init=0xFFFF) lookup table.
 * @details Precomputed so compute_crc16() can fold in one byte per
 *          iteration with a single table lookup instead of looping
 *          8 conditional shifts per byte.
 */
constexpr uint16_t CRC16_CCITT_TABLE[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

} // namespace


uint16_t SerialCommProtocol::compute_crc16(
    const uint8_t* data,
    size_t len,
    uint16_t crc
) {
    for (size_t i = 0; i < len; i++) {
        crc = static_cast<uint16_t>(
            (crc << 8) ^ CRC16_CCITT_TABLE[ ((crc >> 8) ^ data[i]) & 0xFF ]
        );
    }
    return crc;
}

bool SerialCommProtocol::validate_crc(
    const SerialCommProtoPacket& packet
) {
    // CRC excludes only the preamble header and the CRC field itself.
    // The covered fields are not contiguous inside SerialCommProtoPacket
    // (struct padding separates the header from the payload array), so
    // instead of memcpy-ing everything into a >1KB scratch buffer first,
    // the checksum is folded incrementally straight from the struct
    // fields, carrying the running CRC value across each chunk.
    const uint8_t seq_bytes[2] = {
        static_cast<uint8_t>( packet.header.seq_id & 0xFF ),
        static_cast<uint8_t>( (packet.header.seq_id >> 8) & 0xFF )
    };
    const uint8_t meta_bytes[2] = {
        packet.header.version,
        static_cast<uint8_t>( packet.header.command )
    };
    const uint8_t len_bytes[2] = {
        static_cast<uint8_t>( packet.header.payload_len & 0xFF ),
        static_cast<uint8_t>( (packet.header.payload_len >> 8) & 0xFF )
    };

    uint16_t crc = compute_crc16( seq_bytes, sizeof(seq_bytes) );
    crc = compute_crc16( meta_bytes, sizeof(meta_bytes), crc );
    crc = compute_crc16( len_bytes, sizeof(len_bytes), crc );
    crc = compute_crc16( packet.payload, packet.header.payload_len, crc );

    return ( crc == packet.crc );
}


bool SerialCommProtocol::validate_packet(
    const SerialCommProtoPacket& packet
) {
    // Validate the version (only acepting V1 for now)
    if ( packet.header.version != SERIAL_COMM_PROTOCOL_VER1 ) {
        return false;
    }
    // Validate payload length
    if ( packet.header.payload_len > SERIAL_COMM_MAX_PAYLOAD ) {
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
        SERIAL_COMM_SEQ_SIZE +
        SERIAL_COMM_PROTOCOL_VER_SIZE +
        SERIAL_COMM_COMMAND_SIZE +
        SERIAL_COMM_LENGTH_SIZE +
        packet.header.payload_len +
        SERIAL_COMM_CRC_SIZE;
}

void SerialCommProtocol::clear_packet( SerialCommProtoPacket& packet ) {
    memset(&packet, 0, sizeof(SerialCommProtoPacket));
    packet.header.version = SERIAL_COMM_PROTOCOL_VER1;
    packet.header.command = SerialCommCommand::UNDEFINED;
}
