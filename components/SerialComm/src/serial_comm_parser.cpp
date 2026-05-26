/**
 * @file    serial_comm_parser.cpp
 * @brief   Incremental stream parser implementation
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Created on 22 of May, 2026
 */

#include "serial_comm_parser.h"

/* To use the errCode and err_to_str easily */
using namespace SerialCommResult_Codes;


bool SerialCommParser::parse_byte(
    uint8_t byte,
    SerialCommProtoPacket& out_packet
) {
    switch ( this->state_) {
        // Header 0 
        case State::WAIT_HEADER_0: {
            if (byte == SERIAL_COMM_HEADER_0) {
                this->state_ = State::WAIT_HEADER_1;
            }
            break;
        }

        // Header 1
        case State::WAIT_HEADER_1: {
            if (byte == SERIAL_COMM_HEADER_1) {
                this->state_ = State::WAIT_HEADER_2;
            } else {
                reset();
            }
            break;
        }

        // Header 2
        case State::WAIT_HEADER_2: {
            if (byte == SERIAL_COMM_HEADER_2) {
                synchronized_ = true;
                this->state_ = State::READ_VERSION;
            } else {
                reset();
            }
            break;
        }

        // READ VERSION
        case State::READ_VERSION: {
            packet_.header.version = byte;
            this->state_ = State::READ_COMMAND;
            break;
        }

        // READ COMMAND
        case State::READ_COMMAND: {
            packet_.header.command =
                static_cast<SerialCommProtoCommand>(byte);
            this->state_ = State::READ_LENGTH_L;
            break;
        }

        // READ LENGTH LOW BYTE
        case State::READ_LENGTH_L: {
            packet_.header.payload_len = byte;
            this->state_ = State::READ_LENGTH_H;
            break;
        }

        // READ LENGTH HIGH BYTE
        case State::READ_LENGTH_H: {
            packet_.header.payload_len |= (byte << 8);
            // VALIDATE PAYLOAD SIZE
            if ( packet_.header.payload_len > SERIAL_COMM_MAX_PAYLOAD_V1 ) {
                reset();
                break;
            }
            // NO PAYLOAD
            if ( packet_.header.payload_len == 0 ) {
                this->state_ =
                    State::READ_CRC_L;
            } else {
                payload_index_ = 0;
                this->state_ = State::READ_PAYLOAD;
            }
            break;
        }

        // READ PAYLOAD
        case State::READ_PAYLOAD: {
            packet_.payload[ payload_index_++] = byte;
            if ( payload_index_ >= packet_.header.payload_len
            ) {
                this->state_ = State::READ_CRC_L;
            }
            break;
        }

        // READ CRC LOW BYTE
        case State::READ_CRC_L: {
            crc_l_ = byte;
            this->state_ = State::READ_CRC_H;
            break;
        }

        // READ CRC HIGH BYTE
        case State::READ_CRC_H: {
            packet_.crc = crc_l_ | (byte << 8);

            // VALIDATE PACKET
            if ( SerialCommProtocol::validate_packet(packet_)) {
                out_packet = packet_;
                this->state_ = State::PACKET_READY;
                reset();
                return true;
            }
            reset();
            break;
        }

        // PACKET READY
        case State::PACKET_READY: {
            reset();
            break;
        }

        // ERROR
        case State::ERROR:
        default: {
            reset();
            break;
        }
    }
    return false;
}


bool SerialCommParser::parse_buffer(
    const uint8_t* data,
    size_t len,
    SerialCommProtoPacket& out_packet
) {
    if (data == nullptr)
        return false;
    for (size_t i = 0; i < len; i++) {
        if ( parse_byte( data[i], out_packet ) ) {
            return true;
        }
    }
    return false;
}


void SerialCommParser::reset() {
    this->state_ = State::WAIT_HEADER_0;
    payload_index_ = 0;
    crc_l_ = 0;
    synchronized_ = false;
    SerialCommProtocol::clear_packet( packet_ );
}


SerialCommParser::State SerialCommParser::state() const {
    return this->state_;
}


bool SerialCommParser::synchronized() const {
    return synchronized_;
}


size_t SerialCommParser::payload_index() const {
    return payload_index_;
}


uint16_t SerialCommParser::expected_payload_size() const {
    return packet_.header.payload_len;
}


const char* SerialCommParser::state_to_str( State state ) {
    switch (state) {
        case State::WAIT_HEADER_0:  return "WAIT_HEADER_0";
        case State::WAIT_HEADER_1:  return "WAIT_HEADER_1";
        case State::WAIT_HEADER_2:  return "WAIT_HEADER_2";
        case State::READ_VERSION:   return "READ_VERSION";
        case State::READ_COMMAND:   return "READ_COMMAND";
        case State::READ_LENGTH_L:  return "READ_LENGTH_L";
        case State::READ_LENGTH_H:  return "READ_LENGTH_H";
        case State::READ_PAYLOAD:   return "READ_PAYLOAD";
        case State::READ_CRC_L:     return "READ_CRC_L";
        case State::READ_CRC_H:     return "READ_CRC_H";
        case State::PACKET_READY:   return "PACKET_READY";
        case State::ERROR:          return "ERROR";
        default:                    return "UNKNOWN";
    }
}
