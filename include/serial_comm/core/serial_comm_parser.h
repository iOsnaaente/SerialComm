/**
 * @file    serial_comm_parser.h
 * @brief   Incremental stream parser for serial communication middleware
 * @details This file defines the SerialCommParser class, which implements 
 *          an incremental receive buffer and parser for the serial 
 *          communication middleware. 
 *          The parser allows for efficient handling of incoming data 
 *          streams, supporting partial messages and buffering until 
 *          complete messages are received.
 *          It also provides error handling for malformed messages and buffer
 *          overflows.
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Created on 22 of May, 2026 
 */

#pragma once

#include "serial_comm/core/serial_comm_protocol.h"
#include "serial_comm/core/serial_comm_utils.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>


class SerialCommParser {
public:

    /**
     * @brief   Parser internal states
     * @details This enum defines the states of the parser state machine
     */
    enum class State : uint8_t {
        WAIT_HEADER_0 = 0,
        WAIT_HEADER_1,
        WAIT_HEADER_2,
        GET_SEQ_ID_L,
        GET_SEQ_ID_H,
        READ_VERSION,
        READ_COMMAND,
        READ_LENGTH_L,
        READ_LENGTH_H,
        READ_PAYLOAD,
        READ_CRC_L,
        READ_CRC_H,
        PACKET_READY,
        ERROR
    };

private:
    /** Internal parser state  */
    State state_;
    /** Internal packet buffer  */
    SerialCommProtoPacket packet_;
    /** Current payload index  */
    uint16_t payload_index_;
    /** Temporary CRC low byte  */
    uint8_t crc_l_;

public:
    /**
     * @brief Constructor
     */
    SerialCommParser() :
        state_( State::WAIT_HEADER_0 ),
        payload_index_(0),
        crc_l_(0) { 
        SerialCommProtocol::clear_packet( packet_ );
    }


    /**
     * @brief Destructor
     */
    virtual ~SerialCommParser() = default;


public:
    /**
     * @brief       Parse a single byte from stream
     * @details     Incrementally parses incoming serial stream.
     * @param[in]   byte Incoming stream byte
     * @param[out]  out_packet Parsed packet
     * @return      True Packet completed parsed and ready in out_packet
     * @return      False Packet not completed yet
     */
    bool parse_byte( uint8_t byte, SerialCommProtoPacket& out_packet );

    /**
     * @brief       Parse a byte stream buffer
     * @param[in]   data Stream buffer
     * @param[in]   len Buffer length
     * @param[out]  consumed_bytes Number of bytes consumed from buffer
     * @param[out]  out_packet Parsed packet
     * @return      True Packet completed
     * @return      False No valid packet found
     */
    bool parse_next_packet(
        const uint8_t* data,
        size_t len,
        size_t &consumed_bytes,
        SerialCommProtoPacket& out_packet
    );

    /**
     * @brief       Reset parser state machine
     * @details     Clears all parser internal states and buffers.
     */
    void reset();


public:

    /**
     * @brief       Get current parser state
     * @return      Current parser state
     */
    State state() const;


    /**
     * @brief       Get current payload index
     * @return      Current payload byte index
     */
    size_t payload_index() const;

    /**
     * @brief       Get expected payload length
     * @return      Payload expected size
     */
    uint16_t expected_payload_size() const;


public:
    /**
     * @brief       Convert parser state to string
     * @note        Useful for logs/debugging.
     * @param[in]   state Parser state
     * @return      State string
     */
    static const char* state_to_str( State state );

};