/**
 * @file    serial_comm_serializer.h
 * @brief   Generic serialization abstraction for SerialComm middleware
 * @details Provides typed serialization/deserialization interfaces for
 *          middleware messages.
 *
 *          Responsibilities:
 *              - Struct -> Payload serialization
 *              - Payload -> Struct deserialization
 *              - Middleware message ABI abstraction
 *
 * @note    This is intentionally generic and must be specialized by the
 *          application layer for each message type.
 *
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Created on 26 of May, 2026
 */

#pragma once

#include "core/serial_comm_utils.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>


/* To use the errCode and err_to_str easily */
using namespace SerialCommResult_Codes;

/**
 * @brief Generic serializer interface
 * @tparam T Message type
 */
template<typename T>
struct Serializer {

    /**
     * @brief       Serialize message into payload buffer
     * @param[in]   msg Message object
     * @param[out]  buffer Output payload buffer
     * @param[in]   buffer_size Buffer capacity
     * @param[out]  serialized_size Bytes written
     * @return      true if serialization succeeded
     * @return      false otherwise
     */
    static bool serialize(
        const T& msg,
        uint8_t* buffer,
        size_t buffer_size,
        size_t& serialized_size
    ) {
        (void)msg;
        (void)buffer;
        (void)buffer_size;
        (void)serialized_size;
        return false;
    }


    /**
     * @brief       Deserialize payload buffer into message object
     * @param[in]   buffer Input payload buffer
     * @param[in]   buffer_size Payload size
     * @param[out]  msg Output message object
     * @return      true if deserialization succeeded
     * @return      false otherwise
     */
    static bool deserialize(
        const uint8_t* buffer,
        size_t buffer_size,
        T& msg
    ) {
        (void)buffer;
        (void)buffer_size;
        (void)msg;
        return false;
    }
};