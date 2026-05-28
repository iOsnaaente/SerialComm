/**
 * @file    serial_comm_serializer_base.hpp
 * @brief   Generic serializer interface for SerialComm generated types
 * @details This file defines the generic serializer abstraction used by
 *          the generated code and middleware runtime.
 *          Responsibilities:
 *              - Generic serialization interface
 *              - Generic deserialization interface
 *              - Compile-time serializer specialization
 *              - Type abstraction layer
 *
 * @note    All generated serializers specialize this template.
 * @author  Bruno Gabriel Flores Sampaio
 */

#pragma once

#include <stdint.h>
#include <stddef.h>


/**
 * @brief   Generic serializer template
 * @tparam  T Type to serialize/deserialize
 * @note    This template MUST be specialized for each generated type.
 */
template<typename T>
struct SerialCommSerializer {

    /**
     * @brief       Serialize object into byte buffer
     * @param[in]   msg Serialized object
     * @param[out]  buffer Output byte buffer
     * @param[in]   buffer_size Buffer maximum size
     * @param[out]  serialized_size Total serialized bytes
     * @return      true Serialization successful
     * @return      false Serialization failed
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
        serialized_size = 0;
        return false;
    }


    /**
     * @brief       Deserialize object from byte buffer
     * @param[in]   buffer Input serialized buffer
     * @param[in]   buffer_size Input buffer size
     * @param[out]  msg Output object
     * @param[out]  consumed_size Consumed bytes
     * @return      true Deserialization successful
     * @return      false Deserialization failed
     */
    static bool deserialize(
        const uint8_t* buffer,
        size_t buffer_size,
        T& msg,
        size_t* consumed_size = nullptr
    ) {
        (void)buffer;
        (void)buffer_size;
        (void)msg;
        if (consumed_size) {
            *consumed_size = 0;
        }
        return false;
    }
};


template<typename T>
using Serializer = SerialCommSerializer<T>;