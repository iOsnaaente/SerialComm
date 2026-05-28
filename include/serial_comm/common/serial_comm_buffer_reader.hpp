/**
 * @file    serial_comm_buffer_reader.hpp
 * @brief   Runtime deserialization reader
 */

#pragma once

#include "serial_comm/common/serial_comm_serializer_base.hpp"
#include "serial_comm/common/serial_comm_types.hpp"

#include <type_traits>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <array>


class SerialCommBufferReader {

    public:

        SerialCommBufferReader(
            const uint8_t* buffer,
            size_t buffer_size,
            SerialCommBufferEndian endian
        ) :
            buffer_(buffer),
            buffer_size_(buffer_size),
            endian_(endian)
        { }


    private:
        const uint8_t* buffer_ = nullptr;
        size_t buffer_size_ = 0;
        size_t offset_ = 0;
        SerialCommBufferEndian endian_ =
            SerialCommBufferEndian::LITTLE;

    public:
        size_t offset() const {
            return this->offset_;
        }


    public:

        /* ====================================================================
         * GENERIC READ
         * ================================================================= */
        template<typename T>
        bool read( T& value ) {
            /* ------------------------------------------------------------
             * Primitive types
             * --------------------------------------------------------- */
            if constexpr (
                std::is_integral_v<T> ||
                std::is_floating_point_v<T>
            ) {
                return this->read_primitive(value);
            }

            /* ------------------------------------------------------------
             * std::array
             * --------------------------------------------------------- */
            else if constexpr (
                SerialCommIsStdArray<T>::value
            ) {
                for ( size_t i = 0; i < value.size(); i++ ) {
                    if ( !this->read(value[i]) ) {
                        return false;
                    }
                }
                return true;
            }

            /* ------------------------------------------------------------
             * Generic serialized types
             * --------------------------------------------------------- */
            else {
                size_t remaining = buffer_size_ - offset_;
                size_t consumed_size = 0;
                bool ok = SerialCommSerializer<T>::deserialize(
                    &buffer_[offset_],
                    remaining,
                    value,
                    &consumed_size
                );
                if ( !ok ) {
                    return false;
                }
                offset_ += consumed_size;
                return true;
            }
        }

        /* ====================================================================
         * DYNAMIC ARRAY
         * ================================================================= */
        template<typename T>
        bool read_dynamic_array(
            SerialCommDynamicArray<T>& array
        ) {
            uint16_t recv_size = 0;
            if ( !this->read<uint16_t>(recv_size) ) {
                return false;
            }
            if ( recv_size > array.capacity ) {
                return false;
            }
            array.size = recv_size;
            for ( uint16_t i = 0; i < recv_size; i++ ) {
                if ( !this->read(array.data[i]) ) {
                    return false;
                }
            }
            return true;
        }


        /* ====================================================================
         * DYNAMIC STRING
         * ================================================================= */
        bool read( SerialCommDynamicString& str ) {
            uint16_t recv_size = 0;
            if ( !this->read<uint16_t>(recv_size) ) {
                return false;
            }
            if ( recv_size > str.capacity ) {
                return false;
            }
            if ( offset_ + recv_size > buffer_size_ ) {
                return false;
            }
            memcpy( str.data, &buffer_[offset_], recv_size );
            str.size = recv_size;
            offset_ += recv_size;
            return true;
        }

    private:

        /* ====================================================================
         * PRIMITIVE DESERIALIZATION
         * ================================================================= */
        template<typename T>
        bool read_primitive( T& value ) {
            if ( offset_ + sizeof(T) > buffer_size_ ) {
                return false;
            }
            if ( endian_ == SerialCommBufferEndian::LITTLE ) {
                memcpy( &value, &buffer_[offset_], sizeof(T) );
            }
            else {
                for ( size_t i = 0; i < sizeof(T); i++ ) {
                    reinterpret_cast<uint8_t*>(&value)
                        [sizeof(T) - 1 - i] =
                            buffer_[offset_ + i];
                }
            }
            offset_ += sizeof(T);
            return true;
        }
};