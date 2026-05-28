/**
 * @file    serial_comm_buffer_writer.hpp
 * @brief   Runtime serialization writer
 */

#pragma once

#include "serial_comm/common/serial_comm_serializer_base.hpp"
#include "serial_comm/common/serial_comm_types.hpp"

#include <type_traits>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <array>


class SerialCommBufferWriter {

    public:

        SerialCommBufferWriter(
            uint8_t* buffer,
            size_t buffer_size,
            SerialCommBufferEndian endian
        ) :
            buffer_(buffer),
            buffer_size_(buffer_size),
            endian_(endian)
        { }


    private:

        uint8_t* buffer_ = nullptr;
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
         * GENERIC WRITE
         * ================================================================= */
        template<typename T>
        bool write(
            const T& value
        ) {

            /* ------------------------------------------------------------
             * Primitive types
             * --------------------------------------------------------- */
            if constexpr (
                std::is_integral_v<T> ||
                std::is_floating_point_v<T>
            ) {
                return this->write_primitive(value);
            }
            /* ------------------------------------------------------------
             * std::array
             * --------------------------------------------------------- */
            else if constexpr (
                SerialCommIsStdArray<T>::value
            ) {
                for ( size_t i = 0; i < value.size(); i++ ) {
                    if ( !this->write(value[i]) ) {
                        return false;
                    }
                }

                return true;
            }

            /* ------------------------------------------------------------
             * Generic serialized types
             * --------------------------------------------------------- */
            else {
                size_t serialized_size = 0;
                bool ok =
                    SerialCommSerializer<T>::serialize(
                        value,
                        &buffer_[offset_],
                        buffer_size_ - offset_,
                        serialized_size
                    );
                if ( !ok ) {
                    return false;
                }
                offset_ += serialized_size;
                return true;
            }
        }


        /* ====================================================================
         * DYNAMIC ARRAY
         * ================================================================= */

        template<typename T>
        bool write_dynamic_array(
            const SerialCommDynamicArray<T>& array
        ) {
            if ( !this->write<uint16_t>(array.size) ) {
                return false;
            }
            for ( uint16_t i = 0; i < array.size; i++ ) {
                if ( !this->write(array.data[i]) ) {
                    return false;
                }
            }
            return true;
        }


        /* ====================================================================
         * DYNAMIC STRING
         * ================================================================= */

        bool write(
            const SerialCommDynamicString& str
        ) {
            if ( !this->write<uint16_t>(str.size) ) {
                return false;
            }
            if (
                offset_ + str.size > buffer_size_
            ) {
                return false;
            }
            memcpy( &buffer_[offset_], str.data, str.size );
            offset_ += str.size;

            return true;
        }


    private:

        /* ====================================================================
         * PRIMITIVE SERIALIZATION
         * ================================================================= */
        template<typename T>
        bool write_primitive(
            const T& value
        ) {
            if ( offset_ + sizeof(T) > buffer_size_ ) {
                return false;
            }
            if ( endian_ == SerialCommBufferEndian::LITTLE ) {
                memcpy( &buffer_[offset_], &value, sizeof(T) );
            }
            else {
                for ( size_t i = 0; i < sizeof(T); i++ ) {
                    buffer_[offset_ + i] =
                        reinterpret_cast<
                            const uint8_t*
                        >(&value)
                        [sizeof(T) - 1 - i];
                }
            }
            offset_ += sizeof(T);
            return true;
        }
};