/**
 * @file    serial_comm_service_base.h
 * @brief   Base polymorphic interface for middleware services
 */

#pragma once

#include "core/serial_comm_protocol.h"
#include "core/serial_comm_utils.h"

#include <stdint.h>


/* To use the errCode and err_to_str easily */
using namespace SerialCommResult_Codes;

/**
 * @brief Base service interface
 */
class IServiceBase {
    public:
        virtual ~IServiceBase() = default;

    public:
        /**
         * @brief Get service command ID
         */
        virtual Command command() const = 0;

        /**
         * @brief   Handle incoming service packet
         * @param   request Incoming request packet
         * @param   response Output reply packet
         * @return  Result code
         */
        virtual errCode handle_packet(
            const SerialCommProtoPacket& request,
            SerialCommProtoPacket& response
        ) = 0;
};