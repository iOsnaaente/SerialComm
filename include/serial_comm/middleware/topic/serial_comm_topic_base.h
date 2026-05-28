/**
 * @file    serial_comm_topic_base.h
 * @brief   Base polymorphic interface for middleware topics
 */

#pragma once

#include "serial_comm/core/serial_comm_protocol.h"
#include "serial_comm/core/serial_comm_utils.h"

#include <stdint.h>


/* To use the errCode and err_to_str easily */
using namespace SerialCommResult_Codes;

/**
 * @brief Base topic interface
 */
class ITopicBase {
    public:
        virtual ~ITopicBase() = default;

    public:

        /**
         * @brief Get topic command ID
         */
        virtual Command command() const = 0;

        /**
         * @brief   Handle incoming topic packet
         * @param   packet Incoming packet
         * @return  Result code
         */
        virtual errCode handle_packet(
            const SerialCommProtoPacket& packet
        ) = 0;
};