/**
 * @file    serial_comm_action_base.h
 * @brief   Base polymorphic interface for middleware actions
 */

#pragma once

#include "serial_comm/core/serial_comm_protocol.h"
#include "serial_comm/core/serial_comm_utils.h"

#include <stdint.h>


/* To use the errCode and err_to_str easily */
using namespace SerialCommResult_Codes;

/**
 * @brief Base action interface
 */
class IActionBase {
    public:
        virtual ~IActionBase() = default;

    public:

        /**
         * @brief Get action command ID
         */
        virtual Command command() const = 0;

        /**
         * @brief   Handle incoming action packet
         * @param   packet Incoming action packet
         * @return  Result code
         */
        virtual errCode handle_packet(
            const SerialCommProtoPacket& packet
        ) = 0;
};