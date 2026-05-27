/**
 * @file    serial_comm_action.h
 * @brief   Generic Action abstraction for SerialComm middleware
 * @details Provides a ROS-like action abstraction for long-running
 *          asynchronous tasks over the SerialComm protocol.
 *
 *          Responsibilities:
 *              - Goal handling
 *              - Feedback handling
 *              - Result handling
 *              - Action execution abstraction
 *              - Cancel support (future)
 *
 * @note    Initial implementation only defines the action abstraction.
 *          Full execution engine and state machine are future work.
 *
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Created on 26 of May, 2026
 */

#pragma once

#include "messages/serial_comm_messages.h"

#include "core/serial_comm_protocol.h"
#include "core/serial_comm_utils.h"

#include "serial_comm_action_base.h"
#include "serial_comm_serializer.h"

#include <stdint.h>
#include <stddef.h>

/* To use the errCode and err_to_str easily */
using namespace SerialCommResult_Codes;


/**
 * @brief Generic Action abstraction
 * @tparam Goal     Action goal type
 * @tparam Feedback Action feedback type
 * @tparam Result   Action result type
 */
template< typename Goal, typename Feedback, typename Result >
class SerialCommAction : public IActionBase {
    public:

        /**
         * @brief Goal callback type
         * @param goal Incoming goal
         * @param result Final result
         * @return true  Goal accepted/executed
         * @return false Goal rejected/failed
         */
        using goal_callback_t = bool (*)( const Goal& goal, Result& result );


        /**
         * @brief Feedback callback type
         * @param feedback Action feedback
         */
        using feedback_callback_t = void (*)( const Feedback& feedback );


    private:

        /**
         * @brief Action command ID
         */
        Command command_ = static_cast<Command>(0);

        /**
         * @brief Goal callback
         */
        goal_callback_t goal_callback_ = nullptr;

        /**
         * @brief Feedback callback
         */
        feedback_callback_t feedback_callback_ = nullptr;
        bool initialized_ = false;


    public:

        /**
         * @brief Constructor
         */
        SerialCommAction() = default;


    public:

        /**
         * @brief   Initialize action
         * @param   command Action command ID
         * @param   goal_cb Goal callback
         * @param   feedback_cb Feedback callback
         * @return  Result code
         */
        errCode init(
            Command command,
            goal_callback_t goal_cb,
            feedback_callback_t feedback_cb = nullptr
        );


    private:
        /**
         * @brief   Execute action goal
         * @param   goal Goal object
         * @param   result Result object
         * @return  true  Goal executed successfully
         * @return  false Goal execution failed
         */
        bool execute_goal( const Goal& goal, Result& result );


    public:
        /**
         * @brief   Publish feedback
         * @param   feedback Feedback object
         */
        void publish_feedback( const Feedback& feedback );


        /**
         * @brief Get action command ID
         */
        Command command() const override;


        /**
         * @brief   Handle incoming action packet
         * @param   packet Incoming action packet
         * @return  Result code
         */
        errCode handle_packet(
            const SerialCommProtoPacket& packet
        ) override;


        /**
         * @brief Check if action is initialized
         * @return true  Action is initialized
         * @return false Action is not initialized
         */
        bool initialized() const;

        /**
         * @brief Check if action is valid
         * @return true  Action is valid
         * @return false Action is invalid (e.g. missing callbacks)
         */
        bool valid() const;


    public:
        /**
         * @brief Destructor
         */
        virtual ~SerialCommAction() = default;
};