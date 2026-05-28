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

#include "serial_comm/messages/serial_comm_messages.h"

#include "serial_comm/core/serial_comm_protocol.h"
#include "serial_comm/core/serial_comm_utils.h"

#include "serial_comm/middleware/action/serial_comm_action_base.h"
#include "serial_comm/common/serial_comm_serializer_base.hpp"

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
        ) {
            if ( goal_cb == nullptr ) {
                return errCode::ERR_NULL_POINTER;
            }
            this->command_ = command;
            this->goal_callback_ = goal_cb;
            this->feedback_callback_ = feedback_cb;
            this->initialized_ = true;
            return errCode::OK;
        }


    private:
        /**
         * @brief   Execute action goal
         * @param   goal Goal object
         * @param   result Result object
         * @return  true  Goal executed successfully
         * @return  false Goal execution failed
         */
        bool execute_goal( const Goal& goal, Result& result ) {
            if ( !this->initialized_ ) {
                return false;
            }
            if ( this->goal_callback_ == nullptr ) {
                return false;
            }
            return this->goal_callback_( goal, result );
        }


    public:
        /**
         * @brief   Publish feedback
         * @param   feedback Feedback object
         */
        void publish_feedback( const Feedback& feedback ) {
            if ( !this->initialized_ ) {
                return;
            }
            if ( this->feedback_callback_ == nullptr ) {
                return;
            }
            this->feedback_callback_( feedback );
        }


        /**
         * @brief Get action command ID
         */
        Command command() const override {
            return this->command_;
        }


        /**
         * @brief   Handle incoming action packet
         * @param   packet Incoming action packet
         * @return  Result code
         */
        errCode handle_packet(
            const SerialCommProtoPacket& packet
        ) override {
            if ( !this->initialized_ ) {
                return errCode::ERR_NOT_INITIALIZED;
            }
            Goal goal;
            bool ret = SerialCommSerializer<Goal>::deserialize(
                packet.payload,
                packet.header.payload_len,
                goal
            );
            if ( !ret ) {
                return errCode::ERR_PARSER;
            }

            Result result;
            ret = this->execute_goal( goal, result );
            if ( !ret ) {
                return errCode::ERR_FAIL;
            }

            return errCode::OK;
        }


        /**
         * @brief Check if action is initialized
         * @return true  Action is initialized
         * @return false Action is not initialized
         */
        bool initialized() const {
            return this->initialized_;
        }

        /**
         * @brief Check if action is valid
         * @return true  Action is valid
         * @return false Action is invalid (e.g. missing callbacks)
         */
        bool valid() const {
            return ( this->goal_callback_ != nullptr );
        }


    public:
        /**
         * @brief Destructor
         */
        virtual ~SerialCommAction() = default;
};