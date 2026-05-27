/**
 * @file    serial_comm_topic.h
 * @brief   Generic Topic abstraction for SerialComm middleware
 * @details Provides a ROS-like publish/subscribe abstraction over the
 *          SerialComm protocol.
 *
 *          Responsibilities:
 *              - Typed publish/subscribe abstraction
 *              - Topic callback encapsulation
 *              - Lightweight async message delivery
 *
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Created on 26 of May, 2026
 */

#pragma once

#include "messages/serial_comm_messages.h"

#include "core/serial_comm_protocol.h"
#include "core/serial_comm_utils.h"

#include "serial_comm_topic_base.h"
#include "serial_comm_serializer.h"

#include <stdint.h>
#include <stddef.h>


/* To use the errCode and err_to_str easily */
using namespace SerialCommResult_Codes;


/**
 * @brief   Generic Topic abstraction
 * @tparam  Msg Topic message type
 */
template<typename Msg>
class SerialCommTopic : public ITopicBase {
    public:

        /**
         * @brief Topic callback type
         * @param msg Received message
         */
        using topic_callback_t = void (*)( const Msg& msg );


    private:

        /**
         * @brief Topic command ID
         */
        Command command_ = static_cast<Command>(0);

        /**
         * @brief Subscription callback
         */
        topic_callback_t callback_ = nullptr;
        bool initialized_ = false;


    private:
        /**
         * @brief Execute topic callback
         * @param msg Received message
         */
        void execute( const Msg& msg );


    public:

        /**
         * @brief Constructor
         */
        SerialCommTopic() = default;


    public:

        /**
         * @brief   Initialize topic
         * @param   command Topic command ID
         * @param   callback Topic callback
         * @return  Result code
         */
        errCode init( Command command, topic_callback_t callback );


        /**
         * @brief   Handle incoming topic packet
         * @param   packet Incoming packet
         * @return  Result code
         */
        errCode handle_packet( const SerialCommProtoPacket& packet ) override;


        /**
         * @brief   Get topic command ID
         * @return  Command ID
         */
        Command command() const override;


        /**
         * @brief   Check if topic is initialized
         * @return  True if initialized
         * @return  False if not initialized
         */
        bool initialized() const;


        /**
         * @brief Check if topic callback is valid
         * @return true Callback is valid
         * @return false Callback is not valid
         */
        bool valid() const;

        
    public:

        /**
         * @brief Destructor
         */
        virtual ~SerialCommTopic() = default;
};