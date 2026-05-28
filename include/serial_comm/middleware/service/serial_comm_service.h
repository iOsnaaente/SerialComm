/**
 * @file    serial_comm_service.h
 * @brief   Generic Service abstraction for SerialComm middleware
 * @details Provides a ROS-like request/reply abstraction over the
 *          SerialComm protocol.
 *
 *          Responsibilities:
 *              - Typed request/reply callbacks
 *              - Automatic serialization abstraction
 *              - Automatic response packet generation
 *              - Service callback encapsulation
 *
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Created on 26 of May, 2026
 */

#pragma once

#include "serial_comm/messages/serial_comm_messages.h"

#include "serial_comm/core/serial_comm_protocol.h"
#include "serial_comm/core/serial_comm_utils.h"

#include "serial_comm/middleware/service/serial_comm_service_base.h"
#include "serial_comm/common/serial_comm_serializer_base.hpp"

#include <stdint.h>
#include <stddef.h>

/* To use the errCode and err_to_str easily */
using namespace SerialCommResult_Codes;


/**
 * @brief Generic SerialComm Service abstraction
 * @tparam Req Request type
 * @tparam Res Response type
 */
template<typename Req, typename Res>
class SerialCommService: public IServiceBase  {
    public:

        /**
         * @brief Service callback type
         * @param request Typed request object
         * @param response Typed response object
         * @return true  Service executed successfully
         * @return false Service execution failed
         */
        using service_callback_t =
            bool (*)( const Req& request, Res& response );


    public:

        /**
         * @brief Constructor
         */
        SerialCommService() = default;


    private:

        /**
         * @brief Registered command ID
         */
        Command command_ = static_cast<Command>(0);

        /**
         * @brief Service callback
         */
        service_callback_t callback_ = nullptr;
        bool initialized_ = false;


    private: 
        /**
         * @brief Execute service callback
         * @param request Request object
         * @param response Response object
         * @return true  Callback executed successfully
         * @return false Callback execution failed
         */
        bool execute( const Req& request, Res& response ) {
            if ( !this->initialized_ ) {
                return false;
            }
            if ( this->callback_ == nullptr ) {
                return false;
            }
            return this->callback_( request, response );
        }


    public:

        /**
         * @brief Initialize service
         * @param command Service command ID
         * @param callback User callback
         * @return Result code
         */
        errCode init( Command command, service_callback_t callback ) {
            if ( callback == nullptr ) {
                return errCode::ERR_NULL_POINTER;
            }
            this->command_ = command;
            this->callback_ = callback;
            this->initialized_ = true;
            return errCode::OK;
        }

    public:

        /**
         * @brief   Get command ID
         * @return  Command ID
         */
        Command command() const override {
            return this->command_;
        }

        
        /**
         * @brief   Handle incoming service request packet, execute 
         *          callback and generate response packet
         * @param   request Incoming request packet
         * @param   response Output response packet to fill
         * @return  Result code
         */
        errCode handle_packet(
            const SerialCommProtoPacket& request,
            SerialCommProtoPacket& response
        ) override {
            if ( !this->initialized_ ) {
                return errCode::ERR_NOT_INITIALIZED;
            }
            if ( this->callback_ == nullptr ) {
                return errCode::ERR_NULL_POINTER;
            }

            Req request_msg;
            size_t consumed_size = 0;
            bool ret = SerialCommSerializer<Req>::deserialize(
                request.payload,
                request.header.payload_len,
                request_msg,
                &consumed_size
            );
            if ( !ret ) {
                ESP_LOGW(
                    "SERVICE",
                    "Deserialization failed for cmd=0x%02X, consumed=%u/%u",
                    static_cast<uint8_t>(request.header.command),
                    consumed_size,
                    request.header.payload_len
                );
                return errCode::ERR_PARSER;
            }

            Res response_msg;
            ret = this->execute( request_msg, response_msg );
            if ( !ret ) {
                return errCode::ERR_FAIL;
            }

            size_t serializer_size = 0;
            ret = SerialCommSerializer<Res>::serialize(
                response_msg,
                response.payload,
                sizeof(response.payload),
                serializer_size
            );
            if ( !ret ) {
                return errCode::ERR_PARSER;
            }

            SerialCommProtocol::clear_packet( response );
            response.header.seq_id = request.header.seq_id;
            response.header.version = SERIAL_COMM_PROTOCOL_VER1;
            response.header.command = make_reply( request.header.command );
            response.header.payload_len = static_cast<uint16_t>( serializer_size );
            return errCode::OK;
        }
        

        /**
         * @brief   Check if service is initialized
         * @return  true  Service is initialized
         * @return  false Service is not initialized
         */
        bool initialized() const {
            return this->initialized_;
        }


        /**
         * @brief Check if callback is valid
         * @return true Callback is valid
         * @return false Callback is not valid
         */
        bool valid() const {
            return ( this->callback_ != nullptr );
        }


    public:
    
        /**
         * @brief Destructor
         */
        virtual ~SerialCommService() = default;
};