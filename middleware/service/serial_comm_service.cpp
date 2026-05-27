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

#include "serial_comm_service.h"


using namespace SerialCommResult_Codes;


bool SerialCommService::execute( const Req& request, Res& response ) {
    if ( !this->initialized_ ) {
        return false;
    }
    if ( this->callback_ == nullptr ) {
        return false;
    }
    return this->callback_( request, response );
}


errCode SerialCommService::handle_packet(
    const SerialCommProtoPacket& request,
    SerialCommProtoPacket& response
) override {
    if ( !this->initialized_ ) {
        return errCode::ERR_NOT_INITIALIZED;
    }
    if ( this->callback_ == nullptr ) {
        return errCode::ERR_NULL_POINTER;
    }

    // Deserialize request
    Req request_msg;
    bool ret = Serializer<Req>::deserialize( 
        request.payload, 
        request.header.payload_len, 
        request_msg 
    );
    if ( !ret ) {
        return errCode::ERR_PARSER;
    }

    // Execute callback
    Res response_msg;
    ret = this->execute( request_msg, response_msg );
    if ( !ret ) {
        return errCode::ERR_FAIL;
    }

    // Serialize response
    size_t serializer_size = 0;
    ret = Serializer<Res>::serialize(
        response_msg,
        response.payload,
        sizeof(response.payload),
        serializer_size
    );
    if ( !ret ) {
        return errCode::ERR_PARSER;
    }

    // Build response packet
    SerialCommProtocol::clear_packet( response );
    response.header.seq_id = request.header.seq_id;
    response.header.version = SERIAL_COMM_PROTOCOL_VER1;
    response.header.command = make_reply( request.header.command );
    response.header.payload_len = static_cast<uint16_t>( serializer_size );
    return errCode::OK;
}


errCode SerialCommService::init( Command command, service_callback_t callback ) {
    if ( callback == nullptr ) {
        return errCode::ERR_NULL_POINTER;
    }
    this->command_ = command;
    this->callback_ = callback;
    this->initialized_ = true;
    return errCode::OK;
}


Command SerialCommService::command() const {
    return this->command_;
}


bool SerialCommService::initialized() const {
    return this->initialized_;
}


bool SerialCommService::valid() const {
    return ( this->callback_ != nullptr );
}
