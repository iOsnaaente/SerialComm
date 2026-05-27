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

#include "serial_comm_topic.h"


void SerialCommTopic::execute( const Msg& msg ) {
    if ( !this->initialized_ ) {
        return;
    }
    if ( this->callback_ == nullptr ) {
        return;
    }
    this->callback_(msg);
}


errCode SerialCommTopic::handle_packet( const SerialCommProtoPacket& packet ) {
    if ( !this->initialized_ ) {
        return errCode::ERR_NOT_INITIALIZED;
    }
    Msg msg;
    errCode res = Serializer<Msg>::deserialize( 
        packet.payload, 
        packet.header.payload_len,
        msg
    );
    if ( res != errCode::OK ) {
        return res;
    }
    this->execute( msg );
    return errCode::OK;
}


errCode SerialCommTopic::init( 
    Command command, 
    topic_callback_t callback 
) {
    if ( callback == nullptr ) {
        return errCode::ERR_NULL_POINTER;
    }
    this->command_ = command;
    this->callback_ = callback;
    this->initialized_ = true;
    return errCode::OK;
}


Command SerialCommTopic::command() const {
    return this->command_;
}


bool SerialCommTopic::initialized() const {
    return this->initialized_;
}


bool SerialCommTopic::valid() const {
    return ( this->callback_ != nullptr );
}

        