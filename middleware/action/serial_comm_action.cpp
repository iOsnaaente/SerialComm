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

#include "serial_comm_action.h"


errCode SerialCommAction::init(
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

bool SerialCommAction::execute_goal( const Goal& goal, Result& result ) {
    if ( !this->initialized_ ) {
        return false;
    }
    if ( this->goal_callback_ == nullptr ) {
        return false;
    }
    return this->goal_callback_( goal, result );
}

void SerialCommAction::publish_feedback( const Feedback& feedback ) {
    if ( !this->initialized_ ) {
        return;
    }
    if ( this->feedback_callback_ == nullptr ) {
        return;
    }
    this->feedback_callback_( feedback );
}

Command SerialCommAction::command() const override {
    return this->command_;
}


errCode SerialCommAction::handle_packet(
    const SerialCommProtoPacket& packet
){
    if ( !this->initialized_ ) {
        return errCode::ERR_NOT_INITIALIZED;
    }
    Goal goal;
    bool ret = Serializer<Goal>::deserialize( 
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

    /**
     * @TODO: Publish result packet
     * - feedback stream 
     * - goal handling
     * - cancelation 
     * - async execution support
     */

    return errCode::OK;
}


bool SerialCommAction::initialized() const {
    return this->initialized_;
}

bool SerialCommAction::valid() const {
    return ( this->goal_callback_ != nullptr );
}
