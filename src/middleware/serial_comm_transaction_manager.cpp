/**
 * @file    serial_comm_transaction_manager.h
 * @brief   Transaction manager for SerialComm middleware
 * @details Responsible for:
 *              - Pending request tracking
 *              - Reply matching using seq_id
 *              - Request timeout management
 *              - Synchronous service waiting
 *              - Transaction lifecycle management
 *
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Created on 26 of May, 2026
 */


#include "serial_comm/middleware/serial_comm_transaction_manager.h"


/* To use the errCode and err_to_str easily */
using namespace SerialCommResult_Codes;


errCode SerialCommTransactionManager::init() {
    if ( this->initialized_ ) {
        return errCode::ERR_ALREADY_INITIALIZED;
    }
    this->mutex_ = xSemaphoreCreateMutex();
    if ( this->mutex_ == nullptr ) {
        return errCode::ERR_NO_MEMORY;
    }
    for ( size_t i = 0; i < MAX_TRANSACTIONS; i++ ) {
        this->transactions_[i].semaphore = xSemaphoreCreateBinary();
        if ( this->transactions_[i].semaphore == nullptr ) {
            return errCode::ERR_NO_MEMORY;
        }
    }
    this->initialized_ = true;
    return errCode::OK;
}


errCode SerialCommTransactionManager::deinit() {
    for ( size_t i = 0; i < MAX_TRANSACTIONS; i++ ) {
        if ( this->transactions_[i].semaphore != nullptr ) {
            vSemaphoreDelete( this->transactions_[i].semaphore );
            this->transactions_[i].semaphore = nullptr;
        }
    }
    if ( this->mutex_ != nullptr ) {
        vSemaphoreDelete( this->mutex_ );
        this->mutex_ = nullptr;
    }
    this->initialized_ = false;
    return errCode::OK;
}


errCode SerialCommTransactionManager::create_transaction( uint16_t seq_id ) {
    if ( !this->initialized_ ) {
        return errCode::ERR_NOT_INITIALIZED;
    }
    xSemaphoreTake( this->mutex_, portMAX_DELAY );
    for ( size_t i = 0; i < MAX_TRANSACTIONS; i++ ) {
        if ( !this->transactions_[i].active ) {
            this->transactions_[i].active = true;
            this->transactions_[i].completed = false;
            this->transactions_[i].seq_id = seq_id;
            xSemaphoreGive( this->mutex_ );
            return errCode::OK;
        }
    }
    xSemaphoreGive( this->mutex_ );
    return errCode::ERR_NO_MEMORY;
}


errCode SerialCommTransactionManager::wait_reply(
    uint16_t seq_id,
    SerialCommProtoPacket& out_reply,
    uint32_t timeout_ms
) {
    Transaction* transaction = this->find_transaction(seq_id);
    if ( transaction == nullptr ) {
        return errCode::ERR_EMPTY;
    }
    if ( xSemaphoreTake(
            transaction->semaphore,
            pdMS_TO_TICKS(timeout_ms)
        ) != pdTRUE
    ) {
        this->destroy_transaction(seq_id);
        return errCode::ERR_TIMEOUT;
    }
    out_reply = transaction->reply;
    this->destroy_transaction(seq_id);
    return errCode::OK;
}


errCode SerialCommTransactionManager::resolve_transaction(
    const SerialCommProtoPacket& packet
) {
    Transaction* transaction = 
        this->find_transaction( packet.header.seq_id );
    if ( transaction == nullptr ) {
        return errCode::ERR_EMPTY;
    }
    transaction->reply = packet;
    transaction->completed = true;
    xSemaphoreGive( transaction->semaphore );
    return errCode::OK;
}


SerialCommTransactionManager::Transaction* SerialCommTransactionManager::find_transaction( uint16_t seq_id ) {
    for ( size_t i = 0; i < MAX_TRANSACTIONS; i++ ) {
        if (
            this->transactions_[i].active &&
            this->transactions_[i].seq_id == seq_id
        ) {
            return &this->transactions_[i];
        }
    }
    return nullptr;
}


void SerialCommTransactionManager::destroy_transaction( uint16_t seq_id ) {
    xSemaphoreTake( this->mutex_, portMAX_DELAY );
    for ( size_t i = 0; i < MAX_TRANSACTIONS; i++ ) {
        if (
            this->transactions_[i].active &&
            this->transactions_[i].seq_id == seq_id
        ) {
            this->transactions_[i].active = false;
            this->transactions_[i].completed = false;
            this->transactions_[i].seq_id = 0;
            memset( 
                &this->transactions_[i].reply, 
                0, 
                sizeof( SerialCommProtoPacket )
            );
            break;
        }
    }
    xSemaphoreGive( this->mutex_ );
}


SerialCommTransactionManager::~SerialCommTransactionManager() {
    this->deinit();
};
