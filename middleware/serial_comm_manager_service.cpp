/**
 * @file    serial_comm_manager_service.cpp
 * @brief   Serial communication middleware service handling 
 *          implementation
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Created on 26 of May, 2026
 */

#include "serial_comm_manager.h"


template<typename Req, typename Res>
errCode SerialCommManager::create_service(
    SerialCommService<Req, Res>* service
) {
    if ( service == nullptr ) {
        return errCode::ERR_NULL_POINTER;
    }
    if ( !service->initialized() ) {
        return errCode::ERR_NOT_INITIALIZED;
    }
    if ( xSemaphoreTake( this->registry_mutex_, portMAX_DELAY ) != pdTRUE ) {
        return errCode::ERR_TIMEOUT;
    }
    for ( size_t i = 0; i < MAX_SERVICES; i++ ) {
        if ( !this->services_[i].used ) {
            this->services_[i].used = true;
            this->services_[i].command = service->command();
            this->services_[i].service = service;
            xSemaphoreGive( this->registry_mutex_ );
            return errCode::OK;
        }
    }
    xSemaphoreGive( this->registry_mutex_ );
    return errCode::ERR_NO_MEMORY;
}


template<typename Req, typename Res>
errCode SerialCommManager::call_service(
    Command command,
    const Req& request,
    Res& response,
    uint32_t timeout_ms
) {
    uint8_t payload[ SERIAL_COMM_MAX_PAYLOAD_V1 ];
    size_t payload_size = 0;

    // SERIALIZE REQUEST
    bool ok = Serializer<Req>::serialize(
        request,
        payload,
        sizeof(payload),
        payload_size
    );
    if ( !ok ) {
        return errCode::ERR_FAIL;
    }
    // ALLOCATE SEQ ID
    uint16_t seq_id = this->allocate_seq_id();

    // CREATE TRANSACTION
    errCode err = this->transactions_.create_transaction( seq_id );
    if ( err != errCode::OK ) {
        return err;
    }

    // BUILD REQUEST PACKET
    SerialCommProtoPacket request_packet;
    err = build_packet(
        command,
        seq_id,
        payload,
        payload_size,
        request_packet
    );
    if ( err != errCode::OK ) {
        return err;
    }

    // SEND REQUEST
    err = this->serial_->send( request_packet );
    if ( err != errCode::OK ) {
        return err;
    }

    // WAIT REPLY
    SerialCommProtoPacket reply_packet;
    err = this->transactions_.wait_reply(
        seq_id,
        reply_packet,
        timeout_ms
    );
    if ( err != errCode::OK ) {
        return err;
    }
    
    //  DESERIALIZE RESPONSE
    ok = Serializer<Res>::deserialize(
        reply_packet.payload,
        reply_packet.header.payload_len,
        response
    );
    if ( !ok ) {
        return errCode::ERR_FAIL;
    }
    return errCode::OK;
}


void SerialCommManager::handle_service_request(
    const SerialCommProtoPacket& packet
) {
    ESP_LOGD(
        TAG,
        "Request packet received: cmd=0x%02X seq=%u len=%u",
        static_cast<uint8_t>( packet.header.command ),
        packet.header.seq_id,
        packet.header.payload_len
    );

    /**
     * TODO:
     *  - SERVICE ROUTING
     *  - TOPIC ROUTING
     *  - ACTION ROUTING
     * --------------------------------------------------------------------- */
}


SerialCommManager::ServiceEntry* SerialCommManager::find_service(
    Command command
) {
    xSemaphoreTake( this->registry_mutex_, portMAX_DELAY );
    for ( size_t i = 0; i < MAX_SERVICES; i++ ) {
        if ( 
            this->services_[i].used && 
            this->services_[i].command == command
        ) {
            xSemaphoreGive( this->registry_mutex_ );
            return &this->services_[i];
        }
    }
    xSemaphoreGive( this->registry_mutex_ );
    return nullptr;
}


bool SerialCommManager::is_service_command( Command command ) const {
    for ( size_t i = 0; i < MAX_SERVICES; i++ ) {
        if (
            this->services_[i].used &&
            this->services_[i].command == command
        ) {
            return true;
        }
    }
    return false;
}