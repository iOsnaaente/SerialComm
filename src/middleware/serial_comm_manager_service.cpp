/**
 * @file    serial_comm_manager_service.cpp
 * @brief   Serial communication middleware service handling 
 *          implementation
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Created on 26 of May, 2026
 */

#include "serial_comm/middleware/serial_comm_manager.h"

static const char* TAG = "SERIAL_COMM_MANAGER_SERVICE";


void SerialCommManager::handle_service_request(
    const SerialCommProtoPacket& packet
) {
    auto* entry = this->find_service( packet.header.command );
    if ( entry == nullptr || entry->service == nullptr ) {
        ESP_LOGW(
            TAG,
            "No service handler for command=0x%02X",
            static_cast<uint8_t>( packet.header.command )
        );
        return;
    }

    static SerialCommProtoPacket response;
    SerialCommProtocol::clear_packet( response );
    errCode res = entry->service->handle_packet( packet, response );
    if ( res != errCode::OK ) {
        ESP_LOGW(
            TAG,
            "Service handler failed: cmd=0x%02X err=%s",
            static_cast<uint8_t>( packet.header.command ),
            err_to_str( res )
        );
        return;
    }

    if ( this->cfg_.enable_auto_reply ) {
        res = this->serial_->send( response );
        if ( res != errCode::OK ) {
            ESP_LOGW(
                TAG,
                "Failed to send service reply: cmd=0x%02X err=%s",
                static_cast<uint8_t>( packet.header.command ),
                err_to_str( res )
            );
        }
    }
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