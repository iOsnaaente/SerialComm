/**
 * @file    serial_comm_manager.cpp
 * @brief   High-level semantic middleware manager implementation
 */

#include "serial_comm/middleware/serial_comm_manager.h"


/* To use the errCode and err_to_str easily */
using namespace SerialCommResult_Codes;


static const char* TAG = "SERIAL_COMM_MANAGER";


errCode SerialCommManager::init( const Config& cfg ) {
    if ( this->serial_ == nullptr ) {
        return errCode::ERR_NULL_POINTER;
    }
    if ( this->initialized_ ) {
        return errCode::ERR_ALREADY_INITIALIZED;
    }

    this->cfg_ = cfg;
    
    // Create mutexes
    this->seq_mutex_ = xSemaphoreCreateMutex();
    if ( this->seq_mutex_ == nullptr ) {
        return errCode::ERR_NO_MEMORY;
    }
    this->registry_mutex_ = xSemaphoreCreateMutex();
    if ( this->registry_mutex_ == nullptr ) {
        vSemaphoreDelete( this->seq_mutex_ );
        this->seq_mutex_ = nullptr;
        return errCode::ERR_NO_MEMORY;
    }

    // Clear registries 
    this->clear_registries();

    // Init transaction manager
    errCode res = this->transactions_.init();
    if ( res != errCode::OK ) {
        vSemaphoreDelete( this->seq_mutex_ );
        this->seq_mutex_ = nullptr;
        vSemaphoreDelete( this->registry_mutex_ );
        this->registry_mutex_ = nullptr;
        return res;
    }

    // Register internal packet router for all commands
    for ( uint16_t cmd = 0; cmd < 256; cmd++ ) {
        this->serial_->register_callback(
            static_cast<Command>(cmd),
            serial_packet_callback,
            this
        );
    }
    this->initialized_ = true;
    ESP_LOGI( TAG, "SerialCommManager initialized" );
    return errCode::OK;
}


errCode SerialCommManager::start() {
    if ( !this->initialized_ ) {
        return errCode::ERR_NOT_INITIALIZED;
    }
    if ( this->running_ ) {
        return errCode::ERR_INVALID_STATE;
    }
    this->running_ = true;
    ESP_LOGI( TAG, "SerialCommManager started" );
    return errCode::OK;
}


errCode SerialCommManager::stop() {
    if ( !this->running_ ) {
        return errCode::ERR_INVALID_STATE;
    }
    this->running_ = false;
    ESP_LOGI( TAG, "SerialCommManager stopped" );
    return errCode::OK;
}


errCode SerialCommManager::deinit() {
    if ( this->running_ ) {
        this->stop();
    }
    this->transactions_.deinit();
    if ( this->seq_mutex_ != nullptr ) {
        vSemaphoreDelete( this->seq_mutex_ );
        this->seq_mutex_ = nullptr;
    }
    if( this->registry_mutex_ != nullptr ) {
        vSemaphoreDelete( this->registry_mutex_ );
        this->registry_mutex_ = nullptr;
    }
    this->clear_registries();
    this->initialized_ = false;
    ESP_LOGI( TAG, "SerialCommManager deinitialized" );
    return errCode::OK;
}


void SerialCommManager::serial_packet_callback(
    void* ctx,
    const SerialCommProtoPacket& packet
) {
    if ( ctx == nullptr ) {
        return;
    }
    auto* self = static_cast<SerialCommManager*>( ctx );
    self->route_packet(packet);
}


void SerialCommManager::handle_reply(
    const SerialCommProtoPacket& packet
) {
    ESP_LOGD(
        TAG,
        "Reply packet received: cmd=0x%02X seq=%u len=%u",
        static_cast<uint8_t>( packet.header.command ),
        packet.header.seq_id,
        packet.header.payload_len
    );
    // Try to resolve transaction
    errCode res = this->transactions_.resolve_transaction( packet );
    if ( res != errCode::OK ) {
        ESP_LOGW(
            TAG,
            "Failed to resolve transaction for reply: seq=%u err=%s",
            packet.header.seq_id,
            err_to_str( res )
        );
    } else {
        ESP_LOGD(
            TAG,
            "Transaction resolved for reply: seq=%u",
            packet.header.seq_id
        );
    }
}


uint16_t SerialCommManager::allocate_seq_id() {
    uint16_t seq = 0;
    if ( xSemaphoreTake( this->seq_mutex_, portMAX_DELAY ) == pdTRUE ) {
        seq = this->next_seq_id_++;
        if ( this->next_seq_id_ == 0 ) {
            this->next_seq_id_ = 1;
        }
        xSemaphoreGive( this->seq_mutex_ );
    }
    return seq;
}


errCode SerialCommManager::build_packet(
    Command command,
    uint16_t seq_id,
    const uint8_t* payload,
    size_t payload_len,
    SerialCommProtoPacket& out_packet
) {
    if ( payload_len > SERIAL_COMM_MAX_PAYLOAD ) {
        return errCode::ERR_INVALID_ARG;
    }
    SerialCommProtocol::clear_packet( out_packet );
    out_packet.header.seq_id = seq_id;
    out_packet.header.version = SERIAL_COMM_PROTOCOL_VER1;
    out_packet.header.command = command;
    out_packet.header.payload_len = payload_len;
    if ( payload != nullptr && payload_len > 0 ) {
        memcpy( out_packet.payload, payload, payload_len );
    }
    return errCode::OK;
}


bool SerialCommManager::initialized() const {
    return this->initialized_;
}


bool SerialCommManager::running() const {
    return this->running_;
}


uint16_t SerialCommManager::current_seq_id() const {
    return this->next_seq_id_;
}


void SerialCommManager::clear_registries() {
    memset( this->services_, 0, sizeof(this->services_) );
    memset( this->topics_, 0, sizeof(this->topics_) );
    memset( this->actions_, 0, sizeof(this->actions_) );
}


errCode SerialCommManager::send_reply(
    Command command,
    uint16_t seq_id,
    const uint8_t* payload,
    size_t payload_len
) {
    SerialCommProtoPacket packet;
    errCode err = build_packet(
            make_reply(command), seq_id, payload, payload_len, packet
        );
    if ( err != errCode::OK ) {
        return err;
    }
    return this->serial_->send( packet );
}



void SerialCommManager::route_packet( const SerialCommProtoPacket& packet ) {
    if ( is_reply( packet.header.command ) ) {
        this->handle_reply(packet);
        return;
    }
    if ( this->is_service_command( packet.header.command ) ) {
        this->handle_service_request(packet);
        return;
    }
    if ( this->is_topic_command( packet.header.command ) ) {
        this->handle_topic_message( packet );
        return;
    }
    if ( this->is_action_command( packet.header.command ) ) {
        this->handle_action_packet( packet );
        return;
    }
    ESP_LOGW(
        TAG,
        "Unhandled packet command=0x%02X",
        static_cast<uint8_t>( packet.header.command )
    );
}


void SerialCommManager::handle_topic_message( const SerialCommProtoPacket& packet ) {
    ESP_LOGD(
        TAG,
        "Topic packet received: cmd=0x%02X seq=%u len=%u",
        static_cast<uint8_t>( packet.header.command ),
        packet.header.seq_id,
        packet.header.payload_len
    );
    auto* entry = find_topic( packet.header.command );
    if ( entry == nullptr || entry->topic == nullptr ) {
        ESP_LOGW(TAG, "No topic handler for command=0x%02X", static_cast<uint8_t>( packet.header.command ));
        return;
    }
    errCode res = entry->topic->handle_packet( packet );
    if ( res != errCode::OK ) {
        ESP_LOGW(TAG, "Topic handler failed: cmd=0x%02X err=%s", static_cast<uint8_t>(packet.header.command), err_to_str(res));
    }
}


void SerialCommManager::handle_action_packet( const SerialCommProtoPacket& packet ) {
    ESP_LOGD(
        TAG,
        "Action packet received: cmd=0x%02X seq=%u len=%u",
        static_cast<uint8_t>( packet.header.command ),
        packet.header.seq_id,
        packet.header.payload_len
    );
    auto* entry = find_action( packet.header.command );
    if ( entry == nullptr || entry->action == nullptr ) {
        ESP_LOGW(TAG, "No action handler for command=0x%02X", static_cast<uint8_t>( packet.header.command ));
        return;
    }
    errCode res = entry->action->handle_packet( packet );
    if ( res != errCode::OK ) {
        ESP_LOGW(TAG, "Action handler failed: cmd=0x%02X err=%s", static_cast<uint8_t>(packet.header.command), err_to_str(res));
    }
}


void SerialCommManager::handle_request( const SerialCommProtoPacket& packet ) {
    ESP_LOGD(TAG, "handle_request called for cmd=0x%02X (not implemented fully)", static_cast<uint8_t>(packet.header.command));
}


SerialCommManager::~SerialCommManager() {
    this->deinit();
}

