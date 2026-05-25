/**
 * @file    serial_comm_dispatcher.cpp
 * @brief   Dispatcher implementation for SerialComm middleware
 */


#include "serial_comm_dispatcher.h"


/* To use the errCode and err_to_str easily */
using namespace SerialCommResult_Codes;

static const char* TAG = "SERIAL_DISPATCHER";


SerialCommDispatcher::SerialCommDispatcher() {
    memset( this->callbacks_, 0, sizeof(this->callbacks_) );
}


errCode SerialCommDispatcher::init( const Config& cfg ) {
    // Verify if already initialized
    if ( this->initialized_ ) {
        return errCode::ERR_ALREADY_INITIALIZED;
    }

    this->cfg_ = cfg;

    // Create Rx Queue 
    this->rx_queue_ =
        xQueueCreate(
            this->cfg_.rx_queue_len,
            sizeof(SerialCommProtoPacket)
        );

    if ( this->rx_queue_ == nullptr ) {
        ESP_LOGE( TAG, "Failed to create RX queue");
        return errCode::ERR_NO_MEMORY;
    }

    // Create callback mutex
    this->callback_mutex_ = xSemaphoreCreateMutex();
    if ( this->callback_mutex_ == nullptr ) {
        cleanup_resources();
        ESP_LOGE( TAG, "Failed to create callback mutex" );
        return errCode::ERR_NO_MEMORY;
    }

    this->initialized_ = true;
    ESP_LOGI( TAG, "Dispatcher initialized" );
    return errCode::OK;
}

errCode SerialCommDispatcher::start() {
    // VERIFY INITIALIZATION
    if ( !this->initialized_ ) {
        return errCode::ERR_NOT_INITIALIZED;
    }

    // VERIFY RUNNING STATE
    if ( this->running_ ) {
        return errCode::ERR_INVALID_STATE;
    }

    // CREATE DISPATCHER TASK
    BaseType_t ret =
        xTaskCreate(
            dispatcher_task_entry,
            this->cfg_.task_name,
            this->cfg_.task_stack_size,
            this,
            this->cfg_.task_priority,
            &this->dispatcher_task_
        );
    if ( ret != pdPASS ) {
        ESP_LOGE( TAG, "Failed to create dispatcher task");
        return errCode::ERR_FAIL;
    }

    this->running_ = true;
    ESP_LOGI( TAG, "Dispatcher started" );
    return errCode::OK;
}


errCode SerialCommDispatcher::stop() {
    if ( !this->running_ ) {
        return errCode::ERR_INVALID_STATE;
    }

    // DELETE DISPATCHER TASK
    if ( this->dispatcher_task_ != nullptr ) {
        vTaskDelete( this->dispatcher_task_ );
        this->dispatcher_task_ = nullptr;
    }

    this->running_ = false;
    ESP_LOGI( TAG, "Dispatcher stopped" );
    return errCode::OK;
}


errCode SerialCommDispatcher::deinit() {
    if ( this->running_ ) {
        this->stop();
    }
    cleanup_resources();
    // Limpa os Callbacks 
    memset( this->callbacks_, 0, sizeof(this->callbacks_));
    this->initialized_ = false;
    ESP_LOGI( TAG, "Dispatcher deinitialized" );
    return errCode::OK;
}


errCode SerialCommDispatcher::enqueue(
    const SerialCommProtoPacket& packet
) {
    if ( !this->running_ ) {
        ESP_LOGE( TAG, "Failed to enqueue packet: Dispatcher not running" );
        return errCode::ERR_INVALID_STATE;
    }
    if ( !this->initialized_ ) {
        ESP_LOGE( TAG, "Failed to enqueue packet: Dispatcher not initialized" );
        return errCode::ERR_NOT_INITIALIZED;
    }

    // Enqueue packet
    if ( xQueueSend( this->rx_queue_, &packet, 0 ) != pdTRUE ) {
        if ( this->rx_dropped_packets_ < UINT32_MAX ) {
            this->rx_dropped_packets_++;
        } else {
            this->rx_dropped_packets_ = 0;
        }
        ESP_LOGW( 
            TAG, 
            "RX queue full. Dropped command: 0x%02X",
            packet.header.command 
        );
        return errCode::ERR_QUEUE_FULL;
    }

    // Update statistics
    if( this->rx_packets_ < UINT32_MAX ) {
        this->rx_packets_++;
    } else {
        this->rx_packets_ = 0;
    }
    return errCode::OK;
}


errCode SerialCommDispatcher::register_callback(
    SerialCommProtoCommand command,
    packet_callback_t callback,
    void* ctx
) {
    // Validate pointers
    if ( callback == nullptr ) {
        return errCode::ERR_NULL_POINTER;
    }
    // Validate callback mutex
    if ( this->callback_mutex_ == nullptr ) {
        return errCode::ERR_INVALID_STATE;
    }

    uint8_t cmd = static_cast<uint8_t>(command );
    xSemaphoreTake( this->callback_mutex_, portMAX_DELAY );
    this->callbacks_[cmd].callback = callback;
    this->callbacks_[cmd].ctx = ctx;
    xSemaphoreGive( this->callback_mutex_ );
    return errCode::OK;
}


errCode SerialCommDispatcher::unregister_callback(
    SerialCommProtoCommand command
) {
    uint8_t cmd = static_cast<uint8_t>( command );
    xSemaphoreTake( this->callback_mutex_, portMAX_DELAY );
    this->callbacks_[cmd].callback = nullptr;
    this->callbacks_[cmd].ctx = nullptr;
    xSemaphoreGive( this->callback_mutex_ );
    return errCode::OK;
}


void SerialCommDispatcher::dispatch_packet(
    const SerialCommProtoPacket& packet
) {
    uint8_t cmd = static_cast<uint8_t>( packet.header.command );
    xSemaphoreTake( this->callback_mutex_, portMAX_DELAY );
    CallbackEntry entry = this->callbacks_[cmd];
    xSemaphoreGive( this->callback_mutex_ );
    if ( entry.callback != nullptr ) {
        entry.callback( entry.ctx, packet );
    }
}


void SerialCommDispatcher::dispatcher_task_entry( void* args ) {
    auto* self = static_cast<SerialCommDispatcher*>(args);
    self->dispatcher_task();
}


void SerialCommDispatcher::dispatcher_task() {
    SerialCommProtoPacket packet;
    while ( true ) {
        if ( xQueueReceive( this->rx_queue_, &packet, portMAX_DELAY ) == pdTRUE ) {
            dispatch_packet( packet );
        }
    }
}


void SerialCommDispatcher::cleanup_resources(){
    // DELETE RX QUEUE
    if ( this->rx_queue_ != nullptr ) {
        vQueueDelete( this->rx_queue_ );
        this->rx_queue_ = nullptr;
    }
    // DELETE CALLBACK MUTEX
    if ( this->callback_mutex_ != nullptr ) {
        vSemaphoreDelete( this->callback_mutex_ );
        this->callback_mutex_ = nullptr;
    }
}


SerialCommDispatcher::~SerialCommDispatcher() {
    this->deinit();
}