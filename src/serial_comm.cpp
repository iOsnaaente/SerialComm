/**
 * @file    serial_comm.cpp
 * @brief   Core serial communication middleware implementation
 */

#include "serial_comm.h"


/* To use the errCode and err_to_str easily */
using namespace SerialCommResult_Codes;


static const char* TAG = "SERIAL_COMM";


errCode SerialComm::init( const Config &cfg ) {
    // Verify transport is set correctly
    if ( this->transport_ == nullptr ) {
        ESP_LOGE( TAG, "Transport is null" );
        return errCode::ERR_NULL_POINTER;
    }
    // Verify if already initialized
    if ( this->initialized_ ) {
        ESP_LOGE( TAG, "Already initialized" );
        return errCode::ERR_ALREADY_INITIALIZED;
    }
    // Get transport configuration
    this->cfg_ = cfg;

    // Tx Mutex
    this->tx_mutex_ = xSemaphoreCreateMutex();
    if ( this->tx_mutex_ == nullptr ) {
        ESP_LOGE( TAG, "Failed to create TX mutex" );
        return errCode::ERR_NO_MEMORY;
    }

    // Parser Mutex
    this->parser_mutex_ = xSemaphoreCreateMutex();
    if ( this->parser_mutex_ == nullptr ) {
        ESP_LOGE( TAG, "Failed to create parser mutex" );
        vSemaphoreDelete( this->tx_mutex_ );
        this->tx_mutex_ = nullptr;
        return errCode::ERR_NO_MEMORY;
    }

    // Dispatcher initialization
    SerialCommDispatcher::Config dispatcher_cfg = {};
    dispatcher_cfg.task_name = "SerialCommDispatcher";
    dispatcher_cfg.rx_queue_len = 16;
    dispatcher_cfg.task_stack_size = 4096;
    dispatcher_cfg.task_priority = 5;
    errCode err = this->dispatcher_.init( dispatcher_cfg );
    if ( err != errCode::OK ) {
        ESP_LOGE( TAG, "Failed to initialize dispatcher: %s", err_to_str(err) );
        this->cleanup_resources();
        return err;
    }

    // Inter-Byte Watchdog
    if ( this->cfg_.enable_inter_byte_timeout ) {
        err = this->setup_interbyte_watchdog();
        if ( err != errCode::OK ) {
            ESP_LOGE( TAG, "Failed to setup inter-byte timeout watchdog: %s", err_to_str(err) );
            this->cleanup_resources();
            return err;
        }
    }

    // Transport Rx Callback
    err = this->transport_->set_rx_callback(
        transport_rx_callback, 
        this
    );
    if ( err != errCode::OK ) {
        ESP_LOGE( TAG, "Failed to set transport RX callback: %s", err_to_str(err) );
        this->cleanup_resources();
        return err;
    }

    this->initialized_ = true;
    ESP_LOGI( TAG, "SerialComm initialized" );
    return errCode::OK;
}


errCode SerialComm::start() {
    // Verify if initialized
    if (!this->initialized_) {
        ESP_LOGE( TAG, "Cannot start: Not initialized" );
        return errCode::ERR_NOT_INITIALIZED;
    }
    // Verify if already running
    if ( this->running_ ) {
        ESP_LOGE( TAG, "Cannot start: Already running" );
        return errCode::ERR_INVALID_STATE;
    }
    // Start Dispatcher
    errCode err = this->dispatcher_.start();
    if ( err != errCode::OK ) {
        ESP_LOGE( TAG, "Failed to start dispatcher: %s", err_to_str(err) );
        return err;
    }
    // Start transport
    errCode err = this->transport_->start();
    if (err != errCode::OK) {
        ESP_LOGE( TAG, "Failed to start transport: %s", err_to_str(err) );
        this->dispatcher_.stop();
        return err;
    }
    // Start Watchdog if enabled
    if ( this->interbyte_watchdog_ != nullptr ) {
        this->interbyte_watchdog_->start();
    } else {
        ESP_LOGW( TAG, "Inter-byte timeout watchdog disabled" );
    }
    this->running_ = true;
    ESP_LOGI( TAG, "SerialComm started");
    return errCode::OK;
}


errCode SerialComm::stop() {
    if (!this->running_) {
        ESP_LOGE( TAG, "Cannot stop: Not running" );
        return errCode::ERR_INVALID_STATE;
    }
    // Stop Watchdog if enabled
    if ( this->interbyte_watchdog_ != nullptr ) {
        this->interbyte_watchdog_->stop();
    }
    this->transport_->stop();
    this->dispatcher_.stop();
    this->running_ = false;
    ESP_LOGI( TAG, "SerialComm stopped" );
    return errCode::OK;
}


errCode SerialComm::deinit() {
    if ( this->running_ ) {
        this->stop();
    }
    this->dispatcher_.deinit();
    this->cleanup_resources();
    this->initialized_ = false;
    ESP_LOGI( TAG, "SerialComm deinitialized" );
    return errCode::OK;
}


errCode SerialComm::send( const SerialCommProtoPacket& packet ) {
    if ( !this->running_ ){
        ESP_LOGE( TAG, "Cannot send: Not running" );
        return errCode::ERR_INVALID_STATE;
    }

    if ( xSemaphoreTake( this->tx_mutex_, pdMS_TO_TICKS(100) ) != pdTRUE ) {
        ESP_LOGE( TAG, "Failed to take TX mutex" );
        return errCode::ERR_TIMEOUT;
    }

    uint8_t tx_buffer[ SERIAL_COMM_MAX_PACKET_SIZE_V1 ];
    size_t packet_len =
        SerialCommProtocol::encode(
            packet,
            tx_buffer,
            sizeof(tx_buffer)
        );
    if (packet_len == 0) {
        ESP_LOGE( TAG, "Failed to encode packet" );
        xSemaphoreGive(this->tx_mutex_);
        return errCode::ERR_FAIL;
    }

    int written = this->transport_->write( tx_buffer, packet_len );
    xSemaphoreGive(this->tx_mutex_);

    if (written != (int)packet_len) {
        ESP_LOGE( TAG,"Transport write failed [%d/%d]", written, packet_len);
        return errCode::ERR_IO;
    }
    return errCode::OK;
}


errCode SerialComm::setup_interbyte_watchdog( ){
    this->interbyte_watchdog_ = new SerialCommWatchdogTimer(
        "SerialRxTimeout",
        this->cfg_.inter_byte_timeout_us,
        [this](){
            this->on_interbyte_timeout();
        }
    );
    if ( this->interbyte_watchdog_ == nullptr ) {
        return errCode::ERR_NO_MEMORY;
    }
    return errCode::OK;
}


void SerialComm::on_interbyte_timeout() {
    if ( xSemaphoreTake( this->parser_mutex_, portMAX_DELAY ) == pdTRUE ) {
        this->parser_.reset();
        xSemaphoreGive( this->parser_mutex_ );
        ESP_LOGW( TAG, "Parser timeout reset" );
    }
}


errCode SerialComm::register_callback(
    SerialCommProtoCommand command,
    packet_callback_t callback,
    void* ctx
) {
    return this->dispatcher_.register_callback( 
        command, 
        callback, 
        ctx 
    );
}


errCode SerialComm::unregister_callback( SerialCommProtoCommand command ) {
    return this->dispatcher_.unregister_callback( command );
}


void SerialComm::transport_rx_callback( 
    void* ctx, 
    const uint8_t* data, 
    size_t len
) {
    if (ctx == nullptr){
        return;
    }
    auto* self = static_cast<SerialComm*>(ctx);
    self->process_rx_data( data, len );
}


void SerialComm::process_rx_data( const uint8_t* data, size_t len ) {
    if (data == nullptr){
        ESP_LOGE( TAG, "Received null data pointer" );
        return;
    }
    // Kick inter-byte timeout watchdog
    if ( this->interbyte_watchdog_ != nullptr ) {
        this->interbyte_watchdog_->kick();
    }
    // Parse Stream 
    SerialCommProtoPacket packet;
    xSemaphoreTake( this->parser_mutex_, portMAX_DELAY );
    bool valid_packet = this->parser_.parse_buffer( data, len, packet );
    xSemaphoreGive( this->parser_mutex_ );

    // Dispatch packet if valid
    if ( valid_packet ) {
        errCode err = this->dispatcher_.enqueue( packet );
        if ( err != errCode::OK ) {
            ESP_LOGE( TAG, "Failed to enqueue packet" );
        }
    }
}


void SerialComm::cleanup_resources(){
    // Watchdog cleanup
    if ( this->interbyte_watchdog_ != nullptr ) {
        this->interbyte_watchdog_->stop();
        delete this->interbyte_watchdog_;
        this->interbyte_watchdog_ = nullptr;
    }
    // Tx Mutex cleanup
    if ( this->tx_mutex_ != nullptr ) {
        vSemaphoreDelete( this->tx_mutex_ );
        this->tx_mutex_ = nullptr;
    }
    // Parser Mutex cleanup
    if ( this->parser_mutex_ != nullptr ) {
        vSemaphoreDelete( this->parser_mutex_ );
        this->parser_mutex_ = nullptr;
    }
}

SerialComm::~SerialComm() {
    deinit();
}
