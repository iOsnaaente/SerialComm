/**
 * @file uart_serial_comm.cpp
 * @brief ESP32 UART transport implementation
 */

#include "serial_comm/transport/uart_serial_comm.h"


/* To use the errCode and err_to_str easily */
using namespace SerialCommResult_Codes;


static const char* TAG = "UART_TRANSPORT";


UARTTransport::UARTTransport( const HardwareConfig& hw_cfg ) :   
    hw_cfg_(hw_cfg),
    uart_mutex_(nullptr),
    state_mutex_(nullptr),
    uart_queue_(nullptr),
    uart_task_(nullptr),
    tx_done_callback_(nullptr),
    event_callback_(nullptr),
    rx_callback_(nullptr),
    event_ctx_(nullptr),
    rx_ctx_(nullptr),
    tx_ctx_(nullptr),
    cfg_{},
    state_(State::UNINITIALIZED)
{ }


errCode UARTTransport::init( const Config& cfg ) {
    if (state_ != State::UNINITIALIZED) {
        return errCode::ERR_ALREADY_INITIALIZED;
    }
    cfg_ = cfg;

    uart_config_t uart_cfg = {};
    uart_cfg.baud_rate  = (int)cfg.baudrate;
    uart_cfg.data_bits  = UART_DATA_8_BITS;
    uart_cfg.parity     = UART_PARITY_DISABLE;
    uart_cfg.stop_bits  = UART_STOP_BITS_1;
    uart_cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    uart_cfg.source_clk = UART_SCLK_DEFAULT;

    esp_err_t err;
    err = uart_driver_install(
        (uart_port_t)hw_cfg_.uart_port,
        cfg.rx_buffer_size,
        cfg.tx_buffer_size,
        SerialCommConfig::UART_EVENT_QUEUE_SIZE,
        &uart_queue_,
        0
    );

    if (err != ESP_OK) {
        ESP_LOGE( TAG, "uart_driver_install failed" );
        return errCode::ERR_FAIL;
    }

    err = uart_param_config(
        (uart_port_t)hw_cfg_.uart_port,
        &uart_cfg
    );

    if (err != ESP_OK) {
        return errCode::ERR_FAIL;
    }

    err = uart_set_pin(
        (uart_port_t)hw_cfg_.uart_port,
        hw_cfg_.tx_pin,
        hw_cfg_.rx_pin,
        hw_cfg_.rts_pin,
        hw_cfg_.cts_pin
    );

    if (err != ESP_OK) {
        return errCode::ERR_FAIL;
    }

    if ( cfg.half_duplex && hw_cfg_.de_pin >= 0 ) {
        gpio_config_t io_cfg = {};
        io_cfg.pin_bit_mask = (1ULL << hw_cfg_.de_pin);
        io_cfg.mode = GPIO_MODE_OUTPUT;
        gpio_config(&io_cfg);
        /* start in RX mode */
        gpio_set_level( (gpio_num_t)hw_cfg_.de_pin, 0 );
    }

    state_ = State::INITIALIZED;
    ESP_LOGI( TAG, "UART initialized" );
    return errCode::OK;
}


errCode UARTTransport::start() {
    if (
        state_ != State::INITIALIZED &&
        state_ != State::STOPPED
    ) {
        return errCode::ERR_INVALID_STATE;
    }

    // Check if the Task have Core afinity enabled
    BaseType_t task_result;
    if ( SerialCommConfig::UART_USE_TASK_CORE_AFINITY ){
        task_result = xTaskCreatePinnedToCore(
            uart_event_task,
            "uart_event_task",
            SerialCommConfig::UART_TASK_STACK_SIZE,
            this,
            SerialCommConfig::UART_TASK_PRIORITY,
            &uart_task_,
            SerialCommConfig::UART_TASK_CORE
        );
    } else {
        task_result = xTaskCreate(
            uart_event_task,
            "uart_event_task",
            SerialCommConfig::UART_TASK_STACK_SIZE,
            this,
            SerialCommConfig::UART_TASK_PRIORITY,
            &uart_task_
        );
    }
    if ( task_result != pdPASS ) {
        ESP_LOGE( TAG, "Failed to create UART event task" );
        return errCode::ERR_FAIL;
    }

    state_ = State::RUNNING;
    ESP_LOGI( TAG, "UART started" );
    return errCode::OK;
}


errCode UARTTransport::stop() {
    if (uart_task_) {
        vTaskDelete(uart_task_);
        uart_task_ = nullptr;
    }
    state_ = State::STOPPED;
    ESP_LOGI( TAG, "UART stopped" );
    return errCode::OK;
}

errCode UARTTransport::deinit() {
    this->stop();
    uart_driver_delete( (uart_port_t)hw_cfg_.uart_port );
    state_ = State::UNINITIALIZED;
    ESP_LOGI( TAG, "UART deinitialized" );
    return errCode::OK;
}


int UARTTransport::write( const uint8_t* data, size_t len ) {
    if ( state_ != State::RUNNING ) {
        return -1;
    }

    if (cfg_.half_duplex && hw_cfg_.de_pin >= 0) {
        gpio_set_level((gpio_num_t)hw_cfg_.de_pin, 1);
    }

    int written =
        uart_write_bytes(
            (uart_port_t)hw_cfg_.uart_port,
            data,
            len
        );

    uart_wait_tx_done(
        (uart_port_t)hw_cfg_.uart_port,
        pdMS_TO_TICKS(cfg_.timeout_ms)
    );

    if (cfg_.half_duplex && hw_cfg_.de_pin >= 0) {
        gpio_set_level((gpio_num_t)hw_cfg_.de_pin, 0);
    }
    return written;
}


errCode UARTTransport::write_async( const uint8_t* data, size_t len ) {
    if ( state_ != State::RUNNING ) {
        return errCode::ERR_INVALID_STATE;
    }
    
    if (cfg_.half_duplex && hw_cfg_.de_pin >= 0) {
        gpio_set_level((gpio_num_t)hw_cfg_.de_pin, 1);
    }

    int written =
        uart_write_bytes(
            (uart_port_t)hw_cfg_.uart_port,
            data,
            len
        );

    if (written < 0) {
        return errCode::ERR_IO;
    }
    return errCode::OK;
}


int UARTTransport::read( uint8_t* data, size_t len, uint32_t timeout_ms ) {
    return uart_read_bytes(
        (uart_port_t)hw_cfg_.uart_port,
        data,
        len,
        pdMS_TO_TICKS(timeout_ms)
    );
}

int UARTTransport::available() const {
    size_t available = 0;
    uart_get_buffered_data_len(
        (uart_port_t)hw_cfg_.uart_port,
        &available
    );
    return (int)available;
}

errCode UARTTransport::flush() {
    esp_err_t err =
        uart_flush(
            (uart_port_t)hw_cfg_.uart_port
        );
    if (err != ESP_OK) {
        return errCode::ERR_FAIL;
    }
    return errCode::OK;
}


errCode UARTTransport::set_rx_callback( 
    rx_callback_t cb, 
    void* ctx 
) {
    rx_callback_ = cb;
    rx_ctx_ = ctx;
    return errCode::OK;
}


errCode UARTTransport::set_tx_done_callback(
    tx_done_callback_t cb,
    void* ctx
) {
    tx_done_callback_ = cb;
    tx_ctx_ = ctx;
    return errCode::OK;
}


errCode UARTTransport::set_event_callback(
    event_callback_t cb,
    void* ctx
){
    event_callback_ = cb;
    event_ctx_ = ctx;
    return errCode::OK;
}


bool UARTTransport::running() const {
    return state_ == State::RUNNING;
}

ISerialCommTransport::State UARTTransport::state() const {
    return state_;
}


size_t UARTTransport::rx_buffer_size() const {
    return cfg_.rx_buffer_size;
}


size_t UARTTransport::tx_buffer_size() const {
    return cfg_.tx_buffer_size;
}


void UARTTransport::uart_event_task( void* args ) {
    auto* self =
        static_cast<UARTTransport*>(args);
    uart_event_t event;
    uint8_t rx_buffer[256];
    while (true) {
        if ( xQueueReceive( self->uart_queue_, &event, portMAX_DELAY ) ){
            switch (event.type) {
                case UART_DATA: {
                    int len =
                        uart_read_bytes(
                            (uart_port_t)
                                self->hw_cfg_.uart_port,
                            rx_buffer,
                            event.size,
                            0
                        );
                    if ( len > 0 && self->rx_callback_ ) {
                        self->rx_callback_(
                            self->rx_ctx_,
                            rx_buffer,
                            len
                        );
                    }
                    break;
                }

                case UART_FIFO_OVF: {
                    uart_flush_input(
                        (uart_port_t)
                            self->hw_cfg_.uart_port
                    );
                    xQueueReset( self->uart_queue_ );
                    if ( self->event_callback_ ) {
                        self->event_callback_(
                            self->event_ctx_,
                            Event::RX_OVERFLOW
                        );
                    }
                    break;
                }

                case UART_PARITY_ERR: {
                    if ( self->event_callback_ ) {
                        self->event_callback_(
                            self->event_ctx_,
                            Event::PARITY_ERROR
                        );
                    }
                    break;
                }

                case UART_FRAME_ERR: {
                    if ( self->event_callback_ ) {
                        self->event_callback_(
                            self->event_ctx_,
                            Event::FRAME_ERROR
                        );
                    }
                    break;
                }

                default:
                    break;
            }
        }
    }
}


UARTTransport::~UARTTransport() {
    deinit();
}