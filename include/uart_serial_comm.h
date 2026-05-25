/**
 * @file    uart_serial_comm.h
 * @brief   ESP32 UART transport implementation for serial_comm middleware
 * @note    It was tested and is compatible with the follow SoCs: 
 *              - ESP32-D0WD-V3
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Created on 22 of May, 2026
 */


#pragma once

#include "serial_comm_transport.h"
#include "serial_comm_config.h"
#include "serial_comm_utils.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_err.h"



class UARTTransport final : public ISerialCommTransport {
    public:
            /**
             * @brief ESP32 UART hardware configuration
             */
            struct HardwareConfig {
                int uart_port   = SERIAL_COMM_UART_PORT;
                int tx_pin      = SERIAL_COMM_UART_TX_PIN;
                int rx_pin      = SERIAL_COMM_UART_RX_PIN;
                int rts_pin     = SERIAL_COMM_UART_RTS_PIN;
                int cts_pin     = SERIAL_COMM_UART_CTS_PIN;
                int de_pin      = SERIAL_COMM_UART_DE_PIN; 
            };


    private:
        HardwareConfig hw_cfg_;

        SemaphoreHandle_t uart_mutex_;
        SemaphoreHandle_t state_mutex_;
        QueueHandle_t uart_queue_;
        TaskHandle_t uart_task_;

        tx_done_callback_t tx_done_callback_;
        event_callback_t event_callback_;
        rx_callback_t rx_callback_;

        void* event_ctx_;
        void* rx_ctx_;
        void* tx_ctx_;

        Config cfg_;
        State state_;


    private:
        static void uart_event_task(
            void* args
        );


    public:
        /**
         * @brief   Constructor for UARTTransport
         * @param   hw_cfg Hardware configuration for the UART transport
         */
        explicit UARTTransport(
            const HardwareConfig& hw_cfg
        );

        /* Deleted copy and move constructors */
        UARTTransport( UARTTransport&& ) = delete;
        UARTTransport& operator=( UARTTransport&& ) = delete;
        UARTTransport( const UARTTransport& ) = delete;
        UARTTransport& operator=( const UARTTransport& ) = delete;

        /**
         * @brief   Destructor for UARTTransport
         */
        ~UARTTransport() override;

    public:

        SerialCommResult::errCode init( const Config& cfg ) override;
        SerialCommResult::errCode start() override;
        SerialCommResult::errCode stop() override;
        SerialCommResult::errCode deinit() override;

    public:
        int write(
            const uint8_t* data,
            size_t len
        ) override;

        SerialCommResult::errCode write_async(
            const uint8_t* data,
            size_t len
        ) override;

        int read(
            uint8_t* data,
            size_t len,
            uint32_t timeout_ms
        ) override;

        int available() const override;

        SerialCommResult::errCode flush() override;

    public:
        SerialCommResult::errCode set_rx_callback(
            rx_callback_t cb,
            void* ctx
        ) override;

        SerialCommResult::errCode set_tx_done_callback(
            tx_done_callback_t cb,
            void* ctx
        ) override;

        SerialCommResult::errCode set_event_callback(
            event_callback_t cb,
            void* ctx
        ) override;

    public:
        bool running() const override;
        State state() const override;

    public:
        size_t rx_buffer_size() const override;
        size_t tx_buffer_size() const override;

};