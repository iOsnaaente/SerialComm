/**
 * @file    serial_comm_parser.h
 * @brief   Core Serial communication middleware class
 * @details Manages the serial communication protocol, including packet
 *          parsing, validation, and dispatching to the appropriate handlers
 * @details This class is the Orchestrator of the Serial Communication, its
 *          have the following responsibilities:
 *              - Transport orchestration
 *              - RX/TX management
 *              - Parser integration
 *              - Packet dispatching
 *              - Inter-byte timeout control
 *              - Command callbacks
 *              - Middleware lifecycle
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Created on 22 of May, 2026 
 */

#pragma once

#include "serial_comm_dispatcher.h"
#include "serial_comm_transport.h"
#include "serial_comm_watchdog.h"
#include "serial_comm_protocol.h"
#include "serial_comm_parser.h"
#include "serial_comm_utils.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_timer.h"
#include "esp_log.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>


using namespace SerialCommResult_Codes;

class SerialComm {
    public:
        /* Packet callback type */
        using packet_callback_t =
            void (*)( void* ctx, const SerialCommProtoPacket& packet );

    public:
        /**
         * @brief   Serial communication runtime configuration
         * @details Defines runtime parameters for the middleware, to use or 
         *          not the inter-byte timeout watchdog, and the timeout 
         *          duration in microseconds. 
         *          The inter-byte timeout is used to detect the end
         *          of a packet when the protocol does not have a fixed length 
         *          or a specific end-of-packet marker. 
         *          By measuring the time between received bytes, the 
         *          middleware can determine when a packet has likely ended, 
         *          allowing it to process the received data even if
         *          the packet is incomplete or does not have a clear 
         *          delimiter.
         */
        struct Config {
            /* Inter-byte timeout in microseconds (us) */
            uint32_t inter_byte_timeout_us = 1000;
            /* Enable timeout watchdog */
            bool enable_inter_byte_timeout = true;
        };

    private:
        /* Transport layer */
        ISerialCommTransport* transport_;
        
        /* Stream parser */
        SerialCommParser parser_;
        
        /* Dispatcher packet */
        SerialCommDispatcher dispatcher_;

        /* Inter-byte timeout watchdog */
        SerialCommWatchdogTimer* interbyte_watchdog_;

        /* Runtime configuration */
        Config cfg_;

        /* TX mutex */
        SemaphoreHandle_t tx_mutex_;
        /* Parser protection Mutex */
        SemaphoreHandle_t parser_mutex_ = nullptr;

        /* Initialization status */
        bool initialized_;
        /* Running status */
        bool running_;


    public:
        /**
         * @brief Constructor
         * @param transport Transport implementation
         */
        explicit SerialComm( ISerialCommTransport* transport ) :
            transport_(transport),
            interbyte_watchdog_(nullptr),
            tx_mutex_(nullptr),
            initialized_(false),
            running_(false)
        { }

        /**
         * @brief Disable copy constructor
         */
        SerialComm( const SerialComm& ) = delete;

        /**
         * @brief Disable copy assignment
         */
        SerialComm& operator=(const SerialComm&) = delete;

        /**
         * @brief Destructor
         */
        virtual ~SerialComm();

        /**
         * @brief Initialize middleware
         * @param cfg Runtime configuration
         * @return Middleware result
         */
        errCode init( const Config& cfg );

        /**
         * @brief Start middleware
         * @return Middleware result
         */
        errCode start();

        /**
         * @brief Stop middleware
         * @return Middleware result
         */
        errCode stop();

        /**
         * @brief Deinitialize middleware
         * @return Middleware result
         */
        errCode deinit();


    // CORE FUNCTIONALITIES
    public:

        /**
         * @brief   Send packet through transport
         * @param   packet Packet to send
         * @return  Middleware result
         */
        errCode send( const SerialCommProtoPacket& packet );

        /**
         * @brief   Register command callback
         * @param   command Protocol command
         * @param   callback Callback function
         * @param   ctx User context
         * @return  Middleware result
         */
        errCode register_callback(
            SerialCommProtoCommand command,
            packet_callback_t callback,
            void* ctx
        );

        /**
         * @brief   Unregister command callback
         * @param   command Protocol command
         * @return  Middleware result
         */
        errCode unregister_callback( 
            SerialCommProtoCommand command 
        );

    private:
        /**
         * @brief   Static RX callback from transport
         */
        static void transport_rx_callback(
            void* ctx,
            const uint8_t* data,
            size_t len
        );

        /**
         * @brief   Process incoming RX data
         */
        void process_rx_data( const uint8_t* data, size_t len );


    private: 
        /**
         * @brief   Configure inter-byte watchdog
         * @details Creates and configures the watchdog used to reset
         *          the parser after RX timeout.
         * @return  Middleware result
         */
        errCode setup_interbyte_watchdog();

        /**
         * @brief   Inter-byte timeout callback
         * @details Called when RX stream timeout occurs.
         */
        void on_interbyte_timeout();

        
    private:
        /**
         * @brief Cleanup allocated resources
         */
        void cleanup_resources();

};