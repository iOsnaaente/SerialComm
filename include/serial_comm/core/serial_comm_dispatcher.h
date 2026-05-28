/**
 * @file    serial_comm_dispatcher.h
 * @brief   Dispatcher class for handling incoming packets and dispatching 
 *          them to the appropriate handlers in the serial communication 
 *          middleware.
 * @details The Dispatcher class is responsible for:
 *              - RX packet parsing and validation
 *              - Dispatching packets to the correct handler based on command
 *              - Packet routing 
 *              - Command callback registry 
 *              - Async packet dispatch 
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Created on 22 of May, 2026 
 */


/**
 * TODO:    Change the xCreateTask to xTaskCreatePinnedToCore to avoid task 
 *          migration and potential timing issues in the dispatcher task.
 *          This will ensure that the dispatcher task runs on a specific 
 *          core, which can help with timing and performance, especially 
 *          in a real-time application where the dispatcher needs to 
 *          process packets with low latency.
 */

#pragma once 

#include "serial_comm/serial_comm_config.h"
#include "serial_comm/core/serial_comm_protocol.h"
#include "serial_comm/core/serial_comm_utils.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"


/* To use the errCode and err_to_str easily */
using namespace SerialCommResult_Codes;


class SerialCommDispatcher {
    public:
        /* Packet callback type */
        using packet_callback_t =
            void (*)( void* ctx, const SerialCommProtoPacket& packet );

            
    public:
        /**
         * @brief Dispatcher configuration
         */
        struct Config {
            /* RX packet queue size */
            uint32_t rx_queue_len =
                SerialCommConfig::DISPATCHER_QUEUE_SIZE;
            /* Dispatcher task stack size */
            uint32_t task_stack_size =
                SerialCommConfig::DISPATCHER_TASK_STACK_SIZE;
            /* Dispatcher task priority */
            UBaseType_t task_priority =
                SerialCommConfig::DISPATCHER_TASK_PRIORITY;
            /* Dispatcher task name */
            const char* task_name = "SerialCommDispatcher";
        };

    private:
        /* Callback entry */
        struct CallbackEntry {
            packet_callback_t callback = nullptr;
            void* ctx = nullptr;
        };

    private:
        /* Dispatcher configuration */
        Config cfg_;

        /* RX packet queue */
        QueueHandle_t rx_queue_ = nullptr;

        /* Dispatcher task handle */
        TaskHandle_t dispatcher_task_ = nullptr;

        /* Callback registry protection mutex */
        SemaphoreHandle_t callback_mutex_ = nullptr;

        /* Callback registry */
        CallbackEntry callbacks_[256];

        /* Runtime state */
        bool initialized_ = false;
        bool running_ = false;

        /* Statistics */
        uint32_t rx_packets_ = 0;
        uint32_t rx_dropped_packets_ = 0;


        /**
         * @brief   Dispatch packet to callback
         * @param   packet Packet to dispatch
         */
        void dispatch_packet( const SerialCommProtoPacket& packet );

        /**
         * @brief Dispatcher task entry
         * @param args Task arguments (pointer to dispatcher instance)
         */
        static void dispatcher_task_entry( void* args );

        /**
         * @brief Dispatcher task loop
         */
        void dispatcher_task();

        /**
         * @brief Cleanup resources (queues, mutexes, etc.)
         */
        void cleanup_resources();


    public:
        /**
         * @brief Constructor
         */
        SerialCommDispatcher();

        /* Disable Copy */
        SerialCommDispatcher( const SerialCommDispatcher& ) = delete;
        SerialCommDispatcher& operator = (
            const SerialCommDispatcher&
        ) = delete;

        /* Disable Move */
        SerialCommDispatcher( SerialCommDispatcher&& ) = delete;
        SerialCommDispatcher& operator=(
            SerialCommDispatcher&&
        ) = delete;

        /**
         * @brief Destructor
         */
        virtual ~SerialCommDispatcher();


    public:

        /**
         * @brief   Initialize dispatcher
         * @param   cfg Dispatcher configuration
         * @return  Result code
         */
        SerialCommResult_Codes::errCode init( const Config& cfg );

        /**
         * @brief   Start dispatcher task
         * @return  Result code
         */
        SerialCommResult_Codes::errCode start();

        /**
         * @brief   Stop dispatcher task
         * @return  Result code
         */
        SerialCommResult_Codes::errCode stop();

        /**
         * @brief   Deinitialize dispatcher
         * @return  Result code
         */
        SerialCommResult_Codes::errCode deinit();

    public:

        /**
         * @brief   Enqueue received packet
         * @param   packet Packet to enqueue
         * @return  Result code
         */
        SerialCommResult_Codes::errCode enqueue(
            const SerialCommProtoPacket& packet
        );

    public:

        /**
         * @brief   Register command callback
         * @param   command Protocol command
         * @param   callback Callback function
         * @param   ctx User context
         * @return  Result code
         */
        SerialCommResult_Codes::errCode register_callback(
            SerialCommCommand command,
            packet_callback_t callback,
            void* ctx
        );

        /**
         * @brief   Unregister command callback
         * @param   command Protocol command
         * @return  Result code
         */
        SerialCommResult_Codes::errCode unregister_callback(
            SerialCommCommand command
        );

    public:

        /**
         * @brief Get received packet counter
         * @return Number of received packets
         */
        uint32_t rx_packets() const {
            return this->rx_packets_;
        }

        /**
         * @brief Get dropped packet counter
         * @return Number of dropped packets
         */
        uint32_t rx_dropped_packets() const {
            return this->rx_dropped_packets_;
        }

        /**
         * @brief Check if dispatcher is initialized
         * @return True if initialized, false otherwise
         */
        bool initialized() const {
            return this->initialized_;
        }

        /**
         * @brief Check if dispatcher is running
         * @return True if running, false otherwise
         */
        bool running() const {
            return this->running_;
        }
};
