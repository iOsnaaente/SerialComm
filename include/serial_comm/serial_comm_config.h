/**
 * @file    serial_comm_config.hpp
 * @brief   Serial communication middleware configuration from menuconfig
 * @note    Please dont modify this file directly, as it is generated from 
 *          the Kconfig system. 
 * @note    If you want to change any configuration, please do it through 
 *          the menuconfig interface. To do that, run: 
 *              `idf.py menuconfig` 
 *          and navigate to the Serial Communication
 */

#pragma once

#include "sdkconfig.h"

#include <stdint.h>
#include <stddef.h>


/**
 * @brief   Namespace containing all configuration parameters for the 
 *          serial communication middleware, populated from Kconfig 
 *          menuconfig options.
 * 
 * @details This namespace serves as a centralized location for all 
 *          configuration parameters, making it easy to access and maintain. 
 * 
 * @example To use the configuration parameters, do:
 *              using namespace SerialCommConfig;
 *              int uart_port = UART_PORT;
 * 
 * @example To use the configuration parameters without qualifying the 
 *          namespace, do:
 *             SerialCommConfig::UART_PORT;
 *             
 */
namespace SerialCommConfig {

    // TRANSPORT
    constexpr int UART_PORT = 
        CONFIG_SERIAL_COMM_UART_PORT;
        
    constexpr uint32_t UART_BAUDRATE = 
        CONFIG_SERIAL_COMM_UART_BAUDRATE;
        
    constexpr int UART_TX_PIN = 
        CONFIG_SERIAL_COMM_UART_TX_PIN;
        
    constexpr int UART_RX_PIN = 
        CONFIG_SERIAL_COMM_UART_RX_PIN;
        
    constexpr int UART_RTS_PIN = 
        CONFIG_SERIAL_COMM_UART_RTS_PIN;
        
    constexpr int UART_CTS_PIN = 
        CONFIG_SERIAL_COMM_UART_CTS_PIN;
        
    constexpr int UART_DE_PIN = 
        CONFIG_SERIAL_COMM_UART_DE_PIN;
        

    // UART DRIVER
    constexpr size_t UART_RX_BUFFER_SIZE = 
        CONFIG_SERIAL_COMM_UART_RX_BUFFER_SIZE;
    
    constexpr size_t UART_TX_BUFFER_SIZE = 
        CONFIG_SERIAL_COMM_UART_TX_BUFFER_SIZE;
    
    constexpr size_t UART_EVENT_QUEUE_SIZE = 
        CONFIG_SERIAL_COMM_UART_EVENT_QUEUE_SIZE;
    

    // PROTOCOL
    constexpr size_t MAX_PAYLOAD_SIZE = 
        CONFIG_SERIAL_COMM_MAX_PAYLOAD_SIZE;
    
    constexpr bool ENABLE_CRC = 
        CONFIG_SERIAL_COMM_ENABLE_CRC;
    
    constexpr bool ENABLE_SEQ_ID = 
        CONFIG_SERIAL_COMM_ENABLE_SEQ_ID;
    

    // PARSER
    constexpr bool ENABLE_INTERBYTE_TIMEOUT = 
        CONFIG_SERIAL_COMM_ENABLE_INTERBYTE_TIMEOUT;

    #if defined(CONFIG_SERIAL_COMM_INTERBYTE_TIMEOUT_DYNAMIC)
        constexpr bool INTERBYTE_TIMEOUT_DYNAMIC = true;
    #else
        constexpr bool INTERBYTE_TIMEOUT_DYNAMIC = false;
    #endif

    #if defined(CONFIG_SERIAL_COMM_INTERBYTE_TIMEOUT_CHARS)
        constexpr uint32_t INTERBYTE_TIMEOUT_CHARS =
            CONFIG_SERIAL_COMM_INTERBYTE_TIMEOUT_CHARS;
    #else
        constexpr uint32_t INTERBYTE_TIMEOUT_CHARS = 0;
    #endif

    #if defined(CONFIG_SERIAL_COMM_INTERBYTE_TIMEOUT_US)
        constexpr uint32_t INTERBYTE_TIMEOUT_US =
            CONFIG_SERIAL_COMM_INTERBYTE_TIMEOUT_US;
    #else
        constexpr uint32_t INTERBYTE_TIMEOUT_US =
            (INTERBYTE_TIMEOUT_DYNAMIC && UART_BAUDRATE > 0)
                ? static_cast<uint32_t>(
                    (1000000ULL * 10ULL * INTERBYTE_TIMEOUT_CHARS) /
                    static_cast<uint64_t>(UART_BAUDRATE)
                )
                : 0;
    #endif


    // DISPATCHER
    constexpr size_t DISPATCHER_QUEUE_SIZE =
        CONFIG_SERIAL_COMM_DISPATCHER_QUEUE_SIZE;

    constexpr size_t DISPATCHER_TASK_STACK_SIZE =
        CONFIG_SERIAL_COMM_DISPATCHER_TASK_STACK_SIZE;

    constexpr uint32_t DISPATCHER_TASK_PRIORITY =
        CONFIG_SERIAL_COMM_DISPATCHER_TASK_PRIORITY;


    // TRANSACTIONS
    constexpr size_t MAX_PENDING_TRANSACTIONS =
        CONFIG_SERIAL_COMM_MAX_PENDING_TRANSACTIONS;

    constexpr uint32_t TRANSACTION_TIMEOUT_MS =
        CONFIG_SERIAL_COMM_TRANSACTION_TIMEOUT_MS;


    // MIDDLEWARE
    constexpr bool ENABLE_SERVICES =
        CONFIG_SERIAL_COMM_ENABLE_SERVICES;

    constexpr bool ENABLE_TOPICS =
        CONFIG_SERIAL_COMM_ENABLE_TOPICS;

    #if defined(CONFIG_SERIAL_COMM_ENABLE_ACTIONS)
        constexpr bool ENABLE_ACTIONS =
            CONFIG_SERIAL_COMM_ENABLE_ACTIONS;
    #else
        constexpr bool ENABLE_ACTIONS = false;
    #endif


    constexpr size_t MAX_SERVICES =
        #ifdef CONFIG_SERIAL_COMM_MAX_SERVICES
            CONFIG_SERIAL_COMM_MAX_SERVICES;
        #else
            0;
        #endif

    constexpr size_t MAX_SUBSCRIPTIONS =
        #ifdef CONFIG_SERIAL_COMM_MAX_SUBSCRIPTIONS
            CONFIG_SERIAL_COMM_MAX_SUBSCRIPTIONS;
        #else
            0;
        #endif

    constexpr size_t MAX_PUBLISHERS =
        #ifdef CONFIG_SERIAL_COMM_MAX_PUBLISHERS
            CONFIG_SERIAL_COMM_MAX_PUBLISHERS;
        #else
            0;
        #endif

    constexpr size_t MAX_ACTIONS =
        #ifdef CONFIG_SERIAL_COMM_MAX_ACTIONS
            CONFIG_SERIAL_COMM_MAX_ACTIONS;
        #else
            0;
        #endif


    // TASKS
    constexpr size_t UART_TASK_STACK_SIZE =
        CONFIG_SERIAL_COMM_UART_TASK_STACK_SIZE;

    constexpr uint32_t UART_TASK_PRIORITY =
        CONFIG_SERIAL_COMM_UART_TASK_PRIORITY;

    // Note: If core affinity is enabled, the task will be pinned to the specified core.
    constexpr bool UART_USE_TASK_CORE_AFINITY =
    #if CONFIG_SERIAL_COMM_UART_TASK_CORE >= 0
        true;
    #else
        false;
    #endif

    constexpr int UART_TASK_CORE =
        CONFIG_SERIAL_COMM_UART_TASK_CORE;


    // -----------------------------------------------------------------------
    // ESP-NOW TRANSPORT
    // -----------------------------------------------------------------------

    constexpr size_t ESPNOW_RX_QUEUE_SIZE =
        #ifdef CONFIG_SERIAL_COMM_ESPNOW_QUEUE_SIZE
            CONFIG_SERIAL_COMM_ESPNOW_QUEUE_SIZE;
        #else
            16;
        #endif

    constexpr size_t ESPNOW_TASK_STACK_SIZE =
        #ifdef CONFIG_SERIAL_COMM_ESPNOW_TASK_STACK_SIZE
            CONFIG_SERIAL_COMM_ESPNOW_TASK_STACK_SIZE;
        #else
            4096;
        #endif

    constexpr uint32_t ESPNOW_TASK_PRIORITY =
        #ifdef CONFIG_SERIAL_COMM_ESPNOW_TASK_PRIORITY
            CONFIG_SERIAL_COMM_ESPNOW_TASK_PRIORITY;
        #else
            5;
        #endif

    constexpr int ESPNOW_TASK_CORE =
        #ifdef CONFIG_SERIAL_COMM_ESPNOW_TASK_CORE
            CONFIG_SERIAL_COMM_ESPNOW_TASK_CORE;
        #else
            -1;
        #endif

    constexpr uint32_t ESPNOW_TX_TIMEOUT_MS =
        #ifdef CONFIG_SERIAL_COMM_ESPNOW_TX_TIMEOUT_MS
            CONFIG_SERIAL_COMM_ESPNOW_TX_TIMEOUT_MS;
        #else
            200;
        #endif

    constexpr bool ESPNOW_USE_TASK_CORE_AFINITY =
    #if defined(CONFIG_SERIAL_COMM_ESPNOW_TASK_CORE) && CONFIG_SERIAL_COMM_ESPNOW_TASK_CORE >= 0
        true;
    #else
        false;
    #endif

}
