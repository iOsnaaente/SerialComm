/**
 * @file    serial_comm_config.hpp
 * @brief   Serial communication middleware configuration from menuconfig
 * 
 * @note    Please dont modify this file directly, as it is generated from 
 *          the Kconfig system. 
 * 
 * @note    If you want to change any configuration, please do it through 
 *          the menuconfig interface. To do that, run: 
 *              `idf.py menuconfig` 
 *          and navigate to the Serial Communication
 */

#pragma once

#include "sdkconfig.h"

#ifndef CONFIG_SERIAL_COMM_UART_PORT
#define CONFIG_SERIAL_COMM_UART_PORT 2
#endif

#ifndef CONFIG_SERIAL_COMM_UART_BAUDRATE
#define CONFIG_SERIAL_COMM_UART_BAUDRATE 921600
#endif

#ifndef CONFIG_SERIAL_COMM_UART_TX_PIN
#define CONFIG_SERIAL_COMM_UART_TX_PIN 26
#endif

#ifndef CONFIG_SERIAL_COMM_UART_RX_PIN
#define CONFIG_SERIAL_COMM_UART_RX_PIN 25
#endif

#ifndef CONFIG_SERIAL_COMM_UART_RTS_PIN
#define CONFIG_SERIAL_COMM_UART_RTS_PIN -1
#endif

#ifndef CONFIG_SERIAL_COMM_UART_CTS_PIN
#define CONFIG_SERIAL_COMM_UART_CTS_PIN -1
#endif

#ifndef CONFIG_SERIAL_COMM_UART_DE_PIN
#define CONFIG_SERIAL_COMM_UART_DE_PIN -1
#endif

#ifndef CONFIG_SERIAL_COMM_UART_RX_BUFFER_SIZE
#define CONFIG_SERIAL_COMM_UART_RX_BUFFER_SIZE 1024
#endif

#ifndef CONFIG_SERIAL_COMM_UART_TX_BUFFER_SIZE
#define CONFIG_SERIAL_COMM_UART_TX_BUFFER_SIZE 1024
#endif

#ifndef CONFIG_SERIAL_COMM_UART_EVENT_QUEUE_SIZE
#define CONFIG_SERIAL_COMM_UART_EVENT_QUEUE_SIZE 32
#endif

#ifndef CONFIG_SERIAL_COMM_UART_TASK_STACK_SIZE
#define CONFIG_SERIAL_COMM_UART_TASK_STACK_SIZE 4096
#endif

#ifndef CONFIG_SERIAL_COMM_UART_TASK_PRIORITY
#define CONFIG_SERIAL_COMM_UART_TASK_PRIORITY 5
#endif

#ifndef CONFIG_SERIAL_COMM_UART_TASK_CORE
#define CONFIG_SERIAL_COMM_UART_TASK_CORE 1
#endif

#ifndef CONFIG_SERIAL_COMM_ENABLE_HALF_DUPLEX
#define CONFIG_SERIAL_COMM_ENABLE_HALF_DUPLEX 0
#endif

/* ============================================================================
 * UART HARDWARE CONFIGURATION
 * ==========================================================================*/

#define SERIAL_COMM_UART_PORT              CONFIG_SERIAL_COMM_UART_PORT

#define SERIAL_COMM_UART_BAUDRATE         CONFIG_SERIAL_COMM_UART_BAUDRATE

#define SERIAL_COMM_UART_TX_PIN           CONFIG_SERIAL_COMM_UART_TX_PIN

#define SERIAL_COMM_UART_RX_PIN           CONFIG_SERIAL_COMM_UART_RX_PIN

#define SERIAL_COMM_UART_RTS_PIN          CONFIG_SERIAL_COMM_UART_RTS_PIN

#define SERIAL_COMM_UART_CTS_PIN          CONFIG_SERIAL_COMM_UART_CTS_PIN

#define SERIAL_COMM_UART_DE_PIN           CONFIG_SERIAL_COMM_UART_DE_PIN

/* ============================================================================
 * UART DRIVER CONFIGURATION
 * ==========================================================================*/

#define SERIAL_COMM_UART_RX_BUFFER_SIZE   CONFIG_SERIAL_COMM_UART_RX_BUFFER_SIZE

#define SERIAL_COMM_UART_TX_BUFFER_SIZE   CONFIG_SERIAL_COMM_UART_TX_BUFFER_SIZE

#define SERIAL_COMM_UART_QUEUE_SIZE       CONFIG_SERIAL_COMM_UART_EVENT_QUEUE_SIZE

/* ============================================================================
 * TASK CONFIGURATION
 * ==========================================================================*/

#define SERIAL_COMM_UART_TASK_STACK       CONFIG_SERIAL_COMM_UART_TASK_STACK_SIZE

#define SERIAL_COMM_UART_TASK_PRIORITY    CONFIG_SERIAL_COMM_UART_TASK_PRIORITY

#define SERIAL_COMM_UART_TASK_CORE        CONFIG_SERIAL_COMM_UART_TASK_CORE

/* ============================================================================
 * HALF DUPLEX
 * ==========================================================================*/

#define SERIAL_COMM_HALF_DUPLEX_ENABLED   CONFIG_SERIAL_COMM_ENABLE_HALF_DUPLEX