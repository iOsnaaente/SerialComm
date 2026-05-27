/**
 * @file    serial_comm_transaction_manager.h
 * @brief   Transaction manager for SerialComm middleware
 * @details Responsible for:
 *              - Pending request tracking
 *              - Reply matching using seq_id
 *              - Request timeout management
 *              - Synchronous service waiting
 *              - Transaction lifecycle management
 *
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Created on 26 of May, 2026
 */

#pragma once

#include "serial_comm/messages/serial_comm_messages.h"
#include "serial_comm/serial_comm_config.h"

#include "serial_comm/core/serial_comm_protocol.h"
#include "serial_comm/core/serial_comm_utils.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>


/* To use the errCode and err_to_str easily */
using namespace SerialCommResult_Codes;


/**
 * @brief Transaction manager
 */
class SerialCommTransactionManager {
    private:
        static constexpr size_t MAX_TRANSACTIONS =
            (SerialCommConfig::MAX_PENDING_TRANSACTIONS > 0)
                ? SerialCommConfig::MAX_PENDING_TRANSACTIONS
                : 1;

    private:
        /**
         * @brief   Pending transaction entry
         * @param   active Whether the transaction slot is active
         * @param   seq_id Sequence ID of the transaction
         * @param   completed Whether the transaction has been completed
         * @param   reply Reply packet for the transaction
         * @param   semaphore Semaphore for transaction synchronization
         */
        struct Transaction {
            bool active = false;
            uint16_t seq_id = 0;
            bool completed = false;
            SerialCommProtoPacket reply;
            SemaphoreHandle_t semaphore = nullptr;
        };


    private:

        /**
         * @brief   Transaction table
         * TODO:    Fixed-size array of transactions for simplicity. 
         *          In a real implementation, a more efficient data structure 
         *          (e.g., hash map) could be used.
         */
        Transaction transactions_[ MAX_TRANSACTIONS ];

        /**
         * @brief Internal protection mutex
         */
        SemaphoreHandle_t mutex_ = nullptr;
        bool initialized_ = false;


    public:

        /**
         * @brief Constructor
         */
        SerialCommTransactionManager() = default;

        /* Delete copy constructor and assignment operator */
        SerialCommTransactionManager( const SerialCommTransactionManager& ) = delete;
        SerialCommTransactionManager& operator=(const SerialCommTransactionManager& ) = delete;


    public:

        /**
         * @brief   Initialize transaction manager
         * @return  Result code
         */
        errCode init();


        /**
         * @brief   Deinitialize transaction manager
         * @return  Result code
         */
        errCode deinit();


    public:

        /**
         * @brief   Allocate transaction slot
         * @param   seq_id Transaction sequence ID
         * @return  Result code
         */
        errCode create_transaction( uint16_t seq_id );


        /**
         * @brief   Wait for transaction reply
         * @param   seq_id Transaction sequence ID
         * @param   out_reply Output reply packet
         * @param   timeout_ms Wait timeout
         * @return  Result code
         */
        errCode wait_reply(
            uint16_t seq_id,
            SerialCommProtoPacket& out_reply,
            uint32_t timeout_ms
        );


        /**
         * @brief   Resolve transaction reply
         * @param   packet Reply packet
         * @return  Result code
         */
        errCode resolve_transaction(
            const SerialCommProtoPacket& packet
        );


    private:

        /**
         * @brief Find transaction by seq_id
         */
        Transaction* find_transaction( uint16_t seq_id );


        /**
         * @brief Destroy transaction
         */
        void destroy_transaction( uint16_t seq_id );


    public:
        /**
         * @brief Destructor
         */
        virtual ~SerialCommTransactionManager();

};