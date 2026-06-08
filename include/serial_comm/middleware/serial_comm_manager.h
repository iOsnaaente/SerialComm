/**
 * @file    serial_comm_manager.h
 * @brief   High-level semantic middleware manager for SerialComm
 * @details Responsible for:
 *              - Service orchestration
 *              - Topic orchestration
 *              - Action orchestration
 *              - Transaction management
 *              - Packet semantic routing
 *              - Request/Reply abstraction
 *              - Typed communication API
 *
 *          This class is the main public middleware API and abstracts
 *          low-level packet manipulation from the application layer.
 *
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Created on 26 of May, 2026
 */

#pragma once


#include "serial_comm/messages/serial_comm_messages.h"
#include "serial_comm/serial_comm_config.h"
#include "serial_comm/core/serial_comm_protocol.h"
#include "serial_comm/core/serial_comm_utils.h"
#include "serial_comm/serial_comm.h"

#include "serial_comm/middleware/serial_comm_transaction_manager.h"
#include "serial_comm/common/serial_comm_serializer_base.hpp"

#include "serial_comm/generated/generated_serializers.hpp"

#include "serial_comm/middleware/service/serial_comm_service.h"
#include "serial_comm/middleware/action/serial_comm_action.h"
#include "serial_comm/middleware/topic/serial_comm_topic.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <stdint.h>
#include <stddef.h>


/* To use the errCode and err_to_str easily */
using namespace SerialCommResult_Codes;

// Alias for command type used across the middleware
using Command = SerialCommCommand;


/**
 * @brief High-level Serial Communication Middleware Manager
 */
class SerialCommManager {
    private:
        static constexpr size_t MAX_SERVICES =
            (SerialCommConfig::MAX_SERVICES > 0)
                ? SerialCommConfig::MAX_SERVICES
                : 1;
        static constexpr size_t MAX_TOPICS =
            ((SerialCommConfig::MAX_SUBSCRIPTIONS +
                SerialCommConfig::MAX_PUBLISHERS) > 0)
                    ? (SerialCommConfig::MAX_SUBSCRIPTIONS + SerialCommConfig::MAX_PUBLISHERS)
                    : 1;
        static constexpr size_t MAX_ACTIONS =
            (SerialCommConfig::MAX_ACTIONS > 0)
                ? SerialCommConfig::MAX_ACTIONS
                : 1;

    // Service, Topic and Action registries
    private: 
        struct ServiceEntry {
            bool used = false;
            Command command = static_cast<Command>(0);
            IServiceBase* service = nullptr;
        };
        struct TopicEntry {
            bool used = false;
            Command command = static_cast<Command>(0);
            ITopicBase* topic = nullptr;
        };
        struct ActionEntry {
            bool used = false;
            Command command = static_cast<Command>(0);
            IActionBase* action = nullptr;
        };

    private:
        ServiceEntry services_[ MAX_SERVICES ];
        TopicEntry topics_[ MAX_TOPICS ];
        ActionEntry actions_[ MAX_ACTIONS ];

    
    private:
        SerialCommTransactionManager transactions_;
        SemaphoreHandle_t registry_mutex_ = nullptr;


    public:

        /**
         * @brief   Middleware runtime configuration
         * @param   enable_auto_reply If true, the manager will 
         *          automatically send replies for handled requests
         * @param   enable_transactions If true, the manager will manage 
         *          request-reply transactions and timeouts
         * @param   service_timeout_ms Default timeout for service calls 
         *          in milliseconds
         * @param   enable_logs If true, the manager will output logs for 
         *          operations and errors
         */
        struct Config {
            bool enable_auto_reply = true;      
            bool enable_transactions = true;
            uint32_t service_timeout_ms =
                SerialCommConfig::TRANSACTION_TIMEOUT_MS;
            bool enable_logs = true;            
        };

    private:

        /* Underlying SerialComm instance */
        SerialComm* serial_ = nullptr;
        Config cfg_;

        bool initialized_ = false;
        bool running_ = false;

        /* Sequence ID generator */
        uint16_t next_seq_id_ = 1;
        /* Sequence ID protection mutex */
        SemaphoreHandle_t seq_mutex_ = nullptr;


    public:

        /**
         * @brief Constructor
         * @param serial Underlying SerialComm instance
         */
        explicit SerialCommManager( SerialComm* serial )
            : serial_( serial ) { }

        /* Delete copy constructor and assignment operator */
        SerialCommManager( const SerialCommManager& ) = delete;
        SerialCommManager& operator=(const SerialCommManager& ) = delete;
        /* Delete move constructor and assignment operator */
        SerialCommManager( SerialCommManager&& ) = delete;
        SerialCommManager& operator=( SerialCommManager&& ) = delete;


    public:

        /**
         * @brief   Initialize middleware manager
         * @param   cfg Runtime configuration
         * @return  Result code
         */
        errCode init( const Config& cfg);

        /**
         * @brief   Start middleware manager
         * @return  Result code
         */
        errCode start();

        /**
         * @brief   Stop middleware manager
         * @return  Result code
         */
        errCode stop();

        /**
         * @brief   Deinitialize middleware manager
         * @return  Result code
         */
        errCode deinit();


    // SERVICES
    public:

        /**
         * @brief   Create service server
         * @tparam  Req Request type
         * @tparam  Res Response type
         * @return  Result code
         */
        template< typename Req, typename Res >
        errCode create_service(
            SerialCommService<Req, Res>* service
        ) {
            if ( service == nullptr ) {
                return errCode::ERR_NULL_POINTER;
            }
            if ( !service->initialized() ) {
                return errCode::ERR_NOT_INITIALIZED;
            }
            if ( xSemaphoreTake( this->registry_mutex_, portMAX_DELAY ) != pdTRUE ) {
                return errCode::ERR_TIMEOUT;
            }
            for ( size_t i = 0; i < MAX_SERVICES; i++ ) {
                if ( !this->services_[i].used ) {
                    this->services_[i].used = true;
                    this->services_[i].command = service->command();
                    this->services_[i].service = service;
                    xSemaphoreGive( this->registry_mutex_ );
                    return errCode::OK;
                }
            }
            xSemaphoreGive( this->registry_mutex_ );
            return errCode::ERR_NO_MEMORY;
        }


        /**
         * @brief   Call remote service
         * @tparam  Req Request type
         * @tparam  Res Response type
         * @param   command Service command ID
         * @param   request Request object
         * @param   response Output response object
         * @param   timeout_ms Timeout in milliseconds
         * @return  Result code
         */
        template< typename Req, typename Res>
        errCode call_service(
            Command command,
            const Req& request,
            Res& response,
            uint32_t timeout_ms = SerialCommConfig::TRANSACTION_TIMEOUT_MS
        ) {
            uint8_t payload[ SERIAL_COMM_MAX_PAYLOAD ];
            size_t payload_size = 0;

            bool ok = SerialCommSerializer<Req>::serialize(
                request,
                payload,
                sizeof(payload),
                payload_size
            );
            if ( !ok ) {
                return errCode::ERR_FAIL;
            }

            uint16_t seq_id = this->allocate_seq_id();
            errCode err = this->transactions_.create_transaction( seq_id );
            if ( err != errCode::OK ) {
                return err;
            }

            SerialCommProtoPacket request_packet;
            err = build_packet(
                command,
                seq_id,
                payload,
                payload_size,
                request_packet
            );
            if ( err != errCode::OK ) {
                return err;
            }

            err = this->serial_->send( request_packet );
            if ( err != errCode::OK ) {
                return err;
            }

            SerialCommProtoPacket reply_packet;
            err = this->transactions_.wait_reply(
                seq_id,
                reply_packet,
                timeout_ms
            );
            if ( err != errCode::OK ) {
                return err;
            }
            
            size_t consumed_size = 0;
            ok = SerialCommSerializer<Res>::deserialize(
                reply_packet.payload,
                reply_packet.header.payload_len,
                response,
                &consumed_size
            );
            if ( !ok ) {
                return errCode::ERR_FAIL;
            }
            return errCode::OK;
        }


    // TOPICS
    public:

        /**
         * @brief   Create topic publisher
         * @tparam  Msg Topic message type
         * @return  Result code
         */
        template< typename Msg >
        errCode create_publisher( 
            SerialCommTopic<Msg>* publisher
        ) {
            if ( publisher == nullptr ) {
                return errCode::ERR_NULL_POINTER;
            }
            if ( !publisher->initialized() ) {
                return errCode::ERR_NOT_INITIALIZED;
            }
            xSemaphoreTake( this->registry_mutex_, portMAX_DELAY );
            for ( size_t i = 0; i < MAX_TOPICS; i++) {
                if ( !this->topics_[i].used ) {
                    this->topics_[i].used = true;
                    this->topics_[i].command = publisher->command();
                    this->topics_[i].topic = publisher;
                    xSemaphoreGive( this->registry_mutex_ );
                    return errCode::OK;
                }
            }
            xSemaphoreGive( this->registry_mutex_ );
            return errCode::ERR_NO_MEMORY;
        }

        
        /**
         * @brief   Create topic subscription
         * @tparam  Msg Topic message type
         * @return  Result code
         */
        template< typename Msg >
        errCode create_subscription(
            SerialCommTopic<Msg>* subscription
        ) {
            if ( subscription == nullptr ) {
                return errCode::ERR_NULL_POINTER;
            }
            if ( !subscription->initialized() ) {
                return errCode::ERR_NOT_INITIALIZED;
            }
            xSemaphoreTake( this->registry_mutex_, portMAX_DELAY );
            for ( size_t i = 0; i < MAX_TOPICS; i++) {
                if ( !this->topics_[i].used ) {
                    this->topics_[i].used = true;
                    this->topics_[i].command = subscription->command();
                    this->topics_[i].topic = subscription;
                    xSemaphoreGive( this->registry_mutex_ );
                    return errCode::OK;
                }
            }
            xSemaphoreGive( this->registry_mutex_ );
            return errCode::ERR_NO_MEMORY;
        }


        /**
         * @brief   Publish topic message
         * @tparam  Msg Topic message type
         * @param   command Topic command ID
         * @param   msg Message object
         * @return  Result code
         */
        template< typename Msg >
        errCode publish( Command command, const Msg& msg ) {
            uint8_t payload[ SERIAL_COMM_MAX_PAYLOAD ];
            size_t payload_size = 0;
            bool ok = Serializer<Msg>::serialize(
                msg,
                payload,
                sizeof(payload),
                payload_size
            );
            if ( !ok ) {
                return errCode::ERR_FAIL;
            }

            SerialCommProtoPacket packet;
            errCode err = build_packet(
                command,
                0,
                payload,
                payload_size,
                packet
            );
            if ( err != errCode::OK ) {
                return err;
            }
            return this->serial_->send( packet );
        }


    // ACTIONS
    public:

        /**
         * @brief   Create action server
         * @tparam  Goal Action goal type
         * @tparam  Feedback Action feedback type
         * @tparam  Result Action result type
         * @param   command Action command ID
         * @param   callback Action callback
         * @return  Result code
         * @note    Future implementation
         */
        template< typename Goal, typename Feedback, typename Result >
        errCode create_action(
            SerialCommAction<Goal, Feedback, Result>* action
        ) {
            if ( action == nullptr ) {
                return errCode::ERR_NULL_POINTER;
            }
            if ( !action->initialized() ) {
                return errCode::ERR_NOT_INITIALIZED;
            }
            xSemaphoreTake( this->registry_mutex_, portMAX_DELAY );
            for ( size_t i = 0; i < MAX_ACTIONS; i++ ) {
                if ( !this->actions_[i].used ) {
                    this->actions_[i].used = true;
                    this->actions_[i].command = action->command();
                    this->actions_[i].action = action;
                    xSemaphoreGive( this->registry_mutex_ );
                    return errCode::OK;
                }
            }
            xSemaphoreGive( this->registry_mutex_ );
            return errCode::ERR_NO_MEMORY;
        }


    // INTERNAL ROUTING
    private:

        /**
         * @brief   Internal packet callback from SerialComm
         * @param   ctx User context (pointer to this manager instance)
         * @param   packet Received protocol packet
         */
        static void serial_packet_callback(
            void* ctx,
            const SerialCommProtoPacket& packet
        );

        /**
         * @brief   Route received packet
         * @param   packet Incoming packet
         */
        void route_packet( const SerialCommProtoPacket& packet );

        /**
         * @brief   Handle reply packet
         * @param   packet Incoming reply packet
         */
        void handle_reply( const SerialCommProtoPacket& packet );


    // UTILITIES
    private:

        /**
         * @brief   Generate next sequence ID
         * @return  Sequence ID
         */
        uint16_t allocate_seq_id();

        /**
         * @brief   Build protocol packet
         * @param   command Service command ID
         * @param   seq_id Sequence ID
         * @param   payload Payload data
         * @param   payload_len Payload length
         * @param   out_packet Output protocol packet
         * @return  Result code
         */
        errCode build_packet(
            Command command,
            uint16_t seq_id,
            const uint8_t* payload,
            size_t payload_len,
            SerialCommProtoPacket& out_packet
        );

    // HELPERS
    private:
    
        /**
         * @brief   Find service entry by command ID
         * @param   command Service command ID
         * @return  Pointer to service entry or nullptr if not found
         */
        ServiceEntry* find_service( Command command );


        /**
         * @brief   Find topic entry by command ID
         * @param   command Topic command ID
         * @return  Pointer to topic entry or nullptr if not found
         */
        TopicEntry* find_topic( Command command );


        /**
         * @brief   Find action entry by command ID
         * @param   command Action command ID
         * @return  Pointer to action entry or nullptr if not found
         */
        ActionEntry* find_action( Command command );


        /**
         * @brief   Clear all service, topic and action registries
         */
        void clear_registries();


    // ROUTERS
    private:

        /**
         * @brief   Look up and run the service handler for a request packet
         * @param   packet Incoming service request packet
         * @return  true if a registered service handled the packet,
         *          false if no service is registered for its command
         */
        bool handle_service_request( const SerialCommProtoPacket& packet );

        /**
         * @brief   Look up and run the topic handler for a message packet
         * @param   packet Incoming topic message packet
         * @return  true if a registered topic handled the packet,
         *          false if no topic is registered for its command
         */
        bool handle_topic_message( const SerialCommProtoPacket& packet );

        /**
         * @brief   Look up and run the action handler for a goal/feedback/result packet
         * @param   packet Incoming action packet
         * @return  true if a registered action handled the packet,
         *          false if no action is registered for its command
         */
        bool handle_action_packet( const SerialCommProtoPacket& packet );


    // GETTERS
    public:

        /**
         * @brief Check if manager is initialized
         */
        bool initialized() const;

        /**
         * @brief Check if manager is running
         */
        bool running() const;

        /**
         * @brief Get next sequence ID value
         */
        uint16_t current_seq_id() const;


    public:
    
        /**
         * @brief Destructor
         */
        virtual ~SerialCommManager();
};