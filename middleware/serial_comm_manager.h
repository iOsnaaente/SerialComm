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


#include "core/serial_comm_messages.h"
#include "core/serial_comm_protocol.h"
#include "core/serial_comm_utils.h"
#include "core/serial_comm.h"

#include "middleware/serial_comm_transaction_manager.h"
#include "middleware/serial_comm_serializer.h"

#include "middleware/service/serial_comm_service.h"
#include "middleware/action/serial_comm_action.h"
#include "middleware/topic/serial_comm_topic.h"

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
        static constexpr size_t MAX_SERVICES = 32;
        static constexpr size_t MAX_TOPICS = 32;
        static constexpr size_t MAX_ACTIONS = 16;

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
            uint32_t service_timeout_ms = 1000; 
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
            SerialCommService<Req, Res>* service,
        );


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
            uint32_t timeout_ms = portMAX_DELAY
        );


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
        );

        
        /**
         * @brief   Create topic subscription
         * @tparam  Msg Topic message type
         * @return  Result code
         */
        template< typename Msg >
        errCode create_subscription(
            SerialCommTopic<Msg>* subscription
        );


        /**
         * @brief   Publish topic message
         * @tparam  Msg Topic message type
         * @param   command Topic command ID
         * @param   msg Message object
         * @return  Result code
         */
        template< typename Msg >
        errCode publish( Command command, const Msg& msg );


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
        );


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
         * @brief   Handle request packet
         * @param   packet Incoming request packet
         */
        void handle_request( const SerialCommProtoPacket& packet );

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

        /**
         * @brief   Send reply packet
         * @param   command Service command ID
         * @param   seq_id Sequence ID to reply to
         * @param   payload Payload data
         * @param   payload_len Payload length
         * @return  Result code
         */
        errCode send_reply(
            Command command,
            uint16_t seq_id,
            const uint8_t* payload,
            size_t payload_len
        );

        /**
         * @brief   Service send reply message helper
         * @tparam  Res Response type
         * @param   command Service command ID
         * @param   seq_id Sequence ID to reply to
         * @param   response Response object to serialize and send
         * @return  Result code
         */
        template< typename Res >
        errCode send_reply_message(
            Command command,
            uint16_t seq_id,
            const Res& response
        );

        /**
         * @brief   Publish topic message helper
         * @tparam  Msg Topic message type
         * @param   command Topic command ID
         * @param   message Message object to serialize and publish
         * @return  Result code
         */
        template< typename Msg >
        errCode publish(
            Command command,
            const Msg& message
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
         * @brief   Check if command ID corresponds to a service
         * @param   command Command ID
         * @return  True if it's a service command
         * @result  False if it's not a service command 
         */
        bool is_service_command( Command command ) const;
        
        
        /**
         * @brief   Check if command ID corresponds to a topic
         * @param   command Command ID
         * @return  True if it's a topic command
         * @result  False if it's not a topic command
         */
        bool is_topic_command( Command command ) const;
        

        /**
         * @brief   Check if command ID corresponds to an action
         * @param   command Command ID
         * @return  True if it's an action command
         * @result  False if it's not an action command
         */
        bool is_action_command( Command command ) const;


        /**
         * @brief   Clear all service, topic and action registries
         */
        void clear_registries();


    // ROUTERS
    private:
    
        /**
         * @brief   Route service request packet
         * @param   packet Incoming service request packet
         */
        void handle_service_request( const SerialCommProtoPacket& packet );

        /**
         * @brief   Route topic message packet
         * @param   packet Incoming topic message packet
         */
        void handle_topic_message( const SerialCommProtoPacket& packet );

        /**
         * @brief   Route action goal/feedback/result packet
         * @param   packet Incoming action packet
         */
        void handle_action_packet( const SerialCommProtoPacket& packet );


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