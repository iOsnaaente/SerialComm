/**
 * @file    serial_comm_manager_topic.cpp
 * @brief   SerialCommManager topic handling implementation
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Created on 26 of May, 2026
 */

#include "serial_comm_manager.h"


template<typename Msg>
errCode SerialCommManager::create_subscription(
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


template<typename Msg>
errCode SerialCommManager::create_publisher( 
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


template<typename Msg>
errCode SerialCommManager::publish(
    Command command,
    const Msg& msg
) {
    uint8_t payload[ SERIAL_COMM_MAX_PAYLOAD_V1 ];
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
    return this->serial_->send(
        packet
    );
}


SerialCommManager::TopicEntry* SerialCommManager::find_topic(
    Command command
) {
    xSemaphoreTake( this->registry_mutex_, portMAX_DELAY );
    for ( size_t i = 0; i < MAX_TOPICS; i++ ) {
        if ( 
            this->topics_[i].used && 
            this->topics_[i].command == command
        ) {
            xSemaphoreGive( this->registry_mutex_ );
            return &this->topics_[i];
        }
    }
    xSemaphoreGive( this->registry_mutex_ );
    return nullptr;
}


bool SerialCommManager::is_topic_command( Command command ) const {
    for ( size_t i = 0; i < MAX_TOPICS; i++ ) {
        if (
            this->topics_[i].used &&
            this->topics_[i].command == command
        ) {
            return true;
        }
    }
    return false;
}
