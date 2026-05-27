/**
 * @file    serial_comm_manager_topic.cpp
 * @brief   SerialCommManager topic handling implementation
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Created on 26 of May, 2026
 */

#include "serial_comm_manager.h"


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
