/**
 * @file    serial_comm_manager_action.cpp
 * @brief   SerialCommManager action handling implementation
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Created on 26 of May, 2026
 */

#include "serial_comm/middleware/serial_comm_manager.h"

bool SerialCommManager::is_action_command( Command command ) const {
    for ( size_t i = 0; i < MAX_ACTIONS; i++ ) {
        if (
            this->actions_[i].used &&
            this->actions_[i].command == command
        ) {
            return true;
        }
    }
    return false;
}


SerialCommManager::ActionEntry* SerialCommManager::find_action(
    Command command
) {
    xSemaphoreTake( this->registry_mutex_, portMAX_DELAY );
    for ( size_t i = 0; i < MAX_ACTIONS; i++ ) {
        if ( 
            this->actions_[i].used && 
            this->actions_[i].command == command
        ) {
            xSemaphoreGive( this->registry_mutex_ );
            return &this->actions_[i];
        }
    }
    xSemaphoreGive( this->registry_mutex_ );
    return nullptr;
}