/**
 * @file    serial_comm_manager_action.cpp
 * @brief   SerialCommManager action handling implementation
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Created on 26 of May, 2026
 */

#include "serial_comm_manager.h"

template< typename Goal, typename Feedback, typename Result >
errCode SerialCommManager::create_action(
    SerialCommAction< Goal, Feedback, Result >* action
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