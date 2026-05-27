/**
 * @file    serial_comm_messages.h
 * @brief   Serial communication protocol, structs, types and constants
 */

#pragma once 

#include <stdint.h>
#include <stddef.h>

/**
 * @brief   Serial communication protocol commands
 * @note    To add a new command, add it before the REPLY command and 
 *          ensure the REPLY_MASK is set correctly
 */
enum class SerialCommCommand : uint8_t {
    UNDEFINED     = 0x00,
    READ          = 0x01,
    WRITE         = 0x02,
    PING          = 0x03,
    STATUS_TOPIC  = 0x06,
    READ_UTILITY  = 0x04,
    WRITE_UTILITY = 0x05,
};


/**
 * @brief       Serial communication protocol commands to string conversion
 * @details     Useful for logs/debugging.
 * @param[in]   cmd SerialCommCommand enum
 * @return      const char* SerialCommCommand string
 */
static inline const char* serial_command_to_str(SerialCommCommand cmd){
    switch (cmd) {
        case SerialCommCommand::UNDEFINED:      return "UNDEFINED";
        case SerialCommCommand::READ:           return "READ";
        case SerialCommCommand::WRITE:          return "WRITE";
        case SerialCommCommand::PING:           return "PING";
        case SerialCommCommand::STATUS_TOPIC:   return "STATUS_TOPIC";
        case SerialCommCommand::READ_UTILITY:   return "READ_UTILITY";
        case SerialCommCommand::WRITE_UTILITY:  return "WRITE_UTILITY";
        default:                                return "UNKNOWN_COMMAND";
    }
}

using Command = SerialCommCommand;

/**
 * @brief   Command flag bit mask
 */
#define SERIAL_COMM_CMD_REPLY_MASK    0x80


/**
 * @brief   Commands Helpers 
 */
static inline bool is_reply( SerialCommCommand cmd ) {
    return ( static_cast<uint8_t>(cmd) & SERIAL_COMM_CMD_REPLY_MASK ) != 0;
}

static inline bool is_request( SerialCommCommand cmd ) {
    return !is_reply(cmd);
}


static inline SerialCommCommand make_reply( SerialCommCommand cmd ) {
    return static_cast<SerialCommCommand>( 
        static_cast<uint8_t>(cmd) | 
        SERIAL_COMM_CMD_REPLY_MASK
    );
}


static inline SerialCommCommand make_request( SerialCommCommand cmd ) {
    return static_cast<SerialCommCommand>(
        static_cast<uint8_t>(cmd) &
        ~SERIAL_COMM_CMD_REPLY_MASK
    );
}