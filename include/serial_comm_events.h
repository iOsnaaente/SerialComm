/**
 * @brief This file defines the events for the serial communication component.
 */

#pragma once

#include "esp_event.h"


ESP_EVENT_DECLARE_BASE(SERIAL_COMM_EVENTS);

enum EventID{
    EVENT_PACKET_RECEIVED = 0,
    EVENT_READ_JOINTS,
    EVENT_WRITE_JOINTS,
    EVENT_PING,
    EVENT_READ_UTILITY,
    EVENT_WRITE_UTILITY,
    EVENT_SMOOTH_MOVE,
    EVENT_PROTOCOL_ERROR
};
