#pragma once 
/**
 * @brief This file defines the event loop for the serial communication component.
 */

#include "serial_comm_types.h"

#include "esp_event.h"
#include "esp_err.h"

using packet_callback_t =
    void (*)(const Packet& pkt);

class EventLoop {
public:

    static esp_err_t init();

    static esp_err_t register_callback(
        int32_t event_id,
        esp_event_handler_t callback
    );

    static esp_err_t post_event(
        int32_t event_id,
        const Packet& pkt
    );

private:

    static esp_event_loop_handle_t loop_;
};
