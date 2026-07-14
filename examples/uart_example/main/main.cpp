/**
 * @file    main.cpp
 * @brief   SerialComm — UART Transport Example
 *
 * Demonstrates the SerialComm middleware stack running over a UART link.
 * Two ESP32 devices are connected TX↔RX / RX↔TX with a shared GND.
 *
 * Select which role this board plays by editing the define below:
 *
 *   ROLE_SERVER  – provides the PING service and publishes SensorData
 *   ROLE_CLIENT  – calls the PING service and subscribes to SensorData
 *
 * Default is ROLE_SERVER.  Flash two boards with opposite roles and watch
 * the interaction over the serial monitor.
 *
 * Hardware (default pins, change via menuconfig):
 *   TX  →  GPIO 17
 *   RX  →  GPIO 16
 *   GND →  GND (shared)
 *
 * Required menuconfig settings (Component config → Serial Communication):
 *   UART_PORT   = 2
 *   TX_PIN      = 17
 *   RX_PIN      = 16
 *   BAUD_RATE   = 115200
 */

// ---------------------------------------------------------------------------
// Role selection — edit here or add a proper Kconfig entry
// ---------------------------------------------------------------------------
#define ROLE_SERVER
// #define ROLE_CLIENT

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "serial_comm/transport/uart_serial_comm.h"
#include "serial_comm/serial_comm.h"
#include "serial_comm/middleware/serial_comm_manager.h"
#include "serial_comm/middleware/service/serial_comm_service.h"
#include "serial_comm/middleware/topic/serial_comm_topic.h"
#include "serial_comm/common/serial_comm_serializer_base.hpp"

static const char* TAG = "UART_EXAMPLE";

// ---------------------------------------------------------------------------
// Application message types
// ---------------------------------------------------------------------------

struct PingRequest {
    uint8_t node_id;
};

struct PingResponse {
    uint8_t node_id;
    bool    alive;
};

struct SensorData {
    float    temperature;   // degrees Celsius
    float    humidity;      // %RH
    uint32_t timestamp_ms;  // milliseconds since boot
};

// ---------------------------------------------------------------------------
// Serialiser specialisations
// ---------------------------------------------------------------------------

template<>
struct SerialCommSerializer<PingRequest> {
    static bool serialize(
        const PingRequest& msg, uint8_t* buf, size_t buf_sz, size_t& out_sz
    ) {
        if (buf_sz < 1) return false;
        buf[0] = msg.node_id;
        out_sz = 1;
        return true;
    }
    static bool deserialize(
        const uint8_t* buf, size_t buf_sz, PingRequest& msg, size_t* consumed = nullptr
    ) {
        if (buf_sz < 1) return false;
        msg.node_id = buf[0];
        if (consumed) *consumed = 1;
        return true;
    }
};

template<>
struct SerialCommSerializer<PingResponse> {
    static bool serialize(
        const PingResponse& msg, uint8_t* buf, size_t buf_sz, size_t& out_sz
    ) {
        if (buf_sz < 2) return false;
        buf[0] = msg.node_id;
        buf[1] = msg.alive ? 1u : 0u;
        out_sz = 2;
        return true;
    }
    static bool deserialize(
        const uint8_t* buf, size_t buf_sz, PingResponse& msg, size_t* consumed = nullptr
    ) {
        if (buf_sz < 2) return false;
        msg.node_id = buf[0];
        msg.alive   = (buf[1] != 0);
        if (consumed) *consumed = 2;
        return true;
    }
};

template<>
struct SerialCommSerializer<SensorData> {
    static bool serialize(
        const SensorData& msg, uint8_t* buf, size_t buf_sz, size_t& out_sz
    ) {
        if (buf_sz < sizeof(SensorData)) return false;
        memcpy(buf, &msg, sizeof(SensorData));
        out_sz = sizeof(SensorData);
        return true;
    }
    static bool deserialize(
        const uint8_t* buf, size_t buf_sz, SensorData& msg, size_t* consumed = nullptr
    ) {
        if (buf_sz < sizeof(SensorData)) return false;
        memcpy(&msg, buf, sizeof(SensorData));
        if (consumed) *consumed = sizeof(SensorData);
        return true;
    }
};

// ---------------------------------------------------------------------------
// Server-side handlers
// ---------------------------------------------------------------------------

static bool ping_server_handler(const PingRequest& req, PingResponse& res) {
    ESP_LOGI(TAG, "[SERVICE] PING from node %u — responding", req.node_id);
    res.node_id = req.node_id;
    res.alive   = true;
    return true;
}

static void sensor_subscriber_cb(const SensorData& msg) {
    ESP_LOGI(TAG, "[SUB] Sensor @ %lu ms: T=%.1f°C  H=%.1f%%",
             (unsigned long)msg.timestamp_ms, msg.temperature, msg.humidity);
}

// ---------------------------------------------------------------------------
// app_main
// ---------------------------------------------------------------------------

extern "C" void app_main(void) {

    // -----------------------------------------------------------------------
    // 1. Transport
    // -----------------------------------------------------------------------
    UARTTransport::HardwareConfig hw{};
    hw.uart_port = UART_NUM_2;
    hw.tx_pin    = 17;
    hw.rx_pin    = 16;
    // Baud rate and buffer sizes are configured via menuconfig
    // (Component config → Serial Communication → UART settings)

    static UARTTransport transport(hw);

    if (transport.init() != errCode::OK) {
        ESP_LOGE(TAG, "UART transport init failed");
        return;
    }
    if (transport.start() != errCode::OK) {
        ESP_LOGE(TAG, "UART transport start failed");
        return;
    }
    ESP_LOGI(TAG, "UART transport running on UART%d (TX=GPIO%d RX=GPIO%d)",
             (int)hw.uart_port, hw.tx_pin, hw.rx_pin);

    // -----------------------------------------------------------------------
    // 2. SerialComm core
    // -----------------------------------------------------------------------
    static SerialComm comm(&transport);

    if (comm.init() != errCode::OK) {
        ESP_LOGE(TAG, "SerialComm init failed");
        return;
    }
    if (comm.start() != errCode::OK) {
        ESP_LOGE(TAG, "SerialComm start failed");
        return;
    }

    // -----------------------------------------------------------------------
    // 3. Middleware manager
    // -----------------------------------------------------------------------
    static SerialCommManager mgr(&comm);

    if (mgr.init() != errCode::OK) {
        ESP_LOGE(TAG, "SerialCommManager init failed");
        return;
    }

    // -----------------------------------------------------------------------
    // 4. Services and topics
    // -----------------------------------------------------------------------

#ifdef ROLE_SERVER
    // --- Service: PING server ---
    static SerialCommService<PingRequest, PingResponse> ping_svc;
    ping_svc.init(SerialCommCommand::PING, ping_server_handler);
    mgr.create_service<PingRequest, PingResponse>(&ping_svc);

    // --- Topic: SensorData subscriber (receives from the remote client) ---
    static SerialCommTopic<SensorData> sensor_sub;
    sensor_sub.init(SerialCommCommand::STATUS_TOPIC, sensor_subscriber_cb);
    mgr.create_subscription<SensorData>(&sensor_sub);

    ESP_LOGI(TAG, "Role: SERVER — PING service active, publishing SensorData");
#endif

#ifdef ROLE_CLIENT
    // --- Topic: SensorData subscriber ---
    static SerialCommTopic<SensorData> sensor_sub;
    sensor_sub.init(SerialCommCommand::STATUS_TOPIC, sensor_subscriber_cb);
    mgr.create_subscription<SensorData>(&sensor_sub);

    ESP_LOGI(TAG, "Role: CLIENT — subscribed to SensorData, calling PING every 5 s");
#endif

    if (mgr.start() != errCode::OK) {
        ESP_LOGE(TAG, "SerialCommManager start failed");
        return;
    }

    // -----------------------------------------------------------------------
    // 5. Main loop
    // -----------------------------------------------------------------------
    uint32_t tick = 0;

    while (true) {
#ifdef ROLE_SERVER
        // Publish SensorData — the remote client subscribes to STATUS_TOPIC
        SensorData reading{};
        reading.temperature  = 25.0f + (tick % 10) * 0.5f;
        reading.humidity     = 60.0f + (tick % 20) * 0.5f;
        reading.timestamp_ms = pdTICKS_TO_MS(xTaskGetTickCount());

        errCode pub_err = mgr.publish<SensorData>(SerialCommCommand::STATUS_TOPIC, reading);
        if (pub_err == errCode::OK) {
            ESP_LOGD(TAG, "Published SensorData tick=%lu", (unsigned long)tick);
        } else {
            ESP_LOGW(TAG, "Publish failed: %s", err_to_str(pub_err));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
#endif

#ifdef ROLE_CLIENT
        // Every 5 seconds: call the PING service on the remote server
        if (tick % 5 == 0) {
            PingRequest req{};
            req.node_id = 0x01;

            PingResponse res{};
            errCode svc_err = mgr.call_service<PingRequest, PingResponse>(
                SerialCommCommand::PING, req, res, /*timeout_ms=*/2000
            );

            if (svc_err == errCode::OK) {
                ESP_LOGI(TAG, "PING reply: node=%u alive=%s",
                         res.node_id, res.alive ? "true" : "false");
            } else {
                ESP_LOGW(TAG, "PING call failed: %s", err_to_str(svc_err));
            }
        }

        tick++;
        vTaskDelay(pdMS_TO_TICKS(1000));
#endif
    }
}
