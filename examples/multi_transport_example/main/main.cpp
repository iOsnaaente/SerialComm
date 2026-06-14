/**
 * @file    main.cpp
 * @brief   SerialComm — MultiTransport Example
 *
 * Demonstrates simultaneous use of UART and ESP-NOW transports behind a
 * single SerialComm / SerialCommManager instance using MultiTransport.
 *
 * This device acts as a BRIDGE SERVER node:
 *   • Receives packets from EITHER UART or ESP-NOW.
 *   • Publishes SensorData to BOTH transports at the same time.
 *   • Serves the PING service — replies to any peer, regardless of which
 *     transport the request arrived on.
 *
 * Counterpart test devices:
 *   • uart_example   (ROLE_CLIENT) — communicates via UART
 *   • espnow_example (ROLE_CLIENT) — communicates via ESP-NOW
 *   Both can be active simultaneously; this node handles both.
 *
 * Hardware:
 *   UART TX → GPIO 17    (connect to peer RX)
 *   UART RX → GPIO 16    (connect to peer TX)
 *   GND    → GND          (shared with UART peer)
 *   Wi-Fi  → any ESP-NOW peer on the same channel (default broadcast)
 *
 * Slot configuration used in this example:
 *   Slot 0: UART    rx=true  tx=true   (wired link)
 *   Slot 1: ESP-NOW rx=true  tx=true   (wireless link)
 *
 * Other useful configurations (change SlotConfig below):
 *   • UART RX only + ESP-NOW TX only:
 *       {.rx_enabled=true,  .tx_enabled=false}  for UART
 *       {.rx_enabled=false, .tx_enabled=true}   for ESP-NOW
 *   • Broadcast only via ESP-NOW, receive from both:
 *       {.rx_enabled=true,  .tx_enabled=false}  for UART
 *       {.rx_enabled=true,  .tx_enabled=true}   for ESP-NOW
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_mac.h"

#include "serial_comm/transport/uart_serial_comm.h"
#include "serial_comm/transport/espnow_serial_comm.h"
#include "serial_comm/transport/multi_transport.h"
#include "serial_comm/serial_comm.h"
#include "serial_comm/middleware/serial_comm_manager.h"
#include "serial_comm/middleware/service/serial_comm_service.h"
#include "serial_comm/middleware/topic/serial_comm_topic.h"
#include "serial_comm/common/serial_comm_serializer_base.hpp"

static const char* TAG = "MULTI_EXAMPLE";

// ---------------------------------------------------------------------------
// Application message types
// ---------------------------------------------------------------------------

struct PingRequest  { uint8_t node_id; };
struct PingResponse { uint8_t node_id; bool alive; };
struct SensorData   { float temperature; float humidity; uint32_t timestamp_ms; };

// ---------------------------------------------------------------------------
// Serialiser specialisations
// ---------------------------------------------------------------------------

template<>
struct SerialCommSerializer<PingRequest> {
    static bool serialize(
        const PingRequest& m, uint8_t* buf, size_t sz, size_t& out
    ) {
        if (sz < 1) return false;
        buf[0] = m.node_id; out = 1; return true;
    }
    static bool deserialize(
        const uint8_t* buf, size_t sz, PingRequest& m, size_t* consumed = nullptr
    ) {
        if (sz < 1) return false;
        m.node_id = buf[0];
        if (consumed) *consumed = 1;
        return true;
    }
};

template<>
struct SerialCommSerializer<PingResponse> {
    static bool serialize(
        const PingResponse& m, uint8_t* buf, size_t sz, size_t& out
    ) {
        if (sz < 2) return false;
        buf[0] = m.node_id; buf[1] = m.alive ? 1u : 0u; out = 2; return true;
    }
    static bool deserialize(
        const uint8_t* buf, size_t sz, PingResponse& m, size_t* consumed = nullptr
    ) {
        if (sz < 2) return false;
        m.node_id = buf[0]; m.alive = (buf[1] != 0);
        if (consumed) *consumed = 2;
        return true;
    }
};

template<>
struct SerialCommSerializer<SensorData> {
    static bool serialize(
        const SensorData& m, uint8_t* buf, size_t sz, size_t& out
    ) {
        if (sz < sizeof(SensorData)) return false;
        memcpy(buf, &m, sizeof(SensorData)); out = sizeof(SensorData); return true;
    }
    static bool deserialize(
        const uint8_t* buf, size_t sz, SensorData& m, size_t* consumed = nullptr
    ) {
        if (sz < sizeof(SensorData)) return false;
        memcpy(&m, buf, sizeof(SensorData));
        if (consumed) *consumed = sizeof(SensorData);
        return true;
    }
};

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------

static bool ping_handler(const PingRequest& req, PingResponse& res) {
    ESP_LOGI(TAG, "[PING] From node %u (transport: UART or ESP-NOW — transparent)",
             req.node_id);
    res.node_id = req.node_id;
    res.alive   = true;
    return true;
}

static void sensor_received_cb(const SensorData& msg) {
    ESP_LOGI(TAG, "[SUB] Sensor @ %lu ms: T=%.1f°C H=%.1f%%",
             (unsigned long)msg.timestamp_ms, msg.temperature, msg.humidity);
}

static void transport_event_cb(void* ctx, ISerialCommTransport::Event ev) {
    const char* name = static_cast<const char*>(ctx);
    switch (ev) {
        case ISerialCommTransport::Event::CONNECTED:
            ESP_LOGI(TAG, "[%s] Connected", name); break;
        case ISerialCommTransport::Event::DISCONNECTED:
            ESP_LOGI(TAG, "[%s] Disconnected", name); break;
        case ISerialCommTransport::Event::ERROR:
            ESP_LOGW(TAG, "[%s] Error", name); break;
        case ISerialCommTransport::Event::BUFFER_FULL:
            ESP_LOGW(TAG, "[%s] Buffer full — frame dropped", name); break;
        default: break;
    }
}

// ---------------------------------------------------------------------------
// WiFi initialisation (required for ESP-NOW; no AP needed)
// ---------------------------------------------------------------------------

static void wifi_init_for_espnow(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// ---------------------------------------------------------------------------
// app_main
// ---------------------------------------------------------------------------

extern "C" void app_main(void) {

    ESP_LOGI(TAG, "=== MultiTransport Example (UART + ESP-NOW) ===");

    // -----------------------------------------------------------------------
    // 1. WiFi init  (ESP-NOW needs the Wi-Fi driver even without an AP)
    // -----------------------------------------------------------------------
    wifi_init_for_espnow();

    // -----------------------------------------------------------------------
    // 2a. UART transport
    // -----------------------------------------------------------------------
    UARTTransport::HardwareConfig uart_hw{};
    uart_hw.uart_port = UART_NUM_2;
    uart_hw.tx_pin    = 17;
    uart_hw.rx_pin    = 16;

    static UARTTransport uart_transport(uart_hw);
    uart_transport.set_event_callback(transport_event_cb,
                                      const_cast<char*>("UART"));

    ISerialCommTransport::Config uart_cfg{};
    uart_cfg.timeout_ms = 500;

    if (uart_transport.init(uart_cfg) != errCode::OK) {
        ESP_LOGE(TAG, "UART transport init failed");
        return;
    }
    if (uart_transport.start() != errCode::OK) {
        ESP_LOGE(TAG, "UART transport start failed");
        return;
    }
    ESP_LOGI(TAG, "UART transport ready (UART%d TX=GPIO%d RX=GPIO%d)",
             (int)uart_hw.uart_port, uart_hw.tx_pin, uart_hw.rx_pin);

    // -----------------------------------------------------------------------
    // 2b. ESP-NOW transport
    // -----------------------------------------------------------------------
    ESPNowTransport::HardwareConfig espnow_hw{};
    // tx_peer_mac defaults to broadcast {0xFF,...}

    static ESPNowTransport espnow_transport(espnow_hw);
    espnow_transport.set_event_callback(transport_event_cb,
                                        const_cast<char*>("ESP-NOW"));

    ISerialCommTransport::Config espnow_cfg{};
    espnow_cfg.timeout_ms = 500;

    if (espnow_transport.init(espnow_cfg) != errCode::OK) {
        ESP_LOGE(TAG, "ESP-NOW transport init failed");
        return;
    }
    if (espnow_transport.start() != errCode::OK) {
        ESP_LOGE(TAG, "ESP-NOW transport start failed");
        return;
    }
    ESP_LOGI(TAG, "ESP-NOW transport ready (broadcast)");

    // -----------------------------------------------------------------------
    // 3. MultiTransport — combines both
    // -----------------------------------------------------------------------
    static MultiTransport multi;

    // Slot 0: UART — receive and transmit (SlotConfig defaults: rx=true, tx=true)
    MultiTransport::SlotConfig both_rxtx{};
    multi.add_transport(&uart_transport,   both_rxtx);
    // Slot 1: ESP-NOW — receive and transmit
    multi.add_transport(&espnow_transport, both_rxtx);

    ISerialCommTransport::Config multi_cfg{};
    if (multi.init(multi_cfg) != errCode::OK) {
        ESP_LOGE(TAG, "MultiTransport init failed");
        return;
    }
    if (multi.start() != errCode::OK) {
        ESP_LOGE(TAG, "MultiTransport start failed");
        return;
    }
    ESP_LOGI(TAG, "MultiTransport started (%zu slot(s))", multi.slot_count());

    // -----------------------------------------------------------------------
    // 4. SerialComm — uses MultiTransport transparently
    // -----------------------------------------------------------------------
    static SerialComm comm(&multi);     // <-- only change vs. single transport

    SerialComm::Config sc_cfg{};
    if (comm.init(sc_cfg) != errCode::OK) {
        ESP_LOGE(TAG, "SerialComm init failed");
        return;
    }
    if (comm.start() != errCode::OK) {
        ESP_LOGE(TAG, "SerialComm start failed");
        return;
    }

    // -----------------------------------------------------------------------
    // 5. SerialCommManager — identical to any single-transport example
    // -----------------------------------------------------------------------
    static SerialCommManager mgr(&comm);

    SerialCommManager::Config mgr_cfg{};
    mgr_cfg.service_timeout_ms = 2000;

    if (mgr.init(mgr_cfg) != errCode::OK) {
        ESP_LOGE(TAG, "SerialCommManager init failed");
        return;
    }

    // Service: PING — responds to requests from UART *or* ESP-NOW
    static SerialCommService<PingRequest, PingResponse> ping_svc;
    ping_svc.init(SerialCommCommand::PING, ping_handler);
    mgr.create_service<PingRequest, PingResponse>(&ping_svc);

    // Subscription: SensorData from either transport
    static SerialCommTopic<SensorData> sensor_sub;
    sensor_sub.init(SerialCommCommand::STATUS_TOPIC, sensor_received_cb);
    mgr.create_subscription<SensorData>(&sensor_sub);

    if (mgr.start() != errCode::OK) {
        ESP_LOGE(TAG, "SerialCommManager start failed");
        return;
    }

    ESP_LOGI(TAG, "Ready — publishing SensorData to BOTH transports every 1 s");
    ESP_LOGI(TAG, "         UART peers and ESP-NOW peers receive the same data");
    ESP_LOGI(TAG, "         PING replies go to whichever transport sent the request");

    // -----------------------------------------------------------------------
    // 6. Main loop — publish to all transports simultaneously
    // -----------------------------------------------------------------------
    uint32_t tick = 0;

    while (true) {
        SensorData reading{};
        reading.temperature  = 22.0f + (tick % 10) * 0.3f;
        reading.humidity     = 55.0f + (tick % 20) * 0.5f;
        reading.timestamp_ms = pdTICKS_TO_MS(xTaskGetTickCount());

        // publish() calls MultiTransport::write() → sends via UART AND ESP-NOW
        errCode err = mgr.publish<SensorData>(SerialCommCommand::STATUS_TOPIC, reading);

        if (err == errCode::OK) {
            ESP_LOGD(TAG, "Published SensorData to all transports (tick=%lu)",
                     (unsigned long)tick);
        } else {
            ESP_LOGW(TAG, "Publish failed: %s", err_to_str(err));
        }

        tick++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
