/**
 * @file    main.cpp
 * @brief   SerialComm — ESP-NOW Transport Example
 *
 * Demonstrates the SerialComm middleware stack running over ESP-NOW,
 * the same high-level API (Services / Topics / Actions) as the UART example
 * but using a wireless, infrastructure-free transport.
 *
 * Select which role this board plays by editing the define below:
 *
 *   ROLE_SERVER  – initialises WiFi+ESP-NOW, provides the PING service and
 *                  broadcasts SensorData.
 *   ROLE_CLIENT  – initialises WiFi+ESP-NOW, subscribes to SensorData and
 *                  calls the PING service every 5 seconds.
 *
 * Default is ROLE_SERVER.  Flash two boards with opposite roles and watch
 * the interaction over the serial monitor.
 *
 * No access point required — ESP-NOW is peer-to-peer.
 *
 * By default both roles use the Wi-Fi broadcast MAC (FF:FF:FF:FF:FF:FF) so
 * they can discover each other without hard-coding addresses.  For unicast,
 * set PEER_MAC below to the partner's MAC address.
 */

// ---------------------------------------------------------------------------
// Role selection — edit here or add a proper Kconfig entry
// ---------------------------------------------------------------------------
#define ROLE_SERVER
// #define ROLE_CLIENT

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_mac.h"

#include "serial_comm/transport/espnow_serial_comm.h"
#include "serial_comm/serial_comm.h"
#include "serial_comm/middleware/serial_comm_manager.h"
#include "serial_comm/middleware/service/serial_comm_service.h"
#include "serial_comm/middleware/topic/serial_comm_topic.h"
#include "serial_comm/common/serial_comm_serializer_base.hpp"

static const char* TAG = "ESPNOW_EXAMPLE";

// Optional: set partner MAC for unicast; broadcast by default.
static const uint8_t PEER_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ---------------------------------------------------------------------------
// Application message types (identical to the UART example)
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
// Handler callbacks
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
// WiFi initialisation (ESP-NOW requires WiFi driver; no AP is needed)
// ---------------------------------------------------------------------------

static void wifi_init_for_espnow(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Disable power-save for lower latency in ESP-NOW
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "Local MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// ---------------------------------------------------------------------------
// Transport event callback (optional — logs transport-level events)
// ---------------------------------------------------------------------------

static void transport_event_cb(void* ctx, ISerialCommTransport::Event ev) {
    switch (ev) {
        case ISerialCommTransport::Event::CONNECTED:
            ESP_LOGI(TAG, "ESP-NOW transport connected");
            break;
        case ISerialCommTransport::Event::DISCONNECTED:
            ESP_LOGI(TAG, "ESP-NOW transport disconnected");
            break;
        case ISerialCommTransport::Event::ERROR:
            ESP_LOGW(TAG, "ESP-NOW transport error");
            break;
        case ISerialCommTransport::Event::BUFFER_FULL:
            ESP_LOGW(TAG, "ESP-NOW RX queue full — frame dropped");
            break;
        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// app_main
// ---------------------------------------------------------------------------

extern "C" void app_main(void) {

    // -----------------------------------------------------------------------
    // 1. WiFi (mandatory before ESP-NOW)
    // -----------------------------------------------------------------------
    wifi_init_for_espnow();

    // -----------------------------------------------------------------------
    // 2. ESP-NOW transport
    // -----------------------------------------------------------------------
    ESPNowTransport::HardwareConfig hw{};
    // tx_peer_mac defaults to broadcast {0xFF,...}; override for unicast:
    memcpy(hw.tx_peer_mac, PEER_MAC, 6);
    hw.channel        = 0;             // 0 = follow current channel
    hw.wifi_if        = WIFI_IF_STA;
    hw.use_long_range = false;
    hw.encrypt        = false;

    static ESPNowTransport transport(hw);

    // Register the transport-level event hook before init
    transport.set_event_callback(transport_event_cb, nullptr);

    ISerialCommTransport::Config t_cfg{};
    t_cfg.timeout_ms = 500;  // used as per-fragment TX ack timeout

    if (transport.init(t_cfg) != errCode::OK) {
        ESP_LOGE(TAG, "ESP-NOW transport init failed");
        return;
    }

    // (Optional) add a specific unicast peer after init
    // const uint8_t peer_mac[6] = {0x24, 0xDC, 0xC3, 0xAA, 0xBB, 0xCC};
    // transport.add_peer(peer_mac, /*node_id=*/0x01);

    if (transport.start() != errCode::OK) {
        ESP_LOGE(TAG, "ESP-NOW transport start failed");
        return;
    }

    // -----------------------------------------------------------------------
    // 3. SerialComm core  (identical sequence to the UART example)
    // -----------------------------------------------------------------------
    static SerialComm comm(&transport);

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
    // 4. Middleware manager  (identical to the UART example)
    // -----------------------------------------------------------------------
    static SerialCommManager mgr(&comm);

    SerialCommManager::Config mgr_cfg{};
    mgr_cfg.service_timeout_ms = 2000;

    if (mgr.init(mgr_cfg) != errCode::OK) {
        ESP_LOGE(TAG, "SerialCommManager init failed");
        return;
    }

    // -----------------------------------------------------------------------
    // 5. Services and topics
    // -----------------------------------------------------------------------

#ifdef ROLE_SERVER
    static SerialCommService<PingRequest, PingResponse> ping_svc;
    ping_svc.init(SerialCommCommand::PING, ping_server_handler);
    mgr.create_service<PingRequest, PingResponse>(&ping_svc);

    static SerialCommTopic<SensorData> sensor_sub;
    sensor_sub.init(SerialCommCommand::STATUS_TOPIC, sensor_subscriber_cb);
    mgr.create_subscription<SensorData>(&sensor_sub);

    ESP_LOGI(TAG, "Role: SERVER — PING service active, broadcasting SensorData via ESP-NOW");
#endif

#ifdef ROLE_CLIENT
    static SerialCommTopic<SensorData> sensor_sub;
    sensor_sub.init(SerialCommCommand::STATUS_TOPIC, sensor_subscriber_cb);
    mgr.create_subscription<SensorData>(&sensor_sub);

    ESP_LOGI(TAG, "Role: CLIENT — subscribed to SensorData, calling PING every 5 s via ESP-NOW");
#endif

    if (mgr.start() != errCode::OK) {
        ESP_LOGE(TAG, "SerialCommManager start failed");
        return;
    }

    // -----------------------------------------------------------------------
    // 6. Main loop
    // -----------------------------------------------------------------------
    uint32_t tick = 0;

    while (true) {
#ifdef ROLE_SERVER
        SensorData reading{};
        reading.temperature  = 22.0f + (tick % 10) * 0.3f;
        reading.humidity     = 55.0f + (tick % 20) * 0.5f;
        reading.timestamp_ms = pdTICKS_TO_MS(xTaskGetTickCount());

        errCode pub_err = mgr.publish<SensorData>(SerialCommCommand::STATUS_TOPIC, reading);
        if (pub_err == errCode::OK) {
            ESP_LOGD(TAG, "ESP-NOW: published SensorData tick=%lu T=%.1f",
                     (unsigned long)tick, reading.temperature);
        } else {
            ESP_LOGW(TAG, "Publish failed: %s", err_to_str(pub_err));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
#endif

#ifdef ROLE_CLIENT
        if (tick % 5 == 0) {
            PingRequest req{};
            req.node_id = 0x02;

            PingResponse res{};
            errCode svc_err = mgr.call_service<PingRequest, PingResponse>(
                SerialCommCommand::PING, req, res, /*timeout_ms=*/2000
            );

            if (svc_err == errCode::OK) {
                ESP_LOGI(TAG, "ESP-NOW PING reply: node=%u alive=%s",
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
