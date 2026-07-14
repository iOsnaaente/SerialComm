/**
 * @file    main.cpp
 * @brief   VSSS UDP Bridge — 3-robot wireless control over WiFi + ESP-NOW
 *
 * ── System overview ───────────────────────────────────────────────────
 *
 *   PC/Controller ──UDP──▶ Bridge ──ESP-NOW──▶ Robot 0
 *                  ◀────── Bridge ◀─────────── Robot 0 (telemetry)
 *                                   ──────▶ Robot 1
 *                                   ◀────── Robot 1 (telemetry)
 *                                   ──────▶ Robot 2
 *                                   ◀────── Robot 2 (telemetry)
 *
 * ── Node roles ────────────────────────────────────────────────────────
 *
 *   NODE_TYPE 0  = BRIDGE
 *     • WiFi STA + UDP RX  : receives RobotVelocityCmd from PC (SerialComm framed)
 *     • ESP-NOW TX          : forwards cmd to target robot (unicast per robot_id)
 *     • ESP-NOW RX          : receives RobotTelemetry from robots
 *     • UDP TX              : forwards telemetry back to PC
 *     Uses TWO independent SerialComm stacks (one per transport).
 *
 *   NODE_TYPE 1/2/3  = Robot 0 / 1 / 2
 *     • ESP-NOW RX: receives RobotVelocityCmd; filters by robot_id
 *     • Motor driver: drives motors (stub — replace with real HAL)
 *     • ESP-NOW TX: publishes RobotTelemetry to bridge at 20 Hz
 *
 * ── WiFi / UDP configuration ──────────────────────────────────────────
 *
 *   Update WIFI_SSID, WIFI_PASSWORD, PC_IP, and port numbers below.
 *
 *   The bridge connects to the PC's WiFi network.
 *   PC sends SerialComm-framed packets to BRIDGE_UDP_PORT.
 *   Bridge sends telemetry back to PC_IP:PC_UDP_PORT.
 *
 * ── MAC addresses — HOW TO FILL IN ───────────────────────────────────
 *
 *   1. Flash NODE_TYPE 1 on each robot.  On boot each prints its MAC:
 *        "Robot0 MAC: AA:BB:CC:DD:EE:FF"
 *      Copy each MAC into ROBOT_MACS[] below.
 *
 *   2. Re-flash bridge (NODE_TYPE 0) with the filled-in MACs.
 *
 *   3. Re-flash robots with correct NODE_TYPE and BRIDGE_MAC.
 *
 * ── Python usage ──────────────────────────────────────────────────────
 *
 *   python serial_comm_py/vsss_controller.py \
 *       --transport udp \
 *       --bridge-ip <BRIDGE_IP> \
 *       --bridge-port 4210
 *
 * ── ESP-NOW peer registration — explained ────────────────────────────
 *
 *   add_peer(mac, node_id, channel, encrypt, lmk):
 *     mac      = 6-byte destination MAC
 *     node_id  = logical robot number (stored for routing)
 *     channel  = Wi-Fi channel; 0 = follow current (recommended)
 *     encrypt  = per-peer AES encryption (false here)
 *     lmk      = 16-byte LMK (only when encrypt=true)
 *
 *   set_tx_peer(mac): changes default TX destination without touching
 *   the peer table.  Must protect set_tx_peer() + publish() with a
 *   mutex when multiple tasks may transmit concurrently.
 *
 * ── Generated files used ─────────────────────────────────────────────
 *
 *   user_app/event/RobotVelocityCmd.event  → serial_comm/generated/event/robot_velocity_cmd.hpp
 *   user_app/event/RobotTelemetry.event    → serial_comm/generated/event/robot_telemetry.hpp
 *
 *   Regenerate with:
 *     python user_app/generate.py \
 *         --input  user_app \
 *         --output include/serial_comm/generated
 */

/* ── Includes ──────────────────────────────────────────────────────── */
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_mac.h"

#include "serial_comm/transport/udp_serial_comm.h"
#include "serial_comm/transport/espnow_serial_comm.h"
#include "serial_comm/serial_comm.h"
#include "serial_comm/middleware/serial_comm_manager.h"
#include "serial_comm/middleware/topic/serial_comm_topic.h"

/* Generated message types + serializers */
#include "serial_comm/generated/generated_serializers.hpp"
#include "serial_comm/generated/event/robot_velocity_cmd.hpp"
#include "serial_comm/generated/event/robot_telemetry.hpp"

/* ── Node type ────────────────────────────────────────────────────────
 *   0 = Bridge (UDP ↔ ESP-NOW relay)
 *   1 = Robot 0
 *   2 = Robot 1
 *   3 = Robot 2                                                        */
#define NODE_TYPE 0

/* ── WiFi credentials (bridge only) ──────────────────────────────── */
#define WIFI_SSID     "your_ssid_here"
#define WIFI_PASSWORD "your_password_here"

/* ── PC UDP endpoint ──────────────────────────────────────────────── */
#define PC_IP         "192.168.1.100"
#define PC_UDP_PORT   4211

/* ── Bridge UDP listen port ──────────────────────────────────────── */
#define BRIDGE_UDP_PORT 4210

/* ── Robot MACs — fill in after first boot of each robot node ─────── */
static const uint8_t ROBOT_MACS[3][6] = {
    {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x01},  /* Robot 0 — replace */
    {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x02},  /* Robot 1 — replace */
    {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x03},  /* Robot 2 — replace */
};

/* ── MAC of bridge node (robot nodes send telemetry here) ───────── */
static const uint8_t BRIDGE_MAC[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x00};

#define NUM_ROBOTS   3
#define ROBOT_ID_ALL 0xFF   /* robot_id value meaning "broadcast to all" */

/* ── Derived robot ID (robot nodes only) ─────────────────────────── */
#if NODE_TYPE >= 1
#define MY_ROBOT_ID  (NODE_TYPE - 1)   /* 0, 1 or 2 */
#endif

static const char* TAG = "VSSS_UDP";

/* ── Command IDs from IDL @id metadata ─────────────────────────────── */
static constexpr SerialCommCommand VEL_CMD =
    static_cast<SerialCommCommand>(ROBOTVELOCITYCMD_ID);  /* 0x10 */
static constexpr SerialCommCommand TEL_CMD =
    static_cast<SerialCommCommand>(ROBOTTELEMETRY_ID);    /* 0x11 */


/* ============================================================================
 * BRIDGE NODE  (NODE_TYPE == 0)
 * ============================================================================
 *
 *  Two independent SerialComm stacks so each transport is fully decoupled:
 *
 *   udp_comm / udp_mgr       — PC ↔ Bridge (UDP)
 *   espnow_comm / espnow_mgr — Bridge ↔ Robots (ESP-NOW)
 *
 *  Relay:
 *   on_velocity_cmd_udp    : UDP RX → route to robot N via ESP-NOW TX
 *   on_telemetry_espnow    : ESP-NOW RX → forward to PC via UDP TX
 * ============================================================================ */
#if NODE_TYPE == 0

static constexpr int WIFI_CONNECTED_BIT = BIT0;
static EventGroupHandle_t s_wifi_event_group = nullptr;

/* Globals for cross-stack relay */
static ESPNowTransport*   g_espnow     = nullptr;
static SerialCommManager* g_espnow_mgr = nullptr;
static SerialCommManager* g_udp_mgr    = nullptr;

/* Protects set_tx_peer() + publish() pair — must be atomic */
static SemaphoreHandle_t  g_espnow_tx_mutex = nullptr;

/* ── WiFi event handler ──────────────────────────────────────────────── */
static void wifi_event_handler(
    void* arg, esp_event_base_t base,
    int32_t id, void* data
) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGW(TAG, "WiFi disconnected — reconnecting...");
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* ev = (ip_event_got_ip_t*)data;
        ESP_LOGI(TAG, "WiFi IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* ── Relay: PC → Bridge (UDP RX) → Robot N (ESP-NOW TX) ──────────────── */
static void on_velocity_cmd_udp(const RobotVelocityCmd& cmd) {

    uint8_t target = cmd.robot_id;

    if (target >= NUM_ROBOTS && target != ROBOT_ID_ALL) {
        ESP_LOGW(TAG, "[Bridge] Unknown robot_id=0x%02X — ignored", target);
        return;
    }

    xSemaphoreTake(g_espnow_tx_mutex, portMAX_DELAY);

    if (target == ROBOT_ID_ALL) {
        for (int i = 0; i < NUM_ROBOTS; i++) {
            g_espnow->set_tx_peer(ROBOT_MACS[i]);
            g_espnow_mgr->publish<RobotVelocityCmd>(VEL_CMD, cmd);
        }
        ESP_LOGD(TAG, "[Bridge] VelCmd → ALL  vl=%.2f vr=%.2f",
                 cmd.vl, cmd.vr);
    } else {
        g_espnow->set_tx_peer(ROBOT_MACS[target]);
        g_espnow_mgr->publish<RobotVelocityCmd>(VEL_CMD, cmd);
        ESP_LOGI(TAG, "[Bridge] VelCmd → Robot%u  vl=%.2f vr=%.2f",
                 target, cmd.vl, cmd.vr);
    }

    xSemaphoreGive(g_espnow_tx_mutex);
}

/* ── Relay: Robot N (ESP-NOW RX) → Bridge → PC (UDP TX) ─────────────── */
static void on_telemetry_espnow(const RobotTelemetry& tel) {

    ESP_LOGD(TAG, "[Bridge] Telemetry from Robot%u  ax=%.2f ay=%.2f gz=%.2f  "
                  "vl=%.2f vr=%.2f  ball=%d",
             tel.robot_id, tel.ax, tel.ay, tel.gz,
             tel.vl_measured, tel.vr_measured, tel.ball_detected);

    errCode err = g_udp_mgr->publish<RobotTelemetry>(TEL_CMD, tel);
    if (err != errCode::OK) {
        ESP_LOGW(TAG, "[Bridge] Telemetry UDP forward failed: %s",
                 err_to_str(err));
    }
}

/* ── Bridge app_main ─────────────────────────────────────────────────── */
extern "C" void app_main(void) {

    ESP_LOGI(TAG, "=== VSSS UDP Bridge (NODE_TYPE=0) ===");

    /* NVS and network stack — initialized once */
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* ── WiFi STA init ──────────────────────────────────────────────── */
    s_wifi_event_group = xEventGroupCreate();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    esp_event_handler_instance_t h_any, h_got;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, nullptr, &h_any));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, nullptr, &h_got));

    wifi_config_t wifi_cfg = {};
    strncpy((char*)wifi_cfg.sta.ssid,     WIFI_SSID,
            sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char*)wifi_cfg.sta.password, WIFI_PASSWORD,
            sizeof(wifi_cfg.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    esp_wifi_connect();

    /* Block until we have an IP — UDP transport needs it */
    xEventGroupWaitBits(
        s_wifi_event_group, WIFI_CONNECTED_BIT,
        pdFALSE, pdTRUE, portMAX_DELAY
    );

    g_espnow_tx_mutex = xSemaphoreCreateMutex();

    /* ── UDP transport — PC ↔ Bridge ───────────────────────────────── */
    UDPTransport::HardwareConfig udp_hw{};
    udp_hw.local_port  = BRIDGE_UDP_PORT;
    udp_hw.remote_port = PC_UDP_PORT;
    strncpy(udp_hw.remote_ip, PC_IP, sizeof(udp_hw.remote_ip) - 1);
    udp_hw.reuse_addr  = true;
    udp_hw.broadcast   = false;

    static UDPTransport udp(udp_hw);

    if (udp.init() != errCode::OK) { ESP_LOGE(TAG, "UDP init fail");  return; }
    if (udp.start()       != errCode::OK) { ESP_LOGE(TAG, "UDP start fail"); return; }

    char local_ip[16] = {};
    udp.get_local_ip(local_ip, sizeof(local_ip));
    ESP_LOGI(TAG, "UDP ready  listen=%u  ip=%s  → PC %s:%u",
             BRIDGE_UDP_PORT, local_ip, PC_IP, PC_UDP_PORT);

    static SerialComm udp_comm(&udp);
    udp_comm.init();
    udp_comm.start();

    static SerialCommManager udp_mgr(&udp_comm);
    g_udp_mgr = &udp_mgr;
    udp_mgr.init();

    static SerialCommTopic<RobotVelocityCmd> vel_sub;
    vel_sub.init(VEL_CMD, on_velocity_cmd_udp);
    udp_mgr.create_subscription<RobotVelocityCmd>(&vel_sub);

    udp_mgr.start();

    /* ── ESP-NOW transport — Bridge ↔ Robots ───────────────────────── */
    /* WiFi is already up (STA mode); ESP-NOW shares the same interface  */
    ESPNowTransport::HardwareConfig espnow_hw{};

    static ESPNowTransport espnow(espnow_hw);
    g_espnow = &espnow;

    if (espnow.init() != errCode::OK) { ESP_LOGE(TAG, "ESP-NOW init fail"); return; }

    /* Register every robot as a peer before attempting unicast TX */
    for (int i = 0; i < NUM_ROBOTS; i++) {
        errCode err = espnow.add_peer(
            ROBOT_MACS[i],
            static_cast<uint8_t>(i),
            0,      /* channel: follow current */
            false   /* no encryption */
        );
        if (err != errCode::OK) {
            ESP_LOGW(TAG, "add_peer Robot%d failed: %s", i, err_to_str(err));
        }
    }

    if (espnow.start() != errCode::OK) { ESP_LOGE(TAG, "ESP-NOW start fail"); return; }

    static SerialComm espnow_comm(&espnow);
    espnow_comm.init();
    espnow_comm.start();

    static SerialCommManager espnow_mgr(&espnow_comm);
    g_espnow_mgr = &espnow_mgr;
    espnow_mgr.init();

    static SerialCommTopic<RobotTelemetry> tel_sub;
    tel_sub.init(TEL_CMD, on_telemetry_espnow);
    espnow_mgr.create_subscription<RobotTelemetry>(&tel_sub);

    espnow_mgr.start();

    ESP_LOGI(TAG, "Bridge ready — waiting for UDP commands from PC");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

#endif  /* NODE_TYPE == 0 */


/* ============================================================================
 * ROBOT NODE  (NODE_TYPE 1 / 2 / 3)
 * ============================================================================ */
#if NODE_TYPE >= 1 && NODE_TYPE <= 3

static SerialCommManager* g_mgr = nullptr;

/* Drive motors — replace with real HAL */
static void drive_motors(float vl, float vr) {
    ESP_LOGI(TAG, "Robot%u  MOTOR  vl=%.2f  vr=%.2f", MY_ROBOT_ID, vl, vr);
}

static void on_velocity_cmd_espnow(const RobotVelocityCmd& cmd) {
    if (cmd.robot_id != MY_ROBOT_ID && cmd.robot_id != ROBOT_ID_ALL) {
        return;
    }
    drive_motors(cmd.vl, cmd.vr);
}

static void telemetry_task(void* arg) {
    while (true) {
        RobotTelemetry tel{};
        tel.robot_id      = MY_ROBOT_ID;
        tel.ax            = 0.0f;   /* replace with real sensor reads */
        tel.ay            = 0.0f;
        tel.gz            = 0.0f;
        tel.vl_measured   = 0.0f;
        tel.vr_measured   = 0.0f;
        tel.ball_detected = false;

        if (g_mgr) {
            g_mgr->publish<RobotTelemetry>(TEL_CMD, tel);
        }
        vTaskDelay(pdMS_TO_TICKS(50));  /* 20 Hz */
    }
}

extern "C" void app_main(void) {

    ESP_LOGI(TAG, "=== VSSS Robot%u (NODE_TYPE=%d) ===", MY_ROBOT_ID, NODE_TYPE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Minimal WiFi init — ESP-NOW requires the WiFi driver */
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    /* Print MAC so user can populate ROBOT_MACS[] in bridge firmware */
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "Robot%u MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             MY_ROBOT_ID, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    /* ── ESP-NOW transport ─────────────────────────────────────────── */
    ESPNowTransport::HardwareConfig espnow_hw{};

    static ESPNowTransport espnow(espnow_hw);

    if (espnow.init() != errCode::OK) { ESP_LOGE(TAG, "ESP-NOW init fail"); return; }

    /* Register bridge as peer for telemetry TX */
    espnow.add_peer(BRIDGE_MAC, 0 /* node_id */, 0 /* channel */, false);
    espnow.set_tx_peer(BRIDGE_MAC);

    if (espnow.start() != errCode::OK) { ESP_LOGE(TAG, "ESP-NOW start fail"); return; }

    static SerialComm comm(&espnow);
    comm.init();
    comm.start();

    static SerialCommManager mgr(&comm);
    g_mgr = &mgr;
    mgr.init();

    static SerialCommTopic<RobotVelocityCmd> vel_sub;
    vel_sub.init(VEL_CMD, on_velocity_cmd_espnow);
    mgr.create_subscription<RobotVelocityCmd>(&vel_sub);

    mgr.start();
    ESP_LOGI(TAG, "Robot%u ready", MY_ROBOT_ID);

    xTaskCreate(telemetry_task, "telem_task", 4096, nullptr, 4, nullptr);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

#endif  /* NODE_TYPE >= 1 */
