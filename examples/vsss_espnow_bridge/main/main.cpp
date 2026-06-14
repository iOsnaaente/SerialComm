/**
 * @file    main.cpp
 * @brief   VSSS ESP-NOW Bridge — 3-robot wireless control system
 *
 * ── System overview ───────────────────────────────────────────────────
 *
 *   PC/Controller ──UART──▶ Bridge ──ESP-NOW──▶ Robot 0
 *                  ◀─────── Bridge ◀─────────── Robot 0 (telemetry)
 *                                     ──────▶ Robot 1
 *                                     ◀────── Robot 1 (telemetry)
 *                                     ──────▶ Robot 2
 *                                     ◀────── Robot 2 (telemetry)
 *
 * ── Node roles ────────────────────────────────────────────────────────
 *
 *   NODE_TYPE 0  = BRIDGE
 *     • UART RX : receives RobotVelocityCmd events from PC
 *     • ESP-NOW TX: forwards each cmd to the target robot (unicast)
 *     • ESP-NOW RX: receives RobotTelemetry from robots
 *     • UART TX : forwards telemetry to PC
 *     Uses TWO independent SerialComm stacks (one per transport).
 *
 *   NODE_TYPE 1/2/3  = Robot 0 / 1 / 2
 *     • ESP-NOW RX: receives RobotVelocityCmd from bridge
 *     • Filters by robot_id; drives motors (stub here)
 *     • Reads accelerometer (stub — replace with real sensor call)
 *     • ESP-NOW TX: publishes RobotTelemetry to bridge at 20 Hz
 *
 * ── MAC addresses — HOW TO FILL IN ───────────────────────────────────
 *
 *   1. Flash NODE_TYPE 0 on the bridge ESP32 and open the serial monitor.
 *      On boot it prints:  "Bridge MAC: AA:BB:CC:DD:EE:FF"
 *      Copy those bytes into BRIDGE_MAC below.
 *
 *   2. Flash NODE_TYPE 1 on robot 0.  On boot it prints its own MAC.
 *      Copy into ROBOT_MACS[0].  Repeat for robots 1 and 2.
 *
 *   3. Re-flash all devices with the filled-in addresses and the
 *      correct NODE_TYPE.
 *
 * ── ESP-NOW peer registration — explained ────────────────────────────
 *
 *   add_peer(mac, node_id, channel, encrypt, lmk):
 *     mac      = 6-byte destination MAC
 *     node_id  = logical robot number (stored locally for V2 routing)
 *     channel  = Wi-Fi channel; 0 = follow current channel (recommended)
 *     encrypt  = per-peer link-layer AES encryption (false here)
 *     lmk      = 16-byte local master key (only when encrypt=true)
 *
 *   Why we must call add_peer() before sending unicast:
 *     ESP-NOW internally maintains a peer table.  esp_now_send() looks
 *     up the destination MAC in that table to find channel and ifidx.
 *     If the MAC is not in the table the call fails with ESP_ERR_ESPNOW_NOT_FOUND.
 *     Broadcast (0xFF:FF:FF:FF:FF:FF) is auto-registered by init().
 *
 *   set_tx_peer(mac):
 *     Changes the default TX destination without touching the peer table.
 *     Use this to switch between already-registered peers at runtime
 *     (e.g., routing different packets to different robots).
 *
 * ── Command IDs ───────────────────────────────────────────────────────
 *
 *   The IDs come from @id metadata in the IDL files:
 *     RobotVelocityCmd  @id 0x10  → ROBOTVELOCITYCMD_ID
 *     RobotTelemetry    @id 0x11  → ROBOTTELEMETRY_ID
 *
 *   Cast to SerialCommCommand to use with the middleware.
 *
 * ── Generated files used ─────────────────────────────────────────────
 *
 *   user_app/event/RobotVelocityCmd.event  → serial_comm/generated/event/robot_velocity_cmd.hpp
 *   user_app/event/RobotTelemetry.event   → serial_comm/generated/event/robot_telemetry.hpp
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
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_mac.h"

#include "serial_comm/transport/uart_serial_comm.h"
#include "serial_comm/transport/espnow_serial_comm.h"
#include "serial_comm/serial_comm.h"
#include "serial_comm/middleware/serial_comm_manager.h"
#include "serial_comm/middleware/topic/serial_comm_topic.h"

/* Generated message types + serializers */
#include "serial_comm/generated/generated_serializers.hpp"
#include "serial_comm/generated/event/robot_velocity_cmd.hpp"
#include "serial_comm/generated/event/robot_telemetry.hpp"

/* ── Node type ────────────────────────────────────────────────────────
 *   0 = Bridge (UART ↔ ESP-NOW relay)
 *   1 = Robot 0
 *   2 = Robot 1
 *   3 = Robot 2                                                        */
#define NODE_TYPE 0

/* ── MAC addresses ────────────────────────────────────────────────────
 * Fill in from the boot log of each device (see instructions above). */
static const uint8_t BRIDGE_MAC[6]      = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x00};
static const uint8_t ROBOT_MACS[3][6]   = {
    {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x01},  /* Robot 0 */
    {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x02},  /* Robot 1 */
    {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x03},  /* Robot 2 */
};

#define NUM_ROBOTS   3
#define ROBOT_ID_ALL 0xFF   /* robot_id value meaning "send to all robots" */

/* ── Derived robot ID (robot nodes only) ─────────────────────────────*/
#if NODE_TYPE >= 1
#define MY_ROBOT_ID  (NODE_TYPE - 1)   /* 0, 1 or 2 */
#endif

static const char* TAG = "VSSS";

/* ── Command IDs from IDL @id metadata ─────────────────────────────── */
static constexpr SerialCommCommand VEL_CMD =
    static_cast<SerialCommCommand>(ROBOTVELOCITYCMD_ID);  /* 0x10 */
static constexpr SerialCommCommand TEL_CMD =
    static_cast<SerialCommCommand>(ROBOTTELEMETRY_ID);    /* 0x11 */

/* ── WiFi init (required before ESPNowTransport::init()) ───────────── */
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


/* ============================================================================
 * BRIDGE NODE  (NODE_TYPE == 0)
 * ============================================================================
 *
 *  Two independent SerialComm stacks so each transport is fully decoupled:
 *
 *   uart_comm / uart_mgr   — PC ↔ Bridge (UART)
 *   espnow_comm / espnow_mgr — Bridge ↔ Robots (ESP-NOW)
 *
 *  Relay:
 *   on_velocity_cmd_uart  : UART RX → route to robot N via ESP-NOW TX
 *   on_telemetry_espnow   : ESP-NOW RX → forward to PC via UART TX
 * ============================================================================ */
#if NODE_TYPE == 0

/* Globals for cross-stack relay */
static ESPNowTransport*    g_espnow     = nullptr;
static SerialCommManager*  g_espnow_mgr = nullptr;
static SerialCommManager*  g_uart_mgr   = nullptr;

/* Mutex: only one ESP-NOW TX at a time (set_tx_peer + write must be atomic) */
static SemaphoreHandle_t   g_espnow_tx_mutex = nullptr;

/* ── Relay: PC → Bridge (UART RX) → Robot N (ESP-NOW TX) ───────────── */
static void on_velocity_cmd_uart(const RobotVelocityCmd& cmd) {

    uint8_t target = cmd.robot_id;

    if (target >= NUM_ROBOTS && target != ROBOT_ID_ALL) {
        ESP_LOGW(TAG, "[Bridge] Unknown robot_id=0x%02X — ignored", target);
        return;
    }

    xSemaphoreTake(g_espnow_tx_mutex, portMAX_DELAY);

    if (target == ROBOT_ID_ALL) {
        /* Send the same command to every registered robot */
        for (int i = 0; i < NUM_ROBOTS; i++) {
            g_espnow->set_tx_peer(ROBOT_MACS[i]);
            g_espnow_mgr->publish<RobotVelocityCmd>(VEL_CMD, cmd);
        }
        ESP_LOGD(TAG, "[Bridge] VelCmd → ALL  vl=%.2f vr=%.2f",
                 cmd.vl, cmd.vr);
    } else {
        /* Unicast to the specific robot */
        g_espnow->set_tx_peer(ROBOT_MACS[target]);
        g_espnow_mgr->publish<RobotVelocityCmd>(VEL_CMD, cmd);
        ESP_LOGI(TAG, "[Bridge] VelCmd → Robot%u  vl=%.2f vr=%.2f",
                 target, cmd.vl, cmd.vr);
    }

    xSemaphoreGive(g_espnow_tx_mutex);
}

/* ── Relay: Robot N (ESP-NOW RX) → Bridge → PC (UART TX) ───────────── */
static void on_telemetry_espnow(const RobotTelemetry& tel) {

    ESP_LOGD(TAG, "[Bridge] Telemetry from Robot%u  ax=%.2f ay=%.2f gz=%.2f  "
                  "vl=%.2f vr=%.2f  ball=%d",
             tel.robot_id, tel.ax, tel.ay, tel.gz,
             tel.vl_measured, tel.vr_measured, tel.ball_detected);

    /* Forward to PC — uart_mgr.publish() calls uart_comm.write() → UART TX */
    errCode err = g_uart_mgr->publish<RobotTelemetry>(TEL_CMD, tel);
    if (err != errCode::OK) {
        ESP_LOGW(TAG, "[Bridge] Telemetry UART forward failed: %s",
                 err_to_str(err));
    }
}


/* ── Bridge app_main ─────────────────────────────────────────────────── */
extern "C" void app_main(void) {

    ESP_LOGI(TAG, "=== VSSS Bridge (NODE_TYPE=0) ===");
    wifi_init_for_espnow();

    g_espnow_tx_mutex = xSemaphoreCreateMutex();

    /* ── UART transport ─────────────────────────────────────────────── */
    UARTTransport::HardwareConfig uart_hw{};
    uart_hw.uart_port = UART_NUM_2;
    uart_hw.tx_pin    = 17;
    uart_hw.rx_pin    = 16;

    static UARTTransport uart(uart_hw);
    ISerialCommTransport::Config uart_cfg{};
    uart_cfg.timeout_ms = 500;

    if (uart.init(uart_cfg)  != errCode::OK) { ESP_LOGE(TAG, "UART init fail");  return; }
    if (uart.start()         != errCode::OK) { ESP_LOGE(TAG, "UART start fail"); return; }
    ESP_LOGI(TAG, "UART ready (RX=GPIO%d  TX=GPIO%d)", uart_hw.rx_pin, uart_hw.tx_pin);

    /* ── ESP-NOW transport ──────────────────────────────────────────── */
    /* Default TX peer is broadcast; we override per-message with set_tx_peer() */
    ESPNowTransport::HardwareConfig espnow_hw{};

    static ESPNowTransport espnow(espnow_hw);
    g_espnow = &espnow;

    ISerialCommTransport::Config espnow_cfg{};
    espnow_cfg.timeout_ms = 1000;

    if (espnow.init(espnow_cfg) != errCode::OK) { ESP_LOGE(TAG, "ESP-NOW init fail");  return; }

    /* ── Register every robot as a peer ─────────────────────────────
     *
     *  Why: esp_now_send() requires the destination MAC to be in the
     *  peer table, otherwise it returns ESP_ERR_ESPNOW_NOT_FOUND.
     *
     *  We register all 3 robots here at startup so that set_tx_peer()
     *  + publish() can switch between them freely without re-registering.
     *
     *  add_peer(mac, node_id, channel=0, encrypt=false, lmk=nullptr)
     *    node_id is our logical label (0/1/2) — used for future routing.
     *    channel=0 means "follow current Wi-Fi channel" (safest default).  */
    for (int i = 0; i < NUM_ROBOTS; i++) {
        errCode err = espnow.add_peer(
            ROBOT_MACS[i],
            static_cast<uint8_t>(i),  /* node_id = robot index */
            0,                         /* channel: follow current */
            false                      /* no encryption */
        );
        if (err == errCode::OK) {
            ESP_LOGI(TAG, "Peer registered: Robot%d  %02X:%02X:%02X:%02X:%02X:%02X",
                     i,
                     ROBOT_MACS[i][0], ROBOT_MACS[i][1], ROBOT_MACS[i][2],
                     ROBOT_MACS[i][3], ROBOT_MACS[i][4], ROBOT_MACS[i][5]);
        } else {
            ESP_LOGW(TAG, "Peer Robot%d registration failed: %s",
                     i, err_to_str(err));
        }
    }

    if (espnow.start() != errCode::OK) { ESP_LOGE(TAG, "ESP-NOW start fail"); return; }
    ESP_LOGI(TAG, "ESP-NOW ready (%d peers registered)", NUM_ROBOTS);

    /* ── SerialComm stack A: UART (PC ↔ Bridge) ─────────────────────── */
    static SerialComm uart_comm(&uart);
    SerialComm::Config sc_cfg{};
    if (uart_comm.init(sc_cfg)  != errCode::OK) { ESP_LOGE(TAG, "uart_comm init fail");  return; }
    if (uart_comm.start()       != errCode::OK) { ESP_LOGE(TAG, "uart_comm start fail"); return; }

    static SerialCommManager uart_mgr(&uart_comm);
    g_uart_mgr = &uart_mgr;
    SerialCommManager::Config mgr_cfg{};
    if (uart_mgr.init(mgr_cfg) != errCode::OK) { ESP_LOGE(TAG, "uart_mgr init fail"); return; }

    /* Subscribe to velocity commands arriving from PC via UART */
    static SerialCommTopic<RobotVelocityCmd> uart_vel_sub;
    uart_vel_sub.init(VEL_CMD, on_velocity_cmd_uart);
    uart_mgr.create_subscription<RobotVelocityCmd>(&uart_vel_sub);

    if (uart_mgr.start() != errCode::OK) { ESP_LOGE(TAG, "uart_mgr start fail"); return; }

    /* ── SerialComm stack B: ESP-NOW (Bridge ↔ Robots) ──────────────── */
    static SerialComm espnow_comm(&espnow);
    if (espnow_comm.init(sc_cfg)  != errCode::OK) { ESP_LOGE(TAG, "espnow_comm init fail");  return; }
    if (espnow_comm.start()       != errCode::OK) { ESP_LOGE(TAG, "espnow_comm start fail"); return; }

    static SerialCommManager espnow_mgr(&espnow_comm);
    g_espnow_mgr = &espnow_mgr;
    if (espnow_mgr.init(mgr_cfg) != errCode::OK) { ESP_LOGE(TAG, "espnow_mgr init fail"); return; }

    /* Subscribe to telemetry arriving from robots via ESP-NOW */
    static SerialCommTopic<RobotTelemetry> espnow_tel_sub;
    espnow_tel_sub.init(TEL_CMD, on_telemetry_espnow);
    espnow_mgr.create_subscription<RobotTelemetry>(&espnow_tel_sub);

    if (espnow_mgr.start() != errCode::OK) { ESP_LOGE(TAG, "espnow_mgr start fail"); return; }

    ESP_LOGI(TAG, "Bridge ready:");
    ESP_LOGI(TAG, "  PC ──UART──▶  RobotVelocityCmd → route to Robot 0/1/2 via ESP-NOW");
    ESP_LOGI(TAG, "  Robot N ──ESP-NOW──▶  RobotTelemetry → forward to PC via UART");
    ESP_LOGI(TAG, "  (tip: robot_id=0xFF in VelocityCmd sends to all %d robots)", NUM_ROBOTS);

    /* app_main returns — callbacks and SerialComm tasks handle everything */
}

/* ============================================================================
 * ROBOT NODE  (NODE_TYPE == 1/2/3)
 * ============================================================================
 *
 *  Single ESP-NOW SerialComm stack.
 *
 *  Receives RobotVelocityCmd from bridge, filters by MY_ROBOT_ID,
 *  drives motors (stub — replace with actual motor driver calls).
 *
 *  Reads accelerometer (stub — replace with I2C/SPI sensor reads) and
 *  publishes RobotTelemetry to bridge at 20 Hz.
 * ============================================================================ */
#elif NODE_TYPE >= 1 && NODE_TYPE <= 3

/* ── Motor driver stub ───────────────────────────────────────────────── */
static void drive_motors(float vl, float vr) {
    /* TODO: write vl/vr to your motor driver (PWM, MCPWM, etc.) */
    (void)vl; (void)vr;
}

/* ── Accelerometer stub ──────────────────────────────────────────────── */
static void read_accel(float* ax, float* ay, float* gz) {
    /* TODO: read from your IMU over I2C/SPI (MPU6050, LSM6DS, etc.) */
    *ax = 0.0f;
    *ay = 0.0f;
    *gz = 0.0f;
}

/* ── Wheel encoder stub ──────────────────────────────────────────────── */
static void read_encoders(float* vl, float* vr) {
    /* TODO: read from encoder counters / pulse counters */
    *vl = 0.0f;
    *vr = 0.0f;
}

/* ── Ball detection stub ─────────────────────────────────────────────── */
static bool detect_ball(void) {
    /* TODO: IR sensor, camera, or other ball-detection logic */
    return false;
}

/* ── Velocity command callback (from bridge, via ESP-NOW) ───────────── */
static void on_velocity_cmd(const RobotVelocityCmd& cmd) {

    /* Filter: only act on commands addressed to this robot or broadcast */
    if (cmd.robot_id != MY_ROBOT_ID && cmd.robot_id != ROBOT_ID_ALL) {
        return;
    }

    ESP_LOGI(TAG, "[Robot%d] VelCmd  vl=%.2f  vr=%.2f",
             MY_ROBOT_ID, cmd.vl, cmd.vr);

    drive_motors(cmd.vl, cmd.vr);
}


/* ── Robot app_main ──────────────────────────────────────────────────── */
extern "C" void app_main(void) {

    ESP_LOGI(TAG, "=== VSSS Robot %d (NODE_TYPE=%d) ===", MY_ROBOT_ID, NODE_TYPE);
    wifi_init_for_espnow();

    /* ── ESP-NOW transport ──────────────────────────────────────────── */
    /* Set default TX destination to the bridge */
    ESPNowTransport::HardwareConfig espnow_hw{};
    memcpy(espnow_hw.tx_peer_mac, BRIDGE_MAC, 6);

    static ESPNowTransport espnow(espnow_hw);

    ISerialCommTransport::Config espnow_cfg{};
    espnow_cfg.timeout_ms = 500;

    if (espnow.init(espnow_cfg) != errCode::OK) { ESP_LOGE(TAG, "ESP-NOW init fail"); return; }

    /* ── Register bridge as a peer ───────────────────────────────────
     *
     *  The bridge MAC was already set in tx_peer_mac and auto-registered
     *  by init().  This explicit add_peer() call is included here to
     *  show the pattern clearly and to attach a logical node_id.
     *
     *  If you want to add more recipients (e.g., a secondary base station)
     *  just call add_peer() again with the new MAC and then set_tx_peer()
     *  before publishing.                                                */
    errCode peer_err = espnow.add_peer(
        BRIDGE_MAC,
        0x00,    /* node_id: bridge is node 0 in our scheme */
        0,       /* channel: follow current */
        false    /* no encryption */
    );

    if (peer_err == errCode::OK) {
        ESP_LOGI(TAG, "Bridge registered as peer  %02X:%02X:%02X:%02X:%02X:%02X",
                 BRIDGE_MAC[0], BRIDGE_MAC[1], BRIDGE_MAC[2],
                 BRIDGE_MAC[3], BRIDGE_MAC[4], BRIDGE_MAC[5]);
    } else {
        /* Not fatal: init() already registered tx_peer_mac, this only
         * updates the node_id metadata in the internal peer table.    */
        ESP_LOGW(TAG, "Bridge add_peer: %s (non-fatal)", err_to_str(peer_err));
    }

    if (espnow.start() != errCode::OK) { ESP_LOGE(TAG, "ESP-NOW start fail"); return; }

    /* ── SerialComm + Manager ───────────────────────────────────────── */
    static SerialComm comm(&espnow);
    SerialComm::Config sc_cfg{};
    if (comm.init(sc_cfg)  != errCode::OK) { ESP_LOGE(TAG, "comm init fail");  return; }
    if (comm.start()       != errCode::OK) { ESP_LOGE(TAG, "comm start fail"); return; }

    static SerialCommManager mgr(&comm);
    SerialCommManager::Config mgr_cfg{};
    if (mgr.init(mgr_cfg) != errCode::OK) { ESP_LOGE(TAG, "mgr init fail"); return; }

    /* Subscribe to velocity commands from the bridge */
    static SerialCommTopic<RobotVelocityCmd> vel_sub;
    vel_sub.init(VEL_CMD, on_velocity_cmd);
    mgr.create_subscription<RobotVelocityCmd>(&vel_sub);

    if (mgr.start() != errCode::OK) { ESP_LOGE(TAG, "mgr start fail"); return; }

    ESP_LOGI(TAG, "Robot %d ready — waiting for velocity commands from bridge",
             MY_ROBOT_ID);

    /* ── Main loop: read sensors and publish telemetry at 20 Hz ─────── */
    while (true) {

        RobotTelemetry tel{};
        tel.robot_id = MY_ROBOT_ID;

        read_accel(&tel.ax, &tel.ay, &tel.gz);
        read_encoders(&tel.vl_measured, &tel.vr_measured);
        tel.ball_detected = detect_ball();

        errCode err = mgr.publish<RobotTelemetry>(TEL_CMD, tel);

        if (err == errCode::OK) {
            ESP_LOGD(TAG, "[Robot%d] Telemetry sent  ax=%.2f ay=%.2f gz=%.2f  "
                          "vl=%.2f vr=%.2f  ball=%d",
                     MY_ROBOT_ID,
                     tel.ax, tel.ay, tel.gz,
                     tel.vl_measured, tel.vr_measured,
                     (int)tel.ball_detected);
        } else {
            ESP_LOGW(TAG, "[Robot%d] Telemetry send failed: %s",
                     MY_ROBOT_ID, err_to_str(err));
        }

        vTaskDelay(pdMS_TO_TICKS(50));  /* 20 Hz telemetry */
    }
}

#else
#error "NODE_TYPE must be 0 (Bridge) or 1/2/3 (Robot 0/1/2)"
#endif
