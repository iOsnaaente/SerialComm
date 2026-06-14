/**
 * @file    espnow_serial_comm.h
 * @brief   ESP-NOW transport implementation for serial_comm middleware
 * @details Implements ISerialCommTransport using the ESP-NOW protocol,
 *          providing a wireless alternative to UART transport.
 *
 *          ESP-NOW features used:
 *            - Point-to-point and broadcast delivery
 *            - Transparent fragmentation for large SerialComm packets
 *              (ESP-NOW hard limit = 250 bytes per frame; SerialComm
 *               packets can be up to ~1038 bytes with default config)
 *            - Optional link-layer encryption (PMK + per-peer LMK)
 *            - Peer management with logical node-ID mapping
 *              (ready for Protocol V2 addressing when implemented)
 *
 *          Prerequisites (caller's responsibility):
 *            1. Call esp_netif_init() and esp_event_loop_create_default()
 *            2. Initialize NVS (nvs_flash_init())
 *            3. Call esp_wifi_init() / esp_wifi_set_mode() / esp_wifi_start()
 *            4. Optionally set a fixed channel with esp_wifi_set_channel()
 *
 * @note    Only one ESPNowTransport instance may exist at a time because
 *          the ESP-NOW receive and send callbacks are global.
 *
 * @note    Tested on ESP32 family with ESP-IDF >= 5.0
 *
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Created on 14 of June, 2026
 */

#pragma once

#include "serial_comm/core/serial_comm_transport.h"
#include "serial_comm/serial_comm_config.h"
#include "serial_comm/core/serial_comm_utils.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"

#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_mac.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>


/* ---- Fragmentation constants ----------------------------------------- */

/** Hard limit imposed by the ESP-NOW protocol */
constexpr size_t    ESPNOW_MAX_FRAME_SIZE      = ESP_NOW_MAX_DATA_LEN; /* 250 */

/** Magic bytes that identify a SerialComm/ESP-NOW fragment */
constexpr uint8_t   ESPNOW_FRAG_MAGIC_0        = 0xEC;
constexpr uint8_t   ESPNOW_FRAG_MAGIC_1        = 0x50;

/**
 * @brief Fragment header layout (7 bytes, packed):
 *   magic[2]  |  msg_seq(1)  |  frag_idx(1)  |  frag_total(1)  |  frag_data_len(2-LE)
 */
constexpr size_t    ESPNOW_FRAG_HDR_SIZE       = 7;

/** Maximum data bytes carried in a single fragment */
constexpr size_t    ESPNOW_FRAG_DATA_SIZE      = ESPNOW_MAX_FRAME_SIZE - ESPNOW_FRAG_HDR_SIZE; /* 243 */

/**
 * @brief Maximum fragments per logical message.
 *        16 * 243 = 3888 bytes, comfortably above the largest possible
 *        SerialComm V2 packet (14 + MAX_PAYLOAD, default ≈ 526 bytes).
 */
constexpr uint8_t   ESPNOW_MAX_FRAGS           = 16;

/** Maximum reconstructable message size in bytes */
constexpr size_t    ESPNOW_MAX_MSG_SIZE        = ESPNOW_FRAG_DATA_SIZE * ESPNOW_MAX_FRAGS; /* 3888 */

/** Maximum number of peers that can be tracked internally */
constexpr size_t    ESPNOW_MAX_PEERS           = 8;


/* ---- Transport class -------------------------------------------------- */

/**
 * @brief   ESP-NOW transport for serial_comm middleware
 *
 * Usage example (UART-style drop-in):
 * @code
 *   // After WiFi init:
 *   ESPNowTransport::HardwareConfig hw;
 *   // hw.tx_peer_mac defaults to broadcast {0xFF,...}
 *
 *   ESPNowTransport espnow_transport(hw);
 *   SerialComm comm(&espnow_transport);
 *
 *   ISerialCommTransport::Config cfg;
 *   espnow_transport.init(cfg);
 *   espnow_transport.start();
 *
 *   SerialComm::Config sc_cfg;
 *   comm.init(sc_cfg);
 *   comm.start();
 * @endcode
 *
 * Switching between transports at runtime:
 * @code
 *   // Stop current transport
 *   serial_comm.stop();
 *   uart_transport.stop();
 *
 *   // Start ESP-NOW transport
 *   espnow_transport.init(cfg);
 *   espnow_transport.start();
 *
 *   // Restart SerialComm with new transport
 *   SerialComm comm_espnow(&espnow_transport);
 *   comm_espnow.init(sc_cfg);
 *   comm_espnow.start();
 * @endcode
 */
class ESPNowTransport final : public ISerialCommTransport {

    /* ---- Fragment header (packed layout on the wire) ------------------- */
    private:
        struct __attribute__((packed)) FragHeader {
            uint8_t  magic[2];       ///< { ESPNOW_FRAG_MAGIC_0, ESPNOW_FRAG_MAGIC_1 }
            uint8_t  msg_seq;        ///< Per-message sequence number (wraps 0–255)
            uint8_t  frag_idx;       ///< Zero-based fragment index
            uint8_t  frag_total;     ///< Total fragment count for this message
            uint16_t frag_data_len;  ///< Payload bytes in this fragment (little-endian)
        };
        static_assert(sizeof(FragHeader) == ESPNOW_FRAG_HDR_SIZE,
                      "FragHeader size does not match ESPNOW_FRAG_HDR_SIZE");


    /* ---- Internal types ------------------------------------------------ */
    private:
        /** Item stored in the RX queue (one ESP-NOW frame per item) */
        struct RxQueueItem {
            uint8_t  src_mac[6];
            uint8_t  data[ESPNOW_MAX_FRAME_SIZE];
            uint16_t data_len;
        };

        /** State machine for fragment reassembly */
        struct ReassemblyState {
            uint8_t  buf[ESPNOW_MAX_MSG_SIZE]; ///< Accumulation buffer
            size_t   total_written;            ///< Bytes written so far
            uint8_t  msg_seq;                  ///< Sequence number of ongoing message
            uint8_t  frags_expected;           ///< Total fragment count expected
            uint8_t  frags_received;           ///< Fragment count received so far
            bool     active;                   ///< True while assembly is in progress
        };

        /** Internal peer registry entry */
        struct PeerEntry {
            bool    used;
            uint8_t node_id;   ///< Logical node ID (for future V2 SRC/DST routing)
            uint8_t mac[6];
        };


    /* ---- Hardware / network configuration ----------------------------- */
    public:
        /**
         * @brief   ESP-NOW hardware and link configuration
         */
        struct HardwareConfig {
            /** Default destination MAC (broadcast by default) */
            uint8_t tx_peer_mac[6]   = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
            /** WiFi channel for peer; 0 = follow current channel */
            uint8_t channel          = 0;
            /** WiFi interface used by ESP-NOW (STA is typical) */
            wifi_interface_t wifi_if = WIFI_IF_STA;
            /** Enable 802.11lr long-range mode */
            bool use_long_range      = false;
            /** Enable per-peer encryption */
            bool encrypt             = false;
            /** Primary master key (16 bytes; required when encrypt = true) */
            uint8_t pmk[16]         = {};
            /** Local master key for the default peer (16 bytes) */
            uint8_t lmk[16]         = {};
        };


    /* ---- Private members ----------------------------------------------- */
    private:
        HardwareConfig       hw_cfg_;
        Config               cfg_;
        State                state_;

        uint8_t              tx_msg_seq_;   ///< Monotonic TX message counter

        SemaphoreHandle_t    tx_done_sem_;  ///< Signals send_cb completion
        volatile bool        tx_success_;   ///< Set by send_cb

        QueueHandle_t        rx_queue_;     ///< Raw ESP-NOW frames from recv_cb
        TaskHandle_t         rx_task_;      ///< Reassembly / dispatch task

        ReassemblyState      reassembly_;
        SemaphoreHandle_t    reassembly_mutex_;

        PeerEntry            peers_[ESPNOW_MAX_PEERS];

        StreamBufferHandle_t poll_stream_;  ///< Buffer for polling read()/available()

        rx_callback_t        rx_callback_;
        tx_done_callback_t   tx_done_callback_;
        event_callback_t     event_callback_;
        void*                rx_ctx_;
        void*                tx_ctx_;
        void*                event_ctx_;

        SemaphoreHandle_t    state_mutex_;

        /**
         * @brief Global singleton pointer required because ESP-NOW callbacks
         *        carry no user context.  Only one instance is supported.
         */
        static ESPNowTransport* instance_;


    /* ---- Private helpers ---------------------------------------------- */
    private:
        /** ESP-NOW receive callback (called from Wi-Fi task context) */
        static void recv_cb(
            const esp_now_recv_info_t* recv_info,
            const uint8_t*             data,
            int                        data_len
        );

        /** ESP-NOW send-done callback (called from Wi-Fi task context) */
        static void send_cb(
            const esp_now_send_info_t* send_info,
            esp_now_send_status_t      status
        );

        /** FreeRTOS task that drains rx_queue_ and reassembles fragments */
        static void rx_task_entry(void* args);

        /** Process one RxQueueItem: validate magic, reassemble, dispatch */
        void process_rx_item(const RxQueueItem& item);

        /**
         * @brief   Encode and transmit a single fragment via esp_now_send()
         * @param   data        Payload bytes for this fragment
         * @param   len         Payload byte count (≤ ESPNOW_FRAG_DATA_SIZE)
         * @param   msg_seq     Message sequence number
         * @param   frag_idx    Zero-based fragment index
         * @param   frag_total  Total fragment count
         * @return  errCode::OK on success
         */
        errCode send_fragment(
            const uint8_t* data,
            size_t         len,
            uint8_t        msg_seq,
            uint8_t        frag_idx,
            uint8_t        frag_total
        );

        /** Release all FreeRTOS objects and clear state */
        void cleanup_resources();


    /* ---- Public interface ---------------------------------------------- */
    public:
        /**
         * @brief   Constructor
         * @param   hw_cfg  Hardware and network configuration.
         *                  WiFi must be initialised before calling init().
         * @note    Pass a default-constructed HardwareConfig to use broadcast
         *          as the TX peer:  ESPNowTransport::HardwareConfig hw{};
         *                           ESPNowTransport t(hw);
         */
        explicit ESPNowTransport(const HardwareConfig& hw_cfg);

        ESPNowTransport(const ESPNowTransport&)             = delete;
        ESPNowTransport& operator=(const ESPNowTransport&)  = delete;
        ESPNowTransport(ESPNowTransport&&)                  = delete;
        ESPNowTransport& operator=(ESPNowTransport&&)       = delete;

        ~ESPNowTransport() override;


    /* ---- ISerialCommTransport lifecycle -------------------------------- */
    public:
        /**
         * @brief   Initialise ESP-NOW transport
         * @param   cfg  Base transport config (timeout_ms is used as TX ack timeout)
         * @return  errCode::OK on success
         * @note    Calls esp_now_init() internally; WiFi must already be running
         */
        errCode init(const Config& cfg)  override;
        errCode start()                   override;
        errCode stop()                    override;
        errCode deinit()                  override;


    /* ---- ISerialCommTransport data transfer --------------------------- */
    public:
        /**
         * @brief   Synchronous write: fragments data and sends each fragment,
         *          waiting for the ESP-NOW send-done callback after each one.
         * @return  Bytes written, or -1 on error
         */
        int write(const uint8_t* data, size_t len) override;

        /**
         * @brief   Asynchronous write: fragments and sends without waiting
         *          for per-fragment send confirmation.
         * @return  errCode::OK if all fragments were queued
         */
        errCode write_async(const uint8_t* data, size_t len) override;

        /**
         * @brief   Polling read from internal stream buffer
         * @return  Bytes read, or -1 on timeout
         */
        int read(uint8_t* data, size_t len, uint32_t timeout_ms) override;

        /** @return Bytes available in the polling stream buffer */
        int available() const override;

        /** Discard all data pending in the polling stream buffer */
        errCode flush() override;


    /* ---- ISerialCommTransport callbacks ------------------------------- */
    public:
        errCode set_rx_callback(
            rx_callback_t cb, void* ctx
        ) override;

        errCode set_tx_done_callback(
            tx_done_callback_t cb, void* ctx
        ) override;

        errCode set_event_callback(
            event_callback_t cb, void* ctx
        ) override;


    /* ---- ISerialCommTransport state ----------------------------------- */
    public:
        bool  running() const  override;
        State state()   const  override;


    /* ---- ISerialCommTransport buffer info ----------------------------- */
    public:
        size_t rx_buffer_size() const override;
        size_t tx_buffer_size() const override;


    /* ---- ESP-NOW specific API ----------------------------------------- */
    public:
        /**
         * @brief   Register a peer with ESP-NOW and the internal peer table
         * @param   mac      Peer MAC address (6 bytes)
         * @param   node_id  Logical node ID used for future V2 SRC/DST routing
         * @param   channel  Wi-Fi channel; 0 = use current channel
         * @param   encrypt  Enable link-layer encryption for this peer
         * @param   lmk      Local master key (16 bytes, required if encrypt = true)
         * @return  errCode::OK on success
         */
        errCode add_peer(
            const uint8_t* mac,
            uint8_t        node_id  = 0xFF,
            uint8_t        channel  = 0,
            bool           encrypt  = false,
            const uint8_t* lmk     = nullptr
        );

        /**
         * @brief   Remove a peer from ESP-NOW and the internal peer table
         * @param   mac  Peer MAC address (6 bytes)
         * @return  errCode::OK on success
         */
        errCode remove_peer(const uint8_t* mac);

        /**
         * @brief   Change the default TX peer MAC at runtime
         * @param   mac  New destination MAC (6 bytes)
         */
        void set_tx_peer(const uint8_t* mac);

        /**
         * @brief   Read this device's MAC address
         * @param   out_mac  Output buffer (6 bytes)
         * @return  errCode::OK on success
         */
        errCode get_local_mac(uint8_t* out_mac) const;
};
