/**
 * @file    udp_serial_comm.h
 * @brief   WiFi UDP transport implementation for serial_comm middleware
 * @details Implements ISerialCommTransport using POSIX UDP sockets over the
 *          ESP-IDF LwIP stack.  Provides both unicast and broadcast TX to a
 *          configurable remote endpoint, with callback-driven and polled RX.
 *
 *          Feature overview:
 *            - Unicast TX to a specific IP:port
 *            - Broadcast TX (SO_BROADCAST) to 255.255.255.255 or subnet addr
 *            - Callback-driven RX via set_rx_callback()
 *            - Polled RX via read() / available() (StreamBuffer backed)
 *            - Runtime endpoint switching via set_remote_endpoint()
 *            - Multi-peer table with write_to_peer() for point-to-point TX
 *            - Local IP query via get_local_ip()
 *
 *          Prerequisites (caller's responsibility before init()):
 *            1. nvs_flash_init()
 *            2. esp_netif_init() + esp_event_loop_create_default()
 *            3. WiFi initialized, connected, and IP obtained
 *               (wait for IP_EVENT_STA_GOT_IP before calling init())
 *
 *          Topology examples:
 *
 *          @code
 *          // A. Broadcast — one sender to all listeners on LAN
 *          UDPTransport::HardwareConfig hw{};
 *          hw.local_port  = 4210;
 *          hw.remote_ip   = "255.255.255.255";   // default
 *          hw.remote_port = 4210;
 *          hw.broadcast   = true;                // default
 *
 *          UDPTransport t(hw);
 *          t.init({});
 *          t.start();
 *
 *          SerialComm comm(&t);
 *          comm.init({});
 *          comm.start();
 *          @endcode
 *
 *          @code
 *          // B. Unicast — PC at 192.168.4.2, ESP at 192.168.4.1
 *          UDPTransport::HardwareConfig hw{};
 *          hw.local_port  = 4210;
 *          hw.remote_port = 4210;
 *          hw.broadcast   = false;
 *          strncpy(hw.remote_ip, "192.168.4.2", sizeof(hw.remote_ip));
 *          @endcode
 *
 *          @code
 *          // C. Multi-peer — send to specific robots
 *          t.add_peer("192.168.4.2", 4210, /*node_id=*‌/1);
 *          t.add_peer("192.168.4.3", 4210, /*node_id=*‌/2);
 *          t.write_to_peer(1, data, len);   // → robot 1
 *          t.write_to_peer(2, data, len);   // → robot 2
 *          @endcode
 *
 * @note    WiFi must be connected and IP assigned before calling init().
 *          The transport does NOT manage WiFi provisioning.
 *
 * @note    Multiple UDPTransport instances may coexist on different ports.
 *          If two instances bind the same port, only one will receive packets
 *          unless SO_REUSEADDR and SO_REUSEPORT are both set.
 *
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Created on 13 of July, 2026
 */

#pragma once

#include "serial_comm/core/serial_comm_transport.h"
#include "serial_comm/serial_comm_config.h"
#include "serial_comm/core/serial_comm_utils.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_netif.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>


/* ---- Constants ----------------------------------------------------------- */

/** Maximum raw packet size accepted from a single recvfrom() call.
 *  Ethernet MTU (1500) − IP header (20) − UDP header (8) = 1472 bytes.
 *  SerialComm V1 max packet ≈ 526 bytes (11-byte header + 512 payload + 2 CRC). */
constexpr size_t UDP_MAX_RX_PACKET_SIZE = 1472;

/** Maximum number of peers in the TX peer table */
constexpr size_t UDP_MAX_PEERS = 8;


/* ---- Transport class ----------------------------------------------------- */

/**
 * @brief   WiFi/UDP transport for the serial_comm middleware
 *
 * Drop-in replacement for UARTTransport or ESPNowTransport.  All SerialComm
 * middleware layers (SerialComm, SerialCommManager, SerialCommParser) work
 * without any modification.
 */
class UDPTransport final : public ISerialCommTransport {

    /* ---- Internal peer entry -------------------------------------------- */
    private:
        struct PeerEntry {
            bool              used    = false;
            uint8_t           node_id = 0xFF;
            struct sockaddr_in addr   = {};
        };


    /* ---- Hardware / network configuration -------------------------------- */
    public:
        /**
         * @brief   UDP endpoint and socket configuration
         */
        struct HardwareConfig {
            /** Local UDP port to bind for receiving packets */
            uint16_t local_port    = 4210;

            /** Default TX destination IP address.
             *  Use "255.255.255.255" for LAN broadcast or a specific IPv4
             *  string (e.g. "192.168.4.2") for unicast. */
            char     remote_ip[16] = "255.255.255.255";

            /** Default TX destination port */
            uint16_t remote_port   = 4210;

            /** Enable SO_BROADCAST — required when remote_ip is a broadcast address */
            bool     broadcast     = true;

            /** Enable SO_REUSEADDR — allows multiple processes or restarts
             *  to bind the same port without TIME_WAIT blocking */
            bool     reuse_addr    = true;
        };


    /* ---- Private members ------------------------------------------------- */
    private:
        HardwareConfig       hw_cfg_;
        Config               cfg_;
        State                state_;

        int                  sock_;           ///< Bound UDP socket file descriptor
        struct sockaddr_in   remote_addr_;    ///< Current default TX destination

        TaskHandle_t         rx_task_;        ///< RX receive-and-dispatch task
        StreamBufferHandle_t poll_stream_;    ///< Backing store for read()/available()

        rx_callback_t        rx_callback_;
        tx_done_callback_t   tx_done_callback_;
        event_callback_t     event_callback_;
        void*                rx_ctx_;
        void*                tx_ctx_;
        void*                event_ctx_;

        /**
         * @brief   Mutex protecting remote_addr_ and peers_[].
         *          Held briefly during set_remote_endpoint(), add_peer(),
         *          remove_peer(), and the address-copy step inside write().
         *          Not held during the actual sendto() to avoid blocking the
         *          RX task unnecessarily.
         */
        SemaphoreHandle_t    net_mutex_;

        /**
         * @brief   Binary semaphore signalled by the RX task just before it
         *          calls vTaskDelete(nullptr).  stop() waits on this to confirm
         *          the task has exited cleanly before clearing rx_task_.
         */
        SemaphoreHandle_t    rx_done_sem_;

        PeerEntry            peers_[UDP_MAX_PEERS];


    /* ---- Private helpers ------------------------------------------------- */
    private:
        /** FreeRTOS task: blocking recvfrom() loop; dispatches to callbacks */
        static void rx_task_entry(void* args);

        /** Release FreeRTOS objects and close socket */
        void cleanup_resources();

        /** Internal helper: send a datagram to a specific sockaddr */
        int sendto_addr(
            const struct sockaddr_in& dest,
            const uint8_t*            data,
            size_t                    len
        );


    /* ---- Construction ----------------------------------------------------- */
    public:
        /**
         * @brief   Constructor
         * @param   hw_cfg  Network endpoint configuration.
         *                  WiFi must be connected before calling init().
         */
        explicit UDPTransport(const HardwareConfig& hw_cfg);

        UDPTransport(const UDPTransport&)            = delete;
        UDPTransport& operator=(const UDPTransport&) = delete;
        UDPTransport(UDPTransport&&)                 = delete;
        UDPTransport& operator=(UDPTransport&&)      = delete;

        ~UDPTransport() override;


    /* ---- ISerialCommTransport lifecycle ----------------------------------- */
    public:
        /**
         * @brief   Create and bind the UDP socket, configure sockopts.
         * @param   cfg  Base transport config (timeout_ms unused; rx/tx buffer
         *               sizes may be used for future tuning).
         * @return  errCode::OK on success
         * @note    WiFi must be connected and IP assigned before this call.
         */
        errCode init(const Config& cfg)  override;

        /**
         * @brief   Spawn the RX task and transition to RUNNING state.
         * @return  errCode::OK on success
         */
        errCode start()                   override;

        /**
         * @brief   Stop the RX task; socket remains bound (can restart).
         * @return  errCode::OK
         */
        errCode stop()                    override;

        /**
         * @brief   Stop the task, close the socket, and free all resources.
         * @return  errCode::OK
         */
        errCode deinit()                  override;


    /* ---- ISerialCommTransport data transfer ------------------------------ */
    public:
        /**
         * @brief   Synchronous write — sendto() to the current remote endpoint.
         * @param   data  Bytes to send
         * @param   len   Byte count
         * @return  Bytes sent (>0), or -1 on error
         * @note    UDP sendto() is effectively non-blocking at the application
         *          level; this call returns as soon as the datagram is handed
         *          to LwIP.  The tx_done_callback is invoked after sendto().
         */
        int write(const uint8_t* data, size_t len) override;

        /**
         * @brief   Asynchronous write — identical to write() for UDP.
         *          Provided for interface symmetry with UART and ESP-NOW.
         * @return  errCode::OK on success, errCode::ERR_IO on socket error
         */
        errCode write_async(const uint8_t* data, size_t len) override;

        /**
         * @brief   Blocking poll read from the internal StreamBuffer.
         * @param   data        Destination buffer
         * @param   len         Maximum bytes to read
         * @param   timeout_ms  Maximum wait time in milliseconds
         * @return  Bytes read, or 0 on timeout
         * @note    The RX task continuously drains recvfrom() into the
         *          StreamBuffer.  This function just reads from that buffer.
         */
        int read(uint8_t* data, size_t len, uint32_t timeout_ms) override;

        /** @return Bytes immediately available in the StreamBuffer */
        int available() const override;

        /** Discard all data in the StreamBuffer */
        errCode flush() override;


    /* ---- ISerialCommTransport callbacks ---------------------------------- */
    public:
        errCode set_rx_callback(rx_callback_t cb, void* ctx)           override;
        errCode set_tx_done_callback(tx_done_callback_t cb, void* ctx) override;
        errCode set_event_callback(event_callback_t cb, void* ctx)     override;


    /* ---- ISerialCommTransport state -------------------------------------- */
    public:
        bool  running() const  override;
        State state()   const  override;


    /* ---- ISerialCommTransport buffer info -------------------------------- */
    public:
        /** Returns StreamBuffer capacity (== CONFIG_SERIAL_COMM_UDP_RX_BUFFER_SIZE) */
        size_t rx_buffer_size() const override;

        /** UDP has no software TX buffer; returns 0 */
        size_t tx_buffer_size() const override;


    /* ---- UDP-specific API ------------------------------------------------ */
    public:
        /**
         * @brief   Change the default TX destination at runtime.
         * @param   ip    Dotted-decimal IPv4 string (e.g. "192.168.4.2")
         * @param   port  Destination UDP port
         * @note    Thread-safe; takes effect on the next write() / write_async().
         */
        void set_remote_endpoint(const char* ip, uint16_t port);

        /**
         * @brief   Register a named peer for targeted TX via write_to_peer().
         * @param   ip       Dotted-decimal IPv4 string
         * @param   port     Destination UDP port
         * @param   node_id  Application-level node identifier (0x00–0xFE)
         * @return  errCode::OK on success
         * @return  errCode::ERR_NO_MEMORY if the peer table is full
         * @return  errCode::ERR_ALREADY_INITIALIZED if node_id already exists
         */
        errCode add_peer(const char* ip, uint16_t port, uint8_t node_id = 0xFF);

        /**
         * @brief   Remove a peer from the table by node_id.
         * @param   node_id  Node identifier to remove
         * @return  errCode::OK (idempotent — removing absent node is fine)
         */
        errCode remove_peer(uint8_t node_id);

        /**
         * @brief   Send a datagram to a specific registered peer.
         * @param   node_id  Target peer node identifier
         * @param   data     Bytes to send
         * @param   len      Byte count
         * @return  Bytes sent (>0), or -1 on error / peer not found
         */
        int write_to_peer(uint8_t node_id, const uint8_t* data, size_t len);

        /**
         * @brief   Query the local IPv4 address of the active WiFi interface.
         * @param   buf      Output buffer for dotted-decimal string
         * @param   buf_len  Buffer capacity (must be ≥ 16)
         * @return  errCode::OK on success
         * @return  errCode::ERR_FAIL if no IP is assigned yet
         */
        errCode get_local_ip(char* buf, size_t buf_len) const;
};
