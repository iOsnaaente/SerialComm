/**
 * @file    multi_transport.h
 * @brief   Composite transport — aggregates multiple ISerialCommTransport
 *          instances behind a single ISerialCommTransport interface.
 *
 * @details Enables simultaneous use of several physical transports without
 *          any change to SerialComm, SerialCommManager, or the application
 *          layer.  Typical use cases:
 *
 *          • Dual RX: receive from UART *and* ESP-NOW — whichever carries
 *            a packet first delivers it to the middleware.
 *          • Dual TX: broadcast a message to UART *and* ESP-NOW peers at
 *            the same time.
 *          • Bridge: receive from UART, forward to ESP-NOW (and vice-versa)
 *            by setting tx_enabled on both slots but filtering as desired.
 *          • Selective: rx from all, tx via only one (e.g. ESP-NOW-only TX).
 *
 * Usage:
 * @code
 *   // 1. Init sub-transports independently (MultiTransport does NOT own them)
 *   UARTTransport uart(uart_hw);
 *   uart.init(uart_cfg);
 *   uart.start();
 *
 *   ESPNowTransport espnow(espnow_hw);
 *   espnow.init(espnow_cfg);
 *   espnow.start();
 *
 *   // 2. Build composite  (add before init)
 *   MultiTransport multi;
 *   multi.add_transport(&uart,   {.rx_enabled=true, .tx_enabled=true});
 *   multi.add_transport(&espnow, {.rx_enabled=true, .tx_enabled=true});
 *
 *   // 3. Same lifecycle as any other transport
 *   multi.init({});
 *   multi.start();
 *
 *   SerialComm comm(&multi);
 *   comm.init({});
 *   comm.start();
 *
 *   SerialCommManager mgr(&comm);
 *   mgr.init({});
 *   mgr.start();
 * @endcode
 *
 * @note  Sub-transports must be initialised and started before MultiTransport
 *        is initialised.  MultiTransport does NOT call init/start/stop/deinit
 *        on sub-transports — the caller keeps full lifecycle control.
 *
 * @note  Do not call set_rx_callback / set_event_callback directly on a
 *        sub-transport after multi.init() — that would override the internal
 *        forwarding hooks and break reception.
 *
 * @note  write() is synchronous on every tx-enabled slot in order.  If one
 *        transport is slow (e.g. many ESP-NOW fragments), that latency is
 *        added to the total.  Use write_async() to avoid blocking.
 *
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Created on 14 of June, 2026
 */

#pragma once

#include "serial_comm/core/serial_comm_transport.h"
#include "serial_comm/core/serial_comm_utils.h"

#include "esp_log.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>


class MultiTransport final : public ISerialCommTransport {

    /* ---- Slot configuration ----------------------------------------------- */
    public:
        /** Maximum number of sub-transports supported */
        static constexpr size_t MAX_SLOTS = 4;

        /**
         * @brief   Per-slot RX/TX participation flags
         */
        struct SlotConfig {
            bool rx_enabled = true;  ///< Receive packets from this transport
            bool tx_enabled = true;  ///< Transmit packets via this transport
        };


    /* ---- Internal slot table ---------------------------------------------- */
    private:
        struct Slot {
            ISerialCommTransport* transport = nullptr;
            SlotConfig            cfg;
            bool                  used      = false;
        };

        Slot   slots_[MAX_SLOTS];
        size_t slot_count_ = 0;


    /* ---- Unified callbacks ------------------------------------------------- */
    private:
        rx_callback_t      rx_callback_       = nullptr;
        void*              rx_ctx_            = nullptr;

        tx_done_callback_t tx_done_callback_  = nullptr;
        void*              tx_ctx_            = nullptr;

        event_callback_t   event_callback_    = nullptr;
        void*              event_ctx_         = nullptr;


    /* ---- State ------------------------------------------------------------- */
    private:
        State  state_ = State::UNINITIALIZED;
        Config cfg_;


    /* ---- Static forwarders ------------------------------------------------- */
    private:
        /**
         * @brief   RX forwarder registered as rx_callback on every rx-enabled
         *          slot.  ctx = this MultiTransport instance.
         */
        static void sub_rx_forwarder(
            void* ctx, const uint8_t* data, size_t len
        );

        /** TX-done forwarder */
        static void sub_tx_done_forwarder(void* ctx);

        /** Transport event forwarder */
        static void sub_event_forwarder(void* ctx, Event ev);

        /** Helper: register forwarders on one slot */
        void register_slot_callbacks(size_t idx);


    /* ---- Construction ------------------------------------------------------ */
    public:
        MultiTransport();

        MultiTransport(const MultiTransport&)            = delete;
        MultiTransport& operator=(const MultiTransport&) = delete;
        MultiTransport(MultiTransport&&)                 = delete;
        MultiTransport& operator=(MultiTransport&&)      = delete;

        ~MultiTransport() override;


    /* ---- Slot management --------------------------------------------------- */
    public:
        /**
         * @brief   Add a sub-transport to the composite
         *
         * @param   transport  Pointer to an already-started transport instance.
         *                     Ownership stays with the caller.
         * @param   cfg        RX / TX participation for this slot.
         *                     Default (both enabled): pass SlotConfig{} explicitly.
         * @return  errCode::OK on success, ERR_NO_MEMORY if all slots are full,
         *          ERR_NULL_POINTER if transport == nullptr,
         *          ERR_ALREADY_INITIALIZED if the same pointer was already added.
         *
         * @note    May be called before OR after init().  If called after init(),
         *          the forwarder callbacks are registered immediately.
         */
        errCode add_transport(
            ISerialCommTransport* transport,
            const SlotConfig& cfg
        );

        /**
         * @brief   Remove a previously added sub-transport
         * @param   transport  Pointer to identify the slot to remove.
         * @return  errCode::OK (idempotent — removing an absent pointer is fine)
         */
        errCode remove_transport(ISerialCommTransport* transport);

        /** @return Number of slots currently in use */
        size_t slot_count() const { return slot_count_; }


    /* ---- ISerialCommTransport lifecycle ------------------------------------ */
    public:
        /**
         * @brief   Register forwarder callbacks on all sub-transports and
         *          transition to INITIALIZED state.
         * @note    Sub-transports should be running before this call.
         */
        errCode init(const Config& cfg) override;
        errCode start()                  override;
        errCode stop()                   override;
        errCode deinit()                 override;


    /* ---- ISerialCommTransport data transfer -------------------------------- */
    public:
        /**
         * @brief   Synchronous write to ALL tx-enabled sub-transports
         * @return  Bytes written if at least one transport succeeded; -1 if all failed
         */
        int     write(const uint8_t* data, size_t len)             override;

        /**
         * @brief   Asynchronous write to ALL tx-enabled sub-transports
         * @return  errCode::OK if at least one transport succeeded
         */
        errCode write_async(const uint8_t* data, size_t len)       override;

        /**
         * @brief   Poll first rx-enabled slot that has data; falls back to
         *          blocking read on the first rx-enabled slot.
         */
        int     read(uint8_t* data, size_t len, uint32_t timeout_ms) override;

        /** @return Total bytes available across all rx-enabled slots */
        int     available() const                                  override;

        /** Flush all slots */
        errCode flush()                                            override;


    /* ---- ISerialCommTransport callbacks ------------------------------------ */
    public:
        errCode set_rx_callback(rx_callback_t cb, void* ctx)           override;
        errCode set_tx_done_callback(tx_done_callback_t cb, void* ctx) override;
        errCode set_event_callback(event_callback_t cb, void* ctx)     override;


    /* ---- ISerialCommTransport state --------------------------------------- */
    public:
        bool  running() const override;
        State state()   const override;


    /* ---- ISerialCommTransport buffer info --------------------------------- */
    public:
        /** Sum of rx_buffer_size() across all rx-enabled slots */
        size_t rx_buffer_size() const override;

        /** Sum of tx_buffer_size() across all tx-enabled slots */
        size_t tx_buffer_size() const override;
};
