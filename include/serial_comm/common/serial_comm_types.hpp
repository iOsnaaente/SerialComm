/**
 * @file    serial_comm_types.hpp
 * @brief   Type abstraction for serialization runtime
 */

#pragma once

#include <type_traits>
#include <stdint.h>
#include <array>


/* ============================================================================
 * DELIVERY MODE
 * ========================================================================== */

/**
 * @brief   Transport-level delivery guarantee for a message type.
 *
 * BEST_EFFORT — fire and forget.  Uses write_async() on the transport.
 *   ESP-NOW: sends each fragment without waiting for ACK status.
 *   UART:    queues bytes in the TX ring buffer and returns immediately.
 *
 * RELIABLE — wait for link-layer confirmation.  Uses write() on the transport.
 *   ESP-NOW: waits for the send-done callback per fragment and checks success.
 *   UART:    waits until the hardware TX FIFO drains (uart_wait_tx_done).
 *
 * The mode is declared per-type in the IDL via @best_effort / @reliable and
 * emitted as a static constexpr member:
 *   static constexpr DeliveryMode DELIVERY_MODE = DeliveryMode::BEST_EFFORT;
 *
 * SerialCommManager::publish<Msg>() reads Msg::DELIVERY_MODE automatically.
 * For non-generated types the trait below falls back to BEST_EFFORT.
 */
enum class DeliveryMode : uint8_t {
    BEST_EFFORT = 0,
    RELIABLE    = 1,
};


/* ---- Trait: safely read T::DELIVERY_MODE with fallback to BEST_EFFORT ---- */

template<typename T, typename = void>
struct SerialCommHasDeliveryMode : std::false_type {};

template<typename T>
struct SerialCommHasDeliveryMode<
    T,
    std::void_t<decltype(T::DELIVERY_MODE)>
> : std::true_type {};

template<typename T>
constexpr DeliveryMode serial_comm_delivery_mode() {
    if constexpr (SerialCommHasDeliveryMode<T>::value) {
        return T::DELIVERY_MODE;
    } else {
        return DeliveryMode::BEST_EFFORT;
    }
}


/* ---- Trait: @timeout_ms — per-type call_service default timeout ---------- */

template<typename T, typename = void>
struct SerialCommHasTimeoutMs : std::false_type {};

template<typename T>
struct SerialCommHasTimeoutMs<
    T,
    std::void_t<decltype(T::TIMEOUT_MS)>
> : std::true_type {};

/**
 * @brief   Returns T::TIMEOUT_MS if the type declares @timeout_ms in its IDL,
 *          otherwise returns @p default_ms (typically from
 *          SerialCommConfig::TRANSACTION_TIMEOUT_MS).
 */
template<typename T>
constexpr uint32_t serial_comm_timeout_ms(uint32_t default_ms) {
    if constexpr (SerialCommHasTimeoutMs<T>::value) {
        return static_cast<uint32_t>(T::TIMEOUT_MS);
    } else {
        return default_ms;
    }
}


/* ---- Trait: @retain — last-value cache flag ------------------------------ */

template<typename T, typename = void>
struct SerialCommHasRetain : std::false_type {};

template<typename T>
struct SerialCommHasRetain<
    T,
    std::void_t<decltype(T::RETAIN)>
> : std::true_type {};

/**
 * @brief   Returns true if the type declares @retain in its IDL.
 *          Used by SerialCommManager::create_subscription<Msg>() to
 *          automatically register the command's last-value cache.
 */
template<typename T>
constexpr bool serial_comm_retain() {
    if constexpr (SerialCommHasRetain<T>::value) {
        return T::RETAIN;
    } else {
        return false;
    }
}


/* ============================================================================
 * STD ARRAY DETECTION
 * ========================================================================== */
template<typename T>
struct SerialCommIsStdArray :
    std::false_type {};


template<typename T, size_t N>
struct SerialCommIsStdArray<std::array<T, N>> :
    std::true_type {};


enum class SerialCommBufferEndian {
    LITTLE,
    BIG
};

struct SerialCommDynamicString {
    char* data = nullptr;
    uint16_t size = 0;
    uint16_t capacity = 0;
};



template<typename T>
struct SerialCommDynamicArray {
    T* data = nullptr;
    uint16_t size = 0;
    uint16_t capacity = 0;
};