/**
 * @file    serial_comm_types.hpp
 * @brief   Type abstraction for serialization runtime
 */

#pragma once

#include <type_traits>
#include <stdint.h>
#include <array>


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