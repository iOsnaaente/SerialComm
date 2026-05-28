/**
 * @brief   Generic communication transport interface
 * @details Only transports bytes, does not know nothing about the 
 *          Application Layer or how the interface is implemented by 
 *          the hardware. 
 * @note    This is the lowest level of abstraction in the serial
 *          communication middleware, and is designed to be implemented by
 *          various kind of hardware-specific transport layers or brands.
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Created on 22 of May, 2026 
 */

#pragma once

#include "serial_comm/serial_comm_config.h"
#include "serial_comm/core/serial_comm_utils.h"

#include <stdint.h>
#include <stddef.h>

using namespace SerialCommResult_Codes;

class ISerialCommTransport{
    
    /* STRUCTURES OF EVENTS, STATES AND CONFIGURATIONS */
    public:
        /* TRANSPORT EVENTS */
        enum class Event {
            CONNECTED,
            DISCONNECTED,
            RX_OVERFLOW,
            PARITY_ERROR,
            FRAME_ERROR,
            BUFFER_FULL,
            TIMEOUT,
            TX_DONE,
            ERROR
        };

        /* TRANSPORT STATE */
        enum class State {
            UNINITIALIZED,
            INITIALIZED,
            RUNNING,
            STOPPED,
            ERROR
        };

        /* TRANSPORT CONFIG */
        struct Config{
            uint32_t baudrate           = SerialCommConfig::UART_BAUDRATE;
            size_t rx_buffer_size       = SerialCommConfig::UART_RX_BUFFER_SIZE;
            size_t tx_buffer_size       = SerialCommConfig::UART_TX_BUFFER_SIZE;
            bool auto_interbyte_timeout = true;
            float interbyte_chars       = 3.5f;
            uint32_t timeout_ms         = 10;
            bool half_duplex = false;
        };
        

    /* CALLBACKS */
    public:
        /**
         * @brief   Callback type for received data
         * @param   data Pointer to the received data buffer
         * @param   len Length of the received data in bytes
         */
        using rx_callback_t =
            void (*)( void *ctx, const uint8_t* data,  size_t len );

        /**
         * @brief   Callback type for transmission completion
         * @note    No parameters, as it simply indicates that the 
         *          transmission is done
         */
        using tx_done_callback_t =
            void (*)(void *ctx);
        /**
         * @brief   Callback type for transport events
         * @param   event The event that occurred, represented as an 
         *          integer
         */
        using event_callback_t =
            void (*)( void *ctx, Event event);


    /* LIFE CYCLE */
    public:
        /**
         * @brief   Default constructor for interface
         */
        ISerialCommTransport() = default;

        /**
         * @brief   Initialize the transport with the given configuration
         * @param   cfg The configuration parameters for the transport
         * @return  SerialCommResult::OK on success
         * @return  SerialCommResult::ERR_FAIL if initialization fails 
         *          for other reasons.
         */
        virtual errCode init(
            const Config& cfg
        ) = 0;

        /**
         * @brief   Start the transport, making it ready for communication
         * @return  SerialCommResult::OK on success
         * @return  SerialCommResult::ERR_FAIL if the transport fails to 
         *          start.
         */
        virtual errCode start() = 0;

        /**
         * @brief   Stop the transport, halting all communication
         * @return  SerialCommResult::OK on success
         * @return  SerialCommResult::ERR_FAIL if the transport fails to 
         *          stop.
         */
        virtual errCode stop() = 0;

        /**
         * @brief   Deinitialize the transport, releasing all resources
         * @return  SerialCommResult::OK on success
         * @return  SerialCommResult::ERR_FAIL if the transport fails to 
         *          deinitialize.
         */
        virtual errCode deinit() = 0;


    /* DATA TRANSFER AND IO FLUX */
    public:
        /**
         * @brief   Write data to the transport
         * @param   data Pointer to the data buffer to be transmitted
         * @param   len Length of the data to be transmitted in bytes
         * @return  Number of bytes written on success
         * @return  Negative value on failure
         */
        virtual int write(
            const uint8_t* data,
            size_t len
        ) = 0;

        /**
         * @brief   Write data to the transport asynchronously
         * @param   data Pointer to the data buffer to be transmitted
         * @param   len Length of the data to be transmitted in bytes
         * @return  SerialCommResult::OK if the data was successfully 
         *          queued for transmission
         * @return  SerialCommResult::ERR_INVALID_PARAM if the input 
         *          parameters are invalid
         * @return  SerialCommResult::ERR_FAIL if the transport fails to 
         *          queue the data for transmission
         */
        virtual errCode write_async(
            const uint8_t* data,
            size_t len
        ) = 0;

        /**
         * @brief   Read data from the transport
         * @param   data Pointer to the buffer where the received data 
         *          will be stored
         * @param   len Length of the buffer in bytes
         * @param   timeout_ms Timeout in milliseconds for the read 
         *          operation
         * @return  Number of bytes read on success
         * @return  Negative value on failure
         */
        virtual int read(
            uint8_t* data,
            size_t len,
            uint32_t timeout_ms
        ) = 0;

        /**
         * @brief   Get the number of bytes available to read from the 
         *          transport
         * @return  Number of bytes available to read
         */
        virtual int available() const = 0;

        /**
         * @brief   Flush the transport, ensuring all pending data is 
         *          transmitted and buffers are cleared
         * @return  SerialCommResult::OK on success
         * @return  SerialCommResult::ERR_FAIL if the transport fails to 
         *          flush
         */
        virtual errCode flush() = 0;


    /* CALLBACKS HANDLERS */
    public:
        /**
         * @brief   Set the callback function for received data
         * @param   cb The callback function to be called when data is 
         *          received
         * @return  SerialCommResult::OK on success
         * @return  SerialCommResult::ERR_FAIL if the transport fails to 
         *          set the callback
         */
        virtual errCode set_rx_callback(
            rx_callback_t cb, void *ctx
        ) = 0;

        /**
         * @brief   Set the callback function for transmission completion
         * @param   cb The callback function to be called when transmission 
         *          is completed
         * @return  SerialCommResult::OK on success
         * @return  SerialCommResult::ERR_FAIL if the transport fails to 
         *          set the callback
         */
        virtual errCode set_tx_done_callback(
            tx_done_callback_t cb, void *ctx
        ) = 0;

        /**
         * @brief   Set the callback function for transport events
         * @param   cb The callback function to be called when a transport 
         *          event occurs
         * @return  SerialCommResult::OK on success
         * @return  SerialCommResult::ERR_FAIL if the transport fails to 
         *          set the callback
         */
        virtual errCode set_event_callback(
            event_callback_t cb, void *ctx
        ) = 0;

        
    /* STATE MANAGEMENT */
    public:
        /**
         * @brief   Check if the transport is running
         * @return  true if running, false otherwise
         */
        virtual bool running() const = 0;

        /**
         * @brief   Get the current state of the transport
         * @return  The current state
         */
        virtual State state() const = 0;


    /* HALF-DUPLEX SUPPORT - FUTURE IMPLEMENTATION */
    private:
        /**
         * @brief   Check if the transport supports half-duplex 
         *          communication
         * @return  true if half-duplex is supported, false otherwise
         */
        virtual bool is_half_duplex() const {
            /* NOT IMPLEMENTED */
            return false;
        }

        /**
         * @brief   Set the transport to transmit mode
         * @return  SerialCommResult::OK on success
         * @return  SerialCommResult::ERR_NOT_SUPPORTED if the transport 
         *          does not support half-duplex communication
         */
        virtual errCode set_tx_mode(){
            /* NOT IMPLEMENTED */
            return errCode::ERR_NOT_SUPPORTED;
        }

        /**
         * @brief   Set the transport to receive mode
         * @return  SerialCommResult::OK on success
         * @return  SerialCommResult::ERR_NOT_SUPPORTED if the transport 
         *          does not support half-duplex communication
         */
        virtual errCode set_rx_mode(){
            /* NOT IMPLEMENTED */
            return errCode::ERR_NOT_SUPPORTED;
        }


    /* BUFFER MANAGEMENT */
    public:
        /**
         * @brief   Get the size of the receive buffer
         * @return  The size of the receive buffer in bytes
         */
        virtual size_t rx_buffer_size() const = 0;

        /**
         * @brief   Get the size of the transmit buffer
         * @return  The size of the transmit buffer in bytes
         */
        virtual size_t tx_buffer_size() const = 0;


    /* DESTRUCTOR */
    public:
        /**
         * @brief   Virtual destructor for interface
         */
        virtual ~ISerialCommTransport() = default;
};