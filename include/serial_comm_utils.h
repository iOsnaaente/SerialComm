/**
 * @file    serial_comm_utils.h
 * @brief   Utilities and common definitions for serial communication 
 *          middleware.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>


/**
 * @brief   Compute UART byte time in microseconds
 * @details Assiming a UART frame format of 8N1:
 *              - 1 start bit
 *              - 8 data bits
 *              - 1 stop bit
 *          Total = 10 bits per frame
 * @param baudrate Baud rate in bits per second
 * @return Time for one UART byte in microseconds
 */
static inline uint32_t uart_byte_time_us( uint32_t baudrate ) {
    return ( 10ULL * 1000000ULL ) / baudrate;
}


/**
 * @brief   Compute inter-byte timeout
 * @details Compute the time for inter-byte spacing based on baudrate 
 *          and character count.
 * @note    COnsidering 3.5 characters as a common threshold for 
 *          inter-byte timeout.
 */
static inline uint32_t uart_interbyte_timeout_us(
    uint32_t baudrate,
    float chars = 3.5f
){
    return static_cast<uint32_t>( uart_byte_time_us(baudrate) * chars );
}


/**
 * @brief   Namespace containing result codes for the serial communication 
 *          middleware.
 * 
 * @example To use the namespace, do:
 * 
 *              using namespace SerialCommResult_Codes;
 *              ESP_LOGI(
 *                  "SerialCommResult", 
 *                  "Result: %s", err_to_str( errCode::OK ) 
 *              );
 * 
 *          This will allow you to use the `errCode` enum and `err_to_str` 
 *          function without qualifying the `SerialCommResult` class, 
 *          while still keeping the `SerialCommResult` class available for 
 *          any helper functions or utilities.
 */
namespace SerialCommResult_Codes {


    /**
     * @brief   Generic result codes for the serial communication middleware
     *          This class defines a set of result codes that can be used 
     *          across the serial communication middleware to indicate the 
     *          outcome of operations, including success, various error 
     *          conditions, and specific errors related to the transport layer.
     */
    class SerialCommResult {
        public:
            /**
             * @brief Generic middleware result codes
             */
            enum errCode : int32_t {
                OK                          =  0,
                ERR_FAIL                    = -1,
                ERR_INVALID_ARG             = -2,
                ERR_INVALID_STATE           = -3,
                ERR_TIMEOUT                 = -4,
                ERR_NOT_SUPPORTED           = -5,
                ERR_NO_MEMORY               = -6,
                ERR_BUFFER_FULL             = -7,
                ERR_NOT_INITIALIZED         = -8,
                ERR_ALREADY_INITIALIZED     = -9,
                ERR_IO                      = -10,
                ERR_CRC                     = -11,
                ERR_PARSER                  = -12,
                ERR_DISCONNECTED            = -13,
                ERR_OVERFLOW                = -14,
                ERR_BUSY                    = -15,
                ERR_EMPTY                   = -16,
                ERR_NULL_POINTER            = -17
            };

        public:

            /**
             * @brief   Convert result code to string
             * @param   code Result code
             * @return  const char* Human readable string
             */
            static constexpr const char* err_to_str( errCode code ){
                switch ( code ) {
                    case OK:                        return "OK";
                    case ERR_FAIL:                  return "ERR_FAIL";
                    case ERR_INVALID_ARG:           return "ERR_INVALID_ARG";
                    case ERR_INVALID_STATE:         return "ERR_INVALID_STATE";
                    case ERR_TIMEOUT:               return "ERR_TIMEOUT";
                    case ERR_NOT_SUPPORTED:         return "ERR_NOT_SUPPORTED";
                    case ERR_NO_MEMORY:             return "ERR_NO_MEMORY";
                    case ERR_BUFFER_FULL:           return "ERR_BUFFER_FULL";
                    case ERR_NOT_INITIALIZED:       return "ERR_NOT_INITIALIZED";
                    case ERR_ALREADY_INITIALIZED:   return "ERR_ALREADY_INITIALIZED";
                    case ERR_IO:                    return "ERR_IO";
                    case ERR_CRC:                   return "ERR_CRC";
                    case ERR_PARSER:                return "ERR_PARSER";
                    case ERR_DISCONNECTED:          return "ERR_DISCONNECTED";
                    case ERR_OVERFLOW:              return "ERR_OVERFLOW";
                    case ERR_BUSY:                  return "ERR_BUSY";
                    case ERR_EMPTY:                 return "ERR_EMPTY";
                    case ERR_NULL_POINTER:          return "ERR_NULL_POINTER";
                    default:                        return "ERR_UNKNOWN";
                }
            }
    };

    /**
     * @brief   Make the nested enum available at namespace scope
     * @details This allows users of the namespace to refer to the enum 
     *          values as `errCode::OK` instead of `SerialCommResult::errCode::OK`
     */
    using errCode = SerialCommResult::errCode;

    /**
     * @brief   Helper function to convert result code to string at 
     *          namespace scope
     * @details This allows users of the namespace to call `err_to_str` 
     *          without qualifying it with `SerialCommResult::`
     * @example To use the helper function, do:
     *              `err_to_str( errCode::OK )` 
     *          instead of:
     *              `SerialCommResult::err_to_str( SerialCommResult::errCode::OK )` 
     */
    static constexpr const char* err_to_str( errCode code ) {
        return SerialCommResult::err_to_str(
            static_cast<SerialCommResult::errCode>(code)
        );
    }

} // namespace SerialCommResult_Codes