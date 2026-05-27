#pragma once

#include "serial_comm_config.h"
#include "serial_comm_utils.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"

#include "driver/gptimer.h"

#include <functional>
#include <stdint.h>
#include <stddef.h>
#include <string.h>


using namespace SerialCommUtils;

/**
 * @brief   Serial communication middleware watchdog timer
 * @details This class implements a watchdog timer using the ESP32's general 
 *          purpose timer (GPTimer) and FreeRTOS tasks. It allows you to set 
 *          a timeout duration and a callback function that will be called 
 *          when the timer expires. The timer can be started, stopped, and 
 *          kicked (reset) as needed.
 */
class SerialCommWatchdogTimer {
    public:
        /**
         * @brief   Callback function type for the watchdog
         * @details The callback function is called when the timer expires 
         *          and must be defined by the user to perform the desired 
         *          action.
         */
        using Callback = std::function<void()>;

    private:
        gptimer_handle_t    _timer = nullptr;
        TaskHandle_t        _taskHandle = nullptr;
        Callback            _callback;
        
        uint64_t            _timeout_us = 0;
        bool                _running = false;

        const char          *task_name;
        uint32_t            stack_size;
        UBaseType_t         priority;


        /**
         * @brief   Timer Callback Function
         * @details This function is called when the timer expires, and 
         *          it sends a notification to the watchdog task to call 
         *          the callback.    
         */
        static bool IRAM_ATTR timer_callback(
            gptimer_handle_t timer,
            const gptimer_alarm_event_data_t *data,
            void *user_ctx
        );


        /**
         * @brief   Watchdog task function
         * @details This function is executed by the watchdog task and 
         *          waits for timer notification events. When it receives 
         *          a notification, it calls the user-defined callback.
         */
        static void task_function(void *pvArg);


    public:
        /**
         * @brief   Constructor
         * @details This constructor initializes the watchdog timer with the
         *          specified parameters. It creates a FreeRTOS task for the
         *          watchdog and configures the GPTimer with the specified
         *          timeout and callback.
         * @param   task_name Name of the FreeRTOS task for the watchdog
         * @param   timeout_us Timeout duration in microseconds
         * @param   callback User-defined callback function to call on timeout
         * @param   stack_size Stack size for the FreeRTOS task
         * @param   priority Priority for the FreeRTOS task
         */
        SerialCommWatchdogTimer(
            const char* task_name   = "SerialCommWatchdog",
            uint64_t timeout_us     = uart_interbyte_timeout_us(SERIAL_COMM_UART_BAUDRATE),
            Callback callback       = nullptr,
            uint32_t stack_size     = 1024*4,
            UBaseType_t priority    = 5
        );

        
        /**
         * @brief   Start the watchdog timer
         * @details This function starts the watchdog timer. If the timer 
         *          is already running, it does nothing.
         * @return  ESP_OK if the timer was started successfully
         * @return  ESP_ERR_INVALID_STATE if the timer is already running
         * @return  Other error codes from the underlying GPTimer functions
         */ 
        esp_err_t start();


        /**
         * @brief   Stop the watchdog timer
         * @details This function stops the watchdog timer. If the timer 
         *          is not running, it does nothing.
         * @return  ESP_OK if the timer was stopped successfully
         * @return  ESP_ERR_INVALID_STATE if the timer is not running
         * @return  Other error codes from the underlying GPTimer functions
         */
        esp_err_t stop();


        /**
         * @brief   Kick (reset) the watchdog timer
         * @details This function resets the watchdog timer by stopping it,
         *          setting the timer count back to zero, and starting it 
         *          again.
         * @return  ESP_OK if the timer was kicked successfully
         * @return  Other error codes from the underlying GPTimer functions
         * @return  ESP_ERR_INVALID_STATE if the timer is not running
         */
        esp_err_t kick();


        /**
         * @brief   Destructor
         */
        ~SerialCommWatchdogTimer();
};