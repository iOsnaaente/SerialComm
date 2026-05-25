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
        ){
            (void)timer;
            (void)data;

            // Gets the user context
            SerialCommWatchdogTimer* usr_timer = 
                (SerialCommWatchdogTimer *)(user_ctx);

            // Send a notification to the watchdog task to call the callback
            BaseType_t high_task_wakeup = pdFALSE;
            vTaskNotifyGiveFromISR(
                usr_timer->_taskHandle,
                &high_task_wakeup
            );
            return (high_task_wakeup == pdTRUE);
        }

        /**
         * @brief   Watchdog task function
         * @details This function is executed by the watchdog task and 
         *          waits for timer notification events. When it receives 
         *          a notification, it calls the user-defined callback.
         */
        static void task_function(void *pvArg){
            SerialCommWatchdogTimer* timer = (SerialCommWatchdogTimer *)(pvArg);
            while (true) {
                ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
                timer->stop();
                if ( timer->_callback ) {
                    timer->_callback();
                }
            }
        }

    public:
        SerialCommWatchdogTimer(
            const char* task_name   = "SerialCommWatchdog",
            uint64_t timeout_us     = uart_interbyte_timeout_us(SERIAL_COMM_UART_BAUDRATE),
            Callback callback       = nullptr,
            uint32_t stack_size     = 1024*4,
            UBaseType_t priority    = 5
        ):
            _callback(callback),
            _timeout_us(timeout_us),
            task_name(task_name),
            stack_size(stack_size),
            priority(priority)
        {
            xTaskCreate(
                this->task_function,
                this->task_name,
                this->stack_size,
                this,
                this->priority,
                &this->_taskHandle
            );

            gptimer_config_t timer_config = {
                .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
                .direction     = GPTIMER_COUNT_UP,
                .resolution_hz = 1000000, // 1 tick = 1 microsecond
            };
            ESP_ERROR_CHECK(
                gptimer_new_timer(
                    &timer_config,
                    &_timer
                )
            );

            gptimer_event_callbacks_t callbacks = {
                .on_alarm = timer_callback,
            };
            ESP_ERROR_CHECK(
                gptimer_register_event_callbacks(
                    _timer,
                    &callbacks,
                    this
                )
            );

            gptimer_alarm_config_t alarm_config = {
                .alarm_count = timeout_us,
                .reload_count = 0,
                .flags = {
                    .auto_reload_on_alarm = false,
                }
            };
            ESP_ERROR_CHECK(
                gptimer_set_alarm_action(
                    _timer,
                    &alarm_config
                )
            );
            ESP_ERROR_CHECK(gptimer_enable(_timer));
        }


        esp_err_t start() {
            if ( this->_running) {
                return ESP_OK;
            }
            ESP_ERROR_CHECK(
                gptimer_set_raw_count(this->_timer, 0)
            );
            esp_err_t err = gptimer_start(this->_timer);
            if (err == ESP_OK) {
                this->_running = true;
            }
            return err;
        }


        esp_err_t stop(){
            if ( !this->_running ) {
                return ESP_OK;
            }
            esp_err_t err = gptimer_stop( this->_timer);
            if ( err == ESP_OK || err == ESP_ERR_INVALID_STATE ) {
                this->_running = false;
                return ESP_OK;
            }
            return err;
        }


        esp_err_t kick(){    
            ESP_ERROR_CHECK(this->stop());
            ESP_ERROR_CHECK(
                gptimer_set_raw_count(this->_timer, 0)
            );
            return this->start();
        }
                

        ~SerialCommWatchdogTimer(){
            this->stop();
            if (this->_timer != nullptr) {
                gptimer_disable(this->_timer);
                gptimer_del_timer(this->_timer);
                this->_timer = nullptr;
            }
            if (this->_taskHandle != nullptr) {
                vTaskDelete(this->_taskHandle);
                this->_taskHandle = nullptr;
            }
        }
};