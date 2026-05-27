/**
 * @file    serial_comm_watchdog.cpp
 * @brief   Serial communication watchdog timer implementation
 * @author  Bruno Gabriel Flores Sampaio
 * @date    Created on 26 of May, 2026
 */


#pragma once

#include "serial_comm_watchdog.h"


static bool IRAM_ATTR SerialCommWatchdogTimer::timer_callback(
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
static void SerialCommWatchdogTimer::task_function(void *pvArg){
    SerialCommWatchdogTimer* timer = (SerialCommWatchdogTimer *)(pvArg);
    while (true) {
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
        timer->stop();
        if ( timer->_callback ) {
            timer->_callback();
        }
    }
}

SerialCommWatchdogTimer::SerialCommWatchdogTimer(
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


esp_err_t SerialCommWatchdogTimer::start() {
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


esp_err_t SerialCommWatchdogTimer::stop(){
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


esp_err_t SerialCommWatchdogTimer::kick(){    
    ESP_ERROR_CHECK(this->stop());
    ESP_ERROR_CHECK(
        gptimer_set_raw_count(this->_timer, 0)
    );
    return this->start();
}


SerialCommWatchdogTimer::~SerialCommWatchdogTimer(){
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
