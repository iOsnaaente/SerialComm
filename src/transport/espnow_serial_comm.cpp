/**
 * @file    espnow_serial_comm.cpp
 * @brief   ESP-NOW transport implementation for serial_comm middleware
 */

#include "serial_comm/transport/espnow_serial_comm.h"

using namespace SerialCommResult_Codes;

static const char* TAG = "ESPNOW_TRANSPORT";

/* Global singleton — ESP-NOW callbacks have no user-context parameter */
ESPNowTransport* ESPNowTransport::instance_ = nullptr;


/* =========================================================================
 * Constructor / Destructor
 * ========================================================================= */

ESPNowTransport::ESPNowTransport(const HardwareConfig& hw_cfg) :
    hw_cfg_(hw_cfg),
    cfg_{},
    state_(State::UNINITIALIZED),
    tx_msg_seq_(0),
    tx_done_sem_(nullptr),
    tx_success_(false),
    rx_queue_(nullptr),
    rx_task_(nullptr),
    reassembly_mutex_(nullptr),
    poll_stream_(nullptr),
    rx_callback_(nullptr),
    tx_done_callback_(nullptr),
    event_callback_(nullptr),
    rx_ctx_(nullptr),
    tx_ctx_(nullptr),
    event_ctx_(nullptr),
    rx_done_sem_(nullptr)   /* ESPNOW-01/04: replaced unused state_mutex_ */
{
    memset(&reassembly_, 0, sizeof(reassembly_));
    memset(peers_,       0, sizeof(peers_));
}


ESPNowTransport::~ESPNowTransport() {
    deinit();
}


/* =========================================================================
 * Lifecycle
 * ========================================================================= */

errCode ESPNowTransport::init(const Config& cfg) {
    if (state_ != State::UNINITIALIZED) {
        ESP_LOGW(TAG, "Already initialized");
        return errCode::ERR_ALREADY_INITIALIZED;
    }

    cfg_ = cfg;

    /* ------------------------------------------------------------------
     * FreeRTOS objects
     * ------------------------------------------------------------------ */
    /* ESPNOW-03 fix: reject a second init() call while another instance exists.
     * Without this, the second instance overwrites instance_, silently
     * hijacking all ESP-NOW callbacks from the first. */
    if (instance_ != nullptr) {
        ESP_LOGE(TAG, "Only one ESPNowTransport instance allowed at a time");
        return errCode::ERR_ALREADY_INITIALIZED;
    }

    tx_done_sem_ = xSemaphoreCreateBinary();
    if (!tx_done_sem_) {
        ESP_LOGE(TAG, "Failed to create TX semaphore");
        return errCode::ERR_NO_MEMORY;
    }

    /* ESPNOW-01 fix: rx_done_sem_ replaces the unused state_mutex_ (ESPNOW-04).
     * Signals stop() that the RX task has exited cleanly before self-deleting,
     * preventing vTaskDelete() from being called while the task holds
     * reassembly_mutex_ (which would deadlock the next init()). */
    rx_done_sem_ = xSemaphoreCreateBinary();
    if (!rx_done_sem_) {
        ESP_LOGE(TAG, "Failed to create rx_done semaphore");
        cleanup_resources();
        return errCode::ERR_NO_MEMORY;
    }

    reassembly_mutex_ = xSemaphoreCreateMutex();
    if (!reassembly_mutex_) {
        cleanup_resources();
        return errCode::ERR_NO_MEMORY;
    }

    rx_queue_ = xQueueCreate(
        SerialCommConfig::ESPNOW_RX_QUEUE_SIZE,
        sizeof(RxQueueItem)
    );
    if (!rx_queue_) {
        ESP_LOGE(TAG, "Failed to create RX queue");
        cleanup_resources();
        return errCode::ERR_NO_MEMORY;
    }

    /* StreamBuffer for polling read() / available() */
    poll_stream_ = xStreamBufferCreate(
        ESPNOW_MAX_MSG_SIZE * 2,
        1                        /* trigger level = 1 byte */
    );
    if (!poll_stream_) {
        ESP_LOGE(TAG, "Failed to create poll stream buffer");
        cleanup_resources();
        return errCode::ERR_NO_MEMORY;
    }

    /* ------------------------------------------------------------------
     * ESP-NOW initialisation
     * ------------------------------------------------------------------ */
    esp_err_t err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_init failed: %s", esp_err_to_name(err));
        cleanup_resources();
        return errCode::ERR_FAIL;
    }

    /* Optional Primary Master Key */
    if (hw_cfg_.encrypt) {
        err = esp_now_set_pmk(hw_cfg_.pmk);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_now_set_pmk failed: %s", esp_err_to_name(err));
            esp_now_deinit();
            cleanup_resources();
            return errCode::ERR_FAIL;
        }
    }

    /* Register static callbacks (ESP-NOW has no user-context pointer) */
    instance_ = this;

    err = esp_now_register_recv_cb(recv_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_register_recv_cb failed: %s", esp_err_to_name(err));
        esp_now_deinit();
        instance_ = nullptr;
        cleanup_resources();
        return errCode::ERR_FAIL;
    }

    err = esp_now_register_send_cb(send_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_register_send_cb failed: %s", esp_err_to_name(err));
        esp_now_deinit();
        instance_ = nullptr;
        cleanup_resources();
        return errCode::ERR_FAIL;
    }

    /* Add default TX peer */
    if (!esp_now_is_peer_exist(hw_cfg_.tx_peer_mac)) {
        esp_now_peer_info_t peer_info = {};
        memcpy(peer_info.peer_addr, hw_cfg_.tx_peer_mac, 6);
        peer_info.channel = hw_cfg_.channel;
        peer_info.ifidx   = hw_cfg_.wifi_if;
        peer_info.encrypt = hw_cfg_.encrypt;
        if (hw_cfg_.encrypt) {
            memcpy(peer_info.lmk, hw_cfg_.lmk, sizeof(peer_info.lmk));
        }

        err = esp_now_add_peer(&peer_info);
        if (err != ESP_OK) {
            /* Non-fatal — user may add peers manually */
            ESP_LOGW(TAG, "Default peer add failed: %s", esp_err_to_name(err));
        }
    }

    /* Initialise reassembly state */
    memset(&reassembly_, 0, sizeof(reassembly_));

    state_ = State::INITIALIZED;
    ESP_LOGI(TAG, "ESP-NOW transport initialized");
    return errCode::OK;
}


errCode ESPNowTransport::start() {
    if (state_ != State::INITIALIZED && state_ != State::STOPPED) {
        ESP_LOGE(TAG, "Cannot start: invalid state");
        return errCode::ERR_INVALID_STATE;
    }

    /* Reset semaphore in case of restart after stop() */
    xSemaphoreTake(rx_done_sem_, 0);

    /* Start reassembly / dispatch task */
    BaseType_t task_result;

    if (SerialCommConfig::ESPNOW_TASK_CORE >= 0) {
        task_result = xTaskCreatePinnedToCore(
            rx_task_entry,
            "espnow_rx",
            SerialCommConfig::ESPNOW_TASK_STACK_SIZE,
            this,
            SerialCommConfig::ESPNOW_TASK_PRIORITY,
            &rx_task_,
            SerialCommConfig::ESPNOW_TASK_CORE
        );
    } else {
        task_result = xTaskCreate(
            rx_task_entry,
            "espnow_rx",
            SerialCommConfig::ESPNOW_TASK_STACK_SIZE,
            this,
            SerialCommConfig::ESPNOW_TASK_PRIORITY,
            &rx_task_
        );
    }

    if (task_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ESP-NOW RX task");
        return errCode::ERR_FAIL;
    }

    state_ = State::RUNNING;

    if (event_callback_) {
        event_callback_(event_ctx_, Event::CONNECTED);
    }

    ESP_LOGI(TAG, "ESP-NOW transport started");
    return errCode::OK;
}


errCode ESPNowTransport::stop() {
    if (state_ != State::RUNNING) {
        return errCode::ERR_INVALID_STATE;
    }

    /* ESPNOW-01 fix: signal the RX task to exit its queue-receive loop,
     * then wait for it to release reassembly_mutex_ and self-delete.
     * The old pattern (vTaskDelete while task may hold the mutex) could
     * deadlock the next init() because reassembly_mutex_ would never be Given. */
    state_ = State::STOPPED;

    if (xSemaphoreTake(rx_done_sem_, pdMS_TO_TICKS(500)) == pdFALSE) {
        /* Task is stuck — force-delete as safety net */
        ESP_LOGW(TAG, "RX task did not exit in 500 ms — force deleting");
        if (rx_task_) {
            vTaskDelete(rx_task_);
        }
    }
    rx_task_ = nullptr;

    if (event_callback_) {
        event_callback_(event_ctx_, Event::DISCONNECTED);
    }

    ESP_LOGI(TAG, "ESP-NOW transport stopped");
    return errCode::OK;
}


errCode ESPNowTransport::deinit() {
    if (state_ == State::RUNNING) {
        stop();
    }

    if (state_ != State::UNINITIALIZED) {
        esp_now_unregister_recv_cb();
        esp_now_unregister_send_cb();
        esp_now_deinit();
        instance_ = nullptr;
        state_    = State::UNINITIALIZED;
    }

    cleanup_resources();
    ESP_LOGI(TAG, "ESP-NOW transport deinitialized");
    return errCode::OK;
}


/* =========================================================================
 * Data Transfer
 * ========================================================================= */

int ESPNowTransport::write(const uint8_t* data, size_t len) {
    if (state_ != State::RUNNING) {
        ESP_LOGE(TAG, "write() called while not running");
        return -1;
    }
    if (!data || len == 0) {
        return 0;
    }

    size_t frag_count = (len + ESPNOW_FRAG_DATA_SIZE - 1) / ESPNOW_FRAG_DATA_SIZE;
    if (frag_count > ESPNOW_MAX_FRAGS) {
        ESP_LOGE(TAG, "Message too large: %zu bytes requires %zu fragments (max %u)",
                 len, frag_count, ESPNOW_MAX_FRAGS);
        return -1;
    }

    uint8_t msg_seq     = tx_msg_seq_++;
    uint8_t frag_total  = static_cast<uint8_t>(frag_count);
    size_t  offset      = 0;
    uint32_t timeout_ms = (cfg_.timeout_ms > 0) ? cfg_.timeout_ms : 200;

    for (uint8_t frag_idx = 0; frag_idx < frag_total; frag_idx++) {
        size_t chunk = len - offset;
        if (chunk > ESPNOW_FRAG_DATA_SIZE) {
            chunk = ESPNOW_FRAG_DATA_SIZE;
        }

        errCode err = send_fragment(
            data + offset,
            chunk,
            msg_seq,
            frag_idx,
            frag_total
        );
        if (err != errCode::OK) {
            ESP_LOGE(TAG, "send_fragment() failed at frag %u/%u", frag_idx + 1, frag_total);
            return -1;
        }

        /* Wait for ESP-NOW send-done callback */
        if (xSemaphoreTake(tx_done_sem_, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
            ESP_LOGE(TAG, "TX timeout on fragment %u/%u", frag_idx + 1, frag_total);
            return -1;
        }

        if (!tx_success_) {
            ESP_LOGE(TAG, "TX failed on fragment %u/%u", frag_idx + 1, frag_total);
            return -1;
        }

        offset += chunk;
    }

    ESP_LOGD(TAG, "Sent %zu bytes in %u fragment(s), seq=%u", len, frag_total, msg_seq);
    return static_cast<int>(len);
}


errCode ESPNowTransport::write_async(const uint8_t* data, size_t len) {
    if (state_ != State::RUNNING) {
        return errCode::ERR_INVALID_STATE;
    }
    if (!data || len == 0) {
        return errCode::ERR_INVALID_ARG;
    }

    size_t frag_count = (len + ESPNOW_FRAG_DATA_SIZE - 1) / ESPNOW_FRAG_DATA_SIZE;
    if (frag_count > ESPNOW_MAX_FRAGS) {
        ESP_LOGE(TAG, "write_async: message too large (%zu bytes)", len);
        return errCode::ERR_OVERFLOW;
    }

    uint8_t msg_seq    = tx_msg_seq_++;
    uint8_t frag_total = static_cast<uint8_t>(frag_count);
    size_t  offset     = 0;

    for (uint8_t frag_idx = 0; frag_idx < frag_total; frag_idx++) {
        size_t chunk = len - offset;
        if (chunk > ESPNOW_FRAG_DATA_SIZE) {
            chunk = ESPNOW_FRAG_DATA_SIZE;
        }

        errCode err = send_fragment(
            data + offset,
            chunk,
            msg_seq,
            frag_idx,
            frag_total
        );
        if (err != errCode::OK) {
            return err;
        }

        /* Even in async mode we must wait for the send-done callback before
         * queuing the next fragment: esp_now_send() rejects a second call
         * while the previous one is still outstanding (ESP_ERR_ESPNOW_NO_MEM).
         * ESPNOW-02 fix: was ignoring the return value — if the 100ms window
         * elapsed (congested Wi-Fi), the next esp_now_send() would fail and
         * the message would be silently truncated. Now we abort explicitly. */
        if (xSemaphoreTake(tx_done_sem_, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGW(TAG, "write_async: TX timeout on fragment %u/%u — aborting",
                     frag_idx + 1, frag_total);
            return errCode::ERR_TIMEOUT;
        }

        offset += chunk;
    }

    return errCode::OK;
}


int ESPNowTransport::read(uint8_t* data, size_t len, uint32_t timeout_ms) {
    if (!poll_stream_ || !data || len == 0) {
        return -1;
    }
    size_t received = xStreamBufferReceive(
        poll_stream_,
        data,
        len,
        pdMS_TO_TICKS(timeout_ms)
    );
    return static_cast<int>(received);
}


int ESPNowTransport::available() const {
    if (!poll_stream_) {
        return 0;
    }
    return static_cast<int>(xStreamBufferBytesAvailable(poll_stream_));
}


errCode ESPNowTransport::flush() {
    if (poll_stream_) {
        xStreamBufferReset(poll_stream_);
    }
    if (rx_queue_) {
        xQueueReset(rx_queue_);
    }
    return errCode::OK;
}


/* =========================================================================
 * Callbacks
 * ========================================================================= */

errCode ESPNowTransport::set_rx_callback(rx_callback_t cb, void* ctx) {
    rx_callback_ = cb;
    rx_ctx_      = ctx;
    return errCode::OK;
}

errCode ESPNowTransport::set_tx_done_callback(tx_done_callback_t cb, void* ctx) {
    tx_done_callback_ = cb;
    tx_ctx_           = ctx;
    return errCode::OK;
}

errCode ESPNowTransport::set_event_callback(event_callback_t cb, void* ctx) {
    event_callback_ = cb;
    event_ctx_      = ctx;
    return errCode::OK;
}


/* =========================================================================
 * State
 * ========================================================================= */

bool ESPNowTransport::running() const {
    return state_ == State::RUNNING;
}

ISerialCommTransport::State ESPNowTransport::state() const {
    return state_;
}

size_t ESPNowTransport::rx_buffer_size() const {
    return ESPNOW_MAX_MSG_SIZE * 2;
}

size_t ESPNowTransport::tx_buffer_size() const {
    return ESPNOW_MAX_MSG_SIZE;
}


/* =========================================================================
 * ESP-NOW Specific API
 * ========================================================================= */

errCode ESPNowTransport::add_peer(
    const uint8_t* mac,
    uint8_t        node_id,
    uint8_t        channel,
    bool           encrypt,
    const uint8_t* lmk
) {
    if (!mac) {
        return errCode::ERR_INVALID_ARG;
    }

    /* Register with ESP-NOW */
    if (!esp_now_is_peer_exist(mac)) {
        esp_now_peer_info_t info = {};
        memcpy(info.peer_addr, mac, 6);
        info.channel = channel;
        info.ifidx   = hw_cfg_.wifi_if;
        info.encrypt = encrypt;
        if (encrypt && lmk) {
            memcpy(info.lmk, lmk, sizeof(info.lmk));
        }

        esp_err_t err = esp_now_add_peer(&info);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_now_add_peer failed: %s", esp_err_to_name(err));
            return errCode::ERR_FAIL;
        }
    }

    /* Store in internal peer table */
    for (size_t i = 0; i < ESPNOW_MAX_PEERS; i++) {
        if (!peers_[i].used) {
            peers_[i].used    = true;
            peers_[i].node_id = node_id;
            memcpy(peers_[i].mac, mac, 6);
            return errCode::OK;
        }
    }

    ESP_LOGW(TAG, "Peer table full (max %zu)", ESPNOW_MAX_PEERS);
    return errCode::ERR_NO_MEMORY;
}


errCode ESPNowTransport::remove_peer(const uint8_t* mac) {
    if (!mac) {
        return errCode::ERR_INVALID_ARG;
    }

    esp_err_t err = esp_now_del_peer(mac);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_del_peer failed: %s", esp_err_to_name(err));
        return errCode::ERR_FAIL;
    }

    for (size_t i = 0; i < ESPNOW_MAX_PEERS; i++) {
        if (peers_[i].used && memcmp(peers_[i].mac, mac, 6) == 0) {
            memset(&peers_[i], 0, sizeof(PeerEntry));
            return errCode::OK;
        }
    }
    return errCode::OK; /* not in table is fine */
}


void ESPNowTransport::set_tx_peer(const uint8_t* mac) {
    if (mac) {
        memcpy(hw_cfg_.tx_peer_mac, mac, 6);
    }
}


errCode ESPNowTransport::get_local_mac(uint8_t* out_mac) const {
    if (!out_mac) {
        return errCode::ERR_INVALID_ARG;
    }
    esp_err_t err = esp_wifi_get_mac(hw_cfg_.wifi_if, out_mac);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_get_mac failed: %s", esp_err_to_name(err));
        return errCode::ERR_FAIL;
    }
    return errCode::OK;
}


/* =========================================================================
 * Static ESP-NOW callbacks
 * ========================================================================= */

void ESPNowTransport::recv_cb(
    const esp_now_recv_info_t* recv_info,
    const uint8_t*             data,
    int                        data_len
) {
    if (!instance_ || !recv_info || !data || data_len <= 0) {
        return;
    }
    if (!instance_->rx_queue_) {
        return;
    }

    RxQueueItem item;
    memcpy(item.src_mac, recv_info->src_addr, 6);

    size_t copy_len = static_cast<size_t>(data_len);
    if (copy_len > ESPNOW_MAX_FRAME_SIZE) {
        copy_len = ESPNOW_MAX_FRAME_SIZE;
    }
    memcpy(item.data, data, copy_len);
    item.data_len = static_cast<uint16_t>(copy_len);

    /* Called from Wi-Fi task — use non-ISR queue send with zero timeout */
    if (xQueueSend(instance_->rx_queue_, &item, 0) != pdTRUE) {
        ESP_LOGW(TAG, "RX queue full — frame dropped");
        if (instance_->event_callback_) {
            instance_->event_callback_(instance_->event_ctx_, Event::BUFFER_FULL);
        }
    }
}


void ESPNowTransport::send_cb(
    const esp_now_send_info_t* send_info,
    esp_now_send_status_t      status
) {
    if (!instance_) {
        return;
    }

    instance_->tx_success_ = (status == ESP_NOW_SEND_SUCCESS);
    xSemaphoreGive(instance_->tx_done_sem_);

    if (status == ESP_NOW_SEND_SUCCESS) {
        if (instance_->tx_done_callback_) {
            instance_->tx_done_callback_(instance_->tx_ctx_);
        }
    } else {
        if (send_info && send_info->des_addr) {
            const uint8_t* mac = send_info->des_addr;
            ESP_LOGW(TAG, "ESP-NOW send failed for peer %02X:%02X:%02X:%02X:%02X:%02X",
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        } else {
            ESP_LOGW(TAG, "ESP-NOW send failed (unknown peer)");
        }
        if (instance_->event_callback_) {
            instance_->event_callback_(instance_->event_ctx_, Event::ERROR);
        }
    }
}


/* =========================================================================
 * RX task
 * ========================================================================= */

void ESPNowTransport::rx_task_entry(void* args) {
    auto* self = static_cast<ESPNowTransport*>(args);
    RxQueueItem item;

    /* ESPNOW-01 fix: replaced infinite loop + portMAX_DELAY with a state-
     * aware loop and 50ms timeout.  The task now exits cleanly when stop()
     * sets state_ = STOPPED, instead of being killed mid-mutex by vTaskDelete. */
    while (self->state_ == State::RUNNING) {
        if (xQueueReceive(self->rx_queue_, &item, pdMS_TO_TICKS(50)) == pdTRUE) {
            self->process_rx_item(item);
        }
    }

    /* Signal stop() that it is now safe to return (mutex is not held) */
    xSemaphoreGive(self->rx_done_sem_);
    vTaskDelete(nullptr);
}


void ESPNowTransport::process_rx_item(const RxQueueItem& item) {
    /* Validate minimum size */
    if (item.data_len < ESPNOW_FRAG_HDR_SIZE) {
        ESP_LOGW(TAG, "RX frame too short (%u bytes) — discarded", item.data_len);
        return;
    }

    const FragHeader* hdr = reinterpret_cast<const FragHeader*>(item.data);

    /* Validate magic bytes */
    if (hdr->magic[0] != ESPNOW_FRAG_MAGIC_0 || hdr->magic[1] != ESPNOW_FRAG_MAGIC_1) {
        ESP_LOGW(TAG, "RX frame: bad magic (0x%02X 0x%02X) — discarded",
                 hdr->magic[0], hdr->magic[1]);
        return;
    }

    /* Validate fragment geometry */
    if (hdr->frag_total == 0 || hdr->frag_idx >= hdr->frag_total) {
        ESP_LOGW(TAG, "RX frame: bad frag geometry idx=%u total=%u",
                 hdr->frag_idx, hdr->frag_total);
        return;
    }

    size_t payload_len = static_cast<size_t>(hdr->frag_data_len);
    if (payload_len == 0 || payload_len > ESPNOW_FRAG_DATA_SIZE) {
        ESP_LOGW(TAG, "RX frame: bad payload_len=%zu", payload_len);
        return;
    }
    if (item.data_len < static_cast<uint16_t>(ESPNOW_FRAG_HDR_SIZE + payload_len)) {
        ESP_LOGW(TAG, "RX frame truncated (header says %zu bytes, got %u)",
                 ESPNOW_FRAG_HDR_SIZE + payload_len, item.data_len);
        return;
    }

    const uint8_t* frag_data = item.data + ESPNOW_FRAG_HDR_SIZE;

    xSemaphoreTake(reassembly_mutex_, portMAX_DELAY);

    /* ------------------------------------------------------------------
     * Fragment 0: begin a new message
     * ------------------------------------------------------------------ */
    if (hdr->frag_idx == 0) {
        if (reassembly_.active) {
            /* Abandon previous incomplete message */
            ESP_LOGW(TAG, "Incomplete message (seq=%u, %u/%u frags) abandoned — new seq=%u",
                     reassembly_.msg_seq,
                     reassembly_.frags_received,
                     reassembly_.frags_expected,
                     hdr->msg_seq);
        }
        memset(&reassembly_, 0, sizeof(reassembly_));
        reassembly_.active          = true;
        reassembly_.msg_seq         = hdr->msg_seq;
        reassembly_.frags_expected  = hdr->frag_total;
        reassembly_.frags_received  = 0;
        reassembly_.total_written   = 0;
    } else {
        /* Validate that this fragment belongs to the active message */
        if (!reassembly_.active || hdr->msg_seq != reassembly_.msg_seq) {
            ESP_LOGW(TAG, "Orphan fragment seq=%u idx=%u — discarded",
                     hdr->msg_seq, hdr->frag_idx);
            xSemaphoreGive(reassembly_mutex_);
            return;
        }
        /* Check sequence order */
        if (hdr->frag_idx != reassembly_.frags_received) {
            ESP_LOGW(TAG, "Out-of-order fragment idx=%u expected=%u — reset",
                     hdr->frag_idx, reassembly_.frags_received);
            memset(&reassembly_, 0, sizeof(reassembly_));
            xSemaphoreGive(reassembly_mutex_);
            return;
        }
    }

    /* ------------------------------------------------------------------
     * Append fragment payload
     * ------------------------------------------------------------------ */
    if (reassembly_.total_written + payload_len > ESPNOW_MAX_MSG_SIZE) {
        ESP_LOGE(TAG, "Reassembly buffer overflow — resetting");
        memset(&reassembly_, 0, sizeof(reassembly_));
        xSemaphoreGive(reassembly_mutex_);
        return;
    }

    memcpy(
        reassembly_.buf + reassembly_.total_written,
        frag_data,
        payload_len
    );
    reassembly_.total_written  += payload_len;
    reassembly_.frags_received += 1;

    ESP_LOGD(TAG, "RX frag %u/%u seq=%u (%zu bytes)",
             hdr->frag_idx + 1, hdr->frag_total,
             hdr->msg_seq, payload_len);

    /* ------------------------------------------------------------------
     * Check if message is complete
     * ------------------------------------------------------------------ */
    if (reassembly_.frags_received < reassembly_.frags_expected) {
        /* Message still incomplete */
        xSemaphoreGive(reassembly_mutex_);
        return;
    }

    /* Reset only the reassembly metadata — keep buf intact.
     * rx_task is the sole consumer of rx_queue_, so reassembly_.buf cannot
     * be overwritten until we return and dequeue the next item.
     * Both xStreamBufferSend() and rx_callback_() copy the data internally,
     * so it is safe to release the mutex before calling them.           */
    size_t msg_len = reassembly_.total_written;

    reassembly_.active         = false;
    reassembly_.frags_received = 0;
    reassembly_.frags_expected = 0;
    reassembly_.total_written  = 0;
    xSemaphoreGive(reassembly_mutex_);

    ESP_LOGD(TAG, "RX complete message: seq=%u, %zu bytes", hdr->msg_seq, msg_len);

    /* Push to polling stream buffer (copies internally) */
    if (poll_stream_) {
        xStreamBufferSend(poll_stream_, reassembly_.buf, msg_len, 0);
    }

    /* Invoke the upper-layer RX callback (SerialComm::process_rx_data) */
    if (rx_callback_) {
        rx_callback_(rx_ctx_, reassembly_.buf, msg_len);
    }
}


/* =========================================================================
 * Private helpers
 * ========================================================================= */

errCode ESPNowTransport::send_fragment(
    const uint8_t* data,
    size_t         len,
    uint8_t        msg_seq,
    uint8_t        frag_idx,
    uint8_t        frag_total
) {
    uint8_t frame[ESPNOW_MAX_FRAME_SIZE];

    auto* hdr         = reinterpret_cast<FragHeader*>(frame);
    hdr->magic[0]     = ESPNOW_FRAG_MAGIC_0;
    hdr->magic[1]     = ESPNOW_FRAG_MAGIC_1;
    hdr->msg_seq      = msg_seq;
    hdr->frag_idx     = frag_idx;
    hdr->frag_total   = frag_total;
    hdr->frag_data_len = static_cast<uint16_t>(len);

    memcpy(frame + ESPNOW_FRAG_HDR_SIZE, data, len);

    size_t    frame_len = ESPNOW_FRAG_HDR_SIZE + len;
    esp_err_t err       = esp_now_send(hw_cfg_.tx_peer_mac, frame, frame_len);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_send failed: %s", esp_err_to_name(err));
        return errCode::ERR_IO;
    }
    return errCode::OK;
}


void ESPNowTransport::cleanup_resources() {
    if (rx_task_) {
        vTaskDelete(rx_task_);
        rx_task_ = nullptr;
    }
    if (rx_queue_) {
        vQueueDelete(rx_queue_);
        rx_queue_ = nullptr;
    }
    if (poll_stream_) {
        vStreamBufferDelete(poll_stream_);
        poll_stream_ = nullptr;
    }
    if (tx_done_sem_) {
        vSemaphoreDelete(tx_done_sem_);
        tx_done_sem_ = nullptr;
    }
    if (reassembly_mutex_) {
        vSemaphoreDelete(reassembly_mutex_);
        reassembly_mutex_ = nullptr;
    }
    /* ESPNOW-04 fix: state_mutex_ was declared but never used (88 bytes wasted).
     * Replaced by rx_done_sem_ (ESPNOW-01) which is functional. */
    if (rx_done_sem_) {
        vSemaphoreDelete(rx_done_sem_);
        rx_done_sem_ = nullptr;
    }
}
