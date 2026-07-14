/**
 * @file    udp_serial_comm.cpp
 * @brief   WiFi UDP transport implementation for serial_comm middleware
 *
 * Bug fixes applied (2026-07-13):
 *   1. inet_addr("255.255.255.255") == INADDR_NONE — default broadcast was
 *      incorrectly rejected. Replaced inet_addr() with inet_pton() throughout.
 *   2. Static rx_buf in rx_task_entry was shared across instances.
 *      Now heap-allocated per task invocation and freed before self-delete.
 *   3. Double vTaskDelete — parent called vTaskDelete(rx_task_) while task
 *      also called vTaskDelete(nullptr). Fixed with rx_done_sem_: task signals
 *      it, parent waits (with fallback force-delete if task hangs).
 *   4. remote_addr_ and peers_[] had no mutex; concurrent set_remote_endpoint()
 *      + write() could produce a torn sockaddr. Added net_mutex_ for protection.
 *   5. state_mutex_ was created but never used. Removed; net_mutex_ replaces it.
 */

#include "serial_comm/transport/udp_serial_comm.h"
#include <stdlib.h>  /* malloc / free */

using namespace SerialCommResult_Codes;

static const char* TAG = "UDP_TRANSPORT";

/* Maximum time stop() waits for the RX task to self-exit cleanly (ms).
 * Must be > select() timeout (50 ms) + one recvfrom() round-trip. */
static constexpr uint32_t RX_TASK_EXIT_TIMEOUT_MS = 500;


/* ---- Constructor / Destructor ------------------------------------------- */

UDPTransport::UDPTransport(const HardwareConfig& hw_cfg)
    : hw_cfg_(hw_cfg)
    , cfg_{}
    , state_(State::UNINITIALIZED)
    , sock_(-1)
    , remote_addr_{}
    , rx_task_(nullptr)
    , poll_stream_(nullptr)
    , rx_callback_(nullptr)
    , tx_done_callback_(nullptr)
    , event_callback_(nullptr)
    , rx_ctx_(nullptr)
    , tx_ctx_(nullptr)
    , event_ctx_(nullptr)
    , net_mutex_(nullptr)
    , rx_done_sem_(nullptr)
    , peers_{}
{}


UDPTransport::~UDPTransport() {
    deinit();
}


/* ---- Lifecycle ----------------------------------------------------------- */

errCode UDPTransport::init(const Config& cfg) {
    if (state_ != State::UNINITIALIZED) {
        return errCode::ERR_ALREADY_INITIALIZED;
    }
    cfg_ = cfg;

    /* Bug-fix 4/5: replaced state_mutex_ (unused) with net_mutex_ */
    net_mutex_ = xSemaphoreCreateMutex();
    if (!net_mutex_) {
        ESP_LOGE(TAG, "Failed to create net_mutex_");
        return errCode::ERR_NO_MEMORY;
    }

    /* Bug-fix 3: semaphore for clean RX-task shutdown */
    rx_done_sem_ = xSemaphoreCreateBinary();
    if (!rx_done_sem_) {
        ESP_LOGE(TAG, "Failed to create rx_done_sem_");
        cleanup_resources();
        return errCode::ERR_NO_MEMORY;
    }

    /* StreamBuffer for polling read()/available() */
    poll_stream_ = xStreamBufferCreate(
        SerialCommConfig::UDP_RX_BUFFER_SIZE,
        1   /* trigger level: wake receiver on every byte */
    );
    if (!poll_stream_) {
        ESP_LOGE(TAG, "Failed to create poll StreamBuffer");
        cleanup_resources();
        return errCode::ERR_NO_MEMORY;
    }

    /* Create UDP socket */
    sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_ < 0) {
        ESP_LOGE(TAG, "socket() failed: errno=%d", errno);
        cleanup_resources();
        return errCode::ERR_FAIL;
    }

    /* SO_REUSEADDR — allow quick restart without TIME_WAIT issues */
    if (hw_cfg_.reuse_addr) {
        int opt = 1;
        if (setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
            ESP_LOGW(TAG, "SO_REUSEADDR failed: errno=%d", errno);
        }
    }

    /* SO_BROADCAST — required when remote_ip is a broadcast address */
    if (hw_cfg_.broadcast) {
        int opt = 1;
        if (setsockopt(sock_, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt)) < 0) {
            ESP_LOGE(TAG, "SO_BROADCAST failed: errno=%d", errno);
            cleanup_resources();
            return errCode::ERR_FAIL;
        }
    }

    /* Bind to local port; INADDR_ANY accepts packets from all interfaces */
    struct sockaddr_in local_addr = {};
    local_addr.sin_family         = AF_INET;
    local_addr.sin_port           = htons(hw_cfg_.local_port);
    local_addr.sin_addr.s_addr    = htonl(INADDR_ANY);

    if (bind(sock_, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        ESP_LOGE(TAG, "bind() on port %u failed: errno=%d",
                 hw_cfg_.local_port, errno);
        cleanup_resources();
        return errCode::ERR_FAIL;
    }

    /* Build default TX destination.
     *
     * Bug-fix 1: inet_addr("255.255.255.255") == INADDR_NONE == 0xFFFFFFFF,
     * so the INADDR_NONE check incorrectly rejected the broadcast address.
     * inet_pton() returns 1 on success, 0 on bad format, -1 on bad AF —
     * it does NOT encode any valid IPv4 address as an error sentinel. */
    remote_addr_            = {};
    remote_addr_.sin_family = AF_INET;
    remote_addr_.sin_port   = htons(hw_cfg_.remote_port);

    if (inet_pton(AF_INET, hw_cfg_.remote_ip, &remote_addr_.sin_addr) != 1) {
        ESP_LOGE(TAG, "Invalid remote IP: '%s'", hw_cfg_.remote_ip);
        cleanup_resources();
        return errCode::ERR_INVALID_PARAM;
    }

    state_ = State::INITIALIZED;
    ESP_LOGI(TAG, "UDP initialized  local_port=%u  remote=%s:%u",
             hw_cfg_.local_port, hw_cfg_.remote_ip, hw_cfg_.remote_port);
    return errCode::OK;
}


errCode UDPTransport::start() {
    if (state_ != State::INITIALIZED && state_ != State::STOPPED) {
        return errCode::ERR_INVALID_STATE;
    }

    /* Reset semaphore in case of restart after stop() */
    xSemaphoreTake(rx_done_sem_, 0);

    BaseType_t result;
    if (SerialCommConfig::UDP_USE_TASK_CORE_AFFINITY) {
        result = xTaskCreatePinnedToCore(
            rx_task_entry,
            "udp_rx_task",
            SerialCommConfig::UDP_TASK_STACK_SIZE,
            this,
            SerialCommConfig::UDP_TASK_PRIORITY,
            &rx_task_,
            SerialCommConfig::UDP_TASK_CORE
        );
    } else {
        result = xTaskCreate(
            rx_task_entry,
            "udp_rx_task",
            SerialCommConfig::UDP_TASK_STACK_SIZE,
            this,
            SerialCommConfig::UDP_TASK_PRIORITY,
            &rx_task_
        );
    }

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UDP RX task");
        return errCode::ERR_FAIL;
    }

    state_ = State::RUNNING;
    ESP_LOGI(TAG, "UDP transport started");
    return errCode::OK;
}


errCode UDPTransport::stop() {
    if (state_ != State::RUNNING) {
        return errCode::OK;
    }

    /* Signal the RX task to exit its select() loop */
    state_ = State::STOPPED;

    /* Bug-fix 3: wait for the task to signal rx_done_sem_ before self-deleting.
     * This avoids calling vTaskDelete() on a handle the task has already freed.
     * Timeout = RX_TASK_EXIT_TIMEOUT_MS > select() timeout (50 ms), so under
     * normal conditions the task exits within one select() period. */
    if (xSemaphoreTake(rx_done_sem_, pdMS_TO_TICKS(RX_TASK_EXIT_TIMEOUT_MS))
            == pdFALSE)
    {
        /* Task is stuck (e.g. recvfrom blocked without select, or socket error).
         * Force-delete as a safety net; small heap leak (rx_buf) is acceptable. */
        ESP_LOGW(TAG, "RX task did not exit in %lu ms — force deleting",
                 RX_TASK_EXIT_TIMEOUT_MS);
        if (rx_task_) {
            vTaskDelete(rx_task_);
        }
    }
    rx_task_ = nullptr;

    ESP_LOGI(TAG, "UDP transport stopped");
    return errCode::OK;
}


errCode UDPTransport::deinit() {
    stop();
    cleanup_resources();
    state_ = State::UNINITIALIZED;
    ESP_LOGI(TAG, "UDP transport deinitialized");
    return errCode::OK;
}


/* ---- Private helpers ----------------------------------------------------- */

void UDPTransport::cleanup_resources() {
    if (sock_ >= 0) {
        close(sock_);
        sock_ = -1;
    }
    if (poll_stream_) {
        vStreamBufferDelete(poll_stream_);
        poll_stream_ = nullptr;
    }
    if (rx_done_sem_) {
        vSemaphoreDelete(rx_done_sem_);
        rx_done_sem_ = nullptr;
    }
    if (net_mutex_) {
        vSemaphoreDelete(net_mutex_);
        net_mutex_ = nullptr;
    }
}


int UDPTransport::sendto_addr(
    const struct sockaddr_in& dest,
    const uint8_t*            data,
    size_t                    len
) {
    return sendto(
        sock_,
        data,
        len,
        0,
        (const struct sockaddr*)&dest,
        sizeof(dest)
    );
}


/* ---- RX task ------------------------------------------------------------- */

void UDPTransport::rx_task_entry(void* args) {
    auto* self = static_cast<UDPTransport*>(args);

    /* Bug-fix 2: was "static uint8_t rx_buf[...]" — static storage is shared
     * if multiple UDPTransport instances run concurrently.  Heap-allocate so
     * each task invocation owns its own buffer. */
    uint8_t* rx_buf = static_cast<uint8_t*>(malloc(UDP_MAX_RX_PACKET_SIZE));
    if (!rx_buf) {
        ESP_LOGE(TAG, "RX task: malloc failed — task exiting");
        xSemaphoreGive(self->rx_done_sem_);
        vTaskDelete(nullptr);
        return;
    }

    struct sockaddr_in src_addr;
    socklen_t src_len = sizeof(src_addr);

    while (self->state_ == State::RUNNING) {
        /* select() with 50 ms timeout keeps the state_ check responsive */
        fd_set rdset;
        FD_ZERO(&rdset);
        FD_SET(self->sock_, &rdset);
        struct timeval tv = { .tv_sec = 0, .tv_usec = 50000 };

        int sel = select(self->sock_ + 1, &rdset, nullptr, nullptr, &tv);
        if (sel == 0) {
            continue;   /* Timeout — loop back and re-check state_ */
        }
        if (sel < 0) {
            if (errno == EINTR) continue;
            ESP_LOGW(TAG, "select() error: errno=%d", errno);
            if (self->event_callback_) {
                self->event_callback_(self->event_ctx_, Event::ERROR);
            }
            break;
        }

        src_len = sizeof(src_addr);
        int len = recvfrom(
            self->sock_,
            rx_buf,
            UDP_MAX_RX_PACKET_SIZE,
            0,
            (struct sockaddr*)&src_addr,
            &src_len
        );

        if (len < 0) {
            ESP_LOGW(TAG, "recvfrom() error: errno=%d", errno);
            if (self->event_callback_) {
                self->event_callback_(self->event_ctx_, Event::ERROR);
            }
            continue;
        }
        if (len == 0) {
            continue;
        }

        /* Push into poll StreamBuffer for read()/available() */
        if (self->poll_stream_) {
            xStreamBufferSend(self->poll_stream_, rx_buf, (size_t)len, 0);
        }

        /* Fire the rx_callback registered by SerialCommParser/SerialComm */
        if (self->rx_callback_) {
            self->rx_callback_(self->rx_ctx_, rx_buf, (size_t)len);
        }
    }

    /* Bug-fix 2: free heap buffer before exiting */
    free(rx_buf);

    /* Bug-fix 3: signal stop() that this task is about to self-delete.
     * After this Give, stop() will NOT call vTaskDelete(rx_task_) — the task
     * owns its own deletion from here on. */
    xSemaphoreGive(self->rx_done_sem_);
    vTaskDelete(nullptr);
}


/* ---- Data transfer ------------------------------------------------------- */

int UDPTransport::write(const uint8_t* data, size_t len) {
    if (state_ != State::RUNNING) {
        return -1;
    }

    /* Bug-fix 4: copy remote_addr_ under lock so a concurrent
     * set_remote_endpoint() cannot produce a torn sockaddr_in. */
    struct sockaddr_in dest;
    xSemaphoreTake(net_mutex_, portMAX_DELAY);
    dest = remote_addr_;
    xSemaphoreGive(net_mutex_);

    int sent = sendto_addr(dest, data, len);
    if (sent < 0) {
        ESP_LOGW(TAG, "write() sendto failed: errno=%d", errno);
        if (event_callback_) {
            event_callback_(event_ctx_, Event::ERROR);
        }
        return -1;
    }

    if (tx_done_callback_) {
        tx_done_callback_(tx_ctx_);
    }
    return sent;
}


errCode UDPTransport::write_async(const uint8_t* data, size_t len) {
    if (state_ != State::RUNNING) {
        return errCode::ERR_INVALID_STATE;
    }

    /* Bug-fix 4: same address-copy-under-lock pattern as write() */
    struct sockaddr_in dest;
    xSemaphoreTake(net_mutex_, portMAX_DELAY);
    dest = remote_addr_;
    xSemaphoreGive(net_mutex_);

    int sent = sendto_addr(dest, data, len);
    if (sent < 0) {
        ESP_LOGW(TAG, "write_async() sendto failed: errno=%d", errno);
        return errCode::ERR_IO;
    }

    /* UDP is inherently fire-and-forget; tx_done fires immediately */
    if (tx_done_callback_) {
        tx_done_callback_(tx_ctx_);
    }
    return errCode::OK;
}


int UDPTransport::read(uint8_t* data, size_t len, uint32_t timeout_ms) {
    if (!poll_stream_) {
        return 0;
    }
    return (int)xStreamBufferReceive(
        poll_stream_,
        data,
        len,
        pdMS_TO_TICKS(timeout_ms)
    );
}


int UDPTransport::available() const {
    if (!poll_stream_) {
        return 0;
    }
    return (int)xStreamBufferBytesAvailable(poll_stream_);
}


errCode UDPTransport::flush() {
    if (!poll_stream_) {
        return errCode::OK;
    }
    /* xStreamBufferReset() is only safe when no task is blocked on the buffer.
     * In practice flush() is only called during shutdown / re-init, so this
     * is acceptable.  Do NOT call flush() while read() may be blocking. */
    xStreamBufferReset(poll_stream_);
    return errCode::OK;
}


/* ---- Callbacks ----------------------------------------------------------- */

errCode UDPTransport::set_rx_callback(rx_callback_t cb, void* ctx) {
    rx_callback_ = cb;
    rx_ctx_      = ctx;
    return errCode::OK;
}

errCode UDPTransport::set_tx_done_callback(tx_done_callback_t cb, void* ctx) {
    tx_done_callback_ = cb;
    tx_ctx_           = ctx;
    return errCode::OK;
}

errCode UDPTransport::set_event_callback(event_callback_t cb, void* ctx) {
    event_callback_ = cb;
    event_ctx_      = ctx;
    return errCode::OK;
}


/* ---- State --------------------------------------------------------------- */

bool UDPTransport::running() const {
    return state_ == State::RUNNING;
}

ISerialCommTransport::State UDPTransport::state() const {
    return state_;
}


/* ---- Buffer info --------------------------------------------------------- */

size_t UDPTransport::rx_buffer_size() const {
    return SerialCommConfig::UDP_RX_BUFFER_SIZE;
}

size_t UDPTransport::tx_buffer_size() const {
    /* No software TX buffer; sendto() hands bytes directly to LwIP. */
    return 0;
}


/* ---- UDP-specific API ---------------------------------------------------- */

void UDPTransport::set_remote_endpoint(const char* ip, uint16_t port) {
    struct in_addr tmp;
    if (inet_pton(AF_INET, ip, &tmp) != 1) {
        ESP_LOGW(TAG, "set_remote_endpoint: invalid IP '%s'", ip);
        return;
    }

    /* Bug-fix 4: hold net_mutex_ while writing remote_addr_ */
    xSemaphoreTake(net_mutex_, portMAX_DELAY);
    remote_addr_.sin_addr = tmp;
    remote_addr_.sin_port = htons(port);
    xSemaphoreGive(net_mutex_);

    ESP_LOGD(TAG, "Remote endpoint → %s:%u", ip, port);
}


errCode UDPTransport::add_peer(const char* ip, uint16_t port, uint8_t node_id) {
    /* Bug-fix 1: inet_pton instead of inet_addr (handles broadcast correctly) */
    struct in_addr tmp;
    if (inet_pton(AF_INET, ip, &tmp) != 1) {
        ESP_LOGW(TAG, "add_peer: invalid IP '%s'", ip);
        return errCode::ERR_INVALID_PARAM;
    }

    /* Bug-fix 4: protect the peers_ table */
    xSemaphoreTake(net_mutex_, portMAX_DELAY);

    for (size_t i = 0; i < UDP_MAX_PEERS; ++i) {
        if (peers_[i].used && peers_[i].node_id == node_id) {
            xSemaphoreGive(net_mutex_);
            return errCode::ERR_ALREADY_INITIALIZED;
        }
    }

    for (size_t i = 0; i < UDP_MAX_PEERS; ++i) {
        if (!peers_[i].used) {
            peers_[i].used              = true;
            peers_[i].node_id           = node_id;
            peers_[i].addr              = {};
            peers_[i].addr.sin_family   = AF_INET;
            peers_[i].addr.sin_port     = htons(port);
            peers_[i].addr.sin_addr     = tmp;
            xSemaphoreGive(net_mutex_);
            ESP_LOGI(TAG, "Peer added: node_id=%u  %s:%u", node_id, ip, port);
            return errCode::OK;
        }
    }

    xSemaphoreGive(net_mutex_);
    ESP_LOGW(TAG, "Peer table full (%zu slots)", UDP_MAX_PEERS);
    return errCode::ERR_NO_MEMORY;
}


errCode UDPTransport::remove_peer(uint8_t node_id) {
    xSemaphoreTake(net_mutex_, portMAX_DELAY);
    for (size_t i = 0; i < UDP_MAX_PEERS; ++i) {
        if (peers_[i].used && peers_[i].node_id == node_id) {
            peers_[i] = PeerEntry{};
            xSemaphoreGive(net_mutex_);
            ESP_LOGI(TAG, "Peer removed: node_id=%u", node_id);
            return errCode::OK;
        }
    }
    xSemaphoreGive(net_mutex_);
    return errCode::OK;   /* Idempotent */
}


int UDPTransport::write_to_peer(uint8_t node_id, const uint8_t* data, size_t len) {
    if (state_ != State::RUNNING) {
        return -1;
    }

    /* Bug-fix 4: copy peer address under lock, then send outside lock */
    struct sockaddr_in dest = {};
    bool found = false;

    xSemaphoreTake(net_mutex_, portMAX_DELAY);
    for (size_t i = 0; i < UDP_MAX_PEERS; ++i) {
        if (peers_[i].used && peers_[i].node_id == node_id) {
            dest  = peers_[i].addr;
            found = true;
            break;
        }
    }
    xSemaphoreGive(net_mutex_);

    if (!found) {
        ESP_LOGW(TAG, "write_to_peer: node_id=%u not found", node_id);
        return -1;
    }

    return sendto_addr(dest, data, len);
}


errCode UDPTransport::get_local_ip(char* buf, size_t buf_len) const {
    if (!buf || buf_len < 16) {
        return errCode::ERR_INVALID_PARAM;
    }

    /* Try STA interface first, then AP */
    const char* if_keys[] = { "WIFI_STA_DEF", "WIFI_AP_DEF" };
    for (const char* key : if_keys) {
        esp_netif_t* netif = esp_netif_get_handle_from_ifkey(key);
        if (!netif) continue;

        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK &&
            ip_info.ip.addr != 0)
        {
            esp_ip4addr_ntoa(&ip_info.ip, buf, (int)buf_len);
            return errCode::OK;
        }
    }

    ESP_LOGW(TAG, "get_local_ip: no IP assigned yet");
    return errCode::ERR_FAIL;
}
