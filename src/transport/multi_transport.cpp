/**
 * @file    multi_transport.cpp
 * @brief   MultiTransport composite implementation
 */

#include "serial_comm/transport/multi_transport.h"

using namespace SerialCommResult_Codes;

static const char* TAG = "MULTI_TRANSPORT";


/* =========================================================================
 * Constructor / Destructor
 * ========================================================================= */

MultiTransport::MultiTransport() :
    slot_count_(0),
    rx_callback_(nullptr),
    rx_ctx_(nullptr),
    tx_done_callback_(nullptr),
    tx_ctx_(nullptr),
    event_callback_(nullptr),
    event_ctx_(nullptr),
    state_(State::UNINITIALIZED)
{
    memset(slots_, 0, sizeof(slots_));
}


MultiTransport::~MultiTransport() {
    deinit();
}


/* =========================================================================
 * Slot management
 * ========================================================================= */

errCode MultiTransport::add_transport(
    ISerialCommTransport* transport,
    const SlotConfig& cfg
) {
    if (!transport) {
        return errCode::ERR_NULL_POINTER;
    }

    /* Reject duplicates */
    for (size_t i = 0; i < MAX_SLOTS; i++) {
        if (slots_[i].used && slots_[i].transport == transport) {
            ESP_LOGW(TAG, "Transport already registered in slot %zu", i);
            return errCode::ERR_ALREADY_INITIALIZED;
        }
    }

    /* Find empty slot */
    for (size_t i = 0; i < MAX_SLOTS; i++) {
        if (!slots_[i].used) {
            slots_[i].transport = transport;
            slots_[i].cfg       = cfg;
            slots_[i].used      = true;
            slot_count_++;

            /* If already initialised, register callbacks immediately */
            if (state_ != State::UNINITIALIZED) {
                register_slot_callbacks(i);
            }

            ESP_LOGI(TAG, "Transport added to slot %zu (rx=%d tx=%d)",
                     i, (int)cfg.rx_enabled, (int)cfg.tx_enabled);
            return errCode::OK;
        }
    }

    ESP_LOGE(TAG, "No free slots (max %zu)", MAX_SLOTS);
    return errCode::ERR_NO_MEMORY;
}


errCode MultiTransport::remove_transport(ISerialCommTransport* transport) {
    if (!transport) {
        return errCode::ERR_NULL_POINTER;
    }

    for (size_t i = 0; i < MAX_SLOTS; i++) {
        if (slots_[i].used && slots_[i].transport == transport) {
            slots_[i] = Slot{};
            slot_count_--;
            ESP_LOGI(TAG, "Transport removed from slot %zu", i);
            return errCode::OK;
        }
    }

    return errCode::OK; /* idempotent — removing an absent transport is fine */
}


/* =========================================================================
 * Lifecycle
 * ========================================================================= */

errCode MultiTransport::init(const Config& cfg) {
    if (state_ != State::UNINITIALIZED) {
        return errCode::ERR_ALREADY_INITIALIZED;
    }

    cfg_ = cfg;

    /* Register forwarders on every slot */
    for (size_t i = 0; i < MAX_SLOTS; i++) {
        if (slots_[i].used) {
            register_slot_callbacks(i);
        }
    }

    state_ = State::INITIALIZED;
    ESP_LOGI(TAG, "MultiTransport initialized (%zu slot(s))", slot_count_);
    return errCode::OK;
}


errCode MultiTransport::start() {
    if (state_ != State::INITIALIZED && state_ != State::STOPPED) {
        return errCode::ERR_INVALID_STATE;
    }

    state_ = State::RUNNING;

    if (event_callback_) {
        event_callback_(event_ctx_, Event::CONNECTED);
    }

    ESP_LOGI(TAG, "MultiTransport started");
    return errCode::OK;
}


errCode MultiTransport::stop() {
    if (state_ != State::RUNNING) {
        return errCode::ERR_INVALID_STATE;
    }

    state_ = State::STOPPED;

    if (event_callback_) {
        event_callback_(event_ctx_, Event::DISCONNECTED);
    }

    ESP_LOGI(TAG, "MultiTransport stopped");
    return errCode::OK;
}


errCode MultiTransport::deinit() {
    if (state_ == State::RUNNING) {
        stop();
    }
    state_ = State::UNINITIALIZED;
    ESP_LOGI(TAG, "MultiTransport deinitialized");
    return errCode::OK;
}


/* =========================================================================
 * Data transfer
 * ========================================================================= */

int MultiTransport::write(const uint8_t* data, size_t len) {
    if (state_ != State::RUNNING) {
        return -1;
    }
    if (!data || len == 0) {
        return 0;
    }

    int written = -1;

    for (size_t i = 0; i < MAX_SLOTS; i++) {
        if (!slots_[i].used || !slots_[i].cfg.tx_enabled) {
            continue;
        }
        int result = slots_[i].transport->write(data, len);
        if (result >= 0) {
            written = result; /* at least one transport succeeded */
        } else {
            ESP_LOGW(TAG, "write() failed on slot %zu", i);
        }
    }

    return written;
}


errCode MultiTransport::write_async(const uint8_t* data, size_t len) {
    if (state_ != State::RUNNING) {
        return errCode::ERR_INVALID_STATE;
    }
    if (!data || len == 0) {
        return errCode::ERR_INVALID_ARG;
    }

    errCode ret = errCode::ERR_FAIL;

    for (size_t i = 0; i < MAX_SLOTS; i++) {
        if (!slots_[i].used || !slots_[i].cfg.tx_enabled) {
            continue;
        }
        errCode result = slots_[i].transport->write_async(data, len);
        if (result == errCode::OK) {
            ret = errCode::OK; /* at least one transport succeeded */
        } else {
            ESP_LOGW(TAG, "write_async() failed on slot %zu", i);
        }
    }

    return ret;
}


int MultiTransport::read(uint8_t* data, size_t len, uint32_t timeout_ms) {
    if (!data || len == 0) {
        return -1;
    }

    /* Check each rx-enabled slot for immediately available data */
    for (size_t i = 0; i < MAX_SLOTS; i++) {
        if (!slots_[i].used || !slots_[i].cfg.rx_enabled) {
            continue;
        }
        if (slots_[i].transport->available() > 0) {
            return slots_[i].transport->read(data, len, timeout_ms);
        }
    }

    /* Nothing immediately available — block on first rx-enabled slot */
    for (size_t i = 0; i < MAX_SLOTS; i++) {
        if (!slots_[i].used || !slots_[i].cfg.rx_enabled) {
            continue;
        }
        return slots_[i].transport->read(data, len, timeout_ms);
    }

    return -1;
}


int MultiTransport::available() const {
    int total = 0;

    for (size_t i = 0; i < MAX_SLOTS; i++) {
        if (!slots_[i].used || !slots_[i].cfg.rx_enabled) {
            continue;
        }
        int avail = slots_[i].transport->available();
        if (avail > 0) {
            total += avail;
        }
    }

    return total;
}


errCode MultiTransport::flush() {
    for (size_t i = 0; i < MAX_SLOTS; i++) {
        if (slots_[i].used) {
            slots_[i].transport->flush();
        }
    }
    return errCode::OK;
}


/* =========================================================================
 * Callbacks
 * ========================================================================= */

errCode MultiTransport::set_rx_callback(rx_callback_t cb, void* ctx) {
    rx_callback_ = cb;
    rx_ctx_      = ctx;

    /* Re-register forwarder on all rx-enabled sub-transports so they point
     * to our forwarder (which now has the updated rx_callback_).            */
    for (size_t i = 0; i < MAX_SLOTS; i++) {
        if (slots_[i].used && slots_[i].cfg.rx_enabled) {
            slots_[i].transport->set_rx_callback(sub_rx_forwarder, this);
        }
    }

    return errCode::OK;
}


errCode MultiTransport::set_tx_done_callback(tx_done_callback_t cb, void* ctx) {
    tx_done_callback_ = cb;
    tx_ctx_           = ctx;
    return errCode::OK;
}


errCode MultiTransport::set_event_callback(event_callback_t cb, void* ctx) {
    event_callback_ = cb;
    event_ctx_      = ctx;

    /* MULTI-01 fix: was filtering on rx_enabled — TX-only transport errors
     * (e.g. ESP-NOW send failure, UART FIFO overflow on a write-only link)
     * were silently dropped.  Events must be forwarded from every slot. */
    for (size_t i = 0; i < MAX_SLOTS; i++) {
        if (slots_[i].used) {
            slots_[i].transport->set_event_callback(sub_event_forwarder, this);
        }
    }

    return errCode::OK;
}


/* =========================================================================
 * State
 * ========================================================================= */

bool MultiTransport::running() const {
    return state_ == State::RUNNING;
}

ISerialCommTransport::State MultiTransport::state() const {
    return state_;
}

size_t MultiTransport::rx_buffer_size() const {
    size_t total = 0;
    for (size_t i = 0; i < MAX_SLOTS; i++) {
        if (slots_[i].used && slots_[i].cfg.rx_enabled) {
            total += slots_[i].transport->rx_buffer_size();
        }
    }
    return total;
}

size_t MultiTransport::tx_buffer_size() const {
    size_t total = 0;
    for (size_t i = 0; i < MAX_SLOTS; i++) {
        if (slots_[i].used && slots_[i].cfg.tx_enabled) {
            total += slots_[i].transport->tx_buffer_size();
        }
    }
    return total;
}


/* =========================================================================
 * Private helpers
 * ========================================================================= */

void MultiTransport::register_slot_callbacks(size_t idx) {
    Slot& s = slots_[idx];
    if (!s.used) return;

    /* MULTI-01 fix: event forwarder must cover all slots, not just rx_enabled.
     * A TX-only slot can still surface errors (send failure, FIFO overflow). */
    s.transport->set_event_callback(sub_event_forwarder, this);

    if (s.cfg.rx_enabled) {
        s.transport->set_rx_callback(sub_rx_forwarder, this);
    }
    if (s.cfg.tx_enabled) {
        s.transport->set_tx_done_callback(sub_tx_done_forwarder, this);
    }
}


/* =========================================================================
 * Static forwarders
 * ========================================================================= */

void MultiTransport::sub_rx_forwarder(
    void* ctx, const uint8_t* data, size_t len
) {
    auto* self = static_cast<MultiTransport*>(ctx);
    if (self && self->rx_callback_) {
        self->rx_callback_(self->rx_ctx_, data, len);
    }
}


void MultiTransport::sub_tx_done_forwarder(void* ctx) {
    auto* self = static_cast<MultiTransport*>(ctx);
    if (self && self->tx_done_callback_) {
        self->tx_done_callback_(self->tx_ctx_);
    }
}


void MultiTransport::sub_event_forwarder(void* ctx, Event ev) {
    auto* self = static_cast<MultiTransport*>(ctx);
    if (self && self->event_callback_) {
        self->event_callback_(self->event_ctx_, ev);
    }
}
