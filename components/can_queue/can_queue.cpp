#include "can_queue.hpp"

#include <cstring>
#include <algorithm>

#include "esp_log.h"
#include "freertos/task.h"
#include "esp_twai_onchip.h"

static const char *TAG = "CanTransport";

static bool clone_frame(const twai_frame_t &src, twai_frame_t *dst)
{
    if (dst == nullptr) {
        return false;
    }

    *dst = src;
    dst->buffer = nullptr;
    if (src.buffer_len > 0) {
        dst->buffer = static_cast<uint8_t *>(malloc(src.buffer_len));
        if (dst->buffer == nullptr) {
            return false;
        }
        memcpy(dst->buffer, src.buffer, src.buffer_len);
    }

    return true;
}

void log_frame(twai_frame_t *frame) {
    ESP_EARLY_LOGI(TAG, "RX: %x [%d] %x %x %x %x %x %x %x %x", \
                     frame->header.id, frame->header.dlc, frame->buffer[0], frame->buffer[1], frame->buffer[2], frame->buffer[3], frame->buffer[4], frame->buffer[5], frame->buffer[6], frame->buffer[7]);
}

can_queue_t::can_queue_t(const can_queue_config_t &cfg) : cfg_(cfg) {}
can_queue_t::~can_queue_t() { (void)stop(); }

esp_err_t can_queue_t::start()
{
    if (running_)
        return ESP_OK;

    default_handle_ = new can_queue_handle_t(this);

    // App-facing queues
    tx_queue_ = xQueueCreate(cfg_.tx_queue_len, sizeof(twai_frame_t));

    ESP_ERROR_CHECK(twai_new_node_onchip(&cfg_.node_cfg, &node_));

    twai_event_callbacks_t cbs = {};
    cbs.on_rx_done = &can_queue_t::onRxDoneCb;
    cbs.on_error = &can_queue_t::onErrorCb;
    cbs.on_state_change = &can_queue_t::onStateChangeCb;

    ESP_ERROR_CHECK(twai_node_register_event_callbacks(node_, &cbs, this));

    ESP_ERROR_CHECK(twai_node_enable(node_));

    stop_requested_ = false;

    xTaskCreate(&can_queue_t::txTaskEntry, "can_tx_task", cfg_.tx_task_stack, this,
                cfg_.tx_task_prio, &tx_task_);

    xTaskCreate(&can_queue_t::recovery_task_entry, "can_recovery_task", 4096, this, 5, &recovery_task_);

    running_ = true;
    return ESP_OK;
}

esp_err_t can_queue_t::stop()
{
    ESP_LOGI(TAG, "destroying if");
    stop_requested_ = true;
    vTaskDelay(pdMS_TO_TICKS(10)); // Let tasks exit their loops

    // Stop tasks first
    if (tx_task_)
    {
        vTaskDelete(tx_task_);
        tx_task_ = nullptr;
    }

    if(recovery_task_)
    {
        vTaskDelete(recovery_task_);
        recovery_task_ = nullptr;
    }

    // Disable/delete node
    if (node_)
    {
        (void)twai_node_disable(node_);
        (void)twai_node_delete(node_);
        node_ = nullptr;
    }

    // Destroy queues
    if (tx_queue_)
    {
        twai_frame_t frame{};
        while (xQueueReceive(tx_queue_, &frame, 0) == pdTRUE)
        {
            free(frame.buffer);
        }
        vQueueDelete(tx_queue_);
        tx_queue_ = nullptr;
    }

    for (auto &desc : handle_descriptors_)
    {
        delete desc.handle;
    }
    handle_descriptors_.clear();

    delete default_handle_;
    default_handle_ = nullptr;

    running_ = false;
    return ESP_OK;
}

bool can_queue_t::enqueue_tx(const twai_frame_t &msg, TickType_t timeout)
{
    if (!tx_queue_)
        return false;

    twai_frame_t msg_copy = msg;
    msg_copy.buffer = nullptr;

    if (msg.buffer_len > 0)
    {
        if (!msg.buffer)
            return false;

        auto *copied = static_cast<uint8_t *>(malloc(msg.buffer_len));

        if (!copied)
            return false;

        memcpy(copied, msg.buffer, msg.buffer_len);
        msg_copy.buffer = copied;
    }

    if (xQueueSend(tx_queue_, &msg_copy, timeout) != pdTRUE)
    {
        ESP_LOGW(TAG, "TX queue full, dropping frame id=0x%08" PRIX32, msg.header.id);
        free(msg_copy.buffer);
        return false;
    }

    return true;
}

bool can_queue_t::inject_local_rx(const twai_frame_t &msg, TickType_t timeout)
{
    bool matched = false;
    bool delivered = false;

    for (const auto &desc : handle_descriptors_)
    {
        if (msg.header.id < desc.id_range_start || msg.header.id > desc.id_range_end) {
            continue;
        }

        matched = true;

        twai_frame_t queued_frame{};
        if (!clone_frame(msg, &queued_frame)) {
            continue;
        }

        if (xQueueSend(desc.handle->rx_queue(), &queued_frame, timeout) != pdTRUE)
        {
            free(queued_frame.buffer);
            continue;
        }

        delivered = true;
    }

    if (!matched && default_handle_ != nullptr)
    {
        twai_frame_t queued_frame{};
        if (!clone_frame(msg, &queued_frame)) {
            return delivered;
        }

        if (xQueueSend(default_handle_->rx_queue(), &queued_frame, timeout) != pdTRUE)
        {
            free(queued_frame.buffer);
            return delivered;
        }

        delivered = true;
    }

    return delivered;
}

bool can_queue_t::is_ok()
{
    twai_node_status_t st{};
    twai_node_record_t rec{};

    if (!node_) {
        ESP_LOGI(TAG, "no node found?");
        return false;
    }

    ESP_ERROR_CHECK(twai_node_get_info(node_, &st, &rec));

    return st.state != TWAI_ERROR_BUS_OFF;
}

bool can_queue_t::is_idle()
{
    return uxQueueMessagesWaiting(tx_queue_) == 0;
}

can_queue_handle_t *can_queue_t::get_handle(uint32_t id_start, uint32_t id_end)
{
    for (const auto &desc : handle_descriptors_)
    {
        if (desc.id_range_start == id_start && desc.id_range_end == id_end)
            return desc.handle;
    }

    auto *handle = new can_queue_handle_t(this, cfg_.rx_queue_len);
    handle_descriptors_.push_back({.handle = handle, .id_range_start = id_start, .id_range_end = id_end});
    return handle;
}

void can_queue_t::txTaskEntry(void *arg)
{
    static_cast<can_queue_t *>(arg)->txTask();
}

void can_queue_t::txTask()
{
    twai_frame_t frame{};
    while (!stop_requested_)
    {
        if (xQueueReceive(tx_queue_, &frame, portMAX_DELAY) != pdTRUE)
            continue;
        
        esp_err_t err = twai_node_transmit(node_, &frame, cfg_.tx_driver_timeout);
        twai_node_transmit_wait_all_done(node_ , -1);

        if (err == ESP_ERR_TIMEOUT)
        {
            ESP_LOGW(TAG, "TX timeout id=0x%08" PRIX32, frame.header.id);
        }
        else if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "TX error=%s id=0x%08" PRIX32, esp_err_to_name(err), frame.header.id);
        }

        if (err == ESP_OK && tx_tap_queue_ != nullptr) {
            twai_frame_t tapped{};
            if (clone_frame(frame, &tapped)) {
                if (xQueueSend(tx_tap_queue_, &tapped, 0) != pdTRUE) {
                    free(tapped.buffer);
                }
            }
        }

        free(frame.buffer);
        frame.buffer = nullptr;
    }

    vTaskDelete(nullptr);
}

void can_queue_t::recovery_task_entry(void *arg)
{
    static_cast<can_queue_t *>(arg)->recovery_task();
    vTaskDelete(NULL);
}

void can_queue_t::recovery_task()
{
    int fail_streak = 0;
    while (!stop_requested_) {
        twai_node_status_t st{};
        twai_node_record_t rec{};
        ESP_ERROR_CHECK(twai_node_get_info(node_, &st, &rec));

        //ESP_LOGI(TAG, "Recovery task: state=%d, tx_err=%d, rx_err=%d, bus_err_num=%u", st.state, st.tx_error_count, st.rx_error_count, rec.bus_err_num);

        if (st.state == TWAI_ERROR_BUS_OFF) {
            fail_streak++;
            ESP_LOGW(TAG, "Node in bus-off state, attempting recovery... (fail streak: %d)", fail_streak);
            esp_err_t err = twai_node_recover(node_);

            if (err != ESP_OK) {
                fail_streak++;
                uint32_t millis = std::min(1000, 50 * (1 << std::min(fail_streak, 9))); // Exponential backoff with cap
                ESP_LOGW(TAG, "Recovery attempt failed with error %s, retrying in %u ms (fail streak: %d)", esp_err_to_name(err), millis, fail_streak);
                vTaskDelay(pdMS_TO_TICKS(millis));

                continue;
            }
        }

        if (fail_streak > 0 && (st.state == TWAI_ERROR_ACTIVE || st.state == TWAI_ERROR_WARNING)) {
            fail_streak = 0;
            ESP_LOGI(TAG, "Node recovered to active state");
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

bool can_queue_t::onRxDoneCb(twai_node_handle_t node,
                          const twai_rx_done_event_data_t *edata,
                          void *user_ctx)
{
    (void)edata;
    auto *self = static_cast<can_queue_t *>(user_ctx);
    if (!self)
        return false;

    BaseType_t hp_task_woken = pdFALSE;

    uint8_t* buf = (uint8_t*)malloc(TWAI_FRAME_MAX_LEN);
    if (!buf)
    {
        ESP_EARLY_LOGW(TAG, "RX alloc failed");
        return hp_task_woken == pdTRUE;
    }

    twai_frame_t f = {};
    f.buffer = buf;
    f.buffer_len = TWAI_FRAME_MAX_LEN;

    esp_err_t err = twai_node_receive_from_isr(node, &f);
    if (err != ESP_OK)
    {
        free(buf);
        return hp_task_woken == pdTRUE; // no more frames right now
    }

    bool matched = false;
    bool original_delivered = false;

    for (const auto& desc : self->handle_descriptors_)
    {
        if (f.header.id < desc.id_range_start || f.header.id > desc.id_range_end) {
            continue;
        }

        matched = true;
        twai_frame_t queued_frame{};
        if (!original_delivered) {
            queued_frame = f;
            original_delivered = true;
        } else if (!clone_frame(f, &queued_frame)) {
            continue;
        }

        if (xQueueSendFromISR(desc.handle->rx_queue(), &queued_frame, &hp_task_woken) != pdTRUE)
        {
            free(queued_frame.buffer);
        }
    }

    if (!matched) {
        if (xQueueSendFromISR(self->default_handle_->rx_queue(), &f, &hp_task_woken) != pdTRUE)
        {
            free(buf);
            return hp_task_woken == pdTRUE;
        }
        original_delivered = true;
    }

    if (!original_delivered) {
        free(buf);
    }

    return hp_task_woken == pdTRUE;
}

bool can_queue_t::onErrorCb(twai_node_handle_t handle,
                         const twai_error_event_data_t *edata,
                         void *user_ctx)
{
    
    ESP_EARLY_LOGW(TAG, "bus error: 0x%x, ack: %d, arbitration: %d, bit: %d, form: %d, stuff: %d", edata->err_flags.val, edata->err_flags.ack_err, edata->err_flags.arb_lost, edata->err_flags.bit_err, edata->err_flags.form_err, edata->err_flags.stuff_err);
    return false;
}

bool can_queue_t::onStateChangeCb(twai_node_handle_t handle,
                               const twai_state_change_event_data_t *edata,
                               void *user_ctx)
{
    const char *twai_state_name[] = {"error_active", "error_warning", "error_passive", "bus_off"};
    ESP_EARLY_LOGI(TAG, "state changed: %s -> %s", twai_state_name[edata->old_sta], twai_state_name[edata->new_sta]);

    return false;
}

can_queue_handle_t::can_queue_handle_t(can_queue_t *can, size_t rx_queue_len)
{
    _can = can;
    _rx_queue = xQueueCreate(rx_queue_len, sizeof(twai_frame_t));
}

can_queue_handle_t::~can_queue_handle_t()
{
    if (_rx_queue)
    {
        twai_frame_t frame{};
        while (xQueueReceive(_rx_queue, &frame, 0) == pdTRUE)
        {
            free(frame.buffer);
        }
        vQueueDelete(_rx_queue);
        _rx_queue = nullptr;
    }
}

bool can_queue_handle_t::enqueue_tx(const twai_frame_t &msg, TickType_t timeout)
{
    return _can->enqueue_tx(msg, timeout);
}

bool can_queue_handle_t::dequeue_rx(twai_frame_t *out, TickType_t timeout)
{
    if (!_rx_queue || !out)
        return false;
    return xQueueReceive(_rx_queue, out, timeout) == pdTRUE;
}
