#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include <atomic>
#include <vector>

extern "C" {
#include "driver/gpio.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
}

struct can_queue_config_t {
    twai_onchip_node_config_t node_cfg{};

    uint16_t tx_queue_len = 64;
    uint16_t rx_queue_len = 64;

    uint32_t tx_task_stack = 4096;
    uint32_t rx_task_stack = 4096;
    UBaseType_t tx_task_prio = 10;
    UBaseType_t rx_task_prio = 10;
    
    int tx_driver_timeout = pdMS_TO_TICKS(20);
    TickType_t rx_driver_timeout = pdMS_TO_TICKS(50);
};

void log_frame(twai_frame_t *frame);

class can_queue_t;

class can_queue_handle_t {
public:
    can_queue_handle_t(can_queue_t *can, size_t rx_queue_len = 16);
    ~can_queue_handle_t();

    // TX: enqueue_tx deep-copies msg.buffer payload; caller retains ownership.
    bool enqueue_tx(const twai_frame_t& msg, TickType_t timeout = 0);

    // RX: dequeue_rx returns a frame whose buffer is heap-allocated by can_queue_t;
    // caller owns out->buffer and must free() it when done.
    bool dequeue_rx(twai_frame_t* out, TickType_t timeout = portMAX_DELAY);

    QueueHandle_t rx_queue() const { return _rx_queue; }
private:
    QueueHandle_t _rx_queue;
    can_queue_t *_can;
};

struct can_queue_handle_descriptor_t {
    can_queue_handle_t* handle;
    uint32_t id_range_start;
    uint32_t id_range_end;
};

class can_queue_t {
public:
    explicit can_queue_t(const can_queue_config_t& cfg);
    ~can_queue_t();

    esp_err_t start();
    esp_err_t stop();

    // TX: enqueue_tx deep-copies msg.buffer payload; caller retains ownership.
    bool enqueue_tx(const twai_frame_t& msg, TickType_t timeout = 0);

    // Inject a frame directly into local RX consumers without requiring bus loopback.
    bool inject_local_rx(const twai_frame_t& msg, TickType_t timeout = 0);

    bool is_ok();
    bool is_idle();
    void set_tx_tap_queue(QueueHandle_t queue) { tx_tap_queue_ = queue; }

    can_queue_handle_t* get_handle(uint32_t id_start, uint32_t id_end);
    can_queue_handle_t* default_handle() { return default_handle_; }

private:
    static void txTaskEntry(void* arg);
    void txTask();

    static void recovery_task_entry(void* arg);
    void recovery_task();

    static bool onRxDoneCb(twai_node_handle_t node,
                           const twai_rx_done_event_data_t *edata,
                           void *user_ctx);

    static bool onErrorCb(twai_node_handle_t node,
                          const twai_error_event_data_t *edata,
                          void *user_ctx);

    static bool onStateChangeCb(twai_node_handle_t node,
                                const twai_state_change_event_data_t *edata,
                                void *user_ctx);

    can_queue_config_t cfg_;
    QueueHandle_t tx_queue_ = nullptr;
    TaskHandle_t tx_task_ = nullptr;
    TaskHandle_t recovery_task_ = nullptr;

    std::vector<can_queue_handle_descriptor_t> handle_descriptors_;
    can_queue_handle_t* default_handle_ = nullptr;
    

    twai_node_handle_t node_ = nullptr;
    QueueHandle_t tx_tap_queue_ = nullptr;

    volatile bool stop_requested_ = false;
    bool running_ = false;
};
