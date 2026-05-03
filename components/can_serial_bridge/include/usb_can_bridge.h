#pragma once

#include "can_queue.hpp"

#include "esp_err.h"

extern "C" {
#include "driver/uart.h"
}

typedef struct usb_can_bridge_t usb_can_bridge_t;

typedef struct {
    uart_port_t uart_port;
    can_queue_t *can_queue;
    can_queue_handle_t *sniffer_handle;
} usb_can_bridge_config_t;

esp_err_t usb_can_bridge_init(usb_can_bridge_t **out_bridge, const usb_can_bridge_config_t *config);
void usb_can_bridge_deinit(usb_can_bridge_t *bridge);