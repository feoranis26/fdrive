#pragma once

#include "usb_can_bridge.h"

typedef usb_can_bridge_t can_serial_bridge_t;
typedef usb_can_bridge_config_t can_serial_bridge_config_t;

static inline esp_err_t can_serial_bridge_init(can_serial_bridge_t **out_bridge,
                                               const can_serial_bridge_config_t *config)
{
    return usb_can_bridge_init((usb_can_bridge_t **)out_bridge, (const usb_can_bridge_config_t *)config);
}

static inline void can_serial_bridge_deinit(can_serial_bridge_t *bridge)
{
    usb_can_bridge_deinit((usb_can_bridge_t *)bridge);
}