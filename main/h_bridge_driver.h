#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct h_bridge_driver_t h_bridge_driver_t;

typedef enum {
    H_BRIDGE_DRIVE_ACCEL = 0,
    H_BRIDGE_DRIVE_BRAKE,
} h_bridge_drive_mode_t;

typedef struct {
    gpio_num_t enable_gpio;
    gpio_num_t a_hi_gpio;
    gpio_num_t a_lo_gpio;
    gpio_num_t b_hi_gpio;
    gpio_num_t b_lo_gpio;

    ledc_mode_t ledc_mode;
    ledc_timer_t ledc_timer;
    ledc_timer_bit_t duty_resolution;
    ledc_channel_t a_hi_channel;
    ledc_channel_t a_lo_channel;
    ledc_channel_t b_lo_channel;
    uint32_t pwm_frequency_hz;
} h_bridge_driver_config_t;

esp_err_t h_bridge_driver_init(h_bridge_driver_t **out_driver, const h_bridge_driver_config_t *config);
void h_bridge_driver_deinit(h_bridge_driver_t *driver);

esp_err_t h_bridge_set(h_bridge_driver_t *driver, h_bridge_drive_mode_t mode, float target_norm, uint32_t frequency_hz);
esp_err_t h_bridge_get_state(h_bridge_driver_t *driver,
                             h_bridge_drive_mode_t *out_mode,
                             float *out_target_norm,
                             uint32_t *out_frequency_hz,
                             bool *out_running);
bool h_bridge_is_running(h_bridge_driver_t *driver);

#ifdef __cplusplus
}
#endif
