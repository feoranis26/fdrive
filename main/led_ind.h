#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct led_ind_t led_ind_t;

typedef struct {
    gpio_num_t strip_gpio;
    uint32_t rmt_resolution_hz;
    uint32_t pixel_count;
} led_ind_config_t;

esp_err_t led_ind_init(led_ind_t **out_led, const led_ind_config_t *config);
void led_ind_deinit(led_ind_t *led);

esp_err_t led_ind_set_color(led_ind_t *led, uint8_t r, uint8_t g, uint8_t b);
esp_err_t led_ind_set_off_color(led_ind_t *led, uint8_t r, uint8_t g, uint8_t b);
esp_err_t led_ind_set_blink(led_ind_t *led, float hz, float duty_cycle);

#ifdef __cplusplus
}
#endif
