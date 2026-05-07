#pragma once

#include <stdint.h>

#include "config_store.h"
#include "drive_protocol.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t can_base_id;
    float current_zero_channel_volts;
    float bus_voltage_offset_volts;
    uint32_t control_mode;
    float pwm_ramp_up_per_sec;
    float pwm_backoff_per_sec;
    float pwm_error_clamp;
    float current_limit_amps;
    float current_ki_up;
    float current_ki_down;
    float current_overcurrent_margin_amps;
    float current_overcurrent_margin_percent;
    float current_error_clamp_amps;
    uint32_t current_inverted;
} drive_config_defaults_t;

esp_err_t drive_config_store_init(config_store_t **out_store, const drive_config_defaults_t *defaults);
esp_err_t drive_config_store_get_u32(config_store_t *store, drive_config_key_t key, uint32_t *out_value);
esp_err_t drive_config_store_set_u32(config_store_t *store, drive_config_key_t key, uint32_t value);
esp_err_t drive_config_store_get_float(config_store_t *store, drive_config_key_t key, float *out_value);
esp_err_t drive_config_store_set_float(config_store_t *store, drive_config_key_t key, float value);
esp_err_t drive_config_store_save(config_store_t *store);

#ifdef __cplusplus
}
#endif