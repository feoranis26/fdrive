#include "drive_config_store.h"

esp_err_t drive_config_store_init(config_store_t **out_store, const drive_config_defaults_t *defaults)
{
    if (out_store == NULL || defaults == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const config_store_default_entry_t entries[] = {
        {
            .key = DRIVE_CONFIG_KEY_CAN_BASE_ID,
            .type = CONFIG_STORE_VALUE_TYPE_U32,
            .value = {.u32 = defaults->can_base_id},
        },
        {
            .key = DRIVE_CONFIG_KEY_CURRENT_ZERO_CHANNEL_VOLTS,
            .type = CONFIG_STORE_VALUE_TYPE_FLOAT,
            .value = {.f32 = defaults->current_zero_channel_volts},
        },
        {
            .key = DRIVE_CONFIG_KEY_BUS_VOLTAGE_OFFSET_VOLTS,
            .type = CONFIG_STORE_VALUE_TYPE_FLOAT,
            .value = {.f32 = defaults->bus_voltage_offset_volts},
        },
        {
            .key = DRIVE_CONFIG_KEY_CONTROL_MODE,
            .type = CONFIG_STORE_VALUE_TYPE_U32,
            .value = {.u32 = defaults->control_mode},
        },
        {
            .key = DRIVE_CONFIG_KEY_PWM_RAMP_UP_PER_SEC,
            .type = CONFIG_STORE_VALUE_TYPE_FLOAT,
            .value = {.f32 = defaults->pwm_ramp_up_per_sec},
        },
        {
            .key = DRIVE_CONFIG_KEY_PWM_BACKOFF_PER_SEC,
            .type = CONFIG_STORE_VALUE_TYPE_FLOAT,
            .value = {.f32 = defaults->pwm_backoff_per_sec},
        },
        {
            .key = DRIVE_CONFIG_KEY_PWM_ERROR_CLAMP,
            .type = CONFIG_STORE_VALUE_TYPE_FLOAT,
            .value = {.f32 = defaults->pwm_error_clamp},
        },
        {
            .key = DRIVE_CONFIG_KEY_CURRENT_LIMIT_AMPS,
            .type = CONFIG_STORE_VALUE_TYPE_FLOAT,
            .value = {.f32 = defaults->current_limit_amps},
        },
        {
            .key = DRIVE_CONFIG_KEY_CURRENT_KI_UP,
            .type = CONFIG_STORE_VALUE_TYPE_FLOAT,
            .value = {.f32 = defaults->current_ki_up},
        },
        {
            .key = DRIVE_CONFIG_KEY_CURRENT_KI_DOWN,
            .type = CONFIG_STORE_VALUE_TYPE_FLOAT,
            .value = {.f32 = defaults->current_ki_down},
        },
        {
            .key = DRIVE_CONFIG_KEY_CURRENT_OVERCURRENT_MARGIN_AMPS,
            .type = CONFIG_STORE_VALUE_TYPE_FLOAT,
            .value = {.f32 = defaults->current_overcurrent_margin_amps},
        },
        {
            .key = DRIVE_CONFIG_KEY_CURRENT_OVERCURRENT_MARGIN_PERCENT,
            .type = CONFIG_STORE_VALUE_TYPE_FLOAT,
            .value = {.f32 = defaults->current_overcurrent_margin_percent},
        },
        {
            .key = DRIVE_CONFIG_KEY_CURRENT_ERROR_CLAMP_AMPS,
            .type = CONFIG_STORE_VALUE_TYPE_FLOAT,
            .value = {.f32 = defaults->current_error_clamp_amps},
        },
        {
            .key = DRIVE_CONFIG_KEY_CURRENT_INVERTED,
            .type = CONFIG_STORE_VALUE_TYPE_U32,
            .value = {.u32 = defaults->current_inverted},
        },
    };

    const config_store_config_t config = {
        .base_path = "/littlefs",
        .file_path = "/littlefs/config.dat",
        .partition_label = "storage",
        .format_magic = 0x4B565354UL,
        .format_version = 1UL,
        .defaults = entries,
        .default_count = sizeof(entries) / sizeof(entries[0]),
        .format_if_mount_failed = true,
    };
    return config_store_init(out_store, &config);
}

esp_err_t drive_config_store_get_u32(config_store_t *store, drive_config_key_t key, uint32_t *out_value)
{
    return config_store_get_u32(store, (uint16_t)key, out_value);
}

esp_err_t drive_config_store_set_u32(config_store_t *store, drive_config_key_t key, uint32_t value)
{
    return config_store_set_u32(store, (uint16_t)key, value);
}

esp_err_t drive_config_store_get_float(config_store_t *store, drive_config_key_t key, float *out_value)
{
    return config_store_get_float(store, (uint16_t)key, out_value);
}

esp_err_t drive_config_store_set_float(config_store_t *store, drive_config_key_t key, float value)
{
    return config_store_set_float(store, (uint16_t)key, value);
}

esp_err_t drive_config_store_save(config_store_t *store)
{
    return config_store_save(store);
}