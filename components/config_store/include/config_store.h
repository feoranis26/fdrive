#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct config_store_t config_store_t;

typedef enum {
    CONFIG_STORE_VALUE_TYPE_U32 = 1,
    CONFIG_STORE_VALUE_TYPE_I32 = 2,
    CONFIG_STORE_VALUE_TYPE_FLOAT = 3,
    CONFIG_STORE_VALUE_TYPE_BLOB = 4,
} config_store_value_type_t;

typedef struct {
    uint16_t key;
    config_store_value_type_t type;
    union {
        uint32_t u32;
        int32_t i32;
        float f32;
        struct {
            const void *data;
            size_t size;
        } blob;
    } value;
} config_store_default_entry_t;

typedef struct {
    const char *base_path;
    const char *file_path;
    const char *partition_label;
    uint32_t format_magic;
    uint32_t format_version;
    const config_store_default_entry_t *defaults;
    size_t default_count;
    bool format_if_mount_failed;
} config_store_config_t;

esp_err_t config_store_init(config_store_t **out_store, const config_store_config_t *config);
void config_store_deinit(config_store_t *store);

esp_err_t config_store_get_u32(config_store_t *store, uint16_t key, uint32_t *out_value);
esp_err_t config_store_set_u32(config_store_t *store, uint16_t key, uint32_t value);

esp_err_t config_store_get_i32(config_store_t *store, uint16_t key, int32_t *out_value);
esp_err_t config_store_set_i32(config_store_t *store, uint16_t key, int32_t value);

esp_err_t config_store_get_float(config_store_t *store, uint16_t key, float *out_value);
esp_err_t config_store_set_float(config_store_t *store, uint16_t key, float value);

esp_err_t config_store_get_blob(config_store_t *store, uint16_t key, void *out_buffer, size_t *inout_size);
esp_err_t config_store_set_blob(config_store_t *store, uint16_t key, const void *data, size_t size);

esp_err_t config_store_contains(config_store_t *store, uint16_t key, bool *out_present);
esp_err_t config_store_remove(config_store_t *store, uint16_t key);
esp_err_t config_store_reset_to_defaults(config_store_t *store);
esp_err_t config_store_save(config_store_t *store);

#ifdef __cplusplus
}
#endif
