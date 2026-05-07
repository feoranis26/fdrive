#include "config_store.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <vector>

#include "esp_check.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "config_store";

namespace {

struct stored_entry_header_t {
    uint16_t key;
    uint16_t type;
    uint32_t size;
};

struct stored_file_header_t {
    uint32_t magic;
    uint32_t version;
    uint32_t entry_count;
    uint32_t crc32;
};

struct entry_t {
    uint16_t key = 0;
    config_store_value_type_t type = CONFIG_STORE_VALUE_TYPE_U32;
    std::vector<uint8_t> bytes;
};

static uint32_t config_store_crc32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFFU;

    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint32_t bit = 0; bit < 8U; ++bit) {
            const uint32_t mask = -(crc & 1U);
            crc = (crc >> 1) ^ (0xEDB88320U & mask);
        }
    }

    return ~crc;
}

static size_t config_store_expected_size_for_type(config_store_value_type_t type)
{
    switch (type) {
    case CONFIG_STORE_VALUE_TYPE_U32:
    case CONFIG_STORE_VALUE_TYPE_I32:
    case CONFIG_STORE_VALUE_TYPE_FLOAT:
        return sizeof(uint32_t);
    case CONFIG_STORE_VALUE_TYPE_BLOB:
        return 0U;
    default:
        return SIZE_MAX;
    }
}

} // namespace

struct config_store_t {
    SemaphoreHandle_t lock;
    config_store_config_t config;
    std::vector<entry_t> defaults;
    std::vector<entry_t> entries;
    bool mounted;
};

static entry_t config_store_entry_from_default(const config_store_default_entry_t &source)
{
    entry_t entry;
    entry.key = source.key;
    entry.type = source.type;

    const size_t fixed_size = config_store_expected_size_for_type(source.type);
    if (source.type == CONFIG_STORE_VALUE_TYPE_BLOB) {
        entry.bytes.resize(source.value.blob.size);
        if (source.value.blob.size > 0U && source.value.blob.data != nullptr) {
            memcpy(entry.bytes.data(), source.value.blob.data, source.value.blob.size);
        }
    } else if (fixed_size != SIZE_MAX) {
        entry.bytes.resize(fixed_size);
        switch (source.type) {
        case CONFIG_STORE_VALUE_TYPE_U32:
            memcpy(entry.bytes.data(), &source.value.u32, sizeof(source.value.u32));
            break;
        case CONFIG_STORE_VALUE_TYPE_I32:
            memcpy(entry.bytes.data(), &source.value.i32, sizeof(source.value.i32));
            break;
        case CONFIG_STORE_VALUE_TYPE_FLOAT:
            memcpy(entry.bytes.data(), &source.value.f32, sizeof(source.value.f32));
            break;
        default:
            break;
        }
    }

    return entry;
}

static entry_t *config_store_find_entry(config_store_t *store, uint16_t key)
{
    if (store == nullptr) {
        return nullptr;
    }

    for (entry_t &entry : store->entries) {
        if (entry.key == key) {
            return &entry;
        }
    }
    return nullptr;
}

static const entry_t *config_store_find_entry_const(const config_store_t *store, uint16_t key)
{
    if (store == nullptr) {
        return nullptr;
    }

    for (const entry_t &entry : store->entries) {
        if (entry.key == key) {
            return &entry;
        }
    }
    return nullptr;
}

static esp_err_t config_store_reset_to_defaults_locked(config_store_t *store)
{
    if (store == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    store->entries = store->defaults;
    return ESP_OK;
}

static bool config_store_entry_has_default_shape(const entry_t &entry, const entry_t &default_entry)
{
    return entry.type == default_entry.type && entry.bytes.size() == default_entry.bytes.size();
}

static bool config_store_merge_defaults_locked(config_store_t *store)
{
    if (store == nullptr) {
        return false;
    }

    bool changed = false;
    for (const entry_t &default_entry : store->defaults) {
        entry_t *entry = config_store_find_entry(store, default_entry.key);
        if (entry == nullptr) {
            store->entries.push_back(default_entry);
            changed = true;
        } else if (!config_store_entry_has_default_shape(*entry, default_entry)) {
            *entry = default_entry;
            changed = true;
        }
    }
    return changed;
}

static esp_err_t config_store_load_from_disk_locked(config_store_t *store)
{
    FILE *file = fopen(store->config.file_path, "rb");
    if (file == nullptr) {
        return ESP_ERR_NOT_FOUND;
    }

    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    if (file_size < (long)sizeof(stored_file_header_t)) {
        fclose(file);
        return ESP_FAIL;
    }
    rewind(file);

    std::vector<uint8_t> file_bytes((size_t)file_size);
    const size_t read_bytes = fread(file_bytes.data(), 1U, file_bytes.size(), file);
    fclose(file);
    if (read_bytes != file_bytes.size()) {
        return ESP_FAIL;
    }

    stored_file_header_t header = {};
    memcpy(&header, file_bytes.data(), sizeof(header));
    if (header.magic != store->config.format_magic || header.version != store->config.format_version) {
        return ESP_ERR_INVALID_VERSION;
    }

    const uint32_t stored_crc = header.crc32;
    header.crc32 = 0U;
    memcpy(file_bytes.data(), &header, sizeof(header));
    const uint32_t computed_crc = config_store_crc32(file_bytes.data(), file_bytes.size());
    if (stored_crc != computed_crc) {
        return ESP_ERR_INVALID_CRC;
    }

    size_t offset = sizeof(stored_file_header_t);
    std::vector<entry_t> loaded_entries;
    loaded_entries.reserve(header.entry_count);

    for (uint32_t i = 0; i < header.entry_count; ++i) {
        if ((offset + sizeof(stored_entry_header_t)) > file_bytes.size()) {
            return ESP_ERR_INVALID_SIZE;
        }

        stored_entry_header_t entry_header = {};
        memcpy(&entry_header, file_bytes.data() + offset, sizeof(entry_header));
        offset += sizeof(entry_header);

        if ((offset + entry_header.size) > file_bytes.size()) {
            return ESP_ERR_INVALID_SIZE;
        }

        entry_t entry;
        entry.key = entry_header.key;
        entry.type = (config_store_value_type_t)entry_header.type;
        entry.bytes.resize(entry_header.size);
        if (entry_header.size > 0U) {
            memcpy(entry.bytes.data(), file_bytes.data() + offset, entry_header.size);
        }
        offset += entry_header.size;

        const size_t expected_size = config_store_expected_size_for_type(entry.type);
        if (expected_size != SIZE_MAX && entry.type != CONFIG_STORE_VALUE_TYPE_BLOB && entry.bytes.size() != expected_size) {
            return ESP_ERR_INVALID_SIZE;
        }

        loaded_entries.push_back(std::move(entry));
    }

    store->entries = std::move(loaded_entries);
    return ESP_OK;
}

static esp_err_t config_store_save_locked(config_store_t *store)
{
    if (store == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    std::vector<uint8_t> bytes(sizeof(stored_file_header_t), 0U);
    stored_file_header_t header = {};
    header.magic = store->config.format_magic;
    header.version = store->config.format_version;
    header.entry_count = store->entries.size();
    memcpy(bytes.data(), &header, sizeof(header));

    for (const entry_t &entry : store->entries) {
        stored_entry_header_t entry_header = {};
        entry_header.key = entry.key;
        entry_header.type = (uint16_t)entry.type;
        entry_header.size = entry.bytes.size();

        const size_t old_size = bytes.size();
        bytes.resize(old_size + sizeof(entry_header) + entry.bytes.size());
        memcpy(bytes.data() + old_size, &entry_header, sizeof(entry_header));
        if (!entry.bytes.empty()) {
            memcpy(bytes.data() + old_size + sizeof(entry_header), entry.bytes.data(), entry.bytes.size());
        }
    }

    header.crc32 = 0U;
    memcpy(bytes.data(), &header, sizeof(header));
    header.crc32 = config_store_crc32(bytes.data(), bytes.size());
    memcpy(bytes.data(), &header, sizeof(header));

    FILE *file = fopen(store->config.file_path, "wb");
    if (file == nullptr) {
        return ESP_FAIL;
    }

    const size_t written = fwrite(bytes.data(), 1U, bytes.size(), file);
    fclose(file);

    return (written == bytes.size()) ? ESP_OK : ESP_FAIL;
}

extern "C" esp_err_t config_store_init(config_store_t **out_store, const config_store_config_t *config)
{
    if (out_store == nullptr || config == nullptr || config->base_path == nullptr || config->file_path == nullptr ||
        config->partition_label == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    config_store_t *store = new config_store_t();
    if (store == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    store->lock = xSemaphoreCreateMutex();
    if (store->lock == nullptr) {
        delete store;
        return ESP_ERR_NO_MEM;
    }

    store->config = *config;
    store->defaults.reserve(config->default_count);
    for (size_t i = 0; i < config->default_count; ++i) {
        store->defaults.push_back(config_store_entry_from_default(config->defaults[i]));
    }
    store->entries = store->defaults;

    esp_vfs_littlefs_conf_t littlefs_conf = {
        .base_path = store->config.base_path,
        .partition_label = store->config.partition_label,
        .partition = nullptr,
        .format_if_mount_failed = store->config.format_if_mount_failed,
        .read_only = false,
        .dont_mount = false,
        .grow_on_mount = false,
    };

    esp_err_t err = esp_vfs_littlefs_register(&littlefs_conf);
    if (err != ESP_OK) {
        vSemaphoreDelete(store->lock);
        delete store;
        return err;
    }
    store->mounted = true;

    if (xSemaphoreTake(store->lock, portMAX_DELAY) != pdTRUE) {
        config_store_deinit(store);
        return ESP_FAIL;
    }

    err = config_store_load_from_disk_locked(store);
    bool should_save = false;
    if (err != ESP_OK && err != ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "config load failed (%s), using defaults", esp_err_to_name(err));
        (void)config_store_reset_to_defaults_locked(store);
        should_save = true;
    }

    if (err == ESP_ERR_NOT_FOUND) {
        should_save = true;
    } else if (err == ESP_OK) {
        should_save = config_store_merge_defaults_locked(store);
    } else {
        err = ESP_OK;
    }

    if (should_save) {
        err = config_store_save_locked(store);
    } else {
        err = ESP_OK;
    }

    xSemaphoreGive(store->lock);

    if (err != ESP_OK) {
        config_store_deinit(store);
        return err;
    }

    *out_store = store;
    return ESP_OK;
}

extern "C" void config_store_deinit(config_store_t *store)
{
    if (store == nullptr) {
        return;
    }

    if (store->mounted) {
        esp_vfs_littlefs_unregister(store->config.partition_label);
    }
    if (store->lock != nullptr) {
        vSemaphoreDelete(store->lock);
    }
    delete store;
}

static esp_err_t config_store_get_scalar(config_store_t *store, uint16_t key, config_store_value_type_t type, void *out_value, size_t size)
{
    if (store == nullptr || out_value == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(store->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    const entry_t *entry = config_store_find_entry_const(store, key);
    esp_err_t err = ESP_OK;
    if (entry == nullptr) {
        err = ESP_ERR_NOT_FOUND;
    } else if (entry->type != type || entry->bytes.size() != size) {
        err = ESP_ERR_INVALID_ARG;
    } else {
        memcpy(out_value, entry->bytes.data(), size);
    }

    xSemaphoreGive(store->lock);
    return err;
}

static esp_err_t config_store_set_scalar(config_store_t *store, uint16_t key, config_store_value_type_t type, const void *value, size_t size)
{
    if (store == nullptr || value == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(store->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    entry_t *entry = config_store_find_entry(store, key);
    if (entry == nullptr) {
        store->entries.push_back({});
        entry = &store->entries.back();
        entry->key = key;
    }

    entry->type = type;
    entry->bytes.resize(size);
    memcpy(entry->bytes.data(), value, size);

    xSemaphoreGive(store->lock);
    return ESP_OK;
}

extern "C" esp_err_t config_store_get_u32(config_store_t *store, uint16_t key, uint32_t *out_value)
{
    return config_store_get_scalar(store, key, CONFIG_STORE_VALUE_TYPE_U32, out_value, sizeof(*out_value));
}

extern "C" esp_err_t config_store_set_u32(config_store_t *store, uint16_t key, uint32_t value)
{
    return config_store_set_scalar(store, key, CONFIG_STORE_VALUE_TYPE_U32, &value, sizeof(value));
}

extern "C" esp_err_t config_store_get_i32(config_store_t *store, uint16_t key, int32_t *out_value)
{
    return config_store_get_scalar(store, key, CONFIG_STORE_VALUE_TYPE_I32, out_value, sizeof(*out_value));
}

extern "C" esp_err_t config_store_set_i32(config_store_t *store, uint16_t key, int32_t value)
{
    return config_store_set_scalar(store, key, CONFIG_STORE_VALUE_TYPE_I32, &value, sizeof(value));
}

extern "C" esp_err_t config_store_get_float(config_store_t *store, uint16_t key, float *out_value)
{
    return config_store_get_scalar(store, key, CONFIG_STORE_VALUE_TYPE_FLOAT, out_value, sizeof(*out_value));
}

extern "C" esp_err_t config_store_set_float(config_store_t *store, uint16_t key, float value)
{
    return config_store_set_scalar(store, key, CONFIG_STORE_VALUE_TYPE_FLOAT, &value, sizeof(value));
}

extern "C" esp_err_t config_store_get_blob(config_store_t *store, uint16_t key, void *out_buffer, size_t *inout_size)
{
    if (store == nullptr || inout_size == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(store->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    const entry_t *entry = config_store_find_entry_const(store, key);
    esp_err_t err = ESP_OK;
    if (entry == nullptr) {
        err = ESP_ERR_NOT_FOUND;
    } else if (entry->type != CONFIG_STORE_VALUE_TYPE_BLOB) {
        err = ESP_ERR_INVALID_ARG;
    } else if (out_buffer == nullptr || *inout_size < entry->bytes.size()) {
        *inout_size = entry->bytes.size();
        err = ESP_ERR_INVALID_SIZE;
    } else {
        memcpy(out_buffer, entry->bytes.data(), entry->bytes.size());
        *inout_size = entry->bytes.size();
    }

    xSemaphoreGive(store->lock);
    return err;
}

extern "C" esp_err_t config_store_set_blob(config_store_t *store, uint16_t key, const void *data, size_t size)
{
    if (store == nullptr || (size > 0U && data == nullptr)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(store->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    entry_t *entry = config_store_find_entry(store, key);
    if (entry == nullptr) {
        store->entries.push_back({});
        entry = &store->entries.back();
        entry->key = key;
    }

    entry->type = CONFIG_STORE_VALUE_TYPE_BLOB;
    entry->bytes.resize(size);
    if (size > 0U) {
        memcpy(entry->bytes.data(), data, size);
    }

    xSemaphoreGive(store->lock);
    return ESP_OK;
}

extern "C" esp_err_t config_store_contains(config_store_t *store, uint16_t key, bool *out_present)
{
    if (store == nullptr || out_present == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(store->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    *out_present = (config_store_find_entry_const(store, key) != nullptr);
    xSemaphoreGive(store->lock);
    return ESP_OK;
}

extern "C" esp_err_t config_store_remove(config_store_t *store, uint16_t key)
{
    if (store == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(store->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    esp_err_t err = ESP_ERR_NOT_FOUND;
    for (auto it = store->entries.begin(); it != store->entries.end(); ++it) {
        if (it->key == key) {
            store->entries.erase(it);
            err = ESP_OK;
            break;
        }
    }

    xSemaphoreGive(store->lock);
    return err;
}

extern "C" esp_err_t config_store_reset_to_defaults(config_store_t *store)
{
    if (store == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(store->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    esp_err_t err = config_store_reset_to_defaults_locked(store);
    xSemaphoreGive(store->lock);
    return err;
}

extern "C" esp_err_t config_store_save(config_store_t *store)
{
    if (store == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(store->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    const esp_err_t err = config_store_save_locked(store);

    xSemaphoreGive(store->lock);
    return err;
}
