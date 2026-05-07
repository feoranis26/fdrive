#include "sense_service.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define SENSE_SERVICE_CURRENT_CHANNEL 3U
#define SENSE_SERVICE_VOLTAGE_CHANNEL 4U
#define SENSE_SERVICE_CURRENT_SENSE_RESISTOR_OHMS 50e-6f
#define SENSE_SERVICE_CURRENT_SENSE_AMPLIFIER_GAIN 20.0f
#define SENSE_SERVICE_VOLTAGE_SENSE_RIN_OHMS 920000.0f
#define SENSE_SERVICE_VOLTAGE_SENSE_RBIAS_OHMS 22000.0f
#define SENSE_SERVICE_VOLTAGE_SENSE_RGND_OHMS 15000.0f
#define SENSE_SERVICE_VOLTAGE_SENSE_AINN_BIAS_VOLTS 1.2f
#define SENSE_SERVICE_FAST_CURRENT_ALPHA 0.5f
#define SENSE_SERVICE_SLOW_CURRENT_ALPHA 0.1f

struct sense_service_t {
    ads131m06_t *adc;
    config_store_t *config_store;
    SemaphoreHandle_t lock;
    TaskHandle_t calibration_task;
    uint32_t calibration_settle_ms;
    uint32_t calibration_samples;
    uint32_t calibration_sample_period_ms;
    bool calibrating;
    bool calibration_result_pending;
    float current_zero_channel_volts;
    float bus_voltage_offset_volts;
    bool current_inverted;
    float fast_current_amps;
    float slow_current_amps;
    bool current_filters_initialized;
    bool latest_snapshot_valid;
    sense_service_snapshot_t latest_snapshot;
    sense_service_calibration_result_t last_calibration_result;
};

static float sense_service_current_from_channel_volts(float channel_volts, float zero_offset_volts)
{
    return (channel_volts - zero_offset_volts) /
           (SENSE_SERVICE_CURRENT_SENSE_RESISTOR_OHMS * SENSE_SERVICE_CURRENT_SENSE_AMPLIFIER_GAIN);
}

static float sense_service_apply_current_sign(float current_amps, bool current_inverted)
{
    return current_inverted ? -current_amps : current_amps;
}

static float sense_service_bus_from_channel_volts(float channel_volts)
{
    const float conductance_sum = (1.0f / SENSE_SERVICE_VOLTAGE_SENSE_RIN_OHMS) +
                                  (1.0f / SENSE_SERVICE_VOLTAGE_SENSE_RBIAS_OHMS) +
                                  (1.0f / SENSE_SERVICE_VOLTAGE_SENSE_RGND_OHMS);
    const float input_gain = (1.0f / SENSE_SERVICE_VOLTAGE_SENSE_RIN_OHMS) / conductance_sum;
    const float bias_offset = ((SENSE_SERVICE_VOLTAGE_SENSE_AINN_BIAS_VOLTS / SENSE_SERVICE_VOLTAGE_SENSE_RBIAS_OHMS) /
                               conductance_sum) -
                              SENSE_SERVICE_VOLTAGE_SENSE_AINN_BIAS_VOLTS;

    return (channel_volts - bias_offset) / input_gain;
}

static esp_err_t sense_service_refresh_snapshot_locked(sense_service_t *service, sense_service_snapshot_t *out_snapshot)
{
    if (service == NULL || out_snapshot == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    float fast_channel_volts[ADS131M06_CHANNEL_COUNT] = {0};
    float slow_channel_volts[ADS131M06_CHANNEL_COUNT] = {0};
    esp_err_t err = ads131m06_get_avg_volts(service->adc, fast_channel_volts);
    if (err != ESP_OK) {
        return err;
    }
    /* slow averages used for telemetry */
    (void)ads131m06_get_slow_avg_volts(service->adc, slow_channel_volts);

    err = ads131m06_get_sample_count(service->adc, &out_snapshot->sample_count);
    if (err != ESP_OK) {
        return err;
    }

    err = ads131m06_get_last_status(service->adc, &out_snapshot->adc_status);
    if (err != ESP_OK) {
        return err;
    }

    float current_zero_channel_volts = 0.0f;
    float bus_voltage_offset_volts = 0.0f;
    bool current_inverted = false;
    bool calibrating = false;

    if (xSemaphoreTake(service->lock, portMAX_DELAY) == pdTRUE) {
        current_zero_channel_volts = service->current_zero_channel_volts;
        bus_voltage_offset_volts = service->bus_voltage_offset_volts;
        current_inverted = service->current_inverted;
        calibrating = service->calibrating;
        xSemaphoreGive(service->lock);
    }

    out_snapshot->raw_current_channel_volts = fast_channel_volts[SENSE_SERVICE_CURRENT_CHANNEL];
    out_snapshot->raw_voltage_channel_volts = fast_channel_volts[SENSE_SERVICE_VOLTAGE_CHANNEL];

    const float measured_current_fast_amps = sense_service_apply_current_sign(
        sense_service_current_from_channel_volts(out_snapshot->raw_current_channel_volts, current_zero_channel_volts),
        current_inverted);
    const float measured_current_slow_amps = sense_service_apply_current_sign(
        sense_service_current_from_channel_volts(slow_channel_volts[SENSE_SERVICE_CURRENT_CHANNEL], current_zero_channel_volts),
        current_inverted);
    out_snapshot->averaged_current_amps = measured_current_fast_amps;

    if (xSemaphoreTake(service->lock, portMAX_DELAY) == pdTRUE) {
        out_snapshot->fast_current_amps = measured_current_fast_amps;
        out_snapshot->current_amps = measured_current_slow_amps;
        out_snapshot->bus_voltage_volts = sense_service_bus_from_channel_volts(slow_channel_volts[SENSE_SERVICE_VOLTAGE_CHANNEL]) +
                                          bus_voltage_offset_volts;
        out_snapshot->calibrating = calibrating;
        service->latest_snapshot = *out_snapshot;
        service->latest_snapshot_valid = true;
        xSemaphoreGive(service->lock);
        return ESP_OK;
    }

    out_snapshot->fast_current_amps = measured_current_fast_amps;
    out_snapshot->current_amps = measured_current_slow_amps;
    out_snapshot->bus_voltage_volts = sense_service_bus_from_channel_volts(slow_channel_volts[SENSE_SERVICE_VOLTAGE_CHANNEL]) +
                                      bus_voltage_offset_volts;
    out_snapshot->calibrating = calibrating;
    return ESP_OK;
}

static void sense_service_calibration_task(void *arg)
{
    sense_service_t *service = (sense_service_t *)arg;
    float known_voltage = 0.0f;

    if (xSemaphoreTake(service->lock, portMAX_DELAY) == pdTRUE) {
        known_voltage = service->last_calibration_result.known_bus_voltage;
        xSemaphoreGive(service->lock);
    }

    vTaskDelay(pdMS_TO_TICKS(service->calibration_settle_ms));

    float accumulated_current_channel_volts = 0.0f;
    float accumulated_voltage_channel_volts = 0.0f;
    uint32_t collected_samples = 0U;

    for (uint32_t i = 0; i < service->calibration_samples; ++i) {
        float channel_volts[ADS131M06_CHANNEL_COUNT] = {0};
        if (ads131m06_get_avg_volts(service->adc, channel_volts) == ESP_OK) {
            accumulated_current_channel_volts += channel_volts[SENSE_SERVICE_CURRENT_CHANNEL];
            accumulated_voltage_channel_volts += channel_volts[SENSE_SERVICE_VOLTAGE_CHANNEL];
            collected_samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(service->calibration_sample_period_ms));
    }

    esp_err_t result = ESP_OK;
    float measured_bus_voltage = 0.0f;

    if (collected_samples == 0U) {
        result = ESP_FAIL;
    } else {
        const float avg_current_channel_volts = accumulated_current_channel_volts / (float)collected_samples;
        const float avg_voltage_channel_volts = accumulated_voltage_channel_volts / (float)collected_samples;
        measured_bus_voltage = sense_service_bus_from_channel_volts(avg_voltage_channel_volts);
        const float bus_voltage_offset = known_voltage - measured_bus_voltage;

        if (xSemaphoreTake(service->lock, portMAX_DELAY) == pdTRUE) {
            service->current_zero_channel_volts = avg_current_channel_volts;
            service->bus_voltage_offset_volts = bus_voltage_offset;
            xSemaphoreGive(service->lock);
        }

        if (drive_config_store_set_float(service->config_store, DRIVE_CONFIG_KEY_CURRENT_ZERO_CHANNEL_VOLTS, avg_current_channel_volts) != ESP_OK ||
            drive_config_store_set_float(service->config_store, DRIVE_CONFIG_KEY_BUS_VOLTAGE_OFFSET_VOLTS, bus_voltage_offset) != ESP_OK ||
            drive_config_store_save(service->config_store) != ESP_OK) {
            result = ESP_FAIL;
        }
    }

    if (xSemaphoreTake(service->lock, portMAX_DELAY) == pdTRUE) {
        service->last_calibration_result.result = result;
        service->last_calibration_result.measured_bus_voltage = measured_bus_voltage;
        service->calibration_result_pending = true;
        service->calibrating = false;
        service->calibration_task = NULL;
        xSemaphoreGive(service->lock);
    }

    vTaskDelete(NULL);
}

esp_err_t sense_service_init(sense_service_t **out_service, const sense_service_config_t *config)
{
    if (out_service == NULL || config == NULL || config->adc == NULL || config->config_store == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    sense_service_t *service = (sense_service_t *)calloc(1, sizeof(*service));
    if (service == NULL) {
        return ESP_ERR_NO_MEM;
    }

    service->adc = config->adc;
    service->config_store = config->config_store;
    service->calibration_settle_ms = (config->calibration_settle_ms == 0U) ? 1500U : config->calibration_settle_ms;
    service->calibration_samples = (config->calibration_samples == 0U) ? 64U : config->calibration_samples;
    service->calibration_sample_period_ms = (config->calibration_sample_period_ms == 0U) ? 25U : config->calibration_sample_period_ms;
    service->fast_current_amps = 0.0f;
    service->slow_current_amps = 0.0f;
    service->current_filters_initialized = false;
    service->latest_snapshot_valid = false;
    service->lock = xSemaphoreCreateMutex();
    if (service->lock == NULL) {
        free(service);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = sense_service_reload_config(service);
    if (err != ESP_OK) {
        vSemaphoreDelete(service->lock);
        free(service);
        return err;
    }

    *out_service = service;
    return ESP_OK;
}

esp_err_t sense_service_reload_config(sense_service_t *service)
{
    if (service == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    float current_zero_channel_volts = 0.0f;
    float bus_voltage_offset_volts = 0.0f;
    uint32_t current_inverted = 0U;

    esp_err_t err = drive_config_store_get_float(service->config_store,
                                                 DRIVE_CONFIG_KEY_CURRENT_ZERO_CHANNEL_VOLTS,
                                                 &current_zero_channel_volts);
    if (err != ESP_OK) {
        return err;
    }
    err = drive_config_store_get_float(service->config_store,
                                       DRIVE_CONFIG_KEY_BUS_VOLTAGE_OFFSET_VOLTS,
                                       &bus_voltage_offset_volts);
    if (err != ESP_OK) {
        return err;
    }
    err = drive_config_store_get_u32(service->config_store, DRIVE_CONFIG_KEY_CURRENT_INVERTED, &current_inverted);
    if (err != ESP_OK) {
        return err;
    }

    if (xSemaphoreTake(service->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    service->current_zero_channel_volts = current_zero_channel_volts;
    service->bus_voltage_offset_volts = bus_voltage_offset_volts;
    service->current_inverted = current_inverted != 0U;
    service->latest_snapshot_valid = false;
    xSemaphoreGive(service->lock);
    return ESP_OK;
}

void sense_service_deinit(sense_service_t *service)
{
    if (service == NULL) {
        return;
    }

    if (service->calibration_task != NULL) {
        vTaskDelete(service->calibration_task);
    }
    if (service->lock != NULL) {
        vSemaphoreDelete(service->lock);
    }
    free(service);
}

esp_err_t sense_service_get_snapshot(sense_service_t *service, sense_service_snapshot_t *out_snapshot)
{
    if (service == NULL || out_snapshot == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    bool have_snapshot = false;
    if (xSemaphoreTake(service->lock, portMAX_DELAY) == pdTRUE) {
        have_snapshot = service->latest_snapshot_valid;
        if (have_snapshot) {
            *out_snapshot = service->latest_snapshot;
        }
        xSemaphoreGive(service->lock);
    }

    if (have_snapshot) {
        return ESP_OK;
    }

    return sense_service_refresh_snapshot(service, out_snapshot);
}

esp_err_t sense_service_refresh_snapshot(sense_service_t *service, sense_service_snapshot_t *out_snapshot)
{
    if (service == NULL || out_snapshot == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return sense_service_refresh_snapshot_locked(service, out_snapshot);
    return ESP_OK;
}

bool sense_service_is_calibrating(sense_service_t *service)
{
    if (service == NULL) {
        return false;
    }

    bool calibrating = false;
    if (xSemaphoreTake(service->lock, portMAX_DELAY) == pdTRUE) {
        calibrating = service->calibrating;
        xSemaphoreGive(service->lock);
    }
    return calibrating;
}

esp_err_t sense_service_start_calibration(sense_service_t *service, float known_bus_voltage)
{
    if (service == NULL || !isfinite(known_bus_voltage)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(service->lock, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    if (service->calibrating) {
        xSemaphoreGive(service->lock);
        return ESP_ERR_INVALID_STATE;
    }

    service->calibrating = true;
    service->calibration_result_pending = false;
    service->last_calibration_result.result = ESP_ERR_NOT_FINISHED;
    service->last_calibration_result.known_bus_voltage = known_bus_voltage;
    service->last_calibration_result.measured_bus_voltage = 0.0f;
    xSemaphoreGive(service->lock);

    if (xTaskCreate(sense_service_calibration_task,
                    "sense_cal_task",
                    4096,
                    service,
                    5,
                    &service->calibration_task) != pdPASS) {
        if (xSemaphoreTake(service->lock, portMAX_DELAY) == pdTRUE) {
            service->calibrating = false;
            service->calibration_task = NULL;
            xSemaphoreGive(service->lock);
        }
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

bool sense_service_take_calibration_result(sense_service_t *service, sense_service_calibration_result_t *out_result)
{
    if (service == NULL || out_result == NULL) {
        return false;
    }

    bool have_result = false;
    if (xSemaphoreTake(service->lock, portMAX_DELAY) == pdTRUE) {
        if (service->calibration_result_pending) {
            *out_result = service->last_calibration_result;
            service->calibration_result_pending = false;
            have_result = true;
        }
        xSemaphoreGive(service->lock);
    }

    return have_result;
}

uint32_t sense_service_get_drdy_count(sense_service_t *service)
{
    if (service == NULL || service->adc == NULL) {
        return 0U;
    }
    return adc_sense_get_drdy_count((adc_sense_t *)service->adc);
}

uint16_t sense_service_get_clock_register(sense_service_t *service)
{
    if (service == NULL || service->adc == NULL) {
        return 0U;
    }
    uint16_t clock_reg = 0U;
    adc_sense_read_clock_register((adc_sense_t *)service->adc, &clock_reg);
    return clock_reg;
}

esp_err_t sense_service_register_data_ready_task(sense_service_t *service, TaskHandle_t task)
{
    if (service == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return ads131m06_register_data_ready_task(service->adc, task);
}