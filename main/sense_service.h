#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "ads131m06.h"
#include "config_store.h"
#include "drive_config_store.h"
#include "esp_err.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct sense_service_t sense_service_t;

typedef struct {
    ads131m06_t *adc;
    config_store_t *config_store;
    uint32_t calibration_settle_ms;
    uint32_t calibration_samples;
    uint32_t calibration_sample_period_ms;
} sense_service_config_t;

typedef struct {
    uint32_t sample_count;
    uint16_t adc_status;
    float averaged_current_amps;
    float fast_current_amps;
    float current_amps;
    float bus_voltage_volts;
    float raw_current_channel_volts;
    float raw_voltage_channel_volts;
    bool calibrating;
} sense_service_snapshot_t;

typedef struct {
    esp_err_t result;
    float known_bus_voltage;
    float measured_bus_voltage;
} sense_service_calibration_result_t;

esp_err_t sense_service_init(sense_service_t **out_service, const sense_service_config_t *config);
void sense_service_deinit(sense_service_t *service);

esp_err_t sense_service_get_snapshot(sense_service_t *service, sense_service_snapshot_t *out_snapshot);
esp_err_t sense_service_refresh_snapshot(sense_service_t *service, sense_service_snapshot_t *out_snapshot);
bool sense_service_is_calibrating(sense_service_t *service);
esp_err_t sense_service_start_calibration(sense_service_t *service, float known_bus_voltage);
bool sense_service_take_calibration_result(sense_service_t *service, sense_service_calibration_result_t *out_result);

/**
 * Get DRDY interrupt count since last call (for diagnostic/rate measurement).
 */
uint32_t sense_service_get_drdy_count(sense_service_t *service);

/**
 * Read ADC CLOCK register for diagnostic verification of OSR/channel settings.
 */
uint16_t sense_service_get_clock_register(sense_service_t *service);

esp_err_t sense_service_register_data_ready_task(sense_service_t *service, TaskHandle_t task);

#ifdef __cplusplus
}
#endif