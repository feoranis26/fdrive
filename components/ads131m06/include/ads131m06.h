#pragma once

#include "adc_sense.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ADS131M06_CHANNEL_COUNT ADC_SENSE_CHANNEL_COUNT

typedef adc_sense_t ads131m06_t;
typedef adc_sense_config_t ads131m06_config_t;

static inline esp_err_t ads131m06_init(ads131m06_t **out_device, const ads131m06_config_t *config)
{
    return adc_sense_init((adc_sense_t **)out_device, (const adc_sense_config_t *)config);
}

static inline void ads131m06_deinit(ads131m06_t *device)
{
    adc_sense_deinit((adc_sense_t *)device);
}

static inline esp_err_t ads131m06_get_avg_volts(ads131m06_t *device, float out_volts[ADS131M06_CHANNEL_COUNT])
{
    return adc_sense_get_avg_volts((adc_sense_t *)device, out_volts);
}

static inline esp_err_t ads131m06_get_slow_avg_volts(ads131m06_t *device, float out_volts[ADS131M06_CHANNEL_COUNT])
{
    return adc_sense_get_slow_avg_volts((adc_sense_t *)device, out_volts);
}

static inline esp_err_t ads131m06_get_sample_count(ads131m06_t *device, uint32_t *out_sample_count)
{
    return adc_sense_get_sample_count((adc_sense_t *)device, out_sample_count);
}

static inline esp_err_t ads131m06_get_last_status(ads131m06_t *device, uint16_t *out_status)
{
    return adc_sense_get_last_status((adc_sense_t *)device, out_status);
}

static inline esp_err_t ads131m06_register_data_ready_task(ads131m06_t *device, TaskHandle_t task)
{
    return adc_sense_register_data_ready_task((adc_sense_t *)device, task);
}

#ifdef __cplusplus
}
#endif