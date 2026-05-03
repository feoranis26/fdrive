#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "freertos/task.h"

/* ADS131M06 OSR masks for CLOCK register bits OSR[2:0] at [4:2].
 * Datasheet Table 8-17: 000=128, 001=256, 010=512, 011=1024,
 *                       100=2048, 101=4096, 110=8192, 111=16384.
 */
#define ADS131M06_CLOCK_OSR_128_MASK (0U << 2)
#define ADS131M06_CLOCK_OSR_256_MASK (1U << 2)
#define ADS131M06_CLOCK_OSR_512_MASK (2U << 2)
#define ADS131M06_CLOCK_OSR_1024_MASK (3U << 2)
#define ADS131M06_CLOCK_OSR_2048_MASK (4U << 2)
#define ADS131M06_CLOCK_OSR_4096_MASK (5U << 2)
#define ADS131M06_CLOCK_OSR_8192_MASK (6U << 2)
#define ADS131M06_CLOCK_OSR_16384_MASK (7U << 2)

#ifdef __cplusplus
extern "C" {
#endif

#define ADC_SENSE_CHANNEL_COUNT 6

typedef struct adc_sense_t adc_sense_t;

typedef struct {
    spi_host_device_t spi_host;
    gpio_num_t mosi_gpio;
    gpio_num_t miso_gpio;
    gpio_num_t sclk_gpio;
    gpio_num_t cs_gpio;
    gpio_num_t drdy_gpio;

    int spi_clock_hz;
    uint8_t spi_mode;

    uint16_t average_sample_count;
    /* Optional slow average window used for telemetry (0 = disabled). */
    uint16_t average_slow_sample_count;
    float vref_volts;
    float gain;

    bool external_reference_enable;
    bool drdy_pulse_mode;
    uint16_t osr_mask;  /* ADS131M06_CLOCK_OSR_* mask for CLOCK.OSR[2:0] */

    /*
     * Bit i enables channel i.
     * A value of 0 keeps the ADS131M06 default of enabling all six channels.
     */
    uint8_t channel_enable_mask;
} adc_sense_config_t;

esp_err_t adc_sense_init(adc_sense_t **out_sense, const adc_sense_config_t *config);
void adc_sense_deinit(adc_sense_t *sense);

esp_err_t adc_sense_get_avg_volts(adc_sense_t *sense, float out_volts[ADC_SENSE_CHANNEL_COUNT]);
esp_err_t adc_sense_get_slow_avg_volts(adc_sense_t *sense, float out_volts[ADC_SENSE_CHANNEL_COUNT]);
esp_err_t adc_sense_get_sample_count(adc_sense_t *sense, uint32_t *out_sample_count);
esp_err_t adc_sense_get_last_status(adc_sense_t *sense, uint16_t *out_status);

/**
 * Get DRDY interrupt count since last call and reset counter.
 * Useful for measuring the actual ADC output rate.
 */
uint32_t adc_sense_get_drdy_count(adc_sense_t *sense);

/**
 * Read the ADS131M06 CLOCK register to verify OSR and channel config.
 */
esp_err_t adc_sense_read_clock_register(adc_sense_t *sense, uint16_t *out_clock_reg);

esp_err_t adc_sense_register_data_ready_task(adc_sense_t *sense, TaskHandle_t task);

#ifdef __cplusplus
}
#endif
