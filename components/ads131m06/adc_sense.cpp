#include "adc_sense.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define ADC_SENSE_TASK_STACK_SIZE 2048
#define ADC_SENSE_TASK_PRIORITY 10  /* Higher than control task (8) to prevent preemption; no yielding needed */
#define ADC_SENSE_TASK_YIELD_INTERVAL 64U

#define ADS131M06_WORD_BYTES_24 3U
#define ADS131M06_WORD_BYTES_32 4U
#define ADS131M06_MAX_WORD_BYTES ADS131M06_WORD_BYTES_32
#define ADS131M06_FRAME_WORD_COUNT 8U
#define ADS131M06_DOUBLE_FRAME_WORD_COUNT (ADS131M06_FRAME_WORD_COUNT * 2U)
#define ADS131M06_MAX_FRAME_BYTES (ADS131M06_MAX_WORD_BYTES * ADS131M06_DOUBLE_FRAME_WORD_COUNT)
#define ADS131M06_STATUS_WORD_INDEX 0U
#define ADS131M06_FIRST_CHANNEL_WORD_INDEX 1U
#define ADS131M06_CRC_WORD_INDEX 7U
#define ADS131M06_CHANNEL_FULL_SCALE_CODE 8388608.0f
#define ADS131M06_DEFAULT_CHANNEL_ENABLE_MASK 0x3FU

#define ADS131M06_REG_ID 0x00U
#define ADS131M06_REG_STATUS 0x01U
#define ADS131M06_REG_MODE 0x02U
#define ADS131M06_REG_CLOCK 0x03U
#define ADS131M06_REG_GAIN1 0x04U
#define ADS131M06_REG_GAIN2 0x05U
#define ADS131M06_REG_CH0_CFG 0x09U
#define ADS131M06_REG_CH1_CFG 0x0EU
#define ADS131M06_REG_CH2_CFG 0x13U
#define ADS131M06_REG_CH3_CFG 0x18U
#define ADS131M06_REG_CH4_CFG 0x1DU
#define ADS131M06_REG_CH5_CFG 0x22U

#define ADS131M06_CMD_NULL 0x0000U
#define ADS131M06_CMD_RESET 0x0011U
#define ADS131M06_CMD_UNLOCK 0x0655U
#define ADS131M06_RESET_ACK 0xFF26U

#define ADS131M06_ID_CHANCNT_MASK 0x0F00U
#define ADS131M06_ID_CHANCNT_SHIFT 8U
#define ADS131M06_ID_CHANCNT_EXPECTED 6U

#define ADS131M06_STATUS_LOCK_MASK (1U << 15)
#define ADS131M06_STATUS_F_RESYNC_MASK (1U << 14)
#define ADS131M06_STATUS_REG_MAP_MASK (1U << 13)
#define ADS131M06_STATUS_CRC_ERR_MASK (1U << 12)
#define ADS131M06_STATUS_RESET_MASK (1U << 10)
#define ADS131M06_STATUS_WLENGTH_MASK (3U << 8)
#define ADS131M06_STATUS_DRDY_MASK 0x003FU

#define ADS131M06_MODE_RESET_MASK (1U << 10)
#define ADS131M06_MODE_WLENGTH_24 (1U << 8)
#define ADS131M06_MODE_TIMEOUT_MASK (1U << 4)
#define ADS131M06_MODE_DRDY_FMT_MASK (1U << 0)

#define ADS131M06_STATUS_WLENGTH_24 ADS131M06_MODE_WLENGTH_24
#define ADS131M06_STATUS_WLENGTH_32_ZERO_PAD (2U << 8)
#define ADS131M06_STATUS_WLENGTH_32_SIGN_EXT (3U << 8)

#define ADS131M06_CLOCK_CH5_EN_MASK (1U << 13)
#define ADS131M06_CLOCK_CH4_EN_MASK (1U << 12)
#define ADS131M06_CLOCK_CH3_EN_MASK (1U << 11)
#define ADS131M06_CLOCK_CH2_EN_MASK (1U << 10)
#define ADS131M06_CLOCK_CH1_EN_MASK (1U << 9)
#define ADS131M06_CLOCK_CH0_EN_MASK (1U << 8)
#define ADS131M06_CLOCK_EXTREF_EN_MASK (1U << 6)
#define ADS131M06_CLOCK_PWR_HR_MASK (2U << 0)

#define ADS131M06_SPI_READY_TIMEOUT_MS 100U
#define ADS131M06_POWERUP_SETTLE_MS 2U
#define ADS131M06_RESET_SETTLE_MS 2U
#define ADS131M06_CS_DELAY_CYCLES 2U
#define ADS131M06_SAMPLING_YIELD_INTERVAL 64U

static const char *TAG = "adc_sense";

typedef struct {
    float channel[ADC_SENSE_CHANNEL_COUNT];
} adc_sense_sample_t;

struct adc_sense_t {
    adc_sense_config_t _config;
    spi_device_handle_t _spi;

    SemaphoreHandle_t _lock;
    TaskHandle_t _task;
    TaskHandle_t _data_ready_task;

    adc_sense_sample_t *_history;
    adc_sense_sample_t *_slow_history;
    float _sum[ADC_SENSE_CHANNEL_COUNT];
    float _avg[ADC_SENSE_CHANNEL_COUNT];
    float _slow_sum[ADC_SENSE_CHANNEL_COUNT];
    float _slow_avg[ADC_SENSE_CHANNEL_COUNT];

    uint16_t _history_len;
    uint16_t _history_index;
    uint16_t _history_fill;
    uint16_t _slow_history_len;
    uint16_t _slow_history_index;
    uint16_t _slow_history_fill;

    uint32_t _sample_count;
    uint32_t _drdy_count;  /* Count of DRDY ISR fires for rate measurement */
    uint16_t _last_status;
    uint8_t _channel_enable_mask;

    bool _running;
    bool _spi_bus_owned;
    bool _isr_handler_installed;
    bool _startup_sync_pending;
    uint8_t _word_bytes;
};

static esp_err_t adc_sense_send_simple_command(adc_sense_t *sense, const char *label, uint16_t command,
                                               uint16_t *out_response_word);
static esp_err_t adc_sense_probe_device(adc_sense_t *sense, uint16_t *out_id, uint16_t *out_status, uint16_t *out_mode);
static esp_err_t adc_sense_transfer_frame(adc_sense_t *sense, const char *label,
                                          const uint32_t tx_words[ADS131M06_FRAME_WORD_COUNT],
                                          uint32_t rx_words[ADS131M06_FRAME_WORD_COUNT]);
static esp_err_t adc_sense_transfer_command_frames(adc_sense_t *sense, const char *label,
                                                   const uint32_t tx_frame0[ADS131M06_FRAME_WORD_COUNT],
                                                   uint32_t rx_words[ADS131M06_DOUBLE_FRAME_WORD_COUNT]);
static esp_err_t adc_sense_transfer_words(adc_sense_t *sense, const char *label,
                                          const uint32_t tx_words[ADS131M06_FRAME_WORD_COUNT],
                                          uint32_t rx_words[ADS131M06_FRAME_WORD_COUNT],
                                          size_t word_count);
static esp_err_t adc_sense_read_status_from_null(adc_sense_t *sense, const char *label, uint16_t *out_status);

static int32_t adc_sense_sign_extend_24(uint32_t value)
{
    if ((value & 0x800000U) != 0U) {
        value |= 0xFF000000U;
    }
    return (int32_t)value;
}

static uint32_t adc_sense_pack_16bit_word(uint16_t value, uint8_t word_bytes)
{
    if (word_bytes == ADS131M06_WORD_BYTES_24) {
        return ((uint32_t)value) << 8;
    }

    return ((uint32_t)value) << 16;
}

static uint16_t adc_sense_unpack_16bit_word(uint32_t value, uint8_t word_bytes)
{
    if (word_bytes == ADS131M06_WORD_BYTES_24) {
        return (uint16_t)((value >> 8) & 0xFFFFU);
    }

    return (uint16_t)((value >> 16) & 0xFFFFU);
}

static void adc_sense_store_word(uint8_t *dst, uint32_t word, uint8_t word_bytes)
{
    if (word_bytes == ADS131M06_WORD_BYTES_24) {
        dst[0] = (uint8_t)((word >> 16) & 0xFFU);
        dst[1] = (uint8_t)((word >> 8) & 0xFFU);
        dst[2] = (uint8_t)(word & 0xFFU);
        return;
    }

    dst[0] = (uint8_t)((word >> 24) & 0xFFU);
    dst[1] = (uint8_t)((word >> 16) & 0xFFU);
    dst[2] = (uint8_t)((word >> 8) & 0xFFU);
    dst[3] = (uint8_t)(word & 0xFFU);
}

static uint32_t adc_sense_load_word(const uint8_t *src, uint8_t word_bytes)
{
    if (word_bytes == ADS131M06_WORD_BYTES_24) {
        return ((uint32_t)src[0] << 16) | ((uint32_t)src[1] << 8) | (uint32_t)src[2];
    }

    return ((uint32_t)src[0] << 24) | ((uint32_t)src[1] << 16) | ((uint32_t)src[2] << 8) | (uint32_t)src[3];
}

static void adc_sense_log_frame_words(const char *label, const uint32_t words[ADS131M06_FRAME_WORD_COUNT])
{
    if (label == NULL || words == NULL) {
        return;
    }

    char buffer[192];
    int offset = snprintf(buffer, sizeof(buffer),
                          "%s w0=%08lx w1=%08lx w2=%08lx w3=%08lx w4=%08lx w5=%08lx w6=%08lx w7=%08lx",
                          label,
                          (unsigned long)words[0],
                          (unsigned long)words[1],
                          (unsigned long)words[2],
                          (unsigned long)words[3],
                          (unsigned long)words[4],
                          (unsigned long)words[5],
                          (unsigned long)words[6],
                          (unsigned long)words[7]);
    if (offset < 0) {
        return;
    }

    ESP_LOGI(TAG, "%s", buffer);
}

static void adc_sense_log_transfer(const adc_sense_t *sense, const char *label,
                                   const uint32_t tx_words[ADS131M06_FRAME_WORD_COUNT],
                                   const uint32_t rx_words[ADS131M06_FRAME_WORD_COUNT])
{
    if (sense == NULL || label == NULL || tx_words == NULL || rx_words == NULL) {
        return;
    }

    char tx_label[48];
    char rx_label[48];

    (void)snprintf(tx_label, sizeof(tx_label), "%s TX", label);
    (void)snprintf(rx_label, sizeof(rx_label), "%s RX", label);

    adc_sense_log_frame_words(tx_label, tx_words);
    adc_sense_log_frame_words(rx_label, rx_words);

    ESP_LOGI(TAG, "%s decoded: rx0_16=0x%04x status16=0x%04x drdy_gpio=%d",
             label,
             adc_sense_unpack_16bit_word(rx_words[0], sense->_word_bytes),
             adc_sense_unpack_16bit_word(rx_words[ADS131M06_STATUS_WORD_INDEX], sense->_word_bytes),
             gpio_get_level(sense->_config.drdy_gpio));
}

static bool adc_sense_status_wlength_is_supported(uint16_t status_word)
{
    const uint16_t wlength = status_word & ADS131M06_STATUS_WLENGTH_MASK;
    return (wlength == ADS131M06_STATUS_WLENGTH_24) ||
           (wlength == ADS131M06_STATUS_WLENGTH_32_ZERO_PAD) ||
           (wlength == ADS131M06_STATUS_WLENGTH_32_SIGN_EXT);
}

static uint8_t adc_sense_word_bytes_from_status(uint16_t status_word)
{
    const uint16_t wlength = status_word & ADS131M06_STATUS_WLENGTH_MASK;

    if (wlength == ADS131M06_STATUS_WLENGTH_24) {
        return ADS131M06_WORD_BYTES_24;
    }
    if (wlength == ADS131M06_STATUS_WLENGTH_32_ZERO_PAD ||
        wlength == ADS131M06_STATUS_WLENGTH_32_SIGN_EXT) {
        return ADS131M06_WORD_BYTES_32;
    }

    return 0U;
}

static int adc_sense_score_detect_frame(const adc_sense_t *sense,
                                        const uint32_t rx_words[ADS131M06_FRAME_WORD_COUNT],
                                        uint16_t *out_status_word)
{
    if (sense == NULL || rx_words == NULL) {
        return -100;
    }

    const uint16_t status_word = adc_sense_unpack_16bit_word(rx_words[0], sense->_word_bytes);
    if (out_status_word != NULL) {
        *out_status_word = status_word;
    }
    if (!adc_sense_status_wlength_is_supported(status_word)) {
        return -100;
    }

    const uint8_t reported_word_bytes = adc_sense_word_bytes_from_status(status_word);

    size_t nonzero_word_count = 0U;
    size_t repeated_status_count = 0U;

    for (size_t i = 0; i < ADS131M06_FRAME_WORD_COUNT; ++i) {
        if (rx_words[i] != 0U) {
            nonzero_word_count++;
            if (i != ADS131M06_STATUS_WORD_INDEX &&
                adc_sense_unpack_16bit_word(rx_words[i], sense->_word_bytes) == status_word) {
                repeated_status_count++;
            }
        }
    }

    int score = 8;
    if (rx_words[ADS131M06_STATUS_WORD_INDEX] != 0U) {
        score += 4;
    }
    if (rx_words[ADS131M06_CRC_WORD_INDEX] != 0U) {
        score += 4;
    } else {
        score -= 2;
    }

    if (nonzero_word_count == 2U) {
        score += 4;
    } else if (nonzero_word_count == 1U) {
        score += 1;
    } else if (nonzero_word_count > 2U) {
        score -= (int)((nonzero_word_count - 2U) * 3U);
    }

    score -= (int)(repeated_status_count * 4U);

    if (reported_word_bytes != 0U) {
        if (reported_word_bytes == sense->_word_bytes) {
            score += 6;
        } else {
            score -= 6;
        }
    }

    return score;
}

static esp_err_t adc_sense_detect_interface_word_bytes(adc_sense_t *sense)
{
    if (sense == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t candidates[] = {ADS131M06_WORD_BYTES_24, ADS131M06_WORD_BYTES_32};
    int best_score = -100;
    uint8_t best_word_bytes = 0U;
    uint16_t best_status_word = 0U;

    for (size_t i = 0; i < (sizeof(candidates) / sizeof(candidates[0])); ++i) {
        sense->_word_bytes = candidates[i];

        uint32_t tx[ADS131M06_FRAME_WORD_COUNT] = {0};
        uint32_t rx[ADS131M06_FRAME_WORD_COUNT] = {0};
        if (adc_sense_transfer_frame(sense, "detect-null0", tx, rx) != ESP_OK) {
            continue;
        }

        memset(rx, 0, sizeof(rx));
        if (adc_sense_transfer_frame(sense, "detect-null1", tx, rx) != ESP_OK) {
            continue;
        }

        uint16_t status_word = 0U;
        const int score = adc_sense_score_detect_frame(sense, rx, &status_word);
        if (score <= -100) {
            continue;
        }

        uint16_t id = 0U;
        if (adc_sense_probe_device(sense, &id, NULL, NULL) == ESP_OK) {
            ESP_LOGI(TAG,
                     "Detected ADS131M06 interface at %u-bit words (STATUS=0x%04x, ID=0x%04x)",
                     (unsigned int)(sense->_word_bytes * 8U),
                     (unsigned int)status_word,
                     (unsigned int)id);
            return ESP_OK;
        }

        ESP_LOGW(TAG,
             "Inferred ADS131M06 %u-bit framing candidate with score=%d STATUS=0x%04x (reported_word_bits=%u) despite failed RREG probe",
                 (unsigned int)(sense->_word_bytes * 8U),
                 score,
             (unsigned int)status_word,
             (unsigned int)(adc_sense_word_bytes_from_status(status_word) * 8U));

        if (score > best_score) {
            best_score = score;
            best_word_bytes = sense->_word_bytes;
            best_status_word = status_word;
        }
    }

    if (best_word_bytes != 0U) {
        sense->_word_bytes = best_word_bytes;
        ESP_LOGW(TAG,
                 "Using inferred ADS131M06 interface at %u-bit words (STATUS=0x%04x, score=%d) because RREG probe did not succeed during detection",
                 (unsigned int)(sense->_word_bytes * 8U),
                 (unsigned int)best_status_word,
                 best_score);
        return ESP_OK;
    }

    return ESP_FAIL;
}

static uint16_t adc_sense_build_rreg_cmd(uint8_t reg_addr, uint8_t count_minus_one)
{
    return (uint16_t)(0xA000U | (((uint16_t)reg_addr & 0x3FU) << 7) | ((uint16_t)count_minus_one & 0x7FU));
}

static uint16_t adc_sense_build_wreg_cmd(uint8_t reg_addr, uint8_t count_minus_one)
{
    return (uint16_t)(0x6000U | (((uint16_t)reg_addr & 0x3FU) << 7) | ((uint16_t)count_minus_one & 0x7FU));
}

static uint16_t adc_sense_build_wreg_ack(uint8_t reg_addr, uint8_t count_minus_one)
{
    return (uint16_t)(0x4000U | (((uint16_t)reg_addr & 0x3FU) << 7) | ((uint16_t)count_minus_one & 0x7FU));
}

static bool adc_sense_id_is_plausible(uint16_t id_word)
{
    const uint16_t channel_count = (uint16_t)((id_word & ADS131M06_ID_CHANCNT_MASK) >> ADS131M06_ID_CHANCNT_SHIFT);
    return channel_count == ADS131M06_ID_CHANCNT_EXPECTED;
}

static esp_err_t adc_sense_gain_to_code(float gain, uint16_t *out_code)
{
    static const float valid_gains[] = {1.0f, 2.0f, 4.0f, 8.0f, 16.0f, 32.0f, 64.0f, 128.0f};

    if (out_code == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    for (uint16_t i = 0; i < (uint16_t)(sizeof(valid_gains) / sizeof(valid_gains[0])); ++i) {
        if (fabsf(gain - valid_gains[i]) <= (valid_gains[i] * 0.001f)) {
            *out_code = i;
            return ESP_OK;
        }
    }

    return ESP_ERR_INVALID_ARG;
}

static uint16_t adc_sense_build_clock_reg(const adc_sense_t *sense)
{
    uint16_t osr = (sense->_config.osr_mask != 0U) ? sense->_config.osr_mask : ADS131M06_CLOCK_OSR_1024_MASK;
    uint16_t reg = osr | ADS131M06_CLOCK_PWR_HR_MASK;

    if ((sense->_channel_enable_mask & (1U << 0)) != 0U) {
        reg |= ADS131M06_CLOCK_CH0_EN_MASK;
    }
    if ((sense->_channel_enable_mask & (1U << 1)) != 0U) {
        reg |= ADS131M06_CLOCK_CH1_EN_MASK;
    }
    if ((sense->_channel_enable_mask & (1U << 2)) != 0U) {
        reg |= ADS131M06_CLOCK_CH2_EN_MASK;
    }
    if ((sense->_channel_enable_mask & (1U << 3)) != 0U) {
        reg |= ADS131M06_CLOCK_CH3_EN_MASK;
    }
    if ((sense->_channel_enable_mask & (1U << 4)) != 0U) {
        reg |= ADS131M06_CLOCK_CH4_EN_MASK;
    }
    if ((sense->_channel_enable_mask & (1U << 5)) != 0U) {
        reg |= ADS131M06_CLOCK_CH5_EN_MASK;
    }

    if (sense->_config.external_reference_enable) {
        reg |= ADS131M06_CLOCK_EXTREF_EN_MASK;
    }

    return reg;
}

static uint16_t adc_sense_build_mode_reg(const adc_sense_t *sense)
{
    uint16_t reg = ADS131M06_MODE_WLENGTH_24 | ADS131M06_MODE_TIMEOUT_MASK;

    if (sense->_config.drdy_pulse_mode) {
        reg |= ADS131M06_MODE_DRDY_FMT_MASK;
    }

    return reg;
}

static uint16_t adc_sense_build_gain1_reg(uint16_t gain_code)
{
    return (uint16_t)((gain_code << 12) | (gain_code << 8) | (gain_code << 4) | gain_code);
}

static uint16_t adc_sense_build_gain2_reg(uint16_t gain_code)
{
    return (uint16_t)((gain_code << 4) | gain_code);
}

static float adc_sense_code_to_volts(const adc_sense_t *sense, int32_t code)
{
    const float full_scale = sense->_config.vref_volts / sense->_config.gain;
    return ((float)code / ADS131M06_CHANNEL_FULL_SCALE_CODE) * full_scale;
}

static esp_err_t adc_sense_wait_for_drdy_level(gpio_num_t pin, int level, uint32_t timeout_ms)
{
    const int64_t deadline_us = esp_timer_get_time() + ((int64_t)timeout_ms * 1000LL);

    while (esp_timer_get_time() < deadline_us) {
        if (gpio_get_level(pin) == level) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return ESP_ERR_TIMEOUT;
}

static void adc_sense_powerup_settle(void)
{
    vTaskDelay(pdMS_TO_TICKS(ADS131M06_POWERUP_SETTLE_MS));
}

static esp_err_t adc_sense_transfer_words(adc_sense_t *sense, const char *label,
                                          const uint32_t tx_words[ADS131M06_FRAME_WORD_COUNT],
                                          uint32_t rx_words[ADS131M06_FRAME_WORD_COUNT],
                                          size_t word_count)
{
    if (sense == NULL || tx_words == NULL || rx_words == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (word_count == 0U || word_count > ADS131M06_FRAME_WORD_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    const size_t frame_bytes = (size_t)sense->_word_bytes * word_count;
    if ((sense->_word_bytes != ADS131M06_WORD_BYTES_24 && sense->_word_bytes != ADS131M06_WORD_BYTES_32) ||
        frame_bytes > ADS131M06_MAX_FRAME_BYTES) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t tx[ADS131M06_MAX_FRAME_BYTES] = {0};
    uint8_t rx[ADS131M06_MAX_FRAME_BYTES] = {0};

    for (size_t i = 0; i < word_count; ++i) {
        adc_sense_store_word(&tx[i * sense->_word_bytes], tx_words[i], sense->_word_bytes);
    }

    spi_transaction_t trans = {
        .length = (uint32_t)frame_bytes * 8U,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t err = spi_device_polling_transmit(sense->_spi, &trans);
    if (err != ESP_OK) {
        if (label != NULL) {
            ESP_LOGE(TAG, "%s SPI transfer failed: 0x%x", label, (unsigned int)err);
            adc_sense_log_frame_words("failed TX", tx_words);
        }
        return err;
    }

    memset(rx_words, 0, sizeof(uint32_t) * ADS131M06_FRAME_WORD_COUNT);
    for (size_t i = 0; i < word_count; ++i) {
        rx_words[i] = adc_sense_load_word(&rx[i * sense->_word_bytes], sense->_word_bytes);
    }

    if (label != NULL) {
        adc_sense_log_transfer(sense, label, tx_words, rx_words);
    }

    return ESP_OK;
}

static esp_err_t adc_sense_transfer_frame(adc_sense_t *sense, const char *label,
                                          const uint32_t tx_words[ADS131M06_FRAME_WORD_COUNT],
                                          uint32_t rx_words[ADS131M06_FRAME_WORD_COUNT])
{
    return adc_sense_transfer_words(sense, label, tx_words, rx_words, ADS131M06_FRAME_WORD_COUNT);
}

static esp_err_t adc_sense_transfer_command_frames(adc_sense_t *sense, const char *label,
                                                   const uint32_t tx_frame0[ADS131M06_FRAME_WORD_COUNT],
                                                   uint32_t rx_words[ADS131M06_DOUBLE_FRAME_WORD_COUNT])
{
    if (sense == NULL || tx_frame0 == NULL || rx_words == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const size_t frame_bytes = (size_t)sense->_word_bytes * ADS131M06_DOUBLE_FRAME_WORD_COUNT;
    if ((sense->_word_bytes != ADS131M06_WORD_BYTES_24 && sense->_word_bytes != ADS131M06_WORD_BYTES_32) ||
        frame_bytes > ADS131M06_MAX_FRAME_BYTES) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t tx_words[ADS131M06_DOUBLE_FRAME_WORD_COUNT] = {0};
    uint8_t tx[ADS131M06_MAX_FRAME_BYTES] = {0};
    uint8_t rx[ADS131M06_MAX_FRAME_BYTES] = {0};

    memcpy(tx_words, tx_frame0, sizeof(uint32_t) * ADS131M06_FRAME_WORD_COUNT);
    for (size_t i = 0; i < ADS131M06_DOUBLE_FRAME_WORD_COUNT; ++i) {
        adc_sense_store_word(&tx[i * sense->_word_bytes], tx_words[i], sense->_word_bytes);
    }

    spi_transaction_t trans = {
        .length = (uint32_t)frame_bytes * 8U,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t err = spi_device_polling_transmit(sense->_spi, &trans);
    if (err != ESP_OK) {
        if (label != NULL) {
            ESP_LOGE(TAG, "%s SPI command transfer failed: 0x%x", label, (unsigned int)err);
            adc_sense_log_frame_words("failed TX frame0", tx_frame0);
        }
        return err;
    }

    for (size_t i = 0; i < ADS131M06_DOUBLE_FRAME_WORD_COUNT; ++i) {
        rx_words[i] = adc_sense_load_word(&rx[i * sense->_word_bytes], sense->_word_bytes);
    }

    if (label != NULL) {
        char tx0_label[48];
        char rx0_label[48];
        char tx1_label[48];
        char rx1_label[48];

        (void)snprintf(tx0_label, sizeof(tx0_label), "%s TX0", label);
        (void)snprintf(rx0_label, sizeof(rx0_label), "%s RX0", label);
        (void)snprintf(tx1_label, sizeof(tx1_label), "%s TX1", label);
        (void)snprintf(rx1_label, sizeof(rx1_label), "%s RX1", label);

        adc_sense_log_frame_words(tx0_label, tx_words);
        adc_sense_log_frame_words(rx0_label, rx_words);
        adc_sense_log_frame_words(tx1_label, &tx_words[ADS131M06_FRAME_WORD_COUNT]);
        adc_sense_log_frame_words(rx1_label, &rx_words[ADS131M06_FRAME_WORD_COUNT]);

        ESP_LOGI(TAG,
                 "%s decoded: frame0_rx0=0x%04x frame1_rx0=0x%04x drdy_gpio=%d",
                 label,
                 adc_sense_unpack_16bit_word(rx_words[0], sense->_word_bytes),
                 adc_sense_unpack_16bit_word(rx_words[ADS131M06_FRAME_WORD_COUNT], sense->_word_bytes),
                 gpio_get_level(sense->_config.drdy_gpio));
    }

    return ESP_OK;
}

static esp_err_t adc_sense_send_simple_command(adc_sense_t *sense, const char *label, uint16_t command,
                                               uint16_t *out_response_word)
{
    uint32_t tx[ADS131M06_FRAME_WORD_COUNT] = {0};
    uint32_t rx[ADS131M06_FRAME_WORD_COUNT] = {0};

    tx[0] = adc_sense_pack_16bit_word(command, sense->_word_bytes);

    esp_err_t err = adc_sense_transfer_frame(sense, label, tx, rx);
    if (err != ESP_OK) {
        return err;
    }

    if (out_response_word != NULL) {
        *out_response_word = adc_sense_unpack_16bit_word(rx[0], sense->_word_bytes);
    }

    return ESP_OK;
}

static esp_err_t adc_sense_read_status_from_null(adc_sense_t *sense, const char *label, uint16_t *out_status)
{
    if (sense == NULL || out_status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t tx[ADS131M06_FRAME_WORD_COUNT] = {0};
    uint32_t rx[ADS131M06_FRAME_WORD_COUNT] = {0};

    esp_err_t err = adc_sense_transfer_frame(sense, label, tx, rx);
    if (err != ESP_OK) {
        return err;
    }

    *out_status = adc_sense_unpack_16bit_word(rx[ADS131M06_STATUS_WORD_INDEX], sense->_word_bytes);
    return ESP_OK;
}

static esp_err_t adc_sense_read_register(adc_sense_t *sense, uint8_t reg_addr, uint16_t *out_value)
{
    if (sense == NULL || out_value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t tx[ADS131M06_FRAME_WORD_COUNT] = {0};
    uint32_t rx[ADS131M06_DOUBLE_FRAME_WORD_COUNT] = {0};

    tx[0] = adc_sense_pack_16bit_word(adc_sense_build_rreg_cmd(reg_addr, 0U), sense->_word_bytes);

    esp_err_t err = adc_sense_transfer_command_frames(sense, "rreg", tx, rx);
    if (err != ESP_OK) {
        return err;
    }

    *out_value = adc_sense_unpack_16bit_word(rx[ADS131M06_FRAME_WORD_COUNT], sense->_word_bytes);
    return ESP_OK;
}

static esp_err_t adc_sense_write_register(adc_sense_t *sense, uint8_t reg_addr, uint16_t value)
{
    if (sense == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t tx[ADS131M06_FRAME_WORD_COUNT] = {0};
    uint32_t rx[ADS131M06_DOUBLE_FRAME_WORD_COUNT] = {0};

    tx[0] = adc_sense_pack_16bit_word(adc_sense_build_wreg_cmd(reg_addr, 0U), sense->_word_bytes);
    tx[1] = adc_sense_pack_16bit_word(value, sense->_word_bytes);

    esp_err_t err = adc_sense_transfer_command_frames(sense, "wreg", tx, rx);
    if (err != ESP_OK) {
        return err;
    }

    const uint16_t ack = adc_sense_unpack_16bit_word(rx[ADS131M06_FRAME_WORD_COUNT], sense->_word_bytes);
    const uint16_t expected_ack = adc_sense_build_wreg_ack(reg_addr, 0U);
    if (ack != expected_ack) {
        ESP_LOGE(TAG, "WREG ack mismatch for reg 0x%02x: got 0x%04x expected 0x%04x",
                 reg_addr, ack, expected_ack);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t adc_sense_probe_device(adc_sense_t *sense, uint16_t *out_id, uint16_t *out_status, uint16_t *out_mode)
{
    if (sense == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t id = 0U;
    esp_err_t err = adc_sense_read_register(sense, ADS131M06_REG_ID, &id);
    if (err != ESP_OK) {
        return err;
    }
    if (!adc_sense_id_is_plausible(id)) {
        return ESP_FAIL;
    }

    if (out_id != NULL) {
        *out_id = id;
    }

    if (out_status != NULL) {
        err = adc_sense_read_register(sense, ADS131M06_REG_STATUS, out_status);
        if (err != ESP_OK) {
            return err;
        }
    }

    if (out_mode != NULL) {
        err = adc_sense_read_register(sense, ADS131M06_REG_MODE, out_mode);
        if (err != ESP_OK) {
            return err;
        }
    }

    return ESP_OK;
}

static esp_err_t adc_sense_unlock_interface_if_needed(adc_sense_t *sense)
{
    if (sense == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t status = 0U;
    esp_err_t err = adc_sense_read_status_from_null(sense, "status-null", &status);
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "Attempting ADS131M06 UNLOCK sequence (status=0x%04x)", status);

    uint32_t tx[ADS131M06_FRAME_WORD_COUNT] = {0};
    uint32_t rx[ADS131M06_DOUBLE_FRAME_WORD_COUNT] = {0};

    tx[0] = adc_sense_pack_16bit_word(ADS131M06_CMD_UNLOCK, sense->_word_bytes);

    err = adc_sense_transfer_command_frames(sense, "unlock", tx, rx);
    if (err != ESP_OK) {
        return err;
    }

    const uint16_t response = adc_sense_unpack_16bit_word(rx[ADS131M06_FRAME_WORD_COUNT], sense->_word_bytes);

    if (response != ADS131M06_CMD_UNLOCK) {
        ESP_LOGW(TAG, "UNLOCK ack mismatch: got 0x%04x expected 0x%04x", response, ADS131M06_CMD_UNLOCK);
    }

    err = adc_sense_read_status_from_null(sense, "status-post-unlock", &status);
    if (err != ESP_OK) {
        return err;
    }

    if ((status & ADS131M06_STATUS_LOCK_MASK) != 0U) {
        ESP_LOGW(TAG, "ADS131M06 still appears locked after UNLOCK attempt (status=0x%04x)", status);
    }

    return ESP_OK;
}

static esp_err_t adc_sense_reset_device(adc_sense_t *sense)
{
    if (sense == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    adc_sense_powerup_settle();

    esp_err_t err = adc_sense_wait_for_drdy_level(sense->_config.drdy_gpio, 1, ADS131M06_SPI_READY_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DRDY did not idle high before initialization; attempting SPI probe anyway (gpio=%d)",
                 gpio_get_level(sense->_config.drdy_gpio));
    }

    err = adc_sense_detect_interface_word_bytes(sense);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to detect ADS131M06 interface word length");
        return err;
    }

    uint16_t id_before_reset = 0U;
    ESP_LOGI(TAG, "Probing ADC before RESET (word_bits=%u drdy_gpio=%d)",
             (unsigned int)(sense->_word_bytes * 8U),
             gpio_get_level(sense->_config.drdy_gpio));
    const bool probe_before_reset_ok = (adc_sense_probe_device(sense, &id_before_reset, NULL, NULL) == ESP_OK);

    for (size_t attempt = 0; attempt < 3U; ++attempt) {
        uint32_t tx[ADS131M06_FRAME_WORD_COUNT] = {0};
        uint32_t rx[ADS131M06_DOUBLE_FRAME_WORD_COUNT] = {0};

        tx[0] = adc_sense_pack_16bit_word(ADS131M06_CMD_RESET, sense->_word_bytes);

        err = adc_sense_transfer_command_frames(sense, "reset", tx, rx);
        if (err != ESP_OK) {
            return err;
        }

        vTaskDelay(pdMS_TO_TICKS(ADS131M06_RESET_SETTLE_MS));

        const uint16_t ack1 = adc_sense_unpack_16bit_word(rx[ADS131M06_FRAME_WORD_COUNT], sense->_word_bytes);

        if (ack1 == ADS131M06_RESET_ACK) {
            ESP_LOGI(TAG, "RESET acknowledged on attempt %u after first NULL frame",
                     (unsigned int)(attempt + 1U));
            return ESP_OK;
        }

        uint16_t ack2 = 0U;
        err = adc_sense_read_status_from_null(sense, "reset-null2", &ack2);
        if (err != ESP_OK) {
            return err;
        }

        if (ack2 == ADS131M06_RESET_ACK) {
            ESP_LOGI(TAG, "RESET acknowledged on attempt %u after second NULL frame",
                     (unsigned int)(attempt + 1U));
            return ESP_OK;
        }

        ESP_LOGW(TAG, "RESET ack mismatch on attempt %u: first=0x%04x second=0x%04x expected=0x%04x",
                 (unsigned int)(attempt + 1U), ack1, ack2, ADS131M06_RESET_ACK);

        uint16_t post_id = 0U;
        if (adc_sense_probe_device(sense, &post_id, NULL, NULL) == ESP_OK) {
            ESP_LOGW(TAG, "ADC still responds after failed RESET ack (ID=0x%04x); continuing without requiring a confirmed reset",
                     post_id);
            return ESP_OK;
        }

        uint16_t ignored = 0U;
        err = adc_sense_send_simple_command(sense, "retry-null", ADS131M06_CMD_NULL, &ignored);
        if (err != ESP_OK) {
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (probe_before_reset_ok) {
        ESP_LOGW(TAG, "Proceeding with ADC configuration without a confirmed RESET ack because the device responded to RREG (ID=0x%04x)",
                 id_before_reset);
        return ESP_OK;
    }

    ESP_LOGW(TAG,
             "Proceeding with ADC configuration without a confirmed RESET ack because SPI framing is stable but the device never returned RESET_ACK");
    return ESP_OK;
}

static esp_err_t adc_sense_configure_device(adc_sense_t *sense)
{
    if (sense == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t gain_code = 0;
    esp_err_t err = adc_sense_gain_to_code(sense->_config.gain, &gain_code);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Unsupported ADS131M06 PGA gain %.3f", (double)sense->_config.gain);
        return err;
    }

    err = adc_sense_unlock_interface_if_needed(sense);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Unable to unlock ADS131M06 interface before configuration");
        return err;
    }

    err = adc_sense_write_register(sense, ADS131M06_REG_MODE, adc_sense_build_mode_reg(sense));
    if (err != ESP_OK) {
        return err;
    }

    if (sense->_word_bytes != ADS131M06_WORD_BYTES_24) {
        ESP_LOGI(TAG, "Switching host SPI framing from %u-bit words to 24-bit words",
                 (unsigned int)(sense->_word_bytes * 8U));
        sense->_word_bytes = ADS131M06_WORD_BYTES_24;
    }

    err = adc_sense_write_register(sense, ADS131M06_REG_CLOCK, adc_sense_build_clock_reg(sense));
    if (err != ESP_OK) {
        return err;
    }

    /* Verify CLOCK register was written correctly (critical for OSR/sampling rate) */
    uint16_t clock_verify = 0U;
    err = adc_sense_read_register(sense, ADS131M06_REG_CLOCK, &clock_verify);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to verify CLOCK register after write");
        return err;
    }
    uint16_t clock_expected = adc_sense_build_clock_reg(sense);
    ESP_LOGI(TAG, "CLOCK register verification: expected=0x%04x readback=0x%04x (OSR bits [3:2] built=0x%02x)",
             (unsigned int)clock_expected,
             (unsigned int)clock_verify,
             (unsigned int)((clock_expected >> 2) & 0x3U));
    if (clock_verify != clock_expected) {
        ESP_LOGW(TAG, "CLOCK register mismatch after write! This may affect sampling rate.");
    }

    err = adc_sense_write_register(sense, ADS131M06_REG_GAIN1, adc_sense_build_gain1_reg(gain_code));
    if (err != ESP_OK) {
        return err;
    }

    err = adc_sense_write_register(sense, ADS131M06_REG_GAIN2, adc_sense_build_gain2_reg(gain_code));
    if (err != ESP_OK) {
        return err;
    }

    err = adc_sense_write_register(sense, ADS131M06_REG_CH0_CFG, 0x0000U);
    if (err != ESP_OK) {
        return err;
    }
    err = adc_sense_write_register(sense, ADS131M06_REG_CH1_CFG, 0x0000U);
    if (err != ESP_OK) {
        return err;
    }
    err = adc_sense_write_register(sense, ADS131M06_REG_CH2_CFG, 0x0000U);
    if (err != ESP_OK) {
        return err;
    }
    err = adc_sense_write_register(sense, ADS131M06_REG_CH3_CFG, 0x0000U);
    if (err != ESP_OK) {
        return err;
    }
    err = adc_sense_write_register(sense, ADS131M06_REG_CH4_CFG, 0x0000U);
    if (err != ESP_OK) {
        return err;
    }
    err = adc_sense_write_register(sense, ADS131M06_REG_CH5_CFG, 0x0000U);
    if (err != ESP_OK) {
        return err;
    }

    uint16_t id = 0;
    err = adc_sense_read_register(sense, ADS131M06_REG_ID, &id);
    if (err != ESP_OK) {
        return err;
    }

    if (!adc_sense_id_is_plausible(id)) {
        ESP_LOGE(TAG, "Unexpected ADS131M06 ID word 0x%04x", id);
        return ESP_FAIL;
    }

    uint16_t mode = 0;
    err = adc_sense_read_register(sense, ADS131M06_REG_MODE, &mode);
    if (err != ESP_OK) {
        return err;
    }

    if ((mode & ADS131M06_STATUS_WLENGTH_24) != ADS131M06_STATUS_WLENGTH_24) {
        ESP_LOGE(TAG, "ADC not left in 24-bit word mode (MODE=0x%04x)", mode);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t adc_sense_capture_data_frame(adc_sense_t *sense, uint16_t *out_status, adc_sense_sample_t *out_sample)
{
    if (sense == NULL || out_status == NULL || out_sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t tx[ADS131M06_FRAME_WORD_COUNT] = {0};
    uint32_t rx[ADS131M06_FRAME_WORD_COUNT] = {0};

    esp_err_t err = adc_sense_transfer_frame(sense, NULL, tx, rx);
    if (err != ESP_OK) {
        return err;
    }

    const uint16_t status = adc_sense_unpack_16bit_word(rx[ADS131M06_STATUS_WORD_INDEX], sense->_word_bytes);
    *out_status = status;

    for (size_t i = 0; i < ADC_SENSE_CHANNEL_COUNT; ++i) {
        const size_t word_index = ADS131M06_FIRST_CHANNEL_WORD_INDEX + i;
        out_sample->channel[i] = adc_sense_code_to_volts(sense, adc_sense_sign_extend_24(rx[word_index]));
    }

    (void)ADS131M06_CRC_WORD_INDEX;
    return ESP_OK;
}

static void adc_sense_update_average_locked(adc_sense_t *sense, const adc_sense_sample_t *sample, uint16_t status)
{
    adc_sense_sample_t *old_sample = &sense->_history[sense->_history_index];

    for (size_t i = 0; i < ADC_SENSE_CHANNEL_COUNT; ++i) {
        if (sense->_history_fill == sense->_history_len) {
            sense->_sum[i] -= old_sample->channel[i];
        }

        old_sample->channel[i] = sample->channel[i];
        sense->_sum[i] += sample->channel[i];

        const float divisor = (sense->_history_fill == sense->_history_len)
                                  ? (float)sense->_history_len
                                  : (float)(sense->_history_fill + 1U);
        sense->_avg[i] = sense->_sum[i] / divisor;
    }

    sense->_history_index = (uint16_t)((sense->_history_index + 1U) % sense->_history_len);
    if (sense->_history_fill < sense->_history_len) {
        sense->_history_fill++;
    }

    /* Update slow history/averages if configured */
    if (sense->_slow_history_len > 0U && sense->_slow_history != NULL) {
        adc_sense_sample_t *old_slow = &sense->_slow_history[sense->_slow_history_index];
        for (size_t i = 0; i < ADC_SENSE_CHANNEL_COUNT; ++i) {
            if (sense->_slow_history_fill == sense->_slow_history_len) {
                sense->_slow_sum[i] -= old_slow->channel[i];
            }
            old_slow->channel[i] = sample->channel[i];
            sense->_slow_sum[i] += sample->channel[i];

            const float sdiv = (sense->_slow_history_fill == sense->_slow_history_len) ? (float)sense->_slow_history_len
                                                                                       : (float)(sense->_slow_history_fill + 1U);
            sense->_slow_avg[i] = sense->_slow_sum[i] / sdiv;
        }
        sense->_slow_history_index = (uint16_t)((sense->_slow_history_index + 1U) % sense->_slow_history_len);
        if (sense->_slow_history_fill < sense->_slow_history_len) {
            sense->_slow_history_fill++;
        }
    }

    sense->_sample_count++;
    sense->_last_status = status;
}

static void adc_sense_sampling_task(void *arg)
{
    adc_sense_t *sense = (adc_sense_t *)arg;
    uint32_t samples_since_yield = 0U;

    while (sense->_running) {
        /* Wait for ISR notification */
        uint32_t samples_to_process = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (samples_to_process == 0U) {
            continue;
        }

        if (!sense->_running) {
            break;
        }

        if (sense->_startup_sync_pending) {
            sense->_startup_sync_pending = false;

            /* Discard the first two frames after startup/reset so the ADC pipeline settles. */
            adc_sense_sample_t dummy_sample = {0};
            uint16_t dummy_status = 0U;
            (void)adc_sense_capture_data_frame(sense, &dummy_status, &dummy_sample);
            (void)adc_sense_capture_data_frame(sense, &dummy_status, &dummy_sample);

            if (samples_to_process > 0U) {
                samples_to_process--;
            }
        }

        /* Perform SPI read (safe in task context) */
        while (samples_to_process-- > 0U) {
            adc_sense_sample_t sample = {0};
            uint16_t status = 0U;

            if (adc_sense_capture_data_frame(sense, &status, &sample) != ESP_OK) {
                break;
            }

            /* Update averages without lock: ADC task is the exclusive writer.
             * Readers (sense_service) take the lock when reading to ensure consistency.
             */
            adc_sense_update_average_locked(sense, &sample, status);

            samples_since_yield++;
            if (samples_since_yield >= ADC_SENSE_TASK_YIELD_INTERVAL) {
                samples_since_yield = 0U;
                vTaskDelay(1);
            }
        }
    }

    sense->_task = NULL;
    vTaskDelete(NULL);
}

static void IRAM_ATTR adc_sense_drdy_isr(void *arg)
{
    adc_sense_t *sense = (adc_sense_t *)arg;
    sense->_drdy_count++;  /* Increment DRDY counter for rate measurement */
    BaseType_t task_woken = pdFALSE;
    if (sense->_task != NULL) {
        vTaskNotifyGiveFromISR(sense->_task, &task_woken);
    }
    if (task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

uint32_t adc_sense_get_drdy_count(adc_sense_t *sense)
{
    if (sense == NULL) {
        return 0U;
    }
    uint32_t count = sense->_drdy_count;
    sense->_drdy_count = 0U;  /* Reset for next measurement window */
    return count;
}

esp_err_t adc_sense_register_data_ready_task(adc_sense_t *sense, TaskHandle_t task)
{
    if (sense == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(sense->_lock, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    sense->_data_ready_task = task;
    xSemaphoreGive(sense->_lock);
    return ESP_OK;
}

esp_err_t adc_sense_init(adc_sense_t **out_sense, const adc_sense_config_t *config)
{
    if (out_sense == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (config->average_sample_count == 0U || config->gain <= 0.0f || config->vref_volts <= 0.0f ||
        config->spi_mode != 1U) {
        return ESP_ERR_INVALID_ARG;
    }

    adc_sense_t *sense = calloc(1, sizeof(*sense));
    if (sense == NULL) {
        return ESP_ERR_NO_MEM;
    }

    sense->_config = *config;
    sense->_history_len = config->average_sample_count;
    sense->_channel_enable_mask = (config->channel_enable_mask == 0U)
                                      ? ADS131M06_DEFAULT_CHANNEL_ENABLE_MASK
                                      : (uint8_t)(config->channel_enable_mask & ADS131M06_DEFAULT_CHANNEL_ENABLE_MASK);
    sense->_startup_sync_pending = true;
    sense->_word_bytes = ADS131M06_WORD_BYTES_24;

    if (adc_sense_gain_to_code(config->gain, &(uint16_t){0}) != ESP_OK) {
        free(sense);
        return ESP_ERR_INVALID_ARG;
    }

    sense->_history = calloc(sense->_history_len, sizeof(adc_sense_sample_t));
    if (sense->_history == NULL) {
        free(sense);
        return ESP_ERR_NO_MEM;
    }

    /* allocate slow history if requested */
    sense->_slow_history_len = (config->average_slow_sample_count == 0U) ? 0U : config->average_slow_sample_count;
    if (sense->_slow_history_len > 0U) {
        sense->_slow_history = calloc(sense->_slow_history_len, sizeof(adc_sense_sample_t));
        if (sense->_slow_history == NULL) {
            free(sense->_history);
            free(sense);
            return ESP_ERR_NO_MEM;
        }
    } else {
        sense->_slow_history = NULL;
    }

    sense->_lock = xSemaphoreCreateMutex();
    if (sense->_lock == NULL) {
        free(sense->_history);
        free(sense);
        return ESP_ERR_NO_MEM;
    }

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = config->mosi_gpio,
        .miso_io_num = config->miso_gpio,
        .sclk_io_num = config->sclk_gpio,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = ADS131M06_MAX_FRAME_BYTES,
    };

    esp_err_t err = spi_bus_initialize(config->spi_host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err == ESP_OK) {
        sense->_spi_bus_owned = true;
    } else if (err != ESP_ERR_INVALID_STATE) {
        vSemaphoreDelete(sense->_lock);
        free(sense->_history);
        free(sense);
        return err;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = config->spi_clock_hz,
        .mode = config->spi_mode,
        .spics_io_num = config->cs_gpio,
        .queue_size = 1,
        .cs_ena_pretrans = ADS131M06_CS_DELAY_CYCLES,
        .cs_ena_posttrans = ADS131M06_CS_DELAY_CYCLES,
    };

    err = spi_bus_add_device(config->spi_host, &dev_cfg, &sense->_spi);
    if (err != ESP_OK) {
        if (sense->_spi_bus_owned) {
            spi_bus_free(config->spi_host);
        }
        vSemaphoreDelete(sense->_lock);
        free(sense->_history);
        free(sense);
        return err;
    }

    gpio_config_t drdy_cfg = {
        .pin_bit_mask = (1ULL << (uint64_t)config->drdy_gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    err = gpio_config(&drdy_cfg);
    if (err != ESP_OK) {
        spi_bus_remove_device(sense->_spi);
        if (sense->_spi_bus_owned) {
            spi_bus_free(config->spi_host);
        }
        vSemaphoreDelete(sense->_lock);
        free(sense->_history);
        free(sense);
        return err;
    }

    err = adc_sense_reset_device(sense);
    if (err == ESP_OK) {
        err = adc_sense_configure_device(sense);
    }
    if (err != ESP_OK) {
        spi_bus_remove_device(sense->_spi);
        if (sense->_spi_bus_owned) {
            spi_bus_free(config->spi_host);
        }
        vSemaphoreDelete(sense->_lock);
        free(sense->_history);
        free(sense);
        return err;
    }

    err = gpio_install_isr_service(0);
    if (err == ESP_OK) {
        sense->_isr_handler_installed = true;
    } else if (err != ESP_ERR_INVALID_STATE) {
        spi_bus_remove_device(sense->_spi);
        if (sense->_spi_bus_owned) {
            spi_bus_free(config->spi_host);
        }
        vSemaphoreDelete(sense->_lock);
        free(sense->_history);
        free(sense);
        return err;
    }

    sense->_running = true;
    if (xTaskCreatePinnedToCore(adc_sense_sampling_task, "adc_sense_task", ADC_SENSE_TASK_STACK_SIZE, sense,
                                 ADC_SENSE_TASK_PRIORITY, &sense->_task, 1) != pdPASS) {
        sense->_running = false;
        spi_bus_remove_device(sense->_spi);
        if (sense->_spi_bus_owned) {
            spi_bus_free(config->spi_host);
        }
        vSemaphoreDelete(sense->_lock);
        free(sense->_history);
        free(sense);
        return ESP_ERR_NO_MEM;
    }

    err = gpio_isr_handler_add(config->drdy_gpio, adc_sense_drdy_isr, sense);
    if (err != ESP_OK) {
        sense->_running = false;
        if (sense->_task != NULL) {
            xTaskNotifyGive(sense->_task);
        }
        while (sense->_task != NULL) {
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        spi_bus_remove_device(sense->_spi);
        if (sense->_spi_bus_owned) {
            spi_bus_free(config->spi_host);
        }
        vSemaphoreDelete(sense->_lock);
        free(sense->_history);
        free(sense);
        return err;
    }

    *out_sense = sense;
    return ESP_OK;
}

void adc_sense_deinit(adc_sense_t *sense)
{
    if (sense == NULL) {
        return;
    }

    gpio_isr_handler_remove(sense->_config.drdy_gpio);

    sense->_running = false;
    if (sense->_task != NULL) {
        xTaskNotifyGive(sense->_task);
    }
    while (sense->_task != NULL) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (sense->_spi != NULL) {
        spi_bus_remove_device(sense->_spi);
    }

    if (sense->_spi_bus_owned) {
        spi_bus_free(sense->_config.spi_host);
    }

    if (sense->_lock != NULL) {
        vSemaphoreDelete(sense->_lock);
    }
    free(sense->_history);
    if (sense->_slow_history != NULL) {
        free(sense->_slow_history);
    }
    free(sense);
}

esp_err_t adc_sense_get_avg_volts(adc_sense_t *sense, float out_volts[ADC_SENSE_CHANNEL_COUNT])
{
    if (sense == NULL || out_volts == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(sense->_lock, portMAX_DELAY) == pdTRUE) {
        memcpy(out_volts, sense->_avg, sizeof(sense->_avg));
        xSemaphoreGive(sense->_lock);
        return ESP_OK;
    }

    return ESP_FAIL;
}

esp_err_t adc_sense_get_slow_avg_volts(adc_sense_t *sense, float out_volts[ADC_SENSE_CHANNEL_COUNT])
{
    if (sense == NULL || out_volts == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(sense->_lock, portMAX_DELAY) == pdTRUE) {
        if (sense->_slow_history_len > 0U && sense->_slow_history != NULL) {
            memcpy(out_volts, sense->_slow_avg, sizeof(sense->_slow_avg));
        } else {
            /* Fallback to fast average when slow not configured */
            memcpy(out_volts, sense->_avg, sizeof(sense->_avg));
        }
        xSemaphoreGive(sense->_lock);
        return ESP_OK;
    }

    return ESP_FAIL;
}

esp_err_t adc_sense_get_sample_count(adc_sense_t *sense, uint32_t *out_sample_count)
{
    if (sense == NULL || out_sample_count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(sense->_lock, portMAX_DELAY) == pdTRUE) {
        *out_sample_count = sense->_sample_count;
        xSemaphoreGive(sense->_lock);
        return ESP_OK;
    }

    return ESP_FAIL;
}

esp_err_t adc_sense_get_last_status(adc_sense_t *sense, uint16_t *out_status)
{
    if (sense == NULL || out_status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(sense->_lock, portMAX_DELAY) == pdTRUE) {
        *out_status = sense->_last_status;
        xSemaphoreGive(sense->_lock);
        return ESP_OK;
    }

    return ESP_FAIL;
}

esp_err_t adc_sense_read_clock_register(adc_sense_t *sense, uint16_t *out_clock_reg)
{
    if (sense == NULL || out_clock_reg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return adc_sense_read_register(sense, ADS131M06_REG_CLOCK, out_clock_reg);
}
