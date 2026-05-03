#include "h_bridge_driver.h"

#include <math.h>
#include <stdlib.h>
#include <inttypes.h>

#include "esp_check.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "h_bridge";

struct h_bridge_driver_t {
    h_bridge_driver_config_t _config;
    uint32_t _pwm_frequency_hz;
    uint32_t _max_duty;
    portMUX_TYPE _state_lock;
    h_bridge_drive_mode_t _mode;
    float _target_norm;
    bool _running;
};

static float h_bridge_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static void h_bridge_set_gpio_level_if_valid(gpio_num_t gpio, int level)
{
    if (gpio >= 0) {
        gpio_set_level(gpio, level);
    }
}

static esp_err_t h_bridge_setup_output_gpio(gpio_num_t gpio)
{
    if (gpio < 0) {
        return ESP_OK;
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << (uint64_t)gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&io_conf);
}

static esp_err_t h_bridge_update_frequency(h_bridge_driver_t *driver, uint32_t frequency_hz)
{
    if (frequency_hz == 0U) {
        return ESP_OK;
    }

    if (driver->_pwm_frequency_hz == frequency_hz) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(ledc_set_freq(driver->_config.ledc_mode, driver->_config.ledc_timer, frequency_hz), "h_bridge", "set PWM freq failed");
    driver->_pwm_frequency_hz = frequency_hz;
    return ESP_OK;
}

static esp_err_t h_bridge_set_pwm(ledc_mode_t mode, ledc_channel_t channel, uint32_t duty)
{
    ESP_RETURN_ON_ERROR(ledc_set_duty(mode, channel, duty), "h_bridge", "set pwm duty failed");
    ESP_RETURN_ON_ERROR(ledc_update_duty(mode, channel), "h_bridge", "update pwm duty failed");
    return ESP_OK;
}

static esp_err_t h_bridge_all_off(h_bridge_driver_t *driver)
{
    ESP_RETURN_ON_ERROR(h_bridge_set_pwm(driver->_config.ledc_mode, driver->_config.a_hi_channel, 0U), "h_bridge", "disable A_HI failed");
    ESP_RETURN_ON_ERROR(h_bridge_set_pwm(driver->_config.ledc_mode, driver->_config.a_lo_channel, 0U), "h_bridge", "disable A_LO failed");
    ESP_RETURN_ON_ERROR(h_bridge_set_pwm(driver->_config.ledc_mode, driver->_config.b_lo_channel, 0U), "h_bridge", "disable B_LO failed");

    h_bridge_set_gpio_level_if_valid(driver->_config.b_hi_gpio, 0);

    return ESP_OK;
}

static esp_err_t h_bridge_apply_accel(h_bridge_driver_t *driver, float target_norm, uint32_t duty)
{
    if (target_norm > 0.0f) {
        ESP_RETURN_ON_ERROR(h_bridge_set_pwm(driver->_config.ledc_mode, driver->_config.a_hi_channel, duty), "h_bridge", "set A_HI pwm failed");
        ESP_RETURN_ON_ERROR(h_bridge_set_pwm(driver->_config.ledc_mode, driver->_config.a_lo_channel, 0U), "h_bridge", "clear A_LO pwm failed");
        ESP_RETURN_ON_ERROR(h_bridge_set_pwm(driver->_config.ledc_mode, driver->_config.b_lo_channel, driver->_max_duty), "h_bridge", "set B_LO on failed");

        h_bridge_set_gpio_level_if_valid(driver->_config.b_hi_gpio, 0);
        return ESP_OK;
    }

    if (target_norm < 0.0f) {
        ESP_RETURN_ON_ERROR(h_bridge_set_pwm(driver->_config.ledc_mode, driver->_config.a_hi_channel, 0U), "h_bridge", "clear A_HI pwm failed");
        ESP_RETURN_ON_ERROR(h_bridge_set_pwm(driver->_config.ledc_mode, driver->_config.a_lo_channel, duty), "h_bridge", "set A_LO pwm failed");
        ESP_RETURN_ON_ERROR(h_bridge_set_pwm(driver->_config.ledc_mode, driver->_config.b_lo_channel, 0U), "h_bridge", "clear B_LO pwm failed");

        h_bridge_set_gpio_level_if_valid(driver->_config.b_hi_gpio, 1);
        return ESP_OK;
    }

    return h_bridge_all_off(driver);
}

static esp_err_t h_bridge_apply_brake(h_bridge_driver_t *driver, float target_norm, uint32_t duty)
{
    (void)target_norm;

    if (duty == 0U) {
        return h_bridge_all_off(driver);
    }

    ESP_RETURN_ON_ERROR(h_bridge_set_pwm(driver->_config.ledc_mode, driver->_config.a_hi_channel, 0U), "h_bridge", "clear A_HI pwm failed");
    ESP_RETURN_ON_ERROR(h_bridge_set_pwm(driver->_config.ledc_mode, driver->_config.a_lo_channel, duty), "h_bridge", "set A_LO brake pwm failed");
    ESP_RETURN_ON_ERROR(h_bridge_set_pwm(driver->_config.ledc_mode, driver->_config.b_lo_channel, duty), "h_bridge", "set B_LO brake pwm failed");
    h_bridge_set_gpio_level_if_valid(driver->_config.b_hi_gpio, 0);

    return ESP_OK;
}

esp_err_t h_bridge_driver_init(h_bridge_driver_t **out_driver, const h_bridge_driver_config_t *config)
{
    if (out_driver == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    h_bridge_driver_t *driver = calloc(1, sizeof(*driver));
    if (driver == NULL) {
        return ESP_ERR_NO_MEM;
    }

    driver->_config = *config;
    driver->_pwm_frequency_hz = config->pwm_frequency_hz;
    driver->_max_duty = (1U << (uint32_t)config->duty_resolution) - 1U;
    driver->_state_lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
    driver->_mode = H_BRIDGE_DRIVE_ACCEL;
    driver->_target_norm = 0.0f;
    driver->_running = false;

    esp_err_t err = h_bridge_setup_output_gpio(config->enable_gpio);
    if (err != ESP_OK) {
        free(driver);
        return err;
    }

    err = h_bridge_setup_output_gpio(config->b_hi_gpio);
    if (err != ESP_OK) {
        free(driver);
        return err;
    }

    ledc_timer_config_t timer_cfg = {
        .speed_mode = config->ledc_mode,
        .duty_resolution = config->duty_resolution,
        .timer_num = config->ledc_timer,
        .freq_hz = config->pwm_frequency_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) {
        free(driver);
        return err;
    }

    ledc_channel_config_t a_hi_cfg = {
        .gpio_num = config->a_hi_gpio,
        .speed_mode = config->ledc_mode,
        .channel = config->a_hi_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = config->ledc_timer,
        .duty = 0,
        .hpoint = 0,
    };
    err = ledc_channel_config(&a_hi_cfg);
    if (err != ESP_OK) {
        free(driver);
        return err;
    }

    ledc_channel_config_t a_lo_cfg = {
        .gpio_num = config->a_lo_gpio,
        .speed_mode = config->ledc_mode,
        .channel = config->a_lo_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = config->ledc_timer,
        .duty = 0,
        .hpoint = 0,
    };
    err = ledc_channel_config(&a_lo_cfg);
    if (err != ESP_OK) {
        free(driver);
        return err;
    }

    ledc_channel_config_t b_lo_cfg = {
        .gpio_num = config->b_lo_gpio,
        .speed_mode = config->ledc_mode,
        .channel = config->b_lo_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = config->ledc_timer,
        .duty = 0,
        .hpoint = 0,
    };
    err = ledc_channel_config(&b_lo_cfg);
    if (err != ESP_OK) {
        free(driver);
        return err;
    }

    h_bridge_set_gpio_level_if_valid(config->enable_gpio, 1);
    ESP_RETURN_ON_ERROR(h_bridge_all_off(driver), "h_bridge", "set initial all-off failed");

    *out_driver = driver;
    return ESP_OK;
}

void h_bridge_driver_deinit(h_bridge_driver_t *driver)
{
    if (driver == NULL) {
        return;
    }

    h_bridge_set(driver, H_BRIDGE_DRIVE_ACCEL, 0.0f, 0U);
    h_bridge_set_gpio_level_if_valid(driver->_config.enable_gpio, 0);

    free(driver);
}

esp_err_t h_bridge_set(h_bridge_driver_t *driver, h_bridge_drive_mode_t mode, float target_norm, uint32_t frequency_hz)
{
    if (driver == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    float requested_target_norm = target_norm;
    target_norm = h_bridge_clampf(target_norm, -1.0f, 1.0f);
    uint32_t duty = (uint32_t)lroundf(fabsf(target_norm) * ((float)driver->_max_duty));

    /*ESP_LOGI(TAG,
             "set request mode=%s target_in=%f target_clamped=%f duty=%" PRIu32 "/%" PRIu32 " freq_hz=%" PRIu32,
             mode == H_BRIDGE_DRIVE_ACCEL ? "ACCEL" : (mode == H_BRIDGE_DRIVE_BRAKE ? "BRAKE" : "UNKNOWN"),
             (double)requested_target_norm,
             (double)target_norm,
             duty,
             driver->_max_duty,
             frequency_hz);*/

    ESP_RETURN_ON_ERROR(h_bridge_update_frequency(driver, frequency_hz), "h_bridge", "frequency update failed");

    esp_err_t err = ESP_ERR_INVALID_ARG;

    if (mode == H_BRIDGE_DRIVE_ACCEL) {
        err = h_bridge_apply_accel(driver, target_norm, duty);
    } else if (mode == H_BRIDGE_DRIVE_BRAKE) {
        err = h_bridge_apply_brake(driver, target_norm, duty);
    }

    if (err != ESP_OK) {
        return err;
    }

    taskENTER_CRITICAL(&driver->_state_lock);
    driver->_mode = mode;
    driver->_target_norm = target_norm;
    driver->_running = (duty != 0U);
    taskEXIT_CRITICAL(&driver->_state_lock);

    /*ESP_LOGI(TAG,
             "set applied mode=%s target=%f running=%d pwm_freq_hz=%" PRIu32,
             mode == H_BRIDGE_DRIVE_ACCEL ? "ACCEL" : (mode == H_BRIDGE_DRIVE_BRAKE ? "BRAKE" : "UNKNOWN"),
             (double)target_norm,
             (duty != 0U) ? 1 : 0,
             driver->_pwm_frequency_hz);*/

    return ESP_OK;
}

esp_err_t h_bridge_get_state(h_bridge_driver_t *driver,
                             h_bridge_drive_mode_t *out_mode,
                             float *out_target_norm,
                             uint32_t *out_frequency_hz,
                             bool *out_running)
{
    if (driver == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    taskENTER_CRITICAL(&driver->_state_lock);
    if (out_mode != NULL) {
        *out_mode = driver->_mode;
    }
    if (out_target_norm != NULL) {
        *out_target_norm = driver->_target_norm;
    }
    if (out_frequency_hz != NULL) {
        *out_frequency_hz = driver->_pwm_frequency_hz;
    }
    if (out_running != NULL) {
        *out_running = driver->_running;
    }
    taskEXIT_CRITICAL(&driver->_state_lock);

    return ESP_OK;
}

bool h_bridge_is_running(h_bridge_driver_t *driver)
{
    bool running = false;

    if (h_bridge_get_state(driver, NULL, NULL, NULL, &running) != ESP_OK) {
        return false;
    }

    return running;
}
