#include "led_ind.h"

#include <math.h>
#include <stdlib.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "led_strip.h"

#define LED_IND_TASK_STACK_SIZE 2048
#define LED_IND_TASK_PRIORITY 3
#define LED_IND_DEFAULT_RMT_RES_HZ (10 * 1000 * 1000)
#define LED_IND_MIN_PERIOD_MS 20U
#define LED_IND_UPDATE_MS 20U

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} led_ind_rgb_t;

struct led_ind_t {
    led_strip_handle_t _strip;
    TaskHandle_t _task;

    led_ind_rgb_t _color;
    led_ind_rgb_t _off_color;
    float _blink_hz;
    float _blink_duty;
    bool _running;
};

static float led_ind_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static esp_err_t led_ind_render(led_ind_t *led)
{
    led_ind_rgb_t color = {0};
    float blink_hz = 0.0f;
    float blink_duty = 1.0f;

    color = led->_color;

    blink_hz = led->_blink_hz;
    blink_duty = led->_blink_duty;

    const float period_ms_f = 1000.0f / blink_hz;
    uint32_t period_ms = (uint32_t)lroundf(period_ms_f);
    uint32_t on_ms = (uint32_t)lroundf(((float)period_ms) * blink_duty);
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    uint32_t phase_ms = now_ms % period_ms;

    bool out_on = true;

    if (blink_hz > 0.0f) {
        out_on = (phase_ms < on_ms);
    }

    if (!out_on) {
        color = led->_off_color;
    }

    esp_err_t err = led_strip_set_pixel(led->_strip, 0, color.r, color.g, color.b);
    if (err != ESP_OK) {
        return err;
    }
    return led_strip_refresh(led->_strip);
}

static void led_ind_task(void *arg)
{
    led_ind_t *led = (led_ind_t *)arg;

    while (led->_running) {
        if (!led->_running) {
            break;
        }

        float blink_hz = led->_blink_hz;
        float blink_duty = led->_blink_duty;

        led_ind_render(led);
        vTaskDelay(pdMS_TO_TICKS(LED_IND_UPDATE_MS));
    }

    led_ind_set_color(led, 0, 0, 0);
    led_ind_render(led);
    led->_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t led_ind_init(led_ind_t **out_led, const led_ind_config_t *config)
{
    if (out_led == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    led_ind_t *led = calloc(1, sizeof(*led));
    if (led == NULL) {
        return ESP_ERR_NO_MEM;
    }

    led_strip_config_t strip_config = {
        .strip_gpio_num = config->strip_gpio,
        .max_leds = (config->pixel_count == 0U) ? 1U : config->pixel_count,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {
            .invert_out = false,
        },
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = (config->rmt_resolution_hz == 0U) ? LED_IND_DEFAULT_RMT_RES_HZ : config->rmt_resolution_hz,
        .mem_block_symbols = 0,
        .flags = {
            .with_dma = 0,
        },
    };

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &led->_strip);
    if (err != ESP_OK) {
        free(led);
        return err;
    }

    led->_color.r = 16;
    led->_blink_duty = 1.0f;
    led->_off_color.r = 0;
    led->_off_color.g = 0;
    led->_off_color.b = 0;
    led->_running = true;

    err = xTaskCreate(led_ind_task, "led_ind_task", LED_IND_TASK_STACK_SIZE, led, LED_IND_TASK_PRIORITY, &led->_task) == pdPASS
              ? ESP_OK
              : ESP_FAIL;
    if (err != ESP_OK) {
        led_strip_del(led->_strip);
        free(led);
        return err;
    }

    *out_led = led;
    return ESP_OK;
}

void led_ind_deinit(led_ind_t *led)
{
    if (led == NULL) {
        return;
    }

    led->_running = false;

    while (led->_task != NULL) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (led->_strip != NULL) {
        led_strip_clear(led->_strip);
        led_strip_del(led->_strip);
    }

    free(led);
}

esp_err_t led_ind_set_color(led_ind_t *led, uint8_t r, uint8_t g, uint8_t b)
{
    if (led == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    led->_color.r = r;
    led->_color.g = g;
    led->_color.b = b;

    return ESP_OK;
}

esp_err_t led_ind_set_off_color(led_ind_t *led, uint8_t r, uint8_t g, uint8_t b)
{
    if (led == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    led->_off_color.r = r;
    led->_off_color.g = g;
    led->_off_color.b = b;

    return ESP_OK;
}

esp_err_t led_ind_set_blink(led_ind_t *led, float hz, float duty_cycle)
{
    if (led == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    float clamped_hz = led_ind_clampf(hz, 0.0f, (1000.0f / (float)LED_IND_MIN_PERIOD_MS));
    float clamped_duty = led_ind_clampf(duty_cycle, 0.0f, 1.0f);

    led->_blink_hz = clamped_hz;
    led->_blink_duty = clamped_duty;

    return ESP_OK;
}
