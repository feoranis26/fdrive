#include <stdio.h>

#include "ads131m06.h"
#include "can_driver.h"
#include "can_serial_bridge.h"
#include "can_queue.hpp"
#include "config_store.h"
#include "drive_config_store.h"
#include "drive_protocol.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "h_bridge_driver.h"
#include "led_ind.h"
#include "sense_service.h"

#define LED_STRIP_RMT_RES_HZ (10 * 1000 * 1000)

#define DRV_EN GPIO_NUM_23
#define A_HI GPIO_NUM_16
#define A_HI_CH LEDC_CHANNEL_0

#define A_LO GPIO_NUM_17
#define A_LO_CH LEDC_CHANNEL_1

#define B_HI GPIO_NUM_18
#define B_LO GPIO_NUM_19
#define B_LO_CH LEDC_CHANNEL_2

#define ADC_DRDY_GPIO GPIO_NUM_34

#define CAN_TX_GPIO GPIO_NUM_26
#define CAN_RX_GPIO GPIO_NUM_27

#define CAN_BASE_ID_DEFAULT 0x100U

#define LED_STATUS_TASK_STACK_SIZE 3072
#define LED_STATUS_TASK_PRIORITY 4
#define LED_STATUS_UPDATE_MS 25
#define APP_BACKGROUND_CORE 0

#define LED_GREEN_R 0
#define LED_GREEN_G 32
#define LED_GREEN_B 0

#define LED_YELLOW_R 24
#define LED_YELLOW_G 16
#define LED_YELLOW_B 0

#define LED_ORANGE_R 32
#define LED_ORANGE_G 12
#define LED_ORANGE_B 0

#define LED_WHITE_R 18
#define LED_WHITE_G 18
#define LED_WHITE_B 18

#define LED_RED_R 32
#define LED_RED_G 0
#define LED_RED_B 0

#define LED_PURPLE_R 24
#define LED_PURPLE_G 0
#define LED_PURPLE_B 24

static const char *TAG = "main";

static led_ind_t *indicator = NULL;
static h_bridge_driver_t *driver = NULL;
static ads131m06_t *adc_device = NULL;
static sense_service_t *sense_service = NULL;
static can_driver_t *can_driver = NULL;
static can_queue_t *can_queue = nullptr;
static can_queue_handle_t *can_control_handle = nullptr;
static can_queue_handle_t *can_command_handle = nullptr;
static can_queue_handle_t *can_sniffer_handle = nullptr;
static config_store_t *config_store = NULL;
static can_serial_bridge_t *serial_bridge = NULL;

static bool get_driver_running(void)
{
    return h_bridge_is_running(driver);
}

static bool get_can_alive(void)
{
    return can_driver_is_alive(can_driver);
}

static float get_driver_abs_duty(void)
{
    float target_norm = 0.0f;

    if (h_bridge_get_state(driver, NULL, &target_norm, NULL, NULL) != ESP_OK) {
        return 0.0f;
    }

    if (target_norm < 0.0f) {
        target_norm = -target_norm;
    }

    if (target_norm > 1.0f) {
        target_norm = 1.0f;
    }

    return target_norm;
}

static h_bridge_drive_mode_t get_driver_mode(void)
{
    h_bridge_drive_mode_t mode = H_BRIDGE_DRIVE_ACCEL;
    (void)h_bridge_get_state(driver, &mode, NULL, NULL, NULL);
    return mode;
}

static float lerpf(float start, float end, float t)
{
    if (t < 0.0f) {
        t = 0.0f;
    }
    if (t > 1.0f) {
        t = 1.0f;
    }
    return start + ((end - start) * t);
}

static void led_status_task(void *arg)
{
    (void)arg;

    while (true) {
        bool can_alive = get_can_alive();
        bool driver_running = get_driver_running();
        h_bridge_drive_mode_t mode = get_driver_mode();
        float duty = get_driver_abs_duty();
        drive_controller_mode_t controller_mode = can_driver_get_controller_mode(can_driver);

        if (controller_mode == DRIVE_CONTROLLER_MODE_FAULT_LOCKOUT) {
            ESP_ERROR_CHECK(led_ind_set_color(indicator, LED_RED_R, LED_RED_G, LED_RED_B));
            ESP_ERROR_CHECK(led_ind_set_off_color(indicator, LED_ORANGE_R, LED_ORANGE_G, LED_ORANGE_B));
            ESP_ERROR_CHECK(led_ind_set_blink(indicator, 8.0f, 0.5f));
        } else if (controller_mode == DRIVE_CONTROLLER_MODE_CALIBRATING) {
            ESP_ERROR_CHECK(led_ind_set_off_color(indicator, 0, 0, 0));
            ESP_ERROR_CHECK(led_ind_set_color(indicator, LED_PURPLE_R, LED_PURPLE_G, LED_PURPLE_B));
            ESP_ERROR_CHECK(led_ind_set_blink(indicator, 2.0f, 0.5f));
        } else if (!can_alive) {
            ESP_ERROR_CHECK(led_ind_set_off_color(indicator, 0, 0, 0));
            ESP_ERROR_CHECK(led_ind_set_color(indicator, LED_RED_R, LED_RED_G, LED_RED_B));
            ESP_ERROR_CHECK(led_ind_set_blink(indicator, 1.0f, 0.5f));
        } else if (controller_mode == DRIVE_CONTROLLER_MODE_DISABLED) {
            ESP_ERROR_CHECK(led_ind_set_off_color(indicator, 0, 0, 0));
            ESP_ERROR_CHECK(led_ind_set_color(indicator, LED_WHITE_R, LED_WHITE_G, LED_WHITE_B));
            ESP_ERROR_CHECK(led_ind_set_blink(indicator, 0.0f, 1.0f));
        } else if (driver_running && mode == H_BRIDGE_DRIVE_BRAKE) {
            ESP_ERROR_CHECK(led_ind_set_off_color(indicator, 0, 0, 0));
            ESP_ERROR_CHECK(led_ind_set_color(indicator, LED_YELLOW_R, LED_YELLOW_G, LED_YELLOW_B));
            ESP_ERROR_CHECK(led_ind_set_blink(indicator, 4.0f, 0.5f));
        } else if (driver_running) {
            ESP_ERROR_CHECK(led_ind_set_off_color(indicator, 0, 0, 0));
            float blink_hz = lerpf(1.0f, 8.0f, duty);
            ESP_ERROR_CHECK(led_ind_set_color(indicator, LED_GREEN_R, LED_GREEN_G, LED_GREEN_B));
            ESP_ERROR_CHECK(led_ind_set_blink(indicator, blink_hz, 0.5f));
        } else {
            ESP_ERROR_CHECK(led_ind_set_off_color(indicator, 0, 0, 0));
            ESP_ERROR_CHECK(led_ind_set_color(indicator, LED_WHITE_R, LED_WHITE_G, LED_WHITE_B));
            ESP_ERROR_CHECK(led_ind_set_blink(indicator, 0.0f, 1.0f));
        }

        vTaskDelay(pdMS_TO_TICKS(LED_STATUS_UPDATE_MS));
    }
}

static void adc_debug_task(void *arg)
{
    sense_service_t *sense = (sense_service_t *)arg;

    ESP_LOGI(TAG, "Starting ADC debug task");

    while (true) {
        sense_service_snapshot_t snapshot = {};
        esp_err_t sense_err = sense_service_get_snapshot(sense, &snapshot);

        if (sense_err == ESP_OK) {
            uint32_t drdy_count = sense_service_get_drdy_count(sense);
            uint32_t drdy_hz = drdy_count * 4U;  /* Called every 250ms, so multiply by 4 for Hz */

            ESP_LOGI(TAG,
                     "ADC drdy_hz=%lu samples=%lu status=0x%04x current=%.3f A bus=%.3f V",
                     (unsigned long)drdy_hz,
                     (unsigned long)snapshot.sample_count,
                     (unsigned int)snapshot.adc_status,
                     snapshot.current_amps,
                     snapshot.bus_voltage_volts);
        } else {
            ESP_LOGW(TAG, "Failed to read sense snapshot (err=0x%x)", (unsigned int)sense_err);
        }

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

extern "C" void app_main(void)
{
    led_ind_config_t indicator_cfg = {
        .strip_gpio = GPIO_NUM_33,
        .rmt_resolution_hz = LED_STRIP_RMT_RES_HZ,
        .pixel_count = 1,
    };
    ESP_ERROR_CHECK(led_ind_init(&indicator, &indicator_cfg));
    ESP_ERROR_CHECK(led_ind_set_color(indicator, LED_RED_R, LED_RED_G, LED_RED_B));
    ESP_ERROR_CHECK(led_ind_set_blink(indicator, 0.0f, 1.0f));

    h_bridge_driver_config_t driver_cfg = {
        .enable_gpio = DRV_EN,
        .a_hi_gpio = A_HI,
        .a_lo_gpio = A_LO,
        .b_hi_gpio = B_HI,
        .b_lo_gpio = B_LO,
        .ledc_mode = LEDC_HIGH_SPEED_MODE,
        .ledc_timer = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .a_hi_channel = A_HI_CH,
        .a_lo_channel = A_LO_CH,
        .b_lo_channel = B_LO_CH,
        .pwm_frequency_hz = 1000,
    };
    ESP_ERROR_CHECK(h_bridge_driver_init(&driver, &driver_cfg));

    can_queue_config_t can_queue_cfg = {};
    can_queue_cfg.node_cfg = {
        .io_cfg = {
            .tx = CAN_TX_GPIO,
            .rx = CAN_RX_GPIO,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .clk_src = TWAI_CLK_SRC_DEFAULT,
        .bit_timing = {
            .bitrate = 500000,
            .sp_permill = 800,
            .ssp_permill = 0,
        },
        .data_timing = {
            .bitrate = 0,
            .sp_permill = 0,
            .ssp_permill = 0,
        },
        .fail_retry_cnt = 3,
        .tx_queue_depth = 16,
        .intr_priority = 0,
        .flags = {
            .enable_self_test = 0,
            .enable_loopback = 0,
            .enable_listen_only = 0,
            .no_receive_rtr = 1,
        },
    };
    can_queue_cfg.tx_queue_len = 32;
    can_queue_cfg.rx_queue_len = 16;

    can_queue = new can_queue_t(can_queue_cfg);
    ESP_ERROR_CHECK(can_queue != nullptr ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(can_queue->start());

    drive_config_defaults_t config_defaults = {
        .can_base_id = CAN_BASE_ID_DEFAULT,
        .current_zero_channel_volts = 0.0f,
        .bus_voltage_offset_volts = 0.0f,
        .control_mode = DRIVE_CONTROL_MODE_PWM,
        .pwm_ramp_up_per_sec = 1.0f,
        .pwm_backoff_per_sec = 2.0f,
        .pwm_error_clamp = 1.0f,
        .current_limit_amps = 5.0f,
        .current_ki_up = 0.30f,
        .current_ki_down = 0.15f,
        .current_overcurrent_margin_amps = 1.0f,
        .current_overcurrent_margin_percent = 0.10f,
        .current_error_clamp_amps = 50.0f,
        .current_inverted = 0U,
    };
    ESP_ERROR_CHECK(drive_config_store_init(&config_store, &config_defaults));

    float pwm_ramp_up_per_sec = config_defaults.pwm_ramp_up_per_sec;
    float pwm_backoff_per_sec = config_defaults.pwm_backoff_per_sec;
    float pwm_error_clamp = config_defaults.pwm_error_clamp;

    uint32_t can_base_id = CAN_BASE_ID_DEFAULT;
    (void)drive_config_store_get_u32(config_store, DRIVE_CONFIG_KEY_CAN_BASE_ID, &can_base_id);
    uint32_t control_mode = DRIVE_CONTROL_MODE_PWM;
    (void)drive_config_store_get_u32(config_store, DRIVE_CONFIG_KEY_CONTROL_MODE, &control_mode);
    (void)drive_config_store_get_float(config_store, DRIVE_CONFIG_KEY_PWM_RAMP_UP_PER_SEC, &pwm_ramp_up_per_sec);
    (void)drive_config_store_get_float(config_store, DRIVE_CONFIG_KEY_PWM_BACKOFF_PER_SEC, &pwm_backoff_per_sec);
    (void)drive_config_store_get_float(config_store, DRIVE_CONFIG_KEY_PWM_ERROR_CLAMP, &pwm_error_clamp);
    float current_limit_amps = 5.0f;
    (void)drive_config_store_get_float(config_store, DRIVE_CONFIG_KEY_CURRENT_LIMIT_AMPS, &current_limit_amps);
    float current_ki_up = 0.30f;
    (void)drive_config_store_get_float(config_store, DRIVE_CONFIG_KEY_CURRENT_KI_UP, &current_ki_up);
    float current_ki_down = 0.15f;
    (void)drive_config_store_get_float(config_store, DRIVE_CONFIG_KEY_CURRENT_KI_DOWN, &current_ki_down);
    float current_overcurrent_margin_amps = 1.0f;
    (void)drive_config_store_get_float(config_store, DRIVE_CONFIG_KEY_CURRENT_OVERCURRENT_MARGIN_AMPS, &current_overcurrent_margin_amps);
    float current_overcurrent_margin_percent = 0.10f;
    (void)drive_config_store_get_float(config_store, DRIVE_CONFIG_KEY_CURRENT_OVERCURRENT_MARGIN_PERCENT, &current_overcurrent_margin_percent);
    float current_error_clamp_amps = 50.0f;
    (void)drive_config_store_get_float(config_store, DRIVE_CONFIG_KEY_CURRENT_ERROR_CLAMP_AMPS, &current_error_clamp_amps);

    const uint32_t can_control_id = drive_protocol_make_id(can_base_id, DRIVE_CAN_OFFSET_CONTROL_RX);
    const uint32_t can_status_id = drive_protocol_make_id(can_base_id, DRIVE_CAN_OFFSET_STATUS_TX);
    const uint32_t can_mode_id = drive_protocol_make_id(can_base_id, DRIVE_CAN_OFFSET_MODE_TX);
    const uint32_t can_measurement_id = drive_protocol_make_id(can_base_id, DRIVE_CAN_OFFSET_MEASUREMENT_TX);
    const uint32_t can_command_id = drive_protocol_make_id(can_base_id, DRIVE_CAN_OFFSET_COMMAND_RX);
    const uint32_t can_response_id = drive_protocol_make_id(can_base_id, DRIVE_CAN_OFFSET_RESPONSE_TX);

    can_control_handle = can_queue->get_handle(can_control_id, can_control_id);
    ESP_ERROR_CHECK(can_control_handle != nullptr ? ESP_OK : ESP_ERR_NO_MEM);

    can_command_handle = can_queue->get_handle(can_command_id, can_command_id);
    ESP_ERROR_CHECK(can_command_handle != nullptr ? ESP_OK : ESP_ERR_NO_MEM);

    can_sniffer_handle = can_queue->get_handle(0U, 0x1FFFFFFFU);
    ESP_ERROR_CHECK(can_sniffer_handle != nullptr ? ESP_OK : ESP_ERR_NO_MEM);

    BaseType_t led_task_ok = xTaskCreatePinnedToCore(led_status_task,
                                                     "led_status_task",
                                                     LED_STATUS_TASK_STACK_SIZE,
                                                     NULL,
                                                     LED_STATUS_TASK_PRIORITY,
                                                     NULL,
                                                     APP_BACKGROUND_CORE);
    if (led_task_ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create led_status_task");
    }

    
    ads131m06_config_t adc_cfg = {
        .spi_host = SPI2_HOST,
        .mosi_gpio = GPIO_NUM_25,
        .miso_gpio = GPIO_NUM_32,
        .sclk_gpio = GPIO_NUM_13,
        .cs_gpio = GPIO_NUM_5,
        .drdy_gpio = ADC_DRDY_GPIO,
        .spi_clock_hz = 2000000, // increased SPI clock for faster transfers
        .spi_mode = 1,
        .average_sample_count = 1, // no additional fast averaging; rely on ADC OSR filtering
        .average_slow_sample_count = 1024, // slow-average window for telemetry
        .vref_volts = 1.2f,
        .gain = 1.0f,
        .external_reference_enable = false,
        .drdy_pulse_mode = false,
        .osr_mask = ADS131M06_CLOCK_OSR_8192_MASK, // HR mode: ~500 SPS with strong ADC-side averaging
        .channel_enable_mask = 0x3F,
    };

    esp_err_t adc_err = ads131m06_init(&adc_device, &adc_cfg);
    if (adc_err == ESP_OK) {
        sense_service_config_t sense_cfg = {
            .adc = adc_device,
            .config_store = config_store,
            .calibration_settle_ms = 1500,
            .calibration_samples = 2048,
            .calibration_sample_period_ms = 25,
        };
        ESP_ERROR_CHECK(sense_service_init(&sense_service, &sense_cfg));

        can_driver_config_t can_cfg = {
            .h_bridge = driver,
            .control_queue_handle = can_control_handle,
            .command_queue_handle = can_command_handle,
            .sense_service = sense_service,
            .config_store = config_store,
            .control_mode = (drive_control_mode_t)control_mode,
            .pwm_ramp_up_per_sec = pwm_ramp_up_per_sec,
            .pwm_backoff_per_sec = pwm_backoff_per_sec,
            .pwm_error_clamp = pwm_error_clamp,
            .current_limit_amps = current_limit_amps,
            .current_ki_up = current_ki_up,
            .current_ki_down = current_ki_down,
            .current_overcurrent_margin_amps = current_overcurrent_margin_amps,
            .current_overcurrent_margin_percent = current_overcurrent_margin_percent,
            .current_error_clamp_amps = current_error_clamp_amps,
            .control_id = can_control_id,
            .status_id = can_status_id,
            .mode_id = can_mode_id,
            .measurement_id = can_measurement_id,
            .command_id = can_command_id,
            .response_id = can_response_id,
            .status_period_ms = 50,
            .command_timeout_ms = 100,
            .task_stack_size = 4096,
            .control_period_ms = 0,
            .task_priority = 8,
        };
        ESP_ERROR_CHECK(can_driver_init(&can_driver, &can_cfg));

        can_serial_bridge_config_t bridge_cfg = {
            .uart_port = UART_NUM_0,
            .can_queue = can_queue,
            .sniffer_handle = can_sniffer_handle,
        };
        ESP_ERROR_CHECK(can_serial_bridge_init(&serial_bridge, &bridge_cfg));

        BaseType_t task_ok = xTaskCreatePinnedToCore(adc_debug_task,
                                 "adc_debug_task",
                                 3072,
                                 sense_service,
                                 4,
                                 NULL,
                                 APP_BACKGROUND_CORE);
        if (task_ok != pdPASS) {
            ESP_LOGE(TAG, "Failed to create adc_debug_task");
        }
    } else {
        ESP_LOGE(TAG, "ADS131M06 init failed (err=0x%x)", (unsigned int)adc_err);
    }

    ESP_LOGI(TAG,
             "Initialization complete, CAN base=0x%03lx control_mode=%lu current_limit=%.3fA ctrl=0x%03lx status=0x%03lx mode=0x%03lx meas=0x%03lx cmd=0x%03lx rsp=0x%03lx",
             (unsigned long)can_base_id,
             (unsigned long)control_mode,
             (double)current_limit_amps,
             (unsigned long)can_control_id,
             (unsigned long)can_status_id,
             (unsigned long)can_mode_id,
             (unsigned long)can_measurement_id,
             (unsigned long)can_command_id,
             (unsigned long)can_response_id);
}