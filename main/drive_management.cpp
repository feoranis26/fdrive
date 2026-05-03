#include "drive_management.h"

#include <string.h>

#include "command_router.h"
#include "drive_config_store.h"
#include "drive_control.h"
#include "drive_protocol.h"
#include "sense_service.h"

struct drive_management_service_t {
    command_router_t *router;
    config_store_t *config_store;
    sense_service_t *sense_service;
    drive_control_t *drive_control;
    bool calibration_in_progress_sent;
};

static command_router_response_context_t drive_management_make_response_context(drive_management_service_t *service,
                                                                                uint8_t opcode)
{
    command_router_response_context_t ctx = {
        .router = service != nullptr ? service->router : nullptr,
        .opcode = opcode,
    };
    return ctx;
}

static bool drive_management_key_is_u32(drive_config_key_t key)
{
    return key == DRIVE_CONFIG_KEY_CAN_BASE_ID || key == DRIVE_CONFIG_KEY_CONTROL_MODE;
}

static bool drive_management_key_is_control_tuning(drive_config_key_t key)
{
    return key == DRIVE_CONFIG_KEY_CONTROL_MODE ||
           key == DRIVE_CONFIG_KEY_PWM_RAMP_UP_PER_SEC ||
           key == DRIVE_CONFIG_KEY_PWM_BACKOFF_PER_SEC ||
           key == DRIVE_CONFIG_KEY_PWM_ERROR_CLAMP ||
           key == DRIVE_CONFIG_KEY_CURRENT_LIMIT_AMPS ||
           key == DRIVE_CONFIG_KEY_CURRENT_KI_UP ||
           key == DRIVE_CONFIG_KEY_CURRENT_KI_DOWN ||
           key == DRIVE_CONFIG_KEY_CURRENT_OVERCURRENT_MARGIN_AMPS ||
           key == DRIVE_CONFIG_KEY_CURRENT_OVERCURRENT_MARGIN_PERCENT ||
           key == DRIVE_CONFIG_KEY_CURRENT_ERROR_CLAMP_AMPS;
}

static void drive_management_refresh_control_config(drive_management_service_t *service)
{
    if (service == NULL || service->drive_control == NULL) {
        return;
    }

    drive_control_config_t control_config = {};
    uint32_t control_mode = 0U;
    float pwm_ramp_up_per_sec = 0.0f;
    float pwm_backoff_per_sec = 0.0f;
    float pwm_error_clamp = 0.0f;
    float current_limit_amps = 0.0f;
    float current_ki_up = 0.0f;
    float current_ki_down = 0.0f;
    float margin = 0.0f;
    float margin_percent = 0.0f;
    float error_clamp = 0.0f;

    if (drive_config_store_get_u32(service->config_store, DRIVE_CONFIG_KEY_CONTROL_MODE, &control_mode) != ESP_OK ||
        drive_config_store_get_float(service->config_store, DRIVE_CONFIG_KEY_PWM_RAMP_UP_PER_SEC, &pwm_ramp_up_per_sec) != ESP_OK ||
        drive_config_store_get_float(service->config_store, DRIVE_CONFIG_KEY_PWM_BACKOFF_PER_SEC, &pwm_backoff_per_sec) != ESP_OK ||
        drive_config_store_get_float(service->config_store, DRIVE_CONFIG_KEY_PWM_ERROR_CLAMP, &pwm_error_clamp) != ESP_OK ||
        drive_config_store_get_float(service->config_store, DRIVE_CONFIG_KEY_CURRENT_LIMIT_AMPS, &current_limit_amps) != ESP_OK ||
        drive_config_store_get_float(service->config_store, DRIVE_CONFIG_KEY_CURRENT_KI_UP, &current_ki_up) != ESP_OK ||
        drive_config_store_get_float(service->config_store, DRIVE_CONFIG_KEY_CURRENT_KI_DOWN, &current_ki_down) != ESP_OK ||
        drive_config_store_get_float(service->config_store, DRIVE_CONFIG_KEY_CURRENT_OVERCURRENT_MARGIN_AMPS, &margin) != ESP_OK ||
        drive_config_store_get_float(service->config_store, DRIVE_CONFIG_KEY_CURRENT_OVERCURRENT_MARGIN_PERCENT, &margin_percent) != ESP_OK ||
        drive_config_store_get_float(service->config_store, DRIVE_CONFIG_KEY_CURRENT_ERROR_CLAMP_AMPS, &error_clamp) != ESP_OK) {
        return;
    }

    control_config.control_mode = (drive_control_mode_t)control_mode;
    control_config.pwm_ramp_up_per_sec = pwm_ramp_up_per_sec;
    control_config.pwm_backoff_per_sec = pwm_backoff_per_sec;
    control_config.pwm_error_clamp = pwm_error_clamp;
    control_config.current_limit_amps = current_limit_amps;
    control_config.current_ki_up = current_ki_up;
    control_config.current_ki_down = current_ki_down;
    control_config.current_overcurrent_margin_amps = margin;
    control_config.current_overcurrent_margin_percent = margin_percent;
    control_config.current_error_clamp_amps = error_clamp;
    drive_control_set_config(service->drive_control, &control_config);
}

static esp_err_t drive_management_handle_config_get(drive_management_service_t *service,
                                                    const uint8_t *payload,
                                                    size_t payload_len,
                                                    command_router_response_context_t *response_ctx)
{
    if (payload_len < 1U) {
        return command_router_send_error(response_ctx, DRIVE_COMMAND_ERROR_INVALID_PAYLOAD);
    }

    const drive_config_key_t key = (drive_config_key_t)payload[0];
    uint8_t response_payload[5] = {(uint8_t)key, 0, 0, 0, 0};
    esp_err_t err = ESP_OK;

    if (drive_management_key_is_u32(key)) {
        uint32_t value = 0U;
        err = drive_config_store_get_u32(service->config_store, key, &value);
        memcpy(&response_payload[1], &value, sizeof(value));
    } else if (key == DRIVE_CONFIG_KEY_CURRENT_ZERO_CHANNEL_VOLTS ||
               key == DRIVE_CONFIG_KEY_BUS_VOLTAGE_OFFSET_VOLTS ||
               key == DRIVE_CONFIG_KEY_PWM_RAMP_UP_PER_SEC ||
               key == DRIVE_CONFIG_KEY_PWM_BACKOFF_PER_SEC ||
               key == DRIVE_CONFIG_KEY_PWM_ERROR_CLAMP ||
               key == DRIVE_CONFIG_KEY_CURRENT_LIMIT_AMPS ||
               key == DRIVE_CONFIG_KEY_CURRENT_KI_UP ||
               key == DRIVE_CONFIG_KEY_CURRENT_KI_DOWN ||
               key == DRIVE_CONFIG_KEY_CURRENT_OVERCURRENT_MARGIN_AMPS ||
               key == DRIVE_CONFIG_KEY_CURRENT_OVERCURRENT_MARGIN_PERCENT ||
               key == DRIVE_CONFIG_KEY_CURRENT_ERROR_CLAMP_AMPS) {
        float value = 0.0f;
        err = drive_config_store_get_float(service->config_store, key, &value);
        memcpy(&response_payload[1], &value, sizeof(value));
    } else {
        return command_router_send_error(response_ctx, DRIVE_COMMAND_ERROR_UNSUPPORTED_KEY);
    }

    if (err != ESP_OK) {
        return command_router_send_error(response_ctx, DRIVE_COMMAND_ERROR_UNSUPPORTED_KEY);
    }

    return command_router_send_ok(response_ctx, response_payload, sizeof(response_payload));
}

static esp_err_t drive_management_handle_config_set(drive_management_service_t *service,
                                                    const uint8_t *payload,
                                                    size_t payload_len,
                                                    command_router_response_context_t *response_ctx)
{
    if (payload_len < 5U) {
        return command_router_send_error(response_ctx, DRIVE_COMMAND_ERROR_INVALID_PAYLOAD);
    }

    const drive_config_key_t key = (drive_config_key_t)payload[0];
    esp_err_t err = ESP_OK;

    if (drive_management_key_is_u32(key)) {
        uint32_t value = 0U;
        memcpy(&value, &payload[1], sizeof(value));
        err = drive_config_store_set_u32(service->config_store, key, value);
    } else if (key == DRIVE_CONFIG_KEY_CURRENT_ZERO_CHANNEL_VOLTS ||
               key == DRIVE_CONFIG_KEY_BUS_VOLTAGE_OFFSET_VOLTS ||
               key == DRIVE_CONFIG_KEY_PWM_RAMP_UP_PER_SEC ||
               key == DRIVE_CONFIG_KEY_PWM_BACKOFF_PER_SEC ||
               key == DRIVE_CONFIG_KEY_PWM_ERROR_CLAMP ||
               key == DRIVE_CONFIG_KEY_CURRENT_LIMIT_AMPS ||
               key == DRIVE_CONFIG_KEY_CURRENT_KI_UP ||
               key == DRIVE_CONFIG_KEY_CURRENT_KI_DOWN ||
               key == DRIVE_CONFIG_KEY_CURRENT_OVERCURRENT_MARGIN_AMPS ||
               key == DRIVE_CONFIG_KEY_CURRENT_OVERCURRENT_MARGIN_PERCENT ||
               key == DRIVE_CONFIG_KEY_CURRENT_ERROR_CLAMP_AMPS) {
        float value = 0.0f;
        memcpy(&value, &payload[1], sizeof(value));
        err = drive_config_store_set_float(service->config_store, key, value);
    } else {
        return command_router_send_error(response_ctx, DRIVE_COMMAND_ERROR_UNSUPPORTED_KEY);
    }

    if (err == ESP_OK) {
        err = drive_config_store_save(service->config_store);
    }

    if (err != ESP_OK) {
        return command_router_send_error(response_ctx, DRIVE_COMMAND_ERROR_STORAGE);
    }

    if (drive_management_key_is_control_tuning(key)) {
        drive_management_refresh_control_config(service);
    }

    return command_router_send_ok(response_ctx, payload, 5U);
}

static esp_err_t drive_management_handle_clear_fault(drive_management_service_t *service,
                                                     command_router_response_context_t *response_ctx)
{
    if (service->drive_control == nullptr) {
        return command_router_send_error(response_ctx, DRIVE_COMMAND_ERROR_INTERNAL);
    }

    drive_control_clear_fault(service->drive_control);
    return command_router_send_ok(response_ctx, NULL, 0);
}

static esp_err_t drive_management_handle_calibrate(drive_management_service_t *service,
                                                   const uint8_t *payload,
                                                   size_t payload_len,
                                                   command_router_response_context_t *response_ctx,
                                                   command_router_handler_result_t *out_result)
{
    if (payload_len < sizeof(float)) {
        return command_router_send_error(response_ctx, DRIVE_COMMAND_ERROR_INVALID_PAYLOAD);
    }

    const esp_err_t ack_err = command_router_send_ack(response_ctx);
    if (ack_err != ESP_OK) {
        return ack_err;
    }

    float known_voltage = 0.0f;
    memcpy(&known_voltage, payload, sizeof(known_voltage));
    const esp_err_t start_err = sense_service_start_calibration(service->sense_service, known_voltage);
    if (start_err != ESP_OK) {
        return command_router_send_error(response_ctx, DRIVE_COMMAND_ERROR_BUSY);
    }

    service->calibration_in_progress_sent = false;
    if (out_result != NULL) {
        *out_result = COMMAND_ROUTER_HANDLER_RESULT_ASYNC;
    }
    return ESP_OK;
}

static esp_err_t drive_management_handle_command(void *handler_ctx,
                                                 const command_router_request_t *request,
                                                 command_router_response_context_t *response_ctx,
                                                 command_router_handler_result_t *out_result)
{
    if (handler_ctx == NULL || request == NULL || response_ctx == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    drive_management_service_t *service = (drive_management_service_t *)handler_ctx;

    switch (request->opcode) {
    case DRIVE_COMMAND_CALIBRATE:
        return drive_management_handle_calibrate(service, request->payload, request->payload_len, response_ctx, out_result);
    case DRIVE_COMMAND_CONFIG_GET:
        return drive_management_handle_config_get(service, request->payload, request->payload_len, response_ctx);
    case DRIVE_COMMAND_CONFIG_SET:
        return drive_management_handle_config_set(service, request->payload, request->payload_len, response_ctx);
    case DRIVE_COMMAND_CLEAR_FAULT:
        return drive_management_handle_clear_fault(service, response_ctx);
    default:
        return command_router_send_error(response_ctx, DRIVE_COMMAND_ERROR_UNSUPPORTED_KEY);
    }
}

esp_err_t drive_management_service_init(drive_management_service_t **out_service,
                                        command_router_t *router,
                                        config_store_t *config_store,
                                        sense_service_t *sense_service,
                                        drive_control_t *drive_control)
{
    if (out_service == NULL || router == NULL || config_store == NULL || sense_service == NULL || drive_control == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    drive_management_service_t *service = (drive_management_service_t *)calloc(1, sizeof(*service));
    if (service == NULL) {
        return ESP_ERR_NO_MEM;
    }

    service->router = router;
    service->config_store = config_store;
    service->sense_service = sense_service;
    service->drive_control = drive_control;
    drive_management_refresh_control_config(service);

    const command_router_handler_entry_t handlers[] = {
        {.opcode = DRIVE_COMMAND_CALIBRATE, .handler = drive_management_handle_command, .handler_ctx = service, .allow_while_busy = true},
        {.opcode = DRIVE_COMMAND_CONFIG_GET, .handler = drive_management_handle_command, .handler_ctx = service, .allow_while_busy = true},
        {.opcode = DRIVE_COMMAND_CONFIG_SET, .handler = drive_management_handle_command, .handler_ctx = service, .allow_while_busy = true},
        {.opcode = DRIVE_COMMAND_CLEAR_FAULT, .handler = drive_management_handle_command, .handler_ctx = service, .allow_while_busy = true},
    };
    const esp_err_t err = command_router_register_handlers(router, handlers, sizeof(handlers) / sizeof(handlers[0]));
    if (err != ESP_OK) {
        free(service);
        return err;
    }

    *out_service = service;
    return ESP_OK;
}

void drive_management_service_deinit(drive_management_service_t *service)
{
    if (service == NULL) {
        return;
    }

    free(service);
}

void drive_management_service_poll(drive_management_service_t *service)
{
    if (service == NULL) {
        return;
    }

    if (sense_service_is_calibrating(service->sense_service)) {
        if (!service->calibration_in_progress_sent) {
            command_router_response_context_t response_ctx =
                drive_management_make_response_context(service, DRIVE_COMMAND_CALIBRATE);
            (void)command_router_send_in_progress(&response_ctx, NULL, 0);
            service->calibration_in_progress_sent = true;
        }
        return;
    }

    sense_service_calibration_result_t calibration_result = {};
    if (!sense_service_take_calibration_result(service->sense_service, &calibration_result)) {
        service->calibration_in_progress_sent = false;
        return;
    }

    service->calibration_in_progress_sent = false;
    command_router_response_context_t response_ctx =
        drive_management_make_response_context(service, DRIVE_COMMAND_CALIBRATE);
    if (calibration_result.result == ESP_OK) {
        uint8_t payload[4] = {0};
        memcpy(payload, &calibration_result.measured_bus_voltage, sizeof(calibration_result.measured_bus_voltage));
        (void)command_router_send_ok(&response_ctx, payload, sizeof(payload));
        return;
    }

    (void)command_router_send_error(&response_ctx, DRIVE_COMMAND_ERROR_INTERNAL);
}