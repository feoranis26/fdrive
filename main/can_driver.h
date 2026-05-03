#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "config_store.h"
#include "esp_err.h"
#include "drive_protocol.h"
#include "freertos/FreeRTOS.h"
#include "h_bridge_driver.h"

typedef struct sense_service_t sense_service_t;

#ifdef __cplusplus
#include "can_queue.hpp"
typedef can_queue_handle_t can_driver_queue_handle_t;
#else
typedef void can_driver_queue_handle_t;
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct can_driver_t can_driver_t;

typedef struct {
	h_bridge_driver_t *h_bridge;
	can_driver_queue_handle_t *control_queue_handle;
	can_driver_queue_handle_t *command_queue_handle;
	sense_service_t *sense_service;
	config_store_t *config_store;
	drive_control_mode_t control_mode;
	float pwm_ramp_up_per_sec;
	float pwm_backoff_per_sec;
	float pwm_error_clamp;
	float current_limit_amps;
	float current_ki_up;
	float current_ki_down;
	float current_overcurrent_margin_amps;
	float current_overcurrent_margin_percent;
	float current_error_clamp_amps;
	uint32_t control_id;
	uint32_t status_id;
	uint32_t mode_id;
	uint32_t measurement_id;
	uint32_t command_id;
	uint32_t response_id;
	uint32_t status_period_ms;
	uint32_t command_timeout_ms;
	uint32_t task_stack_size;
	uint32_t control_period_ms;
	UBaseType_t task_priority;
} can_driver_config_t;

esp_err_t can_driver_init(can_driver_t **out_driver, const can_driver_config_t *config);
void can_driver_deinit(can_driver_t *driver);

bool can_driver_is_alive(can_driver_t *driver);
esp_err_t can_driver_get_last_command_age_ms(can_driver_t *driver, uint32_t *out_age_ms);
drive_controller_mode_t can_driver_get_controller_mode(can_driver_t *driver);

esp_err_t can_driver_get_state(can_driver_t *driver,
							   h_bridge_drive_mode_t *out_mode,
							   float *out_duty_cycle,
							   uint32_t *out_frequency_hz,
							   bool *out_enabled);

#ifdef __cplusplus
}
#endif
