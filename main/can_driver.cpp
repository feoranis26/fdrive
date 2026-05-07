#include "can_driver.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <inttypes.h>

#include "can_queue.hpp"
#include "command_router.h"
#include "drive_config_store.h"
#include "drive_control.h"
#include "drive_management.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "sense_service.h"

static const char *TAG = "can_driver";

namespace {

constexpr uint32_t kDefaultStatusPeriodMs = 50;
constexpr uint32_t kDefaultControlPeriodMs = 5;
constexpr uint32_t kDefaultCommandTimeoutMs = 100;
constexpr uint32_t kDefaultTaskStackSize = 4096;
constexpr UBaseType_t kDefaultTaskPriority = 8;
constexpr BaseType_t kControlCore = 1;
constexpr size_t kCommandPayloadSize = 7;
constexpr size_t kStatusPayloadSize = 8;
constexpr size_t kTelemetryPayloadSize = 8;
constexpr size_t kModePayloadSize = 1;

static TickType_t ms_to_ticks(uint32_t value_ms)
{
	return pdMS_TO_TICKS(value_ms);
}

static uint16_t decode_u16_le(const uint8_t *buffer)
{
	return (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
}

static void encode_u16_le(uint8_t *buffer, uint16_t value)
{
	buffer[0] = (uint8_t)(value & 0xFFU);
	buffer[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static uint16_t encode_frequency_field(uint32_t frequency_hz)
{
	uint32_t encoded = (frequency_hz + 5U) / 10U;
	if (encoded > UINT16_MAX) {
		encoded = UINT16_MAX;
	}
	return (uint16_t)encoded;
}

static uint32_t decode_frequency_field(uint16_t encoded_frequency)
{
	return (uint32_t)encoded_frequency * 10U;
}

static float clamp_duty(float duty)
{
	if (duty < -1.0f) {
		return -1.0f;
	}
	if (duty > 1.0f) {
		return 1.0f;
	}
	return duty;
}

} // namespace

struct can_driver_t {
	explicit can_driver_t(const can_driver_config_t &init_config)
		: config(init_config)
	{
	}

	can_driver_config_t config;
	can_queue_handle_t *queue_handle = nullptr;
	command_router_t *command_router = nullptr;
	drive_control_t *drive_control = nullptr;
	drive_management_service_t *management_service = nullptr;
	TaskHandle_t task = nullptr;
	TaskHandle_t control_task = nullptr;
	volatile bool stop_requested = false;
	portMUX_TYPE state_lock = portMUX_INITIALIZER_UNLOCKED;
	TickType_t last_command_tick = 0;
	TickType_t last_status_log_tick = 0;
	drive_controller_mode_t controller_mode = DRIVE_CONTROLLER_MODE_DISABLED;
	TickType_t last_control_tick = 0;
	drive_control_output_t last_output = {};
	uint8_t last_status_payload[kStatusPayloadSize] = {0};
	bool have_last_status_payload = false;
};

static bool can_driver_is_extended_id(uint32_t id)
{
	return id > TWAI_STD_ID_MASK;
}

static esp_err_t can_driver_apply_state(can_driver_t *driver,
										h_bridge_drive_mode_t mode,
										float duty_cycle,
										uint32_t frequency_hz,
										bool outputs_enabled)
{
	float applied_duty = outputs_enabled ? clamp_duty(duty_cycle) : 0.0f;

	return h_bridge_set(driver->config.h_bridge, mode, applied_duty, frequency_hz);
}

static esp_err_t can_driver_apply_output(can_driver_t *driver, const drive_control_output_t &output)
{
	return can_driver_apply_state(driver, output.mode, output.duty_cycle, output.frequency_hz, output.outputs_enabled);
}

static esp_err_t can_driver_disable_outputs(can_driver_t *driver)
{
	h_bridge_drive_mode_t mode = H_BRIDGE_DRIVE_ACCEL;
	float duty_cycle = 0.0f;
	uint32_t frequency_hz = 0;
	ESP_RETURN_ON_ERROR(h_bridge_get_state(driver->config.h_bridge, &mode, &duty_cycle, &frequency_hz, NULL), TAG, "failed to read h-bridge state");

	return can_driver_apply_state(driver, mode, 0.0f, frequency_hz, false);
}

static uint32_t can_driver_get_last_command_age_ms_internal(can_driver_t *driver, TickType_t now)
{
	TickType_t last_command_tick = 0;
	taskENTER_CRITICAL(&driver->state_lock);
	last_command_tick = driver->last_command_tick;
	taskEXIT_CRITICAL(&driver->state_lock);

	TickType_t age_ticks = now - last_command_tick;
	return (uint32_t)(age_ticks * portTICK_PERIOD_MS);
}

static bool can_driver_is_alive_internal(can_driver_t *driver, TickType_t now)
{
	if (driver->config.command_timeout_ms == 0U) {
		return true;
	}

	TickType_t last_command_tick = 0;
	taskENTER_CRITICAL(&driver->state_lock);
	last_command_tick = driver->last_command_tick;
	taskEXIT_CRITICAL(&driver->state_lock);

	return (now - last_command_tick) < ms_to_ticks(driver->config.command_timeout_ms);
}

static drive_controller_mode_t can_driver_compute_mode(can_driver_t *driver, TickType_t now)
{
	if (driver->config.sense_service != nullptr && sense_service_is_calibrating(driver->config.sense_service)) {
		return DRIVE_CONTROLLER_MODE_CALIBRATING;
	}
	if (drive_control_is_faulted(driver->drive_control)) {
		return DRIVE_CONTROLLER_MODE_FAULT_LOCKOUT;
	}
	if (!can_driver_is_alive_internal(driver, now)) {
		return DRIVE_CONTROLLER_MODE_DISABLED;
	}
	return DRIVE_CONTROLLER_MODE_RUNNING;
}

static esp_err_t can_driver_send_status(can_driver_t *driver)
{
	uint8_t payload[kStatusPayloadSize] = {0};
	h_bridge_drive_mode_t mode = H_BRIDGE_DRIVE_ACCEL;
	float duty_cycle = 0.0f;
	uint32_t frequency_hz = 0;
	ESP_RETURN_ON_ERROR(h_bridge_get_state(driver->config.h_bridge, &mode, &duty_cycle, &frequency_hz, NULL), TAG, "failed to read h-bridge state");

	payload[0] = (uint8_t)mode;

	uint16_t encoded_frequency = encode_frequency_field(frequency_hz);
	encode_u16_le(&payload[1], encoded_frequency);
	std::memcpy(&payload[3], &duty_cycle, sizeof(duty_cycle));
	payload[7] = DRIVE_STATUS_SIGNATURE;

	twai_frame_t frame = {};
	frame.header.id = driver->config.status_id;
	frame.header.ide = can_driver_is_extended_id(driver->config.status_id);
	frame.header.rtr = 0;
	frame.header.fdf = 0;
	frame.header.brs = 0;
	frame.header.esi = 0;
	frame.header.dlc = kStatusPayloadSize;
	frame.buffer = payload;
	frame.buffer_len = sizeof(payload);

	TickType_t now = xTaskGetTickCount();
	bool should_log_status = !driver->have_last_status_payload;
	if (!should_log_status) {
		should_log_status = memcmp(driver->last_status_payload, payload, sizeof(payload)) != 0;
	}
	if (!should_log_status && (now - driver->last_status_log_tick) >= pdMS_TO_TICKS(500)) {
		should_log_status = true;
	}
	if (should_log_status) {
		/*ESP_LOGI(TAG,
				 "tx status id=0x%08" PRIx32 " mode=%s freq_div10=%u freq_hz=%" PRIu32 " duty=%f raw=[%02x %02x %02x %02x %02x %02x %02x]",
				 driver->config.status_id,
				 mode_to_string(mode),
				 (unsigned int)encoded_frequency,
				 frequency_hz,
				 (double)duty_cycle,
				 payload[0],
				 payload[1],
				 payload[2],
				 payload[3],
				 payload[4],
				 payload[5],
				 payload[6]);*/
		memcpy(driver->last_status_payload, payload, sizeof(payload));
		driver->last_status_log_tick = now;
		driver->have_last_status_payload = true;
	}

	if (!driver->queue_handle->enqueue_tx(frame, 0)) {
		ESP_LOGW(TAG, "failed to queue status frame id=0x%08" PRIx32, driver->config.status_id);
		return ESP_FAIL;
	}

	return ESP_OK;
}

static esp_err_t can_driver_send_measurements(can_driver_t *driver)
{
	if (driver->config.sense_service == nullptr) {
		return ESP_ERR_INVALID_STATE;
	}

	sense_service_snapshot_t snapshot = {};
	ESP_RETURN_ON_ERROR(sense_service_get_snapshot(driver->config.sense_service, &snapshot), TAG, "failed to read sense snapshot");

	uint8_t payload[kTelemetryPayloadSize] = {0};
	std::memcpy(&payload[0], &snapshot.bus_voltage_volts, sizeof(snapshot.bus_voltage_volts));
	std::memcpy(&payload[4], &snapshot.current_amps, sizeof(snapshot.current_amps));

	twai_frame_t frame = {};
	frame.header.id = driver->config.measurement_id;
	frame.header.ide = can_driver_is_extended_id(driver->config.measurement_id);
	frame.header.rtr = 0;
	frame.header.fdf = 0;
	frame.header.brs = 0;
	frame.header.esi = 0;
	frame.header.dlc = kTelemetryPayloadSize;
	frame.buffer = payload;
	frame.buffer_len = sizeof(payload);

	if (!driver->queue_handle->enqueue_tx(frame, 0)) {
		return ESP_FAIL;
	}

	return ESP_OK;
}

static esp_err_t can_driver_send_mode(can_driver_t *driver, drive_controller_mode_t mode)
{
	uint8_t payload[kModePayloadSize] = {(uint8_t)mode};

	twai_frame_t frame = {};
	frame.header.id = driver->config.mode_id;
	frame.header.ide = can_driver_is_extended_id(driver->config.mode_id);
	frame.header.rtr = 0;
	frame.header.fdf = 0;
	frame.header.brs = 0;
	frame.header.esi = 0;
	frame.header.dlc = kModePayloadSize;
	frame.buffer = payload;
	frame.buffer_len = sizeof(payload);

	if (!driver->queue_handle->enqueue_tx(frame, 0)) {
		return ESP_FAIL;
	}

	return ESP_OK;
}

static esp_err_t can_driver_process_command(can_driver_t *driver, const twai_frame_t &frame)
{
	if (driver->config.sense_service != nullptr && sense_service_is_calibrating(driver->config.sense_service)) {
		return ESP_ERR_INVALID_STATE;
	}

	if (frame.header.rtr != 0U || frame.header.fdf != 0U) {
		ESP_LOGW(TAG, "ignoring unsupported frame type id=0x%08" PRIx32, frame.header.id);
		return ESP_ERR_NOT_SUPPORTED;
	}

	if (frame.buffer == nullptr || frame.buffer_len < kCommandPayloadSize || frame.header.dlc < kCommandPayloadSize) {
		ESP_LOGW(TAG, "ignoring short command frame id=0x%08" PRIx32 " len=%u dlc=%u",
				 frame.header.id,
				 (unsigned int)frame.buffer_len,
				 (unsigned int)frame.header.dlc);
		return ESP_ERR_INVALID_SIZE;
	}

	const uint8_t *payload = frame.buffer;
	h_bridge_drive_mode_t mode = H_BRIDGE_DRIVE_ACCEL;
	if (payload[0] == 0U) {
		mode = H_BRIDGE_DRIVE_ACCEL;
	} else if (payload[0] == 1U) {
		mode = H_BRIDGE_DRIVE_BRAKE;
	} else {
		ESP_LOGW(TAG, "ignoring command with invalid mode=%u", (unsigned int)payload[0]);
		return ESP_ERR_INVALID_ARG;
	}

	uint16_t encoded_frequency = decode_u16_le(&payload[1]);
	uint32_t frequency_hz = decode_frequency_field(encoded_frequency);

	float duty_cycle = 0.0f;
	std::memcpy(&duty_cycle, &payload[3], sizeof(duty_cycle));
	if (!std::isfinite(duty_cycle)) {
		ESP_LOGW(TAG, "ignoring command with non-finite duty cycle");
		return ESP_ERR_INVALID_ARG;
	}

	duty_cycle = clamp_duty(duty_cycle);
	if (mode == H_BRIDGE_DRIVE_BRAKE) {
		duty_cycle = fabsf(duty_cycle);
	}

	drive_control_set_request(driver->drive_control, mode, duty_cycle, frequency_hz);

	taskENTER_CRITICAL(&driver->state_lock);
	driver->last_command_tick = xTaskGetTickCount();
	taskEXIT_CRITICAL(&driver->state_lock);

	/*ESP_LOGI(TAG,
			 "rx cmd id=0x%08" PRIx32 " mode=%s freq_div10=%u freq_hz=%" PRIu32 " duty=%f raw=[%02x %02x %02x %02x %02x %02x %02x]",
			 frame.header.id,
			 mode_to_string(mode),
			 (unsigned int)encoded_frequency,
			 frequency_hz,
			 (double)duty_cycle,
			 payload[0],
			 payload[1],
			 payload[2],
			 payload[3],
			 payload[4],
			 payload[5],
			 payload[6]);*/

	(void)mode;
	/*ESP_LOGI(TAG,
			 "applied cmd mode=%s duty=%f freq_hz=%" PRIu32 " running=%d",
			 mode_to_string(mode),
			 (double)duty_cycle,
			 frequency_hz,
			 1);*/
	return ESP_OK;
}

static TickType_t can_driver_get_wait_ticks(can_driver_t *driver, TickType_t now, TickType_t next_status_tick)
{
	TickType_t wait_ticks = portMAX_DELAY;
	TickType_t last_command_tick = 0;
	bool driver_running = h_bridge_is_running(driver->config.h_bridge);

	taskENTER_CRITICAL(&driver->state_lock);
	last_command_tick = driver->last_command_tick;
	taskEXIT_CRITICAL(&driver->state_lock);

	if (driver_running && driver->config.command_timeout_ms != 0U) {
		TickType_t timeout_ticks = ms_to_ticks(driver->config.command_timeout_ms);
		TickType_t deadline = last_command_tick + timeout_ticks;
		if (deadline <= now) {
			return 0;
		}
		wait_ticks = deadline - now;
	}

	if (driver->config.status_period_ms != 0U) {
		if (next_status_tick <= now) {
			return 0;
		}
		TickType_t status_wait = next_status_tick - now;
		wait_ticks = (wait_ticks == portMAX_DELAY) ? status_wait : std::min(wait_ticks, status_wait);
	}

	return wait_ticks;
}

static void can_driver_control_task_entry(void *arg)
{
	can_driver_t *driver = static_cast<can_driver_t *>(arg);
	TickType_t frequency_window_start = xTaskGetTickCount();
	const TickType_t wait_ticks = (driver->config.command_timeout_ms == 0U)
		? portMAX_DELAY
		: std::max<TickType_t>(1, ms_to_ticks(driver->config.command_timeout_ms));
	int64_t last_control_time_us = 0;
	uint32_t loops_in_window = 0U;

	while (!driver->stop_requested) {
		(void)ulTaskNotifyTake(pdTRUE, wait_ticks);
		if (driver->stop_requested) {
			break;
		}

		if (driver->drive_control == nullptr || driver->config.sense_service == nullptr) {
			continue;
		}

		sense_service_snapshot_t snapshot = {};
		if (sense_service_refresh_snapshot(driver->config.sense_service, &snapshot) != ESP_OK) {
			continue;
		}

		const float measured_current_amps = snapshot.fast_current_amps;

		TickType_t now = xTaskGetTickCount();
		const int64_t now_us = esp_timer_get_time();
		taskENTER_CRITICAL(&driver->state_lock);
		driver->last_control_tick = now;
		taskEXIT_CRITICAL(&driver->state_lock);

		uint32_t elapsed_ms = driver->config.control_period_ms;
		if (last_control_time_us != 0) {
			elapsed_ms = (uint32_t)std::max<int64_t>(1, (now_us - last_control_time_us) / 1000);
		}
		last_control_time_us = now_us;

		drive_control_output_t output = {};
		(void)drive_control_update(driver->drive_control,
					   measured_current_amps,
						   can_driver_is_alive_internal(driver, now),
						   snapshot.calibrating,
						   elapsed_ms,
						   &output);

		drive_controller_mode_t mode = can_driver_compute_mode(driver, now);
		if (mode == DRIVE_CONTROLLER_MODE_CALIBRATING || mode == DRIVE_CONTROLLER_MODE_DISABLED || mode == DRIVE_CONTROLLER_MODE_FAULT_LOCKOUT) {
			output.outputs_enabled = false;
		}

		(void)can_driver_apply_output(driver, output);

		taskENTER_CRITICAL(&driver->state_lock);
		driver->controller_mode = mode;
		driver->last_output = output;
		taskEXIT_CRITICAL(&driver->state_lock);

		loops_in_window++;
		TickType_t now_after_update = xTaskGetTickCount();
		TickType_t window_elapsed = now_after_update - frequency_window_start;
		if (window_elapsed >= pdMS_TO_TICKS(1000)) {
			const float integrator = drive_control_get_integrator(driver->drive_control);
			const float applied_duty = drive_control_get_applied_duty_cycle(driver->drive_control);
			const float loop_hz = (float)loops_in_window * 1000.0f / (float)(window_elapsed * portTICK_PERIOD_MS);
			ESP_LOGI(TAG,
					 "control loop frequency=%.1f Hz samples=%" PRIu32 " mode=%d enabled=%d current=%.4f A slow_current=%.4f A current_v=%.4f V integrator=%.4f duty=%.4f elapsed=%" PRIu32 " ms",
					 (double)loop_hz,
					 (unsigned int)snapshot.sample_count,
					 (int)mode,
					 (int)output.outputs_enabled,
					 (double)snapshot.fast_current_amps,
					 (double)snapshot.current_amps,
					 (double)snapshot.raw_current_channel_volts,
					 (double)integrator,
					 (double)applied_duty,
					 (unsigned int)elapsed_ms);
			frequency_window_start = now_after_update;
			loops_in_window = 0U;
		}
	}

	driver->control_task = nullptr;
	vTaskDelete(nullptr);
}

static void can_driver_task_entry(void *arg)
{
	can_driver_t *driver = static_cast<can_driver_t *>(arg);
	TickType_t next_status_tick = xTaskGetTickCount() + ms_to_ticks(driver->config.status_period_ms);

	while (!driver->stop_requested) {
		TickType_t now = xTaskGetTickCount();
		TickType_t wait_ticks = can_driver_get_wait_ticks(driver, now, next_status_tick);

		twai_frame_t frame = {};
		bool received = driver->queue_handle->dequeue_rx(&frame, wait_ticks);
		now = xTaskGetTickCount();

		if (received) {
			(void)can_driver_process_command(driver, frame);
			free(frame.buffer);
			frame.buffer = nullptr;
		}
		drive_controller_mode_t mode = can_driver_compute_mode(driver, now);

		taskENTER_CRITICAL(&driver->state_lock);
		driver->controller_mode = mode;
		taskEXIT_CRITICAL(&driver->state_lock);

		if (driver->management_service != nullptr) {
			drive_management_service_poll(driver->management_service);
		}

		if (driver->config.command_timeout_ms != 0U) {
			if (!can_driver_is_alive_internal(driver, now) && h_bridge_is_running(driver->config.h_bridge)) {
				ESP_LOGW(TAG,
						 "disabling outputs due to command timeout age_ms=%" PRIu32,
						 can_driver_get_last_command_age_ms_internal(driver, now));
				(void)can_driver_disable_outputs(driver);
			}
		}

		if (driver->config.status_period_ms != 0U && now >= next_status_tick) {
			(void)can_driver_send_status(driver);
			(void)can_driver_send_mode(driver, mode);
			(void)can_driver_send_measurements(driver);
			next_status_tick = now + ms_to_ticks(driver->config.status_period_ms);
		}
	}

	driver->task = nullptr;
	vTaskDelete(nullptr);
}

extern "C" esp_err_t can_driver_init(can_driver_t **out_driver, const can_driver_config_t *config)
{
	if (out_driver == nullptr || config == nullptr || config->h_bridge == nullptr || config->control_queue_handle == nullptr ||
		config->command_queue_handle == nullptr || config->sense_service == nullptr || config->config_store == nullptr) {
		return ESP_ERR_INVALID_ARG;
	}

	if (config->status_id == config->control_id) {
		return ESP_ERR_INVALID_ARG;
	}

	can_driver_config_t normalized_config = *config;
	if (normalized_config.status_period_ms == 0U) {
		normalized_config.status_period_ms = kDefaultStatusPeriodMs;
	}
	if (normalized_config.command_timeout_ms == 0U) {
		normalized_config.command_timeout_ms = kDefaultCommandTimeoutMs;
	}
	if (normalized_config.task_stack_size == 0U) {
		normalized_config.task_stack_size = kDefaultTaskStackSize;
	}
	if (normalized_config.task_priority == 0U) {
		normalized_config.task_priority = kDefaultTaskPriority;
	}
	if (normalized_config.control_period_ms == 0U) {
		normalized_config.control_period_ms = kDefaultControlPeriodMs;
	}

	can_driver_t *driver = new can_driver_t(normalized_config);
	if (driver == nullptr) {
		return ESP_ERR_NO_MEM;
	}
	driver->queue_handle = static_cast<can_queue_handle_t *>(config->control_queue_handle);
	esp_err_t err = ESP_OK;

	drive_control_config_t control_config = {
		.control_mode = config->control_mode,
		.pwm_ramp_up_per_sec = config->pwm_ramp_up_per_sec,
		.pwm_backoff_per_sec = config->pwm_backoff_per_sec,
		.pwm_error_clamp = config->pwm_error_clamp,
		.current_limit_amps = config->current_limit_amps,
		.current_ki_up = config->current_ki_up,
		.current_ki_down = config->current_ki_down,
		.current_overcurrent_margin_amps = config->current_overcurrent_margin_amps,
		.current_overcurrent_margin_percent = config->current_overcurrent_margin_percent,
		.current_error_clamp_amps = config->current_error_clamp_amps,
	};
	err = drive_control_init(&driver->drive_control, &control_config);
	if (err != ESP_OK) {
		delete driver;
		return err;
	}

	command_router_config_t router_config = {
		.command_rx_id = config->command_id,
		.response_tx_id = config->response_id,
		.rx_handle = static_cast<can_queue_handle_t *>(config->command_queue_handle),
		.tx_handle = driver->queue_handle,
		.task_stack_size = 3072,
		.task_priority = driver->config.task_priority,
	};
	err = command_router_init(&driver->command_router, &router_config);
	if (err != ESP_OK) {
		drive_control_deinit(driver->drive_control);
		driver->drive_control = nullptr;
		delete driver;
		return err;
	}

	err = drive_management_service_init(&driver->management_service,
								   driver->command_router,
								   driver->config.config_store,
							   driver->config.sense_service,
							   driver->drive_control);
	if (err != ESP_OK) {
		drive_control_deinit(driver->drive_control);
		driver->drive_control = nullptr;
		command_router_deinit(driver->command_router);
		delete driver;
		return err;
	}

	driver->last_command_tick = xTaskGetTickCount();
	driver->last_control_tick = driver->last_command_tick;

	err = can_driver_disable_outputs(driver);
	if (err != ESP_OK) {
		drive_management_service_deinit(driver->management_service);
		driver->management_service = nullptr;
		command_router_deinit(driver->command_router);
		driver->command_router = nullptr;
		drive_control_deinit(driver->drive_control);
		driver->drive_control = nullptr;
		delete driver;
		return err;
	}

	BaseType_t task_ok = xTaskCreatePinnedToCore(can_driver_task_entry,
											  "can_driver_task",
											  driver->config.task_stack_size,
											  driver,
											  driver->config.task_priority,
											  &driver->task,
											  kControlCore);
	if (task_ok != pdPASS) {
		drive_management_service_deinit(driver->management_service);
		driver->management_service = nullptr;
		command_router_deinit(driver->command_router);
		drive_control_deinit(driver->drive_control);
		driver->drive_control = nullptr;
		delete driver;
		return ESP_ERR_NO_MEM;
	}

	BaseType_t control_task_ok = xTaskCreatePinnedToCore(can_driver_control_task_entry,
												  "can_control_task",
												  driver->config.task_stack_size,
												  driver,
												  driver->config.task_priority + 1U,
												  &driver->control_task,
												  kControlCore);
	if (control_task_ok != pdPASS) {
		driver->stop_requested = true;
		if (driver->task != nullptr) {
			vTaskDelete(driver->task);
			driver->task = nullptr;
		}
		drive_management_service_deinit(driver->management_service);
		driver->management_service = nullptr;
		command_router_deinit(driver->command_router);
		drive_control_deinit(driver->drive_control);
		driver->drive_control = nullptr;
		delete driver;
		return ESP_ERR_NO_MEM;
	}

	if (driver->config.sense_service != nullptr) {
		err = sense_service_register_data_ready_task(driver->config.sense_service, driver->control_task);
		if (err != ESP_OK) {
			driver->stop_requested = true;
			vTaskDelete(driver->control_task);
			driver->control_task = nullptr;
			vTaskDelete(driver->task);
			driver->task = nullptr;
			drive_management_service_deinit(driver->management_service);
			driver->management_service = nullptr;
			command_router_deinit(driver->command_router);
			drive_control_deinit(driver->drive_control);
			driver->drive_control = nullptr;
			delete driver;
			return err;
		}
	}

	*out_driver = driver;
	return ESP_OK;
}

extern "C" void can_driver_deinit(can_driver_t *driver)
{
	if (driver == nullptr) {
		return;
	}

	if (driver->config.sense_service != nullptr) {
		(void)sense_service_register_data_ready_task(driver->config.sense_service, nullptr);
	}

	driver->stop_requested = true;
	if (driver->control_task != nullptr) {
		xTaskNotifyGive(driver->control_task);
	}
	for (int attempt = 0; attempt < 5 && driver->task != nullptr; ++attempt) {
		vTaskDelay(pdMS_TO_TICKS(5));
	}
	for (int attempt = 0; attempt < 5 && driver->control_task != nullptr; ++attempt) {
		vTaskDelay(pdMS_TO_TICKS(5));
	}

	if (driver->task != nullptr) {
		vTaskDelete(driver->task);
		driver->task = nullptr;
	}
	if (driver->control_task != nullptr) {
		vTaskDelete(driver->control_task);
		driver->control_task = nullptr;
	}

	if (driver->command_router != nullptr) {
		if (driver->management_service != nullptr) {
			drive_management_service_deinit(driver->management_service);
			driver->management_service = nullptr;
		}
		command_router_deinit(driver->command_router);
		driver->command_router = nullptr;
	}

	if (driver->drive_control != nullptr) {
		drive_control_deinit(driver->drive_control);
		driver->drive_control = nullptr;
	}

	(void)can_driver_disable_outputs(driver);
	delete driver;
}

extern "C" esp_err_t can_driver_get_state(can_driver_t *driver,
											h_bridge_drive_mode_t *out_mode,
											float *out_duty_cycle,
											uint32_t *out_frequency_hz,
											bool *out_enabled)
{
	if (driver == nullptr) {
		return ESP_ERR_INVALID_ARG;
	}

	ESP_RETURN_ON_ERROR(h_bridge_get_state(driver->config.h_bridge,
									   out_mode,
									   out_duty_cycle,
									   out_frequency_hz,
									   out_enabled),
					 TAG,
					 "failed to read h-bridge state");

	return ESP_OK;
}

extern "C" bool can_driver_is_alive(can_driver_t *driver)
{
	if (driver == nullptr) {
		return false;
	}

	return can_driver_is_alive_internal(driver, xTaskGetTickCount());
}

extern "C" drive_controller_mode_t can_driver_get_controller_mode(can_driver_t *driver)
{
	if (driver == nullptr) {
		return DRIVE_CONTROLLER_MODE_DISABLED;
	}

	drive_controller_mode_t mode = DRIVE_CONTROLLER_MODE_DISABLED;
	taskENTER_CRITICAL(&driver->state_lock);
	mode = driver->controller_mode;
	taskEXIT_CRITICAL(&driver->state_lock);
	return mode;
}

extern "C" esp_err_t can_driver_get_last_command_age_ms(can_driver_t *driver, uint32_t *out_age_ms)
{
	if (driver == nullptr || out_age_ms == nullptr) {
		return ESP_ERR_INVALID_ARG;
	}

	TickType_t last_command_tick = 0;
	taskENTER_CRITICAL(&driver->state_lock);
	last_command_tick = driver->last_command_tick;
	taskEXIT_CRITICAL(&driver->state_lock);

	TickType_t now = xTaskGetTickCount();
	TickType_t age_ticks = now - last_command_tick;
	*out_age_ms = (uint32_t)(age_ticks * portTICK_PERIOD_MS);
	return ESP_OK;
}
