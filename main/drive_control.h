#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drive_protocol.h"
#include "esp_err.h"
#include "h_bridge_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct drive_control_t drive_control_t;

typedef struct {
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
} drive_control_config_t;

typedef struct {
    h_bridge_drive_mode_t mode;
    float duty_cycle;
    uint32_t frequency_hz;
    bool outputs_enabled;
    drive_controller_mode_t controller_mode;
} drive_control_output_t;

esp_err_t drive_control_init(drive_control_t **out_control, const drive_control_config_t *config);
void drive_control_deinit(drive_control_t *control);

void drive_control_set_config(drive_control_t *control, const drive_control_config_t *config);
void drive_control_set_control_mode(drive_control_t *control, drive_control_mode_t control_mode);
void drive_control_set_request(drive_control_t *control, h_bridge_drive_mode_t mode, float duty_cycle, uint32_t frequency_hz);
void drive_control_clear_fault(drive_control_t *control);
bool drive_control_is_faulted(drive_control_t *control);
drive_control_mode_t drive_control_get_control_mode(drive_control_t *control);

esp_err_t drive_control_update(drive_control_t *control,
                               float measured_current_amps,
                               bool controller_alive,
                               bool calibrating,
                               uint32_t elapsed_ms,
                               drive_control_output_t *out_output);

float drive_control_get_integrator(drive_control_t *control);
float drive_control_get_applied_duty_cycle(drive_control_t *control);

#ifdef __cplusplus
}
#endif
