#include "drive_control.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace {

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int duty_sign(float value)
{
    if (value > 0.0f) {
        return 1;
    }
    if (value < 0.0f) {
        return -1;
    }
    return 0;
}

} // namespace

struct drive_control_t {
    drive_control_config_t config = {};
    h_bridge_drive_mode_t requested_mode = H_BRIDGE_DRIVE_ACCEL;
    float requested_duty_cycle = 0.0f;
    uint32_t requested_frequency_hz = 0;
    float integrator = 0.0f;
    float applied_duty_cycle = 0.0f;
    h_bridge_drive_mode_t applied_mode = H_BRIDGE_DRIVE_ACCEL;
    int applied_sign = 0;
    bool fault_latched = false;
};

esp_err_t drive_control_init(drive_control_t **out_control, const drive_control_config_t *config)
{
    if (out_control == nullptr || config == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    drive_control_t *control = new drive_control_t();
    if (control == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    control->config = *config;
    control->requested_mode = H_BRIDGE_DRIVE_ACCEL;
    control->requested_duty_cycle = 0.0f;
    control->requested_frequency_hz = 0U;
    control->integrator = 0.0f;
    control->applied_duty_cycle = 0.0f;
    control->applied_mode = H_BRIDGE_DRIVE_ACCEL;
    control->applied_sign = 0;
    control->fault_latched = false;

    *out_control = control;
    return ESP_OK;
}

void drive_control_deinit(drive_control_t *control)
{
    delete control;
}

void drive_control_set_config(drive_control_t *control, const drive_control_config_t *config)
{
    if (control == nullptr || config == nullptr) {
        return;
    }

    control->config = *config;
}

void drive_control_set_control_mode(drive_control_t *control, drive_control_mode_t control_mode)
{
    if (control == nullptr) {
        return;
    }

    control->config.control_mode = control_mode;
}

void drive_control_set_request(drive_control_t *control, h_bridge_drive_mode_t mode, float duty_cycle, uint32_t frequency_hz)
{
    if (control == nullptr) {
        return;
    }

    control->requested_mode = mode;
    control->requested_duty_cycle = clamp_float(duty_cycle, -1.0f, 1.0f);
    control->requested_frequency_hz = frequency_hz;
}

void drive_control_clear_fault(drive_control_t *control)
{
    if (control == nullptr) {
        return;
    }

    control->fault_latched = false;
    control->integrator = 0.0f;
    control->applied_duty_cycle = 0.0f;
    control->applied_mode = control->requested_mode;
    control->applied_sign = 0;
}

bool drive_control_is_faulted(drive_control_t *control)
{
    return control != nullptr && control->fault_latched;
}

drive_control_mode_t drive_control_get_control_mode(drive_control_t *control)
{
    if (control == nullptr) {
        return DRIVE_CONTROL_MODE_PWM;
    }

    return control->config.control_mode;
}

esp_err_t drive_control_update(drive_control_t *control,
                               float measured_current_amps,
                               bool controller_alive,
                               bool calibrating,
                               uint32_t elapsed_ms,
                               drive_control_output_t *out_output)
{
    if (control == nullptr || out_output == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    out_output->mode = control->requested_mode;
    out_output->frequency_hz = control->requested_frequency_hz;
    out_output->duty_cycle = 0.0f;
    out_output->outputs_enabled = false;
    out_output->controller_mode = DRIVE_CONTROLLER_MODE_DISABLED;

    if (calibrating) {
        return ESP_OK;
    }

    if (!controller_alive) {
        return ESP_OK;
    }

    if (control->fault_latched) {
        out_output->controller_mode = DRIVE_CONTROLLER_MODE_FAULT_LOCKOUT;
        return ESP_OK;
    }

    if (control->config.control_mode == DRIVE_CONTROL_MODE_PWM) {
        const float requested_duty = clamp_float(control->requested_duty_cycle, -1.0f, 1.0f);
        const float requested_magnitude = std::fabs(requested_duty);
        const float measured_magnitude = std::fabs(measured_current_amps);
        const float current_limit = std::max(control->config.current_limit_amps, 0.0f);
        const float absolute_threshold = control->config.current_limit_amps + control->config.current_overcurrent_margin_amps;
        const float percent_threshold = control->config.current_limit_amps * control->config.current_overcurrent_margin_percent;

        if (measured_magnitude > absolute_threshold && measured_magnitude > percent_threshold) {
            control->fault_latched = true;
            control->integrator = 0.0f;
            control->applied_duty_cycle = 0.0f;
            control->applied_sign = 0;
            out_output->controller_mode = DRIVE_CONTROLLER_MODE_FAULT_LOCKOUT;
            return ESP_OK;
        }

        const int requested_sign = duty_sign(requested_duty);
        const bool reversing = control->applied_duty_cycle > 0.0f &&
                               (control->requested_mode != control->applied_mode ||
                                (requested_sign != 0 && requested_sign != control->applied_sign));
        if (reversing) {
            control->applied_duty_cycle = 0.0f;
            control->applied_sign = 0;
        }

        const float error_clamp = clamp_float(control->config.pwm_error_clamp, 0.0f, 1.0f);
        const float ramp_gain_per_s = std::max(control->config.pwm_ramp_up_per_sec, 0.0f);
        const float backoff_gain_per_s = std::max(control->config.pwm_backoff_per_sec, 0.0f);
        const float elapsed_s = (float)elapsed_ms / 1000.0f;

        if (requested_magnitude <= control->applied_duty_cycle) {
            control->applied_duty_cycle = requested_magnitude;
        } else {
            const float duty_error = requested_magnitude - control->applied_duty_cycle;
            float control_error = duty_error;

            if (current_limit > 0.0f) {
                const float current_error = (current_limit - measured_magnitude) / current_limit;
                control_error = std::min(duty_error, current_error);
            }

            if (control_error < 0.0f) {
                const float clamped_backoff_error = clamp_float(control_error, -error_clamp, 0.0f);
                control->applied_duty_cycle += clamped_backoff_error * backoff_gain_per_s * elapsed_s;
            } else {
                const float clamped_ramp_error = clamp_float(control_error, 0.0f, error_clamp);
                control->applied_duty_cycle += clamped_ramp_error * ramp_gain_per_s * elapsed_s;
            }
            control->applied_duty_cycle = clamp_float(control->applied_duty_cycle, 0.0f, requested_magnitude);
        }

        control->applied_mode = control->requested_mode;
        control->applied_sign = (control->applied_duty_cycle > 0.0f) ? requested_sign : 0;
        out_output->mode = control->requested_mode;
        out_output->duty_cycle = (requested_sign < 0) ? -control->applied_duty_cycle : control->applied_duty_cycle;
        out_output->outputs_enabled = control->applied_duty_cycle > 0.0f || requested_magnitude > 0.0f;
        out_output->controller_mode = DRIVE_CONTROLLER_MODE_RUNNING;
        control->integrator = 0.0f;
        return ESP_OK;
    }

    if (control->config.control_mode != DRIVE_CONTROL_MODE_CURRENT) {
        out_output->controller_mode = DRIVE_CONTROLLER_MODE_DISABLED;
        return ESP_OK;
    }

    const float requested_duty = clamp_float(control->requested_duty_cycle, -1.0f, 1.0f);
    const float command_magnitude = std::fabs(requested_duty);
    const int requested_sign = duty_sign(requested_duty);
    const float current_limit = std::max(control->config.current_limit_amps, 0.0f);

    if (command_magnitude <= 0.0f) {
        control->integrator = 0.0f;
        control->applied_duty_cycle = 0.0f;
        control->applied_mode = control->requested_mode;
        control->applied_sign = 0;
        out_output->mode = control->requested_mode;
        out_output->duty_cycle = 0.0f;
        out_output->outputs_enabled = false;
        out_output->controller_mode = DRIVE_CONTROLLER_MODE_RUNNING;
        return ESP_OK;
    }

    const bool reversing = control->applied_duty_cycle > 0.0f &&
                           (control->requested_mode != control->applied_mode ||
                            (control->requested_mode == H_BRIDGE_DRIVE_ACCEL &&
                             requested_sign != 0 &&
                             requested_sign != control->applied_sign));
    if (reversing) {
        control->integrator = 0.0f;
        control->applied_duty_cycle = 0.0f;
        control->applied_sign = 0;
    }

    const float target_current = clamp_float(command_magnitude * current_limit, 0.0f, current_limit);
    const float measured_magnitude = std::fabs(measured_current_amps);
    const float absolute_threshold = target_current + control->config.current_overcurrent_margin_amps;
    const float percent_threshold = target_current * control->config.current_overcurrent_margin_percent;

    if (measured_magnitude > absolute_threshold && measured_magnitude > percent_threshold) {
        control->fault_latched = true;
        control->integrator = 0.0f;
        control->applied_duty_cycle = 0.0f;
        control->applied_sign = 0;
        out_output->controller_mode = DRIVE_CONTROLLER_MODE_FAULT_LOCKOUT;
        return ESP_OK;
    }

    const float error = target_current - measured_magnitude;
    // Clamp error to prevent integrator windup and overshoot at large current steps.
    const float clamped_error = clamp_float(error, -control->config.current_error_clamp_amps, control->config.current_error_clamp_amps);
    const float elapsed_s = (float)elapsed_ms / 1000.0f;

    // Use asymmetric I-gains: kI_up for positive error, kI_down for negative error.
    const float ki_to_apply = (clamped_error >= 0.0f) ? control->config.current_ki_up : control->config.current_ki_down;
    control->integrator += clamped_error * ki_to_apply * elapsed_s;
    control->integrator = clamp_float(control->integrator, 0.0f, 1.0f);

    control->applied_duty_cycle = clamp_float(control->integrator, 0.0f, 1.0f);
    control->applied_mode = control->requested_mode;
    control->applied_sign = (control->applied_duty_cycle > 0.0f)
                                ? ((control->requested_mode == H_BRIDGE_DRIVE_ACCEL) ? requested_sign : 1)
                                : 0;
    out_output->mode = control->requested_mode;
    out_output->duty_cycle = (control->requested_mode == H_BRIDGE_DRIVE_ACCEL && requested_sign < 0)
                                 ? -control->applied_duty_cycle
                                 : control->applied_duty_cycle;
    out_output->outputs_enabled = control->applied_duty_cycle > 0.0f || target_current > 0.0f;
    out_output->controller_mode = DRIVE_CONTROLLER_MODE_RUNNING;
    return ESP_OK;
}

float drive_control_get_integrator(drive_control_t *control)
{
    if (control == nullptr) {
        return 0.0f;
    }

    return control->integrator;
}

float drive_control_get_applied_duty_cycle(drive_control_t *control)
{
    if (control == nullptr) {
        return 0.0f;
    }

    return control->applied_duty_cycle;
}
