from __future__ import annotations

import tkinter as tk
from tkinter import ttk

from fdrive_py.driver_host import (
    CONTROLLER_MODE_FAULT_LOCKOUT,
    CONTROLLER_MODE_RUNNING,
    CONTROL_MODE_CURRENT,
    MODE_ACCEL,
    control_mode_from_name,
    controller_mode_name,
    response_type_name,
)

from .gauges import DialGauge
from .session import DriveSnapshot


class DriveStatusWidget(ttk.LabelFrame):
    CURRENT_WARNING_RATIO = 0.9
    CURRENT_POSITIVE_HEADROOM_RATIO = 1.25
    CURRENT_NEGATIVE_TO_POSITIVE_RATIO = 0.2
    CURRENT_ZERO_SCALE_RATIO = CURRENT_NEGATIVE_TO_POSITIVE_RATIO / (1.0 + CURRENT_NEGATIVE_TO_POSITIVE_RATIO)
    CURRENT_WARNING_COLOR = "#facc15"
    CURRENT_DANGER_COLOR = "#dc2626"
    CURRENT_LIMIT_FACE_COLOR = "#fef3c7"

    def __init__(self, master: tk.Misc, *, title: str = "Drive Status"):
        super().__init__(master, text=title)
        self.current_limit_amps: float | None = None
        self.control_mode: int | None = None

        gauges = ttk.Frame(self)
        gauges.grid(row=0, column=0, columnspan=2, sticky="ew", padx=8, pady=(8, 6))
        gauges.columnconfigure(0, weight=1)
        gauges.columnconfigure(1, weight=1)

        self.pwm_gauge = DialGauge(gauges, label="PWM", minimum=-100.0, maximum=100.0, unit="%")
        self.pwm_gauge.grid(row=0, column=0, sticky="ew", padx=(0, 4))

        self.current_gauge = DialGauge(gauges, label="Output Current", minimum=-10.0, maximum=10.0, unit="A")
        self.current_gauge.grid(row=0, column=1, sticky="ew", padx=(4, 0))
        self._configure_current_gauge(8.0)

        self.bus_voltage_var = tk.StringVar(value="-- V")
        self.drive_mode_var = tk.StringVar(value="--")
        self.controller_mode_var = tk.StringVar(value="--")
        self.frequency_var = tk.StringVar(value="-- Hz")
        self.fault_var = tk.StringVar(value="No telemetry")
        self.response_var = tk.StringVar(value="--")
        self.age_var = tk.StringVar(value="--")

        self._add_row(1, "Bus Voltage", self.bus_voltage_var)
        self._add_row(2, "Drive Mode", self.drive_mode_var)
        self._add_row(3, "Controller Mode", self.controller_mode_var)
        self._add_row(4, "PWM Frequency", self.frequency_var)
        self._add_row(5, "Warnings / Faults", self.fault_var)
        self._add_row(6, "Last Response", self.response_var)
        self._add_row(7, "Last Frame", self.age_var)

        self.columnconfigure(1, weight=1)

    def set_config_values(self, values: dict[str, object]) -> None:
        raw_limit = values.get("current_limit_amps")
        try:
            self.current_limit_amps = abs(float(raw_limit)) if raw_limit is not None else None
        except (TypeError, ValueError):
            self.current_limit_amps = None

        raw_mode = values.get("control_mode")
        try:
            if isinstance(raw_mode, str):
                self.control_mode = control_mode_from_name(raw_mode)
            elif raw_mode is None:
                self.control_mode = None
            else:
                self.control_mode = int(raw_mode)
        except (TypeError, ValueError):
            self.control_mode = None

        self._configure_current_gauge(self.current_limit_amps or 8.0)

    def _configure_current_gauge(self, nominal_limit: float) -> None:
        positive_gauge_limit = max(1.0, nominal_limit * self.CURRENT_POSITIVE_HEADROOM_RATIO)
        self.current_gauge.set_range(-positive_gauge_limit, positive_gauge_limit)
        self.current_gauge.set_scale_pivot(0.0, self.CURRENT_ZERO_SCALE_RATIO)
        self.current_gauge.set_scale_values(self._current_scale_values(positive_gauge_limit, nominal_limit))
        self._set_current_limit_zones(positive_gauge_limit)

    def set_snapshot(self, snapshot: DriveSnapshot) -> None:
        target_pwm = None if snapshot.target_duty_cycle is None else snapshot.target_duty_cycle * 100.0
        if snapshot.status is None:
            self.pwm_gauge.set_value(None, target=target_pwm, target_text=self._target_text(target_pwm, "%"))
            self.drive_mode_var.set("--")
            self.frequency_var.set("-- Hz")
        else:
            pwm_percent = snapshot.status.duty_cycle * 100.0
            self.pwm_gauge.set_value(pwm_percent, target=target_pwm, target_text=self._target_text(target_pwm, "%"))
            self.drive_mode_var.set("accel" if snapshot.status.mode == MODE_ACCEL else "brake")
            self.frequency_var.set(f"{snapshot.status.frequency_hz} Hz")

        target_current, target_current_text = self._target_current(snapshot)
        if snapshot.measurements is None:
            self.current_gauge.set_value(None, target=target_current, target_text=target_current_text)
            self._set_current_limit_face(None)
            self.bus_voltage_var.set("-- V")
        else:
            self.current_gauge.set_value(snapshot.measurements.current_amps, target=target_current, target_text=target_current_text)
            self._set_current_limit_face(snapshot.measurements.current_amps)
            self.bus_voltage_var.set(f"{snapshot.measurements.bus_voltage_volts:.3f} V")

        if snapshot.controller_mode is None:
            self.controller_mode_var.set("--")
        else:
            self.controller_mode_var.set(controller_mode_name(snapshot.controller_mode.mode))

        self.fault_var.set(self._fault_text(snapshot))

        if snapshot.response is None:
            self.response_var.set("--")
        else:
            self.response_var.set(
                f"{response_type_name(snapshot.response.response_type)} cmd=0x{snapshot.response.command:02X}"
            )

        if snapshot.last_frame_time is None:
            self.age_var.set("--")
        else:
            self.age_var.set("live")

    def _add_row(self, row: int, label: str, value: tk.StringVar) -> None:
        ttk.Label(self, text=label).grid(row=row, column=0, sticky="w", padx=(8, 4), pady=2)
        ttk.Label(self, textvariable=value).grid(row=row, column=1, sticky="ew", padx=(4, 8), pady=2)

    def _target_text(self, value: float | None, unit: str) -> str:
        if value is None:
            return "Target: --"
        return f"Target: {value:.2f} {unit}"

    def _target_current(self, snapshot: DriveSnapshot) -> tuple[float | None, str]:
        if snapshot.target_duty_cycle is None:
            return None, "Target: --"
        if self.current_limit_amps is None:
            return None, "Target: --"
        if self.control_mode != CONTROL_MODE_CURRENT:
            return None, "Target: --"
        target_current = abs(snapshot.target_duty_cycle) * self.current_limit_amps
        return target_current, f"Target: {target_current:.2f} A"

    def _current_scale_values(self, positive_limit: float, current_limit: float) -> tuple[float, ...]:
        return (
            -positive_limit,
            0.0,
            current_limit * 0.25,
            current_limit * 0.5,
            current_limit * 0.75,
            current_limit,
            positive_limit,
        )

    def _set_current_limit_zones(self, positive_gauge_limit: float) -> None:
        if self.current_limit_amps is None or self.current_limit_amps <= 0.0:
            self.current_gauge.set_zones(())
            return

        current_limit = min(self.current_limit_amps, positive_gauge_limit)
        warning_limit = min(current_limit * self.CURRENT_WARNING_RATIO, positive_gauge_limit)
        self.current_gauge.set_zones(
            (
                (-positive_gauge_limit, -current_limit, self.CURRENT_DANGER_COLOR),
                (-current_limit, -warning_limit, self.CURRENT_WARNING_COLOR),
                (warning_limit, current_limit, self.CURRENT_WARNING_COLOR),
                (current_limit, positive_gauge_limit, self.CURRENT_DANGER_COLOR),
            )
        )

    def _set_current_limit_face(self, current_amps: float | None) -> None:
        at_limit = (
            current_amps is not None
            and self.current_limit_amps is not None
            and self.current_limit_amps > 0.0
            and abs(current_amps) >= self.current_limit_amps
        )
        self.current_gauge.set_face_color(self.CURRENT_LIMIT_FACE_COLOR if at_limit else None)

    def _fault_text(self, snapshot: DriveSnapshot) -> str:
        if snapshot.last_error:
            return f"Transport error: {snapshot.last_error}"
        if snapshot.controller_mode is not None and snapshot.controller_mode.mode == CONTROLLER_MODE_FAULT_LOCKOUT:
            return "Fault lockout"
        if snapshot.measurements is not None and self.current_limit_amps:
            current = abs(snapshot.measurements.current_amps)
            if current >= self.current_limit_amps:
                return "At or above current limit"
            if current >= self.current_limit_amps * 0.9:
                return "Near current limit"
        if snapshot.controller_mode is not None and snapshot.controller_mode.mode != CONTROLLER_MODE_RUNNING:
            return controller_mode_name(snapshot.controller_mode.mode)
        if not snapshot.connected:
            return "Disconnected"
        return "OK"