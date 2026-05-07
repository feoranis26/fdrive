from __future__ import annotations

import math
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox, ttk
from typing import Callable

from fdrive_py.driver_host import CONTROL_MODE_CHOICES

from .session import DriverSession


@dataclass(frozen=True)
class JoystickSettings:
    joystick_index: int
    throttle_axis: int
    reverse_toggle_button: int
    quit_button: int
    frequency_hz: int
    max_duty: float
    deadzone: float
    loop_rate_hz: float
    invert_axis: bool
    control_mode: str | None


class JoystickPanel(ttk.Frame):
    def __init__(self, master: tk.Misc, *, session: DriverSession, set_status: Callable[[str], None]):
        super().__init__(master)
        self.session = session
        self.set_status = set_status
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._reverse_enabled = False

        self.joystick_index_var = tk.StringVar(value="0")
        self.throttle_axis_var = tk.StringVar(value="1")
        self.reverse_button_var = tk.StringVar(value="0")
        self.quit_button_var = tk.StringVar(value="6")
        self.frequency_var = tk.StringVar(value="1000")
        self.max_duty_var = tk.StringVar(value="1.0")
        self.deadzone_var = tk.StringVar(value="0.1")
        self.loop_rate_var = tk.StringVar(value="20.0")
        self.invert_axis_var = tk.BooleanVar(value=False)
        self.control_mode_var = tk.StringVar(value="")
        self.demand_var = tk.StringVar(value="Demand: --")
        self.reverse_var = tk.StringVar(value="Negative throttle: brake")

        form = ttk.Frame(self)
        form.grid(row=0, column=0, sticky="nsew", padx=8, pady=8)
        self._add_labeled(form, 0, "Joystick Index", ttk.Entry(form, textvariable=self.joystick_index_var, width=10))
        self._add_labeled(form, 1, "Throttle Axis", ttk.Entry(form, textvariable=self.throttle_axis_var, width=10))
        self._add_labeled(form, 2, "Reverse Button", ttk.Entry(form, textvariable=self.reverse_button_var, width=10))
        self._add_labeled(form, 3, "Stop Button", ttk.Entry(form, textvariable=self.quit_button_var, width=10))
        self._add_labeled(form, 4, "PWM Frequency", ttk.Entry(form, textvariable=self.frequency_var, width=10))
        self._add_labeled(form, 5, "Max Duty", ttk.Entry(form, textvariable=self.max_duty_var, width=10))
        self._add_labeled(form, 6, "Deadzone", ttk.Entry(form, textvariable=self.deadzone_var, width=10))
        self._add_labeled(form, 7, "Loop Rate", ttk.Entry(form, textvariable=self.loop_rate_var, width=10))
        self._add_labeled(
            form,
            8,
            "Set Control Mode",
            ttk.Combobox(form, textvariable=self.control_mode_var, values=("",) + CONTROL_MODE_CHOICES, width=10, state="readonly"),
        )
        ttk.Checkbutton(form, text="Invert Axis", variable=self.invert_axis_var).grid(row=9, column=1, sticky="w", pady=3)
        form.columnconfigure(1, weight=1)

        buttons = ttk.Frame(self)
        buttons.grid(row=0, column=1, sticky="ns", padx=8, pady=8)
        self.start_button = ttk.Button(buttons, text="Start Joystick", command=self.start)
        self.start_button.grid(row=0, column=0, sticky="ew", pady=2)
        self.stop_button = ttk.Button(buttons, text="Stop", command=self.stop, state="disabled")
        self.stop_button.grid(row=1, column=0, sticky="ew", pady=2)
        ttk.Label(buttons, textvariable=self.demand_var).grid(row=2, column=0, sticky="w", pady=(12, 2))
        ttk.Label(buttons, textvariable=self.reverse_var).grid(row=3, column=0, sticky="w", pady=2)

        self.columnconfigure(0, weight=1)

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        try:
            settings = self._read_settings()
        except ValueError as exc:
            messagebox.showerror("Invalid Joystick Settings", str(exc), parent=self)
            return
        if not self.session.connected:
            messagebox.showerror("Not Connected", "Connect to a driver before starting joystick control.", parent=self)
            return

        self._stop_event.clear()
        self._reverse_enabled = False
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self._thread = threading.Thread(target=self._run_joystick, args=(settings,), daemon=True)
        self._thread.start()
        self.set_status("Joystick control started.")

    def stop(self) -> None:
        self._stop_event.set()

    def shutdown(self) -> None:
        self.stop()
        if self._thread is not None and self._thread.is_alive() and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)

    def _read_settings(self) -> JoystickSettings:
        deadzone = float(self.deadzone_var.get())
        max_duty = float(self.max_duty_var.get())
        loop_rate = float(self.loop_rate_var.get())
        if not 0.0 <= deadzone < 1.0:
            raise ValueError("deadzone must be in the range [0.0, 1.0)")
        if not 0.0 < max_duty <= 1.0:
            raise ValueError("max duty must be in the range (0.0, 1.0]")
        if loop_rate <= 0.0:
            raise ValueError("loop rate must be greater than zero")
        return JoystickSettings(
            joystick_index=int(self.joystick_index_var.get()),
            throttle_axis=int(self.throttle_axis_var.get()),
            reverse_toggle_button=int(self.reverse_button_var.get()),
            quit_button=int(self.quit_button_var.get()),
            frequency_hz=int(self.frequency_var.get()),
            max_duty=max_duty,
            deadzone=deadzone,
            loop_rate_hz=loop_rate,
            invert_axis=bool(self.invert_axis_var.get()),
            control_mode=self.control_mode_var.get() or None,
        )

    def _run_joystick(self, settings: JoystickSettings) -> None:
        pygame = None
        try:
            import pygame as pygame_module

            pygame = pygame_module
            pygame.display.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() <= settings.joystick_index:
                raise RuntimeError(
                    f"joystick index {settings.joystick_index} unavailable; found {pygame.joystick.get_count()} device(s)"
                )
            joystick = pygame.joystick.Joystick(settings.joystick_index)
            joystick.init()

            if settings.control_mode is not None:
                self.session.set_config_value("control_mode", settings.control_mode)

            reverse_button_latched = False
            update_period = 1.0 / settings.loop_rate_hz
            while not self._stop_event.is_set():
                loop_start = time.monotonic()
                pygame.event.pump()

                reverse_pressed = bool(joystick.get_button(settings.reverse_toggle_button))
                if reverse_pressed and not reverse_button_latched:
                    self._reverse_enabled = not self._reverse_enabled
                reverse_button_latched = reverse_pressed

                if settings.quit_button >= 0 and joystick.get_button(settings.quit_button):
                    break

                demand = self._read_axis(joystick, settings)
                self.session.send_signed_duty(demand, settings.frequency_hz, reverse_enabled=self._reverse_enabled)
                self.after(0, lambda value=demand, reverse=self._reverse_enabled: self._set_live_values(value, reverse))

                elapsed = time.monotonic() - loop_start
                if elapsed < update_period:
                    time.sleep(update_period - elapsed)
        except Exception as exc:
            self.after(0, lambda error=exc: self._show_error(error))
        finally:
            try:
                if self.session.connected:
                    self.session.stop(settings.frequency_hz)
            except Exception:
                pass
            if pygame is not None:
                pygame.joystick.quit()
                pygame.display.quit()
            self.after(0, self._stopped)

    def _read_axis(self, joystick: object, settings: JoystickSettings) -> float:
        raw_value = joystick.get_axis(settings.throttle_axis)
        if not settings.invert_axis:
            raw_value = -raw_value
        magnitude = abs(raw_value)
        if magnitude <= settings.deadzone:
            return 0.0
        scaled = (magnitude - settings.deadzone) / (1.0 - settings.deadzone)
        scaled = min(max(scaled, 0.0), 1.0)
        return math.copysign(scaled * settings.max_duty, raw_value)

    def _set_live_values(self, demand: float, reverse_enabled: bool) -> None:
        self.demand_var.set(f"Demand: {demand:+.3f}")
        self.reverse_var.set(f"Negative throttle: {'reverse' if reverse_enabled else 'brake'}")

    def _show_error(self, exc: Exception) -> None:
        self.set_status(str(exc))
        messagebox.showerror("Joystick Failed", str(exc), parent=self)

    def _stopped(self) -> None:
        self.start_button.configure(state="normal")
        self.stop_button.configure(state="disabled")
        self.set_status("Joystick control stopped.")

    def _add_labeled(self, parent: ttk.Frame, row: int, label: str, widget: tk.Widget) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", padx=(0, 8), pady=2)
        widget.grid(row=row, column=1, sticky="ew", pady=2)
