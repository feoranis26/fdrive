from __future__ import annotations

import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from typing import Callable

from fdrive_py.driver_host import CONFIG_FIELDS, CONTROL_MODE_CHOICES, DriverConfigProfile, ConfigValue

from .session import ConfigOperationResult, DriverSession


class ConfigPanel(ttk.Frame):
    def __init__(
        self,
        master: tk.Misc,
        *,
        session: DriverSession,
        set_status: Callable[[str], None],
        on_base_id_changed: Callable[[int], None],
        on_config_values: Callable[[dict[str, object]], None],
    ):
        super().__init__(master)
        self.session = session
        self.set_status = set_status
        self.on_base_id_changed = on_base_id_changed
        self.on_config_values = on_config_values
        self.vars = {field.name: tk.StringVar(value="") for field in CONFIG_FIELDS}

        form = ttk.Frame(self)
        form.grid(row=0, column=0, sticky="nsew", padx=8, pady=8)
        for row, field in enumerate(CONFIG_FIELDS):
            ttk.Label(form, text=field.label).grid(row=row, column=0, sticky="w", padx=(0, 8), pady=2)
            if field.value_kind == "control_mode":
                widget = ttk.Combobox(form, textvariable=self.vars[field.name], values=CONTROL_MODE_CHOICES, state="readonly", width=18)
            else:
                widget = ttk.Entry(form, textvariable=self.vars[field.name], width=22)
            widget.grid(row=row, column=1, sticky="ew", pady=2)
        form.columnconfigure(1, weight=1)

        buttons = ttk.Frame(self)
        buttons.grid(row=0, column=1, sticky="ns", padx=8, pady=8)
        ttk.Button(buttons, text="Load From Driver", command=self.load_from_driver).grid(row=0, column=0, sticky="ew", pady=2)
        ttk.Button(buttons, text="Save To Driver", command=self.save_to_driver).grid(row=1, column=0, sticky="ew", pady=2)
        ttk.Separator(buttons).grid(row=2, column=0, sticky="ew", pady=8)
        ttk.Button(buttons, text="Import JSON", command=self.import_json).grid(row=3, column=0, sticky="ew", pady=2)
        ttk.Button(buttons, text="Export JSON", command=self.export_json).grid(row=4, column=0, sticky="ew", pady=2)
        ttk.Separator(buttons).grid(row=5, column=0, sticky="ew", pady=8)

        self.known_voltage_var = tk.StringVar(value="24.0")
        ttk.Label(buttons, text="Known Voltage").grid(row=6, column=0, sticky="w")
        ttk.Entry(buttons, textvariable=self.known_voltage_var, width=12).grid(row=7, column=0, sticky="ew", pady=2)
        ttk.Button(buttons, text="Calibrate", command=self.calibrate).grid(row=8, column=0, sticky="ew", pady=2)
        ttk.Button(buttons, text="Clear Fault", command=self.clear_fault).grid(row=9, column=0, sticky="ew", pady=2)

        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

    def apply_values(self, values: dict[str, object]) -> None:
        for field in CONFIG_FIELDS:
            if field.name in values:
                self.vars[field.name].set(field.format(values[field.name]))
        self.on_config_values(values)

    def read_values(self) -> dict[str, ConfigValue]:
        values: dict[str, ConfigValue] = {}
        for field in CONFIG_FIELDS:
            text = self.vars[field.name].get().strip()
            if text:
                values[field.name] = field.parse(text)
        return values

    def load_from_driver(self) -> None:
        self._run_worker(
            "Loading config from driver...",
            self.session.read_all_config_tolerant,
            self._finish_load_from_driver,
        )

    def _finish_load_from_driver(self, result: object) -> None:
        if not isinstance(result, ConfigOperationResult):
            return
        if result.values:
            self.apply_values(result.values)
        if result.errors:
            self._show_operation_errors("Config Load", result)
            self.set_status(f"Loaded {len(result.values)} config values; {len(result.errors)} failed.")
            return
        self.set_status("Config loaded from driver.")

    def save_to_driver(self) -> None:
        try:
            values = self.read_values()
        except ValueError as exc:
            messagebox.showerror("Invalid Config", str(exc), parent=self)
            return
        if not values:
            messagebox.showerror("Invalid Config", "No config values are filled in.", parent=self)
            return

        def done(result: object) -> None:
            if not isinstance(result, ConfigOperationResult):
                return
            if "can_base_id" in result.values:
                self.on_base_id_changed(int(result.values["can_base_id"]))
            if result.values:
                self.on_config_values(result.values)
            if result.errors:
                self._show_operation_errors("Config Save", result)
                self.set_status(f"Saved {len(result.values)} config values; {len(result.errors)} failed.")
                return
            self.set_status("Config saved to driver.")

        self._run_worker("Saving config to driver...", lambda: self.session.write_all_config_tolerant(values), done)

    def import_json(self) -> None:
        path = filedialog.askopenfilename(
            parent=self,
            title="Import Driver Config",
            filetypes=(("JSON files", "*.json"), ("All files", "*.*")),
        )
        if not path:
            return
        try:
            profile = DriverConfigProfile.load_json(path)
        except Exception as exc:
            messagebox.showerror("Import Failed", str(exc), parent=self)
            return
        self.apply_values(profile.values)
        if profile.base_id is not None:
            self.on_base_id_changed(profile.base_id)
        self.set_status(f"Imported {path}")

    def export_json(self) -> None:
        try:
            values = self.read_values()
        except ValueError as exc:
            messagebox.showerror("Invalid Config", str(exc), parent=self)
            return
        path = filedialog.asksaveasfilename(
            parent=self,
            title="Export Driver Config",
            defaultextension=".json",
            filetypes=(("JSON files", "*.json"), ("All files", "*.*")),
        )
        if not path:
            return
        base_id = self.session.base_id
        if base_id is None and "can_base_id" in values:
            base_id = int(values["can_base_id"])
        try:
            DriverConfigProfile(values=values, base_id=base_id).save_json(path)
        except Exception as exc:
            messagebox.showerror("Export Failed", str(exc), parent=self)
            return
        self.set_status(f"Exported {path}")

    def calibrate(self) -> None:
        try:
            known_voltage = float(self.known_voltage_var.get())
        except ValueError:
            messagebox.showerror("Invalid Voltage", "Known voltage must be a number.", parent=self)
            return
        self._run_worker("Calibrating driver...", lambda: self.session.calibrate(known_voltage), lambda _responses: self.set_status("Calibration complete."))

    def clear_fault(self) -> None:
        self._run_worker("Clearing fault...", self.session.clear_fault, lambda _response: self.set_status("Fault cleared."))

    def _run_worker(self, status: str, work: Callable[[], object], done: Callable[[object], None]) -> None:
        self.set_status(status)

        def target() -> None:
            try:
                result = work()
            except Exception as exc:
                self.after(0, lambda error=exc: self._show_worker_error(error))
                return
            self.after(0, lambda: done(result))

        threading.Thread(target=target, daemon=True).start()

    def _show_worker_error(self, exc: Exception) -> None:
        self.set_status(str(exc))
        messagebox.showerror("Driver Command Failed", str(exc), parent=self)

    def _show_operation_errors(self, title: str, result: ConfigOperationResult) -> None:
        lines = [f"{name}: {message}" for name, message in result.errors.items()]
        detail = "\n".join(lines[:10])
        if len(lines) > 10:
            detail += f"\n... {len(lines) - 10} more"
        if result.values:
            messagebox.showwarning(
                title,
                f"Some config keys failed, but {len(result.values)} value(s) were loaded or saved.\n\n{detail}",
                parent=self,
            )
        else:
            messagebox.showerror(title, f"No config values succeeded.\n\n{detail}", parent=self)
