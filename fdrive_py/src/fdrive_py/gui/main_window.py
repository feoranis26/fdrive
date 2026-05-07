from __future__ import annotations

import threading
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Callable

from .config_panel import ConfigPanel
from .connection_panel import ConnectionPanel
from .discovery import scan_for_drives
from .joystick_panel import JoystickPanel
from .session import DriverSession
from .status_widget import DriveStatusWidget


class FDriveGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("fdrive Driver GUI")
        self.minsize(900, 620)
        self.session = DriverSession()
        self.status_var = tk.StringVar(value="Disconnected.")

        self.connection_panel = ConnectionPanel(
            self,
            on_connect=self.connect_driver,
            on_disconnect=self.disconnect_driver,
            on_scan=self.scan_drivers,
        )
        self.connection_panel.grid(row=0, column=0, columnspan=2, sticky="ew", padx=8, pady=(8, 4))

        notebook = ttk.Notebook(self)
        notebook.grid(row=1, column=0, sticky="nsew", padx=(8, 4), pady=4)

        self.status_widget = DriveStatusWidget(self)
        self.status_widget.grid(row=1, column=1, sticky="nsew", padx=(4, 8), pady=4)

        self.config_panel = ConfigPanel(
            notebook,
            session=self.session,
            set_status=self.set_status,
            on_base_id_changed=self.connection_panel.set_base_id,
            on_config_values=self.status_widget.set_config_values,
        )
        notebook.add(self.config_panel, text="Configuration")

        self.joystick_panel = JoystickPanel(notebook, session=self.session, set_status=self.set_status)
        notebook.add(self.joystick_panel, text="Joystick / Test")

        ttk.Label(self, textvariable=self.status_var, relief="sunken", anchor="w").grid(
            row=2, column=0, columnspan=2, sticky="ew", padx=8, pady=(4, 8)
        )

        self.columnconfigure(0, weight=3)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(1, weight=1)
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(100, self.refresh_status)

    def set_status(self, message: str) -> None:
        self.status_var.set(message)

    def connect_driver(self) -> None:
        try:
            config = self.connection_panel.get_config()
        except ValueError as exc:
            messagebox.showerror("Invalid Connection", str(exc), parent=self)
            return

        def done(_result: object) -> None:
            self.connection_panel.set_connected(True)
            self.set_status(f"Connected to base ID 0x{config.base_id:X}.")

        self._run_worker("Connecting...", lambda: self.session.connect(config), done)

    def disconnect_driver(self) -> None:
        self.joystick_panel.shutdown()
        self.session.disconnect()
        self.connection_panel.set_connected(False)
        self.set_status("Disconnected.")

    def scan_drivers(self) -> None:
        try:
            config = self.connection_panel.get_config()
        except ValueError as exc:
            messagebox.showerror("Invalid Connection", str(exc), parent=self)
            return

        def done(results: object) -> None:
            scan_results = tuple(results)
            if not scan_results:
                self.set_status("No drivers discovered.")
                messagebox.showinfo("Scan Complete", "No drivers discovered.", parent=self)
                return
            first = scan_results[0]
            self.connection_panel.set_base_id(first.base_id)
            summary = ", ".join(f"0x{result.base_id:X}" for result in scan_results)
            self.set_status(f"Discovered driver base IDs: {summary}")

        self._run_worker("Scanning for drivers...", lambda: scan_for_drives(config), done)

    def refresh_status(self) -> None:
        self.status_widget.set_snapshot(self.session.get_snapshot())
        self.after(100, self.refresh_status)

    def on_close(self) -> None:
        self.joystick_panel.shutdown()
        self.session.disconnect()
        self.destroy()

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
        messagebox.showerror("Operation Failed", str(exc), parent=self)


def main() -> int:
    app = FDriveGui()
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
