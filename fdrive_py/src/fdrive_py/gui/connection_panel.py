from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import Callable

from fdrive_py.driver_host import AVAILABLE_BACKENDS, parse_can_id

from .session import DriverConnectionConfig


class ConnectionPanel(ttk.LabelFrame):
    def __init__(
        self,
        master: tk.Misc,
        *,
        on_connect: Callable[[], None],
        on_disconnect: Callable[[], None],
        on_scan: Callable[[], None],
    ):
        super().__init__(master, text="Connection")
        self._on_connect = on_connect
        self._on_disconnect = on_disconnect
        self._on_scan = on_scan

        self.backend_var = tk.StringVar(value="usb-can")
        self.base_id_var = tk.StringVar(value="0x100")
        self.port_var = tk.StringVar(value="COM3")
        self.baudrate_var = tk.StringVar(value="115200")
        self.serial_timeout_var = tk.StringVar(value="0.05")
        self.startup_delay_var = tk.StringVar(value="0.25")
        self.echo_serial_var = tk.BooleanVar(value=False)
        self.channel_var = tk.StringVar(value="can0")
        self.interface_var = tk.StringVar(value="socketcan")
        self.socketcan_timeout_var = tk.StringVar(value="0.05")

        self._add_labeled(0, 0, "Backend", ttk.Combobox(self, textvariable=self.backend_var, values=AVAILABLE_BACKENDS, width=10, state="readonly"))
        self._add_labeled(0, 2, "Base ID", ttk.Entry(self, textvariable=self.base_id_var, width=10))
        self._add_labeled(0, 4, "Port", ttk.Entry(self, textvariable=self.port_var, width=10))
        self._add_labeled(0, 6, "Baud", ttk.Entry(self, textvariable=self.baudrate_var, width=9))

        self._add_labeled(1, 0, "Serial Timeout", ttk.Entry(self, textvariable=self.serial_timeout_var, width=8))
        self._add_labeled(1, 2, "Startup", ttk.Entry(self, textvariable=self.startup_delay_var, width=8))
        ttk.Checkbutton(self, text="Echo Serial", variable=self.echo_serial_var).grid(row=1, column=4, columnspan=2, sticky="w", padx=4, pady=3)

        self._add_labeled(2, 0, "Channel", ttk.Entry(self, textvariable=self.channel_var, width=10))
        self._add_labeled(2, 2, "Interface", ttk.Entry(self, textvariable=self.interface_var, width=10))
        self._add_labeled(2, 4, "CAN Timeout", ttk.Entry(self, textvariable=self.socketcan_timeout_var, width=8))

        self.scan_button = ttk.Button(self, text="Scan", command=self._on_scan)
        self.scan_button.grid(row=0, column=8, sticky="ew", padx=4, pady=3)
        self.connect_button = ttk.Button(self, text="Connect", command=self._on_connect)
        self.connect_button.grid(row=1, column=8, sticky="ew", padx=4, pady=3)
        self.disconnect_button = ttk.Button(self, text="Disconnect", command=self._on_disconnect, state="disabled")
        self.disconnect_button.grid(row=2, column=8, sticky="ew", padx=4, pady=3)

        for column in range(9):
            self.columnconfigure(column, weight=1 if column in (1, 3, 5, 7) else 0)

    def get_config(self) -> DriverConnectionConfig:
        return DriverConnectionConfig(
            backend=self.backend_var.get(),
            base_id=parse_can_id(self.base_id_var.get()),
            port=self.port_var.get().strip(),
            baudrate=int(self.baudrate_var.get()),
            serial_timeout=float(self.serial_timeout_var.get()),
            startup_delay=float(self.startup_delay_var.get()),
            echo_serial=bool(self.echo_serial_var.get()),
            channel=self.channel_var.get().strip(),
            interface=self.interface_var.get().strip(),
            socketcan_timeout=float(self.socketcan_timeout_var.get()),
        )

    def set_base_id(self, base_id: int) -> None:
        self.base_id_var.set(f"0x{base_id:X}")

    def set_connected(self, connected: bool) -> None:
        self.connect_button.configure(state="disabled" if connected else "normal")
        self.disconnect_button.configure(state="normal" if connected else "disabled")

    def _add_labeled(self, row: int, column: int, label: str, widget: tk.Widget) -> None:
        ttk.Label(self, text=label).grid(row=row, column=column, sticky="w", padx=(6, 2), pady=3)
        widget.grid(row=row, column=column + 1, sticky="ew", padx=(2, 6), pady=3)
