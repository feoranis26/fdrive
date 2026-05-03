from __future__ import annotations

from .serial_backend import SerialCanTransport
from .socketcan_backend import SocketCanTransport
from .transport import CanTransport

AVAILABLE_BACKENDS = ("usb-can", "socketcan")


def create_transport(
    backend: str,
    *,
    port: str | None = None,
    baudrate: int = 115200,
    timeout: float = 0.05,
    startup_delay: float = 0.25,
    echo_serial: bool = False,
    channel: str = "can0",
    interface: str = "socketcan",
    receive_timeout: float = 0.05,
) -> CanTransport:
    if backend == "usb-can":
        if not port:
            raise ValueError("port is required for the usb-can backend")
        return SerialCanTransport(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            startup_delay=startup_delay,
            echo_serial=echo_serial,
        )
    if backend == "socketcan":
        return SocketCanTransport(
            channel=channel,
            interface=interface,
            receive_timeout=receive_timeout,
        )
    raise ValueError(f"unsupported backend: {backend}")