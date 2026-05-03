from __future__ import annotations

import argparse

from fdrive_py.driver_host import AVAILABLE_BACKENDS, CanTransport, create_transport


def add_backend_arguments(parser: argparse.ArgumentParser, *, default_port: str, suppress_defaults: bool = False) -> None:
    backend_default = argparse.SUPPRESS if suppress_defaults else "usb-can"
    port_default = argparse.SUPPRESS if suppress_defaults else default_port
    baudrate_default = argparse.SUPPRESS if suppress_defaults else 115200
    serial_timeout_default = argparse.SUPPRESS if suppress_defaults else 0.05
    startup_delay_default = argparse.SUPPRESS if suppress_defaults else 0.25
    channel_default = argparse.SUPPRESS if suppress_defaults else "can0"
    interface_default = argparse.SUPPRESS if suppress_defaults else "socketcan"
    socketcan_timeout_default = argparse.SUPPRESS if suppress_defaults else 0.05

    parser.add_argument(
        "--backend",
        choices=AVAILABLE_BACKENDS,
        default=backend_default,
        help="CAN backend to use: USB serial bridge or direct SocketCAN.",
    )
    parser.add_argument("--port", default=port_default, help="Serial port for the USB-CAN UART bridge.")
    parser.add_argument("--baudrate", type=int, default=baudrate_default, help="Serial baud rate for the UART bridge.")
    parser.add_argument("--serial-timeout", type=float, default=serial_timeout_default, help="Serial read timeout in seconds.")
    parser.add_argument("--startup-delay", type=float, default=startup_delay_default, help="Delay after opening the serial port.")
    parser.add_argument("--echo-serial", action="store_true", help="Print non-protocol serial lines.")
    parser.add_argument("--channel", default=channel_default, help="SocketCAN channel, for example can0.")
    parser.add_argument("--interface", default=interface_default, help="python-can interface name for SocketCAN.")
    parser.add_argument(
        "--socketcan-timeout",
        type=float,
        default=socketcan_timeout_default,
        help="SocketCAN receive timeout in seconds for the background reader.",
    )


def create_transport_from_args(args: argparse.Namespace) -> CanTransport:
    return create_transport(
        backend=args.backend,
        port=getattr(args, "port", None),
        baudrate=getattr(args, "baudrate", 115200),
        timeout=getattr(args, "serial_timeout", 0.05),
        startup_delay=getattr(args, "startup_delay", 0.25),
        echo_serial=getattr(args, "echo_serial", False),
        channel=getattr(args, "channel", "can0"),
        interface=getattr(args, "interface", "socketcan"),
        receive_timeout=getattr(args, "socketcan_timeout", 0.05),
    )