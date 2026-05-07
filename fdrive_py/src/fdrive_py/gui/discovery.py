from __future__ import annotations

import struct
import time
from dataclasses import dataclass

from fdrive_py.driver_host import MODE_ACCEL, STATUS_OFFSET, create_transport, status_signature_matches

from .session import DriverConnectionConfig


@dataclass(frozen=True)
class ScanResult:
    base_id: int
    direction: str
    mode: int
    frequency_hz: int
    duty_cycle: float


def scan_for_drives(config: DriverConnectionConfig, duration: float = 3.0) -> tuple[ScanResult, ...]:
    transport = create_transport(
        config.backend,
        port=config.port,
        baudrate=config.baudrate,
        timeout=config.serial_timeout,
        startup_delay=config.startup_delay,
        echo_serial=config.echo_serial,
        channel=config.channel,
        interface=config.interface,
        receive_timeout=config.socketcan_timeout,
    )
    try:
        if transport.supports_mirroring:
            transport.enable_mirroring()

        discovered: dict[int, ScanResult] = {}
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            frame = transport.recv_frame(timeout=min(0.1, max(0.0, deadline - time.monotonic())))
            if frame is None or not status_signature_matches(frame.data):
                continue

            base_id = frame.arbitration_id - STATUS_OFFSET
            encoded_frequency = struct.unpack_from("<H", frame.data, 1)[0]
            duty_cycle = struct.unpack_from("<f", frame.data, 3)[0]
            discovered[base_id] = ScanResult(
                base_id=base_id,
                direction=frame.direction,
                mode=frame.data[0] if frame.data else MODE_ACCEL,
                frequency_hz=encoded_frequency * 10,
                duty_cycle=duty_cycle,
            )

        return tuple(discovered[base_id] for base_id in sorted(discovered))
    finally:
        transport.close()
