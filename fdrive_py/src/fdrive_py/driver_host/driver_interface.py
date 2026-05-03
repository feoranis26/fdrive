from __future__ import annotations

import struct
import time
from typing import Optional

from .protocol import (
    COMMAND_OFFSET,
    CONTROL_OFFSET,
    DRIVE_COMMAND_CALIBRATE,
    DRIVE_COMMAND_CLEAR_FAULT,
    DRIVE_COMMAND_CONFIG_GET,
    DRIVE_COMMAND_CONFIG_SET,
    DRIVE_CONFIG_KEY_CAN_BASE_ID,
    DRIVE_CONFIG_KEY_CONTROL_MODE,
    MEASUREMENT_OFFSET,
    MODE_ACCEL,
    MODE_BRAKE,
    MODE_OFFSET,
    RESPONSE_OFFSET,
    STATUS_OFFSET,
    DriveControllerMode,
    DriveMeasurements,
    DriveResponse,
    DriveStatus,
    SerialFrame,
    build_id,
    decode_frequency_field,
    encode_frequency_field,
    status_signature_matches,
)
from .transport import CanTransport


class DriverInterface:
    def __init__(self, bridge: CanTransport, base_id: int):
        self.bridge = bridge
        self.base_id = base_id
        self.control_id = build_id(base_id, CONTROL_OFFSET)
        self.status_id = build_id(base_id, STATUS_OFFSET)
        self.mode_id = build_id(base_id, MODE_OFFSET)
        self.measurement_id = build_id(base_id, MEASUREMENT_OFFSET)
        self.command_id = build_id(base_id, COMMAND_OFFSET)
        self.response_id = build_id(base_id, RESPONSE_OFFSET)

        self.latest_status: Optional[DriveStatus] = None
        self.latest_mode: Optional[DriveControllerMode] = None
        self.latest_measurements: Optional[DriveMeasurements] = None
        self.latest_response: Optional[DriveResponse] = None

    def send_signed_duty(self, duty: float, frequency_hz: int, reverse_enabled: bool) -> None:
        duty = max(-1.0, min(1.0, duty))
        if duty >= 0.0:
            mode = MODE_ACCEL
            command_duty = duty
        elif reverse_enabled:
            mode = MODE_ACCEL
            command_duty = duty
        else:
            mode = MODE_BRAKE
            command_duty = abs(duty)

        payload = bytearray(7)
        payload[0] = mode
        struct.pack_into("<H", payload, 1, encode_frequency_field(frequency_hz))
        struct.pack_into("<f", payload, 3, float(command_duty))
        self.bridge.write_frame(self.control_id, payload)

    def stop(self, frequency_hz: int) -> None:
        payload = bytearray(7)
        payload[0] = MODE_BRAKE
        struct.pack_into("<H", payload, 1, encode_frequency_field(frequency_hz))
        struct.pack_into("<f", payload, 3, 0.0)
        self.bridge.write_frame(self.control_id, payload)

    def send_config_get(self, key: int) -> None:
        self.bridge.write_frame(self.command_id, bytes([DRIVE_COMMAND_CONFIG_GET, key]))

    def send_config_set_u32(self, key: int, value: int) -> None:
        payload = bytearray([DRIVE_COMMAND_CONFIG_SET, key])
        payload.extend(struct.pack("<I", value & 0xFFFFFFFF))
        self.bridge.write_frame(self.command_id, payload)

    def send_config_set_float(self, key: int, value: float) -> None:
        payload = bytearray([DRIVE_COMMAND_CONFIG_SET, key])
        payload.extend(struct.pack("<f", value))
        self.bridge.write_frame(self.command_id, payload)

    def send_calibrate(self, known_voltage: float) -> None:
        payload = bytearray([DRIVE_COMMAND_CALIBRATE])
        payload.extend(struct.pack("<f", known_voltage))
        self.bridge.write_frame(self.command_id, payload)

    def send_clear_fault(self) -> None:
        self.bridge.write_frame(self.command_id, bytes([DRIVE_COMMAND_CLEAR_FAULT]))

    def process_frame(self, message: SerialFrame) -> None:
        if message.arbitration_id == self.status_id and len(message.data) >= 7:
            encoded_frequency = struct.unpack_from("<H", message.data, 1)[0]
            duty_cycle = struct.unpack_from("<f", message.data, 3)[0]
            self.latest_status = DriveStatus(
                arbitration_id=message.arbitration_id,
                mode=message.data[0],
                frequency_hz=decode_frequency_field(encoded_frequency),
                duty_cycle=duty_cycle,
                has_signature=status_signature_matches(message.data),
            )
        elif message.arbitration_id == self.mode_id and len(message.data) >= 1:
            self.latest_mode = DriveControllerMode(arbitration_id=message.arbitration_id, mode=message.data[0])
        elif message.arbitration_id == self.measurement_id and len(message.data) >= 8:
            bus_voltage, current = struct.unpack_from("<ff", message.data, 0)
            self.latest_measurements = DriveMeasurements(
                arbitration_id=message.arbitration_id,
                bus_voltage_volts=bus_voltage,
                current_amps=current,
            )
        elif message.arbitration_id == self.response_id and len(message.data) >= 2:
            self.latest_response = DriveResponse(
                arbitration_id=message.arbitration_id,
                response_type=message.data[0],
                command=message.data[1],
                payload=bytes(message.data[2:]),
            )

    def drain_frames(self, timeout: float = 0.0) -> None:
        while True:
            message = self.bridge.recv_frame(timeout=timeout)
            if message is None:
                return
            self.process_frame(message)
            timeout = 0.0

    def wait_for_response(self, command: int, timeout: float) -> Optional[DriveResponse]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            message = self.bridge.recv_frame(timeout=min(0.05, max(0.0, deadline - time.monotonic())))
            if message is None:
                continue
            self.process_frame(message)
            if (
                message.arbitration_id == self.response_id
                and self.latest_response is not None
                and self.latest_response.command == command
            ):
                return self.latest_response
        return None

    def send_config_set(self, key: int, value: int | float) -> None:
        if key in (DRIVE_CONFIG_KEY_CAN_BASE_ID, DRIVE_CONFIG_KEY_CONTROL_MODE):
            self.send_config_set_u32(key, int(value))
        else:
            self.send_config_set_float(key, float(value))