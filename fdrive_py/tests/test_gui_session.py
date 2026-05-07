from __future__ import annotations

import struct
import threading
import unittest

from fdrive_py.driver_host import (
    COMMAND_OFFSET,
    CONFIG_FIELD_BY_KEY,
    CONTROL_MODE_CURRENT,
    DRIVE_COMMAND_CONFIG_GET,
    DRIVE_COMMAND_CONFIG_SET,
    DRIVE_CONFIG_KEY_CONTROL_MODE,
    DRIVE_CONFIG_KEY_CURRENT_LIMIT_AMPS,
    DRIVE_CONFIG_KEY_PWM_ERROR_CLAMP,
    DRIVE_RESPONSE_ERROR,
    DRIVE_RESPONSE_OK,
    RESPONSE_OFFSET,
    CanTransport,
    SerialFrame,
)
from fdrive_py.gui.session import DriverConnectionConfig, DriverSession


class AutoResponseTransport(CanTransport):
    def __init__(self):
        self.sent_frames: list[tuple[int, bytes]] = []
        self.values: dict[int, int | float] = {
            DRIVE_CONFIG_KEY_CONTROL_MODE: 0,
            DRIVE_CONFIG_KEY_CURRENT_LIMIT_AMPS: 10.0,
        }
        self.set_keys: list[int] = []
        self.unsupported_keys: set[int] = set()
        self.closed = False
        self._frames: list[SerialFrame] = []
        self._condition = threading.Condition()

    def close(self) -> None:
        with self._condition:
            self.closed = True
            self._condition.notify_all()

    def write_frame(self, arbitration_id: int, payload: bytes) -> None:
        self.sent_frames.append((arbitration_id, bytes(payload)))
        if len(payload) < 2:
            return
        opcode = payload[0]
        if opcode == DRIVE_COMMAND_CONFIG_GET and len(payload) >= 2:
            self._queue_config_get_response(arbitration_id, payload[1])
        elif opcode == DRIVE_COMMAND_CONFIG_SET and len(payload) >= 6:
            self._queue_config_set_response(arbitration_id, payload[1], payload[2:6])

    def recv_frame(self, timeout: float = 0.0) -> SerialFrame | None:
        with self._condition:
            if not self._frames and timeout > 0.0 and not self.closed:
                self._condition.wait(timeout=timeout)
            if not self._frames:
                return None
            return self._frames.pop(0)

    def _queue_config_get_response(self, command_id: int, key: int) -> None:
        if key in self.unsupported_keys:
            self._queue_response(command_id, bytes([DRIVE_RESPONSE_ERROR, DRIVE_COMMAND_CONFIG_GET, 3]))
            return
        field = CONFIG_FIELD_BY_KEY[key]
        value = self.values.get(key, 0)
        if field.value_kind in ("can_id", "control_mode", "u32"):
            value_bytes = struct.pack("<I", int(value))
        else:
            value_bytes = struct.pack("<f", float(value))
        self._queue_response(command_id, bytes([DRIVE_RESPONSE_OK, DRIVE_COMMAND_CONFIG_GET, key]) + value_bytes)

    def _queue_config_set_response(self, command_id: int, key: int, value_bytes: bytes) -> None:
        if key in self.unsupported_keys:
            self._queue_response(command_id, bytes([DRIVE_RESPONSE_ERROR, DRIVE_COMMAND_CONFIG_SET, 3]))
            return
        field = CONFIG_FIELD_BY_KEY[key]
        if field.value_kind in ("can_id", "control_mode", "u32"):
            self.values[key] = struct.unpack("<I", value_bytes)[0]
        else:
            self.values[key] = struct.unpack("<f", value_bytes)[0]
        self.set_keys.append(key)
        self._queue_response(command_id, bytes([DRIVE_RESPONSE_OK, DRIVE_COMMAND_CONFIG_SET, key]) + bytes(value_bytes))

    def _queue_response(self, command_id: int, data: bytes) -> None:
        base_id = command_id - COMMAND_OFFSET
        with self._condition:
            self._frames.append(SerialFrame(arbitration_id=base_id + RESPONSE_OFFSET, data=data, direction="RX"))
            self._condition.notify_all()


class DriverSessionTests(unittest.TestCase):
    def make_session(self) -> tuple[DriverSession, AutoResponseTransport]:
        transport = AutoResponseTransport()
        session = DriverSession(transport_factory=lambda *_args, **_kwargs: transport)
        session.connect(DriverConnectionConfig(base_id=0x100, port="COM1"))
        return session, transport

    def test_config_get_uses_receive_thread_response(self) -> None:
        session, _transport = self.make_session()
        try:
            value = session.get_config_value("current_limit_amps", timeout=0.5)
        finally:
            session.disconnect(send_stop=False)

        self.assertAlmostEqual(float(value), 10.0)

    def test_config_set_updates_transport_value(self) -> None:
        session, transport = self.make_session()
        try:
            session.set_config_value("control_mode", "current", timeout=0.5)
        finally:
            session.disconnect(send_stop=False)

        self.assertEqual(transport.values[DRIVE_CONFIG_KEY_CONTROL_MODE], CONTROL_MODE_CURRENT)

    def test_write_all_config_writes_base_id_last_and_switches_session(self) -> None:
        session, transport = self.make_session()
        try:
            session.write_all_config({"can_base_id": "0x120", "current_limit_amps": 15.0}, timeout=0.5)
            base_id = session.base_id
        finally:
            session.disconnect(send_stop=False)

        self.assertEqual(transport.set_keys[-1], CONFIG_FIELD_BY_KEY[0x01].key)
        self.assertEqual(base_id, 0x120)

    def test_tolerant_config_read_collects_unsupported_key_errors(self) -> None:
        session, transport = self.make_session()
        transport.unsupported_keys.add(DRIVE_CONFIG_KEY_PWM_ERROR_CLAMP)
        try:
            result = session.read_all_config_tolerant(timeout=0.5)
        finally:
            session.disconnect(send_stop=False)

        self.assertIn("current_limit_amps", result.values)
        self.assertIn("pwm_error_clamp", result.errors)
        self.assertIn("code=3", result.errors["pwm_error_clamp"])

    def test_signed_duty_updates_target_snapshot(self) -> None:
        session, _transport = self.make_session()
        try:
            session.send_signed_duty(0.42, 1200, reverse_enabled=False)
            snapshot = session.get_snapshot()
            session.stop(1200)
            stopped_snapshot = session.get_snapshot()
        finally:
            session.disconnect(send_stop=False)

        self.assertAlmostEqual(snapshot.target_duty_cycle or 0.0, 0.42)
        self.assertEqual(snapshot.target_frequency_hz, 1200)
        self.assertEqual(stopped_snapshot.target_duty_cycle, 0.0)


if __name__ == "__main__":
    unittest.main()