from __future__ import annotations

import struct
import unittest

from fdrive_py.driver_host import (
    DRIVE_COMMAND_CONFIG_GET,
    RESPONSE_OFFSET,
    STATUS_OFFSET,
    CanTransport,
    DriverInterface,
    MODE_ACCEL,
    MODE_BRAKE,
    SerialFrame,
)


class FakeTransport(CanTransport):
    def __init__(self):
        self.sent_frames: list[tuple[int, bytes]] = []
        self.frames: list[SerialFrame] = []
        self.closed = False

    def close(self) -> None:
        self.closed = True

    def write_frame(self, arbitration_id: int, payload: bytes) -> None:
        self.sent_frames.append((arbitration_id, payload))

    def recv_frame(self, timeout: float = 0.0) -> SerialFrame | None:
        del timeout
        if not self.frames:
            return None
        return self.frames.pop(0)


class DriverInterfaceTests(unittest.TestCase):
    def test_send_signed_duty_brakes_on_negative_without_reverse(self) -> None:
        transport = FakeTransport()
        driver = DriverInterface(transport, 0x100)

        driver.send_signed_duty(-0.5, 1000, reverse_enabled=False)

        arbitration_id, payload = transport.sent_frames[-1]
        self.assertEqual(arbitration_id, 0x100)
        self.assertEqual(payload[0], MODE_BRAKE)
        self.assertAlmostEqual(struct.unpack_from("<f", payload, 3)[0], 0.5)

    def test_send_signed_duty_uses_accel_when_reverse_enabled(self) -> None:
        transport = FakeTransport()
        driver = DriverInterface(transport, 0x100)

        driver.send_signed_duty(-0.25, 1000, reverse_enabled=True)

        _, payload = transport.sent_frames[-1]
        self.assertEqual(payload[0], MODE_ACCEL)
        self.assertAlmostEqual(struct.unpack_from("<f", payload, 3)[0], -0.25)

    def test_process_status_frame_updates_latest_status(self) -> None:
        transport = FakeTransport()
        driver = DriverInterface(transport, 0x100)
        frame = SerialFrame(
            arbitration_id=0x100 + STATUS_OFFSET,
            data=bytes([MODE_ACCEL, 100, 0, 0, 0, 64, 63, 0xD7]),
            direction="RX",
        )

        driver.process_frame(frame)

        self.assertIsNotNone(driver.latest_status)
        assert driver.latest_status is not None
        self.assertEqual(driver.latest_status.mode, MODE_ACCEL)
        self.assertEqual(driver.latest_status.frequency_hz, 1000)
        self.assertAlmostEqual(driver.latest_status.duty_cycle, 0.75)
        self.assertTrue(driver.latest_status.has_signature)

    def test_wait_for_response_matches_requested_command(self) -> None:
        transport = FakeTransport()
        driver = DriverInterface(transport, 0x120)
        transport.frames.append(
            SerialFrame(
                arbitration_id=0x120 + RESPONSE_OFFSET,
                data=bytes([0x82, DRIVE_COMMAND_CONFIG_GET, 0x01, 0x34, 0x12, 0x00, 0x00]),
                direction="RX",
            )
        )

        response = driver.wait_for_response(DRIVE_COMMAND_CONFIG_GET, timeout=0.01)

        self.assertIsNotNone(response)
        assert response is not None
        self.assertEqual(response.command, DRIVE_COMMAND_CONFIG_GET)
        self.assertEqual(response.payload[0], 0x01)


if __name__ == "__main__":
    unittest.main()