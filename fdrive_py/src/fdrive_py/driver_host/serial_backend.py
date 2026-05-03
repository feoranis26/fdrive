from __future__ import annotations

import queue
import threading
import time
from typing import Optional

import serial

from .protocol import SerialControlMessage, SerialFrame
from .transport import CanTransport


class SerialCanTransport(CanTransport):
    supports_mirroring = True

    def __init__(self, port: str, baudrate: int, timeout: float, startup_delay: float, echo_serial: bool):
        self.serial_port = serial.Serial()
        self.serial_port.port = port
        self.serial_port.baudrate = baudrate
        self.serial_port.timeout = timeout
        self.serial_port.write_timeout = timeout
        self.serial_port.rtscts = False
        self.serial_port.dsrdtr = False
        self.serial_port.xonxoff = False
        self.serial_port.dtr = False
        self.serial_port.rts = False
        self.serial_port.open()
        self.serial_port.setDTR(False)
        self.serial_port.setRTS(False)

        self._echo_serial = echo_serial
        self._rx_queue: queue.Queue[SerialFrame] = queue.Queue()
        self._control_queue: queue.Queue[SerialControlMessage] = queue.Queue()
        self._stop_event = threading.Event()
        self._last_activity = time.monotonic()
        self._reader = threading.Thread(target=self._reader_loop, name="serial-bridge-reader", daemon=True)
        self._reader.start()
        if startup_delay > 0.0:
            time.sleep(startup_delay)
        self._drain_rx_messages()
        self._drain_control_messages()

    def close(self) -> None:
        self._stop_event.set()
        if self.serial_port.is_open:
            self.serial_port.close()
        self._reader.join(timeout=1.0)

    def enable_mirroring(self, timeout: float = 5.0, retries: int = 8, retry_interval: float = 0.35) -> bool:
        self._drain_control_messages()
        self._drain_rx_messages()

        deadline = time.monotonic() + max(timeout, 0.0)
        attempts = 0
        while attempts < max(retries, 1) and time.monotonic() < deadline:
            self.wait_for_quiet(timeout=min(0.75, max(0.0, deadline - time.monotonic())), quiet_period=0.15)
            attempts += 1
            self.write_line("SEND ENABLE")

            attempt_deadline = min(deadline, time.monotonic() + retry_interval)
            while time.monotonic() < attempt_deadline:
                message = self.recv_control(timeout=min(0.05, max(0.0, attempt_deadline - time.monotonic())))
                if message is None:
                    continue
                if message.text == "SEND ENABLED":
                    return True

        return False

    def write_frame(self, arbitration_id: int, payload: bytes) -> None:
        payload_hex = " ".join(f"{byte:02X}" for byte in payload)
        line = f"SEND ID:{arbitration_id:X}"
        if payload_hex:
            line = f"{line} {payload_hex}"
        self.write_line(line)

    def write_line(self, line: str) -> None:
        self.serial_port.write((line + "\n").encode("ascii"))
        self.serial_port.flush()

    def recv_frame(self, timeout: float = 0.0) -> Optional[SerialFrame]:
        try:
            return self._rx_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def recv_control(self, timeout: float = 0.0) -> Optional[SerialControlMessage]:
        try:
            return self._control_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def wait_for_quiet(self, timeout: float, quiet_period: float) -> bool:
        deadline = time.monotonic() + max(timeout, 0.0)
        while time.monotonic() < deadline:
            if (time.monotonic() - self._last_activity) >= quiet_period:
                return True
            time.sleep(0.02)
        return (time.monotonic() - self._last_activity) >= quiet_period

    def _drain_control_messages(self) -> None:
        while True:
            try:
                self._control_queue.get_nowait()
            except queue.Empty:
                return

    def _drain_rx_messages(self) -> None:
        while True:
            try:
                self._rx_queue.get_nowait()
            except queue.Empty:
                return

    def _reader_loop(self) -> None:
        pending = bytearray()

        while not self._stop_event.is_set():
            try:
                waiting = self.serial_port.in_waiting
                if waiting > 0:
                    raw_chunk = self.serial_port.read(waiting)
                else:
                    raw_chunk = self.serial_port.read(1)
            except serial.SerialException:
                return

            if not raw_chunk:
                continue

            self._last_activity = time.monotonic()
            pending.extend(raw_chunk)

            while True:
                newline_index = pending.find(b"\n")
                if newline_index < 0:
                    break

                raw_line = bytes(pending[:newline_index])
                del pending[: newline_index + 1]

                line = raw_line.decode("ascii", errors="ignore").rstrip("\r")
                if not line:
                    continue

                if line in {"SEND ENABLED", "SEND DISABLED", "NAK"}:
                    self._control_queue.put(SerialControlMessage(text=line))
                    continue

                frame = self._parse_frame_line(line)
                if frame is not None:
                    self._rx_queue.put(frame)
                    continue

                if self._echo_serial:
                    print(f"Serial: {line}")

    @staticmethod
    def _parse_frame_line(line: str) -> Optional[SerialFrame]:
        if not line.startswith("SEND "):
            return None

        parts = line.split()
        if len(parts) < 3:
            return None

        direction = parts[1]
        if direction not in {"RX", "TX"}:
            return None

        if not parts[2].startswith("ID:"):
            return None

        try:
            arbitration_id = int(parts[2][3:], 16)
            data = bytes(int(token, 16) for token in parts[3:])
        except ValueError:
            return None

        return SerialFrame(arbitration_id=arbitration_id, data=data, direction=direction)


SerialBridge = SerialCanTransport