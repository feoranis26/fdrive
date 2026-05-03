from __future__ import annotations

import queue
import threading
from typing import Optional

from .protocol import SerialFrame
from .transport import CanTransport

try:
    import can
except ImportError:  # pragma: no cover - handled at runtime when backend is selected.
    can = None


class SocketCanTransport(CanTransport):
    def __init__(self, channel: str, interface: str = "socketcan", receive_timeout: float = 0.05):
        if can is None:
            raise RuntimeError("python-can is required for the socketcan backend")

        self._bus = can.Bus(interface=interface, channel=channel)
        self._receive_timeout = receive_timeout
        self._rx_queue: queue.Queue[SerialFrame] = queue.Queue()
        self._stop_event = threading.Event()
        self._reader = threading.Thread(target=self._reader_loop, name="socketcan-reader", daemon=True)
        self._reader.start()

    def close(self) -> None:
        self._stop_event.set()
        self._reader.join(timeout=1.0)
        shutdown = getattr(self._bus, "shutdown", None)
        if callable(shutdown):
            shutdown()

    def write_frame(self, arbitration_id: int, payload: bytes) -> None:
        assert can is not None
        message = can.Message(
            arbitration_id=arbitration_id,
            is_extended_id=arbitration_id > 0x7FF,
            data=payload,
        )
        try:
            self._bus.send(message)
        except can.CanError as exc:
            raise OSError(str(exc)) from exc

    def recv_frame(self, timeout: float = 0.0) -> Optional[SerialFrame]:
        try:
            return self._rx_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def _reader_loop(self) -> None:
        assert can is not None
        while not self._stop_event.is_set():
            try:
                message = self._bus.recv(timeout=self._receive_timeout)
            except can.CanError:
                return
            if message is None or message.is_error_frame:
                continue
            self._rx_queue.put(
                SerialFrame(
                    arbitration_id=message.arbitration_id,
                    data=bytes(message.data),
                    direction="RX",
                )
            )