from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

from .protocol import SerialFrame


class CanTransport(ABC):
    supports_mirroring = False

    @abstractmethod
    def close(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def write_frame(self, arbitration_id: int, payload: bytes) -> None:
        raise NotImplementedError

    @abstractmethod
    def recv_frame(self, timeout: float = 0.0) -> Optional[SerialFrame]:
        raise NotImplementedError

    def enable_mirroring(self, timeout: float = 5.0, retries: int = 8, retry_interval: float = 0.35) -> bool:
        del timeout, retries, retry_interval
        return False