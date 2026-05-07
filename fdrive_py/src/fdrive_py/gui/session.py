from __future__ import annotations

import struct
import threading
import time
from dataclasses import dataclass
from typing import Callable

from fdrive_py.driver_host import (
    CONFIG_FIELDS,
    DRIVE_COMMAND_CALIBRATE,
    DRIVE_COMMAND_CLEAR_FAULT,
    DRIVE_COMMAND_CONFIG_GET,
    DRIVE_COMMAND_CONFIG_SET,
    DRIVE_RESPONSE_ACK,
    DRIVE_RESPONSE_ERROR,
    DRIVE_RESPONSE_IN_PROGRESS,
    DRIVE_RESPONSE_OK,
    ConfigField,
    ConfigValue,
    DriveControllerMode,
    DriveMeasurements,
    DriveResponse,
    DriveStatus,
    DriverInterface,
    config_field,
    create_transport,
    iter_config_fields_for_write,
    normalize_config_values,
)
from fdrive_py.driver_host.transport import CanTransport


@dataclass(frozen=True)
class DriverConnectionConfig:
    backend: str = "usb-can"
    base_id: int = 0x100
    port: str = "COM3"
    baudrate: int = 115200
    serial_timeout: float = 0.05
    startup_delay: float = 0.25
    echo_serial: bool = False
    channel: str = "can0"
    interface: str = "socketcan"
    socketcan_timeout: float = 0.05


@dataclass(frozen=True)
class DriveSnapshot:
    connected: bool = False
    base_id: int | None = None
    status: DriveStatus | None = None
    controller_mode: DriveControllerMode | None = None
    measurements: DriveMeasurements | None = None
    response: DriveResponse | None = None
    last_frame_time: float | None = None
    mirroring_enabled: bool | None = None
    last_error: str | None = None
    target_duty_cycle: float | None = None
    target_frequency_hz: int | None = None


@dataclass(frozen=True)
class ConfigOperationResult:
    values: dict[str, ConfigValue]
    errors: dict[str, str]


class DriverSessionError(RuntimeError):
    pass


class DriverCommandTimeout(DriverSessionError):
    pass


class DriverCommandError(DriverSessionError):
    def __init__(self, message: str, response: DriveResponse):
        super().__init__(message)
        self.response = response


class DriverSession:
    def __init__(self, transport_factory: Callable[..., CanTransport] = create_transport):
        self._transport_factory = transport_factory
        self._transport: CanTransport | None = None
        self._driver: DriverInterface | None = None
        self._thread: threading.Thread | None = None
        self._running = threading.Event()
        self._state_lock = threading.RLock()
        self._write_lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._response_condition = threading.Condition()
        self._responses: list[DriveResponse] = []
        self._snapshot = DriveSnapshot()
        self._last_frequency_hz = 1000

    @property
    def connected(self) -> bool:
        with self._state_lock:
            return self._transport is not None and self._driver is not None

    @property
    def base_id(self) -> int | None:
        with self._state_lock:
            return None if self._driver is None else self._driver.base_id

    def connect(self, config: DriverConnectionConfig) -> None:
        self.disconnect(send_stop=False)
        transport = self._transport_factory(
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
            mirroring_enabled: bool | None = None
            if transport.supports_mirroring:
                mirroring_enabled = transport.enable_mirroring()
        except Exception:
            transport.close()
            raise

        driver = DriverInterface(transport, config.base_id)
        with self._state_lock:
            self._transport = transport
            self._driver = driver
            self._snapshot = DriveSnapshot(connected=True, base_id=config.base_id, mirroring_enabled=mirroring_enabled)
        with self._response_condition:
            self._responses.clear()

        self._running.set()
        self._thread = threading.Thread(target=self._receive_loop, name="fdrive-gui-receiver", daemon=True)
        self._thread.start()

    def disconnect(self, *, send_stop: bool = True) -> None:
        with self._state_lock:
            transport = self._transport
            driver = self._driver
            thread = self._thread

        if send_stop and driver is not None:
            try:
                with self._write_lock:
                    driver.stop(self._last_frequency_hz)
            except Exception:
                pass

        self._running.clear()
        if transport is not None:
            try:
                transport.close()
            except Exception:
                pass
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)

        with self._state_lock:
            last_error = self._snapshot.last_error
            self._transport = None
            self._driver = None
            self._thread = None
            self._snapshot = DriveSnapshot(last_error=last_error)
        with self._response_condition:
            self._response_condition.notify_all()

    def set_base_id(self, base_id: int) -> None:
        with self._state_lock:
            if self._transport is None:
                raise DriverSessionError("driver is not connected")
            mirroring_enabled = self._snapshot.mirroring_enabled
            self._driver = DriverInterface(self._transport, base_id)
            self._snapshot = DriveSnapshot(connected=True, base_id=base_id, mirroring_enabled=mirroring_enabled)
        with self._response_condition:
            self._responses.clear()

    def get_snapshot(self) -> DriveSnapshot:
        with self._state_lock:
            return self._snapshot

    def get_config_value(self, field: ConfigField | str | int, timeout: float = 1.5) -> ConfigValue:
        field_info = config_field(field)
        driver = self._require_driver()
        response = self._send_and_wait(
            DRIVE_COMMAND_CONFIG_GET,
            lambda: driver.send_config_get(field_info.key),
            timeout,
        )
        self._raise_for_error(response, f"Config get failed for {field_info.name}")
        if response.response_type != DRIVE_RESPONSE_OK:
            raise DriverSessionError(f"unexpected config get response: 0x{response.response_type:02X}")
        return self._decode_config_payload(field_info, response)

    def read_all_config(self, timeout: float = 1.5) -> dict[str, ConfigValue]:
        values: dict[str, ConfigValue] = {}
        for field_info in CONFIG_FIELDS:
            values[field_info.name] = self.get_config_value(field_info, timeout=timeout)
        return values

    def read_all_config_tolerant(self, timeout: float = 1.5) -> ConfigOperationResult:
        values: dict[str, ConfigValue] = {}
        errors: dict[str, str] = {}
        for field_info in CONFIG_FIELDS:
            try:
                values[field_info.name] = self.get_config_value(field_info, timeout=timeout)
            except Exception as exc:
                errors[field_info.name] = str(exc)
        return ConfigOperationResult(values=values, errors=errors)

    def set_config_value(self, field: ConfigField | str | int, value: object, timeout: float = 1.5) -> DriveResponse:
        field_info = config_field(field)
        parsed_value = field_info.parse(value)
        driver = self._require_driver()
        response = self._send_and_wait(
            DRIVE_COMMAND_CONFIG_SET,
            lambda: driver.send_config_set(field_info.key, parsed_value),
            timeout,
        )
        self._raise_for_error(response, f"Config set failed for {field_info.name}")
        if response.response_type != DRIVE_RESPONSE_OK:
            raise DriverSessionError(f"unexpected config set response: 0x{response.response_type:02X}")
        return response

    def write_all_config(self, values: dict[str, object], timeout: float = 1.5) -> dict[str, DriveResponse]:
        normalized = normalize_config_values(values)
        responses: dict[str, DriveResponse] = {}
        for field_info in iter_config_fields_for_write(normalized):
            response = self.set_config_value(field_info, normalized[field_info.name], timeout=timeout)
            responses[field_info.name] = response
            if field_info.name == "can_base_id":
                self.set_base_id(int(normalized[field_info.name]))
        return responses

    def write_all_config_tolerant(self, values: dict[str, object], timeout: float = 1.5) -> ConfigOperationResult:
        normalized = normalize_config_values(values)
        written: dict[str, ConfigValue] = {}
        errors: dict[str, str] = {}
        for field_info in iter_config_fields_for_write(normalized):
            try:
                self.set_config_value(field_info, normalized[field_info.name], timeout=timeout)
                written[field_info.name] = normalized[field_info.name]
                if field_info.name == "can_base_id":
                    self.set_base_id(int(normalized[field_info.name]))
            except Exception as exc:
                errors[field_info.name] = str(exc)
        return ConfigOperationResult(values=written, errors=errors)

    def calibrate(self, known_voltage: float, timeout: float = 10.0) -> tuple[DriveResponse, ...]:
        driver = self._require_driver()
        responses: list[DriveResponse] = []
        with self._command_lock:
            start_index = self._response_count()
            with self._write_lock:
                driver.send_calibrate(float(known_voltage))

            index = start_index
            deadline = time.monotonic() + timeout
            saw_ack = False
            while time.monotonic() < deadline:
                result = self._wait_for_response(DRIVE_COMMAND_CALIBRATE, index, max(0.0, deadline - time.monotonic()))
                if result is None:
                    break
                index, response = result
                responses.append(response)
                if response.response_type == DRIVE_RESPONSE_ACK:
                    saw_ack = True
                    continue
                if response.response_type == DRIVE_RESPONSE_IN_PROGRESS:
                    continue
                self._raise_for_error(response, "Calibration failed")
                if response.response_type == DRIVE_RESPONSE_OK:
                    return tuple(responses)

        if saw_ack:
            raise DriverCommandTimeout("calibration did not complete before timeout")
        raise DriverCommandTimeout("calibration acknowledgment was not received")

    def clear_fault(self, timeout: float = 1.5) -> DriveResponse:
        driver = self._require_driver()
        response = self._send_and_wait(DRIVE_COMMAND_CLEAR_FAULT, driver.send_clear_fault, timeout)
        self._raise_for_error(response, "Clear fault failed")
        if response.response_type != DRIVE_RESPONSE_OK:
            raise DriverSessionError(f"unexpected clear-fault response: 0x{response.response_type:02X}")
        return response

    def send_signed_duty(self, duty: float, frequency_hz: int, *, reverse_enabled: bool) -> None:
        driver = self._require_driver()
        self._last_frequency_hz = int(frequency_hz)
        with self._write_lock:
            driver.send_signed_duty(float(duty), int(frequency_hz), reverse_enabled=reverse_enabled)
        self._set_target(float(duty), int(frequency_hz))

    def stop(self, frequency_hz: int | None = None) -> None:
        driver = self._require_driver()
        if frequency_hz is not None:
            self._last_frequency_hz = int(frequency_hz)
        with self._write_lock:
            driver.stop(self._last_frequency_hz)
        self._set_target(0.0, self._last_frequency_hz)

    def _receive_loop(self) -> None:
        while self._running.is_set():
            with self._state_lock:
                transport = self._transport
                driver = self._driver
            if transport is None or driver is None:
                break

            try:
                message = transport.recv_frame(timeout=0.05)
            except Exception as exc:
                self._set_error(str(exc))
                break
            if message is None:
                continue

            try:
                driver.process_frame(message)
            except Exception as exc:
                self._set_error(str(exc))
                continue

            now = time.monotonic()
            response = driver.latest_response if message.arbitration_id == driver.response_id else None
            with self._state_lock:
                self._snapshot = DriveSnapshot(
                    connected=True,
                    base_id=driver.base_id,
                    status=driver.latest_status,
                    controller_mode=driver.latest_mode,
                    measurements=driver.latest_measurements,
                    response=driver.latest_response,
                    last_frame_time=now,
                    mirroring_enabled=self._snapshot.mirroring_enabled,
                    last_error=None,
                    target_duty_cycle=self._snapshot.target_duty_cycle,
                    target_frequency_hz=self._snapshot.target_frequency_hz,
                )
            if response is not None:
                with self._response_condition:
                    self._responses.append(response)
                    self._response_condition.notify_all()

    def _set_error(self, message: str) -> None:
        with self._state_lock:
            self._snapshot = DriveSnapshot(
                connected=self._transport is not None and self._driver is not None,
                base_id=None if self._driver is None else self._driver.base_id,
                status=None if self._driver is None else self._driver.latest_status,
                controller_mode=None if self._driver is None else self._driver.latest_mode,
                measurements=None if self._driver is None else self._driver.latest_measurements,
                response=None if self._driver is None else self._driver.latest_response,
                last_frame_time=self._snapshot.last_frame_time,
                mirroring_enabled=self._snapshot.mirroring_enabled,
                last_error=message,
                target_duty_cycle=self._snapshot.target_duty_cycle,
                target_frequency_hz=self._snapshot.target_frequency_hz,
            )
        with self._response_condition:
            self._response_condition.notify_all()

    def _set_target(self, duty_cycle: float, frequency_hz: int) -> None:
        target_duty = max(-1.0, min(1.0, duty_cycle))
        with self._state_lock:
            self._snapshot = DriveSnapshot(
                connected=self._snapshot.connected,
                base_id=self._snapshot.base_id,
                status=self._snapshot.status,
                controller_mode=self._snapshot.controller_mode,
                measurements=self._snapshot.measurements,
                response=self._snapshot.response,
                last_frame_time=self._snapshot.last_frame_time,
                mirroring_enabled=self._snapshot.mirroring_enabled,
                last_error=self._snapshot.last_error,
                target_duty_cycle=target_duty,
                target_frequency_hz=frequency_hz,
            )

    def _require_driver(self) -> DriverInterface:
        with self._state_lock:
            if self._driver is None:
                raise DriverSessionError("driver is not connected")
            return self._driver

    def _response_count(self) -> int:
        with self._response_condition:
            return len(self._responses)

    def _send_and_wait(self, command: int, send: Callable[[], None], timeout: float) -> DriveResponse:
        with self._command_lock:
            start_index = self._response_count()
            with self._write_lock:
                send()
            result = self._wait_for_response(command, start_index, timeout)
            if result is None:
                raise DriverCommandTimeout(f"timed out waiting for command 0x{command:02X} response")
            _, response = result
            return response

    def _wait_for_response(self, command: int, start_index: int, timeout: float) -> tuple[int, DriveResponse] | None:
        deadline = time.monotonic() + timeout
        with self._response_condition:
            index = start_index
            while True:
                while index < len(self._responses):
                    response = self._responses[index]
                    index += 1
                    if response.command == command:
                        return index, response

                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._response_condition.wait(timeout=remaining)

    def _decode_config_payload(self, field_info: ConfigField, response: DriveResponse) -> ConfigValue:
        if len(response.payload) < 5:
            raise DriverSessionError("config response payload is too short")
        response_key = response.payload[0]
        if response_key != field_info.key:
            raise DriverSessionError(
                f"config response key mismatch: expected 0x{field_info.key:02X}, got 0x{response_key:02X}"
            )
        value_bytes = response.payload[1:5]
        if field_info.value_kind in ("can_id", "control_mode", "u32"):
            return struct.unpack("<I", value_bytes)[0]
        return struct.unpack("<f", value_bytes)[0]

    def _raise_for_error(self, response: DriveResponse, message: str) -> None:
        if response.response_type != DRIVE_RESPONSE_ERROR:
            return
        error_code = response.payload[0] if response.payload else None
        suffix = "" if error_code is None else f": code={error_code}"
        raise DriverCommandError(message + suffix, response)