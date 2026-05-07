from __future__ import annotations

import struct
from dataclasses import dataclass


CONTROL_OFFSET = 0
STATUS_OFFSET = 1
MODE_OFFSET = 2
MEASUREMENT_OFFSET = 3
COMMAND_OFFSET = 4
RESPONSE_OFFSET = 5

DRIVE_STATUS_SIGNATURE = 0xD7

MODE_ACCEL = 0
MODE_BRAKE = 1

DRIVE_COMMAND_CALIBRATE = 0x01
DRIVE_COMMAND_CONFIG_GET = 0x02
DRIVE_COMMAND_CONFIG_SET = 0x03
DRIVE_COMMAND_CLEAR_FAULT = 0x04

DRIVE_RESPONSE_ACK = 0x80
DRIVE_RESPONSE_IN_PROGRESS = 0x81
DRIVE_RESPONSE_OK = 0x82
DRIVE_RESPONSE_ERROR = 0xFF

DRIVE_CONFIG_KEY_CAN_BASE_ID = 0x01
DRIVE_CONFIG_KEY_CURRENT_ZERO_CHANNEL_VOLTS = 0x02
DRIVE_CONFIG_KEY_BUS_VOLTAGE_OFFSET_VOLTS = 0x03
DRIVE_CONFIG_KEY_CONTROL_MODE = 0x04
DRIVE_CONFIG_KEY_CURRENT_LIMIT_AMPS = 0x05
DRIVE_CONFIG_KEY_CURRENT_KI_UP = 0x06
DRIVE_CONFIG_KEY_CURRENT_KI_DOWN = 0x07
DRIVE_CONFIG_KEY_CURRENT_OVERCURRENT_MARGIN_AMPS = 0x08
DRIVE_CONFIG_KEY_CURRENT_OVERCURRENT_MARGIN_PERCENT = 0x09
DRIVE_CONFIG_KEY_CURRENT_ERROR_CLAMP_AMPS = 0x0A
DRIVE_CONFIG_KEY_PWM_RAMP_UP_PER_SEC = 0x0B
DRIVE_CONFIG_KEY_PWM_ERROR_CLAMP = 0x0C
DRIVE_CONFIG_KEY_PWM_BACKOFF_PER_SEC = 0x0D
DRIVE_CONFIG_KEY_CURRENT_INVERTED = 0x0E

CONTROL_MODE_PWM = 0
CONTROL_MODE_CURRENT = 1
CONTROL_MODE_OTHER = 2

CONTROLLER_MODE_RUNNING = 0
CONTROLLER_MODE_DISABLED = 1
CONTROLLER_MODE_CALIBRATING = 2
CONTROLLER_MODE_FAULT_LOCKOUT = 3

CONFIG_KEY_NAME_MAP = {
    DRIVE_CONFIG_KEY_CAN_BASE_ID: "can_base_id",
    DRIVE_CONFIG_KEY_CURRENT_ZERO_CHANNEL_VOLTS: "current_zero_channel_volts",
    DRIVE_CONFIG_KEY_BUS_VOLTAGE_OFFSET_VOLTS: "bus_voltage_offset_volts",
    DRIVE_CONFIG_KEY_CONTROL_MODE: "control_mode",
    DRIVE_CONFIG_KEY_CURRENT_LIMIT_AMPS: "current_limit_amps",
    DRIVE_CONFIG_KEY_CURRENT_KI_UP: "current_ki_up",
    DRIVE_CONFIG_KEY_CURRENT_KI_DOWN: "current_ki_down",
    DRIVE_CONFIG_KEY_CURRENT_OVERCURRENT_MARGIN_AMPS: "current_overcurrent_margin_amps",
    DRIVE_CONFIG_KEY_CURRENT_OVERCURRENT_MARGIN_PERCENT: "current_overcurrent_margin_percent",
    DRIVE_CONFIG_KEY_CURRENT_ERROR_CLAMP_AMPS: "current_error_clamp_amps",
    DRIVE_CONFIG_KEY_PWM_RAMP_UP_PER_SEC: "pwm_ramp_up_per_sec",
    DRIVE_CONFIG_KEY_PWM_ERROR_CLAMP: "pwm_error_clamp",
    DRIVE_CONFIG_KEY_PWM_BACKOFF_PER_SEC: "pwm_backoff_per_sec",
    DRIVE_CONFIG_KEY_CURRENT_INVERTED: "current_inverted",
}

CONFIG_KEY_VALUE_MAP = {value: key for key, value in CONFIG_KEY_NAME_MAP.items()}

CONTROL_MODE_NAME_MAP = {
    CONTROL_MODE_PWM: "pwm",
    CONTROL_MODE_CURRENT: "current",
    CONTROL_MODE_OTHER: "other",
}

CONTROL_MODE_VALUE_MAP = {value: key for key, value in CONTROL_MODE_NAME_MAP.items()}

RESPONSE_NAME_MAP = {
    DRIVE_RESPONSE_ACK: "ack",
    DRIVE_RESPONSE_IN_PROGRESS: "in_progress",
    DRIVE_RESPONSE_OK: "ok",
    DRIVE_RESPONSE_ERROR: "error",
}

CONTROLLER_MODE_NAME_MAP = {
    CONTROLLER_MODE_RUNNING: "running",
    CONTROLLER_MODE_DISABLED: "disabled",
    CONTROLLER_MODE_CALIBRATING: "calibrating",
    CONTROLLER_MODE_FAULT_LOCKOUT: "fault_lockout",
}


def parse_can_id(value: str) -> int:
    return int(value, 0)


def build_id(base_id: int, offset: int) -> int:
    return base_id + offset


def encode_frequency_field(frequency_hz: int) -> int:
    encoded = int((frequency_hz + 5) / 10)
    return max(0, min(encoded, 0xFFFF))


def decode_frequency_field(encoded_frequency: int) -> int:
    return int(encoded_frequency) * 10


def status_signature_matches(data: bytes) -> bool:
    return len(data) >= 8 and data[7] == DRIVE_STATUS_SIGNATURE


def config_key_name(key: int) -> str:
    return CONFIG_KEY_NAME_MAP.get(key, f"unknown({key})")


def config_key_from_name(name: str) -> int:
    normalized = name.strip().lower()
    if normalized not in CONFIG_KEY_VALUE_MAP:
        raise ValueError(f"unknown config key: {name}")
    return CONFIG_KEY_VALUE_MAP[normalized]


def control_mode_name(mode: int) -> str:
    return CONTROL_MODE_NAME_MAP.get(mode, f"unknown({mode})")


def control_mode_from_name(name: str) -> int:
    normalized = name.strip().lower()
    if normalized.isdigit():
        return int(normalized)
    if normalized not in CONTROL_MODE_VALUE_MAP:
        raise ValueError(f"unknown control mode: {name}")
    return CONTROL_MODE_VALUE_MAP[normalized]


def response_type_name(response_type: int) -> str:
    return RESPONSE_NAME_MAP.get(response_type, f"unknown({response_type})")


def controller_mode_name(mode: int) -> str:
    return CONTROLLER_MODE_NAME_MAP.get(mode, f"unknown({mode})")


@dataclass
class SerialFrame:
    arbitration_id: int
    data: bytes
    direction: str


@dataclass
class SerialControlMessage:
    text: str


@dataclass
class DriveStatus:
    arbitration_id: int
    mode: int
    frequency_hz: int
    duty_cycle: float
    has_signature: bool


@dataclass
class DriveMeasurements:
    arbitration_id: int
    bus_voltage_volts: float
    current_amps: float


@dataclass
class DriveControllerMode:
    arbitration_id: int
    mode: int


@dataclass
class DriveResponse:
    arbitration_id: int
    response_type: int
    command: int
    payload: bytes

    def payload_as_u32(self) -> int:
        if len(self.payload) < 4:
            raise ValueError("response payload too short for u32")
        return struct.unpack_from("<I", self.payload, 0)[0]

    def payload_as_float(self) -> float:
        if len(self.payload) < 4:
            raise ValueError("response payload too short for float")
        return struct.unpack_from("<f", self.payload, 0)[0]