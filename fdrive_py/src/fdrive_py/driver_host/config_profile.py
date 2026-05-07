from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

from .protocol import (
    CONTROL_MODE_CURRENT,
    CONTROL_MODE_OTHER,
    CONTROL_MODE_PWM,
    DRIVE_CONFIG_KEY_BUS_VOLTAGE_OFFSET_VOLTS,
    DRIVE_CONFIG_KEY_CAN_BASE_ID,
    DRIVE_CONFIG_KEY_CONTROL_MODE,
    DRIVE_CONFIG_KEY_CURRENT_ERROR_CLAMP_AMPS,
    DRIVE_CONFIG_KEY_CURRENT_INVERTED,
    DRIVE_CONFIG_KEY_CURRENT_KI_DOWN,
    DRIVE_CONFIG_KEY_CURRENT_KI_UP,
    DRIVE_CONFIG_KEY_CURRENT_LIMIT_AMPS,
    DRIVE_CONFIG_KEY_CURRENT_OVERCURRENT_MARGIN_AMPS,
    DRIVE_CONFIG_KEY_CURRENT_OVERCURRENT_MARGIN_PERCENT,
    DRIVE_CONFIG_KEY_CURRENT_ZERO_CHANNEL_VOLTS,
    DRIVE_CONFIG_KEY_PWM_BACKOFF_PER_SEC,
    DRIVE_CONFIG_KEY_PWM_ERROR_CLAMP,
    DRIVE_CONFIG_KEY_PWM_RAMP_UP_PER_SEC,
    control_mode_from_name,
    control_mode_name,
    parse_can_id,
)


SCHEMA_VERSION = 1
ConfigValue = int | float


@dataclass(frozen=True)
class ConfigField:
    key: int
    name: str
    label: str
    value_kind: str

    def parse(self, value: object) -> ConfigValue:
        if self.value_kind == "can_id":
            return _parse_int(value, name=self.name)
        if self.value_kind == "control_mode":
            if isinstance(value, str):
                return control_mode_from_name(value)
            return _parse_int(value, name=self.name)
        if self.value_kind == "u32":
            return _parse_int(value, name=self.name)
        if self.value_kind == "float":
            return _parse_float(value, name=self.name)
        raise ValueError(f"unsupported config value kind: {self.value_kind}")

    def format(self, value: object) -> str:
        parsed = self.parse(value)
        if self.value_kind == "can_id":
            return f"0x{int(parsed):X}"
        if self.value_kind == "control_mode":
            return control_mode_name(int(parsed))
        if self.value_kind == "u32":
            return str(int(parsed))
        return f"{float(parsed):.6g}"

    def to_json_value(self, value: object) -> str | int | float:
        parsed = self.parse(value)
        if self.value_kind == "can_id":
            return f"0x{int(parsed):X}"
        if self.value_kind == "control_mode":
            return control_mode_name(int(parsed))
        if self.value_kind == "u32":
            return int(parsed)
        return float(parsed)


CONFIG_FIELDS: tuple[ConfigField, ...] = (
    ConfigField(DRIVE_CONFIG_KEY_CAN_BASE_ID, "can_base_id", "CAN Base ID", "can_id"),
    ConfigField(DRIVE_CONFIG_KEY_CURRENT_ZERO_CHANNEL_VOLTS, "current_zero_channel_volts", "Current Zero Channel (V)", "float"),
    ConfigField(DRIVE_CONFIG_KEY_BUS_VOLTAGE_OFFSET_VOLTS, "bus_voltage_offset_volts", "Bus Voltage Offset (V)", "float"),
    ConfigField(DRIVE_CONFIG_KEY_CONTROL_MODE, "control_mode", "Control Mode", "control_mode"),
    ConfigField(DRIVE_CONFIG_KEY_PWM_RAMP_UP_PER_SEC, "pwm_ramp_up_per_sec", "PWM Ramp Up / s", "float"),
    ConfigField(DRIVE_CONFIG_KEY_PWM_ERROR_CLAMP, "pwm_error_clamp", "PWM Error Clamp", "float"),
    ConfigField(DRIVE_CONFIG_KEY_PWM_BACKOFF_PER_SEC, "pwm_backoff_per_sec", "PWM Backoff / s", "float"),
    ConfigField(DRIVE_CONFIG_KEY_CURRENT_LIMIT_AMPS, "current_limit_amps", "Current Limit (A)", "float"),
    ConfigField(DRIVE_CONFIG_KEY_CURRENT_INVERTED, "current_inverted", "Invert Current", "u32"),
    ConfigField(DRIVE_CONFIG_KEY_CURRENT_KI_UP, "current_ki_up", "Current KI Up", "float"),
    ConfigField(DRIVE_CONFIG_KEY_CURRENT_KI_DOWN, "current_ki_down", "Current KI Down", "float"),
    ConfigField(
        DRIVE_CONFIG_KEY_CURRENT_OVERCURRENT_MARGIN_AMPS,
        "current_overcurrent_margin_amps",
        "Overcurrent Margin (A)",
        "float",
    ),
    ConfigField(
        DRIVE_CONFIG_KEY_CURRENT_OVERCURRENT_MARGIN_PERCENT,
        "current_overcurrent_margin_percent",
        "Overcurrent Margin Ratio",
        "float",
    ),
    ConfigField(DRIVE_CONFIG_KEY_CURRENT_ERROR_CLAMP_AMPS, "current_error_clamp_amps", "Current Error Clamp (A)", "float"),
)

CONTROL_MODE_CHOICES = tuple(control_mode_name(mode) for mode in (CONTROL_MODE_PWM, CONTROL_MODE_CURRENT, CONTROL_MODE_OTHER))
CONFIG_FIELD_BY_NAME = {field.name: field for field in CONFIG_FIELDS}
CONFIG_FIELD_BY_KEY = {field.key: field for field in CONFIG_FIELDS}


def config_field(field: ConfigField | str | int) -> ConfigField:
    if isinstance(field, ConfigField):
        return field
    if isinstance(field, str):
        normalized = field.strip().lower()
        if normalized in CONFIG_FIELD_BY_NAME:
            return CONFIG_FIELD_BY_NAME[normalized]
        raise ValueError(f"unknown config field: {field}")
    if field in CONFIG_FIELD_BY_KEY:
        return CONFIG_FIELD_BY_KEY[field]
    raise ValueError(f"unknown config key: {field}")


def normalize_config_values(values: Mapping[str | int | ConfigField, object]) -> dict[str, ConfigValue]:
    normalized: dict[str, ConfigValue] = {}
    for key, value in values.items():
        field_info = config_field(key)
        normalized[field_info.name] = field_info.parse(value)
    return normalized


def iter_config_fields_for_write(values: Mapping[str, object] | None = None) -> tuple[ConfigField, ...]:
    fields = [field_info for field_info in CONFIG_FIELDS if values is None or field_info.name in values]
    fields.sort(key=lambda field_info: field_info.name == "can_base_id")
    return tuple(fields)


@dataclass
class DriverConfigProfile:
    values: dict[str, ConfigValue] = field(default_factory=dict)
    base_id: int | None = None
    schema_version: int = SCHEMA_VERSION

    def __post_init__(self) -> None:
        if self.schema_version != SCHEMA_VERSION:
            raise ValueError(f"unsupported driver config schema version: {self.schema_version}")
        self.values = normalize_config_values(self.values)
        if self.base_id is not None:
            self.base_id = parse_can_id(str(self.base_id)) if isinstance(self.base_id, str) else int(self.base_id)

    @classmethod
    def from_dict(cls, data: Mapping[str, object]) -> DriverConfigProfile:
        schema_version = _parse_int(data.get("schema_version", SCHEMA_VERSION), name="schema_version")
        raw_config = data.get("config", {})
        if not isinstance(raw_config, Mapping):
            raise ValueError("config profile must contain an object named 'config'")

        raw_base_id = data.get("base_id")
        base_id = None if raw_base_id is None else parse_can_id(str(raw_base_id))
        return cls(values=normalize_config_values(raw_config), base_id=base_id, schema_version=schema_version)

    def to_dict(self) -> dict[str, object]:
        config: dict[str, str | int | float] = {}
        for field_info in CONFIG_FIELDS:
            if field_info.name in self.values:
                config[field_info.name] = field_info.to_json_value(self.values[field_info.name])

        data: dict[str, object] = {
            "schema_version": self.schema_version,
            "config": config,
        }
        if self.base_id is not None:
            data["base_id"] = f"0x{self.base_id:X}"
        return data

    @classmethod
    def from_json(cls, text: str) -> DriverConfigProfile:
        data = json.loads(text)
        if not isinstance(data, Mapping):
            raise ValueError("config profile JSON must contain an object")
        return cls.from_dict(data)

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2) + "\n"

    @classmethod
    def load_json(cls, path: str | Path) -> DriverConfigProfile:
        return cls.from_json(Path(path).read_text(encoding="utf-8"))

    def save_json(self, path: str | Path) -> None:
        Path(path).write_text(self.to_json(), encoding="utf-8")

    def missing_field_names(self) -> tuple[str, ...]:
        return tuple(field_info.name for field_info in CONFIG_FIELDS if field_info.name not in self.values)


def _parse_int(value: object, *, name: str) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be an integer, not a boolean")
    if isinstance(value, int):
        return value
    if isinstance(value, float) and value.is_integer():
        return int(value)
    if isinstance(value, str):
        try:
            return parse_can_id(value.strip())
        except ValueError as exc:
            raise ValueError(f"{name} must be an integer") from exc
    raise ValueError(f"{name} must be an integer")


def _parse_float(value: object, *, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a number, not a boolean")
    if isinstance(value, int | float):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value.strip())
        except ValueError as exc:
            raise ValueError(f"{name} must be a number") from exc
    raise ValueError(f"{name} must be a number")