from __future__ import annotations

import argparse
import struct
import time

from fdrive_py.driver_host import (
    DRIVE_COMMAND_CALIBRATE,
    DRIVE_COMMAND_CLEAR_FAULT,
    DRIVE_COMMAND_CONFIG_GET,
    DRIVE_COMMAND_CONFIG_SET,
    DRIVE_CONFIG_KEY_CAN_BASE_ID,
    DRIVE_CONFIG_KEY_CONTROL_MODE,
    DRIVE_RESPONSE_ACK,
    DRIVE_RESPONSE_ERROR,
    DRIVE_RESPONSE_IN_PROGRESS,
    DRIVE_RESPONSE_OK,
    STATUS_OFFSET,
    CanTransport,
    DriverInterface,
    config_key_from_name,
    config_key_name,
    control_mode_from_name,
    control_mode_name,
    parse_can_id,
    response_type_name,
    status_signature_matches,
)

from .common import add_backend_arguments, create_transport_from_args


def build_parser() -> argparse.ArgumentParser:
    common = argparse.ArgumentParser(add_help=False)
    add_backend_arguments(common, default_port="COM3")
    subcommand_common = argparse.ArgumentParser(add_help=False)
    add_backend_arguments(subcommand_common, default_port="COM3", suppress_defaults=True)

    parser = argparse.ArgumentParser(description="fdrive config and discovery CLI.", parents=[common], conflict_handler="resolve")

    subparsers = parser.add_subparsers(dest="command", required=True)

    scan_parser = subparsers.add_parser("scan", parents=[subcommand_common], conflict_handler="resolve", help="Listen for driver status frames.")
    scan_parser.add_argument("--duration", type=float, default=3.0, help="Scan duration in seconds.")

    get_parser = subparsers.add_parser("get", parents=[subcommand_common], conflict_handler="resolve", help="Read a persisted config value.")
    get_parser.add_argument("--base-id", required=True, type=parse_can_id, help="Base CAN ID for the target driver.")
    get_parser.add_argument(
        "key",
        choices=[
            "can_base_id",
            "current_zero_channel_volts",
            "bus_voltage_offset_volts",
            "control_mode",
            "pwm_ramp_up_per_sec",
            "pwm_error_clamp",
            "pwm_backoff_per_sec",
            "current_limit_amps",
            "current_ki_up",
            "current_ki_down",
            "current_overcurrent_margin_amps",
            "current_overcurrent_margin_percent",
            "current_error_clamp_amps",
        ],
    )
    get_parser.add_argument("--timeout", type=float, default=1.5, help="Response timeout in seconds.")

    set_parser = subparsers.add_parser("set", parents=[subcommand_common], conflict_handler="resolve", help="Write a persisted config value.")
    set_parser.add_argument("--base-id", required=True, type=parse_can_id, help="Base CAN ID for the target driver.")
    set_parser.add_argument(
        "key",
        choices=[
            "can_base_id",
            "current_zero_channel_volts",
            "bus_voltage_offset_volts",
            "control_mode",
            "pwm_ramp_up_per_sec",
            "pwm_error_clamp",
            "pwm_backoff_per_sec",
            "current_limit_amps",
            "current_ki_up",
            "current_ki_down",
            "current_overcurrent_margin_amps",
            "current_overcurrent_margin_percent",
            "current_error_clamp_amps",
        ],
    )
    set_parser.add_argument("value", help="Value to write.")
    set_parser.add_argument("--timeout", type=float, default=1.5, help="Response timeout in seconds.")

    calibrate_parser = subparsers.add_parser("calibrate", parents=[subcommand_common], conflict_handler="resolve", help="Calibrate the driver using a known bus voltage.")
    calibrate_parser.add_argument("--base-id", required=True, type=parse_can_id, help="Base CAN ID for the target driver.")
    calibrate_parser.add_argument("known_voltage", type=float, help="Known bus voltage in volts.")
    calibrate_parser.add_argument("--timeout", type=float, default=10.0, help="Response timeout in seconds.")

    clear_fault_parser = subparsers.add_parser("clear-fault", parents=[subcommand_common], conflict_handler="resolve", help="Clear a latched fault condition.")
    clear_fault_parser.add_argument("--base-id", required=True, type=parse_can_id, help="Base CAN ID for the target driver.")
    clear_fault_parser.add_argument("--timeout", type=float, default=1.5, help="Response timeout in seconds.")

    return parser


def open_transport(args: argparse.Namespace) -> CanTransport:
    return create_transport_from_args(args)


def maybe_prepare_transport(transport: CanTransport, args: argparse.Namespace) -> None:
    if transport.supports_mirroring:
        if transport.enable_mirroring():
            print("Mirroring enabled.")
        else:
            print("Mirroring enable timed out; continuing anyway.")
        return

    print(f"Listening directly on {args.interface}:{args.channel}.")


def command_response_or_die(driver: DriverInterface, opcode: int, timeout: float) -> int:
    response = driver.wait_for_response(opcode, timeout)
    if response is None:
        print("Timed out waiting for response.")
        return 1

    if response.response_type == DRIVE_RESPONSE_ERROR:
        error_code = response.payload[0] if response.payload else None
        print(f"Error response for command 0x{opcode:02X}: code={error_code}")
        return 1

    print(f"Response: {response_type_name(response.response_type)}")
    return 0


def handle_scan(args: argparse.Namespace) -> int:
    transport = open_transport(args)
    try:
        maybe_prepare_transport(transport, args)

        deadline = time.monotonic() + args.duration
        discovered: dict[int, tuple[str, bytes]] = {}

        while time.monotonic() < deadline:
            frame = transport.recv_frame(timeout=min(0.1, max(0.0, deadline - time.monotonic())))
            if frame is None:
                continue
            if not status_signature_matches(frame.data):
                continue

            base_id = frame.arbitration_id - STATUS_OFFSET
            discovered[base_id] = (frame.direction, frame.data)

        if not discovered:
            print("No drivers discovered.")
            return 1

        for base_id in sorted(discovered):
            direction, data = discovered[base_id]
            mode = data[0]
            encoded_frequency = int.from_bytes(data[1:3], byteorder="little", signed=False)
            duty = struct.unpack("<f", data[3:7])[0]
            mode_name = "accel" if mode == 0 else "brake"
            print(
                f"base=0x{base_id:X} status=0x{base_id + STATUS_OFFSET:X} direction={direction} "
                f"mode={mode_name} freq={encoded_frequency * 10}Hz duty={duty:+.3f}"
            )
        return 0
    finally:
        transport.close()


def handle_get(args: argparse.Namespace) -> int:
    transport = open_transport(args)
    try:
        if transport.supports_mirroring:
            transport.enable_mirroring()
        driver = DriverInterface(transport, args.base_id)
        key = config_key_from_name(args.key)
        driver.send_config_get(key)
        response = driver.wait_for_response(DRIVE_COMMAND_CONFIG_GET, args.timeout)
        if response is None:
            print("Timed out waiting for config response.")
            return 1
        if response.response_type == DRIVE_RESPONSE_ERROR:
            code = response.payload[0] if response.payload else None
            print(f"Config get failed: code={code}")
            return 1
        if len(response.payload) < 5:
            print("Config get returned a short payload.")
            return 1

        response_key = response.payload[0]
        value_bytes = response.payload[1:5]
        if response_key == DRIVE_CONFIG_KEY_CAN_BASE_ID:
            value = int.from_bytes(value_bytes, byteorder="little", signed=False)
            print(f"{config_key_name(response_key)} = 0x{value:X} ({value})")
        elif response_key == DRIVE_CONFIG_KEY_CONTROL_MODE:
            value = int.from_bytes(value_bytes, byteorder="little", signed=False)
            print(f"{config_key_name(response_key)} = {control_mode_name(value)} ({value})")
        else:
            value = struct.unpack("<f", value_bytes)[0]
            print(f"{config_key_name(response_key)} = {value:.6f}")
        return 0
    finally:
        transport.close()


def handle_set(args: argparse.Namespace) -> int:
    transport = open_transport(args)
    try:
        if transport.supports_mirroring:
            transport.enable_mirroring()
        driver = DriverInterface(transport, args.base_id)
        key = config_key_from_name(args.key)
        value: int | float
        if key == DRIVE_CONFIG_KEY_CAN_BASE_ID:
            value = parse_can_id(args.value)
        elif key == DRIVE_CONFIG_KEY_CONTROL_MODE:
            value = control_mode_from_name(args.value)
        else:
            value = float(args.value)
        driver.send_config_set(key, value)
        return command_response_or_die(driver, DRIVE_COMMAND_CONFIG_SET, args.timeout)
    finally:
        transport.close()


def handle_calibrate(args: argparse.Namespace) -> int:
    transport = open_transport(args)
    try:
        if transport.supports_mirroring:
            transport.enable_mirroring()
        driver = DriverInterface(transport, args.base_id)
        driver.send_calibrate(args.known_voltage)

        deadline = time.monotonic() + args.timeout
        saw_ack = False
        while time.monotonic() < deadline:
            response = driver.wait_for_response(DRIVE_COMMAND_CALIBRATE, min(0.5, max(0.0, deadline - time.monotonic())))
            if response is None:
                continue
            print(f"Response: {response_type_name(response.response_type)}")
            if response.response_type == DRIVE_RESPONSE_ACK:
                saw_ack = True
                continue
            if response.response_type == DRIVE_RESPONSE_IN_PROGRESS:
                continue
            if response.response_type == DRIVE_RESPONSE_OK:
                return 0
            if response.response_type == DRIVE_RESPONSE_ERROR:
                code = response.payload[0] if response.payload else None
                print(f"Calibration failed: code={code}")
                return 1

        if saw_ack:
            print("Calibration did not complete before timeout.")
        else:
            print("Calibration acknowledgment was not received.")
        return 1
    finally:
        transport.close()


def handle_clear_fault(args: argparse.Namespace) -> int:
    transport = open_transport(args)
    try:
        if transport.supports_mirroring:
            transport.enable_mirroring()
        driver = DriverInterface(transport, args.base_id)
        driver.send_clear_fault()
        return command_response_or_die(driver, DRIVE_COMMAND_CLEAR_FAULT, args.timeout)
    finally:
        transport.close()


def main() -> int:
    args = build_parser().parse_args()
    if args.command == "scan":
        return handle_scan(args)
    if args.command == "get":
        return handle_get(args)
    if args.command == "set":
        return handle_set(args)
    if args.command == "calibrate":
        return handle_calibrate(args)
    if args.command == "clear-fault":
        return handle_clear_fault(args)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())