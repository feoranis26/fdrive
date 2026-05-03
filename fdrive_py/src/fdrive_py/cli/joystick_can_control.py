from __future__ import annotations

import argparse
import math
import time

import pygame

from fdrive_py.driver_host import (
    DRIVE_COMMAND_CONFIG_SET,
    DRIVE_CONFIG_KEY_CONTROL_MODE,
    DRIVE_RESPONSE_ERROR,
    CanTransport,
    DriverInterface,
    MODE_ACCEL,
    control_mode_from_name,
    controller_mode_name,
    parse_can_id,
    response_type_name,
)

from .common import add_backend_arguments, create_transport_from_args


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Joystick interface for a single fdrive node over USB-CAN or SocketCAN.",
    )
    add_backend_arguments(parser, default_port="COM6")
    parser.add_argument("--base-id", type=parse_can_id, default=0x100, help="Base CAN ID for the target driver.")
    parser.add_argument("--joystick-index", type=int, default=0, help="Joystick index to open.")
    parser.add_argument("--throttle-axis", type=int, default=1, help="Axis used for throttle input.")
    parser.add_argument("--reverse-toggle-button", type=int, default=0, help="Button that toggles reverse mode.")
    parser.add_argument("--quit-button", type=int, default=6, help="Optional button that exits the program.")
    parser.add_argument("--frequency", type=int, default=1000, help="PWM frequency to send in fast CAN commands.")
    parser.add_argument("--max-duty", type=float, default=1.0, help="Clamp joystick demand to this magnitude.")
    parser.add_argument(
        "--control-mode",
        choices=["pwm", "current", "other"],
        help="Persistently select the drive control mode before sending joystick commands.",
    )
    parser.add_argument("--deadzone", type=float, default=0.1, help="Joystick deadzone from 0.0 to 1.0.")
    parser.add_argument("--loop-rate", type=float, default=20.0, help="Command update rate in Hz.")
    parser.add_argument("--status-period", type=float, default=0.1, help="Seconds between status prints.")
    parser.add_argument(
        "--invert-axis",
        action="store_true",
        help="Invert throttle axis direction if the joystick reports forward as positive.",
    )
    return parser


class JoystickDriveController:
    def __init__(self, args: argparse.Namespace, transport: CanTransport, driver: DriverInterface):
        if not 0.0 <= args.deadzone < 1.0:
            raise ValueError("deadzone must be in the range [0.0, 1.0)")
        if not 0.0 < args.max_duty <= 1.0:
            raise ValueError("max-duty must be in the range (0.0, 1.0]")
        if args.loop_rate <= 0.0:
            raise ValueError("loop-rate must be greater than zero")

        self.args = args
        self.transport = transport
        self.driver = driver

        pygame.display.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() <= args.joystick_index:
            raise RuntimeError(
                f"joystick index {args.joystick_index} unavailable; found {pygame.joystick.get_count()} device(s)"
            )

        self.joystick = pygame.joystick.Joystick(args.joystick_index)
        self.joystick.init()

        self.reverse_enabled = False
        self.reverse_button_latched = False
        self.last_status_print = 0.0

    def run(self) -> None:
        update_period = 1.0 / self.args.loop_rate
        print(f"Using joystick: {self.joystick.get_name()}")
        print(
            f"Driver IDs control/status/mode/measurement/command/response = "
            f"0x{self.driver.control_id:X}/0x{self.driver.status_id:X}/0x{self.driver.mode_id:X}/"
            f"0x{self.driver.measurement_id:X}/0x{self.driver.command_id:X}/0x{self.driver.response_id:X}"
        )
        if self.transport.supports_mirroring:
            if self.transport.enable_mirroring():
                print("Transport mirroring enabled.")
            else:
                print("Transport mirroring enable timed out; continuing without confirmation.")
        else:
            print("Using direct CAN backend; mirroring step not required.")
        print("Negative throttle brakes by default; press the toggle button to switch negative throttle into reverse.")

        if self.args.control_mode is not None:
            control_mode = control_mode_from_name(self.args.control_mode)
            self.driver.send_config_set(DRIVE_CONFIG_KEY_CONTROL_MODE, control_mode)
            response = self.driver.wait_for_response(DRIVE_COMMAND_CONFIG_SET, 1.5)
            if response is None:
                print("Timed out waiting for control mode configuration response.")
            elif response.response_type == DRIVE_RESPONSE_ERROR:
                print("Failed to update control mode.")
            else:
                print(f"Control mode set to {self.args.control_mode}.")

        try:
            while True:
                loop_start = time.monotonic()
                pygame.event.pump()

                self.poll_frames()

                if self.handle_buttons():
                    break

                demand = self.read_axis(self.args.throttle_axis, invert=self.args.invert_axis)
                self.driver.send_signed_duty(demand, self.args.frequency, reverse_enabled=self.reverse_enabled)
                self.print_status(loop_start, demand)

                elapsed = time.monotonic() - loop_start
                if elapsed < update_period:
                    time.sleep(update_period - elapsed)
        except KeyboardInterrupt:
            pass
        finally:
            self.driver.stop(self.args.frequency)
            pygame.joystick.quit()
            pygame.display.quit()

    def read_axis(self, axis: int, *, invert: bool) -> float:
        raw_value = self.joystick.get_axis(axis)
        if not invert:
            raw_value = -raw_value

        magnitude = abs(raw_value)
        if magnitude <= self.args.deadzone:
            return 0.0

        scaled = (magnitude - self.args.deadzone) / (1.0 - self.args.deadzone)
        scaled = min(max(scaled, 0.0), 1.0)
        return math.copysign(scaled * self.args.max_duty, raw_value)

    def handle_buttons(self) -> bool:
        reverse_pressed = bool(self.joystick.get_button(self.args.reverse_toggle_button))
        if reverse_pressed and not self.reverse_button_latched:
            self.reverse_enabled = not self.reverse_enabled
            state = "reverse" if self.reverse_enabled else "brake"
            print(f"Negative throttle mode switched to {state}.")

        self.reverse_button_latched = reverse_pressed

        if self.args.quit_button >= 0 and self.joystick.get_button(self.args.quit_button):
            print("Quit button pressed.")
            return True

        return False

    def poll_frames(self) -> None:
        while True:
            message = self.transport.recv_frame(timeout=0.0)
            if message is None:
                return
            self.driver.process_frame(message)

    def print_status(self, now: float, demand: float) -> None:
        if now - self.last_status_print < self.args.status_period:
            return

        self.last_status_print = now
        negative_mode = "reverse" if self.reverse_enabled else "brake"
        print(f"Demand={demand:+.3f}, negative throttle={negative_mode}")

        if self.driver.latest_status is None:
            print("Status: no frame received yet.")
        else:
            status = self.driver.latest_status
            mode_name = "accel" if status.mode == MODE_ACCEL else "brake"
            print(
                f"Status: mode={mode_name} duty={status.duty_cycle:+.3f} "
                f"frequency={status.frequency_hz}Hz id=0x{status.arbitration_id:X}"
            )

        if self.driver.latest_mode is None:
            print("Controller mode: no frame received yet.")
        else:
            print(f"Controller mode: {controller_mode_name(self.driver.latest_mode.mode)}")

        if self.driver.latest_measurements is None:
            print("Measurements: no frame received yet.")
        else:
            measurement = self.driver.latest_measurements
            print(
                f"Measurements: bus={measurement.bus_voltage_volts:.3f} V "
                f"current={measurement.current_amps:.3f} A"
            )

        if self.driver.latest_response is not None:
            payload_hex = " ".join(f"{byte:02X}" for byte in self.driver.latest_response.payload)
            print(
                f"Command response: id=0x{self.driver.latest_response.arbitration_id:X} "
                f"type={response_type_name(self.driver.latest_response.response_type)} "
                f"cmd=0x{self.driver.latest_response.command:02X} {payload_hex}"
            )


def main() -> int:
    args = build_parser().parse_args()

    transport = create_transport_from_args(args)
    try:
        driver = DriverInterface(transport, args.base_id)
        controller = JoystickDriveController(args, transport, driver)
        controller.run()
    finally:
        transport.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())