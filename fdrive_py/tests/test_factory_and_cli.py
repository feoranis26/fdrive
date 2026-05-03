from __future__ import annotations

import importlib.util
import unittest
from unittest.mock import patch

from fdrive_py.cli import common
from fdrive_py.cli import driver_config
from fdrive_py.driver_host.factory import create_transport


class FactoryTests(unittest.TestCase):
    def test_create_transport_selects_serial_backend(self) -> None:
        with patch("fdrive_py.driver_host.factory.SerialCanTransport") as serial_cls:
            create_transport("usb-can", port="COM7")

        serial_cls.assert_called_once()

    def test_create_transport_selects_socketcan_backend(self) -> None:
        with patch("fdrive_py.driver_host.factory.SocketCanTransport") as socketcan_cls:
            create_transport("socketcan", channel="can1")

        socketcan_cls.assert_called_once()


class CliTests(unittest.TestCase):
    def test_driver_config_parser_accepts_socketcan_backend(self) -> None:
        parser = driver_config.build_parser()

        args = parser.parse_args(["--backend", "socketcan", "--channel", "can1", "scan"])

        self.assertEqual(args.backend, "socketcan")
        self.assertEqual(args.channel, "can1")
        self.assertEqual(args.command, "scan")

    def test_common_transport_factory_passes_backend_args(self) -> None:
        parser = driver_config.build_parser()
        args = parser.parse_args(["--backend", "usb-can", "--port", "COM9", "scan"])

        with patch("fdrive_py.cli.common.create_transport") as create_transport_mock:
            common.create_transport_from_args(args)

        create_transport_mock.assert_called_once()
        self.assertEqual(create_transport_mock.call_args.kwargs["backend"], "usb-can")
        self.assertEqual(create_transport_mock.call_args.kwargs["port"], "COM9")

    @unittest.skipUnless(importlib.util.find_spec("pygame") is not None, "pygame not installed")
    def test_joystick_parser_accepts_socketcan_backend(self) -> None:
        from fdrive_py.cli import joystick_can_control

        parser = joystick_can_control.build_parser()
        args = parser.parse_args(["--backend", "socketcan", "--channel", "can0"])

        self.assertEqual(args.backend, "socketcan")
        self.assertEqual(args.channel, "can0")


if __name__ == "__main__":
    unittest.main()