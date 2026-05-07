from __future__ import annotations

import unittest

from fdrive_py.driver_host import (
    CONFIG_FIELDS,
    CONTROL_MODE_CURRENT,
    DriverConfigProfile,
    config_field,
    iter_config_fields_for_write,
    normalize_config_values,
)


class ConfigProfileTests(unittest.TestCase):
    def test_profile_round_trips_readable_json_values(self) -> None:
        values = {
            "can_base_id": "0x110",
            "control_mode": "current",
            "current_inverted": 1,
            "current_limit_amps": 22.5,
            "pwm_ramp_up_per_sec": 3.25,
        }

        profile = DriverConfigProfile(values=values, base_id=0x110)
        loaded = DriverConfigProfile.from_json(profile.to_json())

        self.assertEqual(loaded.base_id, 0x110)
        self.assertEqual(loaded.values["can_base_id"], 0x110)
        self.assertEqual(loaded.values["control_mode"], CONTROL_MODE_CURRENT)
        self.assertEqual(loaded.values["current_inverted"], 1)
        self.assertAlmostEqual(float(loaded.values["current_limit_amps"]), 22.5)
        self.assertEqual(profile.to_dict()["config"]["control_mode"], "current")
        self.assertEqual(profile.to_dict()["config"]["can_base_id"], "0x110")

    def test_unknown_key_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            normalize_config_values({"not_a_real_key": 1.0})

    def test_field_parsers_accept_display_values(self) -> None:
        self.assertEqual(config_field("can_base_id").parse("0x120"), 0x120)
        self.assertEqual(config_field("control_mode").parse("current"), CONTROL_MODE_CURRENT)
        self.assertAlmostEqual(float(config_field("current_limit_amps").parse("12.75")), 12.75)

    def test_write_order_puts_base_id_last(self) -> None:
        values = {field.name: 1 for field in CONFIG_FIELDS}

        ordered_names = [field.name for field in iter_config_fields_for_write(values)]

        self.assertEqual(ordered_names[-1], "can_base_id")


if __name__ == "__main__":
    unittest.main()
