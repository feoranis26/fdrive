# fdrive-py

`fdrive-py` packages the Python host tools for the `dc_driver` firmware so they can be installed as a cross-platform Python distribution.

## Included Tools

- `fdrive-config` for driver discovery, configuration reads and writes, calibration, and fault clearing.
- `fdrive-joystick` for live joystick-based control.
- `fdrive-gui` for a Tk-based configuration, telemetry, calibration, fault clearing, and joystick control interface.

## Backends

- `usb-can` uses the existing UART ASCII bridge and is intended for Windows or any host using the current USB serial bridge workflow.
- `socketcan` uses `python-can` on Linux systems such as Raspberry Pi devices connected to a SocketCAN interface like `can0`.

## Install

```bash
pip install .
```

The GUI and joystick tool both use `pygame` for joystick input, so it is installed by default.

## Development

Run the package tests directly from the project root:

```bash
python -m unittest discover -s tests
```

## Examples

```bash
fdrive-config --backend usb-can --port COM3 scan
fdrive-config --backend socketcan --channel can0 scan
fdrive-gui
fdrive-joystick --backend usb-can --port COM6 --base-id 0x100 --control-mode pwm
fdrive-joystick --backend socketcan --channel can0 --base-id 0x100
```

## GUI Config Profiles

The GUI can load the full persisted driver configuration from a connected driver, save edited values back to the driver, and import or export the same values as JSON. Profiles are versioned and store values by protocol key name, for example:

```json
{
	"schema_version": 1,
	"base_id": "0x100",
	"config": {
		"can_base_id": "0x100",
		"control_mode": "pwm",
		"current_limit_amps": 20.0
	}
}
```

When a full profile is saved to the driver, `can_base_id` is written last so the GUI can switch to the new base ID after the firmware accepts the change.