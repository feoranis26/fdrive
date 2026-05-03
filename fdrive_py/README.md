# fdrive-py

`fdrive-py` packages the Python host tools for the `dc_driver` firmware so they can be installed as a cross-platform Python distribution.

## Included Tools

- `fdrive-config` for driver discovery, configuration reads and writes, calibration, and fault clearing.
- `fdrive-joystick` for live joystick-based control.

## Backends

- `usb-can` uses the existing UART ASCII bridge and is intended for Windows or any host using the current USB serial bridge workflow.
- `socketcan` uses `python-can` on Linux systems such as Raspberry Pi devices connected to a SocketCAN interface like `can0`.

## Install

```bash
pip install .
pip install .[joystick]
```

## Development

Run the package tests directly from the project root:

```bash
python -m unittest discover -s tests
```

## Examples

```bash
fdrive-config --backend usb-can --port COM3 scan
fdrive-config --backend socketcan --channel can0 scan
fdrive-joystick --backend usb-can --port COM6 --base-id 0x100 --control-mode pwm
fdrive-joystick --backend socketcan --channel can0 --base-id 0x100
```