# Open-tDCS-Stack

Open-source transcranial Direct Current Stimulation (tDCS) device using the Seeed Studio XIAO BLE nRF52840.

## Overview

Open-tDCS-Stack is a Bluetooth-enabled tDCS device that provides precise current control with real-time monitoring capabilities through a web interface.

## Hardware

- **Microcontroller**: Seeed Studio XIAO BLE nRF52840
- **Connectivity**: Bluetooth 5.0 (BLE)
- **Current Range**: 0-4mA (configurable)
- **Battery**: Rechargeable via USB-C

### Pinout Reference

See [docs/PINOUT.md](docs/PINOUT.md) for complete pinout information and hardware specifications.

## Firmware

The firmware is located in `firmware/opentdcs/` and is built using the Arduino framework with the Adafruit nRF52 BSP.

### Features

- Real-time current monitoring and control
- Battery voltage monitoring
- Impedance calculation
- Configurable session parameters (duration, ramp up/down)
- BLE GATT service for wireless control
- Visual status indication via RGB LED

### Pin Usage

| Function | Pin | Description |
|----------|-----|-------------|
| PWM Output | D3 (P0.29) | Current control signal |
| Current Sense | A0 (P0.02) | Measures output current |
| Battery Voltage | A1 (P0.03) | Battery monitoring |
| Output Voltage | A2 (P0.28) | Output voltage monitoring |

## Web Interface

The web GUI is located in `gui/index.html` and provides:

- Device connection via Web Bluetooth API
- Session control and monitoring
- Real-time current and impedance graphs
- Session history tracking
- Battery and voltage monitoring

### Usage

1. Open `gui/index.html` in a Web Bluetooth-compatible browser (Chrome, Edge)
2. Click "Connect Device" and select your tDCS device
3. Configure session parameters (current, duration, ramp time)
4. Start session and monitor real-time data



## Safety Notes

- This is an experimental device for research purposes only
- Always follow proper tDCS safety protocols
- Maximum current is limited to 4mA in firmware
- Device includes impedance monitoring for electrode contact quality
- Automatic session termination on completion

## License

[License information to be added]

## References

- [Seeed Studio XIAO BLE Documentation](https://wiki.seeedstudio.com/XIAO_BLE/)
- [Hardware Pinout Details](docs/PINOUT.md)
