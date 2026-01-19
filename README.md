# Open-tDCS-Stack

Open-source transcranial Direct Current Stimulation (tDCS) device using the Seeed Studio XIAO BLE nRF52840.

## Overview

Open-tDCS-Stack is a Bluetooth-enabled tDCS device that provides precise current control with real-time monitoring capabilities through a web interface.

## Hardware

- **Microcontroller**: Seeed Studio XIAO BLE nRF52840
- **Connectivity**: Bluetooth 5.0 (BLE)
- **Current Range**: 0-4mA (configurable)
- **Battery**: 4x CR2025



### Schematics 

![Open-tDCS Schematic](docs/schematics.png)


## Firmware

The firmware is located in `firmware/build/opentdcs.ino.uf2`
To install on the XIAO BLE nRF52840. Place it into bootloader mode by quickly pressing the reset button twice. 
A XIAO USB drive will appear on your computer. Drag in the `firmware/build/opentdcs.ino.uf2` file. Done


### Features

- Real-time current monitoring and control
- Battery voltage monitoring
- Impedance calculation
- Configurable session parameters (duration, ramp up/down)
- BLE GATT service for wireless control
- Visual status indication via RGB LED
- Dickson charge pump voltage multiplier for higher output voltage
- Test current (50µA) applied when connected but not in session

### Pin Usage

| Function | Pin | Description |
|----------|-----|-------------|
| PWM Output | D10 | Current control signal |
| Current Sense | A0 | Measures output current via 1kΩ sense resistor |
| Output Voltage | A1 | Output voltage monitoring (5.3x divider) |
| Battery Voltage | A2 | Battery monitoring (5.3x divider) |
| Multiplier A1 | D9 | Dickson charge pump driver |
| Multiplier B | D8 | Dickson charge pump driver (inverted) |
| Multiplier A2 | D7 | Dickson charge pump driver |

### LED Status

| State | Color | Pattern |
|-------|-------|---------|
| Idle/Advertising | Green | Blink (50ms every 4s) |
| Connected | Blue | Solid |
| Session Active | Red | Solid |

## Web Interface

TLDR: Web UI is hosted at https://unrelatedlabs.github.io/Open-tDCS-Stack/gui/



The web GUI is located in `gui/index.html` and provides:

- Device connection via Web Bluetooth API
- Session control and monitoring
- Real-time current and impedance graphs
- Session history tracking
- Battery and voltage monitoring

### Usage

1. Open in Web Bluetooth-compatible browser (Chrome, Edge). On iOS use the V Browser https://apps.apple.com/us/app/the-v-browser/id6446792641
2. Click "Connect Device" and select your tDCS device
3. Configure session parameters (current, duration, ramp time)
4. Start session and monitor real-time data


## Safety Notes

- This is an experimental device for research purposes only
- Always follow proper tDCS safety protocols
- Maximum current is limited to 4mA in hardware.
- Device includes impedance monitoring for electrode contact quality
- Automatic session termination on completion

## License

[License information to be added]

## References

- [Seeed Studio XIAO BLE Documentation](https://wiki.seeedstudio.com/XIAO_BLE/)
 