<!--
SPDX-License-Identifier: CC-BY-SA-4.0
Copyright (C) 2024-2026 Peter Kuhar and OpenTDCS Contributors
-->

# Technical Details

## BLE Service

### Service UUID
`a1b2c3d4-1234-5678-abcd-ef0123456789`

### Characteristics

**Measurement Characteristic** (Read, Notify)
UUID: `a1b2c3d5-1234-5678-abcd-ef0123456789`
Data format (10 bytes):
- Bytes 0-1: Set current (µA, little-endian uint16)
- Bytes 2-3: Measured current (µA, little-endian uint16)
- Bytes 4-5: Battery voltage (mV, little-endian uint16)
- Bytes 6-7: Output voltage (mV, little-endian uint16)
- Bytes 8-9: Impedance (Ω, little-endian uint16, 0xFFFF if current < 10µA)

**Timer Characteristic** (Read, Write, Notify)
UUID: `a1b2c3d6-1234-5678-abcd-ef0123456789`

Write format (8 bytes) - to start/stop a session:
- Bytes 0-1: Target current (µA, little-endian uint16)
- Bytes 2-3: Total time (seconds, little-endian uint16)
- Bytes 4-5: Ramp up time (seconds, little-endian uint16)
- Bytes 6-7: Ramp down time (seconds, little-endian uint16)

Read/Notify format (8 bytes) - session status:
- Bytes 0-1: Target current (µA, little-endian uint16)
- Bytes 2-3: Time remaining (seconds, little-endian uint16)
- Bytes 4-5: Ramp up time (seconds, little-endian uint16)
- Bytes 6-7: Ramp down time (seconds, little-endian uint16)

To stop a session, write with totalTime=0 or targetCurrent=0.

## Hardware Constants

| Constant | Value | Description |
|----------|-------|-------------|
| ADC Reference | 3.3V | nRF52840 ADC reference voltage |
| ADC Resolution | 10-bit | 0-1023 range |
| PWM Resolution | 8-bit | 0-255 range |
| Sense Resistor | 1kΩ | Current measurement resistor |
| Output Voltage Divider | 5.3x | For measuring boosted output |
| Battery Voltage Divider | 5.3x | For battery monitoring |
| Max Current | 4000µA | Hardware-limited maximum |
| Test Current | 50µA | Applied when connected, not in session |

## Dickson Charge Pump

The firmware drives a 3-pin Dickson voltage multiplier using HwPWM3:
- D9 and D7: Normal phase (50% duty cycle)
- D8: Inverted phase (180° out of phase)
- High-drive output mode enabled for increased current capability
- Multiplier is enabled when BLE connected or session active