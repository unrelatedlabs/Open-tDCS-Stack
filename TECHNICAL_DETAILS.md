

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
- Bytes 8-9: Impedance (Ω, little-endian uint16)

**Timer Characteristic** (Read, Write, Notify)
UUID: `a1b2c3d6-1234-5678-abcd-ef0123456789`
Data format (8 bytes):
- Bytes 0-1: Target current (µA, little-endian uint16)
- Bytes 2-3: Time remaining (seconds, little-endian uint16)
- Bytes 4-5: Ramp up time (seconds, little-endian uint16)
- Bytes 6-7: Ramp down time (seconds, little-endian uint16)

