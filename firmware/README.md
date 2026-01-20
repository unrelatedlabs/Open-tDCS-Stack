<!--
SPDX-License-Identifier: CC-BY-SA-4.0
Copyright (C) 2024-2026 Peter Kuhar and OpenTDCS Contributors
-->

# Open-tDCS-Stack Firmware

Firmware for the Seeed Studio XIAO BLE nRF52840 microcontroller.

## Building with Docker

The easiest way to build the firmware is using Docker:

```bash
./build.sh
```

This will:
1. Build the Docker image with Arduino CLI and required libraries
2. Compile the firmware
3. Generate a UF2 file in the `build/` directory

## Output

The build creates a UF2 file that can be flashed to the XIAO BLE nRF52840 by:
1. Double-clicking the reset button on the board
2. A USB drive will appear
3. Copy the UF2 file to the drive
4. The board will automatically flash and reboot

## Manual Building

If you prefer to build manually without Docker:

1. Install [Arduino CLI](https://arduino.github.io/arduino-cli/)
2. Add Seeed Studio board support:
   ```bash
   arduino-cli config add board_manager.additional_urls https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
   arduino-cli core update-index
   arduino-cli core install Seeeduino:nrf52
   ```
3. Compile:
   ```bash
   arduino-cli compile --fqbn Seeeduino:nrf52:xiaonRF52840Sense opentdcs/opentdcs.ino
   ```
4. Convert to UF2 (optional, if not auto-generated):
   ```bash
   python3 ~/.arduino15/packages/Seeeduino/hardware/nrf52/1.1.12/tools/uf2conv/uf2conv.py \
     -c -f 0xADA52840 -o opentdcs.ino.uf2 opentdcs.ino.hex
   ```

## Requirements

- Docker (for containerized builds)
- OR Arduino CLI + Seeeduino nRF52 BSP (for manual builds)
