# OpenTDCS Build Container
# Copyright (C) 2024-2026 Peter Kuhar and OpenTDCS Contributors
# SPDX-License-Identifier: GPL-3.0-or-later

FROM ubuntu:22.04

# Install dependencies
RUN apt-get update && apt-get install -y \
    curl \
    python3 \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/* \
    && ln -s /usr/bin/python3 /usr/bin/python

# Install Arduino CLI
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
ENV PATH="/root/bin:${PATH}"

# Install adafruit-nrfutil for UF2 generation
RUN pip3 install --upgrade adafruit-nrfutil

# Configure Arduino CLI
RUN arduino-cli config init
RUN arduino-cli config add board_manager.additional_urls https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json

# Install board support
RUN arduino-cli core update-index
RUN arduino-cli core install Seeeduino:nrf52

# Set working directory
WORKDIR /firmware

# Default command - compile and convert to UF2
CMD ["sh", "-c", "\
     arduino-cli compile \
     --fqbn Seeeduino:nrf52:xiaonRF52840Sense \
     --output-dir /firmware/build \
     /firmware/opentdcs/opentdcs.ino && \
     python3 /root/.arduino15/packages/Seeeduino/hardware/nrf52/1.1.12/tools/uf2conv/uf2conv.py \
     -c -f 0xADA52840 \
     -o /firmware/build/opentdcs.ino.uf2 \
     /firmware/build/opentdcs.ino.hex"]
