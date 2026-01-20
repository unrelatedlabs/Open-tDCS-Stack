#!/bin/bash
# OpenTDCS Build Script
# Copyright (C) 2024-2026 Peter Kuhar and OpenTDCS Contributors
# SPDX-License-Identifier: GPL-3.0-or-later

set -e

# Change to script directory
cd "$(dirname "$0")"

echo "Building Docker image..."
docker build -t opentdcs-builder .

echo "Building firmware..."
docker run --rm \
  -v "$(pwd)":/firmware \
  opentdcs-builder

echo "Build complete!"
echo "Output files in: $(pwd)/build/"

# Find and display the UF2 file
UF2_FILE=$(find build -name "*.uf2" 2>/dev/null | head -1)
if [ -n "$UF2_FILE" ]; then
  echo "UF2 file: $UF2_FILE"
else
  echo "Note: UF2 file not found in build output"
fi
