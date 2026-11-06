#!/usr/bin/env bash
# Gnimu-CAN - vehicle CAN telemetry over BLE
# Copyright (C) 2026 Chris Halstead
# SPDX-License-Identifier: GPL-3.0-or-later

# Usage: scripts/setup.sh
#
# Prepares a machine to build with arduino-cli: checks the tools the scripts
# rely on, then installs the ESP32 board package at the version the firmware
# is built and tested against. Safe to run again. CAN (TWAI) and BLE (NimBLE)
# come with the board package; there are no libraries to install.
set -euo pipefail
cd "$(dirname "$0")/.."

esp32_core="esp32:esp32@3.3.12"

problems=0
problem() {
  echo "  - $1" >&2
  problems=1
}

echo "Checking tools"
for tool in git g++ python3; do
  command -v "$tool" >/dev/null 2>&1 || problem "$tool not found; on macOS run: xcode-select --install"
done
command -v arduino-cli >/dev/null 2>&1 || problem "arduino-cli not found; on macOS run: brew install arduino-cli"
if [[ $problems -ne 0 ]]; then
  echo "Fix the above and run $0 again" >&2
  exit 1
fi

echo "Installing the board package"
arduino-cli core update-index
arduino-cli core install "$esp32_core"

echo "Setup complete; build with scripts/build.sh"
