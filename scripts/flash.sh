#!/usr/bin/env bash
# Gnimu-CAN - vehicle CAN telemetry over BLE
# Copyright (C) 2026 Chris Halstead
# SPDX-License-Identifier: GPL-3.0-or-later

# Usage: scripts/flash.sh [--build] [--monitor] [--port <port>]
#
# Uploads the firmware scripts/build.sh last built, from build/gnimucan. It
# doesn't build unless asked with --build, which runs build.sh first and stops,
# flashing nothing, if the build fails. Either way, what's flashed is exactly
# what was built, stamp and all. --monitor opens the console afterwards.
#
# Without --port, uses the one connected Espressif USB device (USB vendor ID
# 0x303A, which the XIAO ESP32C6's built-in USB reports). With a USB-C host,
# connect USB before powering the board from 12V, or it won't enumerate (see
# docs/architecture.md, Bench rig).
set -euo pipefail
cd "$(dirname "$0")/.."

fqbn="esp32:esp32:XIAO_ESP32C6"
dir="build/gnimucan"
baud=115200

usage() {
  echo "Usage: $0 [--build] [--monitor] [--port <port>]" >&2
  exit 2
}

build=0
monitor=0
port=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --build) build=1 ;;
    --monitor) monitor=1 ;;
    --port) [[ $# -ge 2 ]] || usage; port="$2"; shift ;;
    *) usage ;;
  esac
  shift
done

if [[ $build -eq 1 ]]; then
  scripts/build.sh || { echo "build failed; nothing flashed" >&2; exit 1; }
fi
if [[ ! -f "$dir/GnimuCAN.ino.bin" ]]; then
  echo "$dir/GnimuCAN.ino.bin not found; run scripts/build.sh first" >&2
  exit 1
fi

if [[ -z "$port" ]]; then
  port="$(python3 -I -c '
import json, sys
data = json.load(sys.stdin)
for entry in data.get("detected_ports", []):
    port = entry.get("port", {})
    if port.get("properties", {}).get("vid", "").lower() == "0x303a":
        print(port.get("address", ""))
' <<<"$(arduino-cli board list --format json)")"
  if [[ "$(grep -c . <<<"$port" || true)" -ne 1 ]]; then
    echo "Found $(grep -c . <<<"$port" || true) connected Espressif USB devices; name one with --port:" >&2
    arduino-cli board list >&2
    exit 1
  fi
fi

echo "Flashing $dir to $port"
arduino-cli upload --fqbn "$fqbn" --port "$port" --input-dir "$dir"
echo "gnimucan: flashed"

if [[ $monitor -eq 1 ]]; then
  echo "Opening console on $port at $baud baud (Ctrl-C to quit)"
  exec arduino-cli monitor --port "$port" --config "baudrate=$baud"
fi
