#!/usr/bin/env bash
# Gnimu-CAN - vehicle CAN telemetry over BLE
# Copyright (C) 2026 Chris Halstead
# SPDX-License-Identifier: GPL-3.0-or-later

# Usage: scripts/tool.sh <name> [--flash] [--monitor] [--port <port>]
#        scripts/tool.sh --list
#
# Builds the diagnostic sketch tools/<name> for the XIAO ESP32C6 into
# build/tools/<name>, then optionally flashes it and opens the serial monitor.
# --monitor on its own skips the build and just opens the console.
#
# Builds with all warnings on and fails on any warning in the tool's own
# source. Warnings from the core and its libraries are shown but don't fail.
#
# Without --port, uses the one connected Espressif USB device (USB vendor ID
# 0x303A, which the XIAO ESP32C6's built-in USB reports). With none or several
# connected, it lists what it found and asks for --port.
set -euo pipefail
cd "$(dirname "$0")/.."

fqbn="esp32:esp32:XIAO_ESP32C6"
baud=115200

usage() {
  echo "Usage: $0 <name> [--flash] [--monitor] [--port <port>]" >&2
  echo "       $0 --list" >&2
  exit 2
}

list_tools() {
  for d in tools/*/; do
    name="$(basename "$d")"
    [[ -f "$d$name.ino" ]] && echo "$name"
  done
}

name=""
flash=0
monitor=0
port=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --list) list_tools; exit 0 ;;
    --flash) flash=1 ;;
    --monitor) monitor=1 ;;
    --port) [[ $# -ge 2 ]] || usage; port="$2"; shift ;;
    -*) usage ;;
    *) [[ -z "$name" ]] || usage; name="$1" ;;
  esac
  shift
done
[[ -n "$name" ]] || usage

sketch="tools/$name"
if [[ ! -f "$sketch/$name.ino" ]]; then
  echo "No tool '$name'. Available:" >&2
  list_tools | sed 's/^/  /' >&2
  exit 2
fi

command -v arduino-cli >/dev/null 2>&1 || {
  echo "arduino-cli not found; on macOS run: brew install arduino-cli" >&2
  exit 2
}

# Prints the port of the one connected Espressif USB device, or explains and
# fails.
find_port() {
  local json ports count
  json="$(arduino-cli board list --format json)"
  ports="$(python3 -I -c '
import json, sys
data = json.load(sys.stdin)
for entry in data.get("detected_ports", []):
    port = entry.get("port", {})
    vid = port.get("properties", {}).get("vid", "")
    if vid.lower() == "0x303a":
        print(port.get("address", ""))
' <<<"$json")"
  count="$(grep -c . <<<"$ports" || true)"
  if [[ "$count" -ne 1 ]]; then
    echo "Found $count connected Espressif USB devices; name one with --port:" >&2
    arduino-cli board list >&2
    return 1
  fi
  echo "$ports"
}

build_dir="build/tools/$name"

# --monitor alone just opens the console.
if [[ $flash -eq 0 && $monitor -eq 1 ]]; then
  [[ -n "$port" ]] || port="$(find_port)"
  exec arduino-cli monitor --port "$port" --config "baudrate=$baud"
fi

echo "Building $sketch"
log="$(mktemp)"
trap 'rm -f "$log"' EXIT
if ! arduino-cli compile --fqbn "$fqbn" --warnings all --build-path "$build_dir" "$sketch" 2>&1 | tee "$log"; then
  echo "$name: build failed" >&2
  exit 1
fi
if grep -E "^[^ ]*$sketch/[^:]+:[0-9]+:[0-9]+: warning:" "$log" >/dev/null; then
  echo "$name: warnings in $sketch; fix them" >&2
  exit 1
fi
echo "$name: built"

if [[ $flash -eq 1 ]]; then
  [[ -n "$port" ]] || port="$(find_port)"
  echo "Flashing $name to $port"
  arduino-cli upload --fqbn "$fqbn" --port "$port" --input-dir "$build_dir" "$sketch"
  echo "$name: flashed"
fi

if [[ $monitor -eq 1 ]]; then
  echo "Opening console on $port at $baud baud (Ctrl-C to quit)"
  exec arduino-cli monitor --port "$port" --config "baudrate=$baud"
fi
