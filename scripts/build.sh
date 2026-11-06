#!/usr/bin/env bash
# Gnimu-CAN - vehicle CAN telemetry over BLE
# Copyright (C) 2026 Chris Halstead
# SPDX-License-Identifier: GPL-3.0-or-later

# Usage: scripts/build.sh
#
# Builds the firmware for the XIAO ESP32C6 into build/gnimucan, stamped with
# the version, git revision and build time that the boot log, V and Device info
# report.
#
# Fails on any compiler warning in the project's own sources (code/GnimuCAN).
# Warnings from the core and its libraries are shown but don't fail: they
# aren't ours to fix, and Arduino has no sketch-only -Werror.
set -euo pipefail
cd "$(dirname "$0")/.."

[[ $# -eq 0 ]] || { echo "Usage: $0" >&2; exit 2; }

fqbn="esp32:esp32:XIAO_ESP32C6"
out="gnimucan"

command -v arduino-cli >/dev/null 2>&1 || {
  echo "arduino-cli not found; run scripts/setup.sh" >&2
  exit 2
}

revision="$(git rev-parse --short=12 HEAD 2>/dev/null || printf 'unknown')"
version="$(git describe --tags --always 2>/dev/null || printf '0.0.0-dev')"
build_utc="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

extra_flags="-DGNIMU_VERSION=\"${version}\" -DGNIMU_REVISION=\"${revision}\""
extra_flags+=" -DGNIMU_BUILD_UTC=\"${build_utc}\""

sketch="$PWD/code/GnimuCAN"
log="$(mktemp)"
trap 'rm -f "$log"' EXIT

# --clean: every source is recompiled, so every warning is printed. An
# incremental build skips unchanged files, and their warnings with them.
arduino-cli compile \
  --clean \
  --build-path "build/work/$out" \
  --fqbn "$fqbn" \
  --warnings all \
  --output-dir "build/$out" \
  --build-property "compiler.cpp.extra_flags=${extra_flags}" \
  "$sketch" 2>&1 | tee "$log"

# Our warnings, found by the sketch's own path, which arduino-cli reports for
# every project source. grep finding none is the pass.
if ours="$(grep -F "$sketch/" "$log" | grep -F ': warning:')"; then
  echo >&2
  echo "$out: compiler warnings in project sources:" >&2
  echo "$ours" >&2
  exit 1
fi
echo "$out: no warnings in project sources"
