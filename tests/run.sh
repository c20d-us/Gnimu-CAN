#!/usr/bin/env bash
# Gnimu-CAN - vehicle CAN telemetry over BLE
# Copyright (C) 2026 Chris Halstead
# SPDX-License-Identifier: GPL-3.0-or-later

# Builds and runs every host test: tests/test_*.cpp against GnimuCAN/src/core.
# Each test runs twice: a plain build (closest to the firmware's codegen) and
# one under AddressSanitizer + UndefinedBehaviorSanitizer, where the first
# finding fails the run. gnu++11 matches the oldest standard the Gnimu family
# builds with, so code shared with Gnimu stays portable.
set -euo pipefail
cd "$(dirname "$0")"
out="${TMPDIR:-/tmp}/gnimucan-tests"
mkdir -p "$out"
flags=(-std=gnu++11 -Wall -Wextra -Werror -O1)
sanitize=(-g -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer)
for t in test_*.cpp; do
  name="${t%.cpp}"
  g++ "${flags[@]}" "$t" -o "$out/$name"
  "$out/$name"
  g++ "${flags[@]}" "${sanitize[@]}" "$t" -o "$out/$name-san"
  "$out/$name-san" > /dev/null
done
echo "all suites passed (plain and sanitized)"
