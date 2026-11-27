#!/usr/bin/env python3
# Gnimu-CAN - vehicle CAN telemetry over BLE
# Copyright (C) 2026 Chris Halstead
# SPDX-License-Identifier: GPL-3.0-or-later

"""Checks core/Obd2Pids.h against the Gnimu frame spec's Mode 01 PID column.

Every decoder must fill a field whose spec entry names the same PID. Run by
tests/run.sh; skipped when the Gnimu-protocol repository isn't checked out
beside this one.
"""

import json
import pathlib
import re
import sys

here = pathlib.Path(__file__).resolve().parent
header = here.parent / "code/GnimuCAN/src/core/Obd2Pids.h"
spec = here.parent.parent / "Gnimu-protocol/spec/frame-v1.json"

if not spec.exists():
    print("check_obd_spec: skipped (no Gnimu-protocol checkout beside this repo)")
    sys.exit(0)

def snake(name):
    return re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()

spec_pids = {}
for block in json.loads(spec.read_text())["blocks"]:
    for field in block.get("fields", []):
        if "obd2_pid" in field and "key" in field:
            spec_pids[field["key"]] = field["obd2_pid"].upper()

# The main table only: helper decoders (kHelperDecoders) have no field of their own.
text = header.read_text()
table = text[text.index("kPidDecoders[] = {"):text.index("kHelperDecoders[]")]
decoders = re.findall(r"\{frame::Field::(\w+),\s*0x([0-9A-Fa-f]{2}),.*?,\s*(0x[0-9A-Fa-f]{2}|0)\},", table)
if not decoders:
    sys.exit("check_obd_spec: no decoders found in " + str(header))

failed = False
for field, pid, minus in decoders:
    key = snake(field)
    want = spec_pids.get(key)
    have = "0X" + pid.upper()
    if minus != "0":
        have += "-" + minus.upper()  # a difference, written as the spec does: 0x0B-0x33
    if want != have:
        print(f"check_obd_spec: {key}: decoder PID {have}, spec says {want}")
        failed = True
if failed:
    sys.exit(1)
print(f"check_obd_spec: ok ({len(decoders)} decoders match the spec)")
