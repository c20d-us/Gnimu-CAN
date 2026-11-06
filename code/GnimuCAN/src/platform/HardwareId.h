// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

namespace gnimu {

// A factory-programmed, per-chip identifier: not the BLE MAC, and not
// something firmware can set.
uint64_t hardwareId();

}  // namespace gnimu
