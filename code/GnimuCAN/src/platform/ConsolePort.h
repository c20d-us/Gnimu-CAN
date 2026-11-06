// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

namespace gnimu {

// Opens the serial console (the C6's built-in USB serial port) so that output
// never blocks, and waits briefly for a host so boot messages aren't lost.
void consoleOpen();

}  // namespace gnimu
