// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "RunStateMachine.h"

namespace gnimu {

// Why the chip last started, as its hardware reports it. platform/esp32c6/
// ResetInfo.cpp maps the chip's reason onto these.
enum class ResetCause : uint8_t {
  PowerOn,
  Brownout,
  Watchdog,
  Software,  // our own restarts, and an upload's reset into the bootloader
  Pin,
  Panic,  // crash
  Usb,    // reset over USB (by an upload or a terminal)
  WakeDeepSleep,
  Unknown,
};

inline const char* resetCauseText(ResetCause c) {
  switch (c) {
    case ResetCause::PowerOn:
      return "power-on";
    case ResetCause::Brownout:
      return "brownout";
    case ResetCause::Watchdog:
      return "watchdog";
    case ResetCause::Software:
      return "software";
    case ResetCause::Pin:
      return "reset pin";
    case ResetCause::Panic:
      return "crash";
    case ResetCause::Usb:
      return "USB";
    case ResetCause::WakeDeepSleep:
      return "wake from deep sleep";
    case ResetCause::Unknown:
      break;
  }
  return "unknown";
}

// The byte kept across a restart we requested: 0 = none, else the
// RestartReason + 1.
constexpr uint8_t restartCode(RestartReason r) { return static_cast<uint8_t>(static_cast<uint8_t>(r) + 1); }

// How this boot started: the hardware cause, plus our own reason when the
// reset was a restart we requested.
struct ResetRecord {
  ResetCause cause;
  bool restartKnown;
  RestartReason restart;  // restartKnown only
};

// `code` is the kept byte as read back at boot. It only counts after a
// software reset: it also survives watchdog and pin resets, and holds garbage
// after power-on.
inline ResetRecord makeResetRecord(ResetCause cause, uint8_t code) {
  ResetRecord r = {cause, false, RestartReason::Requested};
  if (cause == ResetCause::Software && code >= 1 && code <= kRestartReasonCount) {
    r.restartKnown = true;
    r.restart = static_cast<RestartReason>(code - 1);
  }
  return r;
}

// e.g. "watchdog", or "software (Restart requested)"; truncated, never
// overrun.
inline void formatReset(char* buf, size_t cap, const ResetRecord& r) {
  if (r.restartKnown) {
    snprintf(buf, cap, "%s (%s)", resetCauseText(r.cause), restartReasonText(r.restart));
  } else {
    snprintf(buf, cap, "%s", resetCauseText(r.cause));
  }
}

}  // namespace gnimu
