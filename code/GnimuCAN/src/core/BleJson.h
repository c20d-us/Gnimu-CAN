// Gnimu-CAN - vehicle CAN telemetry over BLE
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "BuildInfo.h"
#include "TextOut.h"

namespace gnimu {

// The most a BLE attribute can hold; both documents are cut off here, never
// beyond.
constexpr size_t kBleJsonMaxBytes = 512;

// One fill mask in the schema: a block name and the mask of fields the device
// can provide in it.
struct FillMask {
  const char* block;
  uint32_t mask;
};

// The Schema characteristic (Gnimu protocol, docs/frame.md): spec version,
// product, frame rate, and a fill mask per block the device fills. Aux fields
// get an "aux" object once a vehicle profile defines one.
inline size_t formatSchemaJson(char* buf, size_t cap, uint8_t specVersion, const char* product,
                               uint32_t rateHz, const FillMask* fills, size_t fillCount) {
  detail::Out o{buf, cap, 0};
  if (cap) buf[0] = '\0';
  o.put("{\"v\":%u,\"d\":\"%s\",\"r\":%lu,\"fill\":{", static_cast<unsigned>(specVersion), product,
        static_cast<unsigned long>(rateHz));
  for (size_t i = 0; i < fillCount; i++)
    o.put("%s\"%s\":%lu", i ? "," : "", fills[i].block, static_cast<unsigned long>(fills[i].mask));
  o.put("}}");
  return o.len;
}

// One cfg entry in Device info: a key and its value, already formatted.
// Strings are quoted; numbers and booleans aren't.
struct CfgItem {
  const char* key;
  const char* value;
  bool quoted;
};

// The Device info characteristic: the build and the current settings.
inline size_t formatDeviceInfoJson(char* buf, size_t cap, const BuildInfo& b, const CfgItem* cfg,
                                   size_t cfgCount) {
  detail::Out o{buf, cap, 0};
  if (cap) buf[0] = '\0';
  o.put("{\"v\":1,\"build\":{\"rev\":\"%s\",\"utc\":\"%s\",\"target\":\"%s\"},\"cfg\":{", b.revision,
        b.buildUtc, b.target);
  for (size_t i = 0; i < cfgCount; i++)
    o.put(cfg[i].quoted ? "%s\"%s\":\"%s\"" : "%s\"%s\":%s", i ? "," : "", cfg[i].key, cfg[i].value);
  o.put("}}");
  return o.len;
}

}  // namespace gnimu
