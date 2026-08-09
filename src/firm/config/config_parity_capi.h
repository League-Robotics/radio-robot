// config_parity_capi.h -- generated-parity guard capi export (sprint 132
// ticket 003, "the-configuration-object.md"). Mirrors
// src/firm/motion/planner/capi.cpp's plannerStructSizes()/plannerLimitsOffsets()
// pattern: a flat, extern "C", ctypes-loadable surface a Python harness
// (src/tests/unit/test_config_parity_capi.py) walks to prove the generated
// Config::Robot group structs (src/firm/messages/robot_config.h, emitted by
// scripts/gen_messages.py from protos/robot_config.proto) and the generated
// pydantic model (src/host/robot_radio/config/robot_config_generated.py)
// have not structurally drifted from each other. This is the mechanical
// guarantee that replaces check_config_sync.py's hand-curated allowlist
// (deleted next, ticket 004) -- byte-for-byte, not by lint.
#pragma once

#include <cstdint>

extern "C" {

// One id per Config::Robot group struct this file describes, in the SAME
// order src/firm/messages/robot_config.h declares the structs (also the
// order test_gen_messages_robot_config_emission.py's own _ROBOT_CONFIG_
// GROUPS tuple uses). Deliberately a PLAIN uint32_t at the ABI boundary
// below (not this enum type) -- ctypes has no notion of a scoped C++ enum,
// and the extern "C" boundary should carry only types with an unambiguous
// C layout.
enum class ConfigParityGroup : uint32_t {
  Geometry = 0,
  Motors,
  Drive,
  WheelControl,
  Planner,
  PlannerShaper,  // 132-017: split out of Planner -- see robot_config.proto
  Otos,
  Estimator,
  Count,  // sentinel -- the number of groups this file knows about
};

// Struct-size guard, mirroring plannerStructSizes() (capi.cpp:69) -- one
// entry per group, in ConfigParityGroup order. Writes
// min(count, ConfigParityGroup::Count) entries into `out` and returns the
// true group count.
uint32_t configParityStructSizes(uint32_t* out, uint32_t count);

// Per-field OFFSET guard, mirroring plannerLimitsOffsets() (capi.cpp:93) --
// keyed by group id (a ConfigParityGroup value, passed as plain uint32_t)
// since this schema has 7 groups worth checking, not planner's 1. Writes
// min(count, that group's own field count) entries into `out` and returns
// the group's true field count. An unrecognized `group` returns 0 and
// writes nothing.
uint32_t configParityFieldOffsets(uint32_t group, uint32_t* out, uint32_t count);

}  // extern "C"
