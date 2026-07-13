// devices_types_harness.cpp — off-hardware acceptance harness for ticket
// DB-001 (device-bus-tickets.md): proves every Devices reading/config type
// (source/devices/device_types.h, source/devices/device_config.h) is
// std::is_trivially_copyable AND std::is_standard_layout — the exact
// requirement the issue's "Concurrency contract" rule 2 depends on (a
// MeasurementRing<T> publish, landing in DB-002, is a plain struct
// store/copy, never a constructor/destructor call).
//
// Compiled with -DHOST_BUILD for consistency with every other tests/sim/
// unit harness (see test_nezha_flipflop.py's own docstring for the
// pattern this mirrors), though device_types.h/device_config.h need no
// HOST_BUILD guard themselves — they carry zero includes beyond <cstdint>,
// by construction of the isolation invariant this ticket also enforces
// (see test_devices_isolation.py).
//
// Pure static_assert harness: nothing to run at runtime except report
// success, since every check here is a compile-time property. Mirrors the
// hand-rolled PASS/FAIL-and-nonzero-exit shape of the other tests/sim/unit
// harnesses (e.g. motor_policy_harness.cpp) for consistency, even though a
// static_assert failure would already fail the COMPILE step before this
// binary ever runs.

#include "devices/device_config.h"
#include "devices/device_types.h"

#include <cstdio>
#include <type_traits>

namespace {

// CHECK_TRIVIAL(T) — static_assert T is both trivially copyable and
// standard layout. A single macro so every type below gets the identical
// pair of checks with a self-naming failure message.
#define CHECK_TRIVIAL(T)                                                    \
  static_assert(std::is_trivially_copyable<T>::value,                      \
                #T " must be trivially copyable");                          \
  static_assert(std::is_standard_layout<T>::value,                         \
                #T " must be standard layout")

using namespace Devices;

// --- Reading / value types (device_types.h) ---
CHECK_TRIVIAL(MotorReading);
CHECK_TRIVIAL(ColorReading);
CHECK_TRIVIAL(LineReading);
CHECK_TRIVIAL(PoseReading);
CHECK_TRIVIAL(Neutral);

// --- Config types (device_config.h) ---
CHECK_TRIVIAL(Opt<float>);
CHECK_TRIVIAL(Gains);
CHECK_TRIVIAL(MotorConfig);
CHECK_TRIVIAL(OtosConfig);
CHECK_TRIVIAL(ColorConfig);
CHECK_TRIVIAL(LineConfig);

#undef CHECK_TRIVIAL

}  // namespace

int main() {
  std::printf(
      "PASS: every Devices reading/config type is trivially_copyable and "
      "standard_layout\n");
  return 0;
}
