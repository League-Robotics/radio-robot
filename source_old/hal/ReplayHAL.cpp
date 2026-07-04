// ReplayHAL.cpp — stub translation unit for ROBOT_RUN_MODE=REPLAY (039-005).
//
// Phase F will implement TLM-frame replay (feeding recorded sensor data back
// through the capability interfaces). For now ReplayHAL is a header-only no-op
// HAL (see ReplayHAL.h); this .cpp exists so the REPLAY source set has a
// translation unit to compile and link, and to anchor the RobotMode::REPLAY
// build mode. It is compiled only when ROBOT_RUN_MODE=REPLAY in CMake and is
// NOT wired for real use yet.
#include "ReplayHAL.h"

// RobotMode — build-target run mode (mirrors the CMake ROBOT_RUN_MODE variable:
// REAL | SIM | REPLAY). Declared here alongside the REPLAY stub; Phase F will
// promote it to a shared header when the replay path is wired.
enum class RobotMode { REAL, SIM, REPLAY };

namespace {
// Anchor the REPLAY mode so the linker has a defined symbol in this TU and the
// enum value is exercised at compile time. No runtime effect.
constexpr RobotMode kReplayHalMode = RobotMode::REPLAY;
static_assert(kReplayHalMode == RobotMode::REPLAY, "ReplayHAL is the REPLAY-mode HAL");
}  // namespace
