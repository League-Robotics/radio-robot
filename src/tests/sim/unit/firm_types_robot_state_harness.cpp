// firm_types_robot_state_harness.cpp -- off-hardware acceptance proof for
// sprint 124 ticket 007 (SUC-004), Types::RobotState
// (src/firm/types/robot_state.h).
//
// This harness's OWN include list is the compile-level dependency-free
// assertion: robot_state.h, plus only host-standard-library headers needed
// to exercise it (<cstdint>/<cstdio>/<cstring>/<type_traits>) -- no
// messages/, no app/, no config/, nothing else from src/firm or src/firm/motion.
// test_firm_types_robot_state.py compiles this file with `-I <repo>/src`
// ONLY (matching robot_state.h's own `#include "firm/types/robot_state.h"`-
// style qualified include convention) -- if robot_state.h ever grew a
// `messages/` or `app/` include, either the include would fail to resolve
// (those trees are not on this narrow include path) or, if it happened to
// resolve by accident, the module boundary this ticket exists to hold would
// already be broken; the grep-enforceable check
// (`grep -n "messages/\|msg::" src/firm/types/robot_state.h`) in the
// pytest wrapper is the second, complementary proof.
//
// Proves, per the ticket's own acceptance criteria:
//   1. robot_state.h compiles standalone (this file's own existence + the
//      pytest wrapper's narrow -I path).
//   2. Types::RobotState is trivially copyable (static_assert below).
//   3. Every one of Motion::StateEstimator::Input's former 16 flat fields
//      is representable, no information loss -- proven by round-tripping
//      distinct values through every section a copy touches (the "test
//      fixture" role from the issue: construct, fill, copy, assert).
//   4. RobotState::Wheel carries positionEpoch alongside
//      position/velocity/sampleTime/connected/cmdVelocity.
//
// Mirrors motion_stop_condition_harness.cpp's own hand-rolled
// PASS/FAIL/exit-nonzero shape (no gtest, no pytest-side C++ dependency).
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <type_traits>

#include "firm/types/robot_state.h"

namespace {

int g_failureCount = 0;

void checkTrue(bool condition, const char* what) {
  if (!condition) {
    ++g_failureCount;
    std::printf("  FAIL: %s\n", what);
  }
}

void checkFloatEq(float actual, float expected, const char* what) {
  if (actual != expected) {
    ++g_failureCount;
    std::printf("  FAIL: %s -- expected %g, got %g\n", what, static_cast<double>(expected),
                static_cast<double>(actual));
  }
}

void checkUintEq(uint32_t actual, uint32_t expected, const char* what) {
  if (actual != expected) {
    ++g_failureCount;
    std::printf("  FAIL: %s -- expected %u, got %u\n", what, static_cast<unsigned>(expected),
                static_cast<unsigned>(actual));
  }
}

}  // namespace

// Trivially copyable (issue: "plain fields, no pointers, no heap, no
// virtuals" -- tests copy it for golden comparisons). Also implies
// standard-layout-adjacent plain-data status; not itself asserted
// separately since trivially-copyable is the ticket's own stated bar.
static_assert(std::is_trivially_copyable_v<Types::RobotState>,
              "Types::RobotState must be trivially copyable (plain fields only)");

int main() {
  std::printf("--- Types::RobotState: dependency-free, trivially-copyable, no field loss\n");

  Types::RobotState state{};  // default-constructed: every field its own declared default

  // --- Fill every section a caller genuinely writes, per section, with
  // distinct values (never the same value twice, so a golden-copy bug that
  // swaps two same-typed fields is still caught). Mirrors the 16 fields
  // Motion::StateEstimator::Input used to carry flat, now spread across
  // wheelLeft/wheelRight/pose/otos -- see robot_state.h's own mapping
  // comment in each section. ---
  state.time.cycleStart = 1000;
  state.time.cycleBusy = 2000;
  state.time.cyclePeriod = 40000;

  state.wheelLeft.position = 100.0f;
  state.wheelLeft.velocity = 50.0f;
  state.wheelLeft.sampleTime = 1000;
  state.wheelLeft.connected = true;
  state.wheelLeft.positionEpoch = 3;
  // cmdVelocity -- DELETED (exploratory-kernel rewrite, 2026-08-15): the
  // commanded target lives inside Control::DifferentialDrive's own private
  // Command mailbox now, not on this blackboard.

  state.wheelRight.position = -40.0f;
  state.wheelRight.velocity = -20.0f;
  state.wheelRight.sampleTime = 1001;
  state.wheelRight.connected = false;
  state.wheelRight.positionEpoch = 5;

  state.otos.present = true;
  state.otos.connected = true;
  state.otos.x = 10.0f;
  state.otos.y = 20.0f;
  state.otos.heading = 0.5f;
  state.otos.v_x = 30.0f;
  state.otos.v_y = 40.0f;
  state.otos.omega = 0.75f;
  state.otos.sampleTime = 1002;

  state.perception.line = 0x01020304u;
  state.perception.color = 0x05060708u;
  state.perception.lineFresh = true;
  state.perception.colorFresh = false;

  state.pose.x = 11.0f;
  state.pose.y = 22.0f;
  state.pose.heading = 0.25f;
  state.pose.v_x = 33.0f;
  state.pose.v_y = 0.0f;
  state.pose.omega = 0.125f;

  // estimate -- DELETED (exploratory-kernel rewrite, 2026-08-15): fed the
  // long-gone Core::StateEstimator (deleted sprint 128 ticket 016);
  // nothing has written or read this block since.

  state.command.mode = Types::Mode::Velocity;
  state.command.moveActive = true;
  // command.v_x/omega -- DELETED: nothing read them; the kernel's own
  // Command mailbox is the one place "what is currently commanded" lives.

  state.health.i2cSafetyNetCount = 7;
  state.health.commsMalformedCount = 2;
  state.health.wedgeLatch = true;
  // health.moveTimeout/shapingDisabled -- DELETED: both were written only
  // by the now-deleted Core::RobotLoop::publishMoveResult()/
  // publishGotoResult(), which fed Motion::Planner/Motion::Navigator
  // fault state that no longer exists.

  // --- Golden copy: a plain `=` copy (the trivially-copyable contract in
  // action -- no custom copy constructor to get wrong). Every field below
  // must survive unchanged. ---
  Types::RobotState copy = state;

  checkUintEq(copy.time.cycleStart, 1000, "time.cycleStart");
  checkUintEq(copy.time.cycleBusy, 2000, "time.cycleBusy");
  checkUintEq(copy.time.cyclePeriod, 40000, "time.cyclePeriod");

  checkFloatEq(copy.wheelLeft.position, 100.0f, "wheelLeft.position");
  checkFloatEq(copy.wheelLeft.velocity, 50.0f, "wheelLeft.velocity");
  checkUintEq(copy.wheelLeft.sampleTime, 1000, "wheelLeft.sampleTime");
  checkTrue(copy.wheelLeft.connected, "wheelLeft.connected");
  checkUintEq(copy.wheelLeft.positionEpoch, 3, "wheelLeft.positionEpoch");

  checkFloatEq(copy.wheelRight.position, -40.0f, "wheelRight.position");
  checkFloatEq(copy.wheelRight.velocity, -20.0f, "wheelRight.velocity");
  checkUintEq(copy.wheelRight.sampleTime, 1001, "wheelRight.sampleTime");
  checkTrue(!copy.wheelRight.connected, "wheelRight.connected (false)");
  checkUintEq(copy.wheelRight.positionEpoch, 5, "wheelRight.positionEpoch");

  checkTrue(copy.otos.present, "otos.present");
  checkTrue(copy.otos.connected, "otos.connected");
  checkFloatEq(copy.otos.x, 10.0f, "otos.x");
  checkFloatEq(copy.otos.y, 20.0f, "otos.y");
  checkFloatEq(copy.otos.heading, 0.5f, "otos.heading");
  checkFloatEq(copy.otos.v_x, 30.0f, "otos.v_x");
  checkFloatEq(copy.otos.v_y, 40.0f, "otos.v_y");
  checkFloatEq(copy.otos.omega, 0.75f, "otos.omega");
  checkUintEq(copy.otos.sampleTime, 1002, "otos.sampleTime");

  checkUintEq(copy.perception.line, 0x01020304u, "perception.line");
  checkUintEq(copy.perception.color, 0x05060708u, "perception.color");
  checkTrue(copy.perception.lineFresh, "perception.lineFresh");
  checkTrue(!copy.perception.colorFresh, "perception.colorFresh (false)");

  checkFloatEq(copy.pose.x, 11.0f, "pose.x");
  checkFloatEq(copy.pose.y, 22.0f, "pose.y");
  checkFloatEq(copy.pose.heading, 0.25f, "pose.heading");
  checkFloatEq(copy.pose.v_x, 33.0f, "pose.v_x");
  checkFloatEq(copy.pose.v_y, 0.0f, "pose.v_y");
  checkFloatEq(copy.pose.omega, 0.125f, "pose.omega");

  // estimate -- DELETED, see the setup block's own comment above.

  checkTrue(copy.command.mode == Types::Mode::Velocity, "command.mode");
  checkTrue(copy.command.moveActive, "command.moveActive");
  // command.v_x/omega -- DELETED, see the setup block's own comment above.

  checkUintEq(copy.health.i2cSafetyNetCount, 7, "health.i2cSafetyNetCount");
  checkUintEq(copy.health.commsMalformedCount, 2, "health.commsMalformedCount");
  checkTrue(copy.health.wedgeLatch, "health.wedgeLatch");
  // health.moveTimeout/shapingDisabled -- DELETED, see the setup block's
  // own comment above.

  // The original is untouched by mutating the copy -- proves the copy is a
  // genuine independent value, not a reference/alias (byte-for-byte memcmp
  // against a second untouched copy, exercising the "no pointers, no heap"
  // claim at the byte level, not just field-by-field).
  Types::RobotState untouchedCopy = state;
  checkTrue(std::memcmp(&state, &untouchedCopy, sizeof(Types::RobotState)) == 0,
            "byte-for-byte copy equality (memcmp)");

  if (g_failureCount == 0) {
    std::printf("PASS\n");
    return 0;
  }
  std::printf("FAIL: %d check(s) failed\n", g_failureCount);
  return 1;
}
