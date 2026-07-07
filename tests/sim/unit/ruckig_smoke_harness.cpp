// ruckig_smoke_harness.cpp — first-pass proof that the vendored Ruckig
// (libraries/ruckig, MIT community version) compiles under the firmware's
// EXACT build constraints (C++20, -fno-exceptions, -fno-rtti, compile-time
// DoF so no heap) and produces the trajectory shape we need: accelerate/cruise
// then jerk-limited DECELERATE to REST at the target, never crossing zero into
// a reverse spin. That "arrive at rest, no reverse" property is exactly what
// fixes the terminal reverse-spin (see
// clasi/issues/planner-motion-planning-via-vendored-ruckig.md).
//
// Plain C++ program, hand-rolled assertions (mirrors velocity_pid_harness.cpp).
// Compiled + run by test_ruckig_smoke.py. Uses Ruckig<1> (throw_error defaults
// to false -> validation returns Result codes, no exceptions thrown).
#include <cmath>
#include <cstdio>

#include "ruckig/ruckig.hpp"

using namespace ruckig;

namespace {
int g_fail = 0;
void check(bool ok, const char* what) {
  if (!ok) { ++g_fail; std::printf("  FAIL: %s\n", what); }
}
}  // namespace

int main() {
  Ruckig<1> otg{0.02};   // [s] 20 ms control cycle (offline calculate() ignores it)

  InputParameter<1> input;
  input.current_position = {0.0};
  input.current_velocity = {200.0};      // [mm/s] already moving forward at plan time
  input.current_acceleration = {0.0};
  input.target_position = {1000.0};      // [mm]
  input.target_velocity = {0.0};         // arrive at REST -> the stop-at-zero invariant
  input.target_acceleration = {0.0};
  input.max_velocity = {250.0};          // [mm/s]
  input.max_acceleration = {800.0};      // [mm/s^2]
  input.max_jerk = {4000.0};             // [mm/s^3]

  Trajectory<1> traj;
  Result r = otg.calculate(input, traj);
  check(r == Result::Working, "calculate() returns Working (no error code)");

  const double dur = traj.get_duration();
  check(dur > 0.0, "trajectory duration > 0");

  // Sample the whole trajectory: velocity must never go negative (no reverse),
  // must respect the velocity limit, and must arrive at rest exactly on target.
  double minVel = 1e9, maxVel = -1e9;
  const int N = 400;
  for (int i = 0; i <= N; ++i) {
    const double t = dur * i / N;
    double p, v, a;
    traj.at_time(t, p, v, a);
    if (v < minVel) minVel = v;
    if (v > maxVel) maxVel = v;
  }
  double endPos, endVel, endAcc;
  traj.at_time(dur, endPos, endVel, endAcc);
  std::printf("  duration=%.3fs  minVel=%.2f  maxVel=%.2f  endPos=%.2f  endVel=%.4f\n",
              dur, minVel, maxVel, endPos, endVel);

  check(minVel >= -0.5, "velocity NEVER goes negative across the trajectory (no reverse spin)");
  check(maxVel <= 250.0 + 0.5, "velocity respects max_velocity (250)");
  check(std::fabs(endVel) < 0.5, "arrives at REST (final velocity ~ 0)");
  check(std::fabs(endPos - 1000.0) < 1.0, "arrives AT the target position (1000)");

  if (g_fail == 0) {
    std::printf("OK: vendored Ruckig compiled (C++20/-fno-exceptions/-fno-rtti) and produced a rest-terminating, no-reverse trajectory\n");
    return 0;
  }
  std::printf("FAILED: %d ruckig smoke assertion(s)\n", g_fail);
  return 1;
}
