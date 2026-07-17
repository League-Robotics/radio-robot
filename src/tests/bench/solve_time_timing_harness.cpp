// solve_time_timing_harness.cpp -- host-side solve-time sample generator for
// solve_time_characterize.py's HOST fallback (sprint 109 ticket 001).
//
// Calls Motion::JerkTrajectory::solveToRest()/solveToState() many times over
// a spread of representative inputs (mirroring the D/RT characterization
// this ticket's on-target predecessor exercised) and prints one elapsed
// nanosecond count per call to stdout, one per line, prefixed with the
// channel label -- the Python driver reads these lines and computes
// percentiles. Uses std::chrono::steady_clock (host wall time), NOT the
// Cortex-M4 DWT cycle counter the on-target script uses -- see this
// ticket's completion notes for why the on-target measurement is not
// currently possible (no compiled call site to break on) and the caveat
// this substitution carries (host CPU timing, not ARM).
#include <chrono>
#include <cstdio>

#include "messages/planner.h"
#include "motion/jerk_trajectory.h"

namespace {

msg::PlannerConfig makeLinearConfig() {
  msg::PlannerConfig config;
  config.a_max = 800.0f;
  config.a_decel = 800.0f;
  config.v_body_max = 620.0f;
  config.j_max = 0.0f;
  return config;
}

msg::PlannerConfig makeRotationalConfig() {
  msg::PlannerConfig config;
  config.yaw_acc_max = 500.0f;
  config.yaw_rate_max = 3.0f;
  config.yaw_jerk_max = 0.0f;
  return config;
}

}  // namespace

int main() {
  constexpr int kIterations = 2000;

  // Linear channel: mirrors the old D_linear characterization (a
  // solveToRest() at a representative distance/speed).
  {
    Motion::JerkTrajectory channel;
    channel.configure(makeLinearConfig(), /*isRotational=*/false);
    for (int i = 0; i < kIterations; ++i) {
      channel.reset();
      auto t1 = std::chrono::steady_clock::now();
      channel.solveToRest(150.0f, 400.0f);
      auto t2 = std::chrono::steady_clock::now();
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
      std::printf("D_linear %lld\n", static_cast<long long>(ns));
    }
  }

  // Rotational channel: mirrors the old RT_rotational characterization.
  {
    Motion::JerkTrajectory channel;
    channel.configure(makeRotationalConfig(), /*isRotational=*/true);
    for (int i = 0; i < kIterations; ++i) {
      channel.reset();
      auto t1 = std::chrono::steady_clock::now();
      channel.solveToRest(1.57f, 3.0f);
      auto t2 = std::chrono::steady_clock::now();
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
      std::printf("RT_rotational %lld\n", static_cast<long long>(ns));
    }
  }

  // solveToState (109-001's new entry point): boundary-velocity-carry solve,
  // linear channel.
  {
    Motion::JerkTrajectory channel;
    channel.configure(makeLinearConfig(), /*isRotational=*/false);
    for (int i = 0; i < kIterations; ++i) {
      channel.reset();
      auto t1 = std::chrono::steady_clock::now();
      channel.solveToState(150.0f, 200.0f, 400.0f);
      auto t2 = std::chrono::steady_clock::now();
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
      std::printf("D_solveToState %lld\n", static_cast<long long>(ns));
    }
  }

  return 0;
}
