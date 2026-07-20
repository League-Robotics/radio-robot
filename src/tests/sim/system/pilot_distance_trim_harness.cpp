// pilot_distance_trim_harness.cpp -- ticket 112-003's own targeted
// clamp-authority check: the 087-009 non-regression guardrail (SUC-007)
// for App::Pilot's new bounded linear position-feedback trim.
//
// The concern this file exists to verify (sprint 112 Architecture
// "Guardrails/SUC-007" Main Flow step 3): the trim's clamp ceiling
// (App::kDistanceTrimCeiling, pilot.h) must keep the correction it adds to
// the commanded velocity to a small, bounded "residual-error nudge," even
// when the linear channel's own since-activation reference/measured pair
// (Motion::Executor::Twist::sRef/sMeas) diverges by a large amount -- NEVER
// a magnitude that could look like the kind of solve-side reversal
// 087-009/d-drive-terminal-instability.md documents. The trim must also
// never itself trigger a JerkTrajectory re-solve (no solveToRest/
// solveToState/solveToVelocity/retarget/reanchor call reads sRef/sMeas --
// grep-verifiable directly against pilot.cpp/executor.cpp, this file's own
// job is the BEHAVIORAL half of that guarantee).
//
// Method: drive a real App::Pilot/Motion::Executor/App::Drive graph
// (TestSim::SimHarness) through a plain straight DISTANCE command (no
// heading content -- deltaHeading=0 -- so the heading PD never mixes into
// the signal under test), let it reach cruise (acceleration ~0, so
// App::Drive's own actuation-lag feedforward term, actuation_lag*aRef, is
// itself ~0 and does not confound the comparison below), then FREEZE both
// wheel encoders (TestSim::WheelPlant's own encoder-wedge fault knob,
// SimPlant::freezePosition()) so the MEASURED path
// (Executor::measuredPathSinceActivation_, i.e. Twist::sMeas) stops
// advancing while the PLANNED reference (Twist::sRef) keeps advancing from
// the already-solved trajectory, completely independent of any
// measurement -- exactly the divergence the trim's clamp exists to bound.
// Held only briefly (well under Motion::kDivergenceReanchorLinearMm's own
// 40mm gross-divergence reanchor threshold, executor.cpp) so the
// PRE-EXISTING, unrelated 40mm reanchor mechanism never fires and
// confounds the comparison with a genuine re-solve.
//
// Verifies, every frozen cycle: `|driveTargetVelLeft - plannedRefLeft| <=
// kDistanceTrimCeiling + epsilon` (and the same for right) -- the STAGED
// command (post-trim, post-FF) never departs from the PLANNED reference
// (pre-trim, pre-FF, App::Pilot::refLeft/refRight()) by more than the
// clamp allows -- and that the PLANNED reference itself stays flat
// throughout the freeze (proving the divergence never fed back into a
// re-solve -- had checkDivergence()'s pre-existing 40mm reanchor tier
// fired, or had this ticket's own trim wrongly retargeted a solve, the
// planned reference would visibly move).
#include <cmath>
#include <cstdio>
#include <string>

#include "bench_test_config.h"
#include "sim_harness.h"

namespace {

int g_failureCount = 0;
std::string g_scenarioName;

void beginScenario(const std::string& name) {
  g_scenarioName = name;
  std::printf("--- %s\n", name.c_str());
}

void fail(const std::string& what) {
  ++g_failureCount;
  std::printf("  FAIL [%s]: %s\n", g_scenarioName.c_str(), what.c_str());
}

void checkTrue(bool condition, const std::string& what) {
  if (!condition) fail(what + " -- expected true, got false");
}

// A little slack over the pure clamp ceiling for float rounding in the
// cruise-phase actuation-lag feedforward term (should be exactly 0 while
// truly at constant velocity, but this stays honest about float noise
// rather than asserting bit-exact equality).
constexpr float kFfSlack = 1.0f;  // [mm/s]

}  // namespace

int main() {
  std::printf("=== App::Pilot linear trim clamp-authority guardrail (112-003, 087-009/SUC-007) ===\n\n");

  // --- Scenario 1: frozen encoders during a straight cruise -- the
  //     commanded correction stays bounded by kDistanceTrimCeiling no
  //     matter how far sRef/sMeas have already diverged ---
  {
    beginScenario("frozen encoders during cruise: commanded correction stays within kDistanceTrimCeiling");
    TestSim::SimHarness sim;
    TestSupport::configureSimForBenchTest(sim);
    sim.boot();
    sim.step(3);

    // 8.0/s -- the SAME production default gen_boot_config.py bakes
    // (DISTANCE_KP_DEFAULT, 112-004's own closed-loop-convergence retune
    // down from 112-003's original 15.0 -- see that constant's own
    // comment) -- this scenario doubles as a regression pin for that real
    // number, not just an arbitrary stress value. Explicit here even
    // though it now also matches TestSim::SimHarness::makeExecutorConfig()'s
    // own shipped default (112-004), since a test should never depend on
    // an ambient default silently tracking a production one.
    constexpr float kDistanceKp = 8.0f;  // [1/s]
    sim.setDistanceKp(kDistanceKp);

    // A long, plain straight leg -- no heading content (deltaHeading=0)
    // keeps the heading PD term out of the signal under test entirely,
    // and a distance far longer than anything this scenario steps through
    // keeps the command safely inside its cruise plateau (acceleration 0)
    // the whole time -- no risk of ever reaching the terminal decel phase.
    constexpr float kVMax = 100.0f;  // [mm/s]
    sim.injectMove(/*distance=*/5000.0f, /*deltaHeading=*/0.0f, kVMax, /*omega=*/0.0f,
                    /*timeMs=*/0.0f, /*replace=*/false, /*id=*/1, /*corrId=*/1);

    // Ramp to cruise: a_max=800mm/s^2 (makeExecutorConfig()) reaches
    // 100mm/s in 125ms (~3 cycles at the harness's own 50ms step); step
    // well past that so acceleration has genuinely settled to 0.
    sim.step(10);
    checkTrue(sim.pilotState() == Motion::State::kRunning,
              "the command is still running (cruise phase) before the freeze");

    float plannedRefBeforeFreeze = sim.plannedRefLeft();
    checkTrue(std::fabs(plannedRefBeforeFreeze - kVMax) < 5.0f,
              "planned reference has reached cruise speed (~" + std::to_string(kVMax) +
                  "mm/s) before the freeze, got " + std::to_string(plannedRefBeforeFreeze));

    // Freeze BOTH wheel encoders (port 1 == left, port 2 == right) --
    // measuredPathSinceActivation_ (sMeas) stops advancing while the
    // already-solved trajectory's own sRef keeps advancing, growing an
    // unbounded (if uncapped) divergence. Held for only 3 cycles (150ms):
    // at kVMax=100mm/s that is a ~15mm sRef-sMeas gap, comfortably under
    // the pre-existing 40mm gross-divergence reanchor threshold
    // (Motion::kDivergenceReanchorLinearMm, executor.cpp) -- this
    // scenario is isolating the NEW trim's own clamp, not that unrelated,
    // pre-existing mechanism.
    sim.plant().freezePosition(/*port=*/1, true);
    sim.plant().freezePosition(/*port=*/2, true);

    bool sawNonTrivialDeviation = false;
    for (int i = 0; i < 3; ++i) {
      sim.step(1);

      float plannedLeft = sim.plannedRefLeft();
      float plannedRight = sim.plannedRefRight();
      float commandedLeft = sim.driveTargetVelLeft();
      float commandedRight = sim.driveTargetVelRight();

      float devLeft = commandedLeft - plannedLeft;
      float devRight = commandedRight - plannedRight;

      checkTrue(std::fabs(devLeft) <= App::kDistanceTrimCeiling + kFfSlack,
                "cycle " + std::to_string(i) + ": left commanded/planned deviation (" +
                    std::to_string(devLeft) + "mm/s) exceeds kDistanceTrimCeiling (" +
                    std::to_string(App::kDistanceTrimCeiling) + "mm/s)");
      checkTrue(std::fabs(devRight) <= App::kDistanceTrimCeiling + kFfSlack,
                "cycle " + std::to_string(i) + ": right commanded/planned deviation (" +
                    std::to_string(devRight) + "mm/s) exceeds kDistanceTrimCeiling (" +
                    std::to_string(App::kDistanceTrimCeiling) + "mm/s)");

      // The trim genuinely engaged (this is not a vacuous pass because the
      // trim happened to stay at 0) -- at kDistanceKp=8.0/s, even a single
      // frozen cycle's own small divergence produces a nonzero correction.
      if (std::fabs(devLeft) > 1.0f) sawNonTrivialDeviation = true;

      // The PLANNED reference itself never moves during the freeze -- the
      // divergence never fed back into a re-solve (this ticket adds no
      // solveToRest/solveToState/solveToVelocity/retarget/reanchor call,
      // and the pre-existing 40mm reanchor tier is not reached within this
      // scenario's own short freeze window).
      checkTrue(std::fabs(plannedLeft - plannedRefBeforeFreeze) < 5.0f,
                "cycle " + std::to_string(i) +
                    ": planned reference stayed at cruise speed during the freeze (no re-solve), got " +
                    std::to_string(plannedLeft));
    }
    checkTrue(sawNonTrivialDeviation,
              "the trim actually engaged (nonzero correction) at some point during the freeze -- "
              "otherwise the bound above would pass vacuously");

    sim.plant().freezePosition(/*port=*/1, false);
    sim.plant().freezePosition(/*port=*/2, false);
  }

  // --- Scenario 2: with distance_kp explicitly set to 0 (112-004: no
  //     longer the ambient sim-harness default -- see
  //     TestSim::SimHarness::makeExecutorConfig()'s own comment, since the
  //     unified completion rule now needs a live, nonzero distance_kp/tol
  //     pair to ever reach kDone -- but still a legitimate opt-OUT any
  //     test can request), the trim is a genuine no-op even under the same
  //     frozen-encoder divergence -- proves this ticket's addition does
  //     not FORCE itself onto a scenario that explicitly wants it off. ---
  {
    beginScenario("distance_kp=0 (explicit opt-out): trim is a true no-op even with a frozen-encoder divergence");
    TestSim::SimHarness sim;
    TestSupport::configureSimForBenchTest(sim);
    sim.boot();
    sim.step(3);
    sim.setDistanceKp(0.0f);

    constexpr float kVMax = 100.0f;  // [mm/s]
    sim.injectMove(/*distance=*/5000.0f, /*deltaHeading=*/0.0f, kVMax, /*omega=*/0.0f,
                    /*timeMs=*/0.0f, /*replace=*/false, /*id=*/2, /*corrId=*/2);
    sim.step(10);

    sim.plant().freezePosition(/*port=*/1, true);
    sim.plant().freezePosition(/*port=*/2, true);
    sim.step(3);

    float dev = sim.driveTargetVelLeft() - sim.plannedRefLeft();
    checkTrue(std::fabs(dev) < 1.0f,
              "with distance_kp explicitly set to 0, commanded == planned (deviation " +
                  std::to_string(dev) + "mm/s), even under the same divergence scenario 1 injects");

    sim.plant().freezePosition(/*port=*/1, false);
    sim.plant().freezePosition(/*port=*/2, false);
  }

  std::printf("\n");
  if (g_failureCount == 0) {
    std::printf("OK: all Pilot linear-trim clamp-authority scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Pilot linear-trim clamp-authority scenarios\n",
              g_failureCount);
  return 1;
}
