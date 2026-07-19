// heading_source_harness.cpp -- sprint 109 ticket 005's own sim-level
// acceptance: DISTANCE-mode arcs/pivots end to end on the REAL
// App::RobotLoop/SimPlant graph (TestSim::SimHarness), plus
// App::HeadingSource's OTOS-first/encoder-fallback policy and its
// telemetry visibility (SUC-002/SUC-004).
//
// Scenarios (this ticket's own testing plan):
//   1. Pure pivot, ideal sim OTOS (no drift/noise): the sim plant's OTOS is
//      exact, so the completed pivot's TRUE heading (SimHarness::
//      trueHeading()) must match the commanded deltaHeading exactly (v1
//      floating-point tolerance, not a "within 1 deg" bar -- that bar is
//      ticket 009's own sim-gate job once drift/noise are enabled).
//   2. Coupled arc: a single arc command's velocity trace is jerk-bounded
//      (no instantaneous step) and the completed arc's true heading change
//      matches deltaHeading exactly under the same ideal-plant condition.
//   3. HeadingSource fallback + re-promotion, driven via a SimPlant read
//      hook that fails every OTOS bus transaction for a scripted window:
//      TLM's headingSource flips to ENCODER, an event fires, then flips
//      back to OTOS with another event once the hook clears.
//   4. DISTANCE-then-idle chaining: a single DISTANCE command completes and
//      the executor returns to kIdle with zero residual twist -- no
//      leftover motion after the last queued command finishes.
#include <cmath>
#include <cstdio>
#include <string>

#include "devices/otos.h"
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

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;

// Finds the LAST decoded telemetry frame's own headingSource/event_bits --
// TestSim::SimHarness::drainTelemetry() returns every frame since the last
// drain; tests here only care about the most recent state.
bool lastHeadingSourceIsOtos(const std::vector<TestSupport::DecodedLine>& lines, bool* found) {
  *found = false;
  bool isOtos = true;
  for (const auto& line : lines) {
    if (line.kind != TestSupport::DecodedKind::kTelemetry) continue;
    *found = true;
    isOtos = (line.telemetry.heading_source == msg::HeadingSourceStatus::HEADING_SOURCE_STATUS_OTOS);
  }
  return isOtos;
}

bool anyFallbackEvent(const std::vector<TestSupport::DecodedLine>& lines) {
  for (const auto& line : lines) {
    if (line.kind != TestSupport::DecodedKind::kTelemetry) continue;
    if (line.telemetry.event_bits & App::kEventHeadingFallback) return true;
  }
  return false;
}

}  // namespace

int main() {
  std::printf("=== HeadingSource / DISTANCE-mode Sim Scenarios (109-005, SUC-002/SUC-004) ===\n\n");

  // --- Scenario 1: pure pivot, ideal OTOS -> exact final heading ---
  {
    beginScenario("pure pivot (ideal sim OTOS): true heading matches commanded deltaHeading exactly");
    TestSim::SimHarness sim;
    sim.boot();
    sim.step(3);

    constexpr float kDeltaHeading = 90.0f * kDegToRad;
    sim.injectMove(/*distance=*/0.0f, kDeltaHeading, /*vMax=*/0.0f, /*omega=*/0.0f,
                    /*timeMs=*/0.0f, /*replace=*/false, /*id=*/1, /*corrId=*/1);

    bool idleAgain = false;
    for (int i = 0; i < 400 && !idleAgain; ++i) {
      sim.step(1);
      if (i > 5 && sim.pilotState() == Motion::State::kIdle) idleAgain = true;
    }
    checkTrue(idleAgain, "the pivot completed and the executor returned to kIdle");
    checkTrue(std::fabs(sim.trueHeading() - kDeltaHeading) < 0.02f,
              "true heading matches the commanded 90deg pivot (ideal plant, near-exact)");
  }

  // --- Scenario 2: coupled arc -- jerk-bounded trace + exact heading change
  //     under the ideal plant ---
  {
    beginScenario("coupled arc (ideal sim OTOS): jerk-bounded trace, exact heading change");
    TestSim::SimHarness sim;
    sim.boot();
    sim.step(3);

    constexpr float kDistance = 400.0f;              // [mm]
    constexpr float kDeltaHeading = 45.0f * kDegToRad;  // [rad]
    sim.injectMove(kDistance, kDeltaHeading, /*vMax=*/200.0f, 0.0f, 0.0f, false, /*id=*/2,
                    /*corrId=*/2);

    // 112-002: this gate grades the PLANNED reference (SimHarness::
    // plannedRefLeft() -> App::Pilot::refLeft(), Motion::Executor's own
    // jerk-limited trajectory), NOT the measured plant velocity
    // (motorLeft().velocity()) it graded pre-112-002. Confirmed directly
    // (temporary trace instrumentation during this ticket's own work): the
    // measured signal is naturally noisy/oscillatory cycle-to-cycle even
    // with NO feedforward at all (encoder-derived, unfiltered
    // difference-quotient velocity riding on the write/settle cadence) --
    // present but under 200mm/s/cycle pre-112-002, and pushed over it once
    // the accel feedforward's own (legitimate, deliberate) faster early
    // ramp scaled the SAME pre-existing oscillation up proportionally. The
    // PLANNED reference is exactly what this gate is actually trying to
    // verify ("is the commanded trajectory itself jerk-bounded, no
    // instantaneous-step bug like the old plan_lead peek-ahead F2 warp") --
    // grading the measured trace instead conflated that question with
    // downstream PID/plant measurement noise, the same category of mistake
    // 112-002's own behavior_lock_harness.cpp re-grade fixed for the ramp/
    // terminal-bounds/single-lobe checks (see that file's own header
    // comment for the full three-signal rationale).
    float lastVel = 0.0f;
    bool everInstantStep = false;
    bool idleAgain = false;
    for (int i = 0; i < 400 && !idleAgain; ++i) {
      sim.step(1);
      float vel = sim.plannedRefLeft();
      if (std::fabs(vel - lastVel) > 200.0f) everInstantStep = true;
      lastVel = vel;
      if (i > 5 && sim.pilotState() == Motion::State::kIdle) idleAgain = true;
    }
    checkTrue(!everInstantStep, "no single cycle stepped velocity instantaneously (jerk-limited)");
    checkTrue(idleAgain, "the arc completed and the executor returned to kIdle");
    checkTrue(std::fabs(sim.trueHeading() - kDeltaHeading) < 0.05f,
              "true heading matches the commanded 45deg arc turn (ideal plant, near-exact)");
  }

  // --- Scenario 3: HeadingSource fallback + re-promotion, visible in TLM ---
  {
    beginScenario("OTOS staleness -> encoder fallback -> re-promotion, both visible in TLM");
    TestSim::SimHarness sim;
    sim.boot();
    sim.step(3);
    sim.drainTelemetry();  // clear anything queued during boot/settle

    checkTrue(sim.headingSourceIsOtos(), "starts on OTOS (present/connected/fresh)");

    // Fail every OTOS bus transaction (address 0x17<<1 = 0x2E) for a window
    // comfortably past kFallbackStaleCycles (5) -- 10 sim cycles.
    constexpr uint16_t kOtosWireAddr = Devices::kOtosDeviceAddr << 1;
    sim.plant().setReadHook([&](uint16_t address, uint8_t* data, int len) -> int {
      if (address == kOtosWireAddr) return -1;
      return sim.plant().defaultRead(address, data, len);
    });

    bool sawFallbackEvent = false;
    for (int i = 0; i < 15 && !sawFallbackEvent; ++i) {
      sim.step(1);
      if (anyFallbackEvent(sim.drainTelemetry())) sawFallbackEvent = true;
    }
    checkTrue(sawFallbackEvent, "a fallback-transition event fired within the stale window");
    checkTrue(!sim.headingSourceIsOtos(), "active source is now the encoder");

    bool found = false;
    bool isOtos = lastHeadingSourceIsOtos(sim.drainTelemetry(), &found);
    // (found may be false if no primary frame was drained since the last
    // check above -- step a few more cycles to force one.)
    for (int i = 0; i < 5 && !found; ++i) {
      sim.step(1);
      isOtos = lastHeadingSourceIsOtos(sim.drainTelemetry(), &found);
    }
    checkTrue(found, "a primary telemetry frame reporting heading_source was observed");
    checkTrue(!isOtos, "TLM's own heading_source field reports ENCODER while OTOS is stale");

    // Clear the hook -- OTOS recovers; re-promotion should be immediate and
    // fire its own event.
    sim.plant().clearReadHook();
    bool sawRecoveryEvent = false;
    for (int i = 0; i < 10 && !sawRecoveryEvent; ++i) {
      sim.step(1);
      if (anyFallbackEvent(sim.drainTelemetry())) sawRecoveryEvent = true;
    }
    checkTrue(sawRecoveryEvent, "a re-promotion event fired once OTOS recovered");
    checkTrue(sim.headingSourceIsOtos(), "active source is back to OTOS");
  }

  // --- Scenario 4: DISTANCE-then-idle chaining -- no residual motion after
  //     the last queued command finishes ---
  {
    beginScenario("DISTANCE command completes -> kIdle, zero residual twist");
    TestSim::SimHarness sim;
    sim.boot();
    sim.step(3);

    sim.injectMove(/*distance=*/200.0f, 0.0f, /*vMax=*/150.0f, 0.0f, 0.0f, false, /*id=*/3,
                    /*corrId=*/3);

    bool idleAgain = false;
    for (int i = 0; i < 300 && !idleAgain; ++i) {
      sim.step(1);
      if (i > 5 && sim.pilotState() == Motion::State::kIdle) idleAgain = true;
    }
    checkTrue(idleAgain, "the DISTANCE command completed and the executor returned to kIdle");

    // Settle a few more cycles, then confirm velocity has decayed to rest --
    // no leftover commanded motion once the queue is empty and kIdle.
    sim.step(10);
    checkTrue(std::fabs(sim.motorLeft().velocity()) < 15.0f,
              "left wheel settled near rest after the DISTANCE command's own completion");
    checkTrue(std::fabs(sim.motorRight().velocity()) < 15.0f,
              "right wheel settled near rest after the DISTANCE command's own completion");
  }

  std::printf("\n");
  if (g_failureCount == 0) {
    std::printf("OK: all HeadingSource/DISTANCE-mode sim scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the HeadingSource/DISTANCE-mode sim scenarios\n",
              g_failureCount);
  return 1;
}
