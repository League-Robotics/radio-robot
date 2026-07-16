// sim_api_harness.cpp -- off-hardware acceptance harness, migrated (ticket
// 108-004) from TestSim::SimApi (src/tests/sim/support/sim_api.{h,cpp}, deleted
// ticket 108-003) onto TestSim::SimHarness/TestSim::SimPlant
// (tests/_infra/sim/{sim_harness.h,sim_plant.h}). Proves: boot completes
// through the REAL App::RobotLoop (kEventBootReady visible in decoded
// telemetry), an injected twist drives REAL plant velocity ramping (visible
// as encLeft/encRight/velLeft/velRight in decoded telemetry), an explicit
// STOP command acks and clears `active`, a deadman expiry (no STOP ever
// sent) independently sets kEventDeadmanExpired and clears `active`, and
// the virtual-cycle-timing diagnostic (originally 105-004 AC #3) still
// holds against the real RobotLoop schedule.
//
// The SCENARIO logic is unchanged from the pre-migration SimApi version --
// only the simulator/harness plumbing changed: TestSim::SimHarness replaces
// TestSim::SimApi, and the deadman-expiry scenario's old
// `sim.notePendingActuationChange(3)` call (a SimApi::DutyPredictor-only
// hint for its now-deleted scripted-FIFO bus) is simply gone -- SimPlant
// responds live to whatever firmware actually writes, so there is nothing
// left to hint. The timing scenario now reads Devices::Sleeper deltas
// directly off `sim.sleeper()` (SimHarness's own accessor) instead of a
// bespoke SimApi::CycleTimingReport/measureOneCycle() wrapper -- same
// formula (105-004's own derivation: 3 non-final 4ms settle/clear blocks +
// the observed final pace block == the whole cycle's virtual schedule).
//
// Hand-rolled assertions, PASS/FAIL per scenario, nonzero exit on any
// failure -- mirrors every other src/tests/sim/{unit,plant} harness's own shape
// (see plant_harness.cpp/app_robot_loop_harness.cpp). Run by
// test_sim_api.py, which compiles this file together with sim_plant.cpp,
// wire_test_codec.cpp, the plant sources, and every HOST_BUILD Devices/App
// source it needs, then runs the resulting binary via subprocess.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "messages/envelope.h"
#include "messages/planner.h"
#include "sim_harness.h"
#include "wire_test_codec.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors every other tests/sim harness
// in this codebase) ---------------------------------------------------------

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

void checkFloatGe(float actual, float bound, const std::string& what) {
  if (!(actual >= bound)) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected >= %g, got %g", what.c_str(),
                  static_cast<double>(bound), static_cast<double>(actual));
    fail(buf);
  }
}

// --- Small local helpers -----------------------------------------------

using TestSupport::DecodedKind;
using TestSupport::DecodedLine;

std::vector<DecodedLine> onlyTelemetry(const std::vector<DecodedLine>& lines) {
  std::vector<DecodedLine> out;
  for (const auto& l : lines) {
    if (l.kind == DecodedKind::kTelemetry) out.push_back(l);
  }
  return out;
}

bool anyEventSet(const std::vector<DecodedLine>& frames, uint32_t bit) {
  for (const auto& f : frames) {
    if (f.telemetry.event_bits & bit) return true;
  }
  return false;
}

bool anyAckMatches(const std::vector<DecodedLine>& frames, uint32_t corrId, msg::AckStatus status) {
  for (const auto& f : frames) {
    for (uint8_t i = 0; i < f.telemetry.acks_count; ++i) {
      const msg::AckEntry& e = f.telemetry.acks_[i];
      if (e.corr_id == corrId && e.status == status) return true;
    }
  }
  return false;
}

// ===========================================================================
// 1. Boot: SimHarness::boot() drives App::Preamble to done() and calls
//    the REAL App::RobotLoop::boot() -- both motors and OTOS resolve
//    connected (scripted success); kEventBootReady becomes visible in
//    decoded telemetry once at least one cycle() has run past boot (the
//    bit is SET at the end of boot() but only actually EMITTED by the next
//    cycle()'s own tlm_.emit() call -- robot_loop.cpp's own boot()/cycle()
//    split).
// ===========================================================================

void scenarioBootCompletesThroughRealRobotLoop() {
  beginScenario("boot: SimHarness drives the REAL RobotLoop::boot(), motors+OTOS connect, kEventBootReady visible");

  TestSim::SimHarness sim;
  checkTrue(!sim.booted(), "not booted before boot() is called");

  sim.boot();  // SimHarness's own dedicated boot() entry point -- see sim_harness.h's file header
               // for why boot()/step() are two separate calls here, unlike the deleted SimApi's
               // single overloaded step().
  checkTrue(sim.booted(), "booted() true after boot()");
  checkTrue(sim.motorLeft().connected(), "left motor connected after boot");
  checkTrue(sim.motorRight().connected(), "right motor connected after boot");

  sim.step(1);  // one main cycle -- this is what actually EMITS the boot-ready bit
  std::vector<DecodedLine> frames = onlyTelemetry(sim.drainTelemetry());
  checkTrue(!frames.empty(), "at least one telemetry frame decoded after boot + one cycle");
  checkTrue(anyEventSet(frames, App::kEventBootReady), "kEventBootReady visible in decoded telemetry");

  bool sawConnected = false;
  for (const auto& f : frames) {
    if (f.telemetry.conn_left && f.telemetry.conn_right) sawConnected = true;
  }
  checkTrue(sawConnected, "decoded telemetry reports conn_left/conn_right true");
}

// ===========================================================================
// 2. Twist -> real plant velocity ramp -> encoder movement in TLM (105-004's
//    own AC #2). v_x is chosen far above the plant's own achievable ceiling
//    (TestSim::kDefaultDutyVelMax) so the PID saturates immediately and
//    stays saturated for the whole run -- see sim_api.h's own "Plant/PID
//    tuning" section for the full derivation this scenario's bus-script
//    counts (SimApi::scriptCycleBusResponses()) depend on.
// ===========================================================================

void scenarioTwistDrivesRealPlantRamp() {
  beginScenario("twist: injected command drives REAL plant velocity ramp, visible in decoded TLM");

  TestSim::SimHarness sim;
  sim.boot();
  sim.step(3);  // settle: both leaves' own one-time zero-duty activation writes land (cycles 0, 1)
  (void)sim.drainTelemetry();  // discard boot/settle frames -- this scenario only cares about the ramp

  sim.injectTwist(/*v_x=*/1000.0f, /*omega=*/0.0f, /*duration=*/100000.0f, /*corrId=*/42);
  sim.step(15);  // ~750ms of virtual ramp time -- comfortably >> TestSim::kDefaultTau (130ms)

  std::vector<DecodedLine> frames = onlyTelemetry(sim.drainTelemetry());
  checkTrue(!frames.empty(), "telemetry decoded during the ramp");
  checkTrue(anyAckMatches(frames, 42, msg::AckStatus::ACK_STATUS_OK), "the twist's corrId=42 was acked OK");

  float firstVelLeft = 0.0f, lastVelLeft = 0.0f;
  float firstEncLeft = 0.0f, lastEncLeft = 0.0f;
  float lastVelRight = 0.0f;
  bool first = true;
  for (const auto& f : frames) {
    if (!f.telemetry.has_vel || !f.telemetry.has_enc) continue;
    if (first) {
      firstVelLeft = f.telemetry.vel_left;
      firstEncLeft = f.telemetry.enc_left;
      first = false;
    }
    lastVelLeft = f.telemetry.vel_left;
    lastVelRight = f.telemetry.vel_right;
    lastEncLeft = f.telemetry.enc_left;
  }

  checkTrue(!first, "at least one frame carried has_vel/has_enc data");
  checkFloatGe(lastVelLeft, 300.0f, "velLeft ramped well above its starting value toward the plant's ceiling");
  checkFloatGe(lastVelRight, 300.0f, "velRight ramped well above its starting value toward the plant's ceiling");
  checkTrue(lastVelLeft > firstVelLeft, "velLeft increased over the ramp (moving in the commanded direction)");
  checkTrue(lastEncLeft > firstEncLeft, "encLeft advanced over the ramp (real encoder movement in TLM)");
}

// ===========================================================================
// 3. STOP works: an explicit STOP command acks OK and clears `active` --
//    read entirely through decoded telemetry, never the plant/motor state
//    directly (the point of this scenario is the wire-out contract).
// ===========================================================================

void scenarioStopAcksAndClearsActive() {
  beginScenario("stop: explicit STOP command acks OK, decoded telemetry active clears");

  TestSim::SimHarness sim;
  sim.boot();
  sim.injectTwist(1000.0f, 0.0f, 100000.0f, /*corrId=*/7);
  sim.step(5);  // ramp a bit so there is real motion to stop
  (void)sim.drainTelemetry();

  sim.injectStop(/*corrId=*/99);
  sim.step(3);  // dispatch cycle + emit-lag cycle(s) -- see sim_api.cpp's own one-cycle-lag notes

  std::vector<DecodedLine> frames = onlyTelemetry(sim.drainTelemetry());
  checkTrue(!frames.empty(), "telemetry decoded after STOP");
  checkTrue(anyAckMatches(frames, 99, msg::AckStatus::ACK_STATUS_OK), "the stop's corrId=99 was acked OK");

  bool sawInactive = false;
  bool sawIdleMode = false;
  for (const auto& f : frames) {
    if (!f.telemetry.active) sawInactive = true;
    if (f.telemetry.mode == msg::DriveMode::IDLE) sawIdleMode = true;
  }
  checkTrue(sawInactive, "decoded telemetry shows active=false after STOP");
  checkTrue(sawIdleMode, "decoded telemetry shows mode=IDLE after STOP");
}

// ===========================================================================
// 4. Deadman expiry stops the plant: NO STOP is ever sent -- a short-
//    duration twist's own deadman window lapses, and RobotLoop::cycle()'s
//    own "host silent -> wheels stop" path (robot_loop.cpp) fires on its
//    own. duration=120ms with SimApi::kCycleDtUs=50ms expires exactly 3
//    cycles after the twist is dispatched -- see sim_api.cpp's own
//    scriptCycleBusResponses() comment for the derivation
//    notePendingActuationChange(3) below depends on.
// ===========================================================================

void scenarioDeadmanExpiryStopsPlant() {
  beginScenario("deadman: expiry (no STOP ever sent) sets kEventDeadmanExpired, clears active");

  TestSim::SimHarness sim;
  sim.boot();
  sim.injectTwist(1000.0f, 0.0f, /*duration=*/120.0f, /*corrId=*/5);  // [ms] -- expires in 3 cycles
  sim.step(2);  // cycles 0, 1 -- twist's own R/L activation writes land, not yet expired
  // No SimApi::notePendingActuationChange() hint needed here -- that call
  // primed the deleted scripted-FIFO bus for the expiry-triggered duty
  // write it could not otherwise predict; SimPlant just responds live to
  // whatever RobotLoop actually writes when the deadman trips.
  sim.step(3);  // cycles 2 (quiet), 3 (expiry fires, R writes), 4 (L writes)
  sim.step(1);  // emit-lag buffer cycle

  std::vector<DecodedLine> frames = onlyTelemetry(sim.drainTelemetry());
  checkTrue(!frames.empty(), "telemetry decoded across the deadman window");
  checkTrue(anyEventSet(frames, App::kEventDeadmanExpired), "kEventDeadmanExpired visible in decoded telemetry");

  bool sawInactive = false;
  for (const auto& f : frames) {
    if (!f.telemetry.active) sawInactive = true;
  }
  checkTrue(sawInactive, "decoded telemetry shows active=false once the deadman expires (no STOP was ever sent)");
}

// ===========================================================================
// 5. Virtual-cycle-timing diagnostic (105-004 AC #3) -- PROMOTED (106-001
//    AC #2) from an observational report to a hard pass/fail regression
//    assertion on the schedule's own NUMBERS: a future change that re-adds
//    105-004's diagnosed defect (the three settle/clearance windows
//    stacking additively under the final pace block instead of being
//    absorbed into kCycle's stated total) fails this checkTrue -- and
//    therefore fails `uv run python -m pytest` -- not just a future bench
//    session. The expected numbers below are 106-001's retargeted ~25 Hz
//    (~40ms) schedule (robot_loop.cpp's own kSettle/kClear/kCycle/kPace),
//    not the pre-106-001 16ms-target/28ms-virtual figures this same
//    assertion used to lock in.
// ===========================================================================

void scenarioVirtualCycleTimingDiagnostic() {
  beginScenario("timing: virtual-cycle schedule is exactly kSettle+kClear+kSettle+kPace == kCycle (106-001)");

  TestSim::SimHarness sim;
  sim.boot();

  // Reproduces the deleted SimApi::measureOneCycle()'s own deltas directly
  // off Devices::Sleeper (sim.sleeper(), added to SimHarness by this
  // ticket) -- same formula as the original CycleTimingReport: 3 non-final
  // 4ms settle/clear blocks (robot_loop.cpp's own kSettle/kClear) plus the
  // observed final pace block equal the whole cycle's virtual schedule.
  TestSim::SimSleeper& sleeper = sim.sleeper();
  int sleepsBefore = sleeper.sleepCount();
  int yieldsBefore = sleeper.yieldCount();

  sim.step(1);

  int sleepCount = sleeper.sleepCount() - sleepsBefore;
  uint32_t lastSleepMillis = sleeper.lastSleepMillis();
  int yieldCount = sleeper.yieldCount() - yieldsBefore;
  constexpr uint32_t kNonFinalBlockMillis = 4;
  uint32_t virtualCycleMillis = 3 * kNonFinalBlockMillis + lastSleepMillis;

  checkTrue(sleepCount == 4,
            "exactly 4 Sleeper::sleepMillis() calls per cycle() (3 runAndWait blocks + final pace block)");
  checkTrue(lastSleepMillis == 28,
            "the final (perception+odometry+pace) block requests exactly kPace=28ms "
            "(kCycle=40ms minus the 12ms already consumed by the 3 settle/clear windows -- "
            "NOT a fresh, unabsorbed kCycle=40ms on top of them)");
  checkTrue(yieldCount == 0, "RobotLoop::cycle() never calls Sleeper::yield() directly");
  checkTrue(virtualCycleMillis == 40,
            "derived total virtual schedule == 3*4ms (settle/clear/settle) + 28ms (pace) == 40ms == kCycle -- "
            "proves the three windows are absorbed into the retargeted 40ms budget, not additive on top of "
            "it (106-001; pre-fix this was 28ms > the old kCycle=16ms target)");

  std::printf(
      "  TIMING: sleepCount=%d lastSleepMillis=%ums yieldCount=%d virtualCycleMillis=%ums "
      "(== kCycle=40ms/~25Hz design target, retargeted 106-001 from the unachievable 16ms/28ms-virtual "
      "pre-fix figures -- see this ticket's completion notes for the bench-measured real-hardware "
      "reconciliation against sprint 104's ~36ms and the ack-ring issue's ~72ms/13.87Hz)\n",
      sleepCount, static_cast<unsigned>(lastSleepMillis), yieldCount, static_cast<unsigned>(virtualCycleMillis));
}

// ===========================================================================
// 6. No MicroBit.h dependency (105-004 AC #4) -- this is a static/compile-
//    level property (proven by test_sim_api.py's own HOST_BUILD compile
//    step succeeding with no MicroBit.h anywhere in the include graph), not
//    something a runtime scenario can assert -- recorded here as a comment
//    landmark matching every sibling ticket's own harness convention (e.g.
//    app_robot_loop_harness.cpp's identical note).
// ===========================================================================

}  // namespace

int main() {
  scenarioBootCompletesThroughRealRobotLoop();
  scenarioTwistDrivesRealPlantRamp();
  scenarioStopAcksAndClearsActive();
  scenarioDeadmanExpiryStopsPlant();
  scenarioVirtualCycleTimingDiagnostic();

  if (g_failureCount == 0) {
    std::printf("OK: all sim_api scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the sim_api scenarios\n", g_failureCount);
  return 1;
}
