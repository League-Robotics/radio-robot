// sim_api_harness.cpp -- off-hardware acceptance harness, migrated (ticket
// 108-004) from TestSim::SimApi (src/tests/sim/support/sim_api.{h,cpp}, deleted
// ticket 108-003) onto TestSim::SimHarness/TestSim::SimPlant
// (tests/_infra/sim/{sim_harness.h,sim_plant.h}). Proves: boot completes
// through the REAL App::RobotLoop (kEventBootReady visible in decoded
// telemetry), an injected MOVE drives REAL plant velocity ramping (visible
// as encLeft/encRight/velLeft/velRight in decoded telemetry), an explicit
// STOP command acks and clears `active`, a MOVE's own TIME stop condition
// (no STOP ever sent) independently ends the Move and clears `active`, and
// the virtual-cycle-timing diagnostic (originally 105-004 AC #3) still
// holds against the real RobotLoop schedule.
//
// The SCENARIO logic is unchanged from the pre-migration SimApi version --
// only the simulator/harness plumbing changed: TestSim::SimHarness replaces
// TestSim::SimApi, and the old scenario's `sim.notePendingActuationChange(3)`
// call (a SimApi::DutyPredictor-only hint for its now-deleted scripted-FIFO
// bus) is simply gone -- SimPlant responds live to whatever firmware
// actually writes, so there is nothing left to hint. The timing scenario
// now reads Devices::Sleeper deltas directly off `sim.sleeper()`
// (SimHarness's own accessor) instead of a bespoke SimApi::
// CycleTimingReport/measureOneCycle() wrapper -- same formula (105-004's
// own derivation: 3 non-final 4ms settle/clear blocks + the observed final
// pace block == the whole cycle's virtual schedule).
//
// 116-006 (MOVE protocol cutover): bare TWIST (injectTwist()) and
// App::Deadman are both gone -- every injection below is a TIME-stop MOVE
// (injectMove()) instead, and scenario 4 (originally "deadman expiry") is
// rewritten as scenarioMoveExpiryStopsPlantWithNoFurtherHostTraffic() --
// see that scenario's own comment for why kFlagEventDeadmanExpired is no
// longer the right signal to assert on.
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

#include "app/telemetry.h"
#include "bench_test_config.h"
#include "messages/envelope.h"
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
    if (f.telemetry.flags & bit) return true;
  }
  return false;
}

// Matches against the single ack slot (Telemetry.ack_corr/ack_err, valid
// iff flags bit 5/kFlagAckFresh) that replaced the pre-115 depth-3 AckEntry
// ring (115-003 frame v2). `okOnly=true` (every existing caller) matches
// only a fresh ack for corrId whose ack_err == 0 (OK).
bool anyAckMatches(const std::vector<DecodedLine>& frames, uint32_t corrId, bool okOnly = true) {
  for (const auto& f : frames) {
    if (!(f.telemetry.flags & App::kFlagAckFresh)) continue;
    if (f.telemetry.ack_corr != corrId) continue;
    if (okOnly && f.telemetry.ack_err != 0) continue;
    return true;
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
  TestSupport::configureSimForBenchTest(sim);
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
  checkTrue(anyEventSet(frames, App::kFlagEventBootReady), "kFlagEventBootReady visible in decoded telemetry");

  bool sawConnected = false;
  for (const auto& f : frames) {
    if ((f.telemetry.flags & App::kFlagConnLeft) && (f.telemetry.flags & App::kFlagConnRight)) sawConnected = true;
  }
  checkTrue(sawConnected, "decoded telemetry reports kFlagConnLeft/kFlagConnRight true");
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
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  sim.step(3);  // settle: both leaves' own one-time zero-duty activation writes land (cycles 0, 1)
  (void)sim.drainTelemetry();  // discard boot/settle frames -- this scenario only cares about the ramp

  // 116-006 (MOVE protocol cutover): bare TWIST/injectTwist() is gone --
  // a TIME-stop MOVE with a stop value/timeout far longer than this run
  // is the equivalent "hold this twist indefinitely" injection.
  sim.injectMove(/*v_x=*/1000.0f, /*v_y=*/0.0f, /*omega=*/0.0f, TestSupport::MoveStopKind::kTime,
                 /*stopValue=*/100000.0f, /*timeout=*/100000.0f, /*replace=*/true, /*id=*/42,
                 /*corrId=*/42);
  sim.step(15);  // ~750ms of virtual ramp time -- comfortably >> TestSim::kDefaultTau (130ms)

  std::vector<DecodedLine> frames = onlyTelemetry(sim.drainTelemetry());
  checkTrue(!frames.empty(), "telemetry decoded during the ramp");
  checkTrue(anyAckMatches(frames, 42), "the twist's corrId=42 was acked OK");

  // EncoderReading (enc_left/enc_right) is unconditionally present every
  // frame (115-005 frame v2 -- no has_vel/has_enc presence flag any more;
  // see telemetry.proto's own EncoderReading doc comment), so every decoded
  // frame carries real position/velocity data, not just a filtered subset.
  float firstVelLeft = 0.0f, lastVelLeft = 0.0f;
  float firstEncLeft = 0.0f, lastEncLeft = 0.0f;
  float lastVelRight = 0.0f;
  bool first = true;
  for (const auto& f : frames) {
    if (first) {
      firstVelLeft = f.telemetry.enc_left.velocity;
      firstEncLeft = f.telemetry.enc_left.position;
      first = false;
    }
    lastVelLeft = f.telemetry.enc_left.velocity;
    lastVelRight = f.telemetry.enc_right.velocity;
    lastEncLeft = f.telemetry.enc_left.position;
  }

  checkTrue(!first, "at least one frame carried encoder data");
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
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  // 116-006 (MOVE protocol cutover): bare TWIST/injectTwist() is gone --
  // a TIME-stop MOVE with a stop value/timeout far longer than this run
  // is the equivalent "hold this twist indefinitely" injection.
  sim.injectMove(1000.0f, /*v_y=*/0.0f, 0.0f, TestSupport::MoveStopKind::kTime,
                 /*stopValue=*/100000.0f, /*timeout=*/100000.0f, /*replace=*/true, /*id=*/7,
                 /*corrId=*/7);
  sim.step(5);  // ramp a bit so there is real motion to stop
  (void)sim.drainTelemetry();

  sim.injectStop(/*corrId=*/99);
  sim.step(3);  // dispatch cycle + emit-lag cycle(s) -- see sim_api.cpp's own one-cycle-lag notes

  std::vector<DecodedLine> frames = onlyTelemetry(sim.drainTelemetry());
  checkTrue(!frames.empty(), "telemetry decoded after STOP");
  checkTrue(anyAckMatches(frames, 99), "the stop's corrId=99 was acked OK");

  bool sawInactive = false;
  bool sawIdleMode = false;
  for (const auto& f : frames) {
    if (!(f.telemetry.flags & App::kFlagActive)) sawInactive = true;
    if (f.telemetry.mode == msg::DriveMode::IDLE) sawIdleMode = true;
  }
  checkTrue(sawInactive, "decoded telemetry shows active=false after STOP");
  checkTrue(sawIdleMode, "decoded telemetry shows mode=IDLE after STOP");
}

// ===========================================================================
// 4. MOVE expiry stops the plant (116-006, MOVE protocol cutover -- REPLACES
//    the deleted App::Deadman's own expiry scenario): NO STOP is ever sent
//    -- a short TIME-stop MOVE's own stop condition is met on its own, and
//    MoveQueue::tick()'s own "queue now empty -> Drive::stop()" path
//    (move_queue.cpp) fires unconditionally, every cycle, with no lease to
//    re-arm and no second command required. kFlagEventDeadmanExpired
//    (telemetry.h bit 10) is DEAD CODE post-cutover -- nothing in
//    robot_loop.cpp ever sets it any more -- so this scenario now asserts
//    the actual MOVE-protocol signal instead: the completion ack
//    (ack_corr == Move.id, ack_err == 0) and active clearing, matching
//    move_queue.h's own documented completion contract. stopValue=120ms
//    with SimHarness::kCycleDtUs=50ms meets the stop condition ~3 cycles
//    after activation.
// ===========================================================================

void scenarioMoveExpiryStopsPlantWithNoFurtherHostTraffic() {
  beginScenario("MOVE expiry: no STOP ever sent, TIME stop condition ends the Move on its own, "
                "active clears (116-006)");

  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  const uint32_t kMoveId = 5;
  sim.injectMove(1000.0f, /*v_y=*/0.0f, 0.0f, TestSupport::MoveStopKind::kTime,
                 /*stopValue=*/120.0f, /*timeout=*/100000.0f, /*replace=*/true, kMoveId,
                 /*corrId=*/kMoveId);  // [ms] -- stop condition met ~3 cycles after activation
  sim.step(2);  // cycles 0, 1 -- the Move's own R/L activation writes land, not yet met
  // No further command is ever injected below -- the whole point of this
  // scenario (matching the deleted deadman's own "host silence -> stop"
  // guarantee) is that MoveQueue::tick() ends the Move and stops the
  // plant on its own, with zero additional host traffic.
  sim.step(3);  // cycles 2 (quiet), 3 (stop condition met, chain-empty -> Drive::stop()), 4
  sim.step(1);  // emit-lag buffer cycle

  std::vector<DecodedLine> frames = onlyTelemetry(sim.drainTelemetry());
  checkTrue(!frames.empty(), "telemetry decoded across the MOVE's own window");
  checkTrue(anyAckMatches(frames, kMoveId), "the Move's completion ack (ack_corr==Move.id, ack_err==0) "
                                            "reached the wire with no STOP ever sent");

  bool sawInactive = false;
  for (const auto& f : frames) {
    if (!(f.telemetry.flags & App::kFlagActive)) sawInactive = true;
  }
  checkTrue(sawInactive, "decoded telemetry shows active=false once the Move's own TIME stop "
                         "condition is met (no STOP was ever sent, no deadman lease involved)");
}

// ===========================================================================
// 5. Virtual-cycle-timing diagnostic (105-004 AC #3) -- PROMOTED (106-001
//    AC #2) from an observational report to a hard pass/fail regression
//    assertion on the schedule's own NUMBERS: a future change that re-adds
//    105-004's diagnosed defect (the settle/clearance windows stacking
//    additively under the final pace block instead of being absorbed into
//    kCycle's stated total) fails this checkTrue -- and therefore fails
//    `uv run python -m pytest` -- not just a future bench session.
//
//    The expected numbers below mirror robot_loop.cpp's OWN current
//    kSettle/kClear/kCycle/kPace (anonymous-namespace/internal-linkage, not
//    importable -- duplicated here per this codebase's established
//    per-file fixture-duplication convention; see the coding-standards
//    rule's "grep-ability" rationale for why this file keeps its own
//    copy rather than reaching into robot_loop.cpp's internals).
//    118 (loop schedule truth) restores kSettle=kClear=4/kCycle=40/kPace=28
//    -- 106-001's original figures -- undoing the 111-002 retarget
//    (2026-07-19) to kSettle=kClear=0/kCycle=20/kPace=20 that commit
//    5f5a2ba7 forced onto this schedule (zeroing kSettle/kClear made the
//    vendor's still-mandatory 4ms settle happen as a *blocking* sleep
//    hidden inside motorL_.tick()/motorR_.tick() instead of a visible,
//    budgeted runAndWait window -- see
//    clasi/issues/restore-the-interleaved-request-settle-tick-loop-schedule.md).
//    Whoever next changes robot_loop.cpp's timing constants must update
//    these four to match, the same way any other duplicated-constant
//    fixture in this codebase does.
// ===========================================================================

void scenarioVirtualCycleTimingDiagnostic() {
  beginScenario("timing: virtual-cycle schedule is exactly kSettle+kClear+kSettle+kPace == kCycle (106-001)");

  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();

  // Reproduces the deleted SimApi::measureOneCycle()'s own deltas directly
  // off Devices::Sleeper (sim.sleeper(), added to SimHarness by this
  // ticket) -- same formula as the original CycleTimingReport: the three
  // non-final settle/clear blocks (robot_loop.cpp's own kSettle/kClear,
  // duplicated below) plus the observed final pace block equal the whole
  // cycle's virtual schedule.
  TestSim::SimSleeper& sleeper = sim.sleeper();
  int sleepsBefore = sleeper.sleepCount();
  int yieldsBefore = sleeper.yieldCount();

  sim.step(1);

  int sleepCount = sleeper.sleepCount() - sleepsBefore;
  uint32_t lastSleepMillis = sleeper.lastSleepMillis();
  int yieldCount = sleeper.yieldCount() - yieldsBefore;
  // robot_loop.cpp's own current kSettle/kClear/kCycle/kPace (see this
  // scenario's own file-header comment above for the duplication
  // rationale and the 118 restore note).
  constexpr uint32_t kSettle = 4;  // [ms] mirrors robot_loop.cpp's own kSettle
  constexpr uint32_t kClear = 4;   // [ms] mirrors robot_loop.cpp's own kClear
  constexpr uint32_t kCycle = 40;  // [ms] mirrors robot_loop.cpp's own kCycle
  constexpr uint32_t kWindows = 2 * kSettle + kClear;  // [ms] the 3 settle/clear blocks' own total
  constexpr uint32_t kPace = kCycle - kWindows;        // [ms] mirrors robot_loop.cpp's own kPace
  uint32_t virtualCycleMillis = kWindows + lastSleepMillis;

  checkTrue(sleepCount == 4,
            "exactly 4 Sleeper::sleepMillis() calls per cycle() (3 runAndWait blocks + final pace block)");
  checkTrue(lastSleepMillis == kPace,
            "the final (perception+odometry+pace) block requests exactly kPace=28ms "
            "(kCycle=40ms minus the 12ms already consumed by the 3 settle/clear windows -- "
            "NOT a fresh, unabsorbed kCycle=40ms on top of them)");
  checkTrue(yieldCount == 0, "RobotLoop::cycle() never calls Sleeper::yield() directly");
  checkTrue(virtualCycleMillis == kCycle,
            "derived total virtual schedule == 12ms (settle/clear/settle) + 28ms (pace) == 40ms == kCycle -- "
            "proves the three windows are absorbed into the 40ms budget, not additive on top of it (118, "
            "restoring 106-001)");

  std::printf(
      "  TIMING: sleepCount=%d lastSleepMillis=%ums yieldCount=%d virtualCycleMillis=%ums "
      "(== kCycle=40ms/~25Hz design target -- see this scenario's own file-header comment for the "
      "118 restore of 106-001's original kCycle=40ms figures)\n",
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
  scenarioMoveExpiryStopsPlantWithNoFurtherHostTraffic();
  scenarioVirtualCycleTimingDiagnostic();

  if (g_failureCount == 0) {
    std::printf("OK: all sim_api scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the sim_api scenarios\n", g_failureCount);
  return 1;
}
