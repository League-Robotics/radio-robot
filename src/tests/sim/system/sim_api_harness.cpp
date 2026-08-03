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

// Matches against the bounded ack ring (Telemetry.acks, 124-008: packed
// uint32 corr_id<<4|err -- the single "freshest ack" scalar slot/
// kFlagAckFresh this used to gate on is DELETED, issue §B4; ring membership
// alone means "really acked"). `okOnly=true` (every existing caller)
// matches only an ack for corrId whose err == 0 (OK).
bool anyAckMatches(const std::vector<DecodedLine>& frames, uint32_t corrId, bool okOnly = true) {
  constexpr uint32_t kAckErrBits = 4;
  constexpr uint32_t kAckErrMask = (1u << kAckErrBits) - 1;
  for (const auto& f : frames) {
    for (uint8_t i = 0; i < f.telemetry.acks_count; ++i) {
      const uint32_t packed = f.telemetry.acks_[i];
      if ((packed >> kAckErrBits) != corrId) continue;
      const uint32_t err = packed & kAckErrMask;
      if (okOnly && err != 0) continue;
      return true;
    }
  }
  return false;
}

// ===========================================================================
// 1. Boot: SimHarness::boot() drives App::Preamble to done() and calls the
//    REAL App::RobotLoop::boot() -- both motors and OTOS resolve connected
//    (scripted success). 125-002 (telemetry-emit-policy-rebuild-spec.md
//    Part 1 item 8/Part 8 #1): there is no boot-ready telemetry bit any
//    more, and a silent host gets ZERO unsolicited frames after boot (mode_
//    defaults kAuto, everMoved_ is still false) -- so this scenario now
//    proves connectivity via a bare TLM request (reason 1, honored in every
//    mode) instead of watching for an unsolicited boot-ready frame that no
//    longer exists.
// ===========================================================================

void scenarioBootCompletesThroughRealRobotLoop() {
  beginScenario("boot: SimHarness drives the REAL RobotLoop::boot(), motors+OTOS connect; a silent host stays "
                "silent (issue Part 8 #1); a bare TLM request confirms connectivity");

  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  checkTrue(!sim.booted(), "not booted before boot() is called");

  sim.boot();  // SimHarness's own dedicated boot() entry point -- see sim_harness.h's file header
               // for why boot()/step() are two separate calls here, unlike the deleted SimApi's
               // single overloaded step().
  checkTrue(sim.booted(), "booted() true after boot()");
  checkTrue(sim.motorLeft().connected(), "left motor connected after boot");
  checkTrue(sim.motorRight().connected(), "right motor connected after boot");

  sim.step(3);  // a few idle cycles -- a silent host, parked robot
  std::vector<DecodedLine> idleFrames = onlyTelemetry(sim.drainTelemetry());
  checkTrue(idleFrames.empty(), "issue Part 8 #1: silent host + parked robot -> zero unsolicited telemetry "
                                "frames after boot (kAuto default, everMoved_ still false)");

  // Bare TLM (reason 1, honored in every mode) forces exactly one frame --
  // use it to confirm connectivity is really up, now that there is no
  // unsolicited boot-ready frame to observe it on.
  sim.injectCommand("TLM");
  sim.step(1);
  std::vector<DecodedLine> frames = onlyTelemetry(sim.drainTelemetry());
  checkTrue(!frames.empty(), "the bare TLM request produced telemetry");

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
  // 124-008 (issue §B3): velocity is a raw sint32 wire int (0.1mm/s scale)
  // -- unpackVelocity() is the GENERATED conversion; position is scale=1.0
  // (raw == real mm, safe as-is).
  for (const auto& f : frames) {
    if (first) {
      firstVelLeft = msg::EncoderReading::unpackVelocity(f.telemetry.enc_left.velocity);
      firstEncLeft = f.telemetry.enc_left.position;
      first = false;
    }
    lastVelLeft = msg::EncoderReading::unpackVelocity(f.telemetry.enc_left.velocity);
    lastVelRight = msg::EncoderReading::unpackVelocity(f.telemetry.enc_right.velocity);
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

  // injectEstop(), not injectStop(): this scenario asserts that the
  // drivetrain goes INACTIVE promptly, which is the ESTOP contract since
  // the command-ingestion rework (command-ingestion-ring-buffered-comms-
  // subsystem-routing-two-stops.md §2). STOP is now a QUEUED, planned stop
  // -- it makes the planner active until it lands, so the very thing this
  // scenario checks would (correctly) not hold for it.
  sim.injectEstop(/*corrId=*/99);
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
//    with SimHarness::kCycleDtUs=40ms (118 ticket 003) meets the stop
//    condition ~3 cycles after activation.
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
//    assertion on the schedule's own NUMBERS: a future change that
//    re-introduces an unabsorbed, additive pacing block fails this
//    checkTrue -- and therefore fails `uv run python -m pytest` -- not
//    just a future bench session.
//
//    131-005 REWRITE (2026-08-02): the trailing pacing block's own wait
//    changed from a FIXED, precomputed budget (the deleted `kPace = kCycle
//    - kWindows`, a gap relative to that block's own entry mark) to an
//    ABSOLUTE end-of-cycle deadline (state_.time.cycleStart + kCycle) --
//    see robot_loop.cpp's own runAndWaitUntil()/cycle() call site and
//    robot_loop.h's kCycle doc comment for the defect this fixes (130-011
//    measured a rock-stable 54ms delivered period against a 50ms nominal,
//    traced to the OLD relative-gap scheme letting each block's own
//    rounding/overrun re-add on top of the next block's fixed budget
//    instead of being absorbed by it).
//
//    That change makes this scenario's OLD expected numbers (kPace=38,
//    "virtualCycleMillis == kCycle") the WRONG thing to assert now, for a
//    reason specific to THIS harness rather than to the fix itself:
//    TestSim::SimHarness's own paired SimClock/SimSleeper (sim_clock.h)
//    never advances the fake clock from a sleepMillis() call -- only an
//    explicit setMicros()/advanceMicros() call moves it, and SimHarness's
//    own step() makes exactly ONE such call, BEFORE cycle() runs, never
//    between its four internal pacing blocks (see TickingSleeper's own
//    header comment, app_robot_loop_harness.cpp's SUC-006 scenario, for
//    the identical limitation documented against a different test). So
//    from THIS fake clock's perspective, ZERO time ever elapses between
//    cycleStart and the trailing block's own deadline check, regardless of
//    what the three earlier blocks' bodies did -- the absolute-deadline
//    wait therefore always computes the FULL kCycle=50ms remaining here,
//    not kCycle-minus-windows. That is a property of this fake clock, not
//    a regression in the fix: 131-005's own new host-build unit test
//    (src/tests/sim/unit/app_robot_loop_pacing_harness.cpp) uses a Sleeper
//    that DOES advance its paired clock on every sleepMillis() call (an
//    injected, jittery one), and is where the actual mean-converges-to-
//    kCycle claim is proven -- this scenario cannot prove it and no longer
//    tries to.
//
//    What THIS scenario keeps verifying, and still genuinely can: exactly
//    four Sleeper::sleepMillis() calls per cycle (the schedule's own
//    SHAPE, unaffected by which of the two pacing schemes computed the
//    trailing call's own argument) and no direct yield() call.
// ===========================================================================

void scenarioVirtualCycleTimingDiagnostic() {
  beginScenario("timing: virtual-cycle schedule makes exactly 4 sleepMillis() calls per cycle; "
                "the trailing call targets kCycle directly (131-005 absolute-deadline pacing)");

  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();

  // Reproduces the deleted SimApi::measureOneCycle()'s own deltas directly
  // off Devices::Sleeper (sim.sleeper(), added to SimHarness by ticket
  // 106-001).
  TestSim::SimSleeper& sleeper = sim.sleeper();
  int sleepsBefore = sleeper.sleepCount();
  int yieldsBefore = sleeper.yieldCount();

  sim.step(1);

  int sleepCount = sleeper.sleepCount() - sleepsBefore;
  uint32_t lastSleepMillis = sleeper.lastSleepMillis();
  int yieldCount = sleeper.yieldCount() - yieldsBefore;
  // robot_loop.cpp's own current kSettle/kClear/RobotLoop::kCycle (see
  // this scenario's own file-header comment for the duplication rationale
  // and the 131-005 REWRITE note). kPace (kCycle - kWindows) is GONE from
  // production code as of 131-005 -- the trailing block no longer
  // computes a fixed budget of its own, so there is no local mirror of it
  // to duplicate here any more.
  constexpr uint32_t kSettle = 4;  // [ms] mirrors robot_loop.cpp's own kSettle
  constexpr uint32_t kClear = 4;   // [ms] mirrors robot_loop.cpp's own kClear
  constexpr uint32_t kCycle = 50;  // [ms] mirrors robot_loop.cpp's own RobotLoop::kCycle

  checkTrue(sleepCount == 4,
            "exactly 4 Sleeper::sleepMillis() calls per cycle() (3 runAndWait blocks + the "
            "trailing runAndWaitUntil() block)");
  checkTrue(lastSleepMillis == kCycle,
            "the trailing block's own sleepMillis() call requests kCycle=50ms under THIS "
            "harness's fake clock -- see this scenario's own file-header 131-005 REWRITE note: "
            "SimHarness's SimClock/SimSleeper pair never advances BETWEEN a cycle's own four "
            "pacing blocks, so the absolute deadline (cycleStart+kCycle) always reads ZERO "
            "elapsed at the trailing block regardless of the three earlier blocks' own "
            "requested sleeps -- a property of this fake clock, not of the fix. "
            "app_robot_loop_pacing_harness.cpp's own injected-jitter test is where the actual "
            "convergence-to-kCycle claim is proven");
  checkTrue(yieldCount == 0, "RobotLoop::cycle() never calls Sleeper::yield() directly");

  std::printf(
      "  TIMING: sleepCount=%d lastSleepMillis=%ums yieldCount=%d (kSettle=%ums kClear=%ums "
      "kCycle=%ums -- 131-005: the trailing block targets kCycle directly as an absolute "
      "deadline; see this scenario's own REWRITE note for why lastSleepMillis==kCycle here "
      "rather than the old kCycle-kWindows)\n",
      sleepCount, static_cast<unsigned>(lastSleepMillis), yieldCount,
      static_cast<unsigned>(kSettle), static_cast<unsigned>(kClear), static_cast<unsigned>(kCycle));
}

// ===========================================================================
// 6. No MicroBit.h dependency (105-004 AC #4) -- this is a static/compile-
//    level property (proven by test_sim_api.py's own HOST_BUILD compile
//    step succeeding with no MicroBit.h anywhere in the include graph), not
//    something a runtime scenario can assert -- recorded here as a comment
//    landmark matching every sibling ticket's own harness convention (e.g.
//    app_robot_loop_harness.cpp's identical note).
// ===========================================================================

// ===========================================================================
// 7. 131-001 (SUC-131-001, sprint success criteria): a chained, multi-leg
//    MOVE sequence (2+ legs, "equivalent to a tour") holds a converged
//    Stage C bias across EVERY leg boundary -- through the REAL
//    RobotLoop::handleMove() call site (robot_loop.cpp:227), not a
//    hand-invoked App::Drive::takeover() call (that unit-level proof
//    already lives in app_drive_harness.cpp's own
//    scenarioBiasPersistsAcrossChainedTakeoverBoundaries()).
//    configureSimForBenchTest() installs the EXACT calibration inverse of
//    this plant's own linear response (bench_test_config.cpp), so Stage C
//    would have nothing to close by default -- this scenario deliberately
//    re-installs a DELIBERATELY WRONG (20% low) duty-per-speed calibration
//    on top of it, giving Stage C a real, nonzero error to adapt against
//    under the REAL WheelPlant physics, mirroring app_drive_harness.cpp's
//    own hand-simulated `plantGain` scenarios but through the real
//    plant/planner/RobotLoop stack instead of a hand-rolled one.
// ===========================================================================

void scenarioBiasPersistsAcrossChainedMoveLegs() {
  beginScenario("131-001: a chained, multi-leg MOVE sequence (2+ legs) holds a converged Stage C "
                "bias across every leg boundary -- RobotLoop::handleMove()'s drive_.takeover() "
                "call does not reset it, unlike the drive_.estop() call it replaces");

  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);

  // Deliberately mis-calibrate Stage A below the REAL plant's linear
  // response (configureSimForBenchTest() installs the EXACT inverse,
  // 1/kDefaultDutyVelMax) -- 80% of the true value under-predicts the duty
  // needed for a given commanded speed, so the wheel genuinely
  // under-delivers and Stage C has a real, nonzero error to close.
  const float trueKff = 1.0f / TestSim::kDefaultDutyVelMax;
  sim.drive().setDutyPerSpeed(trueKff * 0.8f, trueKff * 0.8f);

  App::Drive::AdaptationBounds bounds;
  bounds.vMin = 0.0f;
  bounds.biasMax = 80.0f;   // [mm/s]
  bounds.tauAdapt = 1.0f;   // [s] -- converges within a few seconds of virtual cruise
  bounds.aSteady = 1.0e6f;  // effectively unshaped, matching configureSimForBenchTest()'s own
                            // unshaped shaper ceilings -- always "steady" outside the one-cycle
                            // ramp instant
  sim.drive().setAdaptationBounds(bounds);

  sim.boot();

  // Leg 1: a long TIME-stop cruise, long enough for Stage C to converge
  // well away from 0 under the deliberate 20% under-calibration above.
  sim.injectMove(/*v_x=*/200.0f, /*v_y=*/0.0f, /*omega=*/0.0f, TestSupport::MoveStopKind::kTime,
                 /*stopValue=*/8000.0f, /*timeout=*/20000.0f, /*replace=*/true, /*id=*/1,
                 /*corrId=*/1);
  sim.step(160);  // 160 * 50ms == 8s of virtual cruise -- comfortably past tauAdapt=1s
  (void)sim.drainTelemetry();

  const float biasAfterLeg1 = sim.drive().biasLeft();
  checkTrue(std::fabs(biasAfterLeg1) > 5.0f,
            "setup: Stage C genuinely converged away from 0 under the deliberate mis-calibration");

  // Leg 2 -- a SECOND, chained MOVE (a fresh id; leg 1's own TIME stop
  // condition has already ended it). This is the EXACT RobotLoop::
  // handleMove() call site this ticket changes (robot_loop.cpp:227):
  // drive_.takeover(), not drive_.estop().
  sim.injectMove(180.0f, 0.0f, 0.0f, TestSupport::MoveStopKind::kTime, /*stopValue=*/8000.0f,
                 /*timeout=*/20000.0f, /*replace=*/true, /*id=*/2, /*corrId=*/2);
  sim.step(1);  // the cycle handleMove() actually runs on
  checkFloatGe(std::fabs(sim.drive().biasLeft()), 5.0f,
               "leg boundary 1: bias did NOT reset to 0 at the SECOND leg's own handleMove()/"
               "takeover() call");

  sim.step(40);  // ~2s further into leg 2

  // Leg 3 -- a THIRD chained MOVE (2+ leg boundaries, "equivalent to a
  // tour" per this ticket's own acceptance criterion).
  sim.injectMove(160.0f, 0.0f, 0.0f, TestSupport::MoveStopKind::kTime, /*stopValue=*/8000.0f,
                 /*timeout=*/20000.0f, /*replace=*/true, /*id=*/3, /*corrId=*/3);
  sim.step(1);
  checkFloatGe(std::fabs(sim.drive().biasLeft()), 5.0f,
               "leg boundary 2: bias STILL has not reset to 0 at the THIRD leg's boundary -- 2+ "
               "chained legs, no reset anywhere along the tour");
}

}  // namespace

int main() {
  scenarioBootCompletesThroughRealRobotLoop();
  scenarioTwistDrivesRealPlantRamp();
  scenarioStopAcksAndClearsActive();
  scenarioMoveExpiryStopsPlantWithNoFurtherHostTraffic();
  scenarioVirtualCycleTimingDiagnostic();
  scenarioBiasPersistsAcrossChainedMoveLegs();

  if (g_failureCount == 0) {
    std::printf("OK: all sim_api scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the sim_api scenarios\n", g_failureCount);
  return 1;
}
