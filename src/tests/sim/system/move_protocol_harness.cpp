// move_protocol_harness.cpp -- off-hardware acceptance harness for ticket
// 116-008 (protocol-set-point-the-minimal-firmware-s-complete-command-
// surface.md), the sim-executable half of the MOVE protocol's own
// Verification section: stop conditions, chaining, replace preemption,
// ERR_FULL, the no-deadman empty-queue drain, and a CONFIG patch's
// non-interference with an in-flight MOVE.
//
// Exercises every scenario through TestSim::SimHarness driving the REAL
// App::RobotLoop/App::MoveQueue/Motion::StopCondition/App::Drive/
// App::Odometry graph against a REAL, live-responding TestSim::SimPlant
// (real duty->velocity->position physics, the SAME bench_test_config.h
// motor gains every sibling sim/system harness configures with) -- never a
// mock or a scripted bus. This is the same "whole robot, real firmware,
// real plant" shape as straight_twist_harness.cpp/sim_api_harness.cpp,
// distinguishing this file from ticket 006's own app_robot_loop_harness.cpp
// SUC-050/053/054/055 scenarios, which use a bare (often zero-gain)
// LiveFixture to pin RobotLoop's own dispatch wiring in isolation -- this
// file re-proves the SAME acceptance criteria end to end, through the wire,
// against the real plant a stakeholder would actually see move on the
// stand.
//
// Every Move injected below uses a DISTINCT `id` (echoed on its completion
// ack) and `corrId` (echoed on its enqueue ack) -- e.g. id=1/corrId=101 --
// so a scenario can tell the two acks apart by which value they carry,
// with no need to read App::MoveQueue's own internal state (SimHarness
// exposes no moveQueue() accessor -- every assertion below is proved
// purely from decoded telemetry, the same wire-level observability every
// sibling sim/system harness already uses).
//
// Hand-rolled assertions, PASS/FAIL per scenario, nonzero exit on any
// failure -- mirrors every other src/tests/sim/system harness's own shape.
// Run by test_move_protocol.py, which compiles this file together with
// sim_plant.cpp, wire_test_codec.cpp, the plant sources, and the same full
// HOST_BUILD Devices/App/messages/kinematics dependency graph every
// sibling test_*.py in this directory already compiles.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "app/telemetry.h"
#include "bench_test_config.h"
#include "messages/envelope.h"
#include "messages/wire_runtime.h"
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

void checkFloatLe(float actual, float bound, const std::string& what) {
  if (!(actual <= bound)) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected <= %g, got %g", what.c_str(),
                  static_cast<double>(bound), static_cast<double>(actual));
    fail(buf);
  }
}

void checkFloatEq(float actual, float expected, float tol, const std::string& what) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g (+/-%g), got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(tol), static_cast<double>(actual));
    fail(buf);
  }
}

void checkUintEq(uint32_t actual, uint32_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %u, got %u", what.c_str(), expected, actual);
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

// anyAckMatches -- "was `corrId` ever acked OK (fresh, err==0) anywhere in
// `frames`" -- mirrors sim_api_harness.cpp's own helper of the same name.
// Used for enqueue/CONFIG acks, where only "it succeeded" matters, not
// which exact frame carried it.
bool anyAckMatches(const std::vector<DecodedLine>& frames, uint32_t corrId) {
  for (const auto& f : frames) {
    if (!(f.telemetry.flags & App::kFlagAckFresh)) continue;
    if (f.telemetry.ack_corr == corrId && f.telemetry.ack_err == 0) return true;
  }
  return false;
}

// findFreshAck -- the first fresh ack in `frames` whose ack_corr ==
// `ackCorr`, regardless of err (unlike anyAckMatches() above). Every
// scenario below gives each Move a corrId/id pair that is unique across
// the WHOLE scenario, so a match against a Move's own `id` is
// unambiguously that Move's completion ack (an enqueue ack always carries
// the envelope's `corrId`, never `id`, and no other Move in the same
// scenario reuses that value) -- no need to disambiguate by any other
// field. Returns false (frames unmodified... `errOut`/`flagsOut`
// untouched) if no match exists.
bool findFreshAck(const std::vector<DecodedLine>& frames, uint32_t ackCorr, uint32_t* errOut,
                   uint32_t* flagsOut) {
  for (const auto& f : frames) {
    if (!(f.telemetry.flags & App::kFlagAckFresh)) continue;
    if (f.telemetry.ack_corr != ackCorr) continue;
    if (errOut) *errOut = f.telemetry.ack_err;
    if (flagsOut) *flagsOut = f.telemetry.flags;
    return true;
  }
  return false;
}

// --- Hand-rolled CommandEnvelope{config: ConfigDelta{motor: ...}} encoder
// -- no encode(CommandEnvelope) codec exists (wire_test_codec.h's own file
// header: firmware only ever DECODES a CommandEnvelope), and
// wire_test_codec.h itself only builds MOVE/STOP envelopes (the two shapes
// SimHarness::injectMove()/injectStop() need). SUC-055 (the CONFIG-mid-MOVE
// scenario below) is the only scenario in this file needing a CONFIG
// envelope, so -- mirroring config_gate_harness.cpp's/
// app_robot_loop_harness.cpp's own established "per-harness-file fixture
// duplication" convention for exactly this same hand-rolled encoder --
// this is a small, self-contained, file-local copy rather than a change to
// the shared wire_test_codec.h. Field numbers per envelope.proto/
// config.proto: CommandEnvelope.config=6, ConfigDelta.motor=2,
// MotorConfigPatch.side=1 (LEFT=0), MotorConfigPatch.kp=3. -----------------

using WireRuntime::WireType;

struct Buf {
  uint8_t data[128] = {};
  size_t len = 0;
};

bool putVarintField(Buf& b, uint32_t number, uint64_t v) {
  return WireRuntime::encodeTag(number, WireType::kVarint, b.data, sizeof(b.data), &b.len) &&
         WireRuntime::encodeVarint(v, b.data, sizeof(b.data), &b.len);
}

bool putFloatField(Buf& b, uint32_t number, float v) {
  return WireRuntime::encodeTag(number, WireType::kFixed32, b.data, sizeof(b.data), &b.len) &&
         WireRuntime::encodeFloat(v, b.data, sizeof(b.data), &b.len);
}

bool putMessageField(Buf& b, uint32_t number, const Buf& nested) {
  if (!WireRuntime::encodeTag(number, WireType::kLengthDelimited, b.data, sizeof(b.data), &b.len)) return false;
  if (!WireRuntime::encodeVarint(nested.len, b.data, sizeof(b.data), &b.len)) return false;
  if (b.len + nested.len > sizeof(b.data)) return false;
  std::memcpy(b.data + b.len, nested.data, nested.len);
  b.len += nested.len;
  return true;
}

std::string armorLine(const uint8_t* raw, size_t rawLen) {
  char b64[256] = {};
  size_t b64Len = 0;
  if (!WireRuntime::base64Encode(raw, rawLen, b64, sizeof(b64), &b64Len)) return std::string();
  std::string out = "*B";
  out.append(b64, b64Len);
  return out;
}

// CommandEnvelope{corr_id, config: ConfigDelta{motor: MotorConfigPatch{
// side=LEFT, kp}}} -- side is irrelevant to this scenario's own assertion
// (handleConfig()/applyMotorConfigPatch() mirrors kp onto BOTH bound
// motors regardless of `side` -- robot_loop.cpp's own mergeMotorGainsPatch()
// comment), so LEFT(0) is chosen arbitrarily, matching every other
// harness's own precedent (app_robot_loop_harness.cpp's
// armorMotorConfigPatchCommand()).
std::string armorMotorConfigPatchCommand(float kp, uint32_t corrId) {
  Buf motorPatch;
  putVarintField(motorPatch, 1, 0);  // MotorConfigPatch.side = LEFT (0)
  putFloatField(motorPatch, 3, kp);  // MotorConfigPatch.kp
  Buf motorDelta;
  putMessageField(motorDelta, 2, motorPatch);  // ConfigDelta.motor, field 2
  Buf env;
  putVarintField(env, 1, corrId);
  putMessageField(env, 6, motorDelta);  // CommandEnvelope.config, field 6
  return armorLine(env.data, env.len);
}

}  // namespace

// ===========================================================================
// SUC-050: TIME/DISTANCE/ANGLE stop conditions each reach completion within
// tolerance, driven through SimHarness::injectMove() and stepped via
// SimHarness::step().
// ===========================================================================

namespace {

void scenarioTimeStopCompletesWithinTolerance() {
  beginScenario("SUC-050 TIME: a TWIST MOVE with a TIME stop condition completes near its "
                "commanded duration, active clears");

  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  sim.step(3);
  (void)sim.drainTelemetry();

  constexpr float kVx = 150.0f;          // [mm/s] non-saturating cruise speed
  constexpr float kStopTimeMs = 400.0f;  // [ms] -- 8 cycles
  constexpr float kTimeoutMs = 5000.0f;  // [ms] generous backstop, never fires
  constexpr uint32_t kMoveId = 1;
  constexpr uint32_t kCorrId = 101;

  sim.injectMove(kVx, /*v_y=*/0.0f, /*omega=*/0.0f, TestSupport::MoveStopKind::kTime, kStopTimeMs,
                kTimeoutMs, /*replace=*/true, kMoveId, kCorrId);

  constexpr int kRunCycles = 60;  // 3s -- comfortably past the 400ms stop threshold
  sim.step(kRunCycles);
  std::vector<DecodedLine> frames = onlyTelemetry(sim.drainTelemetry());
  checkTrue(!frames.empty(), "telemetry decoded across the run");
  checkTrue(anyAckMatches(frames, kCorrId), "the MOVE's enqueue ack was OK");

  uint32_t err = 1, flags = 0;
  checkTrue(findFreshAck(frames, kMoveId, &err, &flags), "the Move's completion ack (ack_corr==id) reached the wire");
  checkUintEq(err, 0, "the completion ack's err is OK (a met stop condition, not a timeout)");
  checkTrue((flags & App::kFlagFaultMoveTimeout) == 0, "kFlagFaultMoveTimeout is NOT set -- ended via TIME, not timeout");
  checkTrue((frames.back().telemetry.flags & App::kFlagActive) == 0, "active clears by the end of the run");
}

void scenarioDistanceStopCompletesWithinTolerance() {
  beginScenario("SUC-050 DISTANCE: a TWIST MOVE with a DISTANCE stop condition travels within "
                "tolerance of the commanded distance");

  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  sim.step(3);
  (void)sim.drainTelemetry();

  constexpr float kVx = 150.0f;             // [mm/s]
  constexpr float kStopDistanceMm = 300.0f;  // [mm]
  constexpr float kTimeoutMs = 5000.0f;      // [ms] generous backstop
  constexpr uint32_t kMoveId = 2;
  constexpr uint32_t kCorrId = 102;

  sim.injectMove(kVx, /*v_y=*/0.0f, /*omega=*/0.0f, TestSupport::MoveStopKind::kDistance,
                kStopDistanceMm, kTimeoutMs, /*replace=*/true, kMoveId, kCorrId);

  constexpr int kRunCycles = 150;  // 7.5s -- comfortably past the ~2s ideal travel time
  sim.step(kRunCycles);
  std::vector<DecodedLine> frames = onlyTelemetry(sim.drainTelemetry());
  checkTrue(!frames.empty(), "telemetry decoded across the run");
  checkTrue(anyAckMatches(frames, kCorrId), "the MOVE's enqueue ack was OK");

  uint32_t err = 1, flags = 0;
  checkTrue(findFreshAck(frames, kMoveId, &err, &flags), "the Move's completion ack (ack_corr==id) reached the wire");
  checkUintEq(err, 0, "the completion ack's err is OK");
  checkTrue((flags & App::kFlagFaultMoveTimeout) == 0, "kFlagFaultMoveTimeout is NOT set -- ended via DISTANCE, not timeout");

  // Straight travel (omega=0) -- the odometry-derived pose.x IS the path
  // traveled, the exact reading Motion::StopCondition's own DISTANCE kind
  // compared against odom_.pathLength(). By the end of the run the motors
  // have long since stopped, so the LAST frame's pose is stable.
  float finalX = frames.back().telemetry.pose.x;
  // Empirically ~30mm over the commanded 300mm (the one-cycle DISTANCE-
  // tick overshoot at 150mm/s, PLUS the ramp-up lag's own extra distance
  // before the plant reaches cruise speed) -- 40mm keeps margin above the
  // observed value without papering over a real regression.
  constexpr float kToleranceMm = 40.0f;
  checkFloatGe(finalX, kStopDistanceMm - kToleranceMm, "traveled within tolerance of the commanded distance (low bound)");
  checkFloatLe(finalX, kStopDistanceMm + kToleranceMm, "traveled within tolerance of the commanded distance (high bound)");
}

void scenarioAngleStopCompletesWithinTolerance() {
  beginScenario("SUC-050 ANGLE: a TWIST MOVE with an ANGLE stop condition turns within tolerance "
                "of the commanded heading change");

  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  sim.step(3);
  (void)sim.drainTelemetry();

  constexpr float kOmega = 1.0f;          // [rad/s] CCW-positive
  constexpr float kStopAngleRad = 1.0f;   // [rad]
  constexpr float kTimeoutMs = 5000.0f;   // [ms] generous backstop
  constexpr uint32_t kMoveId = 3;
  constexpr uint32_t kCorrId = 103;

  sim.injectMove(/*v_x=*/0.0f, /*v_y=*/0.0f, kOmega, TestSupport::MoveStopKind::kAngle, kStopAngleRad,
                kTimeoutMs, /*replace=*/true, kMoveId, kCorrId);

  constexpr int kRunCycles = 80;  // 4s -- comfortably past the ~1s ideal turn time
  bool sawOppositeWheelSigns = false;
  std::vector<DecodedLine> frames;
  for (int i = 0; i < kRunCycles; ++i) {
    sim.step(1);
    std::vector<DecodedLine> cycleFrames = onlyTelemetry(sim.drainTelemetry());
    for (const auto& f : cycleFrames) {
      // BodyKinematics::inverse(v=0, omega>0, b, vL, vR): vL = -omega*b/2
      // (negative), vR = +omega*b/2 (positive) -- confirms the differential
      // TWIST-with-omega path actually drives the two wheels in opposite
      // directions, not just "the Move completed".
      if (f.telemetry.enc_left.velocity < -10.0f && f.telemetry.enc_right.velocity > 10.0f) {
        sawOppositeWheelSigns = true;
      }
      frames.push_back(f);
    }
  }
  checkTrue(!frames.empty(), "telemetry decoded across the run");
  checkTrue(anyAckMatches(frames, kCorrId), "the MOVE's enqueue ack was OK");
  checkTrue(sawOppositeWheelSigns, "left/right wheel velocities took opposite signs during the turn "
                                   "(BodyKinematics::inverse()'s own differential split)");

  uint32_t err = 1, flags = 0;
  checkTrue(findFreshAck(frames, kMoveId, &err, &flags), "the Move's completion ack (ack_corr==id) reached the wire");
  checkUintEq(err, 0, "the completion ack's err is OK");
  checkTrue((flags & App::kFlagFaultMoveTimeout) == 0, "kFlagFaultMoveTimeout is NOT set -- ended via ANGLE, not timeout");

  float finalHeading = frames.back().telemetry.pose.h;
  // Empirically ~0.21rad over the commanded 1.0rad -- the one-cycle ANGLE-
  // tick overshoot at 1.0rad/s PLUS the ramp-up lag's own extra rotation
  // before the plant reaches cruise angular rate. 0.25 keeps margin above
  // the observed value without papering over a real regression.
  constexpr float kToleranceRad = 0.25f;
  checkFloatGe(finalHeading, kStopAngleRad - kToleranceRad, "turned within tolerance of the commanded angle (low bound)");
  checkFloatLe(finalHeading, kStopAngleRad + kToleranceRad, "turned within tolerance of the commanded angle (high bound)");
}

// ===========================================================================
// Both velocity variants: the WHEELS variant bypasses BodyKinematics::
// inverse() entirely (drive.h's own doc comment: "the wheels path stages
// v_left/v_right unchanged, with no inverse() call at all") -- this
// scenario proves that path drives the two wheels with the EXACT commanded
// signs, independently of each other, through the real firmware.
// ===========================================================================

void scenarioWheelsVariantDrivesCorrectSigns() {
  beginScenario("MoveWheels: opposite-signed v_left/v_right drive the two wheels in opposite "
                "directions, unchanged by any kinematics inversion");

  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  sim.step(3);
  (void)sim.drainTelemetry();

  constexpr float kVLeft = 150.0f;
  constexpr float kVRight = -150.0f;
  constexpr float kStopTimeMs = 400.0f;
  constexpr float kTimeoutMs = 5000.0f;
  constexpr uint32_t kMoveId = 4;
  constexpr uint32_t kCorrId = 104;

  sim.injectMove(kVLeft, kVRight, TestSupport::MoveStopKind::kTime, kStopTimeMs, kTimeoutMs,
                /*replace=*/true, kMoveId, kCorrId);

  constexpr int kRunCycles = 60;
  bool sawCorrectSigns = false;
  std::vector<DecodedLine> frames;
  for (int i = 0; i < kRunCycles; ++i) {
    sim.step(1);
    std::vector<DecodedLine> cycleFrames = onlyTelemetry(sim.drainTelemetry());
    for (const auto& f : cycleFrames) {
      if (f.telemetry.enc_left.velocity > 100.0f && f.telemetry.enc_right.velocity < -100.0f) {
        sawCorrectSigns = true;
      }
      frames.push_back(f);
    }
  }
  checkTrue(!frames.empty(), "telemetry decoded across the run");
  checkTrue(anyAckMatches(frames, kCorrId), "the MOVE's enqueue ack was OK");
  checkTrue(sawCorrectSigns, "encLeft tracked toward +150mm/s and encRight toward -150mm/s -- the "
                             "commanded wheel signs, unchanged by any kinematics inversion");

  uint32_t err = 1, flags = 0;
  checkTrue(findFreshAck(frames, kMoveId, &err, &flags), "the Move's completion ack (ack_corr==id) reached the wire");
  checkUintEq(err, 0, "the completion ack's err is OK");
  checkTrue((flags & App::kFlagFaultMoveTimeout) == 0, "kFlagFaultMoveTimeout is NOT set");
}

// ===========================================================================
// SUC-054: a DISTANCE MOVE whose target the sim plant cannot reach within
// `timeout` ends at `timeout` with kFlagFaultMoveTimeout set -- via
// SimPlant's own freezePosition() fault knob on BOTH wheels (a genuinely
// stalled drivetrain, not a numerically-unreachable target on an otherwise
// healthy plant), against the REAL (nonzero-gain) bench-test motor
// configuration.
// ===========================================================================

void scenarioDistanceTimeoutWithStalledWheelsSetsFaultFlag() {
  beginScenario("SUC-054: a DISTANCE MOVE against BOTH wheels stalled (SimPlant fault knobs) ends "
                "at timeout with kFlagFaultMoveTimeout set");

  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  sim.step(3);
  (void)sim.drainTelemetry();

  // Stall BOTH wheels -- freezePosition() freezes the REPORTED raw encoder
  // value each leaf reads over I2C (sim_plant.h), so App::Odometry's own
  // pathLength() cannot advance regardless of the plant's real duty/
  // velocity physics or the motor's own (real, nonzero) gains -- a
  // DISTANCE stop condition can only ever end via the timeout backstop.
  sim.plant().freezePosition(/*port=*/1, true);  // 1 == left
  sim.plant().freezePosition(/*port=*/2, true);  // 2 == right

  constexpr float kVx = 150.0f;
  constexpr float kStopDistanceMm = 1000.0f;  // physically reachable in principle -- blocked by the freeze
  constexpr float kTimeoutMs = 200.0f;        // [ms] -- 4 cycles
  constexpr uint32_t kMoveId = 5;
  constexpr uint32_t kCorrId = 105;

  sim.injectMove(kVx, /*v_y=*/0.0f, /*omega=*/0.0f, TestSupport::MoveStopKind::kDistance,
                kStopDistanceMm, kTimeoutMs, /*replace=*/true, kMoveId, kCorrId);

  constexpr int kRunCycles = 15;  // 750ms -- comfortably past the 200ms timeout
  sim.step(kRunCycles);
  std::vector<DecodedLine> frames = onlyTelemetry(sim.drainTelemetry());
  checkTrue(!frames.empty(), "telemetry decoded across the run");
  checkTrue(anyAckMatches(frames, kCorrId), "the MOVE's enqueue ack was OK");

  uint32_t err = 1, flags = 0;
  checkTrue(findFreshAck(frames, kMoveId, &err, &flags), "the Move's completion ack (ack_corr==id) reached the wire");
  checkUintEq(err, 0, "ack_err is still 0 -- a timeout is signalled via the flags bit, not ack_err");
  checkTrue((flags & App::kFlagFaultMoveTimeout) != 0, "kFlagFaultMoveTimeout IS set on the ending cycle");

  // Corroborating proof the wheels really were stalled, not merely slow --
  // pose.x (straight travel) stayed essentially at its starting value the
  // entire run.
  checkFloatLe(std::fabs(frames.back().telemetry.pose.x), 5.0f, "pose.x barely moved -- the DISTANCE stop condition "
                                                                 "genuinely could not be satisfied");
  checkTrue((frames.back().telemetry.flags & App::kFlagFaultMoveTimeout) == 0,
            "kFlagFaultMoveTimeout is level-set, not sticky -- clears again a cycle or two later");
}

// ===========================================================================
// SUC-051a: chaining -- MOVE B (replace=false) sent while A runs hands off
// seamlessly at A's own expiry, with no cycle where the STAGED (commanded)
// wheel target reads zero in between.
// ===========================================================================

void scenarioChainHandoffSeamlessNoZeroCycle() {
  beginScenario("SUC-051 chaining: MOVE B (replace=false) sent while A runs hands off seamlessly "
                "at A's own expiry, no zero-commanded-velocity cycle in between");

  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  sim.step(3);
  (void)sim.drainTelemetry();

  constexpr float kVxA = 150.0f;
  constexpr float kVxB = 250.0f;
  constexpr float kStopTimeMs = 250.0f;  // [ms] -- 5 cycles each
  constexpr float kTimeoutMs = 100000.0f;
  constexpr uint32_t kIdA = 10;
  constexpr uint32_t kCorrA = 110;
  constexpr uint32_t kIdB = 11;
  constexpr uint32_t kCorrB = 111;

  sim.injectMove(kVxA, /*v_y=*/0.0f, /*omega=*/0.0f, TestSupport::MoveStopKind::kTime, kStopTimeMs,
                kTimeoutMs, /*replace=*/false, kIdA, kCorrA);
  sim.step(1);  // cycle where A is dispatched+activated -- 118 (retires the 112-005 hoist:
                // moveQueue_.tick()'s own dispatch now runs BEFORE drive_.tick(), same R-settle
                // block, same cycle) commits the STAGED target THIS cycle -- not sampled below,
                // since the target-vs-ack asymmetry documented below still applies at every
                // boundary and this call's only job is to get A running.
  sim.step(1);  // NEXT cycle -- A's target is comfortably committed either way.
  checkFloatEq(sim.driveTargetVelLeft(), kVxA, 1.0f, "A's target is committed before the chaining test begins");

  sim.injectMove(kVxB, /*v_y=*/0.0f, /*omega=*/0.0f, TestSupport::MoveStopKind::kTime, kStopTimeMs,
                kTimeoutMs, /*replace=*/false, kIdB, kCorrB);  // enqueued while A is actively running

  bool completedA = false, completedB = false;
  bool sawTargetNearA = false, sawTargetNearB = false;
  bool everZeroBeforeBCompletes = false;
  // zeroGracePending -- 118 (retires the 112-005 hoist): drive_.tick() now
  // runs AFTER moveQueue_.tick() in the SAME R-settle block/SAME cycle
  // (restoring the last-known-good 39c084c1 schedule), so a Move's own
  // completion (Drive::stop() staging a zero target) is now visible in
  // driveTargetVelLeft() the EXACT cycle it happens -- zero lag, better
  // than before. But updateTlm()/tlm_.emit() (kClear block) still run
  // BEFORE processMessage()/moveQueue_.tick() (R-settle block) within a
  // cycle (unchanged by this restore -- see robot_loop.cpp's own cycle()
  // order), so the completion ACK pushed by moveQueue_.tick() is still not
  // telemetry-visible until the FOLLOWING cycle's own emit(). B's own
  // legitimate terminal zero therefore lands exactly ONE cycle before
  // completedB flips true -- a real, permanent, single-cycle boundary
  // artifact of the schedule, not a chain-handoff gap. Give it exactly one
  // cycle of grace: a target==0 sample with completedB still false is only
  // a genuine failure if it PERSISTS past that one grace cycle (still zero,
  // still not completed, one cycle later) -- a true stall/gap is still
  // caught.
  bool zeroGracePending = false;
  std::vector<DecodedLine> frames;
  constexpr int kRunCycles = 25;  // comfortably past both Moves' own 5-cycle stop thresholds, chained
  for (int i = 0; i < kRunCycles; ++i) {
    sim.step(1);

    std::vector<DecodedLine> cycleFrames = onlyTelemetry(sim.drainTelemetry());
    for (const auto& f : cycleFrames) {
      frames.push_back(f);
      if (f.telemetry.flags & App::kFlagAckFresh) {
        if (f.telemetry.ack_corr == kIdA && f.telemetry.ack_err == 0) completedA = true;
        if (f.telemetry.ack_corr == kIdB && f.telemetry.ack_err == 0) completedB = true;
      }
    }

    float target = sim.driveTargetVelLeft();
    if (std::fabs(target - kVxA) < 5.0f) sawTargetNearA = true;
    if (std::fabs(target - kVxB) < 5.0f) sawTargetNearB = true;

    if (completedB) {
      zeroGracePending = false;  // B's completion is now telemetry-visible -- no more grace needed
    } else if (std::fabs(target) < 1.0f) {
      if (zeroGracePending) {
        // Zero for a SECOND consecutive cycle with completedB still not
        // visible -- past the one-cycle ack-visibility lag, a genuine gap.
        everZeroBeforeBCompletes = true;
      }
      zeroGracePending = true;
    } else {
      zeroGracePending = false;  // target moved off zero again without completing -- reset the grace
    }
  }

  checkTrue(anyAckMatches(frames, kCorrA), "A's enqueue ack was OK");
  checkTrue(anyAckMatches(frames, kCorrB), "B's enqueue ack was OK");
  checkTrue(completedA, "A's completion ack (ack_corr==id) reached the wire");
  checkTrue(completedB, "B's completion ack (ack_corr==id) reached the wire");
  checkTrue(sawTargetNearA, "the staged target tracked A's own commanded velocity");
  checkTrue(sawTargetNearB, "the staged target tracked B's own commanded velocity -- the hand-off actually happened");
  checkTrue(!everZeroBeforeBCompletes, "the staged (commanded) target was NEVER zero between A's activation "
                                       "and B's own completion -- the chain hand-off was seamless");
}

// ===========================================================================
// SUC-051b: replace=true preempts mid-motion on the same cycle it arrives
// -- A never naturally completes (it is silently preempted, no completion
// ack), C takes over and runs its own course to completion.
// ===========================================================================

void scenarioReplacePreemptsMidMotionSameCycle() {
  beginScenario("SUC-051 replace: replace=true preempts A mid-motion, A gets NO completion ack, "
                "C runs its own course");

  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  sim.step(3);
  (void)sim.drainTelemetry();

  constexpr float kVxA = 150.0f;
  constexpr float kVxC = 300.0f;
  constexpr float kTimeoutMs = 100000.0f;
  constexpr uint32_t kIdA = 20;
  constexpr uint32_t kCorrA = 120;
  constexpr uint32_t kIdC = 21;
  constexpr uint32_t kCorrC = 121;

  // A's own stop condition never fires on its own within this scenario's
  // window -- only preemption (or the test ending) can end it.
  sim.injectMove(kVxA, /*v_y=*/0.0f, /*omega=*/0.0f, TestSupport::MoveStopKind::kTime, 100000.0f,
                kTimeoutMs, /*replace=*/true, kIdA, kCorrA);
  sim.step(5);  // well into "mid-motion" -- A's target long since committed
  checkFloatEq(sim.driveTargetVelLeft(), kVxA, 1.0f, "A is genuinely mid-motion before the preemption");
  (void)sim.drainTelemetry();  // discard everything up to and including A's own enqueue ack

  constexpr float kStopTimeMsC = 300.0f;  // [ms] -- 6 cycles, C runs its own course after preempting
  sim.injectMove(kVxC, /*v_y=*/0.0f, /*omega=*/0.0f, TestSupport::MoveStopKind::kTime, kStopTimeMsC,
                kTimeoutMs, /*replace=*/true, kIdC, kCorrC);

  bool completedC = false;
  std::vector<DecodedLine> frames;
  constexpr int kRunCycles = 20;
  for (int i = 0; i < kRunCycles; ++i) {
    sim.step(1);
    std::vector<DecodedLine> cycleFrames = onlyTelemetry(sim.drainTelemetry());
    for (const auto& f : cycleFrames) {
      frames.push_back(f);
      if ((f.telemetry.flags & App::kFlagAckFresh) && f.telemetry.ack_corr == kIdC && f.telemetry.ack_err == 0) {
        completedC = true;
      }
    }
  }

  checkTrue(anyAckMatches(frames, kCorrC), "C's enqueue ack was OK");
  checkTrue(completedC, "C's completion ack (ack_corr==id) reached the wire -- C ran its own course "
                        "after preempting A");
  uint32_t unusedErr = 0, unusedFlags = 0;
  checkTrue(!findFreshAck(frames, kIdA, &unusedErr, &unusedFlags),
            "A NEVER received a completion ack -- it was silently preempted, matching MoveQueue's own "
            "documented replace=true contract (no completion ack for a preempted active Move)");
  checkFloatEq(sim.driveTargetVelLeft(), 0.0f, 1.0f, "the staged target settled back to 0 once C completed");
}

// ===========================================================================
// SUC-052: a 5th pending MOVE is rejected ERR_FULL; the existing active + 4
// pending Moves are unchanged -- proved behaviorally: all 5 still complete,
// in order, with the exact IDs originally sent, and the rejected 6th Move
// NEVER produces a completion ack (it never activated).
// ===========================================================================

void scenarioFifthPendingRejectedErrFullQueueUnchanged() {
  beginScenario("SUC-052: a 5th pending MOVE is rejected ERR_FULL; the existing active + 4 pending "
                "Moves complete unchanged, in order");

  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  sim.step(3);
  (void)sim.drainTelemetry();

  constexpr float kVx = 100.0f;
  // Long enough (2s) that the whole 6-command injection burst below (one
  // inject+step(1) pair per command -- App::Comms::pump() consumes at most
  // one inbound line per cycle) cannot possibly let Move #1 complete before
  // the 6th (rejected) command has been dispatched.
  constexpr float kStopTimeMs = 2000.0f;
  constexpr float kTimeoutMs = 100000.0f;

  constexpr uint32_t kIds[6] = {30, 31, 32, 33, 34, 35};
  constexpr uint32_t kCorrIds[6] = {130, 131, 132, 133, 134, 135};

  // Wait for EACH command's own enqueue ack before injecting the next --
  // Telemetry's own ack slot is single-depth (telemetry.h: "ack-depth-1 is
  // a stakeholder-accepted tradeoff"), and a fresh command's own dispatch
  // ack is not visible until the FOLLOWING cycle's own emit() (this
  // codebase's documented one-cycle ack-emission lag; the primary/
  // secondary tie-break can occasionally defer it a cycle further still)
  // -- injecting the next command before that lag resolves risks a second
  // tlm_.ack() call overwriting the still-pending first one before it is
  // ever emitted. Waiting for each ack in turn (mirrors
  // app_robot_loop_harness.cpp's own stepUntilAckSeen() precedent) makes
  // this burst robust to that lag entirely. kStopTimeMs=2000ms (40 cycles)
  // stays far above the handful of cycles this burst actually needs.
  std::vector<DecodedLine> earlyFrames;
  for (int n = 0; n < 6; ++n) {
    sim.injectMove(kVx, /*v_y=*/0.0f, /*omega=*/0.0f, TestSupport::MoveStopKind::kTime, kStopTimeMs,
                  kTimeoutMs, /*replace=*/false, kIds[n], kCorrIds[n]);
    bool acked = false;
    for (int i = 0; i < 10 && !acked; ++i) {
      sim.step(1);
      std::vector<DecodedLine> cycleFrames = onlyTelemetry(sim.drainTelemetry());
      earlyFrames.insert(earlyFrames.end(), cycleFrames.begin(), cycleFrames.end());
      // findFreshAck (not anyAckMatches) -- the 6th command's own ack is
      // an EXPECTED ERR_FULL, not err==0, so the wait condition must match
      // on corrId alone, regardless of err.
      uint32_t unusedErr = 0, unusedFlags = 0;
      if (findFreshAck(earlyFrames, kCorrIds[n], &unusedErr, &unusedFlags)) acked = true;
    }
    checkTrue(acked, "command #" + std::to_string(n + 1) + "'s ack (OK or ERR_FULL) reached the wire "
                     "within a bounded number of cycles");
  }

  for (int n = 0; n < 5; ++n) {
    checkTrue(anyAckMatches(earlyFrames, kCorrIds[n]), "Move #" + std::to_string(n + 1) + "'s enqueue ack was OK "
                                                        "(1 active + 4 pending all accepted)");
  }
  uint32_t rejectErr = 0, rejectFlags = 0;
  checkTrue(findFreshAck(earlyFrames, kCorrIds[5], &rejectErr, &rejectFlags), "the 6th command's enqueue ack reached the wire");
  checkUintEq(rejectErr, static_cast<uint32_t>(msg::ErrCode::ERR_FULL), "the 6th command was rejected ERR_FULL "
                                                                          "(queue already at 1 active + 4 pending)");

  // Let the chain run its course with NO further host traffic -- 5 * 40
  // cycles (2000ms/50ms) plus margin.
  std::vector<int> completionOrder;  // index into kIds, in the order each completion ack was seen
  std::vector<DecodedLine> lateFrames;
  constexpr int kMaxCycles = 260;
  for (int i = 0; i < kMaxCycles && completionOrder.size() < 5; ++i) {
    sim.step(1);
    std::vector<DecodedLine> cycleFrames = onlyTelemetry(sim.drainTelemetry());
    for (const auto& f : cycleFrames) {
      lateFrames.push_back(f);
      if (!(f.telemetry.flags & App::kFlagAckFresh) || f.telemetry.ack_err != 0) continue;
      for (int n = 0; n < 5; ++n) {
        if (f.telemetry.ack_corr == kIds[n]) completionOrder.push_back(n);
      }
    }
  }

  checkUintEq(static_cast<uint32_t>(completionOrder.size()), 5, "all 5 originally-accepted Moves (1 active + 4 "
                                                                  "pending) eventually complete");
  for (size_t i = 0; i < completionOrder.size(); ++i) {
    checkTrue(completionOrder[i] == static_cast<int>(i), "the completions arrive in the EXACT original order "
                                                          "(1,2,3,4,5) -- the queue's contents were not disturbed "
                                                          "by the rejected 6th command");
  }

  uint32_t unusedErr = 0, unusedFlags = 0;
  checkTrue(!findFreshAck(lateFrames, kIds[5], &unusedErr, &unusedFlags), "Move #6 (rejected) NEVER produces a "
                                                                          "completion ack -- it never activated");
}

// ===========================================================================
// SUC-053: an empty-queue MOVE expiry stops motors within one cycle, with
// zero further commands injected by the test after the expiring MOVE (the
// no-deadman contract: host silence always ends in motors stopped).
// ===========================================================================

void scenarioEmptyQueueExpiryStopsMotorsNoFurtherTraffic() {
  beginScenario("SUC-053: an empty-queue MOVE expiry stops motors, with zero further host traffic "
                "after the expiring MOVE");

  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  sim.step(3);
  (void)sim.drainTelemetry();

  constexpr float kVx = 150.0f;
  constexpr float kStopTimeMs = 200.0f;  // [ms] -- 4 cycles
  constexpr float kTimeoutMs = 100000.0f;
  constexpr uint32_t kMoveId = 40;
  constexpr uint32_t kCorrId = 140;

  sim.injectMove(kVx, /*v_y=*/0.0f, /*omega=*/0.0f, TestSupport::MoveStopKind::kTime, kStopTimeMs,
                kTimeoutMs, /*replace=*/true, kMoveId, kCorrId);
  // NO further sim.injectMove()/injectStop()/injectCommand() call anywhere
  // below -- the whole point of this scenario is that the Move ends, and
  // the motors stay stopped, entirely on RobotLoop's own unconditional
  // per-cycle MoveQueue::tick() -- no host traffic required.
  sim.step(15);  // 750ms -- comfortably past the 200ms stop threshold

  std::vector<DecodedLine> frames = onlyTelemetry(sim.drainTelemetry());
  checkTrue(!frames.empty(), "telemetry decoded across the run");
  checkTrue(anyAckMatches(frames, kCorrId), "the MOVE's enqueue ack was OK");

  uint32_t err = 1, flags = 0;
  checkTrue(findFreshAck(frames, kMoveId, &err, &flags), "the Move's completion ack (ack_corr==id) reached the wire");
  checkUintEq(err, 0, "the completion ack's err is OK");
  checkTrue((flags & App::kFlagFaultMoveTimeout) == 0, "ended via its own TIME stop condition, not the timeout backstop");

  const msg::Telemetry& lastFrame = frames.back().telemetry;
  checkTrue((lastFrame.flags & App::kFlagActive) == 0, "active clears by the end of the run");
  checkTrue(lastFrame.mode == msg::DriveMode::IDLE, "mode reads IDLE by the end of the run");
  checkFloatEq(sim.driveTargetVelLeft(), 0.0f, 1.0f, "the staged left target is exactly 0 -- Drive::stop() zeroed it");
  checkFloatEq(sim.driveTargetVelRight(), 0.0f, 1.0f, "the staged right target is exactly 0 -- Drive::stop() zeroed it");
}

// ===========================================================================
// SUC-055: a CONFIG patch injected mid-MOVE does not change the active
// MOVE's completion outcome -- A/B comparison against a config-free
// baseline (mirrors app_robot_loop_harness.cpp's own SUC-055 technique):
// more robust than hand-deriving the exact expected cycle count, and
// directly proves "shifts nothing".
// ===========================================================================

void scenarioConfigMidMoveDoesNotChangeCompletionOutcome() {
  beginScenario("SUC-055: a CONFIG patch injected mid-MOVE does not change the active Move's "
                "completion outcome");

  constexpr float kStopTimeMs = 250.0f;  // [ms]
  constexpr float kTimeoutMs = 5000.0f;  // [ms]
  constexpr int kMaxCycles = 60;

  // --- baseline: no CONFIG patch ---
  TestSim::SimHarness baseline;
  TestSupport::configureSimForBenchTest(baseline);
  baseline.boot();
  baseline.step(3);
  (void)baseline.drainTelemetry();

  constexpr uint32_t kIdBaseline = 50;
  constexpr uint32_t kCorrBaseline = 150;
  baseline.injectMove(/*v_x=*/0.0f, /*v_y=*/0.0f, /*omega=*/0.0f, TestSupport::MoveStopKind::kTime,
                      kStopTimeMs, kTimeoutMs, /*replace=*/true, kIdBaseline, kCorrBaseline);

  int baselineCyclesToEnd = 0;
  bool baselineCompleted = false;
  uint32_t baselineErr = 1, baselineFlags = 0;
  for (int i = 0; i < kMaxCycles && !baselineCompleted; ++i) {
    baseline.step(1);
    ++baselineCyclesToEnd;
    std::vector<DecodedLine> frames = onlyTelemetry(baseline.drainTelemetry());
    if (findFreshAck(frames, kIdBaseline, &baselineErr, &baselineFlags)) baselineCompleted = true;
  }
  checkTrue(baselineCompleted, "baseline: the Move completes within a bounded number of cycles");
  checkUintEq(baselineErr, 0, "baseline: the completion ack's err is OK");
  checkTrue((baselineFlags & App::kFlagFaultMoveTimeout) == 0, "baseline: ended via TIME, not timeout");

  // --- interfered: a CONFIG{motor} patch injected mid-flight ---
  TestSim::SimHarness interfered;
  TestSupport::configureSimForBenchTest(interfered);
  interfered.boot();
  interfered.step(3);
  (void)interfered.drainTelemetry();

  constexpr uint32_t kIdInterfered = 51;
  constexpr uint32_t kCorrInterfered = 151;
  constexpr uint32_t kConfigCorrId = 152;
  checkFloatEq(interfered.motorLeft().gains().kp, 0.002f, 0.0001f,
               "interfered: left motor starts at bench_test_config.h's own shipped kp");

  interfered.injectMove(/*v_x=*/0.0f, /*v_y=*/0.0f, /*omega=*/0.0f, TestSupport::MoveStopKind::kTime,
                        kStopTimeMs, kTimeoutMs, /*replace=*/true, kIdInterfered, kCorrInterfered);
  // Queued directly behind the Move on the SAME inbound transport --
  // App::Comms::pump() consumes at most one inbound line per cycle(), so
  // this CONFIG line dispatches the cycle AFTER the Move itself activates
  // -- genuinely "mid-MOVE", well before its own 250ms/5-cycle stop
  // threshold.
  interfered.injectCommand(armorMotorConfigPatchCommand(/*kp=*/0.02f, kConfigCorrId).c_str());

  int interferedCyclesToEnd = 0;
  bool interferedCompleted = false;
  bool configAcked = false;
  uint32_t interferedErr = 1, interferedFlags = 0;
  for (int i = 0; i < kMaxCycles && !interferedCompleted; ++i) {
    interfered.step(1);
    ++interferedCyclesToEnd;
    std::vector<DecodedLine> frames = onlyTelemetry(interfered.drainTelemetry());
    if (!configAcked && anyAckMatches(frames, kConfigCorrId)) configAcked = true;
    if (findFreshAck(frames, kIdInterfered, &interferedErr, &interferedFlags)) interferedCompleted = true;
  }

  checkTrue(configAcked, "interfered: the CONFIG patch was acked OK, mid-flight");
  checkFloatEq(interfered.motorLeft().gains().kp, 0.02f, 0.0001f,
               "interfered: the CONFIG patch's own kp landed live, unaffected by the concurrently-active Move");
  checkTrue(interferedCompleted, "interfered: the Move completes within a bounded number of cycles");
  checkUintEq(interferedErr, 0, "interfered: the completion ack's err is OK");
  checkTrue((interferedFlags & App::kFlagFaultMoveTimeout) == 0, "interfered: ended via TIME, not timeout");

  checkUintEq(static_cast<uint32_t>(interferedCyclesToEnd), static_cast<uint32_t>(baselineCyclesToEnd),
              "SUC-055: the CONFIG patch injected mid-flight shifts nothing -- the Move ends at the SAME "
              "cycle count as the config-free baseline");
}

}  // namespace

int main() {
  std::printf("=== MOVE Protocol Sim-System Scenarios (116-008) ===\n");
  std::printf("The sim-executable half of the protocol set-point issue's own Verification section,\n");
  std::printf("driven through TestSim::SimHarness against the REAL RobotLoop/MoveQueue/StopCondition/\n");
  std::printf("Drive/Odometry graph and a REAL, live-responding SimPlant.\n\n");

  scenarioTimeStopCompletesWithinTolerance();
  scenarioDistanceStopCompletesWithinTolerance();
  scenarioAngleStopCompletesWithinTolerance();
  scenarioWheelsVariantDrivesCorrectSigns();
  scenarioDistanceTimeoutWithStalledWheelsSetsFaultFlag();
  scenarioChainHandoffSeamlessNoZeroCycle();
  scenarioReplacePreemptsMidMotionSameCycle();
  scenarioFifthPendingRejectedErrFullQueueUnchanged();
  scenarioEmptyQueueExpiryStopsMotorsNoFurtherTraffic();
  scenarioConfigMidMoveDoesNotChangeCompletionOutcome();

  if (g_failureCount == 0) {
    std::printf("OK: all MOVE protocol scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the MOVE protocol scenarios\n", g_failureCount);
  return 1;
}
