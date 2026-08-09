// goto_protocol_harness.cpp -- 135-005's own acceptance harness: the
// sim-executable half of the goto issue's own Verification section ("Sim:
// end-to-end GO_TO over the codec, streamed-target run") --
// replaceable-go-to-moves-in-the-motion-library.md. Modeled directly on
// move_protocol_harness.cpp (116-008) -- same TestSim::SimHarness/SimPlant
// composition, same "compile a throwaway C++ binary via subprocess, run
// it, assert exit 0, print a per-scenario trace" convention -- but for the
// NEW binary command-plane arm ticket 004 landed (GO_TO/Motion::Navigator)
// instead of MOVE/Motion::Planner.
//
// Exercises the REAL Core::RobotLoop (now including Motion::Navigator)
// against the REAL TestSim::SimPlant, through the actual GO_TO wire codec
// (TestSim::SimHarness::injectGoto() -> TestSupport::armorGotoCommand()) --
// no ARM hardware, no internal accessor peeked out of process for any
// LOAD-BEARING assertion. Every scenario below asserts ONLY on DECODED
// telemetry: the bounded ack ring (acks_/acks_count, packed
// corr_id<<4|err), the flags word (Core::kFlagActive/kFlagOtosConnected/
// kFlagFaultMoveTimeout), and the decoded otos/pose fields -- ticket 005's
// own acceptance criterion, matching move_protocol_harness.cpp's own
// discipline for ITS primary assertions (a handful of scenarios there also
// read sim.driveTargetVelLeft()/sim.planner().active() as ADDITIONAL
// corroborating diagnostics; this file's scenarios need no such
// corroboration -- decoded telemetry alone carries every assertion here).
//
// Three scenarios:
//   1. WORLD-frame GO_TO end to end: decoded OTOS position converges
//      within the goto's arrival tolerance, the robot settles at rest
//      (kFlagActive clears), exactly one completion ack for the sent id
//      lands in the ack ring.
//   2. ROBOT-frame GO_TO, resolved once at acceptance (SUC-001): a
//      PRECEDING plain MOVE (bypassing the Navigator entirely) turns the
//      robot's heading well away from zero and lands it at rest; the
//      ROBOT-frame target is then computed independently here, mirroring
//      Core::RobotLoop::handleGoto()'s own world-resolution formula against
//      the acceptance-time OTOS reading (captured from decoded telemetry,
//      not sim ground truth); the robot's final decoded OTOS position is
//      asserted against THAT fixed point -- a continuously re-resolved
//      frame would chase the robot's own heading instead and would not
//      generally settle there.
//   3. Streamed-target EXTERNAL-mode scenario (SUC-002): a short sequence
//      of GO_TO commands, each replacing the previous target before it is
//      reached, mimicking a host pure-pursuit loop streaming
//      pursuitTarget() output -- asserts the robot never comes to rest
//      before the FINAL target, that none of the superseded targets ever
//      produces a completion ack, and that a deliberately delayed/dropped
//      target update mid-sequence (several cycles of wire silence) neither
//      faults nor halts the Navigator -- it keeps converging on the
//      last-accepted target throughout, entirely on its own per-tick
//      re-solve (navigator.cpp's own tick(): "INTERNAL mode is simply the
//      degenerate case of the Navigator re-solving against its own
//      last-accepted target every cycle").
//
// Run by test_goto_protocol.py, which compiles this file together with
// sim_plant.cpp, wire_test_codec.cpp, the plant sources, and the same full
// HOST_BUILD Devices/App/messages/kinematics/motion dependency graph every
// sibling test_*.py in this directory already compiles (now including
// src/firm/motion/navigator/{arc_solver,navigator}.cpp, landed by ticket 004).
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "core/telemetry.h"
#include "bench_test_config.h"
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

void checkFloatLe(float actual, float bound, const std::string& what) {
  if (!(actual <= bound)) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected <= %g, got %g", what.c_str(),
                  static_cast<double>(bound), static_cast<double>(actual));
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

constexpr uint32_t kAckErrBits = 4;
constexpr uint32_t kAckErrMask = (1u << kAckErrBits) - 1;

// anyAckMatches -- "was `key` ever acked OK (err==0) anywhere in `frames`"
// -- mirrors move_protocol_harness.cpp's own helper of the same name.
bool anyAckMatches(const std::vector<DecodedLine>& frames, uint32_t key) {
  for (const auto& f : frames) {
    for (uint8_t i = 0; i < f.telemetry.acks_count; ++i) {
      const uint32_t packed = f.telemetry.acks_[i];
      if ((packed >> kAckErrBits) == key && (packed & kAckErrMask) == 0) return true;
    }
  }
  return false;
}

// findFreshAck -- the first ack ring entry in `frames` whose corr_id ==
// `key`, regardless of err -- mirrors move_protocol_harness.cpp's own
// helper of the same name. Every scenario below gives each GO_TO/MOVE a
// corrId/id pair that is unique across the WHOLE scenario, so a match
// against a goto's own `id` is unambiguously that goto's completion ack.
bool findFreshAck(const std::vector<DecodedLine>& frames, uint32_t key, uint32_t* errOut) {
  for (const auto& f : frames) {
    for (uint8_t i = 0; i < f.telemetry.acks_count; ++i) {
      const uint32_t packed = f.telemetry.acks_[i];
      if ((packed >> kAckErrBits) != key) continue;
      if (errOut) *errOut = packed & kAckErrMask;
      return true;
    }
  }
  return false;
}

}  // namespace

// ===========================================================================
// Scenario 1: WORLD-frame GO_TO end to end.
// ===========================================================================

namespace {

void scenarioWorldFrameGotoArrivesAndSettles() {
  beginScenario("WORLD-frame GO_TO: decoded OTOS position converges within arrival tolerance, "
                "robot settles at rest, exactly one completion ack for the sent id");

  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  sim.step(3);  // settle: publishOtos() lags boot() by one cycle
  (void)sim.drainTelemetry();

  constexpr float kTargetX = 700.0f;              // [mm]
  constexpr float kTargetY = 300.0f;              // [mm]
  constexpr float kArrivalToleranceMm = 100.0f;    // NavigatorLimits::defaultArrivalTolerance baked
                                                    // default (config/boot_config.cpp)
  constexpr float kToleranceMargin = 60.0f;        // settle-residual margin, same spirit as every
                                                    // other "Empirically ~X over the commanded Y"
                                                    // bound elsewhere in this codebase
  constexpr uint32_t kGotoId = 801;
  constexpr uint32_t kCorrId = 9101;

  sim.injectGoto(kTargetX, kTargetY, /*frame=*/0, /*speed=*/0.0f, /*arrive=*/0.0f,
                 /*timeout=*/20000.0f, kGotoId, kCorrId);

  std::vector<DecodedLine> frames;
  bool sawEnqueueAck = false;
  bool sawCompletionAck = false;
  uint32_t completionErr = 1;
  int drainedSinceCompletion = 0;
  for (int i = 0; i < 900 && drainedSinceCompletion < 5; ++i) {
    sim.step(1);
    std::vector<DecodedLine> cycleFrames = onlyTelemetry(sim.drainTelemetry());
    for (const auto& f : cycleFrames) frames.push_back(f);
    if (!sawEnqueueAck && anyAckMatches(cycleFrames, kCorrId)) sawEnqueueAck = true;
    if (!sawCompletionAck) {
      uint32_t err = 1;
      if (findFreshAck(cycleFrames, kGotoId, &err)) {
        sawCompletionAck = true;
        completionErr = err;
      }
    }
    if (sawCompletionAck) ++drainedSinceCompletion;
  }

  checkTrue(!frames.empty(), "telemetry decoded across the run");
  checkTrue(sawEnqueueAck, "the GO_TO's own enqueue ack (corr_id) reached the wire");
  // The ack ring is a bounded ring that stays visible across several
  // consecutive telemetry frames, not a one-shot event (every sibling
  // harness's own established discipline -- see
  // test_app_robot_loop_goto_harness.cpp's own comment on this exact
  // point) -- a boolean "did this id's completion ack ever appear" is the
  // right assertion; the actual "exactly one completion EVENT" guarantee
  // is Motion::Navigator's own state-machine property, proven directly by
  // navigator_test.cpp's own ctest, not by counting ring-frame
  // occurrences here.
  checkTrue(sawCompletionAck, "the goto's own completion ack (id == kGotoId) reached the ack ring");
  checkUintEq(completionErr, 0, "the completion ack's err is OK (arrival, not an aborted/faulted goto)");

  checkTrue((frames.back().telemetry.flags & Core::kFlagActive) == 0,
            "the robot is at rest (kFlagActive clear) by the end of the run");
  checkTrue((frames.back().telemetry.flags & Core::kFlagFaultMoveTimeout) == 0,
            "kFlagFaultMoveTimeout stayed clear -- the goto ended by arrival, not a safety-backstop "
            "abort");
  checkTrue((frames.back().telemetry.flags & Core::kFlagOtosConnected) != 0,
            "OTOS stayed connected -- the decoded otos reading below is valid");

  const float finalOtosX = msg::OtosReading::unpackX(frames.back().telemetry.otos.x);
  const float finalOtosY = msg::OtosReading::unpackY(frames.back().telemetry.otos.y);
  const float distToTarget = std::hypot(finalOtosX - kTargetX, finalOtosY - kTargetY);
  std::printf("  WORLD_GOTO: final_otos=(%.1f,%.1f) target=(%.1f,%.1f) dist=%.1f\n",
              static_cast<double>(finalOtosX), static_cast<double>(finalOtosY),
              static_cast<double>(kTargetX), static_cast<double>(kTargetY),
              static_cast<double>(distToTarget));
  checkFloatLe(distToTarget, kArrivalToleranceMm + kToleranceMargin,
               "the decoded OTOS position settled within the goto's arrival tolerance of the "
               "commanded WORLD target");
}

// ===========================================================================
// Scenario 2: ROBOT-frame GO_TO, resolved once at acceptance (SUC-001).
// ===========================================================================

void scenarioRobotFrameGotoResolvedOnceAtAcceptance() {
  beginScenario("ROBOT-frame GO_TO: target resolves against the OTOS pose AT ACCEPTANCE, not a "
                "moving/re-resolved frame (SUC-001's own postcondition)");

  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  sim.step(3);
  (void)sim.drainTelemetry();

  // Step 1: a PRECEDING plain MOVE -- Core::RobotLoop::handleMove(), never
  // the Navigator -- turns the robot's heading well away from zero and
  // lands it fully at rest BEFORE the ROBOT-frame goto is ever sent. Only
  // with a genuinely nonzero heading change first do "resolved once at
  // acceptance" and "continuously re-resolved against the current
  // heading" predict materially DIFFERENT final world points -- this is
  // what makes the scenario able to actually distinguish the two.
  constexpr float kTurnOmega = 1.0f;         // [rad/s]
  constexpr float kTurnAngleRad = 1.4f;      // [rad] ~80 deg
  constexpr float kTurnTimeoutMs = 6000.0f;  // [ms] -- bounds the terminal fine-align trim too
  sim.injectMove(/*v_x=*/0.0f, /*v_y=*/0.0f, kTurnOmega, TestSupport::MoveStopKind::kAngle,
                kTurnAngleRad, kTurnTimeoutMs, /*replace=*/true, /*id=*/701, /*corrId=*/9201);

  // Run well past the turn's own landing AND the terminal fine-align trim
  // (MoveLifecycle::Aligning, bounded by the Move's own timeout above) --
  // same budget move_protocol_harness.cpp's own ANGLE scenario uses for a
  // smaller commanded angle.
  std::vector<DecodedLine> turnFrames;
  for (int i = 0; i < 300; ++i) {  // 15s
    sim.step(1);
    for (const auto& f : onlyTelemetry(sim.drainTelemetry())) turnFrames.push_back(f);
  }
  checkTrue(!turnFrames.empty(), "telemetry decoded across the preceding turn");
  checkTrue((turnFrames.back().telemetry.flags & Core::kFlagActive) == 0,
            "the preceding turn landed and the robot is at rest before the ROBOT-frame goto is "
            "sent");
  checkTrue((turnFrames.back().telemetry.flags & Core::kFlagOtosConnected) != 0,
            "OTOS stayed connected through the turn -- the acceptance-time reading below is valid");

  // The EXACT OTOS reading Core::RobotLoop::handleGoto() will read at
  // acceptance -- captured from DECODED telemetry, not sim ground truth,
  // per this ticket's own "decoded telemetry only" discipline. Heading is
  // genuinely fixed at this point (the robot is at rest, not turning), so
  // there is no ambiguity about which cycle's reading this is -- it will
  // not have changed by the time the GO_TO below is actually accepted.
  const float acceptX = msg::OtosReading::unpackX(turnFrames.back().telemetry.otos.x);
  const float acceptY = msg::OtosReading::unpackY(turnFrames.back().telemetry.otos.y);
  const float acceptHeadingRaw = msg::OtosReading::unpackHeading(turnFrames.back().telemetry.otos.heading);

  // Step 2: a ROBOT-frame goto, (dx, dy) relative to the CURRENT facing.
  // Independently mirrors Core::RobotLoop::handleGoto()'s own ROBOT-frame
  // formula (robot_loop.cpp) byte for byte: world = otos.{x,y} +
  // dx*cos(h) -+ dy*sin(h), using the SAME raw (un-negated) otos.heading
  // handleGoto() itself reads (robot_loop.cpp's own comment: "a pure
  // geometric transform ... UNRELATED to NavigatorLimits::yawSign").
  constexpr float kDx = 300.0f;  // [mm] "ahead" in the robot's OWN frame
  constexpr float kDy = 0.0f;    // [mm]
  const float expectedWorldX =
      acceptX + kDx * std::cos(acceptHeadingRaw) - kDy * std::sin(acceptHeadingRaw);
  const float expectedWorldY =
      acceptY + kDx * std::sin(acceptHeadingRaw) + kDy * std::cos(acceptHeadingRaw);
  std::printf("  ROBOT_GOTO: accept_pose=(%.1f,%.1f,%.3frad) expected_world=(%.1f,%.1f)\n",
              static_cast<double>(acceptX), static_cast<double>(acceptY),
              static_cast<double>(acceptHeadingRaw), static_cast<double>(expectedWorldX),
              static_cast<double>(expectedWorldY));

  constexpr float kArrivalToleranceMm = 100.0f;
  constexpr float kToleranceMargin = 60.0f;
  constexpr uint32_t kGotoId = 802;
  constexpr uint32_t kCorrId = 9202;
  sim.injectGoto(kDx, kDy, /*frame=*/1, /*speed=*/0.0f, /*arrive=*/0.0f, /*timeout=*/20000.0f,
                kGotoId, kCorrId);

  std::vector<DecodedLine> frames;
  bool sawEnqueueAck = false;
  bool sawCompletionAck = false;
  uint32_t completionErr = 1;
  int drainedSinceCompletion = 0;
  for (int i = 0; i < 900 && drainedSinceCompletion < 5; ++i) {
    sim.step(1);
    std::vector<DecodedLine> cycleFrames = onlyTelemetry(sim.drainTelemetry());
    for (const auto& f : cycleFrames) frames.push_back(f);
    if (!sawEnqueueAck && anyAckMatches(cycleFrames, kCorrId)) sawEnqueueAck = true;
    if (!sawCompletionAck) {
      uint32_t err = 1;
      if (findFreshAck(cycleFrames, kGotoId, &err)) {
        sawCompletionAck = true;
        completionErr = err;
      }
    }
    if (sawCompletionAck) ++drainedSinceCompletion;
  }

  checkTrue(!frames.empty(), "telemetry decoded across the ROBOT-frame goto's own run");
  checkTrue(sawEnqueueAck, "the ROBOT-frame goto's own enqueue ack reached the wire");
  checkTrue(sawCompletionAck, "the ROBOT-frame goto's own completion ack reached the wire");
  checkUintEq(completionErr, 0, "the completion ack's err is OK");
  checkTrue((frames.back().telemetry.flags & Core::kFlagActive) == 0, "the robot settled at rest");
  checkTrue((frames.back().telemetry.flags & Core::kFlagOtosConnected) != 0,
            "OTOS stayed connected -- the decoded otos reading below is valid");

  const float finalOtosX = msg::OtosReading::unpackX(frames.back().telemetry.otos.x);
  const float finalOtosY = msg::OtosReading::unpackY(frames.back().telemetry.otos.y);
  const float distToExpected = std::hypot(finalOtosX - expectedWorldX, finalOtosY - expectedWorldY);
  std::printf("  ROBOT_GOTO: final_otos=(%.1f,%.1f) dist_to_expected=%.1f\n",
              static_cast<double>(finalOtosX), static_cast<double>(finalOtosY),
              static_cast<double>(distToExpected));
  checkFloatLe(distToExpected, kArrivalToleranceMm + kToleranceMargin,
               "the robot settled near the world point the ACCEPTANCE-time heading resolves to -- "
               "SUC-001's 'resolved once, not re-resolved against a moving frame' postcondition. A "
               "continuously re-resolved frame would chase the robot's own heading as it turns "
               "toward the target and would not generally settle here.");
}

// ===========================================================================
// Scenario 3: Streamed-target EXTERNAL-mode scenario (SUC-002).
// ===========================================================================

struct Waypoint {
  float x;         // [mm]
  float y;         // [mm]
  uint32_t id;
  uint32_t corrId;
};

void scenarioStreamedTargetsNeverRestBeforeFinal() {
  beginScenario("Streamed GO_TO targets (EXTERNAL mode): the robot never comes to rest before the "
                "FINAL target; a mid-sequence gap in target updates neither faults nor halts it");

  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  sim.step(3);
  (void)sim.drainTelemetry();

  // A gently-curving path, ~250-300mm per leg, every leg's bearing change
  // small enough to stay well under NavigatorLimits::turnFirstAngle
  // (~50 deg baked default, config/boot_config.cpp) from wherever the
  // robot actually is when each update lands -- this scenario is about
  // REPLACEMENT continuity, not the pivot-first policy (that is SUC-004's
  // own ctest surface, navigator_test.cpp).
  constexpr Waypoint kWaypoints[4] = {
      {250.0f, 0.0f, 901, 9301},
      {500.0f, 30.0f, 902, 9302},
      {750.0f, 60.0f, 903, 9303},
      {1050.0f, 90.0f, 904, 9304},  // the FINAL target -- the only one that should ever complete
  };
  constexpr float kGotoTimeoutMs = 10000.0f;  // [ms] -- generous; restarts fresh at EACH start()
                                               // (Navigator::start()'s own justStarted_/
                                               // startCycleStart_ reset), so a streamed update never
                                               // inherits a shrinking budget from an earlier target.

  // "At rest" here is measured off decoded ENCODER VELOCITY, not
  // Core::kFlagActive. Measured directly against this exact scenario:
  // kFlagActive tracks Planner Move-SLOT occupancy, not motion -- every
  // internal-segment replace this file's own streamed updates provoke
  // (and every ordinary material-change/half-arc-refresh reissue the
  // Navigator does entirely on its own) clears it for exactly one
  // telemetry-visible cycle even while both wheels are still climbing
  // toward cruise speed (observed: kFlagActive cleared with encL/encR at
  // 173.4mm/s, still ACCELERATING) -- a real, harmless bookkeeping detail
  // of "the old segment ended and the new one has not yet had its own
  // first planner_.tick() call" (per Planner's "activates on Planner's own
  // NEXT tick()" contract), NOT the robot coming to rest. Encoder velocity
  // is the direct, physical, decoded-telemetry signal for "is the robot
  // actually moving" -- kMinMovingVelocity sits well above the ~58mm/s
  // floor observed during ordinary mid-route replace churn and well below
  // cruise (150mm/s default).
  constexpr float kMinMovingVelocity = 30.0f;  // [mm/s]

  std::vector<DecodedLine> frames;
  bool hasStartedMoving = false;         // true once either wheel has ever exceeded the threshold
  bool everRestedDuringStream = false;   // true if a genuine at-rest sample is seen AFTER moving
                                          // began and BEFORE the final completion ack
  bool sawFaultFlag = false;
  bool sawIntermediateCompletion = false;
  bool sawFinalCompletion = false;
  uint32_t finalCompletionErr = 1;

  auto drainCycles = [&](int cycles) {
    for (int i = 0; i < cycles; ++i) {
      sim.step(1);
      for (const auto& f : onlyTelemetry(sim.drainTelemetry())) {
        frames.push_back(f);

        const float velL =
            std::fabs(msg::EncoderReading::unpackVelocity(f.telemetry.enc_left.velocity));
        const float velR =
            std::fabs(msg::EncoderReading::unpackVelocity(f.telemetry.enc_right.velocity));
        const bool atRest = velL < kMinMovingVelocity && velR < kMinMovingVelocity;
        if (!atRest) hasStartedMoving = true;
        if (!sawFinalCompletion && hasStartedMoving && atRest) everRestedDuringStream = true;

        if ((f.telemetry.flags & Core::kFlagFaultMoveTimeout) != 0) sawFaultFlag = true;
        for (uint8_t a = 0; a < f.telemetry.acks_count; ++a) {
          const uint32_t packed = f.telemetry.acks_[a];
          const uint32_t key = packed >> kAckErrBits;
          const uint32_t err = packed & kAckErrMask;
          if (key == kWaypoints[0].id || key == kWaypoints[1].id || key == kWaypoints[2].id) {
            sawIntermediateCompletion = true;
          }
          if (key == kWaypoints[3].id && err == 0) {
            sawFinalCompletion = true;
            finalCompletionErr = err;
          }
        }
      }
    }
  };

  // Stream waypoints 0, 1, and 2, each replacing the previous target
  // comfortably before it could be reached (~250-300mm apart at the
  // 150mm/s cruise default -- ~1.7-2s per leg -- streamed every 10
  // cycles/500ms) -- mimics a host-side pure-pursuit loop pushing
  // pursuitTarget() output down the wire faster than any single leg could
  // complete.
  for (int w = 0; w <= 2; ++w) {
    sim.injectGoto(kWaypoints[w].x, kWaypoints[w].y, /*frame=*/0, /*speed=*/0.0f, /*arrive=*/0.0f,
                   kGotoTimeoutMs, kWaypoints[w].id, kWaypoints[w].corrId);
    drainCycles(10);  // 500ms -- comfortably shorter than any leg's own travel time
  }

  checkTrue(hasStartedMoving, "the robot actually started moving toward the streamed targets");
  checkTrue(!everRestedDuringStream, "the robot never came to rest while waypoints 0/1/2 were "
                                     "streamed in succession -- no intermediate stop-and-restart "
                                     "(SUC-002)");
  checkTrue(!sawFaultFlag, "kFlagFaultMoveTimeout never set while streaming");
  checkTrue(!sawIntermediateCompletion, "none of the streamed-so-far waypoints ever produced a "
                                        "completion ack -- each was replaced before it was reached");

  // Deliberately delayed/dropped target update: nothing sent for several
  // cycles mid-sequence -- the Navigator's own per-tick re-solve against
  // the LAST-accepted target (waypoint 2) is what has to carry this, with
  // NO further host input at all (navigator.cpp's tick(): re-solves and,
  // on material change or the mandatory half-arc refresh, re-issues
  // entirely on its own).
  drainCycles(16);  // 800ms of wire silence

  checkTrue(!everRestedDuringStream, "the mid-sequence gap in target updates did not stop the "
                                     "robot -- it kept converging on the last-accepted target with "
                                     "no further host input");
  checkTrue(!sawFaultFlag, "kFlagFaultMoveTimeout never set during the gap -- silence is not a "
                           "fault");
  checkTrue(!sawIntermediateCompletion, "still no completion ack for any superseded waypoint after "
                                        "the gap");

  // Resume: send the FINAL target and let the goto run all the way to
  // completion.
  sim.injectGoto(kWaypoints[3].x, kWaypoints[3].y, /*frame=*/0, /*speed=*/0.0f, /*arrive=*/0.0f,
                 kGotoTimeoutMs, kWaypoints[3].id, kWaypoints[3].corrId);

  int drainedSinceCompletion = 0;
  for (int i = 0; i < 900 && drainedSinceCompletion < 5; ++i) {
    drainCycles(1);
    if (sawFinalCompletion) ++drainedSinceCompletion;
  }

  std::printf("  STREAM_GOTO: everRestedDuringStream=%d sawFaultFlag=%d "
              "sawIntermediateCompletion=%d sawFinalCompletion=%d finalCompletionErr=%u\n",
              everRestedDuringStream, sawFaultFlag, sawIntermediateCompletion, sawFinalCompletion,
              finalCompletionErr);

  checkTrue(!everRestedDuringStream, "the robot still never came to rest before the FINAL target's "
                                     "own completion (rest is only ever counted AFTER motion "
                                     "started and BEFORE sawFinalCompletion flips true, per "
                                     "drainCycles()'s own ordering)");
  checkTrue(!sawFaultFlag, "kFlagFaultMoveTimeout never set across the WHOLE streamed run");
  checkTrue(!sawIntermediateCompletion, "still no completion ack for any superseded waypoint, "
                                        "across the whole run");
  checkTrue(sawFinalCompletion, "the FINAL target's own completion ack (id == kWaypoints[3].id, "
                                "err == 0) reached the ack ring");
  checkUintEq(finalCompletionErr, 0, "the final completion ack's err is OK");
  checkTrue(!frames.empty(), "telemetry decoded across the whole streamed run");
  checkTrue((frames.back().telemetry.flags & Core::kFlagActive) == 0,
            "the robot is at rest (kFlagActive clear too) once it reaches the FINAL target -- and, "
            "per every check above, not before");
}

}  // namespace

int main() {
  scenarioWorldFrameGotoArrivesAndSettles();
  scenarioRobotFrameGotoResolvedOnceAtAcceptance();
  scenarioStreamedTargetsNeverRestBeforeFinal();

  if (g_failureCount > 0) {
    std::printf("FAILED: %d assertion(s) across the GO_TO protocol sim-system scenarios\n",
                g_failureCount);
    return 1;
  }
  std::printf("PASS: every GO_TO protocol sim-system scenario (135-005)\n");
  return 0;
}
