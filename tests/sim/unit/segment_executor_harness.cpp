// segment_executor_harness.cpp -- off-hardware acceptance harness for ticket
// 094-001 (SUC-001): exercises Motion::SegmentExecutor
// (source/motion/segment_executor.{h,cpp}) in isolation against hand-built
// Motion::Segment/msg::MotorState fixtures -- no real hardware, no wire
// verb, no Subsystems::Drivetrain/blackboard reference (mirrors
// planner_harness.cpp's own shape exactly: #includes only motion/
// segment_executor.h + messages/*.h -- all dependency-free -- links against
// motion/segment_executor.cpp, motion/jerk_trajectory.cpp, and motion/
// stop_condition.cpp, plus the vendored Ruckig sources, compiles with the
// plain system C++ compiler -- no CMake, no ARM toolchain).
//
// Test-side plant model: a simple, self-consistent (zero-lag, zero-slip)
// discrete integrator -- NOT a physical simulation, just enough to drive
// realistic (encL, encR) pairs from the executor's own commanded twist each
// tick, closing the loop the same way planner_harness.cpp's "closely
// tracking" scenarios do (see e.g. scenarioDistanceGoalRuckigTraceNever
// Reverses there): vL = v - omega*trackwidth/2, vR = v + omega*trackwidth/2,
// encL/encR += vL/vR * dt. This is sufficient to drive every phase's own
// encoder-only stop condition (STOP_DISTANCE from (encL+encR)/2,
// STOP_ROTATION from (encR-encL)/2) to fire naturally, and to prove the
// no-reverse-creep property against a plant that is actually being commanded
// tick to tick (not a hand-derived closed form).
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "messages/common.h"
#include "messages/motor.h"
#include "messages/planner.h"
#include "motion/segment.h"
#include "motion/segment_executor.h"

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

void checkFalse(bool condition, const std::string& what) {
  if (condition) fail(what + " -- expected false, got true");
}

void checkFloatNear(float actual, float expected, float tol, const std::string& what) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g (+/- %g), got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(tol),
                  static_cast<double>(actual));
    fail(buf);
  }
}

msg::PlannerConfig generousConfig() {
  msg::PlannerConfig cfg;
  cfg.a_max = 1000.0f;         // [mm/s^2]
  cfg.a_decel = 1000.0f;       // [mm/s^2]
  cfg.v_body_max = 300.0f;     // [mm/s]
  cfg.yaw_rate_max = 3.0f;     // [rad/s]
  cfg.yaw_acc_max = 15.0f;     // [rad/s^2]
  cfg.j_max = 0.0f;            // trapezoid (no S-curve) -- exercised by test_jerk_trajectory.py
  cfg.yaw_jerk_max = 0.0f;
  return cfg;
}

constexpr float kTrackwidth = 150.0f;  // [mm]

// PlantState -- the self-consistent zero-lag/zero-slip encoder integrator
// described in the file header comment.
struct PlantState {
  float encL = 0.0f;  // [mm]
  float encR = 0.0f;  // [mm]

  void advance(const msg::BodyTwist3& twist, float dt) {
    float vL = twist.v_x - twist.omega * kTrackwidth * 0.5f;
    float vR = twist.v_x + twist.omega * kTrackwidth * 0.5f;
    encL += vL * dt;
    encR += vR * dt;
  }

  msg::MotorState leftObs() const {
    msg::MotorState s;
    s.position.has = true;
    s.position.val = encL;
    s.velocity.has = true;
    return s;
  }
  msg::MotorState rightObs() const {
    msg::MotorState s;
    s.position.has = true;
    s.position.val = encR;
    s.velocity.has = true;
    return s;
  }
};

// fixedEnc -- a MotorState fixture holding position artificially fixed at
// `positionVal` with velocity.has == true but velocity.val left at its
// default 0.0f (mirrors PlantState::leftObs()/rightObs()'s own existing
// convention of never populating velocity.val -- these two sprint-098
// scenarios that use it deliberately want omega_measured == 0 always, so
// only the position half of the tolerance+dwell gate / stall geometry is
// exercised).
msg::MotorState fixedEnc(float positionVal) {
  msg::MotorState s;
  s.position.has = true;
  s.position.val = positionVal;
  s.velocity.has = true;
  return s;
}

// VelocityAwarePlant -- like PlantState (same zero-lag/zero-slip vL/vR
// integration), but ALSO reports the last commanded per-wheel velocity via
// MotorState.velocity.val (PlantState never populates it -- see fixedEnc()'s
// comment) and applies `trackingFactor` to the ROTATIONAL component only
// (translation, unused by the PRE_PIVOT-only scenarios below, is
// unaffected) -- a persistent, deliberately-introduced tracking gap
// (trackingFactor < 1.0 systematically under-rotates) sprint 098's M3/M5
// scenarios need a real, consistently-signed lag to correct against /
// prove uncorrected.
struct VelocityAwarePlant {
  float encL = 0.0f;  // [mm]
  float encR = 0.0f;  // [mm]
  float velL = 0.0f;  // [mm/s] last commanded left wheel velocity
  float velR = 0.0f;  // [mm/s] last commanded right wheel velocity
  float trackingFactor = 1.0f;  // dimensionless; < 1 = systematically under-rotates

  void advance(const msg::BodyTwist3& twist, float dt) {
    float omega = twist.omega * trackingFactor;
    velL = twist.v_x - omega * kTrackwidth * 0.5f;
    velR = twist.v_x + omega * kTrackwidth * 0.5f;
    encL += velL * dt;
    encR += velR * dt;
  }

  msg::MotorState leftObs() const {
    msg::MotorState s;
    s.position.has = true;
    s.position.val = encL;
    s.velocity.has = true;
    s.velocity.val = velL;
    return s;
  }
  msg::MotorState rightObs() const {
    msg::MotorState s;
    s.position.has = true;
    s.position.val = encR;
    s.velocity.has = true;
    s.velocity.val = velR;
    return s;
  }
};

// runToConvergence -- ticks `exec` against `plant` at a fixed 20ms cadence
// until Motion::SegmentExecutor::converged() (or `maxTicks` is exhausted),
// tracking the never-reverses invariant against `expectV`/`expectOmega`
// (the segment's own commanded sign -- 0 means "this component is expected
// to stay at 0 throughout, small numerical noise aside"). Returns the tick
// count actually run and writes the LAST commanded twist to `*lastTwist`.
int runToConvergence(Motion::SegmentExecutor& exec, PlantState& plant, uint32_t* clockMs,
                     int maxTicks, float expectV, float expectOmega, bool* everReversedV,
                     bool* everReversedOmega, msg::BodyTwist3* lastTwist) {
  const uint32_t kDt = 20;  // [ms]
  int i = 0;
  for (; i < maxTicks; ++i) {
    *clockMs += kDt;
    msg::BodyTwist3 twist = exec.tick(*clockMs, plant.leftObs(), plant.rightObs());
    if (expectV > 0.0f && twist.v_x < -0.5f) *everReversedV = true;
    if (expectV < 0.0f && twist.v_x > 0.5f) *everReversedV = true;
    if (expectOmega > 0.0f && twist.omega < -0.02f) *everReversedOmega = true;
    if (expectOmega < 0.0f && twist.omega > 0.02f) *everReversedOmega = true;
    plant.advance(twist, static_cast<float>(kDt) * 0.001f);
    *lastTwist = twist;
    if (exec.converged()) return i + 1;
  }
  return i;
}

// 1. A plain straight segment (direction=0, finalHeading=0): only TRANSLATE
// runs -- both pivots are skipped, no reverse, converges via its own
// STOP_DISTANCE (not the STOP_TIME safety net -- proven by finishing well
// under the safety net's ~3.5s-ish window for this segment).
void scenarioStraightSegmentSkipsBothPivots() {
  beginScenario("straight segment (direction=0, finalHeading=0): skips both pivots, no reverse");
  Motion::SegmentExecutor exec;
  exec.configure(generousConfig());

  Motion::Segment seg;
  seg.distance = 500.0f;  // [mm]
  seg.direction = 0.0f;
  seg.finalHeading = 0.0f;

  uint32_t clock = 0;
  exec.start(seg, clock, kTrackwidth);
  checkTrue(exec.active(), "start() with a nonzero distance leaves the executor active");

  PlantState plant;
  bool everReversedV = false;
  bool everReversedOmega = false;
  msg::BodyTwist3 last;
  int ticks = runToConvergence(exec, plant, &clock, 400, 500.0f, 0.0f, &everReversedV,
                               &everReversedOmega, &last);

  checkTrue(exec.converged(), "straight segment converges within 400 ticks (8s)");
  checkFalse(everReversedV, "commanded v_x never goes negative");
  checkFalse(everReversedOmega, "commanded omega never leaves ~0 (no pivot phases ran)");
  checkFloatNear(last.v_x, 0.0f, 1e-3f, "final commanded v_x settles to 0");
  checkFloatNear(last.omega, 0.0f, 1e-6f, "final commanded omega stays exactly 0");
  // Tolerance widened 5 -> 15 (dead-time-projected terminal firing,
  // 2026-07-09): the stop now fires when remaining <= plannedSpeed*kDeadTime
  // so a REAL plant's in-flight actuation lag carries it home -- this
  // harness's ZERO-lag plant therefore lands up to ~v_tail*kDeadTime short
  // by design. The terminal contract is single-profile/no-reverse/prompt
  // completion; absolute travel accuracy is calibration work.
  checkFloatNear(plant.encL, 500.0f, 15.0f, "left encoder travels ~500mm");
  checkFloatNear(plant.encR, 500.0f, 15.0f, "right encoder travels ~500mm");
  checkTrue(ticks < 400, "converges before the safety net exhausts all 400 ticks");
}

// 2. Translate-then-terminal-pivot (direction=0, finalHeading!=0): PRE_PIVOT
// is skipped (direction~=0) but TRANSLATE then TERMINAL_PIVOT both run, in
// that order, no reverse on either channel.
void scenarioTranslateThenTerminalPivot() {
  beginScenario("translate-then-terminal-pivot: PRE_PIVOT skipped, TRANSLATE then TERMINAL_PIVOT");
  Motion::SegmentExecutor exec;
  exec.configure(generousConfig());

  Motion::Segment seg;
  seg.distance = 300.0f;      // [mm]
  seg.direction = 0.0f;       // PRE_PIVOT skipped
  seg.finalHeading = 1.0f;    // [rad] -- TERMINAL_PIVOT target = 1.0 - 0.0 = 1.0 rad

  uint32_t clock = 0;
  exec.start(seg, clock, kTrackwidth);
  checkTrue(exec.active(), "start() leaves the executor active");

  PlantState plant;
  bool everReversedV = false;
  bool everReversedOmega = false;
  msg::BodyTwist3 last;
  int ticks = runToConvergence(exec, plant, &clock, 600, 300.0f, 1.0f, &everReversedV,
                               &everReversedOmega, &last);

  checkTrue(exec.converged(), "translate-then-pivot segment converges within 600 ticks (12s)");
  checkFalse(everReversedV, "commanded v_x never goes negative across TRANSLATE");
  checkFalse(everReversedOmega, "commanded omega never goes negative across TERMINAL_PIVOT");
  checkFloatNear(last.v_x, 0.0f, 1e-3f, "final commanded v_x settles to 0");
  checkFloatNear(last.omega, 0.0f, 1e-6f, "final commanded omega settles to EXACTLY 0 (literal snap)");

  // Geometry check: the average encoder travel is ~300mm (the TRANSLATE
  // phase) and the per-wheel differential settles at ~2*arc = angle*trackwidth
  // (the TERMINAL_PIVOT phase, 1.0 rad * 150mm = 150mm).
  float avg = (plant.encL + plant.encR) * 0.5f;
  float diff = plant.encR - plant.encL;
  // Tolerances widened 8 -> 15 (dead-time-projected terminal firing) -- see
  // the straight-segment scenario's matching comment.
  checkFloatNear(avg, 300.0f, 15.0f, "average encoder travel matches the 300mm TRANSLATE distance");
  checkFloatNear(diff, 150.0f, 15.0f, "encoder differential matches the 1.0rad TERMINAL_PIVOT arc");
  checkTrue(ticks < 600, "converges before the safety net exhausts all 600 ticks");
}

// 3. Pure in-place turn (distance=0): TRANSLATE is skipped entirely; only
// PRE_PIVOT runs (finalHeading == direction, so TERMINAL_PIVOT is also
// skipped) -- this ticket's own "degenerate cases verified" acceptance bar.
void scenarioPureInPlaceTurnSkipsTranslate() {
  beginScenario("pure in-place turn (distance=0): TRANSLATE skipped, only PRE_PIVOT runs");
  Motion::SegmentExecutor exec;
  exec.configure(generousConfig());

  Motion::Segment seg;
  seg.distance = 0.0f;        // TRANSLATE skipped
  seg.direction = 0.8f;       // [rad] -- PRE_PIVOT target
  seg.finalHeading = 0.8f;    // == direction -- TERMINAL_PIVOT skipped too

  uint32_t clock = 0;
  exec.start(seg, clock, kTrackwidth);
  checkTrue(exec.active(), "start() leaves the executor active (PRE_PIVOT still needed)");

  PlantState plant;
  bool everReversedV = false;
  bool everReversedOmega = false;
  msg::BodyTwist3 last;
  int ticks = runToConvergence(exec, plant, &clock, 400, 0.0f, 0.8f, &everReversedV,
                               &everReversedOmega, &last);

  checkTrue(exec.converged(), "pure in-place turn converges within 400 ticks (8s)");
  checkFalse(everReversedV, "commanded v_x stays ~0 -- TRANSLATE never ran");
  checkFalse(everReversedOmega, "commanded omega never goes negative across PRE_PIVOT");
  checkFloatNear(last.v_x, 0.0f, 1e-6f, "final commanded v_x is exactly 0 (TRANSLATE skipped)");
  checkFloatNear(last.omega, 0.0f, 1e-6f, "final commanded omega settles to EXACTLY 0");

  float avg = (plant.encL + plant.encR) * 0.5f;
  float diff = plant.encR - plant.encL;
  checkFloatNear(avg, 0.0f, 3.0f, "average encoder travel stays ~0 -- no TRANSLATE occurred");
  // Tolerance widened 8 -> 15 (dead-time-projected terminal firing) -- see
  // the straight-segment scenario's matching comment.
  checkFloatNear(diff, 0.8f * kTrackwidth, 15.0f, "encoder differential matches the 0.8rad PRE_PIVOT arc");
  checkTrue(ticks < 400, "converges before the safety net exhausts all 400 ticks");
}

// 4. Auto decel-to-zero once the segment's own phases complete: once
// converged() is true, every SUBSEQUENT tick() keeps returning an exact-zero
// twist (idempotent idle -- the segment never "wakes back up" or drifts).
void scenarioAutoDecelToZeroOnceConvergedStaysIdle() {
  beginScenario("auto decel-to-zero on completion: stays converged/idle on every later tick");
  Motion::SegmentExecutor exec;
  exec.configure(generousConfig());

  Motion::Segment seg;
  seg.distance = 200.0f;
  seg.direction = 0.0f;
  seg.finalHeading = 0.0f;

  uint32_t clock = 0;
  exec.start(seg, clock, kTrackwidth);

  PlantState plant;
  bool everReversedV = false;
  bool everReversedOmega = false;
  msg::BodyTwist3 last;
  runToConvergence(exec, plant, &clock, 400, 200.0f, 0.0f, &everReversedV, &everReversedOmega,
                   &last);
  checkTrue(exec.converged(), "precondition: the segment has converged");
  checkFalse(everReversedV, "no reverse observed while converging");

  // Ten further ticks, well past convergence -- must always read back exactly
  // zero, proving there is no lingering per-tick bookkeeping/decay.
  for (int i = 0; i < 10; ++i) {
    clock += 20;
    msg::BodyTwist3 twist = exec.tick(clock, plant.leftObs(), plant.rightObs());
    checkFloatNear(twist.v_x, 0.0f, 1e-9f, "idle tick's v_x is exactly 0");
    checkFloatNear(twist.omega, 0.0f, 1e-9f, "idle tick's omega is exactly 0");
    checkTrue(exec.idle(), "idle() stays true on every post-convergence tick");
  }
}

// 5. A STOP triggered mid-TRANSLATE: the executor gracefully decelerates
// from wherever it was, WITHOUT reaching the original TRANSLATE target and
// WITHOUT proceeding into a still-pending TERMINAL_PIVOT phase (the "abandon
// any remaining phases" contract, architecture-update.md Section 6's "STOP
// triggers the executor's graceful decel-to-zero... then idles").
void scenarioStopMidTranslateAbandonsRemainingPhases() {
  beginScenario(
      "stop() mid-TRANSLATE: gracefully decelerates short of target, abandons the pending "
      "TERMINAL_PIVOT");
  Motion::SegmentExecutor exec;
  exec.configure(generousConfig());

  Motion::Segment seg;
  seg.distance = 500.0f;      // [mm] -- a long TRANSLATE we will interrupt
  seg.direction = 0.0f;
  seg.finalHeading = 1.2f;    // [rad] -- a TERMINAL_PIVOT that must never run

  uint32_t clock = 0;
  exec.start(seg, clock, kTrackwidth);
  checkTrue(exec.active(), "start() leaves the executor active");

  PlantState plant;
  bool everReversedV = false;
  bool everReversedOmega = false;
  const uint32_t kDt = 20;

  // Run partway through TRANSLATE (well short of 500mm at cruise ~300mm/s).
  for (int i = 0; i < 25; ++i) {  // ~500ms
    clock += kDt;
    msg::BodyTwist3 twist = exec.tick(clock, plant.leftObs(), plant.rightObs());
    if (twist.v_x < -0.5f) everReversedV = true;
    plant.advance(twist, static_cast<float>(kDt) * 0.001f);
  }
  float distanceAtStop = (plant.encL + plant.encR) * 0.5f;
  checkTrue(distanceAtStop > 20.0f && distanceAtStop < 400.0f,
            "precondition: partway through TRANSLATE, well short of the 500mm target");

  exec.stop(clock);
  checkTrue(exec.active(), "stop() does not idle instantly -- the graceful decel is still running");

  msg::BodyTwist3 last;
  for (int i = 0; i < 300; ++i) {
    clock += kDt;
    msg::BodyTwist3 twist = exec.tick(clock, plant.leftObs(), plant.rightObs());
    if (twist.v_x < -0.5f) everReversedV = true;
    if (twist.omega < -0.02f || twist.omega > 0.02f) everReversedOmega = true;
    plant.advance(twist, static_cast<float>(kDt) * 0.001f);
    last = twist;
    if (exec.converged()) break;
  }

  checkTrue(exec.converged(), "the forced stop's graceful decel converges");
  checkFalse(everReversedV, "the forced decel-to-zero never reverses");
  checkFalse(everReversedOmega, "TERMINAL_PIVOT never ran -- omega stays ~0 throughout");
  checkFloatNear(last.v_x, 0.0f, 1e-3f, "final commanded v_x settles to 0");

  float finalDistance = (plant.encL + plant.encR) * 0.5f;
  checkTrue(finalDistance < 450.0f,
            "the segment stopped well short of its original 500mm TRANSLATE target");
  float finalDiff = plant.encR - plant.encL;
  checkFloatNear(finalDiff, 0.0f, 5.0f,
                "TERMINAL_PIVOT never ran -- the encoder differential stayed ~0");
}

// 6. [This ticket's own named regression gate] No reverse-creep in the
// terminal decel trace: once a phase's stop condition fires (here, a forced
// stop() mid-PRE_PIVOT), the sampled omega never changes sign, and the
// EXACT tick at which the graceful decel converges reports a LITERAL 0.0f
// (the snap ported from planner.cpp:964-966 -- defeats the PID
// zero-deadband residual reverse-spin) -- not merely "close to zero".
void scenarioNoReverseCreepInTerminalDecelTrace() {
  beginScenario(
      "094-001 named regression: no reverse-creep in the terminal decel trace, literal-0.0f "
      "snap on rotational convergence");
  Motion::SegmentExecutor exec;
  exec.configure(generousConfig());

  Motion::Segment seg;
  seg.distance = 0.0f;
  seg.direction = 1.5f;     // [rad] -- a long PRE_PIVOT we will interrupt
  seg.finalHeading = 1.5f;  // == direction -- TERMINAL_PIVOT would be skipped anyway

  uint32_t clock = 0;
  exec.start(seg, clock, kTrackwidth);

  PlantState plant;
  const uint32_t kDt = 20;

  // Run partway through PRE_PIVOT (well short of the 1.5rad target).
  for (int i = 0; i < 15; ++i) {  // ~300ms
    clock += kDt;
    msg::BodyTwist3 twist = exec.tick(clock, plant.leftObs(), plant.rightObs());
    plant.advance(twist, static_cast<float>(kDt) * 0.001f);
  }

  exec.stop(clock);  // arms the graceful decel-to-zero mid-PRE_PIVOT

  bool everNegative = false;
  bool sawExactZero = false;
  float lastOmega = 1.0f;  // any nonzero sentinel
  for (int i = 0; i < 300; ++i) {
    clock += kDt;
    msg::BodyTwist3 twist = exec.tick(clock, plant.leftObs(), plant.rightObs());
    if (twist.omega < -1e-6f) everNegative = true;
    // Once we observe a literal 0.0f sample, EVERY subsequent sample (up to
    // and including the tick the phase idles out) must also be a literal
    // 0.0f -- proving the snap, not merely an asymptotic approach.
    if (sawExactZero) {
      checkFloatNear(twist.omega, 0.0f, 0.0f,
                    "no reverse-creep: every sample after the first literal-0.0f stays exactly 0");
    }
    if (twist.omega == 0.0f) sawExactZero = true;
    plant.advance(twist, static_cast<float>(kDt) * 0.001f);
    lastOmega = twist.omega;
    if (exec.converged()) break;
  }

  checkFalse(everNegative, "the sampled omega never changes sign after stop() fires");
  checkTrue(sawExactZero, "the decel trace reaches a LITERAL 0.0f sample (the snap), not just near-zero");
  checkFloatNear(lastOmega, 0.0f, 0.0f, "the final sample before convergence is exactly 0.0f");
  checkTrue(exec.converged(), "the forced mid-PRE_PIVOT stop converges");
}

// 7. [Sprint 098 M3] PD correction term is nonzero and in the correcting
// direction. Shadow a PD-enabled executor (B) against an open-loop executor
// (A, Kp=Kd=0) fed the IDENTICAL encoder trajectory: rotational_'s own
// Ruckig solve/replan state depends only on the encoder observations and
// target -- never on heading_kp/heading_kd, which only scale the OUTPUT
// twist after sampling -- so A and B's rotational_ channels evolve
// byte-identically tick for tick as long as they see identical (encL, encR,
// now). B.omega - A.omega therefore isolates EXACTLY the PD correction
// term. A's own plant deliberately under-rotates (trackingFactor 0.5) so
// there is a real, persistent, obviously-signed tracking gap to correct
// against -- proving the loop actually closes, not a no-op.
void scenarioHeadingPdCorrectionNonzeroInCorrectingDirection() {
  beginScenario(
      "sprint-098 M3: PD correction term is nonzero and in the correcting direction against a "
      "lagging plant");

  msg::PlannerConfig openLoopCfg = generousConfig();  // heading_kp/kd default to 0.0f

  msg::PlannerConfig pdCfg = generousConfig();
  pdCfg.heading_kp = 2.0f;  // [1/s] -- "a few /s", per the issue's own loop-separation guidance
  pdCfg.heading_kd = 0.0f;  // isolate the P-term for this scenario

  Motion::SegmentExecutor execA;  // open-loop shadow -- drives the shared plant
  execA.configure(openLoopCfg);
  Motion::SegmentExecutor execB;  // PD-corrected shadow -- fed the SAME encoder trajectory
  execB.configure(pdCfg);

  Motion::Segment seg;
  seg.distance = 0.0f;
  seg.direction = 1.0f;     // [rad] -- PRE_PIVOT only
  seg.finalHeading = 1.0f;  // == direction -- TERMINAL_PIVOT skipped

  uint32_t clock = 0;
  execA.start(seg, clock, kTrackwidth);
  execB.start(seg, clock, kTrackwidth);

  VelocityAwarePlant plant;
  plant.trackingFactor = 0.5f;  // deliberately under-rotates -- a real, persistent lag

  const uint32_t kDt = 20;
  bool sawNonzeroCorrectingTerm = false;
  bool everWrongSign = false;
  for (int i = 0; i < 25; ++i) {
    clock += kDt;
    msg::BodyTwist3 twistA = execA.tick(clock, plant.leftObs(), plant.rightObs());
    msg::BodyTwist3 twistB = execB.tick(clock, plant.leftObs(), plant.rightObs());
    plant.advance(twistA, static_cast<float>(kDt) * 0.001f);  // A's OWN twist drives the shared plant

    float term = twistB.omega - twistA.omega;
    if (term > 0.02f) sawNonzeroCorrectingTerm = true;
    if (term < -1e-3f) everWrongSign = true;
  }

  checkTrue(sawNonzeroCorrectingTerm,
           "PD term is meaningfully nonzero (>0.02 rad/s) at least once while the plant lags");
  checkFalse(everWrongSign,
            "PD term never goes negative (wrong direction) while the plant systematically lags "
            "behind a positive-direction turn");
}

// 8. [Sprint 098 M4] Tolerance+dwell completion: does NOT fire prematurely
// while still outside tolerance or short of the dwell window, DOES fire
// once both |target error| and |rate| hold within tolerance for the full
// dwell -- and a single tick that steps back outside tolerance resets the
// dwell clock (must re-accumulate the FULL window from the re-entry, not
// just top up the gap).
void scenarioToleranceDwellCompletion() {
  beginScenario(
      "sprint-098 M4: tolerance+dwell does not fire early, resets on a tolerance dropout, fires "
      "once held for the full dwell");
  Motion::SegmentExecutor exec;
  exec.configure(generousConfig());

  Motion::Segment seg;
  seg.distance = 0.0f;
  seg.direction = 0.5f;     // [rad] -- PRE_PIVOT only
  seg.finalHeading = 0.5f;  // == direction -- TERMINAL_PIVOT skipped

  uint32_t clock = 0;
  exec.start(seg, clock, kTrackwidth);

  const uint32_t kDt = 20;
  const float kAtTargetDiff = 0.5f * kTrackwidth;     // 75mm -- exactly theta_measured == 0.5rad
  const float kOffTargetDiff = kAtTargetDiff - 5.0f;  // 70mm -- 5mm off, well outside kHeadingTol

  // Ticks 1-5 (100ms): far off target -- must not converge.
  for (int i = 0; i < 5; ++i) {
    clock += kDt;
    exec.tick(clock, fixedEnc(0.0f), fixedEnc(0.0f));
    checkFalse(exec.converged(), "far off target: must not have converged yet");
  }

  // Ticks 6-10 (100ms): AT target -- within tolerance, but short of the
  // 150ms dwell -- must not converge yet.
  for (int i = 0; i < 5; ++i) {
    clock += kDt;
    exec.tick(clock, fixedEnc(0.0f), fixedEnc(kAtTargetDiff));
    checkFalse(exec.converged(), "within tolerance but dwell not yet satisfied: must not converge");
  }

  // Tick 11 (20ms): one tick OUTSIDE tolerance -- resets the dwell clock.
  clock += kDt;
  exec.tick(clock, fixedEnc(0.0f), fixedEnc(kOffTargetDiff));
  checkFalse(exec.converged(), "a single tolerance dropout must not itself complete the phase");

  // Ticks 12-18 (140ms < kHeadingDwellMs): back AT target, held continuously
  // -- must NOT complete before the dwell re-accumulates from THIS re-entry
  // (proves the dwell clock actually reset at tick 11, not just paused).
  bool convergedDuringReaccumulation = false;
  for (int i = 0; i < 7; ++i) {
    clock += kDt;
    exec.tick(clock, fixedEnc(0.0f), fixedEnc(kAtTargetDiff));
    if (exec.converged()) convergedDuringReaccumulation = true;
  }
  checkFalse(convergedDuringReaccumulation,
            "the dwell clock reset at the tolerance dropout -- 140ms back at target is not yet "
            "the full 150ms dwell");

  // Further ticks at target push past the 150ms dwell -- now it must fire
  // and, riding the (already-converged) plan's own tail, complete shortly
  // after.
  bool converged = false;
  for (int i = 0; i < 50 && !converged; ++i) {
    clock += kDt;
    exec.tick(clock, fixedEnc(0.0f), fixedEnc(kAtTargetDiff));
    converged = exec.converged();
  }
  checkTrue(converged, "tolerance+dwell eventually fires once held continuously for >= kHeadingDwellMs");
}

// 9. [Sprint 098 M4, SUC-002 Open Question 2] Dwell-vs-STOP_TIME budget: for
// a representative SLOW (low-ceiling) turn, the tolerance+dwell gate's
// added dwell (<= kHeadingDwellMs, well under the issue's 200ms upper
// bound) does not exhaust the STOP_TIME safety net's own nominal*2+2000ms
// budget -- convergence completes comfortably inside it.
void scenarioDwellDoesNotExhaustStopTimeBudget() {
  beginScenario(
      "sprint-098 M4 (SUC-002 Open Question 2): dwell does not exhaust the STOP_TIME budget for "
      "a slow, low-ceiling turn");
  Motion::SegmentExecutor exec;
  msg::PlannerConfig cfg = generousConfig();
  cfg.yaw_rate_max = 0.3f;  // [rad/s] -- deliberately SLOW, low-ceiling
  cfg.yaw_acc_max = 1.0f;   // [rad/s^2]
  cfg.heading_kp = 2.0f;
  cfg.heading_kd = 0.3f;
  exec.configure(cfg);

  Motion::Segment seg;
  seg.distance = 0.0f;
  seg.direction = 1.2f;  // [rad] -- a substantial slow turn
  seg.finalHeading = 1.2f;

  uint32_t clock = 0;
  exec.start(seg, clock, kTrackwidth);

  // Mirrors beginPrePivot()'s own nominal/STOP_TIME formula exactly, so this
  // assertion tracks the real budget rather than a hand-guessed number.
  float nominal = (fabsf(seg.direction) / cfg.yaw_rate_max) * 1000.0f;  // [ms]
  float stopTimeBudget = nominal * 2.0f + 2000.0f;                     // [ms]

  // VelocityAwarePlant, NOT the bare PlantState: PlantState never populates
  // MotorState.velocity.val (see fixedEnc()'s doc comment), so with a
  // nonzero heading_kd, omega_measured would read a bogus, permanent 0 --
  // turning the D-term into a constant `kd * omega_desired` feedforward
  // boost instead of a real derivative correction, which overdrives the
  // plant past the plan's own velocity ceiling (a test-fixture artifact,
  // confirmed by tracing it: not a real control-law defect -- see this
  // ticket's own completion notes). trackingFactor stays at its 1.0
  // default -- a real, zero-lag/zero-slip plant with HONEST velocity
  // feedback.
  VelocityAwarePlant plant;
  bool everReversedV = false;
  bool everReversedOmega = false;
  msg::BodyTwist3 last;
  int maxTicks = static_cast<int>(stopTimeBudget / 20.0f) + 50;
  const uint32_t kDt = 20;
  int ticks = 0;
  for (; ticks < maxTicks; ++ticks) {
    clock += kDt;
    msg::BodyTwist3 twist = exec.tick(clock, plant.leftObs(), plant.rightObs());
    if (twist.omega < -0.02f) everReversedOmega = true;
    if (twist.v_x < -0.5f) everReversedV = true;
    plant.advance(twist, static_cast<float>(kDt) * 0.001f);
    last = twist;
    if (exec.converged()) { ++ticks; break; }
  }
  (void)everReversedV;
  (void)last;

  checkTrue(exec.converged(), "the slow turn converges at all within the generous loop bound");
  checkFalse(everReversedOmega, "no reversal while converging");
  float elapsedMs = static_cast<float>(ticks) * 20.0f;
  checkTrue(elapsedMs < stopTimeBudget,
           "convergence (including the <=200ms dwell) completes before the STOP_TIME safety "
           "net's own nominal*2+2000ms budget would fire");
}

// 10. [Sprint 098 M5, SUC-002 stall protection] Gross-divergence reanchor
// still fires within its usual divergence-accrual budget when one wheel's
// encoder is artificially held fixed (a simulated stall) against a nonzero
// PRE_PIVOT command -- UNCHANGED by this ticket (only the sub-gross branch
// was retired). Kp=Kd=0 isolates the RAW plan-sampled omega (M3's PD term
// off) so a reanchor's own re-seed-from-near-zero-velocity discontinuity is
// directly visible: an uninterrupted trapezoidal profile toward a target
// large relative to the ceiling only ever rises then plateaus at the
// ceiling for a long cruise -- it never drops this early. An observed drop
// is unambiguous proof the gross-divergence branch fired against the stuck
// wheel.
void scenarioStallProtectionGrossDivergenceStillFires() {
  beginScenario(
      "sprint-098 M5 (SUC-002 stall protection): gross-divergence reanchor still fires against "
      "an artificially stuck wheel");
  Motion::SegmentExecutor exec;
  msg::PlannerConfig cfg = generousConfig();
  cfg.yaw_rate_max = 3.0f;
  cfg.yaw_acc_max = 15.0f;
  cfg.heading_kp = 0.0f;  // isolate the replan's own effect from M3's PD term
  cfg.heading_kd = 0.0f;
  exec.configure(cfg);

  Motion::Segment seg;
  seg.distance = 0.0f;
  seg.direction = 3.0f;  // [rad] -- large relative to the ceiling: a long cruise, so an
                          // uninterrupted profile provably never drops this early
  seg.finalHeading = 3.0f;

  uint32_t clock = 0;
  exec.start(seg, clock, kTrackwidth);

  // Both wheels artificially stuck at 0 -- exactly what a genuinely
  // bogged/stalled drivetrain reports to Hal::Motor's encoders regardless of
  // what the wheel-velocity PID commands it to do.
  msg::MotorState stuck = fixedEnc(0.0f);

  const uint32_t kDt = 20;
  float prevOmega = 0.0f;
  bool sawDrop = false;
  int dropTickIndex = -1;
  const int kEarlyWindowTicks = 25;  // 500ms -- well inside the long cruise, nowhere near decel
  for (int i = 0; i < kEarlyWindowTicks; ++i) {
    clock += kDt;
    msg::BodyTwist3 twist = exec.tick(clock, stuck, stuck);
    if (i > 0 && twist.omega < prevOmega - 0.05f) {
      sawDrop = true;
      if (dropTickIndex < 0) dropTickIndex = i;
    }
    prevOmega = twist.omega;
  }

  checkTrue(sawDrop,
           "a reanchor-induced discontinuity (omega drop) is observed well inside the long "
           "cruise window -- proof the gross-divergence branch fired against the stuck wheel");
  checkTrue(dropTickIndex >= 0 && dropTickIndex <= 20,
           "the reanchor fires within its usual divergence-accrual budget, not late");
}

// 11. [Sprint 098 M5, replan retirement] Under NOMINAL tracking lag -- the
// kind that, pre-sprint, sat in the sub-gross (kRotDivergenceThreshold,
// EXTEND-only) band and would have been chased by retarget()ing the plan to
// compensate -- that branch no longer fires post-sprint: with Kp=Kd=0 (so
// M3's PD term cannot ALSO compensate, isolating the replan's own effect)
// and a plant that persistently under-rotates by a moderate, sub-gross
// factor, the final landed encoder differential lands SHORT of the
// commanded target instead of being chased back onto it.
void scenarioReplanRetirementSubGrossNoLongerFires() {
  beginScenario(
      "sprint-098 M5: under nominal tracking lag, the sub-gross EXTEND branch no longer fires "
      "for PRE_PIVOT/TERMINAL_PIVOT");
  Motion::SegmentExecutor exec;
  msg::PlannerConfig cfg = generousConfig();
  cfg.heading_kp = 0.0f;  // isolate the replan's own (non-)effect from M3's PD term
  cfg.heading_kd = 0.0f;
  exec.configure(cfg);

  Motion::Segment seg;
  seg.distance = 0.0f;
  seg.direction = 1.0f;  // [rad] -- PRE_PIVOT only
  seg.finalHeading = 1.0f;

  uint32_t clock = 0;
  exec.start(seg, clock, kTrackwidth);

  VelocityAwarePlant plant;
  plant.trackingFactor = 0.85f;  // moderate, persistent 15% under-rotation -- a NOMINAL lag,
                                 // not a stall (never crosses kRotGrossDivergenceThreshold)

  bool everReversedOmega = false;
  msg::BodyTwist3 last;
  const uint32_t kDt = 20;
  int i = 0;
  for (; i < 500; ++i) {  // generous bound -- completion now waits on STOP_TIME (Kp=0, so the
                          // phase can never reach the tolerance+dwell gate against an
                          // uncorrected, persistently short plant)
    clock += kDt;
    msg::BodyTwist3 twist = exec.tick(clock, plant.leftObs(), plant.rightObs());
    if (twist.omega < -0.02f) everReversedOmega = true;
    plant.advance(twist, static_cast<float>(kDt) * 0.001f);
    last = twist;
    if (exec.converged()) break;
  }

  checkTrue(exec.converged(), "the phase eventually completes (via the STOP_TIME backstop, with "
                              "nothing else left to correct it)");
  checkFalse(everReversedOmega, "no reversal while converging");

  float finalDiff = plant.encR - plant.encL;               // [mm]
  float commandedTargetDiff = seg.direction * kTrackwidth;  // [mm] -- the UNEXTENDED target
  checkTrue(finalDiff < commandedTargetDiff - 10.0f,
           "with the sub-gross EXTEND branch retired, nothing chases the plant's persistent "
           "under-rotation -- the robot lands measurably SHORT of the original target instead "
           "of being retargeted back onto it");
  (void)last;
}

// 12. [Sprint 098 M6, ticket 004, Stage 2, OPTIONAL] Parity: an invalid/
// absent PoseEstimate produces BIT-IDENTICAL twist output to ticket 002's
// encoder-only path -- the ticket's own load-bearing safety guarantee (any
// caller that never threads a real OTOS pose, or explicitly supplies an
// invalid one, must see EXACTLY Stage 1's behavior, not merely an
// approximation of it). Shadow two executors -- A never receives a pose
// argument at all (the default-parameter path every OTHER scenario in this
// file already exercises); B receives an EXPLICIT, deliberately invalid
// PoseEstimate (stamp.valid == false, a nonzero pose.h to prove it is
// ignored, not just zero-by-coincidence) every tick -- fed the IDENTICAL
// encoder trajectory (A's own twist drives the one shared plant, mirroring
// scenario 7's shadow pattern), across a segment that exercises PRE_PIVOT,
// TRANSLATE, AND TERMINAL_PIVOT (the only phases that ever reach
// measuredHeading(), the ONE place the pose argument is consulted) with
// nonzero heading_kp/heading_kd so the pose-gate itself -- not merely
// Kp=Kd=0 degenerating both paths to the same open-loop passthrough -- is
// what is actually under test.
void scenarioOtosInvalidPoseParityWithEncoderOnlyPath() {
  beginScenario(
      "sprint-098 M6 (ticket 004, Stage 2, optional): invalid/absent PoseEstimate is "
      "BIT-IDENTICAL to ticket 002's encoder-only path");

  msg::PlannerConfig cfg = generousConfig();
  cfg.heading_kp = 2.0f;
  cfg.heading_kd = 0.3f;

  Motion::SegmentExecutor execA;  // default path -- never passed a pose argument
  execA.configure(cfg);
  Motion::SegmentExecutor execB;  // explicit invalid PoseEstimate every tick
  execB.configure(cfg);

  Motion::Segment seg;
  seg.distance = 300.0f;
  seg.direction = 0.8f;      // PRE_PIVOT
  seg.finalHeading = 1.3f;   // TERMINAL_PIVOT target = 1.3 - 0.8 = 0.5 rad

  uint32_t clock = 0;
  execA.start(seg, clock, kTrackwidth);
  execB.start(seg, clock, kTrackwidth);

  msg::PoseEstimate invalidPose;
  invalidPose.pose.h = 2.5f;        // deliberately nonzero -- proves it is ignored, not
                                     // coincidentally zero
  invalidPose.stamp.valid = false;  // the ONLY bit that matters -- explicitly absent/stale

  VelocityAwarePlant plant;   // honest velocity feedback -- exercises the D-term identically
                              // on both shadows (see ticket 002's own completion-notes finding
                              // on PlantState's fabricated-zero-velocity artifact)
  const uint32_t kDt = 20;
  bool everMismatched = false;
  int i = 0;
  for (; i < 700; ++i) {
    clock += kDt;
    msg::BodyTwist3 twistA = execA.tick(clock, plant.leftObs(), plant.rightObs());
    msg::BodyTwist3 twistB = execB.tick(clock, plant.leftObs(), plant.rightObs(), invalidPose);
    if (twistA.v_x != twistB.v_x || twistA.omega != twistB.omega) everMismatched = true;
    plant.advance(twistA, static_cast<float>(kDt) * 0.001f);  // A's own twist drives the ONE
                                                               // shared plant both executors see
    if (execA.converged() && execB.converged()) { ++i; break; }
  }

  checkFalse(everMismatched,
            "an explicit invalid PoseEstimate produces bit-identical twist output to the "
            "default (no-pose-argument) path, every tick");
  checkTrue(execA.converged() && execB.converged(), "both shadows converge");
  checkTrue(i < 700, "both shadows converge within the generous loop bound");
}

// 13. [Sprint 098 M6, ticket 004, Stage 2, OPTIONAL] Source selection: a
// VALID PoseEstimate whose heading deliberately differs from the
// encoder-derived one is actually CONSUMED by measuredHeading() -- proven by
// an observably different PD correction than the encoder-only case, shadowed
// the same way scenario 7 proves the PD term itself is nonzero. A (encoder-
// only, no pose argument) tracks a zero-lag/zero-slip plant almost exactly,
// so its own P-term stays near 0 throughout (nothing to correct against). B
// is fed a fabricated OTOS heading that reports 40% MORE rotation than has
// actually occurred -- a deliberately, obviously wrong reading with no
// physical counterpart, chosen so the resulting divergence from A is
// unambiguous -- proving the measured-heading step preferred pose.h over the
// (still-available, still-consistent) encoder fallback.
void scenarioOtosSourceSelectionUsesOtosHeadingWhenValid() {
  beginScenario(
      "sprint-098 M6 (ticket 004, Stage 2, optional): a valid PoseEstimate whose heading "
      "deliberately differs from the encoder-derived one is actually consumed -- a different PD "
      "correction than the encoder-only case");

  msg::PlannerConfig cfg = generousConfig();
  cfg.heading_kp = 2.0f;
  cfg.heading_kd = 0.0f;  // isolate the P-term, matching scenario 7's own isolation choice

  Motion::SegmentExecutor execA;  // encoder-only (no pose argument at all)
  execA.configure(cfg);
  Motion::SegmentExecutor execB;  // OTOS-sourced -- fed a deliberately offset heading
  execB.configure(cfg);

  Motion::Segment seg;
  seg.distance = 0.0f;
  seg.direction = 1.0f;     // [rad] -- PRE_PIVOT only
  seg.finalHeading = 1.0f;  // == direction -- TERMINAL_PIVOT skipped

  uint32_t clock = 0;
  execA.start(seg, clock, kTrackwidth);
  execB.start(seg, clock, kTrackwidth);

  PlantState plant;   // zero-lag/zero-slip -- the encoder-derived heading tracks the Ruckig
                      // plan's own sample essentially exactly, so A's own P-term stays ~0
                      // throughout (nothing for the encoder path to correct against).
  const uint32_t kDt = 20;
  bool sawDivergentCorrection = false;
  float trueHeading = 0.0f;   // integrated from A's own commanded omega -- the SAME encoder
                              // trajectory both A and B observe via plant.leftObs()/rightObs()
  for (int i = 0; i < 60; ++i) {
    clock += kDt;

    // OTOS deliberately reports 40% MORE rotation than has actually
    // happened -- a fabricated reading with no physical counterpart, chosen
    // so the resulting PD divergence is unambiguous (not a realistic
    // sensor-noise magnitude).
    msg::PoseEstimate otosPose;
    otosPose.pose.h = trueHeading * 1.4f;
    otosPose.stamp.valid = true;

    msg::BodyTwist3 twistA = execA.tick(clock, plant.leftObs(), plant.rightObs());
    msg::BodyTwist3 twistB = execB.tick(clock, plant.leftObs(), plant.rightObs(), otosPose);

    if (fabsf(twistB.omega - twistA.omega) > 0.05f) sawDivergentCorrection = true;

    plant.advance(twistA, static_cast<float>(kDt) * 0.001f);   // both A and B see the SAME
                                                                // encoder trajectory -- A's own
                                                                // twist drives the one shared
                                                                // plant (mirrors scenario 7)
    trueHeading += twistA.omega * static_cast<float>(kDt) * 0.001f;
  }

  checkTrue(sawDivergentCorrection,
           "the OTOS-sourced executor's PD correction diverges measurably from the "
           "encoder-only executor's -- proof the measured-heading step actually consumed "
           "pose.h, not the encoder fallback");
}

}  // namespace

int main() {
  scenarioStraightSegmentSkipsBothPivots();
  scenarioTranslateThenTerminalPivot();
  scenarioPureInPlaceTurnSkipsTranslate();
  scenarioAutoDecelToZeroOnceConvergedStaysIdle();
  scenarioStopMidTranslateAbandonsRemainingPhases();
  scenarioNoReverseCreepInTerminalDecelTrace();
  scenarioHeadingPdCorrectionNonzeroInCorrectingDirection();
  scenarioToleranceDwellCompletion();
  scenarioDwellDoesNotExhaustStopTimeBudget();
  scenarioStallProtectionGrossDivergenceStillFires();
  scenarioReplanRetirementSubGrossNoLongerFires();
  scenarioOtosInvalidPoseParityWithEncoderOnlyPath();
  scenarioOtosSourceSelectionUsesOtosHeadingWhenValid();

  if (g_failureCount > 0) {
    std::printf("\n%d FAILURE(S)\n", g_failureCount);
    return 1;
  }
  std::printf("\nALL SCENARIOS PASSED\n");
  return 0;
}
