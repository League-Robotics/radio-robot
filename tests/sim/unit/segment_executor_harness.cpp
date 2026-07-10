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

}  // namespace

int main() {
  scenarioStraightSegmentSkipsBothPivots();
  scenarioTranslateThenTerminalPivot();
  scenarioPureInPlaceTurnSkipsTranslate();
  scenarioAutoDecelToZeroOnceConvergedStaysIdle();
  scenarioStopMidTranslateAbandonsRemainingPhases();
  scenarioNoReverseCreepInTerminalDecelTrace();

  if (g_failureCount > 0) {
    std::printf("\n%d FAILURE(S)\n", g_failureCount);
    return 1;
  }
  std::printf("\nALL SCENARIOS PASSED\n");
  return 0;
}
