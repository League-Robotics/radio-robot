// drive_policy_harness.cpp -- off-hardware acceptance harness for ticket
// 100-005 (SUC-005/SUC-006/SUC-007): exercises Drive::evaluate() (source/
// drive/policy.{h,cpp}) directly against synthetic RefState/TrackerOutput/
// StepInput values -- mirroring drive_tracker_harness.cpp's compile-and-run
// pattern -- hand-rolled assertions, no gtest/pytest-native C++ framework,
// run via test_drive_policy.py.
//
// Exercising evaluate() directly (rather than through a real solved
// MotionPlan) gives exact, deterministic control over every branch's
// trigger conditions -- the full closed-loop step()-composition tests
// (against a real plan + a first-order plant stub) live in
// drive_step_harness.cpp instead (this ticket's own split, matching the
// dispatch's own "policy harness: mechanics in isolation; step harness:
// closed-loop composition" division).
//
// Scenarios:
//  (a) replan trigger/hold/rate-limit/N-max-then-abort (RUNNING phase).
//  (b) terminal SETTLING: inside-tolerance dwell + literal-0.0f snap to
//      DONE_STOP; held-short-of-dwell does NOT report DONE_STOP early.
//  (c) terminal walk-in bands (outside: floor/ceiling-clamped forward;
//      overshot: 0.0f) -- a dedicated no-reversal regression asserting the
//      emitted setpoint is NEVER negative anywhere in SETTLING.
//  (d) flying-handoff envelope pass (DONE_HANDOFF) / violate (REPLAN_DUE,
//      same tick, sustain bypassed).
//  (e) pose-fix small-absorb (resets sustain) vs. large-bypass (immediate
//      REPLAN_DUE, sustain bypassed); a poseStep during an active terminal
//      dwell does NOT reset/extend the dwell and does NOT trigger a replan.
//  (f) timeout-never-silent: a non-convergent SETTLING plant reaches
//      ABORT_TIMEOUT at T_plan+1.5s (far outside 2x tolerance), never an
//      infinite SETTLING and never a silent DONE_STOP.
//  (g) purity/determinism: the same (plan-facts, ref, tracked, in, state)
//      fed to evaluate() twice produces byte-identical PolicyResult AND
//      resulting StepState.
#include <cmath>
#include <cstdio>
#include <string>

#include "drive/motion_plan.h"
#include "drive/policy.h"
#include "drive/tracker.h"
#include "drive/types.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors drive_tracker_harness.cpp) ---

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

void checkNear(double actual, double expected, double tol, const std::string& what) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected ~%g (tol %g), got %g", what.c_str(), expected,
                  tol, actual);
    fail(buf);
  }
}

void checkExactly(double actual, double expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected EXACTLY %g, got %g", what.c_str(), expected,
                  actual);
    fail(buf);
  }
}

void checkGe(double actual, double bound, const std::string& what) {
  if (!(actual >= bound)) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected >= %g, got %g", what.c_str(), bound, actual);
    fail(buf);
  }
}

template <typename T>
void checkEnum(T actual, T expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %d, got %d", what.c_str(),
                  static_cast<int>(expected), static_cast<int>(actual));
    fail(buf);
  }
}

// --- Shared fixtures ---

Drive::Limits makeLimits() {
  Drive::Limits limits;
  limits.trackKS = 2.0f;  // [1/s] k_s -- the ONLY Limits field evaluate() reads
  return limits;
}

Drive::RefState makeRef(float v, float omega) {
  Drive::RefState ref;
  ref.v = v;
  ref.omega = omega;
  return ref;
}

Drive::TrackerOutput makeTracked(float eAlong, float eCross, float eTheta, bool trimSaturated,
                                  const Drive::WheelVelocities& command) {
  Drive::TrackerOutput out;
  out.eAlong = eAlong;
  out.eCross = eCross;
  out.eTheta = eTheta;
  out.trimSaturated = trimSaturated;
  out.command = command;
  return out;
}

Drive::StepInput makeInput(float t, float vX, float poseStep, float poseStepTheta) {
  Drive::StepInput in;
  in.t = t;
  in.measured.twist.v_x = vX;
  in.poseStep = poseStep;
  in.poseStepTheta = poseStepTheta;
  return in;
}

// --- (a) replan trigger/hold/rate-limit/N-max-then-abort ---

void scenarioReplanTriggerSustainRateLimitNMax() {
  beginScenario("replan: trigger holds sustain, rate-limits between fires, aborts at N-max");

  const Drive::Limits limits = makeLimits();
  const Drive::RefState ref = makeRef(200.0f, 0.0f);  // envelope: 40+0.25*200=90mm along
  // eAlong = 200mm >> 90mm envelope; trimSaturated true -- trigger active
  // continuously for the whole scenario.
  const Drive::TrackerOutput tracked =
      makeTracked(200.0f, 0.0f, 0.0f, /*trimSaturated=*/true, Drive::WheelVelocities{100.0f, 100.0f});

  Drive::StepState state;
  const float duration = 100.0f;  // never exhausted in this scenario
  const float exitSpeed = 100.0f;  // flying, irrelevant while t < duration
  const float dt = 0.01f;

  int replanEvents = 0;
  float lastReplanTime = -1.0f;
  bool sawFirstReplanBeforeSustain = false;
  bool sawAbort = false;
  float t = 0.0f;

  for (int i = 0; i < 20000 && !sawAbort; ++i, t += dt) {
    Drive::StepInput in = makeInput(t, 0.0f, 0.0f, 0.0f);
    Drive::PolicyResult result =
        Drive::evaluate(duration, exitSpeed, /*isPivot=*/false, /*isVelocityMode=*/false, limits,
                        ref, tracked, in, &state);

    if (result.status == Drive::Status::REPLAN_DUE) {
      ++replanEvents;
      if (replanEvents == 1 && t < 0.199f) sawFirstReplanBeforeSustain = true;
      if (lastReplanTime >= 0.0f) {
        checkGe(t - lastReplanTime, 0.300 - 1e-6, "consecutive REPLAN_DUE events are rate-limited >=300ms apart");
      }
      lastReplanTime = t;
    } else if (result.status == Drive::Status::ABORT_REPLAN_LIMIT) {
      sawAbort = true;
    } else {
      checkEnum(result.status, Drive::Status::RUNNING, "no other status possible mid-trigger");
    }
  }

  checkFalse(sawFirstReplanBeforeSustain, "no REPLAN_DUE fires before the 200ms sustain hold elapses");
  checkTrue(replanEvents == 3, "exactly 3 REPLAN_DUE events fire before the N-max abort");
  checkTrue(sawAbort, "a 4th attempt (N-max=3 already reached) aborts with ABORT_REPLAN_LIMIT");
  checkExactly(state.replanCount, 3, "StepState.replanCount stops incrementing at N-max (3)");
}

void scenarioReplanTriggerResetsWhenClear() {
  beginScenario("replan: sustain resets to -1 when the trigger clears before it fires");

  const Drive::Limits limits = makeLimits();
  const Drive::RefState ref = makeRef(200.0f, 0.0f);
  // trimSaturated false -- outOfEnvelope alone must NOT trigger anything.
  const Drive::TrackerOutput tracked =
      makeTracked(200.0f, 0.0f, 0.0f, /*trimSaturated=*/false, Drive::WheelVelocities{});

  Drive::StepState state;
  for (float t = 0.0f; t < 1.0f; t += 0.01f) {
    Drive::StepInput in = makeInput(t, 0.0f, 0.0f, 0.0f);
    Drive::PolicyResult result =
        Drive::evaluate(100.0f, 100.0f, false, false, limits, ref, tracked, in, &state);
    checkEnum(result.status, Drive::Status::RUNNING, "un-saturated large error never triggers a replan");
  }
  checkNear(state.sustainStart, -1.0, 1e-6, "sustainStart stays -1 -- trigger never activated (not saturated)");
  checkExactly(state.replanCount, 0, "replanCount stays 0");
}

// --- (b) terminal SETTLING: dwell + literal-0.0f snap; no early DONE_STOP ---

void scenarioTerminalInsideToleranceDwellAndSnap() {
  beginScenario("terminal: inside-tolerance dwell holds 150ms before DONE_STOP; snaps to literal 0.0f");

  const Drive::Limits limits = makeLimits();
  const Drive::RefState ref = makeRef(0.0f, 0.0f);
  // eAlong native = 0 -> issueEAlong = 0 -- dead center of the tolerance band.
  const Drive::TrackerOutput tracked =
      makeTracked(0.0f, 0.0f, 0.0f, false, Drive::WheelVelocities{0.0f, 0.0f});

  Drive::StepState state;
  const float duration = 5.0f;
  bool sawDoneStop = false;
  float doneStopTime = -1.0f;

  for (float t = duration; t < duration + 1.0f; t += 0.01f) {
    Drive::StepInput in = makeInput(t, 5.0f /* v_hat, well within 15mm/s */, 0.0f, 0.0f);
    Drive::PolicyResult result =
        Drive::evaluate(duration, 0.0f, false, false, limits, ref, tracked, in, &state);

    checkExactly(result.command.left, 0.0, "inside-tolerance band: left wheel is a literal 0.0f");
    checkExactly(result.command.right, 0.0, "inside-tolerance band: right wheel is a literal 0.0f");

    if (!sawDoneStop) {
      // Must not report DONE_STOP before the 150ms dwell has genuinely
      // held -- the dwell starts at t == duration (first tick inside the
      // band), so completion cannot occur before duration + 0.150.
      if (t < duration + 0.150f - 1e-3f) {
        checkTrue(result.status == Drive::Status::SETTLING,
                  "held short of the 150ms dwell: status stays SETTLING, never DONE_STOP early");
      }
      if (result.status == Drive::Status::DONE_STOP) {
        sawDoneStop = true;
        doneStopTime = t;
      }
    }
  }

  checkTrue(sawDoneStop, "DONE_STOP eventually fires once the dwell holds");
  checkGe(doneStopTime - duration, 0.150 - 1e-3, "DONE_STOP fires no earlier than duration + 150ms");
  checkNear(doneStopTime - duration, 0.150, 0.02, "DONE_STOP fires close to duration + 150ms (not much later)");
}

// --- (c) walk-in bands + no-reversal regression ---

void scenarioTerminalWalkInBandsNeverNegative() {
  beginScenario("terminal walk-in: outside (floor/ceiling clamp) and overshot (0.0f) -- NEVER negative");

  const Drive::Limits limits = makeLimits();
  const Drive::RefState ref = makeRef(0.0f, 0.0f);
  const float duration = 5.0f;
  Drive::StepState state;

  // "outside", short of goal by an amount that needs the FLOOR (k_s*20=40 < 50).
  {
    const Drive::TrackerOutput tracked =
        makeTracked(-20.0f /* native eAlong; issueEAlong = +20 */, 0.0f, 0.0f, false,
                    Drive::WheelVelocities{});
    Drive::StepInput in = makeInput(duration, 0.0f, 0.0f, 0.0f);
    Drive::PolicyResult result = Drive::evaluate(duration, 0.0f, false, false, limits, ref, tracked,
                                                 in, &state);
    checkExactly(result.command.left, 50.0, "outside band, below floor: clamped UP to the 50mm/s stiction floor");
    checkExactly(result.command.right, 50.0, "same for the right wheel");
    checkEnum(result.status, Drive::Status::SETTLING, "still SETTLING while walking in");
  }

  // "outside", an amount that needs the CEILING (k_s*80=160 > 100).
  {
    Drive::StepState freshState;
    const Drive::TrackerOutput tracked =
        makeTracked(-80.0f /* issueEAlong = +80 */, 0.0f, 0.0f, false, Drive::WheelVelocities{});
    Drive::StepInput in = makeInput(duration, 0.0f, 0.0f, 0.0f);
    Drive::PolicyResult result = Drive::evaluate(duration, 0.0f, false, false, limits, ref, tracked,
                                                 in, &freshState);
    checkExactly(result.command.left, 100.0, "outside band, above ceiling: clamped DOWN to 100mm/s");
    checkExactly(result.command.right, 100.0, "same for the right wheel");
  }

  // "overshot": issueEAlong < -tol -- command must be a literal 0.0f, NEVER negative.
  {
    Drive::StepState freshState;
    const Drive::TrackerOutput tracked =
        makeTracked(30.0f /* native eAlong; issueEAlong = -30, overshot beyond 15mm tol */, 0.0f,
                    0.0f, false, Drive::WheelVelocities{});
    Drive::StepInput in = makeInput(duration, 0.0f, 0.0f, 0.0f);
    Drive::PolicyResult result = Drive::evaluate(duration, 0.0f, false, false, limits, ref, tracked,
                                                 in, &freshState);
    checkExactly(result.command.left, 0.0, "overshot band: left wheel is a literal 0.0f, never negative");
    checkExactly(result.command.right, 0.0, "overshot band: right wheel is a literal 0.0f, never negative");
  }

  // Dedicated no-reversal regression: sweep a WIDE grid of native eAlong
  // values (short and overshot, small and large) and assert the emitted
  // setpoint is NEVER negative anywhere in SETTLING, for every tick of a
  // walk-in-to-completion run.
  {
    const float nativeEAlongs[] = {500.0f, 100.0f, 40.0f, 16.0f, 0.5f, 0.0f,
                                    -0.5f,  -16.0f, -40.0f, -100.0f, -500.0f};
    long ticksChecked = 0;
    for (float nativeEAlong : nativeEAlongs) {
      Drive::StepState sweepState;
      const Drive::TrackerOutput tracked =
          makeTracked(nativeEAlong, 0.0f, 0.0f, false, Drive::WheelVelocities{});
      for (float t = duration; t < duration + 2.0f; t += 0.02f) {
        Drive::StepInput in = makeInput(t, 0.0f, 0.0f, 0.0f);
        Drive::PolicyResult result = Drive::evaluate(duration, 0.0f, false, false, limits, ref,
                                                     tracked, in, &sweepState);
        ++ticksChecked;
        if (result.command.left < 0.0f || result.command.right < 0.0f) {
          fail("NEGATIVE wheel command observed in SETTLING at t=" + std::to_string(t) +
               " nativeEAlong=" + std::to_string(nativeEAlong));
        }
      }
    }
    checkTrue(ticksChecked > 500, "swept a genuinely wide SETTLING grid (>500 ticks)");
  }
}

// --- (d) flying handoff: pass / violate ---

void scenarioFlyingHandoffPassAndViolate() {
  beginScenario("flying handoff: within envelope -> DONE_HANDOFF; violated -> REPLAN_DUE same tick");

  const Drive::Limits limits = makeLimits();
  const Drive::RefState ref = makeRef(150.0f, 0.0f);
  const float duration = 5.0f;
  const float exitSpeed = 150.0f;  // vExit -- along budget = 0.14*150+40 = 61mm

  // Pass: eCross=10<=30, eTheta=0.05<0.0873(5deg), eAlong=30<=61.
  {
    Drive::StepState state;
    const Drive::WheelVelocities keepCommand{140.0f, 140.0f};
    const Drive::TrackerOutput tracked = makeTracked(30.0f, 10.0f, 0.05f, false, keepCommand);
    Drive::StepInput in = makeInput(duration, 150.0f, 0.0f, 0.0f);
    Drive::PolicyResult result =
        Drive::evaluate(duration, exitSpeed, false, false, limits, ref, tracked, in, &state);
    checkEnum(result.status, Drive::Status::DONE_HANDOFF, "within the handoff envelope -> DONE_HANDOFF");
    checkExactly(result.command.left, 140.0, "handoff pass: command stays the tracked cascade output");
    checkExactly(result.command.right, 140.0, "handoff pass: command stays the tracked cascade output");
  }

  // Violate: eCross=50 > 30mm -- immediate REPLAN_DUE, no sustain wait needed.
  {
    Drive::StepState state;
    const Drive::TrackerOutput tracked = makeTracked(30.0f, 50.0f, 0.05f, false, Drive::WheelVelocities{});
    Drive::StepInput in = makeInput(duration, 150.0f, 0.0f, 0.0f);
    Drive::PolicyResult result =
        Drive::evaluate(duration, exitSpeed, false, false, limits, ref, tracked, in, &state);
    checkEnum(result.status, Drive::Status::REPLAN_DUE,
              "handoff envelope violated -> REPLAN_DUE on the SAME tick (sustain bypassed)");
    checkExactly(state.replanCount, 1, "replanCount increments on the immediate handoff-violation replan");
  }
}

// --- (e) pose-fix small-absorb vs large-bypass; suppressed during dwell ---

void scenarioPoseFixAbsorbBypassAndDwellSuppression() {
  beginScenario("pose-fix: <=30mm/3deg absorbed (sustain reset); >30mm/3deg bypasses sustain; never during dwell");

  const Drive::Limits limits = makeLimits();
  const Drive::RefState ref = makeRef(200.0f, 0.0f);
  const float duration = 100.0f;  // RUNNING phase throughout

  // Small pose-fix: resets an in-progress sustain timer, no replan.
  {
    Drive::StepState state;
    state.sustainStart = 1.0f;  // simulate mid-accumulation
    const Drive::TrackerOutput tracked = makeTracked(10.0f, 0.0f, 0.0f, false, Drive::WheelVelocities{});
    Drive::StepInput in = makeInput(1.05f, 0.0f, 20.0f /* <=30mm */, 0.0f);
    Drive::PolicyResult result =
        Drive::evaluate(duration, 100.0f, false, false, limits, ref, tracked, in, &state);
    checkEnum(result.status, Drive::Status::RUNNING, "a small pose-fix step never itself triggers a replan");
    checkNear(state.sustainStart, -1.0, 1e-6, "small pose-fix step resets sustainStart to -1 (fresh grace)");
  }

  // Large pose-fix (position): immediate REPLAN_DUE, sustain bypassed.
  {
    Drive::StepState state;
    const Drive::TrackerOutput tracked = makeTracked(10.0f, 0.0f, 0.0f, false, Drive::WheelVelocities{});
    Drive::StepInput in = makeInput(1.0f, 0.0f, 45.0f /* >30mm */, 0.0f);
    Drive::PolicyResult result =
        Drive::evaluate(duration, 100.0f, false, false, limits, ref, tracked, in, &state);
    checkEnum(result.status, Drive::Status::REPLAN_DUE,
              "a large pose-fix step (position) triggers REPLAN_DUE on the SAME tick");
    checkExactly(state.replanCount, 1, "replanCount increments");
  }

  // Large pose-fix (heading only): same bypass behavior.
  {
    Drive::StepState state;
    const Drive::TrackerOutput tracked = makeTracked(10.0f, 0.0f, 0.0f, false, Drive::WheelVelocities{});
    const float threeDeg = 3.0f * 3.14159265358979323846f / 180.0f;
    Drive::StepInput in = makeInput(1.0f, 0.0f, 0.0f, threeDeg + 0.01f /* >3deg */);
    Drive::PolicyResult result =
        Drive::evaluate(duration, 100.0f, false, false, limits, ref, tracked, in, &state);
    checkEnum(result.status, Drive::Status::REPLAN_DUE,
              "a large pose-fix step (heading) triggers REPLAN_DUE on the SAME tick");
  }

  // A poseStep injected while the terminal dwell is actively counting: does
  // NOT reset/extend the dwell, does NOT trigger a replan -- the segment
  // completes on its pre-step basis.
  {
    const float termDuration = 5.0f;
    Drive::StepState state;
    const Drive::TrackerOutput tracked =
        makeTracked(0.0f, 0.0f, 0.0f, false, Drive::WheelVelocities{});  // inside tolerance

    // First tick: enter SETTLING, start the dwell (no pose-fix yet).
    Drive::StepInput firstIn = makeInput(termDuration, 0.0f, 0.0f, 0.0f);
    Drive::PolicyResult firstResult = Drive::evaluate(termDuration, 0.0f, false, false, limits, ref,
                                                      tracked, firstIn, &state);
    checkEnum(firstResult.status, Drive::Status::SETTLING, "first tick at t==duration: SETTLING, dwell starts");
    const float dwellStartAfterFirst = state.dwellStart;
    checkGe(dwellStartAfterFirst, 0.0, "dwell has started");

    // Second tick, shortly after: a LARGE poseStep arrives while the dwell
    // is already counting -- must NOT reset the dwell, must NOT replan.
    Drive::StepInput poseFixIn = makeInput(termDuration + 0.02f, 0.0f, 50.0f /* large */, 0.0f);
    Drive::PolicyResult poseFixResult = Drive::evaluate(termDuration, 0.0f, false, false, limits, ref,
                                                        tracked, poseFixIn, &state);
    checkEnum(poseFixResult.status, Drive::Status::SETTLING,
              "a poseStep mid-dwell does NOT trigger REPLAN_DUE");
    checkNear(state.dwellStart, dwellStartAfterFirst, 1e-9,
              "a poseStep mid-dwell does NOT reset/extend dwellStart");
    checkExactly(state.replanCount, 0, "no replan was requested despite the large poseStep, mid-dwell");
  }
}

// --- (f) timeout-never-silent ---

void scenarioTimeoutNeverSilent() {
  beginScenario("timeout: a non-convergent SETTLING plant reaches ABORT_TIMEOUT, never silent/infinite");

  const Drive::Limits limits = makeLimits();
  const Drive::RefState ref = makeRef(0.0f, 0.0f);
  const float duration = 5.0f;

  // Stuck WAY outside tolerance (issueEAlong = +50, beyond 2x the 15mm
  // tolerance) for the plant's entire life -- never converges, never
  // overshoots -- the dwell can never start.
  {
    Drive::StepState state;
    const Drive::TrackerOutput tracked =
        makeTracked(-50.0f /* issueEAlong = +50 */, 0.0f, 0.0f, false, Drive::WheelVelocities{});
    bool sawAbort = false;
    for (float t = duration; t < duration + 2.0f; t += 0.02f) {
      Drive::StepInput in = makeInput(t, 0.0f, 0.0f, 0.0f);
      Drive::PolicyResult result =
          Drive::evaluate(duration, 0.0f, false, false, limits, ref, tracked, in, &state);
      if (t < duration + 1.5f - 1e-3f) {
        checkTrue(result.status == Drive::Status::SETTLING,
                  "never DONE_STOP/DONE_HANDOFF before the T_plan+1.5s timeout, non-convergent plant");
      } else {
        if (result.status == Drive::Status::ABORT_TIMEOUT) sawAbort = true;
        checkFalse(result.status == Drive::Status::DONE_STOP,
                   "far outside 2x tolerance at timeout: NEVER a silent DONE_STOP");
      }
    }
    checkTrue(sawAbort, "ABORT_TIMEOUT is explicitly reached at T_plan+1.5s -- never a silent/infinite SETTLING");
  }

  // Stuck just outside 1x but within 2x tolerance: timeout resolves as
  // complete-with-warning (DONE_STOP), an EXPLICIT status either way --
  // never left hanging in SETTLING forever.
  {
    Drive::StepState state;
    const Drive::TrackerOutput tracked =
        makeTracked(-20.0f /* issueEAlong = +20; tol=15, 2x tol=30 */, 0.0f, 0.0f, false,
                    Drive::WheelVelocities{});
    Drive::Status finalStatus = Drive::Status::SETTLING;
    for (float t = duration; t < duration + 2.0f; t += 0.02f) {
      Drive::StepInput in = makeInput(t, 0.0f, 0.0f, 0.0f);
      Drive::PolicyResult result =
          Drive::evaluate(duration, 0.0f, false, false, limits, ref, tracked, in, &state);
      finalStatus = result.status;
    }
    checkEnum(finalStatus, Drive::Status::DONE_STOP,
              "within 2x tolerance at timeout: complete-with-warning (DONE_STOP), not ABORT_TIMEOUT");
  }
}

// --- (g) purity / determinism ---

void scenarioPurityDeterminism() {
  beginScenario("purity: same (plan-facts, ref, tracked, in, state) fed to evaluate() twice matches exactly");

  const Drive::Limits limits = makeLimits();
  const Drive::RefState ref = makeRef(150.0f, 0.01f);
  const Drive::TrackerOutput tracked =
      makeTracked(45.0f, 12.0f, 0.03f, true, Drive::WheelVelocities{88.0f, 92.0f});
  Drive::StepInput in = makeInput(2.34f, 33.0f, 5.0f, 0.001f);

  Drive::StepState stateA;
  stateA.sustainStart = 1.9f;
  stateA.lastReplan = 0.5f;
  stateA.replanCount = 1;
  Drive::StepState stateB = stateA;  // identical copy

  Drive::PolicyResult resultA =
      Drive::evaluate(5.0f, 100.0f, false, false, limits, ref, tracked, in, &stateA);
  Drive::PolicyResult resultB =
      Drive::evaluate(5.0f, 100.0f, false, false, limits, ref, tracked, in, &stateB);

  checkEnum(resultA.status, resultB.status, "same inputs -> same Status");
  checkExactly(resultA.command.left, resultB.command.left, "same inputs -> byte-identical left wheel command");
  checkExactly(resultA.command.right, resultB.command.right, "same inputs -> byte-identical right wheel command");
  checkExactly(stateA.dwellStart, stateB.dwellStart, "resulting StepState.dwellStart identical");
  checkExactly(stateA.sustainStart, stateB.sustainStart, "resulting StepState.sustainStart identical");
  checkExactly(stateA.lastReplan, stateB.lastReplan, "resulting StepState.lastReplan identical");
  checkExactly(stateA.replanCount, stateB.replanCount, "resulting StepState.replanCount identical");
  checkExactly(stateA.settling ? 1.0 : 0.0, stateB.settling ? 1.0 : 0.0,
               "resulting StepState.settling identical");
}

}  // namespace

int main() {
  scenarioReplanTriggerSustainRateLimitNMax();
  scenarioReplanTriggerResetsWhenClear();
  scenarioTerminalInsideToleranceDwellAndSnap();
  scenarioTerminalWalkInBandsNeverNegative();
  scenarioFlyingHandoffPassAndViolate();
  scenarioPoseFixAbsorbBypassAndDwellSuppression();
  scenarioTimeoutNeverSilent();
  scenarioPurityDeterminism();

  if (g_failureCount == 0) {
    std::printf("OK: all Drive:: policy scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Drive:: policy scenarios\n", g_failureCount);
  return 1;
}
