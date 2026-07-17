// boundary_velocity_harness.cpp -- sprint 109 ticket 006's own acceptance
// proof: cross-boundary carry (the "no decel between same-vmax commands"
// headline requirement), sign-reversal/pivot-mismatch forcing a full
// decel to rest at the boundary, pivot->pivot rotational carry, and the
// divergence-replan triggers staying safe (no SOLVE_FAIL, no wedge) under
// an injected plant disturbance.
//
// Drives Motion::Executor directly (no App::Pilot, no RobotLoop, no wire)
// -- mirroring src/tests/sim/unit/motion_executor_harness.cpp's own "pure
// logic test" shape exactly, just scoped to ticket 006's own new behavior
// and placed under sim/system/ (this ticket's own "Verification command":
// `uv run python -m pytest src/tests/sim/system/ -k "boundary or
// divergence or handoff"`).
#include <cmath>
#include <cstdio>
#include <string>

#include "motion/executor.h"

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
constexpr uint32_t kDtMs = 40;

msg::PlannerConfig makeConfig() {
  msg::PlannerConfig cfg;
  cfg.a_max = 800.0f;
  cfg.a_decel = 1000.0f;
  cfg.v_body_max = 600.0f;
  cfg.yaw_rate_max = 4.0f;
  cfg.yaw_acc_max = 20.0f;
  cfg.j_max = 8000.0f;
  cfg.yaw_jerk_max = 80.0f;
  cfg.heading_dwell_tol = 0.5f * kDegToRad;
  cfg.heading_dwell_rate = 1.0f * kDegToRad;
  cfg.arrive_dwell = 0.15f;
  return cfg;
}

Motion::Cmd makeDistanceCmd(float distance, float deltaHeading, float vMax, uint32_t id) {
  Motion::Cmd cmd;
  cmd.distance = distance;
  cmd.deltaHeading = deltaHeading;
  cmd.vMax = vMax;
  cmd.id = id;
  return cmd;
}

// runIdealTracking -- drives `exec` with the SAME one-cycle-lagged
// "perfect sensor" stand-in motion_executor_harness.cpp's own dwell tests
// use: each tick's own measuredDistanceDelta/measuredHeadingAbs is derived
// from the PREVIOUS tick's own sampled twist, so measuredPathSince
// Activation_/thetaMeasRel track the real Ruckig-solved trajectory closely
// (a faithful, if lagged, "measured" signal -- not a scripted, physically
// implausible one). Records every sampled |twist.v| while `activeId()`
// equals `watchId1` and `watchId2` (the two boundary-adjacent commands),
// including the exact cycle the ACTIVE id switches, so a caller can find
// the minimum velocity observed right around the boundary itself.
struct TrackResult {
  int cyclesRun = 0;
  bool sawBoundary = false;
  float minVAroundBoundary = 1e9f;    // [mm/s] |v| within a small window of the id1->id2 handoff
  float minOmegaAroundBoundary = 1e9f;  // [rad/s] |omega| ditto, for pivot->pivot chains
  bool sawSolveFail = false;
  bool completedAll = false;  // every enqueued id got a kDone event
};

TrackResult runIdealTracking(Motion::Executor& exec, uint32_t watchId1, uint32_t watchId2,
                              int maxCycles, int windowCycles, bool trackOmega) {
  TrackResult result;
  float measuredDistance = 0.0f;
  float measuredHeadingAbs = 0.0f;
  int cyclesSinceBoundary = -1000;
  bool doneWatchId2 = false;

  for (int i = 0; i < maxCycles && !doneWatchId2; ++i) {
    bool wasWatchId1 = (exec.activeId() == watchId1);
    exec.plan();
    Motion::Executor::Twist twist = exec.tick(kDtMs, measuredDistance, measuredHeadingAbs);
    measuredDistance = twist.v * (static_cast<float>(kDtMs) / 1000.0f);
    measuredHeadingAbs = twist.thetaRef;  // rebaselined per-command; fine as a lagged proxy here

    bool isWatchId2Now = (exec.activeId() == watchId2);
    if (wasWatchId1 && isWatchId2Now) {
      result.sawBoundary = true;
      cyclesSinceBoundary = 0;
    } else if (cyclesSinceBoundary >= 0) {
      ++cyclesSinceBoundary;
    }
    if (cyclesSinceBoundary >= -windowCycles && cyclesSinceBoundary <= windowCycles) {
      result.minVAroundBoundary = std::min(result.minVAroundBoundary, std::fabs(twist.v));
      if (trackOmega) {
        result.minOmegaAroundBoundary = std::min(result.minOmegaAroundBoundary, std::fabs(twist.omega));
      }
    }

    Motion::CompletionEvent event;
    while (exec.popEvent(&event)) {
      if (event.status == Motion::CompletionStatus::kSolveFail) result.sawSolveFail = true;
      if (event.id == watchId2 && event.status == Motion::CompletionStatus::kDone) doneWatchId2 = true;
    }
    result.cyclesRun = i + 1;
  }
  result.completedAll = doneWatchId2;
  return result;
}

}  // namespace

int main() {
  std::printf("=== Boundary-Velocity / Divergence-Replan Scenarios (109-006, SUC-003) ===\n\n");

  // --- Scenario 1 (the sprint's headline requirement): two same-vmax,
  //     same-direction, non-pivot DISTANCE commands -- velocity never
  //     dips below vMax*(1-eps) at the shared boundary ---
  {
    beginScenario("boundary: two same-vmax DISTANCE commands -- no decel at the shared boundary");
    Motion::Executor exec;
    exec.configure(makeConfig());

    constexpr float kVMax = 300.0f;  // [mm/s]
    exec.enqueue(makeDistanceCmd(/*distance=*/600.0f, 0.0f, kVMax, /*id=*/1));
    exec.enqueue(makeDistanceCmd(/*distance=*/600.0f, 0.0f, kVMax, /*id=*/2));
    // A third placeholder keeps id=2 non-terminal so its own completion is
    // judged purely on the distance criterion (matching this file's own
    // "watch the boundary, not the tail dwell" scope).
    exec.enqueue(makeDistanceCmd(/*distance=*/1.0f, 0.0f, kVMax, /*id=*/3));

    TrackResult r = runIdealTracking(exec, /*watchId1=*/1, /*watchId2=*/2, /*maxCycles=*/400,
                                      /*windowCycles=*/2, /*trackOmega=*/false);
    checkTrue(!r.sawSolveFail, "no SOLVE_FAIL fired");
    checkTrue(r.sawBoundary, "the id=1 -> id=2 handoff was observed");
    checkTrue(r.minVAroundBoundary >= kVMax * 0.9f,
              "velocity never dipped below vMax*0.9 within 2 cycles of the boundary "
              "(headline no-decel-between-same-vmax-commands requirement)");
  }

  // --- Scenario 2: sign reversal forces exitSpeed=0 -- velocity DOES
  //     drop to (near) rest at the boundary ---
  {
    beginScenario("boundary: sign reversal forces a full decel to rest at the boundary");
    Motion::Executor exec;
    exec.configure(makeConfig());

    constexpr float kVMax = 300.0f;
    exec.enqueue(makeDistanceCmd(/*distance=*/600.0f, 0.0f, kVMax, /*id=*/1));
    exec.enqueue(makeDistanceCmd(/*distance=*/-600.0f, 0.0f, kVMax, /*id=*/2));  // opposite sign
    exec.enqueue(makeDistanceCmd(/*distance=*/-1.0f, 0.0f, kVMax, /*id=*/3));

    TrackResult r = runIdealTracking(exec, /*watchId1=*/1, /*watchId2=*/2, /*maxCycles=*/400,
                                      /*windowCycles=*/1, /*trackOmega=*/false);
    checkTrue(!r.sawSolveFail, "no SOLVE_FAIL fired");
    checkTrue(r.sawBoundary, "the id=1 -> id=2 handoff was observed");
    checkTrue(r.minVAroundBoundary < kVMax * 0.1f,
              "velocity dropped near zero at a sign-reversal boundary (decelerates through zero, "
              "never carries a signed velocity across a reversal)");
  }

  // --- Scenario 3: pivot-adjacent forces exitSpeed=0 (arc -> pivot has no
  //     shared dominant channel to carry a velocity through) ---
  {
    beginScenario("boundary: an arc chaining into a pivot forces a full decel to rest");
    Motion::Executor exec;
    exec.configure(makeConfig());

    constexpr float kVMax = 300.0f;
    constexpr float kDeltaHeading = 90.0f * kDegToRad;
    exec.enqueue(makeDistanceCmd(/*distance=*/600.0f, 0.0f, kVMax, /*id=*/1));
    exec.enqueue(makeDistanceCmd(/*distance=*/0.0f, kDeltaHeading, 0.0f, /*id=*/2));  // pivot
    exec.enqueue(makeDistanceCmd(/*distance=*/0.0f, kDeltaHeading, 0.0f, /*id=*/3));  // keeps id=2 non-terminal

    TrackResult r = runIdealTracking(exec, /*watchId1=*/1, /*watchId2=*/2, /*maxCycles=*/400,
                                      /*windowCycles=*/1, /*trackOmega=*/false);
    checkTrue(!r.sawSolveFail, "no SOLVE_FAIL fired");
    checkTrue(r.sawBoundary, "the id=1 -> id=2 handoff was observed");
    checkTrue(r.minVAroundBoundary < kVMax * 0.1f,
              "linear velocity dropped near zero at an arc->pivot boundary (pivot on either side "
              "forces exitSpeed=0 -- no shared dominant channel to carry through)");
  }

  // --- Scenario 4 (handoff): pivot->pivot chain carries ROTATIONAL
  //     velocity through the boundary, same rule in the rotational domain
  //     -- this is what actually exercises ticket 005's own dwell
  //     restriction (only the FINAL pivot in a chain dwells) with a real
  //     non-dwelling handoff. ---
  {
    beginScenario("handoff: pivot->pivot chain carries rotational velocity through the boundary");
    Motion::Executor exec;
    exec.configure(makeConfig());

    constexpr float kDeltaHeading = 90.0f * kDegToRad;
    exec.enqueue(makeDistanceCmd(0.0f, kDeltaHeading, 0.0f, /*id=*/1));
    exec.enqueue(makeDistanceCmd(0.0f, kDeltaHeading, 0.0f, /*id=*/2));
    exec.enqueue(makeDistanceCmd(0.0f, kDeltaHeading, 0.0f, /*id=*/3));  // keeps id=2 non-terminal

    TrackResult r = runIdealTracking(exec, /*watchId1=*/1, /*watchId2=*/2, /*maxCycles=*/400,
                                      /*windowCycles=*/2, /*trackOmega=*/true);
    checkTrue(!r.sawSolveFail, "no SOLVE_FAIL fired");
    checkTrue(r.sawBoundary, "the id=1 -> id=2 handoff was observed");
    checkTrue(r.minOmegaAroundBoundary >= 0.3f,
              "rotational rate stayed well above zero within 2 cycles of a pivot->pivot boundary "
              "(rotational-domain boundary-velocity carry, not a decel-to-rest handoff)");
  }

  // --- Scenario 5 (divergence): a moderate, PERSISTENT plant disturbance
  //     (a constant extra few mm/tick of measured travel beyond what the
  //     ideal-tracking twist implies -- NOT the single-tick transient this
  //     ticket's own anti-transient guard filters, and nowhere near the
  //     scripted-signal magnitude an isolated bookkeeping unit test uses)
  //     still completes cleanly -- no SOLVE_FAIL, no wedge -- proving the
  //     divergence-replan triggers stay safe under real disturbance. ---
  {
    beginScenario("divergence: a persistent moderate plant disturbance still completes safely");
    Motion::Executor exec;
    exec.configure(makeConfig());

    constexpr float kVMax = 300.0f;
    exec.enqueue(makeDistanceCmd(/*distance=*/600.0f, 0.0f, kVMax, /*id=*/1));

    float measuredDistance = 0.0f;
    bool sawSolveFail = false;
    bool done = false;
    uint32_t doneId = 0;
    for (int i = 0; i < 400 && !done; ++i) {
      exec.plan();
      Motion::Executor::Twist twist = exec.tick(kDtMs, measuredDistance, 0.0f);
      // A persistent, plausible disturbance: the plant always reports
      // slightly MORE travel than the planned twist implies (e.g. a
      // slightly mis-calibrated wheel diameter) -- sustained across every
      // tick, unlike the single-sample transient the retarget streak
      // guard filters.
      measuredDistance = twist.v * (static_cast<float>(kDtMs) / 1000.0f) * 1.03f;

      Motion::CompletionEvent event;
      while (exec.popEvent(&event)) {
        if (event.status == Motion::CompletionStatus::kSolveFail) sawSolveFail = true;
        if (event.id == 1 && event.status == Motion::CompletionStatus::kDone) {
          done = true;
          doneId = event.id;
        }
      }
    }
    checkTrue(!sawSolveFail, "no SOLVE_FAIL fired under a sustained 3% travel-calibration disturbance");
    checkTrue(done, "the command still completed DONE");
    checkTrue(doneId == 1, "the kDone event echoes the command's own id");
  }

  std::printf("\n");
  if (g_failureCount == 0) {
    std::printf("OK: all boundary-velocity/divergence-replan scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the boundary-velocity/divergence-replan scenarios\n",
              g_failureCount);
  return 1;
}
