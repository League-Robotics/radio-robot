// behavior_lock_harness.cpp -- sprint 111 ticket 001's own numeric
// behavior-lock acceptance instrument (SUC-001): drives a D700 kArc
// straight and a 360deg kPivot to completion through the REAL
// App::RobotLoop/App::Pilot/Motion::Executor graph (TestSim::SimHarness),
// captures a per-cycle wheel-velocity trace, numerically differentiates it
// into accel/jerk, and asserts velocity/accel/jerk bounds (read from the
// SAME msg::PlannerConfig the harness itself boots with -- never a
// hand-duplicated numeric limit), single-lobe shape, and a separately
// reportable "no nonzero command survives past the terminal zero" check.
//
// 112-001 (stakeholder decision, reviews Sec5.3 "differentiate the emitted
// setpoints"): the bounds/lobe checks (_ramp_bounds, _terminal_bounds,
// _single_lobe_*, _lobes_opposite_sign) stopped grading the DECODED,
// MEASURED wheel-velocity trace (this file's pre-112-001 shape) -- grading
// it conflated the commanded trajectory's own jerk-boundedness with the
// downstream velocity-PID/actuation-lag tracking response to it (a ~130ms
// plant lag + write-shaping, unrelated to and unfixable by any Motion::
// Executor/App::Pilot control-law change).
//
// 112-001 first re-pointed these checks at the COMMANDED per-wheel setpoint
// (`SimHarness::driveTargetVelLeft/Right()` -> `Devices::Motor::
// velocityTarget()`, the value `App::Drive::tick()` last wrote via
// `setVelocity()`) and confirmed it was clean post-peek-deletion. 112-002
// then added a DELIBERATE model feedforward (`actuation_lag * a`) into that
// SAME commanded signal -- a real, intentional lag-compensation overshoot,
// not a bug -- which reintroduced jerk-scale content into it (Ruckig's own
// acceleration is only piecewise-LINEAR, so the feedforward's own time
// derivative, `actuation_lag * jerk`, inherits the trajectory's jerk-segment
// step discontinuities). Re-verified directly: with the feedforward genuinely
// engaged, `driveTargetVelLeft/Right()` regressed `straight_ramp_bounds`/
// `straight_terminal_bounds`/`straight_single_lobe_left/right` even though
// nothing about the underlying PLANNED trajectory changed.
//
// Stakeholder resolution (112-002, reviews Sec5.3's OTHER clause -- "record
// requested endpoint, PLANNED endpoint, measured endpoint... as separate
// telemetry values"): keep the feedforward (correct engineering), and grade
// these four checks against a THIRD, distinct signal -- the PLANNED
// reference (`SimHarness::plannedRefLeft/Right()` -> `App::Pilot::
// refLeft/right()`: `Motion::Executor`'s own jerk-limited trajectory,
// `BodyKinematics::inverse(twist.v, twist.omega, ...)`, sampled BEFORE
// `Pilot`'s heading-PD correction and BEFORE `Drive`'s accel feedforward).
// This isolates "is the SOLVED trajectory itself jerk-bounded and
// single-lobed" from both downstream stages (PD reaction to noisy measured
// heading, and the feedforward's own deliberate lag anticipation) -- neither
// of which any Motion::Executor solve can be blamed for.
//
// Three signals, three different jobs, sampled once per Sample alongside the
// decoded telemetry `now` this file already timestamps every sample with:
//   - PLANNED (`plannedRefLeft/Right()`) -- `_ramp_bounds`/`_terminal_bounds`/
//     `_single_lobe_*`/`_lobes_opposite_sign`: is the SOLVED trajectory
//     itself well-shaped.
//   - COMMANDED (`driveTargetVelLeft/Right()`) -- `_shelf_collapsed`
//     (measureShelfCycles(), below): does the FINAL command (PD + FF
//     included) reach exactly zero promptly after completion.
//   - MEASURED (decoded telemetry `vel_left`/`vel_right`) --
//     `_no_command_after_terminal_zero`/`checkNoCommandAfterTerminalZero()`
//     (ticket-003's own check): does the DECODED wire trace ever show a
//     stale nonzero value after it first went quiet.
//
// This is Step 0 of clasi/issues/motion-control-terminal-blips-reconciled-
// fix-plan.md -- "land a numeric jerk / single-lobe acceptance test first
// so every subsequent deletion is guarded." It was originally a PURE
// ADDITION (no production motion code touched by ticket 001); 112-001
// itself is the one ticket permitted to touch this file's own OWN
// commanded-vs-measured grading choice (the harness is this ticket's own
// verification instrument), never firmware. Today's patch stack (straight-
// lead padding, terminal top-up, pivot overshoot lead, the min-speed
// floor) is expected to fail some of these assertions -- that is the
// point. See test_behavior_lock.py for which named checks are wrapped
// `xfail(strict=False)` and why.
//
// Two tiers of assertion, deliberately kept separate:
//   1. Harness-plumbing sanity (checkTrue()/fail(), the same idiom
//      move_queue_harness.cpp/heading_source_harness.cpp already use) --
//      things that must ALWAYS hold (the command was actually injected,
//      no ACK_STATUS_SOLVE_FAIL fired, the command reached
//      ACK_STATUS_DONE within budget). A failure here means the HARNESS
//      itself is broken, not that the sprint's driving issue reproduced --
//      it fails the compile-and-run pytest step outright (nonzero exit),
//      never an xfail candidate.
//   2. Named behavior-lock checks (report()) -- the sprint's own numeric
//      fence. Each prints its own machine-parseable
//      "RESULT: <name> :: PASS" / "RESULT: <name> :: FAIL :: <detail>"
//      line; main() always exits 0 regardless of these. test_behavior_
//      lock.py greps stdout for each named line and asserts PASS,
//      xfail(strict=False)-wrapping whichever checks today's patch stack
//      cannot yet satisfy -- giving each named check independent
//      pass/xfail visibility (implementation plan point 6).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "sim_harness.h"

namespace {

int g_failureCount = 0;
std::string g_scenarioName;

void beginScenario(const std::string& name) {
  g_scenarioName = name;
  std::printf("--- %s\n", name.c_str());
}

// Harness-plumbing sanity check -- see this file's header for why this is
// a SEPARATE tier from report() below.
void checkTrue(bool condition, const std::string& what) {
  if (!condition) {
    ++g_failureCount;
    std::printf("  FAIL [%s]: %s -- expected true, got false\n", g_scenarioName.c_str(), what.c_str());
  }
}

// Named behavior-lock check -- prints a machine-parseable RESULT line the
// Python driver greps for; never touches g_failureCount / the process exit
// code (see this file's header -- these are the sprint's OWN numeric
// fence, which today's patch stack is expected to partially fail).
void report(const std::string& name, bool ok, const std::string& detail = "") {
  if (ok) {
    std::printf("RESULT: %s :: PASS\n", name.c_str());
  } else {
    std::printf("RESULT: %s :: FAIL :: %s\n", name.c_str(), detail.c_str());
    std::printf("  [%s] %s -- %s\n", g_scenarioName.c_str(), name.c_str(), detail.c_str());
  }
}

constexpr float kPi = 3.14159265358979323846f;

// "Settled"/"near zero" bar for a wheel velocity sample -- matches every
// sibling harness's own settle threshold (move_queue_harness.cpp,
// heading_source_harness.cpp both use 15.0f as "close enough to rest").
// Reused here both as the lobe-boundary epsilon and the terminal-zero
// epsilon so the two checks agree on what "zero" means.
constexpr float kNearZero = 15.0f;  // [mm/s]

// Headroom applied to every configured PlannerConfig bound before a
// numerically-differentiated sample counts as a violation. As of 112-001
// the bounds/lobe checks differentiate the COMMANDED per-wheel setpoint
// (this file's own header comment), not the closed-loop MEASURED trace --
// a discretely-sampled, per-cycle commanded value rather than the smooth
// continuous-time curve Motion::JerkTrajectory itself solves, so some
// finite-difference slack above the ideal planned bound is still expected
// even on a perfectly healthy trace (a fresh solve's own first cycle,
// sample-cadence quantization at a 40/100ms-alternating telemetry rate,
// etc). This is NOT a relaxation of the real bound the sprint cares about
// (that bound is still read live from msg::PlannerConfig, never
// hardcoded) -- it is numerical headroom on top of it.
constexpr float kBoundTolerance = 1.35f;

// One decoded primary-telemetry sample this harness cares about: the wire
// `now` [ms] (so per-sample dt is the REAL elapsed time between two
// samples, not an assumed constant -- App::Telemetry's primary/secondary
// tie-alternation, telemetry.cpp's own emit(), means not every sim cycle
// necessarily emits a primary frame), the two MEASURED (decoded, wire)
// wheel velocities, the two COMMANDED (SimHarness::driveTargetVelLeft/
// Right(), 112-001) wheel setpoints, and the two PLANNED (SimHarness::
// plannedRefLeft/Right(), 112-002) wheel references, all three sampled at
// the SAME instant, plus whether an ACK_STATUS_DONE for the command this
// scenario is watching arrived bundled in this same frame. See this file's
// own header comment for which named check grades which of the three.
struct Sample {
  uint32_t nowMs = 0;
  float velLeft = 0.0f;   // [mm/s] signed, MEASURED -- Telemetry::Frame.velLeft off the wire
  float velRight = 0.0f;  // [mm/s] signed, MEASURED -- Telemetry::Frame.velRight off the wire
  float cmdLeft = 0.0f;   // [mm/s] signed, COMMANDED -- SimHarness::driveTargetVelLeft() (112-001)
  float cmdRight = 0.0f;  // [mm/s] signed, COMMANDED -- SimHarness::driveTargetVelRight() (112-001)
  float refLeft = 0.0f;   // [mm/s] signed, PLANNED -- SimHarness::plannedRefLeft() (112-002)
  float refRight = 0.0f;  // [mm/s] signed, PLANNED -- SimHarness::plannedRefRight() (112-002)
  bool ackDone = false;
};

// runToCompletion -- steps `sim` until an ACK_STATUS_DONE for `watchId` is
// observed (or `maxCycles` elapses), then continues for `tailCycles` more
// cycles to capture whatever happens AFTER completion (the hump/shelf the
// driving issue documents). Records every decoded primary telemetry
// frame's own vel_left/vel_right along the way (mirrors move_queue_
// harness.cpp's own findAck() helper, generalized to also harvest the
// velocity trace).
//
// `watchId` is the command's own `id` field (Move's `id`, matching
// Motion::CompletionEvent::id), NOT the wire envelope `corr_id` --
// robot_loop.cpp's own async-completion path (`tlm_.ack(event.id,
// toWireAckStatus(event.status), 0)`) reuses AckEntry.corr_id to carry
// `event.id` for the DONE/SOLVE_FAIL/etc. ack it pushes once a command
// actually finishes, which is a DIFFERENT ack than the immediate
// OK/TRIVIAL/ERR one pushed at accept-time keyed on the wire corr_id
// (`tlm_.ack(env.corr_id, ACK_STATUS_OK, 0)`). A caller that wants to know
// when a command's OWN motion is done must watch its `id`, not its
// injection-time `corrId` -- confirmed empirically while building this
// harness (see this ticket's own completion notes).
//
// *doneIndex is set to the sample index the DONE ack arrived on, or left
// at -1 if it was never observed within maxCycles.
std::vector<Sample> runToCompletion(TestSim::SimHarness& sim, uint32_t watchId, int maxCycles,
                                     int tailCycles, int* doneIndex) {
  std::vector<Sample> samples;
  *doneIndex = -1;
  int cyclesSinceDone = -1;

  for (int i = 0; i < maxCycles; ++i) {
    sim.step(1);
    for (const auto& line : sim.drainTelemetry()) {
      if (line.kind != TestSupport::DecodedKind::kTelemetry) continue;

      Sample s;
      s.nowMs = line.telemetry.now;
      // 112-001/112-002: the COMMANDED and PLANNED signals are both read
      // directly off the live SimHarness -- not wire-decoded, so neither
      // needs has_vel/carry-forward handling of its own; each is simply
      // whatever App::Drive::tick() last wrote via setVelocity() (COMMANDED)
      // or App::Pilot's own last tick() computed (PLANNED) as of THIS sim
      // cycle.
      s.cmdLeft = sim.driveTargetVelLeft();
      s.cmdRight = sim.driveTargetVelRight();
      s.refLeft = sim.plannedRefLeft();
      s.refRight = sim.plannedRefRight();
      if (line.telemetry.has_vel) {
        s.velLeft = line.telemetry.vel_left;
        s.velRight = line.telemetry.vel_right;
      } else if (!samples.empty()) {
        // Every primary frame past boot carries has_vel=true in practice
        // (RobotLoop::updateTlm() always sets it) -- if a frame ever
        // doesn't, carry the last known value forward rather than
        // fabricate a false zero that would look like a lobe boundary.
        s.velLeft = samples.back().velLeft;
        s.velRight = samples.back().velRight;
      }
      for (uint8_t a = 0; a < line.telemetry.acks_count; ++a) {
        if (line.telemetry.acks_[a].corr_id == watchId &&
            line.telemetry.acks_[a].status == msg::AckStatus::ACK_STATUS_DONE) {
          s.ackDone = true;
        }
      }
      samples.push_back(s);
      if (s.ackDone && *doneIndex < 0) *doneIndex = static_cast<int>(samples.size()) - 1;
    }
    if (*doneIndex >= 0) {
      ++cyclesSinceDone;
      if (cyclesSinceDone >= tailCycles) break;
    }
  }
  return samples;
}

// Numerically differentiates `v` (finite difference against each sample's
// own real `nowMs`, per this ticket's own implementation plan point 3).
// out[0] is always 0.0f (no prior sample to difference against) -- reused
// for BOTH velocity->accel and accel->jerk, so out[1] from the SECOND call
// is derived from a placeholder accel[0]=0.0f and is itself a placeholder,
// not a real jerk sample; checkBoundsWindow() below only ever evaluates
// accel from index >=1 and jerk from index >=2, which is exactly what
// skips every placeholder this function produces.
std::vector<float> differentiate(const std::vector<uint32_t>& nowMs, const std::vector<float>& v) {
  std::vector<float> out(v.size(), 0.0f);
  for (size_t i = 1; i < v.size(); ++i) {
    float dtS = static_cast<float>(nowMs[i] - nowMs[i - 1]) / 1000.0f;  // [s]
    if (dtS <= 0.0f) continue;
    out[i] = (v[i] - v[i - 1]) / dtS;
  }
  return out;
}

// A lobe = a maximal run of same-signed, non-near-zero samples. Bounded
// EITHER by a near-zero sample OR by an unbroken sign flip (a trace that
// swings from +20 to -20 mm/s without ever crossing under kNearZero is
// itself exactly the "sign-changing terminal tail" shape the driving issue
// documents for the pivot -- it must end one lobe and start a new one, not
// be silently absorbed into a single run).
struct Lobe {
  int sign = 0;
  int startIdx = 0;
  int endIdx = 0;
};

std::vector<Lobe> findLobes(const std::vector<float>& v, float epsilon) {
  std::vector<Lobe> lobes;
  int n = static_cast<int>(v.size());
  int i = 0;
  while (i < n) {
    if (std::fabs(v[i]) < epsilon) {
      ++i;
      continue;
    }
    int sign = (v[i] > 0.0f) ? 1 : -1;
    int start = i;
    while (i < n && std::fabs(v[i]) >= epsilon && ((v[i] > 0.0f) ? 1 : -1) == sign) ++i;
    lobes.push_back(Lobe{sign, start, i - 1});
  }
  return lobes;
}

std::string lobesToString(const std::vector<Lobe>& lobes) {
  std::string s = std::to_string(lobes.size()) + " lobe(s)";
  for (const auto& l : lobes) {
    s += " [" + std::string(l.sign > 0 ? "+" : "-") + " idx " + std::to_string(l.startIdx) + ".." +
         std::to_string(l.endIdx) + "]";
  }
  return s;
}

// Bound check over an inclusive sample-index window -- used for BOTH the
// "first few cycles after activation" and "last few cycles before/at the
// completion event" windows (implementation plan point 4). vBound/aBound/
// jBound already have kBoundTolerance folded in by the caller. velL/velR
// are whichever velocity series the caller wants graded (112-001: the
// COMMANDED setpoint for these bounds checks -- see this file's own header
// comment) -- accelL/accelR/jerkL/jerkR must already be differentiate()'d
// from that SAME series.
bool checkBoundsWindow(const std::vector<float>& velL, const std::vector<float>& velR,
                        const std::vector<float>& accelL, const std::vector<float>& accelR,
                        const std::vector<float>& jerkL, const std::vector<float>& jerkR,
                        int startIdx, int endIdx, float vBound, float aBound, float jBound,
                        std::string* detail) {
  int n = static_cast<int>(velL.size());
  for (int i = std::max(0, startIdx); i <= std::min(endIdx, n - 1); ++i) {
    if (std::fabs(velL[i]) > vBound || std::fabs(velR[i]) > vBound) {
      *detail = "sample " + std::to_string(i) + " velocity (L=" + std::to_string(velL[i]) +
                " R=" + std::to_string(velR[i]) + ") exceeds bound " + std::to_string(vBound);
      return false;
    }
    if (i >= 1 && (std::fabs(accelL[i]) > aBound || std::fabs(accelR[i]) > aBound)) {
      *detail = "sample " + std::to_string(i) + " accel (L=" + std::to_string(accelL[i]) +
                " R=" + std::to_string(accelR[i]) + ") exceeds bound " + std::to_string(aBound);
      return false;
    }
    if (i >= 2 && (std::fabs(jerkL[i]) > jBound || std::fabs(jerkR[i]) > jBound)) {
      *detail = "sample " + std::to_string(i) + " jerk (L=" + std::to_string(jerkL[i]) +
                " R=" + std::to_string(jerkR[i]) + ") exceeds bound " + std::to_string(jBound);
      return false;
    }
  }
  return true;
}

// "No nonzero command survives past the terminal zero": once both wheels
// first read near-zero AT OR AFTER the completion ack, no LATER sample in
// the captured tail may read nonzero again. This is the ONE check ticket
// 003 (the Pilot::tick() stale-twist-on-idle fix) flips from xfail to
// passing -- kept as its own function/report() call, independent of every
// hump/tail-SHAPE assertion above, per this ticket's own acceptance
// criteria.
bool checkNoCommandAfterTerminalZero(const std::vector<Sample>& samples, int doneIndex,
                                      float epsilon, std::string* detail) {
  if (doneIndex < 0) {
    *detail = "command never reached ACK_STATUS_DONE -- cannot evaluate";
    return false;
  }
  int zeroIdx = -1;
  for (size_t i = static_cast<size_t>(doneIndex); i < samples.size(); ++i) {
    if (std::fabs(samples[i].velLeft) < epsilon && std::fabs(samples[i].velRight) < epsilon) {
      zeroIdx = static_cast<int>(i);
      break;
    }
  }
  if (zeroIdx < 0) {
    *detail = "wheels never reached near-zero after completion within the captured tail";
    return false;
  }
  for (size_t i = static_cast<size_t>(zeroIdx) + 1; i < samples.size(); ++i) {
    if (std::fabs(samples[i].velLeft) >= epsilon || std::fabs(samples[i].velRight) >= epsilon) {
      *detail = "a nonzero sample (L=" + std::to_string(samples[i].velLeft) +
                " R=" + std::to_string(samples[i].velRight) + ") was observed at sample " +
                std::to_string(i) + ", AFTER the terminal zero at sample " + std::to_string(zeroIdx);
      return false;
    }
  }
  return true;
}

// Result handed back to the caller so a scenario-specific follow-up check
// (the pivot's own "opposite sign" requirement, below) can reuse the same
// lobe analysis without re-running the sim.
struct ScenarioLobes {
  bool completed = false;
  std::vector<Lobe> left;
  std::vector<Lobe> right;
};

// runBehaviorLockScenario -- shared body for the D700 straight and the
// 360deg pivot: inject, capture, differentiate, and report every named
// check with `prefix`-qualified names. vBound/aBound/jBound are the
// WHEEL-space bounds (already includes kBoundTolerance) the caller derives
// from the harness's own live msg::PlannerConfig -- never hand-duplicated
// here.
ScenarioLobes runBehaviorLockScenario(TestSim::SimHarness& sim, const std::string& prefix,
                                       float distance, float deltaHeading, float vMax, uint32_t id,
                                       uint32_t corrId, int maxCycles, int tailCycles, float vBound,
                                       float aBound, float jBound) {
  sim.injectMove(distance, deltaHeading, vMax, /*omega=*/0.0f, /*timeMs=*/0.0f, /*replace=*/false, id,
                 corrId);

  int doneIndex = -1;
  std::vector<Sample> samples = runToCompletion(sim, /*watchId=*/id, maxCycles, tailCycles, &doneIndex);

  checkTrue(doneIndex >= 0, prefix + ": command reached ACK_STATUS_DONE within " +
                                std::to_string(maxCycles) + " cycles");
  if (doneIndex < 0) return ScenarioLobes{};  // nothing left to differentiate/report meaningfully

  std::vector<uint32_t> nowMs;
  std::vector<float> refLeft, refRight;
  nowMs.reserve(samples.size());
  refLeft.reserve(samples.size());
  refRight.reserve(samples.size());
  for (const auto& s : samples) {
    nowMs.push_back(s.nowMs);
    refLeft.push_back(s.refLeft);
    refRight.push_back(s.refRight);
  }
  // 112-002 (superseding 112-001's own choice here): the ramp/terminal
  // bounds and single-lobe/lobe-sign checks below all grade the PLANNED
  // reference series (refLeft/refRight) -- Motion::Executor's own
  // jerk-limited trajectory, before Pilot's heading-PD and before Drive's
  // accel feedforward -- not the commanded or measured signal. See this
  // file's own header comment for the full three-signal rationale.
  std::vector<float> accelL = differentiate(nowMs, refLeft);
  std::vector<float> accelR = differentiate(nowMs, refRight);
  std::vector<float> jerkL = differentiate(nowMs, accelL);
  std::vector<float> jerkR = differentiate(nowMs, accelR);

  // Activation index: first sample with either wheel's PLANNED reference
  // already nonzero.
  int activationIdx = -1;
  for (size_t i = 0; i < samples.size(); ++i) {
    if (std::fabs(refLeft[i]) >= kNearZero || std::fabs(refRight[i]) >= kNearZero) {
      activationIdx = static_cast<int>(i);
      break;
    }
  }
  if (activationIdx < 0) activationIdx = 0;

  constexpr int kWindow = 5;  // "a few cycles" -- implementation plan point 4
  std::string detail;

  bool rampOk = checkBoundsWindow(refLeft, refRight, accelL, accelR, jerkL, jerkR, activationIdx,
                                   activationIdx + kWindow - 1, vBound, aBound, jBound, &detail);
  report(prefix + "_ramp_bounds", rampOk, detail);

  bool terminalOk = checkBoundsWindow(refLeft, refRight, accelL, accelR, jerkL, jerkR,
                                       doneIndex - kWindow + 1, doneIndex, vBound, aBound, jBound,
                                       &detail);
  report(prefix + "_terminal_bounds", terminalOk, detail);

  std::vector<Lobe> lobesL = findLobes(refLeft, kNearZero);
  std::vector<Lobe> lobesR = findLobes(refRight, kNearZero);
  report(prefix + "_single_lobe_left", lobesL.size() == 1,
         "left wheel: " + lobesToString(lobesL) + " (expected exactly 1)");
  report(prefix + "_single_lobe_right", lobesR.size() == 1,
         "right wheel: " + lobesToString(lobesR) + " (expected exactly 1)");

  bool terminalZeroOk = checkNoCommandAfterTerminalZero(samples, doneIndex, kNearZero, &detail);
  report(prefix + "_no_command_after_terminal_zero", terminalZeroOk, detail);

  return ScenarioLobes{/*completed=*/true, lobesL, lobesR};
}

// measureShelfCycles -- 111-003 verification instrument. Ticket 001's own
// "no nonzero command survives past the terminal zero" check
// (checkNoCommandAfterTerminalZero(), above) is evaluated against the
// DECODED, MEASURED wheel-velocity trace -- and both this file's
// scenarios (D700 straight, 360deg pivot) already PASS that check even
// with the pre-111-003 bug present, because the ideal sim's own terminal
// decel already drives the MEASURED velocity under the 15mm/s near-zero
// bar by the time completion fires (ticket 001's completion notes: "both
// traces settle to <5mm/s within one cycle of the DONE ack"). Holding a
// value that is ALREADY near zero stale for the ~300ms deadman-lease
// window never crosses that bar again, so the measured trace cannot
// distinguish the fixed and unfixed timing.
//
// This function instead measures the COMMANDED PID target
// (SimHarness::driveTargetVelLeft/Right() -> Devices::Motor::
// velocityTarget(), the value App::Drive::tick() last wrote via
// setVelocity() -- see that accessor's own doc comment in sim_harness.h),
// which has no such headroom: pre-fix, App::Pilot::tick() takes NEITHER
// twist-staging branch on a natural running->idle transition, so
// App::Drive keeps commanding whatever twist was staged the PREVIOUS
// cycle until the ~300ms deadman lease force-stops it
// (Drive::stop(), robot_loop.cpp); post-fix, Pilot::tick() stages
// drive_.setTwist(0,0) on the SAME cycle the transition happens, so the
// NEXT cycle's Drive::tick() already commands exactly 0. Counting cycles
// from the completion ack to the first cycle the commanded target reads
// EXACTLY 0.0f measures this timing directly, independent of how close
// to zero the terminal twist already was.
//
// Returns the shelf length in cycles (0 = the cycle immediately at/after
// completion already commands exactly 0; -1 = the command never reached
// ACK_STATUS_DONE within maxCycles; -2 = it completed but the commanded
// target never read exactly 0 within the captured tail).
int measureShelfCycles(TestSim::SimHarness& sim, uint32_t watchId, int maxCycles, int tailCycles) {
  int doneCycle = -1;
  int zeroCycle = -1;
  int cyclesSinceDone = -1;
  for (int i = 0; i < maxCycles; ++i) {
    sim.step(1);
    for (const auto& line : sim.drainTelemetry()) {
      if (line.kind != TestSupport::DecodedKind::kTelemetry) continue;
      for (uint8_t a = 0; a < line.telemetry.acks_count; ++a) {
        if (doneCycle < 0 && line.telemetry.acks_[a].corr_id == watchId &&
            line.telemetry.acks_[a].status == msg::AckStatus::ACK_STATUS_DONE) {
          doneCycle = i;
        }
      }
    }
    if (doneCycle >= 0 && zeroCycle < 0 && sim.driveTargetVelLeft() == 0.0f &&
        sim.driveTargetVelRight() == 0.0f) {
      zeroCycle = i;
    }
    if (doneCycle >= 0) {
      ++cyclesSinceDone;
      if (cyclesSinceDone >= tailCycles) break;
    }
  }
  if (doneCycle < 0) return -1;
  if (zeroCycle < 0) return -2;
  return zeroCycle - doneCycle;
}

// runShelfScenario -- drives one fresh Move to completion on its own
// SimHarness instance and reports measureShelfCycles()'s own named check.
// A fresh instance per scenario (rather than reusing the trace-capture
// SimHarness above) keeps this verification fully independent of the
// hump/tail-shape assertions above -- this ticket's own fix has nothing
// to do with the ramp/lobe-shape findings sprint 2 owns.
void runShelfScenario(const std::string& prefix, float distance, float deltaHeading, float vMax,
                       uint32_t id, uint32_t corrId) {
  beginScenario(prefix + ": shelf length (commanded-target reaches exactly 0 after completion)");
  TestSim::SimHarness sim;
  sim.boot();
  sim.step(3);
  sim.injectMove(distance, deltaHeading, vMax, /*omega=*/0.0f, /*timeMs=*/0.0f, /*replace=*/false, id,
                 corrId);
  int shelf = measureShelfCycles(sim, /*watchId=*/id, /*maxCycles=*/400, /*tailCycles=*/30);
  std::printf("  %s shelf length: %d cycle(s)\n", prefix.c_str(), shelf);
  checkTrue(shelf != -1, prefix + ": command reached ACK_STATUS_DONE within budget (shelf measurement)");
  report(prefix + "_shelf_collapsed", shelf >= 0 && shelf <= 2,
         "shelf=" + std::to_string(shelf) +
             " cycles (expected <=2; a large value here is the pre-111-003 deadman-lease shelf, "
             "~300ms of stale commanded twist)");
}

// runSameBootScenario -- SUC-001 step 5: ONE SimHarness instance, booted
// once, driving 30-50 consecutive alternating straight/pivot Move commands
// with NO reboot between them (unlike turn_windage_sweep.py's own
// deliberate per-run isolation), asserting every one reaches
// ACK_STATUS_DONE. Targets the driving issue's Sec1.8/F7 stale-executor-
// state reliability finding.
void runSameBootScenario() {
  beginScenario("same-boot: 40 consecutive alternating D700 straight / 360deg pivot moves");
  TestSim::SimHarness sim;
  sim.boot();
  sim.step(3);

  constexpr int kMoveCount = 40;  // within the ticket's own 30-50 range
  constexpr float kDistance = 700.0f;
  constexpr float kPivotDeltaHeading = 2.0f * kPi;
  constexpr float kVMax = 400.0f;
  constexpr int kMaxCyclesPerMove = 400;

  int completedCount = 0;
  std::vector<int> failedIndices;

  for (int i = 0; i < kMoveCount; ++i) {
    bool isPivot = (i % 2) == 1;
    uint32_t corrId = 9000 + static_cast<uint32_t>(i);
    uint32_t id = 100 + static_cast<uint32_t>(i);
    if (isPivot) {
      sim.injectMove(0.0f, kPivotDeltaHeading, 0.0f, 0.0f, 0.0f, false, id, corrId);
    } else {
      sim.injectMove(kDistance, 0.0f, kVMax, 0.0f, 0.0f, false, id, corrId);
    }

    bool done = false;
    for (int c = 0; c < kMaxCyclesPerMove && !done; ++c) {
      sim.step(1);
      for (const auto& line : sim.drainTelemetry()) {
        if (line.kind != TestSupport::DecodedKind::kTelemetry) continue;
        for (uint8_t a = 0; a < line.telemetry.acks_count; ++a) {
          // The completion ack is keyed on the command's own `id`, not the
          // injection-time `corrId` -- see runToCompletion()'s own doc
          // comment above for why.
          if (line.telemetry.acks_[a].corr_id == id &&
              line.telemetry.acks_[a].status == msg::AckStatus::ACK_STATUS_DONE) {
            done = true;
          }
        }
      }
    }
    if (done) {
      ++completedCount;
    } else {
      failedIndices.push_back(i);
    }
  }

  bool allDone = (completedCount == kMoveCount);
  std::string detail;
  if (!allDone) {
    detail = std::to_string(kMoveCount - completedCount) + "/" + std::to_string(kMoveCount) +
             " moves never reached ACK_STATUS_DONE within budget (indices:";
    for (int idx : failedIndices) detail += " " + std::to_string(idx);
    detail +=
        ") -- possible match for the driving issue's Sec1.8/F7 stale-executor-state finding, needs "
        "citation in the ticket's completion notes either way";
  }
  std::printf("  same-boot: %d/%d moves completed\n", completedCount, kMoveCount);
  report("same_boot_all_moves_completed", allDone, detail);
}

}  // namespace

int main() {
  std::printf("=== Behavior-Lock Acceptance Harness (111-001, SUC-001) ===\n\n");

  // --- D700 straight (kArc, deltaHeading=0) ---
  {
    beginScenario("D700 straight (kArc): capture, differentiate, assert bounds/shape");
    TestSim::SimHarness sim;
    sim.boot();
    sim.step(3);

    msg::PlannerConfig cfg = sim.plannerConfig();
    float vBound = cfg.v_body_max * kBoundTolerance;
    float aBound = std::max(cfg.a_max, cfg.a_decel) * kBoundTolerance;
    float jBound = cfg.j_max * kBoundTolerance;
    std::printf("  cfg: v_body_max=%.1f a_max=%.1f a_decel=%.1f j_max=%.1f (wheel-space, mm/s "
                "family)\n",
                cfg.v_body_max, cfg.a_max, cfg.a_decel, cfg.j_max);

    runBehaviorLockScenario(sim, "straight", /*distance=*/700.0f, /*deltaHeading=*/0.0f,
                             /*vMax=*/400.0f, /*id=*/1, /*corrId=*/1001, /*maxCycles=*/400,
                             /*tailCycles=*/30, vBound, aBound, jBound);
  }

  // --- 360deg pivot (kPivot, distance=0) ---
  {
    beginScenario("360deg pivot (kPivot): capture, differentiate, assert bounds/shape");
    TestSim::SimHarness sim;
    sim.boot();
    sim.step(3);

    msg::PlannerConfig cfg = sim.plannerConfig();
    float halfTrack = TestSim::kDefaultTrackWidth * 0.5f;  // [mm] wheel offset from the body's own yaw axis
    float vBound = cfg.yaw_rate_max * halfTrack * kBoundTolerance;
    float aBound = cfg.yaw_acc_max * halfTrack * kBoundTolerance;
    float jBound = cfg.yaw_jerk_max * halfTrack * kBoundTolerance;
    std::printf("  cfg: yaw_rate_max=%.2f yaw_acc_max=%.2f yaw_jerk_max=%.2f (rad family) * "
                "halfTrack=%.1fmm\n",
                cfg.yaw_rate_max, cfg.yaw_acc_max, cfg.yaw_jerk_max, halfTrack);

    ScenarioLobes lobes = runBehaviorLockScenario(sim, "pivot", /*distance=*/0.0f,
                                                   /*deltaHeading=*/2.0f * kPi, /*vMax=*/0.0f, /*id=*/2,
                                                   /*corrId=*/2001, /*maxCycles=*/400,
                                                   /*tailCycles=*/30, vBound, aBound, jBound);

    // "one +lobe and one -lobe instead of a single lobe" (SUC-001 step 4,
    // Architecture "What Changed"): a pure pivot spins the two wheels in
    // OPPOSITE directions -- each wheel's own single lobe (pivot_single_
    // lobe_left/right, asserted inside runBehaviorLockScenario() above)
    // should carry the opposite sign from its sibling. This is what
    // distinguishes the pivot's own lobe-shape requirement from the
    // straight's (where both wheels share one sign) -- reported as its own
    // named check rather than folded into the single-lobe checks, so a
    // reader can tell "wrong lobe count" apart from "right count, wrong
    // sign relationship" at a glance.
    if (lobes.completed && lobes.left.size() == 1 && lobes.right.size() == 1) {
      bool oppositeSign = lobes.left[0].sign != lobes.right[0].sign;
      report("pivot_lobes_opposite_sign", oppositeSign,
             "left lobe sign=" + std::to_string(lobes.left[0].sign) +
                 ", right lobe sign=" + std::to_string(lobes.right[0].sign) + " (expected opposite)");
    } else {
      report("pivot_lobes_opposite_sign", false,
             "cannot evaluate sign relationship -- left/right did not each show exactly one lobe "
             "(see pivot_single_lobe_left/right above)");
    }
  }

  // --- Shelf-length verification (111-003, own SimHarness instances) ---
  runShelfScenario("straight", /*distance=*/700.0f, /*deltaHeading=*/0.0f, /*vMax=*/400.0f, /*id=*/11,
                    /*corrId=*/1011);
  runShelfScenario("pivot", /*distance=*/0.0f, /*deltaHeading=*/2.0f * kPi, /*vMax=*/0.0f, /*id=*/12,
                    /*corrId=*/1012);

  runSameBootScenario();

  std::printf("\n");
  if (g_failureCount == 0) {
    std::printf("OK: harness plumbing sane; see RESULT: lines above for the behavior-lock fence "
                "itself (pytest interprets those, PASS/FAIL/xfail).\n");
    return 0;
  }
  std::printf("HARNESS FAILURE: %d plumbing assertion(s) failed (NOT a behavior-lock finding -- "
              "see FAIL lines above)\n",
              g_failureCount);
  return 1;
}
