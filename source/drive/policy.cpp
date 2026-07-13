// policy.cpp -- Drive:: policy implementation. See policy.h for the
// class-level design notes (the sign-convention reconciliation, the
// terminal-machine derivation for both non-pivot and pivot segments, the
// replan-envelope/sustain/rate-limit/N-max mechanism, the flying-handoff
// envelope, and the pose-fix absorption/bypass contract). Every numeric
// constant below is named for, and commented with, its exact source in the
// driving issue's "Control laws and numbers" section -- see policy.h's own
// class comment for the two constants that are NOT transcribed (the
// pivot-terminal tolerances, explicitly flagged there).
#include "drive/policy.h"

#include <math.h>

namespace Drive {

namespace {

// ---- Replan envelopes (issue's own table, transcribed verbatim) ----
constexpr float kAlongEnvelopeBase = 40.0f;  // [mm]
constexpr float kAlongEnvelopeRate = 0.25f;  // [s] * |v_ref| [mm/s] -> [mm]
constexpr float kCrossEnvelope = 35.0f;      // [mm] flat
constexpr float kThetaEnvelopeBase = 0.15f;  // [rad]
constexpr float kThetaEnvelopeRate = 0.20f;  // [s] * |omega_ref| [rad/s] -> [rad]

// ---- Sustain / rate-limit / N-max (issue's own table, transcribed) ----
constexpr float kSustainHold = 0.200f;      // [s] 200ms continuous trigger before replan
constexpr float kReplanRateLimit = 0.300f;  // [s] >=300ms between replan requests
constexpr uint8_t kReplanNMax = 3;          // 4th attempt -> ABORT_REPLAN_LIMIT

// ---- Terminal machine: non-pivot walk-in (issue's own numbers) ----
constexpr float kDwellHold = 0.150f;      // [s] 150ms hold -> DONE_STOP
constexpr float kWalkInFloor = 50.0f;     // [mm/s] stiction floor
constexpr float kWalkInCeiling = 100.0f;  // [mm/s]
// Completion gate: issue states "|e_along| <= 10-15mm" -- a range, not a
// single value; 15mm (the range's own upper bound) is chosen here as the
// initial constant (not a Limits field this ticket -- see policy.h's file
// list note in the ticket; a later bench-tuning ticket may promote it).
constexpr float kArriveTolPos = 15.0f;    // [mm]
constexpr float kArriveTolVel = 15.0f;    // [mm/s] issue's own exact value
constexpr float kTimeoutGrace = 1.5f;     // [s] T_plan + 1.5s
constexpr float kTimeoutToleranceFactor = 2.0f;  // "within 2x tolerance"

// ---- Terminal machine: pivot (documented judgment call -- policy.h) ----
constexpr float kPivotArriveTolTheta = 0.02f;  // [rad] ~1.15 deg
constexpr float kPivotArriveTolOmega = 0.05f;  // [rad/s]

// ---- Pose-fix absorb/bypass threshold (issue's own numbers) ----
constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
constexpr float kPoseFixTolPos = 30.0f;                 // [mm]
constexpr float kPoseFixTolTheta = 3.0f * kDegToRad;    // [rad] 3 deg

// ---- Flying handoff envelope (issue's own numbers) ----
constexpr float kHandoffCrossTol = 30.0f;             // [mm]
constexpr float kHandoffThetaTol = 5.0f * kDegToRad;  // [rad] 5 deg
constexpr float kHandoffAlongRate = 0.14f;            // unitless, * vExit
constexpr float kHandoffAlongBase = 40.0f;            // [mm]

// attemptReplan -- the shared rate-limit/N-max gate behind every replan
// request site (RUNNING's envelope trigger, a large pose-fix step, and the
// flying-handoff envelope violation) -- policy.h's own "sustain/rate-limit/
// N-max" section. The CALLER decides whether the sustain wait has already
// been satisfied (or is being deliberately bypassed); this function only
// applies the rate limit and the N-max cap, both of which "still apply"
// (the issue's own phrase) regardless of how the request was triggered.
//
// On a successful fire (Status::REPLAN_DUE), resets sustainStart/dwellStart/
// settling to their "fresh segment" values -- whatever policy-timer progress
// existed against the OLD plan is moot once the caller swaps to a re-timed
// one; replanCount/lastReplan persist (the whole point of tracking them
// across the segment's replan history).
Status attemptReplan(float t, StepState* state) {
  if (state->replanCount >= kReplanNMax) {
    return Status::ABORT_REPLAN_LIMIT;
  }
  const bool rateLimited = (state->lastReplan >= 0.0f) && ((t - state->lastReplan) < kReplanRateLimit);
  if (rateLimited) {
    // Not yet allowed -- the caller keeps tracking (RUNNING) and this same
    // trigger re-evaluates next tick (the caller's own sustainStart, if any,
    // is left untouched by this function so it can retry).
    return Status::RUNNING;
  }
  state->replanCount += 1;
  state->lastReplan = t;
  state->sustainStart = -1.0f;
  state->dwellStart = -1.0f;
  state->settling = false;
  return Status::REPLAN_DUE;
}

}  // namespace

PolicyResult evaluate(float duration, float exitSpeed, bool isPivot, bool isVelocityMode,
                      const Limits& limits, const RefState& ref, const TrackerOutput& tracked,
                      const StepInput& in, StepState* state) {
  PolicyResult result;
  result.command = tracked.command;  // default: pass the tracker cascade through unmodified

  const float t = in.t;

  // ---- MOVER velocity-mode deadman: short-circuits everything else ----
  // motion_plan.h's own doc comment: "the SAME terminal machine ... also
  // handles MOVER's deadman elapsing" -- there is no pose goal to converge
  // on in velocity mode, so once the deadman elapses the only correct
  // action is to stop, immediately, never a walk-in.
  if (isVelocityMode) {
    if (t >= duration) {
      result.status = Status::DONE_STOP;
      result.command = WheelVelocities{};
    } else {
      result.status = Status::RUNNING;
    }
    return result;
  }

  const bool holdingDwellAtStart = state->dwellStart >= 0.0f;
  const bool poseStepPresent = (in.poseStep > 0.0f) || (in.poseStepTheta != 0.0f);
  const bool poseStepLarge =
      (in.poseStep > kPoseFixTolPos) || (fabsf(in.poseStepTheta) > kPoseFixTolTheta);
  const bool exhausted = t >= duration;

  // ---- RUNNING: replan-envelope evaluation (t < duration) ----
  if (!exhausted) {
    result.status = Status::RUNNING;

    // Pose-fix absorption/bypass -- never during a terminal dwell, but
    // holdingDwellAtStart is trivially false here (dwellStart only ever
    // becomes non-negative inside the SETTLING branch below), so this
    // guard is really "always eligible" during RUNNING; written the same
    // way as the SETTLING branch below for a single, consistent contract.
    if (!holdingDwellAtStart && poseStepPresent) {
      if (poseStepLarge) {
        result.status = attemptReplan(t, state);
        return result;
      }
      state->sustainStart = -1.0f;  // small step: absorbed by trims, fresh grace
    }

    const float alongEnv = kAlongEnvelopeBase + kAlongEnvelopeRate * fabsf(ref.v);
    const float thetaEnv = kThetaEnvelopeBase + kThetaEnvelopeRate * fabsf(ref.omega);
    const bool outOfEnvelope = (fabsf(tracked.eAlong) > alongEnv) ||
                               (fabsf(tracked.eCross) > kCrossEnvelope) ||
                               (fabsf(tracked.eTheta) > thetaEnv);
    // trim-saturated trigger = saturated AND outside envelope (issue's own
    // table). Pivot mode's omegaTrim is UNCLAMPED by construction
    // (tracker.cpp) -- trimSaturated is always false there, so this
    // condition structurally never fires during a pivot's RUNNING phase;
    // only the pose-fix bypass above can request a pivot replan.
    const bool triggerNow = tracked.trimSaturated && outOfEnvelope;

    if (triggerNow) {
      if (state->sustainStart < 0.0f) state->sustainStart = t;
      if ((t - state->sustainStart) >= kSustainHold) {
        result.status = attemptReplan(t, state);
      }
    } else {
      state->sustainStart = -1.0f;
    }
    return result;
  }

  // ---- Exhausted: flying handoff (nonzero exitSpeed) ----
  if (exitSpeed != 0.0f) {
    const bool crossOk = fabsf(tracked.eCross) <= kHandoffCrossTol;
    const bool thetaOk = fabsf(tracked.eTheta) <= kHandoffThetaTol;
    const float alongBudget = kHandoffAlongRate * exitSpeed + kHandoffAlongBase;
    const bool alongOk = fabsf(tracked.eAlong) <= alongBudget;

    if (crossOk && thetaOk && alongOk) {
      result.status = Status::DONE_HANDOFF;
      // Seeding contract (documented here, per this ticket's own AC --
      // the ACTUAL replan()/plan() call is the caller/adapter's job,
      // ticket 100-007): the next segment's PlanRequest must set
      // entrySpeed = exitSpeed (THIS plan's own Goal::exitSpeed -- the
      // REFERENCE's boundary velocity, never `tracked`/`in.measured`'s
      // speed) and entryAccel = 0.0f. Seeding from the reference keeps
      // the chain C1-continuous by construction; seeding from the
      // measured state would inject this tick's ~130-220ms tracking lag
      // into the next plan as phantom deceleration (architecture-
      // update.md's own handoff rationale).
    } else {
      // Envelope violated -- the SAME pure replan mechanism, bypassing
      // sustain (an exhausted plan has no more time left to wait out a
      // 200ms sustain window); rate-limit + N-max still apply.
      result.status = attemptReplan(t, state);
    }
    return result;
  }

  // ---- Exhausted, exitSpeed == 0: SETTLING (the terminal machine) ----
  state->settling = true;
  result.status = Status::SETTLING;

  if (isPivot) {
    // Pivot terminal handling -- see policy.h's class comment for why this
    // does NOT reuse the linear along-walk-in law. `command` stays
    // `tracked.command` (the default set above): the pivot's own proven,
    // unclamped tracker cascade keeps running unmodified through SETTLING.
    const float thetaErr = fabsf(tracked.eTheta);
    const float omegaHat = fabsf(in.measured.twist.omega);
    const bool holdOk = (thetaErr <= kPivotArriveTolTheta) && (omegaHat <= kPivotArriveTolOmega);

    if (!(holdingDwellAtStart && poseStepPresent)) {
      if (holdOk) {
        if (state->dwellStart < 0.0f) state->dwellStart = t;
      } else {
        state->dwellStart = -1.0f;
      }
    }

    if (state->dwellStart >= 0.0f && (t - state->dwellStart) >= kDwellHold) {
      result.status = Status::DONE_STOP;
      result.command = WheelVelocities{};
      return result;
    }

    if (t >= duration + kTimeoutGrace) {
      result.command = WheelVelocities{};
      result.status = (thetaErr <= kTimeoutToleranceFactor * kPivotArriveTolTheta)
                           ? Status::DONE_STOP
                           : Status::ABORT_TIMEOUT;
      return result;
    }

    return result;
  }

  // Non-pivot stop segment: banded one-sided along walk-in. issueEAlong is
  // the RECONCILED (reference - measured) sign -- policy.h's own CRITICAL
  // sign-convention note: positive = short of goal (drive forward),
  // negative = overshot (never correct backward). insideBand/overshotBand
  // are computed once and reused below by the completion-hold check --
  // the third (implicit) band, "outside, short of goal", is exactly
  // issueEAlong > kArriveTolPos, mutually exclusive with both by
  // construction.
  const float issueEAlong = -tracked.eAlong;
  const bool insideBand = fabsf(issueEAlong) <= kArriveTolPos;
  const bool overshotBand = issueEAlong < -kArriveTolPos;

  WheelVelocities walkInCommand{0.0f, 0.0f};  // inside AND overshot both command a literal 0.0f
  if (!insideBand && !overshotBand) {
    // Outside, short of goal: clamp(k_s*e_along, floor, ceiling). e_along >
    // 0 here so k_s*e_along > 0 already; the extra `< 0.0f` guard below is
    // structural belt-and-suspenders (the wedge-hazard "never negative"
    // rule), not a case this arithmetic can actually reach.
    float speed = limits.trackKS * issueEAlong;
    if (speed < kWalkInFloor) speed = kWalkInFloor;
    if (speed > kWalkInCeiling) speed = kWalkInCeiling;
    if (speed < 0.0f) speed = 0.0f;
    walkInCommand = WheelVelocities{speed, speed};
  }
  result.command = walkInCommand;

  const float vHat = fabsf(in.measured.twist.v_x);
  const bool velocityOk = vHat <= kArriveTolVel;
  // Completion hold-eligibility: "inside" satisfies the issue's own
  // |e_along| <= tol position check directly; "overshot" cannot reduce
  // position error further without reversing (forbidden), so it is
  // hold-eligible on velocity alone once the plant has actually settled --
  // never immediately, regardless of current speed (policy.h's own
  // reasoning for why this is NOT an instant, unconditional completion).
  const bool holdEligible = (insideBand || overshotBand) && velocityOk;

  if (!(holdingDwellAtStart && poseStepPresent)) {
    if (holdEligible) {
      if (state->dwellStart < 0.0f) state->dwellStart = t;
    } else {
      state->dwellStart = -1.0f;
    }
  }

  if (state->dwellStart >= 0.0f && (t - state->dwellStart) >= kDwellHold) {
    result.status = Status::DONE_STOP;
    result.command = WheelVelocities{0.0f, 0.0f};  // literal snap
    return result;
  }

  // Pose-fix absorption/bypass -- never during the terminal dwell
  // (holdingDwellAtStart, evaluated at the START of this tick).
  if (!holdingDwellAtStart && poseStepPresent) {
    if (poseStepLarge) {
      result.status = attemptReplan(t, state);
      return result;
    }
    state->sustainStart = -1.0f;
  }

  if (t >= duration + kTimeoutGrace) {
    const bool closeEnough = fabsf(issueEAlong) <= kTimeoutToleranceFactor * kArriveTolPos;
    result.status = closeEnough ? Status::DONE_STOP : Status::ABORT_TIMEOUT;
    result.command = WheelVelocities{0.0f, 0.0f};
    return result;
  }

  return result;  // still SETTLING
}

}  // namespace Drive
