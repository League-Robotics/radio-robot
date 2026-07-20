// pilot.cpp -- App::Pilot implementation. See pilot.h's file header for
// the module's boundary and cycle-placement contract.
#include "app/pilot.h"

#include <cmath>

#include "kinematics/body_kinematics.h"

namespace App {

namespace {
// clampf -- this codebase's own per-file convention (nezha_motor.cpp,
// velocity_pid.cpp, otos.cpp, jerk_trajectory.cpp all declare an
// identically-shaped local copy rather than sharing one) -- see this
// ticket's own completion notes for why a shared utility was not
// introduced instead.
float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
}  // namespace

void Pilot::tick(uint32_t now, uint64_t nowUs) {  // [ms] [us]
  uint32_t dt = hasLastTick_ ? (now - lastTick_) : 0;
  lastTick_ = now;
  hasLastTick_ = true;
  float dtS = static_cast<float>(dt) / 1000.0f;  // [s]

  // HeadingSource is sampled every cycle, IDLE included -- see pilot.h's
  // own header comment. nowUs (109-010) lets HeadingSource's own
  // measurement-age tracker compute the REAL elapsed time since Devices::
  // Otos's own cached pose was actually sampled, without either class
  // needing a Devices::Clock dependency of its own.
  headingSource_.sample(nowUs);

  // 111-003: captured BEFORE executor_.tick() so the twist-staging decision
  // below can tell "already idle before AND after this tick() call" (a
  // same-cycle flush -- see this method's own doc comment in pilot.h) apart
  // from "just transitioned running->idle INSIDE this tick() call" (a
  // natural completion, which must be zeroed exactly once).
  Motion::State stateBefore = executor_.state();

  Motion::Executor::Twist twist = executor_.tick(dt, odom_.lastDistance(), headingSource_.heading(),
                                                  headingSource_.headingLead());

  // 112-006 model-reference: advance a first-order model of the plant's own
  // tracking lag (modelTau_) toward this tick's raw reference. The feedback
  // below (linear trim + heading PD) is measured against THIS lagged model,
  // not the raw reference, so it does not fight the plant's natural,
  // self-correcting transient lag -- only real disturbances. In a clean run
  // the measurement tracks the model, the error is ~0, and the pure planned
  // profile reaches the wheels (which alone lands on target -- verified with
  // all feedback off). See pilot.h's own model-reference doc comment for the
  // root-cause finding this replaces the 112-005 no-reversal clamp with.
  // modelAlpha is the EXACT first-order-hold coefficient 1 - e^(-dt/tau), not
  // the Euler dt/tau (which overshoots for dt/tau ~ 0.5 and makes the model
  // lead the plant -> a start-of-ramp kick).
  float alphaLin = (modelTauLin_ > 0.0f && dtS > 0.0f) ? (1.0f - expf(-dtS / modelTauLin_)) : 1.0f;
  float alphaAng = (modelTauAng_ > 0.0f && dtS > 0.0f) ? (1.0f - expf(-dtS / modelTauAng_)) : 1.0f;
  sRefStage_ += alphaLin * (twist.sRef - sRefStage_);
  sRefModel_ += alphaLin * (sRefStage_ - sRefModel_);
  thetaRefStage_ += alphaAng * (twist.thetaRef - thetaRefStage_);
  float newThetaRefModel = thetaRefModel_ + alphaAng * (thetaRefStage_ - thetaRefModel_);
  float omegaRefModel = (dtS > 0.0f) ? (newThetaRefModel - thetaRefModel_) / dtS : 0.0f;  // [rad/s]
  thetaRefModel_ = newThetaRefModel;

  // 112-002: the PLANNED per-wheel reference -- BodyKinematics::inverse()
  // applied to twist.v/twist.omega EXACTLY as Executor emitted them, before
  // the heading-PD correction below (which only ever modifies the LOCAL
  // `omega` copy, never `twist.omega` itself) and before App::Drive's own
  // actuation-lag feedforward (Drive::tick(), a later, separate stage). See
  // refLeft()/refRight()'s own doc comment (pilot.h) for why this is a
  // live accessor rather than a wire telemetry field.
  BodyKinematics::inverse(twist.v, twist.omega, drive_.trackWidth(), refLeft_, refRight_);

  float omega = twist.omega;
  if (twist.headingActive) {
    // 109-010 locus 1: the PD's own error term uses thetaMeasLead (the
    // measurement-age-projected heading), not the raw thetaMeas -- see
    // Motion::Executor::Twist::thetaMeasLead's own doc comment. The rate
    // estimate below (omegaMeasEst) deliberately stays on the RAW,
    // continuous thetaMeas sequence -- thetaMeasLead's own age-tracked
    // offset resets to 0 on every fresh OTOS sample (App::HeadingSource's
    // own ageMs_ bookkeeping), which would inject a sawtooth into a
    // finite-difference derivative computed across it.
    // 112-006: error is against the plant-lag model of the reference
    // (thetaRefModel_/omegaRefModel), not the raw thetaRef/omegaDes -- so the
    // PD stays ~0 through the natural accel/decel lag and only reacts to real
    // heading disturbances (thetaMeasLead is still the age-compensated
    // measurement; the two adjustments cancel in a clean run).
    float thetaErr = thetaRefModel_ - twist.thetaMeasLead;
    float omegaMeasEst =
        (hasPrevThetaMeas_ && dtS > 0.0f) ? (twist.thetaMeas - prevThetaMeas_) / dtS : 0.0f;
    omega += headingKp_ * thetaErr + headingKd_ * (omegaRefModel - omegaMeasEst);

    // 112-004: the minimum-command floor (2026-07-18, terminal stiction/
    // deadband) that used to live here is DELETED -- it existed because a
    // small residual error times heading_kp could command a per-wheel
    // speed BELOW what actually moves the plant (the write shaping's
    // output deadband clamps sub-0.03 duty to zero), stalling the PD with
    // the error frozen above tolerance (observed: kp=1 froze 5.7deg out,
    // kp=6 froze ~1deg out). Deleting it is safe now that heading_kp is
    // bumped 3.0 -> 6.0 (gen_boot_config.py's own HEADING_KP_DEFAULT) so
    // the deadband inequality (`heading_kp * heading_dwell_tol >=
    // omega_deadband`) holds without a floor -- see this file's own
    // completion notes for the re-derivation against the actual current
    // deadband/track-width/tolerance values (sprint 112 Architecture
    // Design Rationale Decision 5).
  }
  prevThetaMeas_ = twist.thetaMeas;
  hasPrevThetaMeas_ = true;

  // 112-003: bounded linear position-feedback trim -- mirrors the heading
  // PD's own gain/arithmetic split exactly (pilot.h's own kDistanceTrimCeiling
  // doc comment / sprint 112 Architecture Design Rationale Decision 3):
  // Executor exposes the linear channel's own since-activation reference/
  // measured pair (Twist::sRef/sMeas), Pilot owns the gain and the
  // correction arithmetic. `sRef`/`sMeas` are both 0 for kPivot/kTimed
  // (Twist::sRef/sMeas's own doc comment, executor.h) -- `trim` is
  // therefore a harmless 0 no-op in either case, with no mode branching
  // needed here, the same way the deadband guard (`twist.sRef == twist.
  // sMeas == 0`) never needs an explicit `if`. Downstream of Motion::
  // Executor's own PLANNED reference (`twist.v`, already captured into
  // `refLeft_`/`refRight_` above via BodyKinematics::inverse()) -- this
  // trim perturbs only the SAMPLED velocity Drive::setTwist() receives; it
  // never feeds back into a JerkTrajectory solve (no solveToRest/
  // solveToState/solveToVelocity/retarget/reanchor call reads it), so the
  // ramp/lobe/bounds checks that grade the planned reference are
  // unaffected (this ticket claims no new harness xfail flip).
  //
  // 112-004, two changes, both empirically driven (see this ticket's own
  // completion notes for the full sweep/evidence):
  //   1. Gated off once within distance_tol (`twist.withinDistanceTolerance`),
  //      mirroring the heading PD's own terminal-decel gate (`twist.
  //      headingActive`) exactly -- see Twist::withinDistanceTolerance's own
  //      doc comment (executor.h) for why: once the planned trajectory has
  //      settled at rest, the trim's own error term no longer decays (a
  //      stationary plant does not asymptotically approach zero error the
  //      way a still-moving one does), so an UNGATED P-only trim bang-bangs
  //      a small residual back and forth around target forever instead of
  //      converging -- a failure mode the deleted terminal patch stack's own
  //      crossing-based distanceDone test never exposed (it needed the trim
  //      to cross the target once, never to SETTLE there), but 112-004's own
  //      unified completion rule (which requires a sustained, held tolerance
  //      window) does.
  //   2. `distance_kp`'s own shipped default drops 15.0 -> 8.0
  //      (gen_boot_config.py's own DISTANCE_KP_DEFAULT) -- gating alone was
  //      NOT sufficient: at kp=15.0 the trim's own reaction to ordinary
  //      cruise-phase tracking lag is aggressive enough (repeatedly
  //      saturating the +-kDistanceTrimCeiling clamp) to still ring for
  //      several seconds after the gate first engages, particularly right
  //      after a same-sign reversal-dwell-delayed start (e.g. a straight
  //      leg immediately following a pivot, both wheels needing to reverse
  //      direction into NezhaMotor's own 100ms reversal-dwell window) --
  //      confirmed by direct sweep against this sprint's own same-boot
  //      harness scenario: kp in [1, 8] converges cleanly and
  //      deterministically (100% across repeated runs), kp=10 fails
  //      intermittently (1/40), kp>=12 fails increasingly often (4/40,
  //      10/40) as gain rises toward the old 15.0 default. This narrows the
  //      deadband-clearing margin (see pilot.h's own kDistanceTrimCeiling
  //      doc comment and gen_boot_config.py's own DISTANCE_KP_DEFAULT
  //      comment for the honest, un-cleared-against-the-tuned-config
  //      accounting -- the SAME shape as heading_kp's own Decision 5
  //      shortfall, gen_boot_config.py's own HEADING_KP_DEFAULT comment).
  float trim = twist.withinDistanceTolerance
                   ? 0.0f
                   : clampf(distanceKp_ * (sRefModel_ - twist.sMeas), -kDistanceTrimCeiling,
                            kDistanceTrimCeiling);
  float v = twist.v + trim;

  // 112-006: the 112-005 no-reversal clamp is REMOVED. It masked the terminal
  // overshoot by force-clamping any command that crossed zero -- but the
  // overshoot was the outer feedback fighting the plant's natural tracking lag
  // (see the model-reference doc comment above and in pilot.h). With the
  // feedback measured against the plant-lag model, a clean run produces no
  // reversal to clamp in the first place, and clamping would now only BLOCK a
  // legitimate disturbance correction. The command is `v`/`omega` as computed.

  // 111-003 twist-staging decision (pilot.h's own tick() doc comment):
  //   - still running (or just started) -- stage the freshly-computed
  //     twist, unchanged existing behavior.
  //   - a natural running->idle transition happened INSIDE this tick()
  //     call (stateBefore was non-idle, executor_.state() is now kIdle) --
  //     stage a zero twist exactly once, so Drive stops commanding the
  //     PREVIOUS cycle's stale twist instead of creeping until the 300ms
  //     deadman lease force-stops it (robot_loop.cpp's kPilotDeadmanLease).
  //   - already idle BEFORE this tick() call (includes a same-cycle flush:
  //     RobotLoop::handleTwist()/handleStop() call Pilot::flush() BEFORE
  //     Pilot::tick() runs this same cycle, so stateBefore is already
  //     kIdle by the time it's sampled above) -- do nothing, matching
  //     today's "does nothing while kIdle" contract; a raw TWIST/STOP's
  //     own Drive::setTwist() call (already staged earlier this cycle by
  //     handleTwist()/handleStop()) must survive untouched.
  if (executor_.state() != Motion::State::kIdle) {
    // 112-002: aRef/alphaRef forward the SAME sample() result already
    // computed for v/omega above (never a separate solve) -- Drive::tick()
    // folds them into a model feedforward term (actuation_lag * a) on top
    // of the velocity target. 112-003: `v` (not `twist.v`) carries the
    // bounded linear trim computed above.
    drive_.setTwist(v, omega, twist.aRef, twist.alphaRef);
  } else if (stateBefore != Motion::State::kIdle) {
    drive_.setTwist(0.0f, 0.0f);
  }

  // 112-006: once idle, reset the reference model so the next move starts it
  // matched at 0 (sRef/thetaRef are both command-relative -- executor.h's own
  // Twist doc comment), and no stale lag carries across moves.
  if (executor_.state() == Motion::State::kIdle) {
    sRefStage_ = 0.0f;
    sRefModel_ = 0.0f;
    thetaRefStage_ = 0.0f;
    thetaRefModel_ = 0.0f;
  }
}

}  // namespace App
