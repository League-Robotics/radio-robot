// drive.cpp -- App::Drive implementation. See drive.h's file header for the
// controller's three responsibilities and the tick/update contract.
#include <algorithm>
#include <cmath>

#include "app/drive.h"

namespace App {

Drive::Drive(Devices::Motor& left, Devices::Motor& right, float trackWidth)
    : left_(left), right_(right), trackWidth_(trackWidth) {}

// configure -- see drive.h's own doc comment. A thin pull-and-forward:
// every value read here has an existing setter, no new control logic.
void Drive::configure(const Config::Robot& config) {
  setWheelCorrection(
      config.drive.wheel_gain_left_accel, config.drive.wheel_intercept_left_accel,
      config.drive.wheel_gain_left_decel, config.drive.wheel_intercept_left_decel,
      config.drive.wheel_gain_right_accel, config.drive.wheel_intercept_right_accel,
      config.drive.wheel_gain_right_decel, config.drive.wheel_intercept_right_decel);
  setCrawlPulse(config.drive.crawl_pulse);

  ControlGains gains;
  gains.kp = config.wheelControl.pid_kp;
  gains.ki = config.wheelControl.pid_ki;
  gains.iMax = config.wheelControl.pid_i_max;
  gains.kaff = config.wheelControl.pid_kaff;
  gains.pidMax = config.wheelControl.pid_max;
  setControlGains(gains);

  AdaptationBounds bounds;
  bounds.vMin = config.wheelControl.v_min;
  bounds.biasMax = config.wheelControl.bias_max;
  bounds.tauAdapt = config.wheelControl.tau_adapt;
  bounds.aSteady = config.wheelControl.a_steady;
  bounds.posErrMax = config.wheelControl.pos_err_max;
  bounds.deficitThreshold = config.wheelControl.deficit_threshold;
  bounds.deficitWindow = config.wheelControl.deficit_window;
  setAdaptationBounds(bounds);
}

void Drive::command(float vLeft, float vRight, float duration,
                    uint32_t moveId, uint32_t now) {
  targetLeft_ = vLeft;
  targetRight_ = vRight;
  commandDeadline_ = now + static_cast<uint32_t>(duration);
  commandMoveId_ = moveId;
  commandActive_ = true;
}

void Drive::takeover() {
  targetLeft_ = 0.0f;
  targetRight_ = 0.0f;
  commandActive_ = false;
  // completionPending_ is deliberately NOT set: a discarded command does
  // not complete (drive.h's own estop() doc comment, which this shares). An
  // ALREADY-latched completion from a command that expired normally this
  // cycle is left alone -- it describes something that really happened.

  // Deliberately NOT touched (131-001, see drive.h's own header): Stage B's
  // integrators, Stage C's bias, the deficit latch, and
  // stopEnforceCountdown_. This is an ownership handover, not a safety
  // event -- the plant did not change, only the writer did, so there is
  // nothing here to reset and no "verify the wheels actually reached rest"
  // window to arm.
}

void Drive::estop() {
  // Shared with takeover() (131-001, see drive.h's own header): zero
  // targets, disarm the WHEELS command.
  takeover();

  // Arm the stop re-assertion window (129-001, see drive.h's own header):
  // the next kStopEnforceTicks tick() calls re-issue the zero duty write
  // instead of taking the quiet-at-zero shortcut below. NOT part of
  // takeover()'s shared half -- this window's whole purpose is verifying a
  // commanded STOP is actually observed at rest, which only applies to a
  // genuine safety stop, not an ownership handover.
  stopEnforceCountdown_ = kStopEnforceTicks;

  // Stage B/C reset (130-004, see drive.h's own header) -- part of the
  // trim's own safety design (issue 04's folded-in safety intent), not
  // merely PID anti-windup hygiene: a fresh boot-calibration bias is
  // always a safe starting point, and an emergency stop is exactly the
  // moment to stop trusting whatever the slow adaptation had learned
  // about the PRIOR load condition. The deficit-flag latch resets too --
  // a stop is a fresh start, not a continuation of whatever deficit was
  // accumulating before it. NOT shared with takeover() (131-001) -- an
  // ownership handover must KEEP exactly this state, which is the whole
  // point of the split.
  //
  // 133-002: Stage B's reset is now a re-anchor of the POSITION REFERENCE
  // rather than a zeroing of an accumulator -- `PositionRef{}` clears
  // `armed`, so the next tick() re-reads the encoder's current position
  // as the new origin and reports exactly zero error for that cycle. A
  // stop must not leave the loop still owing distance from before it.
  posRefLeft_ = PositionRef{};
  posRefRight_ = PositionRef{};
  biasLeft_ = 0.0f;
  biasRight_ = 0.0f;
  deficitSinceLeft_ = 0;
  deficitSinceRight_ = 0;
  deficitLeft_ = false;
  deficitRight_ = false;
}

bool Drive::takeCompletion(uint32_t* moveId) {
  if (!completionPending_) return false;
  completionPending_ = false;
  *moveId = completedMoveId_;
  return true;
}

void Drive::update(Types::RobotState& state, uint32_t now) {
  // Ownership is sampled BEFORE the expiry test so the cycle a command ends
  // on still publishes -- that publish is the zero pair, which is exactly
  // the value the wheels must be handed next cycle.
  const bool owned = commandActive_;

  if (commandActive_ && static_cast<int32_t>(now - commandDeadline_) >= 0) {
    commandActive_ = false;
    targetLeft_ = 0.0f;
    targetRight_ = 0.0f;
    completionPending_ = true;
    completedMoveId_ = commandMoveId_;
  }

  if (!owned) return;  // the planner owns motion; its update() is the writer

  state.wheelLeft.cmdVelocity = targetLeft_;
  state.wheelRight.cmdVelocity = targetRight_;

  // cmdAccel on the WHEELS path -- 133-002. LANDS UNVALIDATED, ON PURPOSE:
  // it is correct on its own terms (the field genuinely was never set here,
  // which is WHY the earlier kff and aSteady sweeps each measured nothing
  // -- `kaff * cmdAccel` was identically zero and `steady` was pinned
  // true), but the tuning sweep run on top of it returned ~50% dead runs
  // and it has never been shown to IMPROVE anything. It is INERT at
  // kaff == 0, which every shipped robot JSON currently carries, and that
  // is the only reason it is safe to land ahead of its own measurement.
  // Sprint 133 Open Question 2 -- do not treat this as closed.
  //
  // Only Motion::Planner::update() wrote cmdAccel before this, so while
  // Drive owned motion the field held a stale planner value or zero, and
  // two whole mechanisms in tick() silently did nothing:
  //   * Stage B's feedforward, kaff * cmdAccel, was identically ZERO, so a
  //     ramp was tracked by P+I alone -- and a P+I loop tracking a ramp
  //     lags by construction. Measured on the 200 mm/s trapezoid: lagging
  //     on the up-ramp, near-perfect on the plateau, over-travelling on the
  //     down-ramp, which is exactly that signature.
  //   * `steady` (|cmdAccel| < aSteady) was ALWAYS true, so Stage C's bias
  //     was never gated off during a ramp either.
  //
  // Smoothed, not a bare finite difference: the host re-arms WHEELS on its
  // own tick (100 ms measured) while the loop runs at kCycle (50 ms), so
  // the commanded velocity arrives as a STAIRCASE and a raw difference
  // quotient alternates a double-size spike with a zero. A first-order
  // filter recovers the underlying ramp slope instead of handing Stage B a
  // square wave to feed forward.
  const float dt = static_cast<float>(state.time.cyclePeriod) * 1e-6f;  // [s]
  if (dt > 0.0f) {
    const float rawLeft = (targetLeft_ - previousTargetLeft_) / dt;    // [mm/s^2]
    const float rawRight = (targetRight_ - previousTargetRight_) / dt;  // [mm/s^2]
    cmdAccelLeft_ += kAccelSmoothing * (rawLeft - cmdAccelLeft_);
    cmdAccelRight_ += kAccelSmoothing * (rawRight - cmdAccelRight_);
  }
  previousTargetLeft_ = targetLeft_;
  previousTargetRight_ = targetRight_;
  state.wheelLeft.cmdAccel = cmdAccelLeft_;
  state.wheelRight.cmdAccel = cmdAccelRight_;

  state.command.moveActive = commandActive_;
  state.command.mode = commandActive_ ? Types::Mode::Velocity : Types::Mode::Idle;
  state.command.v_x = 0.5f * (targetLeft_ + targetRight_);
  state.command.omega = (targetRight_ - targetLeft_) / trackWidth_;
}

// Crawl shaper: a request below the pulse amplitude becomes a train of
// fixed-amplitude pulses, whole-cycle Bresenham-dithered so the AVERAGE
// duty matches the request -- the wheel's ~230 ms inertia low-passes the
// pulsing. At/above the amplitude the request passes through untouched.
// crawlPulse_ == 0 disables (sim plants have no stiction; the amplitude
// is a per-robot property that must clear the measured breakaway).
void Drive::setWheelCorrection(float gLA, float iLA, float gLD, float iLD,
                               float gRA, float iRA, float gRD, float iRD) {
  corrGain_[0][0] = gLA;  corrIntercept_[0][0] = iLA;
  corrGain_[0][1] = gLD;  corrIntercept_[0][1] = iLD;
  corrGain_[1][0] = gRA;  corrIntercept_[1][0] = iRA;
  corrGain_[1][1] = gRD;  corrIntercept_[1][1] = iRD;
}

// The characterization measured `actual = gain*commanded + intercept` per
// wheel per direction of approach; inverting it gives the command whose
// ACTUAL result is the speed asked for -- the feedforward's first guess.
// The fit was taken on positive speeds, so a reverse command is corrected
// on its magnitude and the sign restored.
//
// `bias` (Stage C's adapted parameter, 130-004) is added to the inverted
// map here, INSIDE this function, rather than at tick()'s call site --
// the `desired == 0.0f` guard immediately below must cover the bias term
// too. A commanded-zero wheel must produce exactly zero corrected
// command regardless of whatever bias the slow adaptation has learned,
// or a nonzero bias would creep a stopped wheel forward/back -- the
// exact defect this same guard already prevents for the map's own
// intercept.
//
// 131-001 (sprint 131 Design Rationale Decision 2, review C3): `bias` is
// applied in the MAGNITUDE domain, following `desired`'s CURRENT sign --
// NOT as a body-frame-fixed additive term added after copysign(). Once
// takeover() (131-001) stops resetting bias on every MOVE, a bias learned
// under sustained motion in ONE direction becomes long-lived enough to be
// exercised across a direction reversal for the first time; a fixed
// additive term would then REDUCE a reverse command's magnitude whenever
// the converged bias is positive (learned while under-delivering forward),
// exactly backwards from what Stage C is for. Folding bias into the
// magnitude before restoring the sign means a bias that boosts one
// direction boosts the other too -- the physical droop it corrects is a
// property of the wheel's instantaneous load, not which way it is
// currently being asked to turn (130-004's own one-bias-per-wheel
// rationale, unchanged). Never let the correction invert the commanded
// direction (same "don't flip sign" posture as applySpeedFloor()'s own
// doc comment below): a bias whose magnitude exceeds the commanded
// magnitude means the correction hasn't converged usefully yet, not that
// the wheel should reverse.
float Drive::correctedCommand(float desired, float previous, bool leftWheel,
                              float bias) const {
  if (desired == 0.0f) return 0.0f;  // stop is stop; never offset it (map OR bias)
  const int w = leftWheel ? 0 : 1;
  // "Accelerating" is about |speed| rising, not about which way the wheel
  // turns: a pivot runs one wheel negative and both must classify sanely.
  const int d = (std::fabs(desired) > std::fabs(previous)) ? 0 : 1;
  const float magnitude =
      (std::fabs(desired) - corrIntercept_[w][d]) / corrGain_[w][d];
  if (magnitude <= 0.0f) return 0.0f;  // below the intercept: unreachable
  const float correctedMagnitude = magnitude + bias;
  if (correctedMagnitude <= 0.0f) return 0.0f;  // never flip direction; not converged yet
  return std::copysign(correctedMagnitude, desired);
}

float Drive::crawlDuty(float duty, float& carry) const {
  const float magnitude = std::fabs(duty);
  if (crawlPulse_ == 0.0f || magnitude >= crawlPulse_) return duty;
  if (magnitude == 0.0f) {
    carry = 0.0f;
    return 0.0f;
  }
  carry += magnitude / crawlPulse_;
  if (carry < 1.0f) return 0.0f;
  carry -= 1.0f;
  return std::copysign(crawlPulse_, duty);
}

// Wheels-solid speed floor (Open Question 2, resolved 130-004; REVISED
// 131-003 post-shipment -- see drive.h's own file header and sprint 131
// sprint.md's "Revision -- Ticket 003's speed-floor semantics" for the
// measurement that invalidated the originally-shipped common-mode-only
// design and the full rationale for this replacement). A nonzero wheel
// pair whose dominant (larger-magnitude) wheel sits below vMin is scaled
// UP so the dominant wheel reaches exactly vMin, rather than silently
// stalling. This mirrors the project's own established fix for the
// analogous duty-domain problem (sprint 114's deadband dead-zone/boost
// fix: a sub-deadband duty command used to be zeroed outright, which
// stalled the wheel at the terminal approach of a move -- fixed by
// boosting to copysign(deadband, cmd) instead). Refusing the command
// outright here would reproduce that exact bug class one layer up, in the
// velocity domain, with a silently stalled wheel and no diagnostic signal.
// `vMin <= 0` (uncalibrated / no population floor configured yet) makes
// this a no-op, so a robot JSON that hasn't run ticket 001's sweep behaves
// exactly as before.
//
// The scale is RATIO-PRESERVING, computed from the raw wheel pair
// directly -- no common-mode/differential decomposition (the superseded
// 131-003 design): `dominantMag = max(|rawLeft|, |rawRight|)`; if nonzero
// and below vMin, both wheels are multiplied by `vMin / dominantMag`,
// which lands the dominant wheel at exactly vMin while preserving the
// EXACT commanded ratio between the two wheels -- a symmetric pivot
// (equal-and-opposite wheels, e.g. every Angle Move) is boosted
// symmetrically to exactly (-vMin, +vMin), bit-identical to the
// pre-131-003 per-wheel-independent floor for that case; an asymmetric arc
// keeps its commanded curvature instead of being flattened toward 1:1 (the
// old per-wheel-independent floor's own latent defect) or left
// unboundedly distorted (the superseded common-mode-only floor's own
// latent defect). This is the same dominant-wheel/ratio-locked idiom
// `Planner::planWheels()` already uses internally for its own
// accel-ceiling tie-break (planner.cpp, the `dominant`/`other` tie-break
// near its ratio-lock computation) -- reused, not new. A wheel
// raw-commanded exactly 0.0f is exactly 0.0f afterward regardless of the
// scale factor applied (`0 * scale == 0` in IEEE-754 for any finite
// scale), so "stop is stop" holds for a genuine full stop without a
// special case, and a raw-zero wheel produced by vector cancellation
// (part of an active, asymmetric command) also stays exactly zero rather
// than being driven nonzero.
//
// 134-002 -- THE FLOOR IS A TELEOP AFFORDANCE, AND ONLY THAT. It engages
// only while Drive itself owns motion (`owns()`, i.e. a live WHEELS
// command); a planner-owned wheel pair reaches actuation exactly as
// Motion::Planner shaped it. Boosting a sub-vMin pair is right for a
// STANDING command below breakaway -- that is this whole function's
// purpose, unchanged -- and wrong for a command that is small ON PURPOSE.
// A profiled decelerating tail and a terminal alignment nudge are both
// shaped small deliberately, and raising them to vMin hands the plant an
// authority nobody asked for while destroying the terminal precision they
// exist to deliver. Both of 2026-08-04's independent bench sessions hit
// this (docs/bench-reports/motion-planning-lab-2026-08-04.md Section 5.3);
// the turn-tuning session had to push `DBG vmin 0` to measure a turn at
// all, and tovez.json's own `_v_min_note` prices the mechanism: at the
// then-baked floor of 99.7 mm/s the sub-floor portions of a trapezoid's two
// ramps integrated to ~+37 mm on a 450 mm move (+8.3%), and that note names
// THIS gate as the right fix rather than re-lowering the floor further.
// READ THAT NUMBER PRECISELY: it was measured through
// velocity_profile_gate.py, which host-shapes its profile and pushes it as
// WHEELS commands -- the teleop path, which this gate deliberately still
// floors. It prices what a floor does to a decelerating ramp, which is the
// same mechanism; it is NOT a run this change would have altered. What this
// change fixes is the FIRMWARE-shaped ramp: Motion::Planner's own profile
// tails and (134-003) its terminal alignment nudges. A host that shapes its
// own ramp and pushes it as teleop is still floored, by construction and on
// purpose -- Drive cannot tell that stream apart from a standing command,
// and the host-side taper that owns that case is issue
// `B-one-owner-per-constant-speed-floor-and-duty-per-speed.md`, out of
// scope here. This resolves the deferral recorded as sprint 131 Design
// Rationale Decision 4.
// Gating on Drive's OWN ownership flag rather than on a planner query is
// sprint 134 Design Rationale D2: App::RobotLoop::handleMove() already
// calls takeover() before every planner Move, so the flag is exactly the
// predicate wanted -- no new Drive->Planner dependency, and no new member.
// Any future non-teleop owner opts out of the floor automatically, which
// is the right default.
//
// DEADMAN EXPIRY -- decided explicitly rather than left to be rediscovered:
// an EXPIRING command is NOT floored. update() clears commandActive_ the
// moment a WHEELS command outlives its deadline, and tick() runs BEFORE
// update() within a cycle (robot_loop.cpp), so the floor covers every cycle
// the command was live and no cycle after it. A command on its way out is
// allowed to reach zero rather than being boosted back up to vMin. This is
// belt-and-braces in practice -- update() zeroes both targets at expiry, so
// the `dominantMag <= 0.0f` guard below would pass that pair through
// untouched either way -- but the ordering is stated, not assumed.
void Drive::applySpeedFloor(float rawLeft, float rawRight, float& speedLeft,
                            float& speedRight) const {
  speedLeft = rawLeft;
  speedRight = rawRight;
  if (!owns()) return;               // 134-002: teleop affordance only, see above
  if (bounds_.vMin <= 0.0f) return;  // uncalibrated: no-op, same as before
  const float dominantMag = std::max(std::fabs(rawLeft), std::fabs(rawRight));
  if (dominantMag <= 0.0f || dominantMag >= bounds_.vMin) return;
  const float scale = bounds_.vMin / dominantMag;
  speedLeft = rawLeft * scale;
  speedRight = rawRight * scale;
}

// Stage B -- the fast controller (small authority, never carries standing
// error because Stage A+C already do -- see drive.h's own header).
//
// THE I TERM IS A POSITION TERM (133-002, stakeholder observation
// 2026-08-03). This used to be `integral += ki * err * dt`, and `err` is a
// VELOCITY, so `err * dt` is MILLIMETRES: the accumulator was already
// (commanded position - measured position), just computed the worst
// possible way -- by summing a derived, quantized velocity whose own noise
// is a FULL DUTY COUNT (sd 8.0 mm/s), gated by a freshness test that FROZE
// the sum on precisely the samples where unobserved distance was being
// travelled. The encoder's position register carried the truth the whole
// time.
//
// So read the error instead of inferring it. `posError` comes straight from
// that register (positionError(), below); the control law, its units, and
// its output are otherwise unchanged -- ki is still [1/s], the term is
// still millimetres of error turned into mm/s of correction, and the loop
// still commands velocity, which is all the motor can take.
//
// Note what this DELETES: there is no accumulator, so there is no windup,
// no `steady` gate to freeze it, no anti-windup pushingIntoClamp test, and
// no reset to lose. A position error that appears while the loop is not
// looking is simply still there when it looks again -- which is the entire
// fix.
//
// Two clamps, two domains, both retained (drive.h's own header):
// bounds_.posErrMax [mm] already bounded the INPUT inside positionError();
// gains_.iMax [mm/s] bounds this term's OUTPUT. iMax == 0 still means the
// I term is OFF ENTIRELY -- the same fail-closed reading the accumulator
// era's `iMax > 0` guard had, kept deliberately so that every shipped
// robot JSON (all carrying iMax = 0 today) cannot acquire UNBOUNDED I
// authority the moment someone pushes a nonzero ki.
float Drive::fastPid(float posError, float err, float aCmd) const {
  const float proportional = gains_.kp * err;
  const float feed = gains_.kaff * aCmd;

  float integral = 0.0f;  // [mm/s]
  if (gains_.iMax > 0.0f) {
    integral = gains_.ki * posError;
    if (integral > gains_.iMax) integral = gains_.iMax;
    if (integral < -gains_.iMax) integral = -gains_.iMax;
  }

  float pid = proportional + feed + integral;
  if (gains_.pidMax > 0.0f) {
    if (pid > gains_.pidMax) pid = gains_.pidMax;
    if (pid < -gains_.pidMax) pid = -gains_.pidMax;
  }
  if (!std::isfinite(pid)) return 0.0f;  // fail closed, never inject NaN
  return pid;
}

// The position error the I term consumes (133-002): the integral of the
// COMMANDED speed -- our own signal, noise-free -- minus the wheel's own
// measured travel since the reference was anchored.
//
// Re-anchors, returning EXACTLY zero error and correcting nothing,
// whenever the reference cannot be trusted. Three conditions, each of
// which earns its place:
//
//   * COMMANDED ZERO. Stop is stop. Holding position at a commanded zero
//     would make this a servo, and a servo creeps a stopped wheel back
//     onto its reference -- the exact runaway class 133-001 and
//     nezha_motor.h's own LOAD-BEARING note exist to prevent. Losing the
//     accumulated reference across a stop is the correct trade. This
//     guard is why tick() calls this function EVERY cycle rather than
//     short-circuiting it at commanded zero -- see that call site.
//   * DISCONNECTED WHEEL. Devices::NezhaMotor::collectEncoder() returns
//     `lastGoodRawEnc_` on a failed read (nezha_motor.cpp), so a
//     disconnected wheel's position is a MANUFACTURED value, not a
//     measurement -- and one that stops advancing while the wheel may
//     still be turning. Anchoring against it would bank a fictitious
//     deficit.
//   * EPOCH CHANGE. App::RobotLoop rebaselines the position register as
//     its raw value nears the wire's range (robot_state.h's own
//     positionEpoch doc comment), which STEPS `position` discontinuously.
//     A step is not travel.
//
// `armed` makes the anchor cycle itself report zero rather than a step:
// the first cycle after any of the above establishes origin/epoch, and
// only the NEXT cycle begins accumulating a reference against it.
//
// Deliberately NOT gated on sample freshness/staleness, unlike Stage C's
// adaptBias() and unlike the accumulator this replaces (131-002's own
// gate). A stale-but-connected reading is the LAST TRUE POSITION, held --
// not a manufactured zero -- so the distance travelled while the loop was
// not looking is still recorded in the register and lands in full on the
// next good read. Freezing there is exactly the behavior that used to
// delete real distance permanently. The authority this hands a
// genuinely-wedged encoder is bounded by posErrMax, then by iMax, then by
// pidMax.
//
// The error is clamped to ±posErrMax so a blocked or slipping wheel
// cannot bank unbounded catch-up debt and then sprint to repay it the
// moment it comes free. That clamp is in MILLIMETRES, which is the whole
// point: the old iMax clamped ki*error, so raising ki silently SHRANK the
// position memory (at ki=6, iMax=20 could remember only 3.3 mm -- less
// than every residual being corrected).
float Drive::positionError(float speed, const Types::RobotState::Wheel& wheel,
                           PositionRef& ref, float dt) const {
  if (speed == 0.0f || dt <= 0.0f || !wheel.connected ||
      wheel.positionEpoch != ref.epoch || !ref.armed) {
    ref.armed = (speed != 0.0f) && wheel.connected;
    ref.epoch = wheel.positionEpoch;
    ref.origin = wheel.position;
    ref.reference = 0.0f;
    return 0.0f;
  }
  ref.reference += speed * dt;                                  // [mm]
  float error = ref.reference - (wheel.position - ref.origin);  // [mm]
  if (bounds_.posErrMax > 0.0f) {
    if (error > bounds_.posErrMax) error = bounds_.posErrMax;
    if (error < -bounds_.posErrMax) error = -bounds_.posErrMax;
  }
  return error;
}

// Stage C -- bounded adaptation of the map's ONE additive parameter,
// `bias` (Open Question 1, resolved 130-004: per-wheel, not
// per-wheel-per-direction -- the physical droop this trim corrects, per
// wheel-speed-controller-moves-into-drive.md's own Cause section, is a
// property of the wheel's instantaneous load, not which side of a ramp
// it arrived from; splitting by accel/decel would fragment ONE physical
// quantity into two separately-converging estimates fed by disjoint
// samples for no modeled benefit, and complicates the bumpless-transfer
// property for no reason -- bias applies identically regardless of which
// accel/decel branch correctedCommand() currently selects). Runs only
// when STEADY (not ramping), at/above the speed floor, and on a fresh,
// connected, non-frozen measurement -- `fresh` folds all three of those
// gates together (see tick()'s own computation) so this function need
// not re-derive them; robot_state.h's own Health::wheelFrozenLeft/Right
// doc comment anticipates exactly this consumer ("the adaptive duty/
// speed gain learner"). `tauAdapt <= 0` disables adaptation outright --
// the same "0 = off" convention this file already uses for crawlPulse_/
// ControlGains::iMax/pidMax.
void Drive::adaptBias(float& bias, float err, float aCmd, float vCmdMagnitude,
                      bool fresh, float dt) const {
  if (bounds_.tauAdapt <= 0.0f || dt <= 0.0f || !fresh) return;
  if (std::fabs(aCmd) >= bounds_.aSteady) return;  // ramping, not steady
  if (vCmdMagnitude < bounds_.vMin) return;         // below the speed floor
  bias += err * dt / bounds_.tauAdapt;
  if (bounds_.biasMax > 0.0f) {
    if (bias > bounds_.biasMax) bias = bounds_.biasMax;
    if (bias < -bounds_.biasMax) bias = -bounds_.biasMax;
  } else {
    // No adaptation authority configured -- never let bias drift
    // unbounded; hold it at exactly 0 (the same fail-closed posture as
    // every other "0 = off" bound in this file).
    bias = 0.0f;
  }
}

// Deficit-flag policy sustained-condition latch: `conditionNow` (the
// caller's own "error exceeds deficitThreshold AND both bias and the
// fast PID are pinned at their configured authority" test, computed in
// tick()) must hold continuously for deficitWindow ms before `latched`
// goes true; it drops the instant `conditionNow` clears. `since == 0` is
// the "not currently accumulating" sentinel (mirrors this file's other
// uint32_t deadline/countdown fields). `deficitThreshold <= 0` OR
// `deficitWindow <= 0` disables the policy outright -- the flag never
// latches, matching this file's "0 = off" convention.
void Drive::updateDeficit(bool conditionNow, uint32_t now, uint32_t& since,
                          bool& latched) const {
  if (bounds_.deficitThreshold <= 0.0f || bounds_.deficitWindow <= 0.0f ||
      !conditionNow) {
    since = 0;
    latched = false;
    return;
  }
  if (since == 0) since = now;
  latched = (now - since) >= static_cast<uint32_t>(bounds_.deficitWindow);
}

uint32_t Drive::sampleAge(uint32_t now, uint32_t sampleTime) const {
  return (now < sampleTime) ? 0u : (now - sampleTime);
}

// Apply the cycle's duty pair to the two leaves, crawl-shaped. Quiet at
// zero: while both the commanded and last-written pairs are exactly zero
// there is nothing to say to the hardware -- writing anyway would flip
// the motors out of Mode::None from the first idle boot cycle and make
// boot-time config pushes race the at-rest reconfigure gate. EXCEPT during
// the post-estop() stop re-assertion window (129-001, see below and
// drive.h's own header) -- neither condition applies there (mode_ is
// already Active, past boot), and the write is worth repeating.
void Drive::tick(const Types::RobotState& state) {
  // Fail closed: with no calibration installed there is no honest
  // speed->duty conversion to make, so write nothing at all rather than
  // guess. A robot whose JSON is missing the
  // calibration never gets here -- codegen fails first -- so reaching this
  // return means a composition root that skipped setDutyPerSpeed(), and
  // standing still is the right answer to that.
  if (!calibrated_) return;

  // Wheels-solid speed floor (Open Question 2), 131-003 REVISED
  // post-shipment (issue A-speed-floor-snaps-the-planner-differential.md;
  // full history in sprint 131 sprint.md's "Revision -- Ticket 003's
  // speed-floor semantics"): a RATIO-PRESERVING SCALE applied to the raw
  // wheel pair directly, computed once in applySpeedFloor() so every stage
  // below (classification, the map, the fast PID, the adaptation gate)
  // sees the SAME effective per-wheel target. The originally-shipped
  // design decomposed the pair into common-mode/differential and floored
  // only the common mode -- correct for a differential trim riding on an
  // already-profiled travel speed (Planner::applyHeadingHold()), but WRONG
  // for `Planner::planWheels()`'s ratio-locked output on a pure pivot
  // (every Angle Move), whose common mode is EXACTLY zero by construction:
  // the floor never engaged, and the turn's own sub-breakaway ramp-in/
  // ramp-out passed through undelivered, measurably undershooting the
  // turn. The ratio-preserving scale fixes this without needing to tell
  // the two cases apart: `dominantMag = max(|cmdVelocityLeft|,
  // |cmdVelocityRight|)`; if nonzero and below vMin, BOTH wheels scale by
  // `vMin / dominantMag`, landing the dominant wheel at exactly vMin while
  // preserving the exact commanded ratio -- a symmetric pivot boosts to
  // exactly (-vMin, +vMin) (bit-identical to the pre-131-003
  // per-wheel-independent floor for that case); an already-above-floor
  // differential trim (the one reachable real-world case) passes through
  // unchanged, because its dominant wheel already clears vMin; an
  // asymmetric arc keeps its commanded curvature instead of being
  // flattened toward 1:1 or left unboundedly distorted. Acknowledged
  // tradeoff (unreachable on any shipped robot profile -- see the sprint.md
  // Revision): a differential trim riding on an exactly-/near-zero
  // common-mode component is now ALSO scaled up toward vMin, reverting to
  // the pre-131-003 behavior for that one sub-case. `applySpeedFloor()`'s
  // own "stop is stop" property (`0 * scale == 0` for any finite scale)
  // means a genuine full stop (both raw wheel commands exactly 0.0f)
  // always yields exactly (0.0f, 0.0f), and a raw-zero wheel produced by
  // vector cancellation (part of an active, asymmetric command) also
  // stays exactly zero. 134-002 RESOLVES sprint 131 Design Rationale
  // Decision 4, which this comment used to record as deferred ("the planner
  // does NOT get its own vMin awareness this ticket"): the resolution is
  // not that the planner learns about vMin, it is that the FLOOR learns
  // whose command it is looking at. applySpeedFloor() now no-ops entirely
  // unless Drive owns motion, so a planner-shaped pair -- a decelerating
  // tail, a terminal alignment nudge -- reaches the stages below exactly as
  // it was profiled, while a standing teleop command below breakaway is
  // still boosted exactly as before. See applySpeedFloor()'s own doc
  // comment for the measurement, the deadman-expiry decision, and why the
  // gate reads Drive's own ownership flag instead of querying the planner.
  // The result feeds the rest of tick() exactly where speedLeft/speedRight
  // were read before 131-003 -- Stage A's/Stage B's own commanded-zero guards
  // (correctedCommand()'s `desired == 0.0f`, below, and the
  // `speedLeft == 0.0f` check ahead of fastPid()) therefore keep reading
  // the SAME post-floor value and so cannot disagree with each other by
  // construction.
  float speedLeft, speedRight;  // [mm/s] x2, post-floor
  applySpeedFloor(state.wheelLeft.cmdVelocity, state.wheelRight.cmdVelocity,
                 speedLeft, speedRight);

  // Stage A: the calibrated conversion map (correctedCommand()) plus
  // Stage C's bias -- v_corrected = map(v_cmd) + bias (drive.h's own
  // header). Slope is never adapted online; bias is the ONE adapted
  // parameter, folded in here (not added separately) so the "stop is
  // stop" guard covers it too -- see correctedCommand()'s own doc
  // comment.
  const float correctedLeft =
      correctedCommand(speedLeft, lastSpeedLeft_, true, biasLeft_);
  const float correctedRight =
      correctedCommand(speedRight, lastSpeedRight_, false, biasRight_);
  lastSpeedLeft_ = speedLeft;
  lastSpeedRight_ = speedRight;

  // Wheel-measurement freshness gate (moved ahead of Stage B, 131-002,
  // issue A-commanded-zero-leaks-through-stage-b.md): the SAME fresh/
  // connected/non-frozen conjunct Stage C already computed, now shared by
  // both stages. A failed encoder collect MANUFACTURES velocity = 0
  // (Devices::NezhaMotor::collectEncoder()'s own doc comment,
  // nezha_motor.cpp) rather than reporting the read as missing, so
  // without this gate Stage B would wind its integrator toward closing a
  // gap against a wheel it cannot actually see. `sampleTime` now reflects
  // NezhaMotor's genuine `lastFreshUs_` (131-002, nezha_motor.h) -- the
  // last SUCCESSFUL collect, not merely the last tick() attempt -- so
  // this age check cannot be fooled by a wedged bus stamping "fresh" on a
  // dead reading.
  const bool freshLeft = state.wheelLeft.connected && !state.health.wheelFrozenLeft &&
                        sampleAge(state.time.cycleStart, state.wheelLeft.sampleTime) <=
                            kMaxSampleAge;
  const bool freshRight = state.wheelRight.connected && !state.health.wheelFrozenRight &&
                         sampleAge(state.time.cycleStart, state.wheelRight.sampleTime) <=
                             kMaxSampleAge;

  // Stage B: the fast controller, small authority -- P and feedforward on
  // wheel-SPEED error, I on POSITION error (133-002, fastPid()'s own doc
  // comment). cyclePeriod is last cycle's measured period [us] --
  // consistent with every other value tick() reads here being "as of last
  // cycle" (this function's own call site comment in robot_loop.cpp: "the
  // speeds the owning subsystem staged onto the blackboard LAST cycle").
  //
  // The per-wheel `steady` locals this block used to compute
  // (|cmdAccel| < aSteady) are gone with 133-002's accumulator: their only
  // consumer was Stage B's integrator gate. Stage C still gates on the
  // same condition, but adaptBias() has always evaluated it itself.
  const float dt = static_cast<float>(state.time.cyclePeriod) * 1e-6f;  // [s]
  const float errLeft = speedLeft - state.wheelLeft.velocity;
  const float errRight = speedRight - state.wheelRight.velocity;

  // Commanded-zero guard (131-002): correctedCommand()'s own
  // `desired == 0.0f` guard (Stage A, above) has NO effect here -- Stage B
  // is a fully separate computation, and a wheel that has not yet
  // physically coasted to rest keeps a nonzero err after its commanded
  // speed reaches exactly 0. Skip fastPid() OUTRIGHT for a commanded-zero
  // wheel: no P/feedforward/clamp math runs at all -- "stop is stop"
  // through Stage B too, matching correctedCommand()'s own guard and
  // applySpeedFloor()'s own guard. positionError() carries the SAME
  // commanded-zero condition independently (it re-anchors rather than
  // holding position, which would make Stage B a servo and creep a
  // stopped wheel), so the two agree by construction even though this
  // call is skipped entirely.
  //
  // 133-002: the freshness conjunct this call site used to fold into
  // fastPid()'s `steady` parameter is gone with the accumulator it gated.
  // The hazard it addressed -- winding against a MANUFACTURED ZERO
  // VELOCITY on a failed collect -- cannot reach the I term any more,
  // because the I term reads position, not velocity. freshLeft/freshRight
  // are still computed above and still gate Stage C's adaptBias(), which
  // does still integrate a velocity error.
  //
  // positionError() is called UNCONDITIONALLY, every cycle, and only
  // fastPid() is skipped at commanded zero. This ordering is load-bearing
  // and is NOT how the source patch (issue B §7-D) had it: there, the
  // whole expression was short-circuited on `speed == 0.0f`, which made
  // positionError()'s OWN commanded-zero guard unreachable and left the
  // reference alive across a stop. Two consequences, both wrong:
  //   * the pre-stop position debt survived the stop and was repaid as a
  //     sprint the moment motion resumed -- the loop chasing a position
  //     it had been explicitly told to abandon; and
  //   * the reference did not advance during the zero period while the
  //     wheel DID coast forward, so the error went NEGATIVE by the coast
  //     distance and the first correction after a stop pushed backwards.
  // Calling it every cycle keeps the reference re-anchored on the live
  // reading while stopped, so "stop is stop" holds for Stage B's STATE
  // and not merely for its output. The output is still exactly 0.0 at
  // commanded zero -- no P, no feedforward, no clamp math runs -- which is
  // 131-002's own contract, unchanged.
  const float posErrorLeft = positionError(speedLeft, state.wheelLeft, posRefLeft_, dt);
  const float posErrorRight = positionError(speedRight, state.wheelRight, posRefRight_, dt);
  const float pidLeft = (speedLeft == 0.0f)
      ? 0.0f
      : fastPid(posErrorLeft, errLeft, state.wheelLeft.cmdAccel);
  const float pidRight = (speedRight == 0.0f)
      ? 0.0f
      : fastPid(posErrorRight, errRight, state.wheelRight.cmdAccel);
  lastPidLeft_ = pidLeft;
  lastPidRight_ = pidRight;

  const float dutyLeft =
      crawlDuty((correctedLeft + pidLeft) * dutyPerSpeedLeft_, crawlCarryLeft_);
  const float dutyRight =
      crawlDuty((correctedRight + pidRight) * dutyPerSpeedRight_, crawlCarryRight_);

  // Stage C: bounded bias adaptation -- steady, at/above the speed
  // floor, and only on a fresh, connected, non-frozen measurement (never
  // on a stale/disconnected reading -- robot_state.h's own
  // Health::wheelFrozenLeft/Right doc comment anticipates this
  // consumer). Computed AFTER this tick's duty (bias changes by at most
  // dt/tauAdapt per cycle, so the update lands on the NEXT tick's Stage
  // A, never stepping this tick's own output -- the bumpless-transfer
  // property). freshLeft/Right computed once, above, shared with Stage B.
  adaptBias(biasLeft_, errLeft, state.wheelLeft.cmdAccel, std::fabs(speedLeft),
           freshLeft, dt);
  adaptBias(biasRight_, errRight, state.wheelRight.cmdAccel, std::fabs(speedRight),
           freshRight, dt);

  // Deficit-flag policy (drive.h's own header): a sustained large error
  // while BOTH bias and the fast PID sit pinned at their configured
  // authority ceiling -- the robot has no more correction to give and
  // must say so loudly rather than silently running slow.
  const bool biasSaturatedLeft =
      bounds_.biasMax > 0.0f && std::fabs(biasLeft_) >= bounds_.biasMax;
  const bool pidSaturatedLeft = gains_.pidMax > 0.0f && std::fabs(pidLeft) >= gains_.pidMax;
  const bool biasSaturatedRight =
      bounds_.biasMax > 0.0f && std::fabs(biasRight_) >= bounds_.biasMax;
  const bool pidSaturatedRight =
      gains_.pidMax > 0.0f && std::fabs(pidRight) >= gains_.pidMax;
  updateDeficit(std::fabs(errLeft) > bounds_.deficitThreshold && biasSaturatedLeft &&
                   pidSaturatedLeft,
               state.time.cycleStart, deficitSinceLeft_, deficitLeft_);
  updateDeficit(std::fabs(errRight) > bounds_.deficitThreshold && biasSaturatedRight &&
                   pidSaturatedRight,
               state.time.cycleStart, deficitSinceRight_, deficitRight_);

  // Stop re-assertion (129-001, see drive.h's own header): for
  // kStopEnforceTicks cycles after estop(), and unconditionally for as long
  // as either wheel still measures above kRestVelocity, a commanded zero
  // duty pair bypasses the quiet-at-zero shortcut below -- the leaves keep
  // being explicitly told to stop rather than Drive trusting a write it
  // already believes landed. This can only ever ADD a write: the condition
  // it overrides (alreadyQuiet, below) never holds unless dutyLeft/Right
  // are already both zero, so a genuinely nonzero command is never
  // touched.
  const bool wheelsMoving = std::fabs(left_.velocity()) > kRestVelocity ||
                            std::fabs(right_.velocity()) > kRestVelocity;
  const bool enforceStop = stopEnforceCountdown_ > 0 || wheelsMoving;
  if (stopEnforceCountdown_ > 0) --stopEnforceCountdown_;

  const bool commandedStop = dutyLeft == 0.0f && dutyRight == 0.0f;
  const bool alreadyQuiet =
      commandedStop && writtenLeft_ == 0.0f && writtenRight_ == 0.0f;

  // ARM ON EVERY STOP, not just estop() (133-001, the 2026-08-03 runaway).
  // The window above was armed only by estop(), and its other half
  // (`wheelsMoving`) reads the ENCODER -- so a stop whose single zero write
  // is lost AND whose encoder reads at rest re-asserts nothing, which is
  // precisely the state a wedged, stale, or manufactured-zero encoder
  // produces. The brick physically latches its last commanded speed, so
  // that combination is a permanent runaway with no witness (nezha_motor.h's
  // own "LOAD-BEARING (129-001)" note describes the same trap one layer
  // down). Arming here, on the nonzero->zero transition of the DUTY pair
  // itself, makes re-assertion depend on what was COMMANDED rather than on
  // what the encoder claims happened -- this condition reads no measured
  // velocity at all. Self-limiting: once the zero write lands,
  // writtenLeft_/Right_ are both 0, alreadyQuiet holds, and the window is
  // not re-armed. Adds writes only -- a nonzero command never satisfies
  // commandedStop.
  if (commandedStop && !alreadyQuiet) stopEnforceCountdown_ = kStopEnforceTicks;

  if (alreadyQuiet && !enforceStop) return;
  left_.setDuty(dutyLeft);
  right_.setDuty(dutyRight);
  writtenLeft_ = dutyLeft;
  writtenRight_ = dutyRight;
}

}  // namespace App
