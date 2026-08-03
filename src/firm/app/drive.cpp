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
  pidIntegralLeft_ = 0.0f;
  pidIntegralRight_ = 0.0f;
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
void Drive::applySpeedFloor(float rawLeft, float rawRight, float& speedLeft,
                            float& speedRight) const {
  speedLeft = rawLeft;
  speedRight = rawRight;
  if (bounds_.vMin <= 0.0f) return;  // uncalibrated: no-op, same as before
  const float dominantMag = std::max(std::fabs(rawLeft), std::fabs(rawRight));
  if (dominantMag <= 0.0f || dominantMag >= bounds_.vMin) return;
  const float scale = bounds_.vMin / dominantMag;
  speedLeft = rawLeft * scale;
  speedRight = rawRight * scale;
}

// Stage B -- the fast PID (small authority, never carries standing error
// because Stage A+C already do -- see drive.h's own header). `integral`
// is the caller's own persistent per-wheel state (pidIntegralLeft_/
// Right_), mutated in place. `steady` gates the integrator exactly like
// Motion::WheelTrim's own Hold-phase gate (wheel_trim.cpp) -- ramping
// error is transient work that belongs to kaff/kp, not the integral;
// outside `steady` the integrator is FROZEN, not reset, so a chain of
// legs does not throw away trim it already learned. Anti-windup: do not
// integrate further into a clamp the output is already pinned against
// (same shape as WheelTrim::compute()'s own pushingIntoClamp gate).
float Drive::fastPid(float& integral, float err, float aCmd, float dt,
                     bool steady) const {
  const float proportional = gains_.kp * err;
  const float feed = gains_.kaff * aCmd;

  const bool clamped = gains_.pidMax > 0.0f;
  const float provisional = proportional + feed + integral;
  const bool pushingIntoClamp =
      clamped && ((provisional >= gains_.pidMax && err > 0.0f) ||
                  (provisional <= -gains_.pidMax && err < 0.0f));

  if (steady && gains_.iMax > 0.0f && gains_.ki != 0.0f && !pushingIntoClamp &&
      dt > 0.0f) {
    integral += gains_.ki * err * dt;
    if (integral > gains_.iMax) integral = gains_.iMax;
    if (integral < -gains_.iMax) integral = -gains_.iMax;
  }
  // else FROZEN, not reset.

  float pid = proportional + feed + integral;
  if (clamped) {
    if (pid > gains_.pidMax) pid = gains_.pidMax;
    if (pid < -gains_.pidMax) pid = -gains_.pidMax;
  }
  if (!std::isfinite(pid)) return 0.0f;  // fail closed, never inject NaN
  return pid;
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
  // stays exactly zero. The planner does NOT get its own vMin awareness
  // this ticket (deferred, sprint 131 Design Rationale Decision 4 -- review
  // C1's terminal-taper mismatch waits on the same loaded-actuation-floor
  // bench measurement as vMin/biasMax themselves). The result feeds the
  // rest of tick() exactly where speedLeft/speedRight were read before
  // this ticket -- Stage A's/Stage B's own commanded-zero guards
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

  // Stage B: the fast PID, small authority, on wheel-speed error.
  // cyclePeriod is last cycle's measured period [us] -- consistent with
  // every other value tick() reads here being "as of last cycle"
  // (this function's own call site comment in robot_loop.cpp: "the
  // speeds the owning subsystem staged onto the blackboard LAST cycle").
  const float dt = static_cast<float>(state.time.cyclePeriod) * 1e-6f;  // [s]
  const float errLeft = speedLeft - state.wheelLeft.velocity;
  const float errRight = speedRight - state.wheelRight.velocity;
  const bool steadyLeft = std::fabs(state.wheelLeft.cmdAccel) < bounds_.aSteady;
  const bool steadyRight = std::fabs(state.wheelRight.cmdAccel) < bounds_.aSteady;

  // Commanded-zero guard (131-002): correctedCommand()'s own
  // `desired == 0.0f` guard (Stage A, above) has NO effect here -- Stage B
  // is a fully separate computation reading the wheel's MEASURED
  // velocity, and a wheel that has not yet physically coasted to rest
  // keeps a nonzero err after its commanded speed reaches exactly 0
  // (steadyLeft/Right can go true the instant the deceleration itself
  // ends, well before the wheel actually stops -- the integrator would
  // then resume winding against the residual coast-down error). Skip
  // fastPid() OUTRIGHT for a commanded-zero wheel: no P/feedforward/clamp
  // math runs, and the integrator is left completely untouched (frozen,
  // not reset) -- "stop is stop" through Stage B too, matching
  // correctedCommand()'s own guard and applySpeedFloor()'s own guard.
  // Otherwise, additionally require this wheel's freshness (computed
  // above) before the integrator may accumulate -- folded into the SAME
  // `steady` parameter fastPid() already gates on, so a stale/
  // disconnected/frozen reading freezes it exactly like the existing
  // !steady case, rather than winding against a manufactured
  // zero-velocity reading.
  const float pidLeft = (speedLeft == 0.0f)
      ? 0.0f
      : fastPid(pidIntegralLeft_, errLeft, state.wheelLeft.cmdAccel, dt,
                steadyLeft && freshLeft);
  const float pidRight = (speedRight == 0.0f)
      ? 0.0f
      : fastPid(pidIntegralRight_, errRight, state.wheelRight.cmdAccel, dt,
                steadyRight && freshRight);
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
  if (alreadyQuiet && !enforceStop) return;
  left_.setDuty(dutyLeft);
  right_.setDuty(dutyRight);
  writtenLeft_ = dutyLeft;
  writtenRight_ = dutyRight;
}

}  // namespace App
