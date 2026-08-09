#include <algorithm>
#include <cmath>

#include "app/drive.h"

namespace App {

Drive::Drive(Hal::Motor& left, Hal::Motor& right, float trackWidth)
    : left_(left), right_(right), trackWidth_(trackWidth) {}

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
  bounds.stallSpeed = config.wheelControl.stall_speed;
  bounds.stallDemand = config.wheelControl.stall_demand;
  bounds.stallWindow = config.wheelControl.stall_window;
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

}

void Drive::estop() {
  takeover();

  stopEnforceCountdown_ = kStopEnforceTicks;

  posRefLeft_ = PositionRef{};
  posRefRight_ = PositionRef{};
  biasLeft_ = 0.0f;
  biasRight_ = 0.0f;
  deficitSinceLeft_ = 0;
  deficitSinceRight_ = 0;
  deficitLeft_ = false;
  deficitRight_ = false;
  stallSinceLeft_ = 0;
  stallSinceRight_ = 0;
  stallLeft_ = false;
  stallRight_ = false;
}

bool Drive::takeCompletion(uint32_t* moveId) {
  if (!completionPending_) return false;
  completionPending_ = false;
  *moveId = completedMoveId_;
  return true;
}

void Drive::update(Types::RobotState& state, uint32_t now) {
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

void Drive::setWheelCorrection(float gLA, float iLA, float gLD, float iLD,
                               float gRA, float iRA, float gRD, float iRD) {
  corrGain_[0][0] = gLA;  corrIntercept_[0][0] = iLA;
  corrGain_[0][1] = gLD;  corrIntercept_[0][1] = iLD;
  corrGain_[1][0] = gRA;  corrIntercept_[1][0] = iRA;
  corrGain_[1][1] = gRD;  corrIntercept_[1][1] = iRD;
}

float Drive::correctedCommand(float desired, float previous, bool leftWheel,
                              float bias) const {
  if (desired == 0.0f) return 0.0f;  // stop is stop; never offset it (map OR bias)
  const int w = leftWheel ? 0 : 1;
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
    bias = 0.0f;
  }
}

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

// updateStall() -- the sustain-and-latch half of stall detection. Same shape
// as updateDeficit() above and deliberately kept a SEPARATE function rather
// than merged with it: the two describe different faults, gate on different
// config keys, and only this one causes the robot to be halted, so a future
// change to either must not silently move the other.
void Drive::updateStall(bool conditionNow, uint32_t now, uint32_t& since,
                        bool& latched) const {
  if (bounds_.stallWindow <= 0.0f || !conditionNow) {
    since = 0;
    latched = false;
    return;
  }
  if (since == 0) since = now;
  latched = (now - since) >= static_cast<uint32_t>(bounds_.stallWindow);
}

uint32_t Drive::sampleAge(uint32_t now, uint32_t sampleTime) const {
  return (now < sampleTime) ? 0u : (now - sampleTime);
}

void Drive::tick(const Types::RobotState& state) {
  if (!calibrated_) return;

  float speedLeft, speedRight;  // [mm/s] x2, post-floor
  applySpeedFloor(state.wheelLeft.cmdVelocity, state.wheelRight.cmdVelocity,
                 speedLeft, speedRight);

  const float correctedLeft =
      correctedCommand(speedLeft, lastSpeedLeft_, true, biasLeft_);
  const float correctedRight =
      correctedCommand(speedRight, lastSpeedRight_, false, biasRight_);
  lastSpeedLeft_ = speedLeft;
  lastSpeedRight_ = speedRight;

  const bool freshLeft = state.wheelLeft.connected && !state.health.wheelFrozenLeft &&
                        sampleAge(state.time.cycleStart, state.wheelLeft.sampleTime) <=
                            kMaxSampleAge;
  const bool freshRight = state.wheelRight.connected && !state.health.wheelFrozenRight &&
                         sampleAge(state.time.cycleStart, state.wheelRight.sampleTime) <=
                             kMaxSampleAge;

  const float dt = static_cast<float>(state.time.cyclePeriod) * 1e-6f;  // [s]
  const float errLeft = speedLeft - state.wheelLeft.velocity;
  const float errRight = speedRight - state.wheelRight.velocity;

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

  adaptBias(biasLeft_, errLeft, state.wheelLeft.cmdAccel, std::fabs(speedLeft),
           freshLeft, dt);
  adaptBias(biasRight_, errRight, state.wheelRight.cmdAccel, std::fabs(speedRight),
           freshRight, dt);

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

  // STALL -- the robot was asked to move and is not moving.
  //
  // MEASURED ON tovez 2026-08-08, driving into the playfield rail: the FIRST
  // version of this test asked "are the WHEELS turning", gated on the encoder
  // being believable (`!wheelFrozen`). It never fired, for two compounding
  // reasons, and both are why the test now lives on the OTOS:
  //
  //  1. On a slippery mat a blocked robot's wheels SLIP rather than stop --
  //     the first rail run logged 239/222 encoder ticks (~17cm of wheel
  //     rotation) while the robot sat still against the rail.
  //  2. When the wheels DID stop dead (second run: encoders frozen 4.4s while
  //     commanded 80mm/s), Hardware::MotorArmor::updateWedgeDetector() had
  //     already latched wedgeSuspect() -- because its condition, "position
  //     unchanged while duty is applied", IS the definition of a stall. The
  //     believability guard therefore suppressed the detector exactly when
  //     the stall was real. Encoders alone CANNOT separate "the wheel is
  //     held" from "the encoder is dead": both look identical.
  //
  // The OTOS separates them, because it measures ground travel optically and
  // is independent of the encoders entirely. Same run: |v_x| read ~150mm/s
  // while rolling and 4.6mm/s while jammed against the rail.
  //
  // Rotation counts as moving: a PIVOT translates nothing, so v_x alone would
  // call every in-place turn a stall. omega is converted to the speed it
  // implies at the wheel rim (omega * halfTrack) so the SAME stallSpeed
  // threshold covers both, and no second config field is needed.
  const float halfTrack = 0.5f * trackWidth_;  // [mm]
  const bool otosBelievable =
      state.otos.present && state.otos.connected &&
      sampleAge(state.time.cycleStart, state.otos.sampleTime) <= kMaxSampleAge;
  const bool bodyStill = std::fabs(state.otos.v_x) <= bounds_.stallSpeed &&
                         std::fabs(state.otos.omega) * halfTrack <= bounds_.stallSpeed;

  // Gate on the RAW cmdVelocity, not the post-floor speed*: applySpeedFloor()
  // boosts sub-v_min commands up to the floor, and a stall test run off the
  // boosted value would read a near-zero request as a demand for motion.
  const bool demanding =
      std::fabs(state.wheelLeft.cmdVelocity) > bounds_.stallDemand ||
      std::fabs(state.wheelRight.cmdVelocity) > bounds_.stallDemand;

  // FALLBACK with no usable OTOS: fall back to the encoders, and deliberately
  // do NOT suppress on wheelFrozen this time -- see reason 2 above. That
  // trades a possible false halt on a genuinely dead encoder for catching a
  // real jam, which is the right way round: stopping a healthy robot is
  // recoverable, grinding a wheel against a rail until a human notices is the
  // failure this exists to end.
  const bool encoderStill =
      std::fabs(state.wheelLeft.velocity) <= bounds_.stallSpeed &&
      std::fabs(state.wheelRight.velocity) <= bounds_.stallSpeed &&
      state.wheelLeft.connected && state.wheelRight.connected;

  const bool stalled = demanding && (otosBelievable ? bodyStill : encoderStill);

  // One physical condition -- the ROBOT is stuck -- so both wheels report it.
  // Which wheel is "at fault" is not a question the OTOS can answer, and
  // pretending otherwise would put a guess on the wire.
  updateStall(stalled, state.time.cycleStart, stallSinceLeft_, stallLeft_);
  updateStall(stalled, state.time.cycleStart, stallSinceRight_, stallRight_);

  const bool wheelsMoving = std::fabs(left_.velocity()) > kRestVelocity ||
                            std::fabs(right_.velocity()) > kRestVelocity;
  const bool enforceStop = stopEnforceCountdown_ > 0 || wheelsMoving;
  if (stopEnforceCountdown_ > 0) --stopEnforceCountdown_;

  const bool commandedStop = dutyLeft == 0.0f && dutyRight == 0.0f;
  const bool alreadyQuiet =
      commandedStop && writtenLeft_ == 0.0f && writtenRight_ == 0.0f;

  if (commandedStop && !alreadyQuiet) stopEnforceCountdown_ = kStopEnforceTicks;

  if (alreadyQuiet && !enforceStop) return;
  left_.setDuty(dutyLeft);
  right_.setDuty(dutyRight);
  writtenLeft_ = dutyLeft;
  writtenRight_ = dutyRight;
}

}  // namespace App
