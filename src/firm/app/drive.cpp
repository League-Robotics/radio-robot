#include <algorithm>
#include <cmath>

#include "app/drive.h"

namespace App {

Drive::Drive(Devices::Motor& left, Devices::Motor& right, float trackWidth)
    : left_(left), right_(right), trackWidth_(trackWidth) {}

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
  const bool owned = commandActive_;

  if (commandActive_ && static_cast<int32_t>(now - commandDeadline_) >= 0) {
    commandActive_ = false;
    targetLeft_ = 0.0f;
    targetRight_ = 0.0f;
    completionPending_ = true;
    completedMoveId_ = commandMoveId_;
  }

  if (!owned) return;

  state.wheelLeft.cmdVelocity = targetLeft_;
  state.wheelRight.cmdVelocity = targetRight_;
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
  if (desired == 0.0f) return 0.0f;
  const int w = leftWheel ? 0 : 1;
  const int d = (std::fabs(desired) > std::fabs(previous)) ? 0 : 1;
  const float magnitude =
      (std::fabs(desired) - corrIntercept_[w][d]) / corrGain_[w][d];
  if (magnitude <= 0.0f) return 0.0f;
  const float correctedMagnitude = magnitude + bias;
  if (correctedMagnitude <= 0.0f) return 0.0f;
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
  if (bounds_.vMin <= 0.0f) return;
  const float dominantMag = std::max(std::fabs(rawLeft), std::fabs(rawRight));
  if (dominantMag <= 0.0f || dominantMag >= bounds_.vMin) return;
  const float scale = bounds_.vMin / dominantMag;
  speedLeft = rawLeft * scale;
  speedRight = rawRight * scale;
}

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

  float pid = proportional + feed + integral;
  if (clamped) {
    if (pid > gains_.pidMax) pid = gains_.pidMax;
    if (pid < -gains_.pidMax) pid = -gains_.pidMax;
  }
  if (!std::isfinite(pid)) return 0.0f;
  return pid;
}

void Drive::adaptBias(float& bias, float err, float aCmd, float vCmdMagnitude,
                      bool fresh, float dt) const {
  if (bounds_.tauAdapt <= 0.0f || dt <= 0.0f || !fresh) return;
  if (std::fabs(aCmd) >= bounds_.aSteady) return;
  if (vCmdMagnitude < bounds_.vMin) return;
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
  const bool steadyLeft = std::fabs(state.wheelLeft.cmdAccel) < bounds_.aSteady;
  const bool steadyRight = std::fabs(state.wheelRight.cmdAccel) < bounds_.aSteady;

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

}
