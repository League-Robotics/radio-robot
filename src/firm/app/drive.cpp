// drive.cpp -- App::Drive implementation. See drive.h's file header for the
// two responsibilities and the tick/update contract.
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

void Drive::estop() {
  targetLeft_ = 0.0f;
  targetRight_ = 0.0f;
  commandActive_ = false;
  // completionPending_ is deliberately NOT set: a discarded command does
  // not complete (drive.h's own estop() doc comment). An ALREADY-latched
  // completion from a command that expired normally this cycle is left
  // alone -- it describes something that really happened.
}

void Drive::setDuty(float left, float right) {
  targetLeft_ = left;
  targetRight_ = right;
}

void Drive::stop() { estop(); }

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

// Apply the cycle's duty pair to the two leaves, crawl-shaped. Quiet at
// zero: while both the commanded and last-written pairs are exactly zero
// there is nothing to say to the hardware -- writing anyway would flip
// the motors out of Mode::None from the first idle boot cycle and make
// boot-time config pushes race the at-rest reconfigure gate.
void Drive::tick(float speedLeft, float speedRight) {
  // Fail closed (command-ingestion-...-two-stops.md §6): with no calibration
  // installed there is no honest speed->duty conversion to make, so write
  // nothing at all rather than guess. A robot whose JSON is missing the
  // calibration never gets here -- codegen fails first -- so reaching this
  // return means a composition root that skipped setDutyPerSpeed(), and
  // standing still is the right answer to that.
  if (!calibrated_) return;

  const float dutyLeft =
      crawlDuty(speedLeft * dutyPerSpeedLeft_, crawlCarryLeft_);
  const float dutyRight =
      crawlDuty(speedRight * dutyPerSpeedRight_, crawlCarryRight_);
  const bool quiet = dutyLeft == 0.0f && dutyRight == 0.0f &&
                     writtenLeft_ == 0.0f && writtenRight_ == 0.0f;
  if (quiet) return;
  left_.setDuty(dutyLeft);
  right_.setDuty(dutyRight);
  writtenLeft_ = dutyLeft;
  writtenRight_ = dutyRight;
}

}  // namespace App
