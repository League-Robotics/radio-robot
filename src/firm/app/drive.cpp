#include <cmath>

#include "app/drive.h"

namespace App {

Drive::Drive(Devices::Motor& left, Devices::Motor& right, float trackWidth)
    : left_(left), right_(right), trackWidth_(trackWidth) {}

// 125-003 INTERIM: still a velocity mm/s target under the duty-shaped name
// -- see drive.h's own file header for why and what replaces this.
void Drive::setDuty(float left, float right) {
  targetLeft_ = left;
  targetRight_ = right;
}

void Drive::setPlannerTargets(float vLeft, float vRight, bool plannerActive) {
  if (commandActive_ || !plannerActive) return;
  targetLeft_ = vLeft;
  targetRight_ = vRight;
}

void Drive::command(float vLeft, float vRight, float durationMs,
                    uint32_t moveId, uint32_t now) {
  targetLeft_ = vLeft;
  targetRight_ = vRight;
  commandDeadline_ = now + static_cast<uint32_t>(durationMs);
  commandMoveId_ = moveId;
  commandActive_ = true;
}

void Drive::stop() {
  targetLeft_ = 0.0f;
  targetRight_ = 0.0f;
  commandActive_ = false;
}

bool Drive::takeCompletion(uint32_t* moveId) {
  if (!completionPending_) return false;
  completionPending_ = false;
  *moveId = completedMoveId_;
  return true;
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
void Drive::tick(uint32_t now) {
  if (commandActive_ &&
      static_cast<int32_t>(now - commandDeadline_) >= 0) {
    commandActive_ = false;
    targetLeft_ = 0.0f;
    targetRight_ = 0.0f;
    completionPending_ = true;
    completedMoveId_ = commandMoveId_;
  }

  float dutyLeft = crawlDuty(targetLeft_ * dutyPerSpeedLeft_, crawlCarryLeft_);
  float dutyRight =
      crawlDuty(targetRight_ * dutyPerSpeedRight_, crawlCarryRight_);
  const bool quiet = dutyLeft == 0.0f && dutyRight == 0.0f &&
                     writtenLeft_ == 0.0f && writtenRight_ == 0.0f;
  if (quiet) return;
  left_.setDuty(dutyLeft);
  right_.setDuty(dutyRight);
  writtenLeft_ = dutyLeft;
  writtenRight_ = dutyRight;
}

}  // namespace App
