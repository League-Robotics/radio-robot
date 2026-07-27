#include <cmath>

#include "app/drive.h"

namespace App {

Drive::Drive(Devices::Motor& left, Devices::Motor& right, float trackWidth)
    : left_(left), right_(right), trackWidth_(trackWidth) {}

// 125-003 INTERIM: still a velocity mm/s target under the duty-shaped name
// -- see drive.h's own file header for why and what replaces this.
void Drive::setDuty(float left, float right) {
  vLeft_ = left;
  vRight_ = right;
}

void Drive::stop() {
  vLeft_ = 0.0f;
  vRight_ = 0.0f;
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
void Drive::tick(float dutyLeft, float dutyRight) {
  dutyLeft = crawlDuty(dutyLeft, crawlCarryLeft_);
  dutyRight = crawlDuty(dutyRight, crawlCarryRight_);
  const bool quiet = dutyLeft == 0.0f && dutyRight == 0.0f &&
                     writtenLeft_ == 0.0f && writtenRight_ == 0.0f;
  if (quiet) return;
  left_.setDuty(dutyLeft);
  right_.setDuty(dutyRight);
  writtenLeft_ = dutyLeft;
  writtenRight_ = dutyRight;
}

}  // namespace App
