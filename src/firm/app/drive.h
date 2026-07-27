// drive.h -- App::Drive: the base's wheel-target sink. Stages a commanded
// per-wheel target and applies it onto the two Devices::NezhaMotor leaves.
// Implements Motion::WheelSink (motion/wheel_sink.h) so Motion::MoveQueue
// can command it through that boundary interface rather than by concrete
// type.
//
// Boundary: inside -- staging vL/vR onto the two NezhaMotor leaves; outside
// -- the kinematics math (122-002: the twist decomposition --
// BodyKinematics::inverse() -- moved to Motion::MoveQueue, which now calls
// the sink's setDuty() directly with already-decomposed wheel targets;
// Drive never sees a twist) and the deadman decision (the loop calls
// Drive::stop(); Drive never polls Deadman).
//
// 125-003 (INTERIM closed loop -- see sprint.md Decision 1/2): callers still
// hand setDuty() raw mm/s velocity targets under the duty-shaped name
// (125-002's own placeholder contract, unchanged) -- `Motion::WheelSink`'s
// real duty semantics land once `Motion::MoveQueue` itself owns the PID
// (ticket 005/006, sprint.md Decision 1). Until then, SOMETHING has to
// convert those mm/s targets into real, clamped [-1,1] duty before they
// reach `Devices::Motor::setDuty()` -- ticket 003 (this shrink) deleted
// `Devices::NezhaMotor`'s own embedded velocity PID outright (it is a
// control DECISION, not hardware protection -- Decision 2's reframing), so
// `Drive` holds two `Motion::WheelVelocityPid` instances here as a
// documented INTERIM placeholder: this is NOT where Decision 1 says the PID
// permanently lives (that's `Motion::MoveQueue`, fed by ticket 004's
// `App::WheelObserver` `WheelEstimate`, not a raw `Devices::Motor::velocity()`
// read) -- it exists here only so the tree keeps driving correctly (real,
// proportional duty, not a `|value|>1` clamp-to-max-speed regression) across
// the gap between this ticket and tickets 004-006. Ticket 005 should DELETE
// pid_/gains_ from this class entirely once MoveQueue owns the real thing.
//
// Known degradation vs. the pre-125-003 embedded-PID behavior (disclosed,
// not silent -- see nezha_motor.cpp's own header for the sibling
// measurement-conditioning degradation): tick() runs at the TOP of
// RobotLoop::cycle(), before either motor's own request/collect this same
// cycle -- so the `measured` velocity fed to pid_ here is one full cycle
// (~40ms) STALER than the pre-125-003 design (which ran the PID compute
// INSIDE NezhaMotor::tick(), against the sample that same tick() call had
// just collected). A fixed nominal dt (kNominalPeriod, matching
// App::RobotLoop::kCycle) stands in for a true elapsed-time read, since
// Drive has no Clock& of its own and the loop's own cycle period is fixed
// by design -- true in both sim (kCycleDtUs, sim_harness.h) and on real
// hardware (kPace's own derivation, robot_loop.cpp).
#pragma once

#include "devices/motor.h"
#include "motion/wheel_sink.h"
#include "motion/wheel_velocity_pid.h"

namespace App {

class Drive : public Motion::WheelSink {
 public:
  // left/right -- the two drive-wheel NezhaMotor leaves, in BodyKinematics'
  // own L/R convention (Motion::MoveQueue's own inverse()-derived vL/vR
  // order). trackWidth -- [mm], BodyKinematics::inverse()/forward()'s own
  // `b` parameter -- Drive no longer uses it for its own kinematics (122-002
  // moved that math to Motion::MoveQueue), but keeps holding/exposing it via
  // trackWidth() below, since App::RobotLoop::updateTlm() still needs the
  // SAME value to fuse the two leaves' measured velocities for telemetry
  // (BodyKinematics::forward()), and Drive is where that value has always
  // been constructed.
  Drive(Devices::Motor& left, Devices::Motor& right, float trackWidth);

  // Stages left/right directly -- the ONE staging path left on Drive
  // (122-002: the twist path, setTwist()/BodyKinematics::inverse(), moved to
  // Motion::MoveQueue, which now calls this method with already-decomposed
  // wheel targets instead). Does not itself reach into the leaves -- tick()
  // is the only method that ever calls Motor::setDuty(). 125-003 INTERIM:
  // still a velocity mm/s target under the duty-shaped name -- see this
  // file's own header.
  void setDuty(float left, float right) override;  // [mm/s] [mm/s] (INTERIM: not yet real duty)

  // Stages a zero target. The next tick() call stages exactly 0 onto both
  // leaves.
  void stop() override;

  // Apply this cycle's duty pair to the two leaves (quiet at zero -- see
  // drive.cpp).
  void tick(float dutyLeft, float dutyRight);  // [-1, 1] each

  // trackWidth -- read-only accessor (109-009: RobotLoop::updateTlm() needs
  // it to fuse the two leaves' measured velocities into the primary frame's
  // `twist` field via BodyKinematics::forward() -- see that method's own
  // call site). No setter: trackWidth_ is fixed at construction, matching
  // Drive's own "no live-reconfigure" contract.
  float trackWidth() const { return trackWidth_; }  // [mm]

  // vLeft/vRight -- read-only accessors onto the last-staged setDuty()/
  // stop() target (124-009): Types::RobotState::Wheel::cmdVelocity's own
  // source (robot_state.h's own doc comment: "writer: App::Drive::tick()'s
  // own staged target") -- RobotLoop::cycle() reads these immediately after
  // both wheels' collects to publish the wheel section's commanded-velocity
  // field, mirroring trackWidth()'s own read-only-accessor shape. No
  // setter, same reasoning as trackWidth(): staging stays exclusively
  // through setDuty()/stop().
  float vLeft() const { return vLeft_; }    // [mm/s] signed
  float vRight() const { return vRight_; }  // [mm/s] signed

  // gainsLeft/gainsRight/applyGainsLeft/applyGainsRight -- 125-003 INTERIM
  // (this file's own header): RobotLoop::applyMotorConfigPatch() reads/
  // writes the interim pid_'s own gains here instead of
  // Devices::Motor::gains()/applyGains() (deleted from the Motor interface
  // -- the PID is no longer motor-resident). Ticket 005/008 should retarget
  // these call sites at whatever MoveQueue-owned surface replaces this
  // class's own interim pid_ members.
  const Motion::Gains& gainsLeft() const { return gainsL_; }
  const Motion::Gains& gainsRight() const { return gainsR_; }
  void applyGainsLeft(const Motion::Gains& gains) { gainsL_ = gains; }
  void applyGainsRight(const Motion::Gains& gains) { gainsR_ = gains; }

 private:
  Devices::Motor& left_;
  Devices::Motor& right_;
  float trackWidth_;  // [mm]

  float vLeft_ = 0.0f;   // [mm/s]
  float vRight_ = 0.0f;  // [mm/s]

  // ---- 125-003 INTERIM closed loop (this file's own header) ----
  // (Members above the interim PID pair -- see setDuty() below, the
  // planner-era path that bypasses it.)
  Motion::WheelVelocityPid pidL_;
  Motion::WheelVelocityPid pidR_;
  // Last tick's staged targets -- the measurement-lead compensation's
  // commanded-accel basis (drive.cpp Drive::tick()).
  float vLeftPrevious_ = 0.0f;    // [mm/s]
  float vRightPrevious_ = 0.0f;   // [mm/s]
  float vLeftPrevious2_ = 0.0f;   // [mm/s] two ticks back
  float vRightPrevious2_ = 0.0f;  // [mm/s]
  // Last duty pair actually written (quiet-at-zero baseline).
  float writtenLeft_ = 0.0f;   // [-1, 1]
  float writtenRight_ = 0.0f;  // [-1, 1]
  Motion::Gains gainsL_;
  Motion::Gains gainsR_;

  // Fixed nominal control period -- matches App::RobotLoop::kCycle (40ms).
  // Not derived (Devices/App::Drive layering means this file cannot
  // reference robot_loop.h's own constant without a circular include --
  // see nezha_motor.cpp's own kMinWriteIntervalUs comment for the same
  // App->Devices/App-internal layering note); the loop's own cycle period
  // is fixed by design (runAndWait's own derived-gap scheduling,
  // robot_loop.cpp), so a fixed dt is exact in both sim (kCycleDtUs,
  // sim_harness.h) and on real hardware, not an approximation.
  static constexpr float kNominalPeriod = 0.04f;  // [s]
};

}  // namespace App
