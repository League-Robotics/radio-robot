// drive.h -- App::Drive: the wheel-drive subsystem. Owns the wheel
// velocity targets, the bounded wheel command (deadline + completion),
// the per-wheel duty calibration, crawl shaping, and the leaf writes.
// No controller -- open-loop duty from calibrated targets.
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

  // Stage wheel velocity targets (WheelSink; also the harness path). No
  // deadline -- targets hold until changed or stop().
  void setDuty(float left, float right) override;  // [mm/s] [mm/s] velocity targets

  // Adopt the planner's published wheel targets (state hand-off) when the
  // planner owns motion and no wheel command is armed.
  void setPlannerTargets(float vLeft, float vRight, bool plannerActive);

  // Arm a bounded wheel command: targets + expiry deadline + the Move id
  // acked on completion (takeCompletion()).
  void command(float vLeft, float vRight, float durationMs, uint32_t moveId,
               uint32_t now);  // [mm/s] [mm/s] [ms] -- now [ms]

  // Clear targets and any armed command.
  void stop() override;

  // One tick: expire an armed command whose deadline passed, convert the
  // targets to duty (per-wheel calibration + crawl shaping + quiet at
  // zero), and write the leaves.
  void tick(uint32_t now);  // [ms]

  // One-shot completion event for an expired command (the loop acks it).
  bool takeCompletion(uint32_t* moveId);

  // Last-staged velocity targets (state publish reads these).
  float targetLeft() const { return targetLeft_; }    // [mm/s] signed
  float targetRight() const { return targetRight_; }  // [mm/s] signed

  // Per-wheel open-loop calibration ([duty/(mm/s)]).
  void setDutyPerSpeed(float left, float right) {
    dutyPerSpeedLeft_ = left;
    dutyPerSpeedRight_ = right;
  }

  // Crawl-mode pulse amplitude; 0 disables (per-robot breakaway property).
  void setCrawlPulse(float crawlPulse) { crawlPulse_ = crawlPulse; }

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
  float vLeft() const { return targetLeft_; }    // [mm/s] signed (legacy alias)
  float vRight() const { return targetRight_; }  // [mm/s] signed (legacy alias)

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

  float targetLeft_ = 0.0f;   // [mm/s]
  float targetRight_ = 0.0f;  // [mm/s]

  // Armed bounded command (command()/tick() expiry).
  bool commandActive_ = false;
  uint32_t commandDeadline_ = 0;  // [ms]
  uint32_t commandMoveId_ = 0;
  bool completionPending_ = false;
  uint32_t completedMoveId_ = 0;

  // Open-loop duty per commanded speed, per wheel (measured plant gains,
  // speed_sweep 2026-07-27: L ~560, R ~510 mm/s per duty).
  float dutyPerSpeedLeft_ = 1.0f / 560.0f;   // [duty/(mm/s)]
  float dutyPerSpeedRight_ = 1.0f / 510.0f;  // [duty/(mm/s)]

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
  // Crawl shaper (drive.cpp crawlDuty()).
  float crawlDuty(float duty, float& carry) const;
  float crawlPulse_ = 0.0f;  // [-1, 1] pulse amplitude; 0 = off
  float crawlCarryLeft_ = 0.0f;   // Bresenham accumulators
  float crawlCarryRight_ = 0.0f;

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
