// drive.h -- App::Drive: the wheel-drive subsystem and the owner of the
// WHEELS command's whole lifecycle. Two responsibilities, and only two:
//
//   1. The bounded wheel command (command-ingestion-ring-buffered-comms-
//      subsystem-routing-two-stops.md §4). `WHEELS` is the dumb teleop
//      primitive -- a per-wheel velocity pair held for a fixed duration --
//      and Drive owns its targets, its deadline, and its completion event.
//      Before this change those four pieces of state lived as loose members
//      on App::RobotLoop and were overwritten in place by each arriving
//      command, so a superseded command never completed and never acked.
//   2. Actuation: commanded wheel SPEED -> motor duty, via the per-wheel
//      open-loop calibration and the crawl shaper, then the leaf writes.
//
// There is no controller here -- duty is open loop from calibrated speed.
// Closed-loop wheel control lives in Motion::Planner's own duty stage.
//
// Two-method contract, adopted from Motion::Planner (planner.h) so the two
// motion deciders read the same way at their call sites:
//   tick(speedLeft, speedRight)      -- DO THE WORK: shape and write duty.
//   update(Types::RobotState&, now)  -- SAVE results into the blackboard:
//                                       expire the armed command and
//                                       publish this subsystem's targets,
//                                       but only while Drive OWNS motion.
//
// Exactly one subsystem owns motion at a time; that exclusivity is enforced
// at routing (App::RobotLoop::routeCommand -- a WHEELS command clears the
// planner, a MOVE clears Drive's armed command), and `owns()` below is how
// the loop reads which one it currently is.
#pragma once

#include <cstdint>

#include "devices/motor.h"
#include "firm/types/robot_state.h"
#include "motion/wheel_sink.h"

namespace App {

class Drive : public Motion::WheelSink {
 public:
  // left/right -- the two drive-wheel NezhaMotor leaves, in BodyKinematics'
  // own L/R convention. trackWidth -- [mm], BodyKinematics::inverse()/
  // forward()'s own `b` parameter: Drive does no kinematics of its own, but
  // holds and exposes this value because RobotLoop::publishPose() needs the
  // SAME number to fuse the two leaves' measured velocities into the
  // telemetry twist, and Drive is where it has always been constructed.
  Drive(Devices::Motor& left, Devices::Motor& right, float trackWidth);

  // Install this robot's own wheel calibration
  // (command-ingestion-ring-buffered-comms-subsystem-routing-two-stops.md
  // §6). Drive carries NO calibration defaults, so this MUST be called --
  // by the composition root, from Config::defaultDriveConfig() -- before
  // any motion is commanded; until it is, tick() writes nothing (see
  // calibrated() below). dutyPerSpeed* is the inverse of the measured plant
  // gain: duty = speed * dutyPerSpeed, per wheel because the two gearboxes
  // genuinely differ ~10%.
  void setDutyPerSpeed(float left, float right) {  // [duty/(mm/s)] x2
    dutyPerSpeedLeft_ = left;
    dutyPerSpeedRight_ = right;
    calibrated_ = left != 0.0f && right != 0.0f;
  }

  // Crawl-mode pulse amplitude; 0 disables (per-robot breakaway property,
  // and the shipped default -- see Config::DriveBootConfig's own doc
  // comment for why 0.20 was wrong).
  void setCrawlPulse(float crawlPulse) { crawlPulse_ = crawlPulse; }

  // False until setDutyPerSpeed() has landed a real (nonzero) pair. An
  // uncalibrated Drive REFUSES to drive: tick() writes no duty at all,
  // rather than quietly running this robot's wheels on some other robot's
  // numbers. Same fail-closed posture RobotLoop's own `configured_` gate
  // already takes for motion commands.
  bool calibrated() const { return calibrated_; }

  // --- The WHEELS command lifecycle ---

  // Arm a bounded wheel command: velocity targets + an expiry deadline +
  // the id acked on completion (takeCompletion()). `duration` is REQUIRED
  // and positive -- a wheel command is always time-bounded, so a dead host
  // can never mean a runaway. Supersedes any command already armed; the
  // superseded one does NOT emit a completion event (the caller that
  // replaced it already knows, and RobotLoop acked its arrival).
  void command(float vLeft, float vRight, float duration, uint32_t moveId,
               uint32_t now);  // [mm/s] [mm/s] [ms] -- now [ms]

  // Halt now: zero the targets, disarm, and emit NO completion ack for the
  // discarded command (the ESTOP path -- the host asked for a stop, not for
  // a report that the thing it cancelled finished). Also the takeover path
  // RobotLoop uses when a MOVE hands motion to the planner.
  void estop();

  // True while an armed command is running -- i.e. while Drive, not the
  // planner, owns motion.
  bool owns() const { return commandActive_; }

  // One-shot completion event for a command that reached its deadline; the
  // loop acks it. False (and *moveId untouched) when there is none pending.
  bool takeCompletion(uint32_t* moveId);

  // --- The two-method contract ---

  // Convert the commanded wheel speeds to duty (per-wheel calibration ->
  // crawl shaping -> quiet-at-zero) and write the leaves. Takes the speeds
  // as parameters rather than reading its own targets: the loop hands it
  // whichever subsystem's targets the blackboard currently carries, so
  // there is one actuation path regardless of who decided the motion.
  void tick(float speedLeft, float speedRight);  // [mm/s] [mm/s] signed

  // Expire an armed command whose deadline has passed (latching the
  // completion event), then publish this subsystem's targets into
  // state.wheelLeft/Right.cmdVelocity -- but ONLY while Drive owns motion.
  // When the planner owns it, this is a no-op on the blackboard, so the
  // planner's own update() is left as the single writer. Must therefore run
  // AFTER Motion::Planner::update() in the cycle.
  void update(Types::RobotState& state, uint32_t now);  // [ms]

  // --- Motion::WheelSink (legacy boundary) ---
  // The velocity-sink interface Motion::MoveQueue drives. RobotLoop no
  // longer routes anything through it -- the live path is command()/tick()/
  // update() above -- but the interface is still implemented so a MoveQueue
  // -era harness keeps compiling. setDuty() stages targets with no deadline
  // and no ownership claim; nothing in the live loop calls either method.
  void setDuty(float left, float right) override;  // [mm/s] [mm/s] velocity targets
  void stop() override;                            // == estop()

  // Last-staged velocity targets (test observability; the blackboard's own
  // copy is written by update()).
  float targetLeft() const { return targetLeft_; }    // [mm/s] signed
  float targetRight() const { return targetRight_; }  // [mm/s] signed

  // trackWidth -- read-only accessor; fixed at construction, matching
  // Drive's own "no live-reconfigure" contract. See the constructor's own
  // doc comment for who reads it and why it lives here.
  float trackWidth() const { return trackWidth_; }  // [mm]

 private:
  // Crawl shaper (drive.cpp).
  float crawlDuty(float duty, float& carry) const;

  Devices::Motor& left_;
  Devices::Motor& right_;
  float trackWidth_;  // [mm]

  float targetLeft_ = 0.0f;   // [mm/s]
  float targetRight_ = 0.0f;  // [mm/s]

  // Armed bounded command (command() -> update() expiry).
  bool commandActive_ = false;
  uint32_t commandDeadline_ = 0;  // [ms]
  uint32_t commandMoveId_ = 0;
  bool completionPending_ = false;
  uint32_t completedMoveId_ = 0;

  // Open-loop duty per commanded speed, per wheel. NO DEFAULTS (§6): zero
  // until the composition root installs this robot's own measured pair via
  // setDutyPerSpeed(), and zero means uncalibrated, which means tick()
  // refuses to write. Baking a value here is what made one robot's
  // gearboxes every robot's -- see Config::DriveBootConfig's own doc
  // comment (config/boot_config.h) for the full history.
  float dutyPerSpeedLeft_ = 0.0f;   // [duty/(mm/s)]
  float dutyPerSpeedRight_ = 0.0f;  // [duty/(mm/s)]
  bool calibrated_ = false;

  float crawlPulse_ = 0.0f;  // [-1, 1] pulse amplitude; 0 = off
  float crawlCarryLeft_ = 0.0f;   // Bresenham accumulators
  float crawlCarryRight_ = 0.0f;

  // Last duty pair actually written (quiet-at-zero baseline).
  float writtenLeft_ = 0.0f;   // [-1, 1]
  float writtenRight_ = 0.0f;  // [-1, 1]
};

}  // namespace App
