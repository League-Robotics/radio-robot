// otos_plant.h -- TestSim::OtosPlant: a deterministic OTOS register
// responder deriving pose from the SAME two wheel positions the firmware's
// own App::Odometry integrates -- and nothing else.
//
// Ticket 105-003 (SUC-020), architecture-update.md Decision 3: "The plant
// carries no heading/angle-wrap state or logic of its own; all heading
// comes from App::Odometry's existing integration." This class's own
// accumulator below is the SAME midpoint-arc update Odometry::integrate()
// (source/app/odometry.cpp) already performs -- literally the same three
// lines (BodyKinematics::forward() + cosf/sinf midpoint-arc accumulation),
// duplicated here per this codebase's established per-file fixture-
// duplication convention, NOT a second, independently-derived heading
// formula. This is the one and only heading integration this file
// performs; no arctangent call and no explicit angle-wrap/normalize helper
// of any kind appear anywhere in this file or wheel_plant.h (part of this
// ticket's own self-check -- see plant_harness.cpp's neighboring
// test_plant_files_carry_no_heading_wrap_logic() for the exact pattern it
// scans for).
//
// NOT PORTED FROM THE DELETED SIM: see wheel_plant.h's file header for the
// full carried-caution rationale (the deleted `drive/` v2 sim plant's own
// 180/360-degree pivot heading bug). No formula in this file originates
// there.
//
// Identity-mounting assumption: a caller packing this plant's own
// centre-frame (x, y, heading) directly as the OTOS chip's raw register
// values (TestSim::SimPlant does exactly this) does so WITHOUT any
// lever-arm (sensorToCentre()/centreToSensor()) or mounting-yaw inverse
// transform. This is only valid when the Devices::OtosConfig under test
// uses offsetX=offsetY=offsetYaw=0 (identity mounting). A future scenario
// wanting a non-identity mount would need to invert Otos::tick()'s own
// transform first.
#pragma once

namespace TestSim {

class OtosPlant {
 public:
  // trackWidth: [mm] BodyKinematics::forward()'s own `b` parameter. MUST
  // match the trackWidth passed to the App::Odometry instance under test in
  // the SAME scenario, so this plant's own OTOS pose and Odometry's
  // independently-integrated pose describe the same physical wheelbase
  // (architecture-update.md Decision 3's own "will always agree closely --
  // by design" consequence).
  explicit OtosPlant(float trackWidth);

  // Advances this plant's own (x, y, heading) accumulator by exactly one
  // cycle's wheel-position delta, via BodyKinematics::forward() -- the SAME
  // function App::Odometry itself calls, over the SAME two absolute wheel
  // positions the caller's two WheelPlant instances just computed. Call
  // once per cycle, after both WheelPlant::step() calls for that cycle.
  void step(float leftPosition, float rightPosition);   // [mm] [mm]

  // Reported pose == the true accumulator (x_/y_/heading_) plus the
  // deterministic drift/bias knobs below. Kept separate from x()/y()/
  // heading() (the TRUE pose) so a future true-pose export (ticket 003's
  // SimHarness) can still see ground truth even while a fault scenario has
  // biased what the OTOS chip itself would report.
  float reportedX() const { return x_ + driftX_; }              // [mm]
  float reportedY() const { return y_ + driftY_; }              // [mm]
  float reportedHeading() const { return heading_ + driftHeading_; }  // [rad]

  float x() const { return x_; }              // [mm]
  float y() const { return y_; }              // [mm]
  float heading() const { return heading_; }  // [rad]

  // Deterministic OTOS drift/bias knob (105-005's WheelPlant fault knobs'
  // sibling for this plant -- see wheel_plant.h's own "seeded" doc: no RNG
  // anywhere in either plant, so "noise" here means a fixed, reproducible
  // bias, not a random jitter). Sets a CONSTANT offset added on top of the
  // true accumulated pose for every reportedX()/reportedY()/
  // reportedHeading() call from this point on -- models a persistent
  // sensor bias/drift (e.g. an uncorrected mounting error or slow Kalman
  // drift), not per-cycle random noise. 0/0/0 (the default) disables the
  // knob -- reportedX/Y/Heading() then equal x()/y()/heading() exactly.
  void setDrift(float xDrift, float yDrift, float headingDrift);  // [mm] [mm] [rad]

  // Plant teleport (sim command-surface fix, host TestGUI Sim "reset to
  // origin"/SI support): snaps the accumulator directly to (x, y, heading)
  // -- bypassing step()'s own incremental wheel-delta integration entirely
  // -- and re-baselines lastLeft_/lastRight_ to 0. This is only correct
  // when the caller (TestSim::SimPlant::setTruePose()) ALSO resets both
  // WheelPlant positions to 0 in the same call: step()'s next call computes
  // this cycle's delta as `wheelPosition - lastLeft_/lastRight_`, so lastLeft_/
  // lastRight_ must match whatever position the just-reset wheels report,
  // or the very next step() would inject a phantom one-cycle jump sized by
  // the (position_before_reset - 0) gap. Drift/bias knob state
  // (driftX_/driftY_/driftHeading_) is left untouched -- a pose reset is
  // not a fault-knob reset.
  void reset(float x, float y, float heading);  // [mm] [mm] [rad]

  // Formerly this class also had scriptPoseResponse(Devices::I2CBus&,
  // uint16_t) const, which packed a 12-byte POSITION_XL+VELOCITY_XL burst
  // (little-endian int16 sextuple, kPosMmPerLsb/kHdgRadPerLsb-scaled) onto
  // the scripted-FIFO Devices::I2CBus fake sprint 108 ticket 001 deleted.
  // That packing is now done directly by TestSim::SimPlant
  // (tests/_infra/sim/sim_plant.cpp's handleOtosRead()) straight off this
  // class's own x()/y()/heading() accessors -- no bus/wire-format
  // knowledge belongs on this class (architecture-update.md Decision 3).

 private:
  float trackWidth_;         // [mm]
  float lastLeft_ = 0.0f;    // [mm]
  float lastRight_ = 0.0f;   // [mm]

  float x_ = 0.0f;          // [mm]
  float y_ = 0.0f;          // [mm]
  float heading_ = 0.0f;    // [rad]

  // ---- Drift/bias knob state ----
  float driftX_ = 0.0f;        // [mm]
  float driftY_ = 0.0f;        // [mm]
  float driftHeading_ = 0.0f;  // [rad]
};

}  // namespace TestSim
