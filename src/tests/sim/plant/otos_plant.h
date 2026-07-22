// otos_plant.h -- TestSim::OtosPlant: a deterministic OTOS register
// responder deriving pose from the SAME two wheel positions the firmware's
// own App::Odometry integrates -- and nothing else.
//
// Ticket 105-003 (SUC-020), architecture-update.md Decision 3: "The plant
// carries no heading/angle-wrap state or logic of its own; all heading
// comes from App::Odometry's existing integration." This class's own
// accumulator below is the SAME midpoint-arc update Odometry::integrate()
// (src/firm/app/odometry.cpp) already performs -- literally the same three
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

#include <cstdint>

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
  //
  // dt (109-010): this cycle's own elapsed time ([s]) -- used to derive
  // `omega()`/`v_x()`/`v_y()` below, each a plain finite-difference rate
  // estimate of THIS cycle's own motion, mirroring a real OTOS chip's own
  // VELOCITY_XL register: an instantaneous linear+angular-rate report
  // alongside the position/heading burst. Defaulted to 0 so no
  // pre-109-010 test caller needs to change -- `dt<=0` reports
  // `omega()==0`/`v_x()==0`/`v_y()==0` (the pre-109-010 behavior:
  // sim_plant.cpp's own handleOtosRead() zeroed the VELOCITY_XL bytes
  // unconditionally, "no scenario asserts on OTOS's twist" -- ticket 010
  // is the first scenario that does, via App::HeadingSource's own
  // measurement-age projection needing a real `omega_meas`).
  void step(float leftPosition, float rightPosition, float dt = 0.0f);   // [mm] [mm] [s]

  // omega -- this plant's own finite-difference angular-rate estimate from
  // the MOST RECENT step() call (see that method's own `dt` doc comment).
  // 0.0f before the first step() with a nonzero dt.
  float omega() const { return omega_; }  // [rad/s]

  // v_x/v_y -- this plant's own finite-difference LINEAR-velocity estimate
  // from the MOST RECENT step() call (115-006, gut S1 optional stretch:
  // Otos::pose()'s v_x/v_y previously always rode the wire as 0 -- see
  // sim_plant.cpp's own handleOtosRead() comment history). v_x is this
  // cycle's own `distance / dt` (the SAME body-forward `distance`
  // BodyKinematics::forward() already computed inside step(), before it was
  // consumed by the midpoint-arc position update) -- a body-FRAME forward
  // velocity, matching the real chip's own linear-velocity report
  // convention (mounting-yaw-corrected only, not heading-rotated -- see
  // Devices::Otos::tick()'s own rotVx/rotVy comment). v_y is always 0.0f:
  // this plant, like the firmware's own encoder-only Odometry, has no
  // lateral-slip model for a differential-drive robot -- there is no
  // sideways component to report, matching the primary telemetry frame's
  // own twist.v_y (BodyKinematics::forward() never produces one either).
  // 0.0f before the first step() with a nonzero dt.
  float v_x() const { return v_x_; }  // [mm/s]
  float v_y() const { return v_y_; }  // [mm/s]

  // Reported pose == the true accumulator (x_/y_/heading_) plus the
  // deterministic drift/bias knobs below. Kept separate from x()/y()/
  // heading() (the TRUE pose) so a future true-pose export (ticket 003's
  // SimHarness) can still see ground truth even while a fault scenario has
  // biased what the OTOS chip itself would report.
  float reportedX() const { return x_ * linearFactor() + driftX_; }              // [mm]
  float reportedY() const { return y_ * linearFactor() + driftY_; }              // [mm]
  float reportedHeading() const { return heading_ * angularFactor() + driftHeading_; }  // [rad]

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

  // Raw OTOS scale error (109-007, sim-honors-otos-calibration.md): models
  // a physically MIS-CALIBRATED chip -- a fractional over/under-report on
  // the true accumulated pose (0=perfect, mirrors WheelPlant::setScaleErr()'s
  // own "fractional over/under-report" vocabulary), applied BEFORE the
  // chip's own calibration-scalar register correction below. This is
  // exactly the stakeholder's own framing (issue, 2026-07-16): "if you are
  // simulating the OTOS / the I2C bus, you should be simulating
  // calibrations" -- SimPlant's OTOS burst-read response becomes
  // `truth * (1+linearFraction)` / `truth * (1+angularFraction)`. 0.0/0.0
  // (the default) is a genuine no-op.
  void setRawScaleErr(float linearFraction, float angularFraction);  // [fractional over/under-report, 0=perfect]
  float rawScaleErrLinear() const { return rawErrorLinear_; }
  float rawScaleErrAngular() const { return rawErrorAngular_; }

  // Chip-internal calibration-scalar register honoring (109-007): mirrors
  // the REAL SparkFun OTOS chip's own documented behavior -- its
  // REG_SCALAR_LINEAR/REG_SCALAR_ANGULAR registers multiply the chip's raw
  // measurement by (1 + reg*0.001) before it is ever reported on the wire
  // (the exact inverse of Devices::Otos::scaleToRegister()'s own
  // scale-to-register conversion). TestSim::SimPlant's handleOtosWrite()
  // captures a firmware write to either register (via the REAL
  // Devices::Otos::setLinearScalar()/setAngularScalar() -- the same OL/OA
  // wire path a live OtosConfigPatch or the OL/OA text verb drives) and
  // calls these setters -- see sim_plant.cpp's own comment. 0 (the
  // default -- an un-calibrated/just-reset chip) is a genuine no-op
  // (multiplier 1.0): net effect is `truth` exactly when
  // rawScaleErr==0 AND the register is 0, `truth * rawError` when a scale
  // error is injected but nothing has calibrated it out yet, and `truth`
  // again once the correct compensating register value is written --
  // SUC-005's "diverges, then converges" contract.
  void setLinearScalarReg(int8_t reg) { linearScalarReg_ = reg; }
  void setAngularScalarReg(int8_t reg) { angularScalarReg_ = reg; }
  int8_t linearScalarReg() const { return linearScalarReg_; }
  int8_t angularScalarReg() const { return angularScalarReg_; }

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
  // Snap this plant's (x, y, heading) truth and re-anchor its wheel-delta
  // baseline to (baseLeft, baseRight). Callers that ALSO zero the wheel
  // plants pass the default 0/0; a caller that keeps the wheel plants
  // continuous (SimPlant::setTruePose, to avoid a firmware encoder
  // discontinuity) passes the wheels' CURRENT positions so the next step()
  // integrates a zero delta.
  void reset(float x, float y, float heading,          // [mm] [mm] [rad]
             float baseLeft = 0.0f, float baseRight = 0.0f);  // [mm] [mm]

  // Formerly this class also had scriptPoseResponse(Devices::I2CBus&,
  // uint16_t) const, which packed a 12-byte POSITION_XL+VELOCITY_XL burst
  // (little-endian int16 sextuple, kPosMmPerLsb/kHdgRadPerLsb-scaled) onto
  // the scripted-FIFO Devices::I2CBus fake sprint 108 ticket 001 deleted.
  // That packing is now done directly by TestSim::SimPlant
  // (tests/_infra/sim/sim_plant.cpp's handleOtosRead()) straight off this
  // class's own x()/y()/heading() accessors -- no bus/wire-format
  // knowledge belongs on this class (architecture-update.md Decision 3).

 private:
  // linearFactor()/angularFactor() (109-007): the combined
  // raw-scale-error * calibration-register multiplier reportedX/Y/Heading()
  // apply. Default (rawErrorLinear_=0, linearScalarReg_=0) collapses to
  // exactly 1.0 -- the pre-109-007 no-op behavior (reportedX()==x_+driftX_)
  // is bit-for-bit preserved when neither knob is touched.
  float linearFactor() const {
    return (1.0f + rawErrorLinear_) * (1.0f + static_cast<float>(linearScalarReg_) * 0.001f);
  }
  float angularFactor() const {
    return (1.0f + rawErrorAngular_) * (1.0f + static_cast<float>(angularScalarReg_) * 0.001f);
  }

  float trackWidth_;         // [mm]
  float lastLeft_ = 0.0f;    // [mm]
  float lastRight_ = 0.0f;   // [mm]

  float x_ = 0.0f;          // [mm]
  float y_ = 0.0f;          // [mm]
  float heading_ = 0.0f;    // [rad]
  float omega_ = 0.0f;      // [rad/s] 109-010, see step()'s own `dt` doc comment
  float v_x_ = 0.0f;        // [mm/s] 115-006, see v_x()'s own doc comment
  float v_y_ = 0.0f;        // [mm/s] 115-006, see v_y()'s own doc comment -- always 0.0f

  // ---- Drift/bias knob state ----
  float driftX_ = 0.0f;        // [mm]
  float driftY_ = 0.0f;        // [mm]
  float driftHeading_ = 0.0f;  // [rad]

  // ---- Raw scale error + calibration-register state (109-007) ----
  float rawErrorLinear_ = 0.0f;    // [fractional over/under-report, 0=perfect]
  float rawErrorAngular_ = 0.0f;   // [fractional over/under-report, 0=perfect]
  int8_t linearScalarReg_ = 0;     // chip register raw value, see setLinearScalarReg()
  int8_t angularScalarReg_ = 0;    // chip register raw value, see setAngularScalarReg()
};

}  // namespace TestSim
