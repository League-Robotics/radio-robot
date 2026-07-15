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
// Identity-mounting assumption: scriptPoseResponse() below packs this
// plant's own centre-frame (x, y, heading) DIRECTLY as the OTOS chip's raw
// register values, without any lever-arm (sensorToCentre()/
// centreToSensor()) or mounting-yaw inverse transform. This is only valid
// when the Devices::OtosConfig under test uses offsetX=offsetY=offsetYaw=0
// (identity mounting) -- plant_harness.cpp's scenarios all construct their
// Devices::Otos this way. A future scenario wanting a non-identity mount
// would need to invert Otos::tick()'s own transform here first.
#pragma once

#include <cstdint>

#include "devices/i2c_bus.h"

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

  float x() const { return x_; }              // [mm]
  float y() const { return y_; }              // [mm]
  float heading() const { return heading_; }  // [rad]

  // Schedules the 12-byte POSITION_XL+VELOCITY_XL burst-read response
  // Devices::Otos's NEXT tick() call will consume, from this plant's
  // CURRENT (x, y, heading) -- mirrors devices_otos_harness.cpp's
  // scriptPosVel() packing exactly (little-endian int16 sextuple, the same
  // kPosMmPerLsb/kHdgRadPerLsb scale factors otos.cpp itself uses).
  // Velocity registers are always scripted as zero -- no scenario in this
  // ticket asserts on OTOS's twist, only its pose.
  //
  // Like WheelPlant::scriptEncoderResponse(), this schedules exactly ONE
  // write (the register-address write) -- Devices::Otos::tick()'s burst
  // read is unconditionally a single write + single read, never a second,
  // "maybe" write the way a motor's duty write is. A caller composing this
  // alongside WheelPlant on the SAME Devices::I2CBus (one global write/read
  // FIFO per direction, shared across every device address -- i2c_bus.h's
  // file header) must still push this AFTER the wheel plants' own pushes
  // for the same cycle, in the same order Devices::Otos::tick() is called
  // relative to the two NezhaMotor::tick() calls, or the shared FIFO's
  // address matching desyncs.
  void scriptPoseResponse(Devices::I2CBus& bus, uint16_t wireAddr) const;

 private:
  float trackWidth_;         // [mm]
  float lastLeft_ = 0.0f;    // [mm]
  float lastRight_ = 0.0f;   // [mm]

  float x_ = 0.0f;          // [mm]
  float y_ = 0.0f;          // [mm]
  float heading_ = 0.0f;    // [rad]
};

}  // namespace TestSim
