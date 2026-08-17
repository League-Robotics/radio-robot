#pragma once

#include <cstdint>

namespace Types {

// Mirrors telemetry.proto's DriveMode value set (kept dependency-free of
// the generated message headers on purpose, per this header's own
// isolation rule -- see that enum's own doc comment for the 135-004
// rename of value 4 (GoTo -> Navigating, resolving a protoc enum-value-
// scope collision with commands.proto's new GO_TO verb; wire-compatible,
// since proto enums encode by number).
enum class Mode : uint8_t {
  Idle = 0,
  Streaming = 1,
  Timed = 2,
  Distance = 3,
  Navigating = 4,
  Velocity = 5,
};

struct RobotState {
  struct Time {
    uint32_t cycleStart = 0;  // [ms] this cycle's own start instant
    uint32_t cycleBusy = 0;  // [us] cycleStart -> frame-staging instant, THIS cycle
    uint32_t cyclePeriod = 0;  // [us] this cycleStart minus the previous cycle's cycleStart
  } time;

  // Wheel -- MEASURED state only, published once per cycle from
  // Control::DifferentialDrive::output() (the wheel kernel now owns both
  // motors on its own fiber; RobotLoop never touches them directly -- see
  // core/robot_loop.h's own header). position/velocity are in MILLIMETRES
  // here, converted per wheel by RobotLoop::publishWheels() from the
  // kernel's counts using the robot JSON's two travel calibrations -- the
  // same pair the inbound WHEELS adapter uses. The KERNEL is counts-native
  // end to end and never sees mm; this blackboard is on the application
  // side of that boundary, and it feeds the telemetry encoder reading,
  // whose wire unit is [mm]/[mm/s] and whose host-side readers
  // (protocol.py, the bench scripts) were never migrated. Publishing
  // counts here changes that field's meaning ~14x under an unchanged
  // frame shape -- see robot_loop.cpp's own kPositionWireBound comment.
  //
  // cmdVelocity/cmdAccel are GONE: the commanded target lives inside the
  // kernel's own Command mailbox, not here -- there is no cross-cycle
  // "what did we ask for" field on this side of the interface any more.
  struct Wheel {
    float position = 0.0f;  // [mm] from Control::DifferentialDrive::Output::positionLeft/Right
    float velocity = 0.0f;  // [mm/s] signed
    uint32_t sampleTime = 0;  // [ms] this reading's own genuine collect time
    bool connected = false;
    uint8_t positionEpoch = 0;  // bumped only when RobotLoop triggers drive_.rebasePosition()
  };
  Wheel wheelLeft;
  Wheel wheelRight;

  struct Otos {
    bool present = false;
    bool connected = false;
    float x = 0.0f;  // [mm]
    float y = 0.0f;  // [mm]
    float heading = 0.0f;  // [rad]
    float v_x = 0.0f;  // [mm/s] signed
    float v_y = 0.0f;  // [mm/s] signed
    float omega = 0.0f;  // [rad/s] signed
    uint32_t sampleTime = 0;  // [ms] this reading's own genuine collect time --
  } otos;

  struct Perception {
    // The pace block reads only ONE of these leaves per cycle (line on odd
    // cycles, colour on even), so on any given cycle exactly one reading is
    // new. Both stay published regardless: `*Valid` says a reading exists
    // and is worth sending, `*Fresh` says it was refreshed THIS cycle.
    // Splitting the two is what lets the untouched sensor go stale on the
    // wire rather than disappear from it -- a consumer that only wants
    // just-sampled data still has `*Fresh`, and one that wants the latest
    // known value (the common case) has it every frame.
    uint32_t line = 0;
    uint32_t color = 0;
    bool lineValid = false;   // a reading has been obtained; `line` is meaningful
    bool colorValid = false;  // a reading has been obtained; `color` is meaningful
    bool lineFresh = false;   // `line` was re-read on THIS cycle
    bool colorFresh = false;  // `color` was re-read on THIS cycle
  } perception;

  struct Pose {
    float x = 0.0f;  // [mm]
    float y = 0.0f;  // [mm]
    float heading = 0.0f;  // [rad]
    float v_x = 0.0f;  // [mm/s] body-frame, signed
    float v_y = 0.0f;  // [mm/s] body-frame, signed
    float omega = 0.0f;  // [rad/s] signed
  } pose;

  // Estimate (WheelEstimate/BodyEstimate/Innovations) -- DELETED. Fed the
  // now-long-gone Core::StateEstimator (deleted sprint 128 ticket 016);
  // nothing has written or read this block since -- confirmed by grep
  // before removing it (zero consumers anywhere in src/firm).

  // Command -- v_x/omega DELETED (nothing read them; the kernel's own
  // Command mailbox is the one place "what is currently commanded" lives
  // now, and it is not exposed on this blackboard). mode/moveActive stay:
  // Telemetry::update() and Comms::updateStatus() both still read them.
  struct Command {
    Mode mode = Mode::Idle;
    bool moveActive = false;
  } command;

  struct Health {
    uint32_t i2cSafetyNetCount = 0;
    uint32_t commsMalformedCount = 0;
    uint32_t commandsDroppedCount = 0;
    bool wedgeLatch = false;
    // moveTimeout/shapingDisabled -- DELETED. Both were written only by
    // Core::RobotLoop::publishMoveResult()/publishGotoResult(), which fed
    // Motion::Planner/Motion::Navigator fault state that no longer
    // exists; Telemetry::update() never read either field directly (the
    // wire flag bits it fed, kFlagFaultMoveTimeout/kFlagFaultShapingDisabled,
    // were set straight from RobotLoop via tlm_.setLiveFlag(), bypassing
    // this struct). Those two wire bits now simply stay clear -- a
    // documented narrowing of the exploratory tree, not a silent drop.
    bool positionClamped = false;
    bool wheelFrozenLeft = false;
    bool wheelFrozenRight = false;
    // Stall -- the drivetrain was asked to move and did not, so Core::RobotLoop
    // halted the robot. LATCHED, unlike every other flag in this struct: it
    // survives the halt that clears the condition, and is cleared only when
    // the host commands a new motion (MOVE/WHEELS/GO_TO) or ESTOPs. Without
    // the latch the halt erases its own evidence within one cycle and the
    // host never learns why the robot stopped.
    bool stallLeft = false;
    bool stallRight = false;
    bool ready = false;
  } health;
};

}
