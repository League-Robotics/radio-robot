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

  struct Wheel {
    float position = 0.0f;  // [mm] Devices::Motor::position()
    float velocity = 0.0f;  // [mm/s] signed, Devices::Motor::velocity()
    uint32_t sampleTime = 0;  // [ms] this reading's own genuine collect time --
    bool connected = false;
    uint8_t positionEpoch = 0;

    float cmdVelocity = 0.0f;  // [mm/s] signed, this cycle's commanded target for this wheel

    float cmdAccel = 0.0f;  // [mm/s^2] signed, this cycle's commanded accel for this wheel
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

  struct WheelEstimate {
    float distance = 0.0f;  // [mm] traveled distance at basisTime (matches Wheel::position)
    float velocity = 0.0f;  // [mm/s] signed, held constant across ZOH extrapolation
    uint32_t basisTime = 0;  // [ms]
    bool valid = false;
  };
  struct BodyEstimate {
    float x = 0.0f;  // [mm]
    float y = 0.0f;  // [mm]
    float heading = 0.0f;  // [rad] v1 complementary blend vs OTOS heading when fresh
    float v_x = 0.0f;  // [mm/s] body-frame, signed
    float v_y = 0.0f;  // [mm/s] body-frame, signed
    float omega = 0.0f;  // [rad/s] signed, v1 complementary blend vs OTOS omega when fresh
    uint32_t basisTime = 0;  // [ms]
    bool valid = false;
  };
  struct Innovations {
    float heading = 0.0f;  // [rad] OTOS heading minus predicted heading, at last blend
    float omega = 0.0f;  // [rad/s] OTOS omega minus predicted omega, at last blend
    bool valid = false;
  };
  struct Estimate {
    WheelEstimate wheelLeft;
    WheelEstimate wheelRight;
    BodyEstimate body;
    Innovations innovations;
  } estimate;

  struct Command {
    Mode mode = Mode::Idle;
    bool moveActive = false;
    float v_x = 0.0f;  // [mm/s] signed, current commanded body-frame forward velocity
    float omega = 0.0f;  // [rad/s] signed, current commanded yaw rate
  } command;

  struct Health {
    uint32_t i2cSafetyNetCount = 0;
    uint32_t commsMalformedCount = 0;
    uint32_t commandsDroppedCount = 0;
    bool wedgeLatch = false;
    bool moveTimeout = false;
    bool shapingDisabled = false;
    bool positionClamped = false;
    bool wheelFrozenLeft = false;
    bool wheelFrozenRight = false;
    // Stall -- the drivetrain was asked to move and did not, so App::RobotLoop
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
