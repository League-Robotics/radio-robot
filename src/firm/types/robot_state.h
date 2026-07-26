// robot_state.h -- Types::RobotState: the one dependency-free blackboard
// struct that is the sole cross-subsystem AND cross-tree data contract
// (sprint 124, clasi/issues/robot-state-blackboard-one-struct-for-all-
// shared-state-and-telemetry.md; sprint.md Step 3's "Firm::Types::
// RobotState" entry). Three roles, one object: the blackboard subsystems
// publish their own section of and read each other's sections from
// (killing multi-argument tick/update signatures); the source
// App::Telemetry projects into the wire frame (App::Telemetry::update()'s
// job, ticket 008/009 -- NOT this file); and a test fixture (construct one,
// fill the fields the module under test reads, call tick(state, now),
// assert on the fields it writes -- trivially copyable, so tests copy it
// for golden comparisons).
//
// Dependency-free (the whole point): cstdint-level includes ONLY. No
// generated wire-message types (the `msg` namespace, `src/firm's generated
// message headers), no protobuf-generated anything, nothing from
// `src/firm/app`, no methods beyond trivial default member initializers.
// This is what lets `src/motion` -- a sibling tree that must build and run
// independently of `src/firm` -- include this header too: `firm/types`
// becomes a second shared floor both trees stand on, alongside
// `Motion::WheelSink` (`src/motion/wheel_sink.h`, the actuation crossing).
// `Motion::StateEstimator` is the first real consumer on the motion side
// (state_estimator.h now `#include`s this file in place of its own former
// private `Input` struct -- see that file's own header for the crossing's
// rationale).
//
// Float-typed, throughout: RobotState holds values in the units the
// odometry/estimator/PID actually compute in (mm, mm/s, rad, rad/s, ...).
// It is the truth. The generated wire Telemetry type holds scaled
// fixed-point integers and is a LOSSY PROJECTION of a subset of this
// struct (App::Telemetry's job) --
// the frame is not the state, and conflating the two was the bug this
// struct exists to kill. See coding-standards.md for the unit-tag
// convention every field below follows (`// [unit]`, leading token of the
// trailing comment; dimensionless/bool fields carry no tag).
//
// Sections are grouped by WRITER (per the issue's own struct sketch), each
// documented with which subsystem publishes it and when -- see sprint.md's
// "Publish rule: as soon as coherent, exactly once per cycle." This ticket
// (124-007) defines the struct only; wiring RobotLoop to actually build and
// publish every section, and Telemetry to project it, is tickets 008/009 --
// this header makes no claim about who currently populates any field.
//
// What stays OUT (issue's own "What stays OUT of RobotState" section):
// config, PID gains, calibration, fusion weights, persisted tuning, and
// ACKS (protocol bookkeeping, not robot state) -- RobotState is per-cycle
// dynamics only. Config keeps its own patch/persistence path
// (Config::TuningSnapshot and friends); acks stay inside App::Telemetry's
// own ack ring.
//
// Trivially copyable: every field below is a plain scalar, bool, or a
// plain nested struct of the same -- no pointers, no heap, no virtuals, no
// user-defined constructors/destructors. Verified by a
// static_assert(std::is_trivially_copyable_v<...>) in this ticket's own
// test (src/tests/sim/unit/firm_types_robot_state_harness.cpp) rather than
// in this header itself, to keep the header's own include list at
// <cstdint> and nothing else (a `static_assert` needs <type_traits>, which
// is fine for a caller to pull in, but not a reason to widen this file's
// own dependency floor).
#pragma once

#include <cstdint>

namespace Types {

// Mode -- mirrors the generated wire DriveMode enum's (telemetry.h, in
// src/firm's generated message headers) value set WITHOUT depending on
// it -- this header may never include anything from that generated tree
// (see file header above). Project-owned scoped enum, UpperCamelCase
// enumerators (naming-and-style.md), unlike the wire type's ALL_CAPS
// enumerators (the generated DriveMode is wire-exempt). App::Telemetry's
// projection step (ticket 008/009) is the one place these two enums are
// ever converted between.
enum class Mode : uint8_t {
  Idle = 0,
  Streaming = 1,
  Timed = 2,
  Distance = 3,
  GoTo = 4,
  Velocity = 5,
};

// RobotState -- see file header for the full contract. Nested nested
// section structs are named for what they hold (Wheel, Otos, Pose, ...),
// never for who writes them -- the writer is documented in each section's
// own comment, not encoded in the name (a section's writer can change,
// e.g. Decision 1's deferred Drive/Sensors ownership reshuffle, sprint 125,
// without forcing a field rename here).
struct RobotState {
  // --- Time --- writer: App::RobotLoop::cycle(), once per cycle, at the
  // top (today's cycleStart local var; MOVED here rather than threaded
  // through every downstream call by parameter, ticket 009's job).
  struct Time {
    uint32_t cycleStart = 0;   // [ms] this cycle's own start instant
    uint32_t cycleBusy = 0;    // [us] cycleStart -> frame-staging instant, THIS cycle
    uint32_t cyclePeriod = 0;  // [us] this cycleStart minus the previous cycle's cycleStart
  } time;

  // --- Wheel (left/right) --- sensed fields (position/velocity/sampleTime/
  // connected) writer: today RobotLoop reads Devices::Motor directly each
  // cycle (Decision 1 -- device ownership stays on RobotLoop this sprint);
  // sprint 125's Drive::update() takes over once the ownership reshuffle
  // lands. cmdVelocity writer: App::Drive::tick()'s own staged target.
  // positionEpoch writer: App::RobotLoop's new per-cycle rebaseline trigger
  // (architecture Decision 6, ticket 008) -- increments each time
  // RobotLoop calls the wheel's existing Devices::Motor::rebaseline() as
  // its raw position nears the wire's sint32/zigzag bound; NOT touched by
  // Devices::Motor/NezhaMotor/MotorArmor themselves (unmodified by this
  // sprint, per sprint.md's own Success Criteria).
  struct Wheel {
    float position = 0.0f;      // [mm] Devices::Motor::position()
    float velocity = 0.0f;      // [mm/s] signed, Devices::Motor::velocity()
    uint32_t sampleTime = 0;    // [ms] cycle-domain capture time of this reading (matches
                                 // the existing generated EncoderReading.time convention --
                                 // the owning cycle's own cycleStart, NOT Devices::Motor's
                                 // raw sampleTime() [us] device-clock accessor value directly)
    bool connected = false;     // Devices::Motor::connected()
    uint8_t positionEpoch = 0;  // wraps; +1 each RobotLoop-triggered rebaseline() (Decision 6)
    float cmdVelocity = 0.0f;   // [mm/s] signed, App::Drive's last-staged target for this wheel
  };
  Wheel wheelLeft;
  Wheel wheelRight;

  // --- Otos --- writer: today RobotLoop's own OTOS-tick block; sprint
  // 125's Sensors subsystem takes over. `present` mirrors
  // App::applyOtosSample()'s existing "chip detected AND this cycle's
  // burst actually refreshed the cached pose" contract (the generated
  // wire Telemetry type's flags bit 0 source); the x/y/heading/v_x/v_y/
  // omega/sampleTime fields
  // below are valid iff `present` is true THIS cycle.
  struct Otos {
    bool present = false;     // fresh THIS cycle -- fields below valid iff true
    bool connected = false;   // live OTOS bus health, independent of `present`
    float x = 0.0f;           // [mm]
    float y = 0.0f;           // [mm]
    float heading = 0.0f;     // [rad]
    float v_x = 0.0f;         // [mm/s] signed
    float v_y = 0.0f;         // [mm/s] signed
    float omega = 0.0f;       // [rad/s] signed
    uint32_t sampleTime = 0;  // [ms] cycle-domain capture time of this reading
  } otos;

  // --- Perception --- writer: today RobotLoop's line/color alternation
  // cursor (exactly one of {line, color} ticks a given cycle); sprint 125's
  // Sensors subsystem takes over the cursor itself, unchanged semantics.
  // `line`/`color` hold the packed wire-layout words (RobotLoop's existing
  // packLine()/packColor() helpers) already, since App::Telemetry's own
  // projection step needs no further transformation of these two --
  // they're opaque payloads to every RobotState reader except Telemetry.
  struct Perception {
    uint32_t line = 0;    // packed 4-channel raw grayscale, ch1 in the low byte
    uint32_t color = 0;   // packed RGBC (8 bits each), R in the low byte
    bool lineFresh = false;   // `line` refreshed THIS cycle
    bool colorFresh = false;  // `color` refreshed THIS cycle
  } perception;

  // --- Pose --- writer: Motion::Odometry::integrate() (dead-reckoned
  // world pose) plus the same-cycle fused body-frame twist
  // (BodyKinematics::forward() over both wheels' current velocities,
  // today computed inline in RobotLoop's own kPace block). Encoder-only --
  // never OTOS-blended (that blend lives one level up, in `estimate`
  // below).
  struct Pose {
    float x = 0.0f;        // [mm]
    float y = 0.0f;        // [mm]
    float heading = 0.0f;  // [rad]
    float v_x = 0.0f;      // [mm/s] body-frame, signed
    float v_y = 0.0f;      // [mm/s] body-frame, signed
    float omega = 0.0f;    // [rad/s] signed
  } pose;

  // --- Estimate --- writer: Motion::StateEstimator::update(). ZOH BASES,
  // not snapshots: each nested estimate carries value(s) + velocity +
  // basisTime + valid, so "predict to time t" is a pure free function over
  // just this section (Motion::StateEstimator::wheelAt()/bodyAt() today;
  // a caller holding a COPIED RobotState gets the same extrapolation for
  // free, no live StateEstimator instance needed). Mirrors
  // Motion::WheelEstimate/BodyEstimate/Innovations field-for-field --
  // those become the CANONICAL shape once ticket 009 threads this section
  // through in place of StateEstimator's own private members; today they
  // remain two independently-valid copies (this ticket lands the type,
  // not the wiring).
  struct WheelEstimate {
    float distance = 0.0f;   // [mm] traveled distance at basisTime (matches Wheel::position)
    float velocity = 0.0f;   // [mm/s] signed, held constant across ZOH extrapolation
    uint32_t basisTime = 0;  // [ms]
    bool valid = false;      // false until this peer's first update()
  };
  struct BodyEstimate {
    float x = 0.0f;          // [mm]
    float y = 0.0f;          // [mm]
    float heading = 0.0f;    // [rad] v1 complementary blend vs OTOS heading when fresh
    float v_x = 0.0f;        // [mm/s] body-frame, signed
    float v_y = 0.0f;        // [mm/s] body-frame, signed
    float omega = 0.0f;      // [rad/s] signed, v1 complementary blend vs OTOS omega when fresh
    uint32_t basisTime = 0;  // [ms]
    bool valid = false;      // false until this peer's first update()
  };
  struct Innovations {
    float heading = 0.0f;  // [rad] OTOS heading minus predicted heading, at last blend
    float omega = 0.0f;    // [rad/s] OTOS omega minus predicted omega, at last blend
    bool valid = false;    // false until a fresh OTOS reading has been blended at least once
  };
  struct Estimate {
    WheelEstimate wheelLeft;
    WheelEstimate wheelRight;
    BodyEstimate body;
    Innovations innovations;
  } estimate;

  // --- Command --- writer: App::RobotLoop::processMessage()'s dispatch
  // (mode/moveActive, mid-cycle, deliberately -- sprint.md's own "Command
  // dispatch writes state.command mid-cycle" note) and
  // Motion::MoveQueue's per-cycle tick (v_x/omega, the currently-commanded
  // body-frame twist target). v_x/omega, not a bare `v` (naming-and-style.md
  // rule 2 -- a twist is never directionless) -- named identically to
  // Pose::v_x/BodyEstimate::v_x rather than `targetVx` (avoids mashing a
  // "target" prefix onto a mathematical subscript); the `command.` section
  // prefix at every call site is what disambiguates "commanded" from
  // "sensed"/"estimated," the same way wheelLeft.velocity vs.
  // wheelLeft.cmdVelocity are already disambiguated by field name within
  // one section rather than by a redundant prefix.
  struct Command {
    Mode mode = Mode::Idle;
    bool moveActive = false;  // Motion::MoveQueue::active()
    float v_x = 0.0f;         // [mm/s] signed, current commanded body-frame forward velocity
    float omega = 0.0f;       // [rad/s] signed, current commanded yaw rate
  } command;

  // --- Health --- writer: the owning module for each signal (I2CBus,
  // Comms, the two Motor leaves' wedge latch, MoveQueue). Per-cycle
  // dynamics, like every other section -- NOT a place for a Deadman-style
  // signal: App::Deadman was fully retired in an earlier sprint (its
  // former telemetry flag, kFlagEventDeadmanExpired, is declared but
  // unwired -- see telemetry.h's own comment), so there is no live
  // "deadman expired" signal today to carry here; the issue's own struct
  // sketch names one, but this ticket derives the actual field list from
  // what genuinely exists, per this ticket's own instructions, and adds
  // moveTimeout/shapingDisabled instead -- both ARE genuinely live
  // (App::RobotLoop's own kFlagFaultMoveTimeout/kFlagFaultShapingDisabled
  // derivation, set every cycle straight from Motion::MoveQueue::tick()'s
  // outcome and Motion::MoveQueue::shapingDisabled()).
  struct Health {
    uint32_t i2cSafetyNetCount = 0;    // Devices::I2CBus::clearanceSafetyNetCount()
    uint32_t commsMalformedCount = 0;  // App::Comms::malformedCount()
    bool wedgeLatch = false;           // either bound motor's Devices::Motor::wedged()
    bool moveTimeout = false;          // MOVE timeout backstop fired THIS cycle
    bool shapingDisabled = false;      // active MOVE, both ShaperLimits axes disabled
  } health;
};

}  // namespace Types
