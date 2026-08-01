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
// `RobotState::Wheel::cmdVelocity` itself (this file, below), THE
// documented base<->motion actuation boundary (sprint 128 ticket 014 --
// cmdVelocity's own field comment carries the deleted-interface history
// this crossing used to go through). `Motion::Planner` (planner.h) is the
// live consumer on the motion side today, `#include`-ing this file
// directly. `Motion::StateEstimator` was an earlier real consumer on this
// same crossing (state_estimator.h used to `#include` this file in place
// of its own former private `Input` struct) -- deleted sprint 128 ticket
// 016 (robot-state-pose-needs-exactly-one-writer.md) as a per-cycle
// computation with no consumer.
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
  // lands. cmdVelocity writer: see cmdVelocity's own field comment below --
  // it is THE base<->motion actuation boundary, not just another sensed
  // field. positionEpoch writer: App::RobotLoop's new per-cycle rebaseline trigger
  // (architecture Decision 6, ticket 008) -- increments each time
  // RobotLoop calls the wheel's existing Devices::Motor::rebaseline() as
  // its raw position nears the wire's sint32/zigzag bound; NOT touched by
  // Devices::Motor/NezhaMotor/MotorArmor themselves (unmodified by this
  // sprint, per sprint.md's own Success Criteria).
  struct Wheel {
    float position = 0.0f;      // [mm] Devices::Motor::position()
    float velocity = 0.0f;      // [mm/s] signed, Devices::Motor::velocity()
    uint32_t sampleTime = 0;    // [ms] this reading's own genuine collect time (ticket 009,
                                 // issue §B2): Devices::Motor::sampleTime() [us], converted to
                                 // the cycle-domain [ms] EncoderReading.age's projection needs --
                                 // NOT the owning cycle's own cycleStart (that was ticket 008's
                                 // interim stand-in, an honest-zero placeholder pending this
                                 // wiring). Left and right genuinely differ (the brick holds one
                                 // pending read, so the two collects are ~kSettle+kClear apart)
                                 // -- this field is where that skew is preserved for
                                 // App::Telemetry::update()'s age = now - sampleTime projection.
    bool connected = false;     // Devices::Motor::connected()
    uint8_t positionEpoch = 0;  // wraps; +1 each RobotLoop-triggered rebaseline() (Decision 6)

    // cmdVelocity -- THE documented actuation boundary between src/firm and
    // src/motion (sprint 128 ticket 014, SUC-002 Decision 1, stakeholder-
    // settled plan of record). Whichever subsystem currently owns motion
    // writes this cycle's commanded wheel speed here; RobotLoop::cycle()
    // reads it back and hands it to the hardware -- no interface required,
    // just this field.
    //   - Writer: sole-or-arbitrated. Motion::Planner::update()
    //     (planner.cpp) writes it when a Move owns motion; App::Drive::
    //     update() (drive.cpp) writes it for WHEELS teleop, but ONLY while
    //     Drive owns motion (owns() true) -- when the planner owns motion,
    //     Drive::update() is a no-op on this field, leaving the planner's
    //     write as the cycle's only one. RobotLoop::publishWheels() never
    //     touches it (see that function's own doc comment) -- exclusivity
    //     is enforced by ORDERING (exactly one of the two update() calls
    //     actually writes, per cycle), not by a shared/locked resource.
    //     RobotLoop::handleEstop() also zeroes it directly, belt-and-
    //     suspenders, so an emergency stop does not depend on the rest of
    //     the cycle's schedule running to completion to take effect.
    //   - Consumer: RobotLoop::cycle() (robot_loop.cpp), which hands both
    //     wheels' cmdVelocity straight to drive_.tick() for actuation.
    //   - History: `Motion::WheelSink` (formerly src/motion/wheel_sink.h)
    //     was the documented interface for this same crossing before this
    //     ticket. It had zero callers -- Motion::Planner::update() already
    //     wrote this field directly, and RobotLoop::cycle() already read
    //     it directly -- so it was deleted outright rather than kept as an
    //     unused seam. See docs/design/design.md §5 and
    //     src/motion/DESIGN.md for the current architecture.
    float cmdVelocity = 0.0f;  // [mm/s] signed, this cycle's commanded target for this wheel
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
    uint32_t sampleTime = 0;  // [ms] this reading's own genuine collect time (ticket 009,
                               // issue §B2): Devices::Otos::sampleTime() [us], converted to
                               // [ms] -- same rationale as Wheel::sampleTime above.
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
  // below). Odometry is the SOLE writer -- no other subsystem may assign
  // into this section.
  //   - History (128-016, robot-state-pose-needs-exactly-one-writer.md):
  //     Motion::Planner::update() used to ALSO write this section, from
  //     its own internal `PoseTracker` (`pose_`, still live as the
  //     planner's own working estimate, just no longer pushed here) --
  //     OTOS-blended whenever the planner's `headingOtosWeight` config
  //     was nonzero, ordering-dependent on which of Odometry/Planner ran
  //     last a given cycle within RobotLoop::cycle()'s kPace block. The
  //     "never OTOS-blended" claim above was therefore aspirational, not
  //     actually true, until that second write was deleted. If a future
  //     caller needs PoseTracker's own OTOS-fused output on the wire, it
  //     belongs in `estimate` below (Motion::StateEstimator's own former
  //     home, now vacant -- see that section's own doc comment), never
  //     by writing here again.
  struct Pose {
    float x = 0.0f;        // [mm]
    float y = 0.0f;        // [mm]
    float heading = 0.0f;  // [rad]
    float v_x = 0.0f;      // [mm/s] body-frame, signed
    float v_y = 0.0f;      // [mm/s] body-frame, signed
    float omega = 0.0f;    // [rad/s] signed
  } pose;

  // --- Estimate --- writer: Motion::Planner::update() (its own
  // WheelChannel/PoseTracker basis readings, written verbatim into this
  // section every tick -- planner.cpp's own state.estimate.* assignments
  // at the end of update()). ZOH BASES, not snapshots: each nested
  // estimate carries value(s) + velocity + basisTime + valid, so "predict
  // to time t" is a pure free function over just this section. Mirrors
  // Motion::WheelPeer/BodyPeer/Innovations field-for-field (125-002:
  // Motion::StateEstimator's own former private peer-basis structs were
  // renamed WheelEstimate -> WheelPeer / BodyEstimate -> BodyPeer to free
  // the `WheelEstimate` name for a retooled actuation-observer boundary
  // struct that briefly lived in a since-deleted motion-boundary
  // interface (sprint 128 ticket 014 -- see cmdVelocity's own field
  // comment above for that deletion's history) -- a DIFFERENT concept
  // from this ZOH peer-basis reading; the name is simply free again
  // today, not reserved for anything).
  //   - History (128-016, robot-state-pose-needs-exactly-one-writer.md):
  //     this section's doc comment used to attribute the write to
  //     Motion::StateEstimator::update() -- ticket 009's own original
  //     plan for this section, which never actually landed that way:
  //     StateEstimator::update() took RobotState by const reference
  //     (read-only input) and held its own PRIVATE peer basis, never
  //     writing back into this section at all -- Motion::Planner::
  //     update() was the section's real writer the whole time, through
  //     its own WheelChannel/PoseTracker bases. Motion::StateEstimator
  //     itself is deleted this ticket: a per-cycle computation with no
  //     consumer (its own former header said so).
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

  // --- Command --- writer: the SAME sole-or-arbitrated pair that writes
  // cmdVelocity above -- Motion::Planner::update() (mode/moveActive/v_x/
  // omega, reporting the trimmed body-frame twist actually being asked of
  // the wheels) when a Move owns motion, App::Drive::update() (mode/
  // moveActive/v_x/omega derived from its own WHEELS targets) when Drive
  // owns motion. RobotLoop::publishHealth() then re-derives mode/moveActive
  // from a fresh `planner_.active() || drive_.owns()` read as its own
  // cross-check, overwriting whichever of the two updates() ran that cycle
  // -- see publishHealth()'s own comment. v_x/omega, not a bare `v`
  // (naming-and-style.md rule 2 -- a twist is never directionless) --
  // named identically to Pose::v_x/BodyEstimate::v_x rather than
  // `targetVx` (avoids mashing a "target" prefix onto a mathematical
  // subscript); the `command.` section prefix at every call site is what
  // disambiguates "commanded" from "sensed"/"estimated," the same way
  // wheelLeft.velocity vs. wheelLeft.cmdVelocity are already disambiguated
  // by field name within one section rather than by a redundant prefix.
  struct Command {
    Mode mode = Mode::Idle;
    bool moveActive = false;  // true while either decider owns motion
    float v_x = 0.0f;         // [mm/s] signed, current commanded body-frame forward velocity
    float omega = 0.0f;       // [rad/s] signed, current commanded yaw rate
  } command;

  // --- Health --- writer: the owning module for each signal (I2CBus,
  // Comms, the two Motor leaves' wedge latch and gated wedge-suspect
  // stall, RobotLoop itself for
  // moveTimeout/shapingDisabled -- RobotLoop::publishMoveResult(), sourced
  // from Motion::TickResult and planner_.shaperConfigured()). Per-cycle
  // dynamics, like every other section --
  // NOT a place for a Deadman-style signal: App::Deadman was fully retired
  // in an earlier sprint (its former telemetry flag,
  // kFlagEventDeadmanExpired, is declared but unwired -- see telemetry.h's
  // own comment), so there is no live "deadman expired" signal today to
  // carry here.
  struct Health {
    uint32_t i2cSafetyNetCount = 0;    // Devices::I2CBus::clearanceSafetyNetCount()
    uint32_t commsMalformedCount = 0;  // App::Comms::malformedCount()
    // commandsDroppedCount (command-ingestion-...-two-stops.md §1):
    // App::Comms::commandsDroppedCount() -- a well-formed command that
    // arrived at a full command ring and was never routed. Sits beside
    // commsMalformedCount because both are "what the command plane lost,"
    // but they lose it for opposite reasons -- see telemetry.h's own
    // kFlagFaultCommandsDropped doc comment.
    uint32_t commandsDroppedCount = 0;
    bool wedgeLatch = false;           // either bound motor's Devices::Motor::wedged()
    bool moveTimeout = false;          // MOVE timeout backstop fired THIS cycle
    bool shapingDisabled = false;      // active MOVE, both ShaperLimits axes disabled
    // positionClamped (ticket 009, ADDITIVE -- genuinely live since 124-008
    // but not yet carried on RobotState until this ticket wired the
    // wheel-section publish point): either wheel's position hit the
    // position-rebaseline policy's defensive clamp fallback this cycle
    // (RobotLoop::clampToPositionWireBound(), Decision 6) -- see
    // telemetry.h's own kFlagFaultPositionClamped doc comment.
    bool positionClamped = false;
    // wheelFrozenLeft/wheelFrozenRight (129-002, wheel-frozen-fault-flag-
    // in-telemetry.md): writer RobotLoop::publishWheels(), sourced from
    // the bound Devices::Motor's own wedgeSuspect() -- the GATED
    // (motion-qualified) stall detector, commanded nonzero duty for N
    // consecutive cycles with no encoder change, NOT the raw,
    // unconditional wedgeLatch above (which also latches on a healthy
    // robot merely sitting parked at rest -- see telemetry.h's own bit
    // 19/20 doc comment for the full distinction). Feeds
    // kFlagFaultWheelFrozenLeft/Right (telemetry.h). This is ALSO the
    // regression guard 129-007's adaptive duty/speed gain learner reads
    // to refuse training on a stalled wheel's bogus near-zero speed --
    // keep this field sourcing wedgeSuspect(), never wedgeLatch/wedged(),
    // or that guard silently stops guarding.
    bool wheelFrozenLeft = false;
    bool wheelFrozenRight = false;
    // ready (128-012, App::Comms::updateStatus()'s own doc comment): the
    // one genuinely loop-owned fact STATUS answers that isn't a projection
    // of some other subsystem's own sensed/derived state -- "we are past
    // App::RobotLoop::boot()," i.e. the loop is now dispatching commands
    // instead of NACKing them via rejectDuringBoot(). Writer: RobotLoop::
    // boot(), set exactly once, at its own end (right after
    // comms_.sendReady()) -- never true before that point, never false
    // again after. Previously hard-coded `true` at the STATUS projection
    // site itself (a fact the projection could not actually get wrong, but
    // one it had no business asserting on the loop's behalf either).
    bool ready = false;
  } health;
};

}  // namespace Types
