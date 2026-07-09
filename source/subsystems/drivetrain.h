// drivetrain.h -- Subsystems::Drivetrain: the two-wheel differential
// Drivetrain faceplate. Turns a body twist or per-wheel velocity targets
// into a ratio-governed pair of msg::MotorCommands for two Hal::Motors,
// held (never pushed) as a Hal::DrivetrainToHardwareCommand -- see
// hasCommand()/takeCommand() below.
//
// Ported from the locked interface sketch in clasi/sprints/077-greenfield-
// faceplate-hal-drivetrain-and-dev-bench-system/issues/greenfield-rebuild-
// faceplate-hal-in-a-fresh-source-old-tree-parked.md, Step 4. Reshaped by
// sprint 079 (architecture-update.md's "The command-edge types" and
// "Authority arbitration -- Drivetrain-owned, not DevLoopState-owned"):
// tick() went from a return value to a held/taken output (hasCommand()/
// takeCommand()); port binding (`DEV DT PORTS`) and drive authority
// (formerly DevLoopState::leftPort/rightPort/drivetrainActive) moved INTO
// this class as ports()/active()/standby() -- see ticket 079-003.
//
// Drivetrain holds NO Hal::Motor reference or pointer: tick() takes the two
// wheels' observations (msg::MotorState) as arguments and HOLDS the
// Hal::DrivetrainToHardwareCommand it wants applied -- the wiring layer
// (main.cpp, ticket 079-005) is the only place that drains hasCommand()/
// takeCommand() and calls Hal::Motor::apply()/Subsystems::NezhaHardware::apply() with
// the result. This keeps Drivetrain free of any dependency on Hal::Motor's
// concrete leaf (NezhaMotor); it only knows the faceplate's message types
// plus the shared, data-only Hal::capability edge type
// (hal/capability/hal_command.h).
//
// No PID lives here -- that stays entirely inside NezhaMotor (ticket 3). The
// only control law this class runs is the ratio (sync) governor: see
// governRatio() below.
//
// Authority arbitration (architecture-update.md "Authority arbitration"):
// active() reports whether this Drivetrain is the one actually driving its
// bound pair right now. setTwist()/setWheelTargets()/setNeutral() each set
// active_ = true (a DEV DT verb that commands the drivetrain (re)activates
// authority, per docs/protocol-v2.md). standby() is the one audited
// "relinquish authority" path: active_ = false only, never touches mode_ or
// the last commanded target. A caller that also wants mode_ == NEUTRAL
// sends that via the SAME command's oneof arm (NEUTRAL) alongside
// msg::DrivetrainCommand.standby=true -- apply() processes the oneof first,
// then the standby side-channel, so both effects compose in one call. This
// reproduces today's neutralizeDrivetrain()/steal-authority semantics
// exactly (see the architecture doc's worked example).
//
// Two deviations from the issue's Step 4 code sketch, both documented at
// their point of use below:
//   1. The ratio governor runs for BOTH the TWIST arm (kinematics-derived
//      targets) and the WHEELS arm (directly-set targets) -- not only after
//      kinematics. The WHEELS arm is exactly how ticket 7's coupled-rig
//      curve test (ratio_governor_curve.py: "command a curve (unequal wheel
//      targets)" on ports 3+4) exercises the governor, so it must be
//      governed too; only the NEUTRAL arm is a true ungoverned pass-through.
//   2. setMotorCapabilities() is a small addition beyond the sketch's public
//      surface, needed so capabilities().onboard_position can be computed
//      without Drivetrain holding a Hal::Motor reference (see its own
//      comment below). Ticket 3 set the precedent for a documented,
//      rationale-backed deviation from the issue's sketch (Motor::apply()
//      returning bool).
#pragma once

#include <stdint.h>

#include "hal/capability/hal_command.h"
#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "runtime/queue.h"

namespace Subsystems {

// The Drivetrain's bound wheel-motor port pair -- moved here (into
// msg::DrivetrainConfig, read via ports()) from
// DevLoopState::leftPort/rightPort per sprint 079 (architecture-update.md
// decision 8): which wheels are mine is the same kind of fact as my
// trackwidth. `DEV DT PORTS <l> <r>` merges left_port/right_port into
// DrivetrainConfig and calls configure() -- wire text unchanged.
struct DrivetrainPorts {
  uint32_t left;
  uint32_t right;
};

class Drivetrain {
 public:
  // --- Primitive setters -- the real implementation of each command arm. ---

  // A twist is a directed body-frame velocity: v_x, v_y, omega (matches
  // msg::BodyTwist3; math subscripts keep their underscore). v_y is honored
  // only on holonomic drivetrains -- this Drivetrain is differential-only
  // (capabilities().holonomic is always false this sprint; Tovez first, per
  // the issue's locked control-architecture decision 5), so v_y is always
  // ignored -- see commandedWheelTargets()'s TWIST case for the ignore site
  // and the forward-pointer to the future mecanum ticket
  // (architecture-update.md Open Question 6).
  void setTwist(float v_x, float v_y, float omega);   // [mm/s] [mm/s] [rad/s]

  // Direct per-wheel velocity targets -- bypasses kinematics entirely (used
  // by, e.g., the coupled bench rig's curve tests, ticket 7, where
  // "left"/"right" are whichever two ports DEV DT PORTS bound, not
  // necessarily a real drivetrain's wheels). Still passes through the ratio
  // governor like the TWIST arm -- see tick().
  void setWheelTargets(float left, float right);      // [mm/s] signed wheel velocities

  void setNeutral(msg::Neutral mode);

  // --- Faceplate verbs. ---

  void configure(const msg::DrivetrainConfig& config);

  // Unpacks the oneof -> the setters above, THEN dispatches the standby
  // side-channel (see the class comment's "Authority arbitration" section)
  // -- both effects compose from one call.
  void apply(const msg::DrivetrainCommand& command);

  // now: [ms]. motors/motorCount: the FULL per-port committed observation
  // array this pass (bb.motors/its length, e.g. Rt::kPortCount) -- arguments
  // only, never stored, never read from a clock or a Motor reference. See
  // the class comment. Drivetrain resolves its OWN bound wheel pair
  // (ports()) against this array internally -- the `- 1` base conversion
  // (Nezha ports are 1-based; motors[] is 0-based) lives here exactly once,
  // with the object that owns the port binding, guarded by a range assert
  // against motorCount so a misconfigured (out-of-range) bound port cannot
  // silently walk off the array (090-001, clasi/sprints/090-mainloop-
  // cohesion-cleanup-.../architecture-update.md Decision 1). The caller
  // (MainLoop::tick()) does no port-cell indexing of its own -- it passes
  // the whole array and lets the port-owner resolve it.
  //
  // driveIn (087-003, clasi/sprints/087-two-plane-blackboard-synchronous-
  // update-loop-configurator-and-command-queue-transport-greenfield/
  // architecture-update-r1.md, "The Faceplate -- concrete example"): the
  // blackboard-sourced command transport (Rt::WorkQueue, source/runtime/
  // queue.h) this Drivetrain now drains INSTEAD OF however it previously
  // received its setpoint. tick() pops driveIn (FIFO, one command per tick)
  // when non-empty, at the top of the call, applying the popped
  // msg::DrivetrainCommand through the SAME apply() this class has always
  // used -- THEN the setpoint-governance math below runs exactly as before,
  // unchanged. An empty driveIn is a no-op: whatever setpoint is already
  // staged (mode_/targets, set by a previous apply()/driveIn post) is
  // governed unchanged -- today's "no new command" behavior. driveIn is the
  // FIFO WorkQueue<DrivetrainCommand> shared with
  // CommandRouter's `DEV DT` path and Planner's own output edge (Decision
  // 1's authority-gated arbitration -- see Planner::takeCommand()'s own doc
  // comment); the authority GATE itself (who is allowed to post) is ticket
  // 006/007's job, out of this ticket's scope. state().active exposes
  // active()/standby() from a state cell so a would-be poster can check
  // authority without holding a Drivetrain* (see state() below).
  //
  // HOLDS its output (a Hal::DrivetrainToHardwareCommand, addressed via
  // ports()) rather than returning it -- see hasCommand()/takeCommand()
  // below. Sets hasCommand() unconditionally whenever it runs; main.cpp
  // (ticket 079-005) only calls tick() when active().
  void tick(uint32_t now,
            const msg::MotorState* motors,
            uint32_t motorCount,
            Rt::WorkQueue<msg::DrivetrainCommand, 8>& driveIn);

  bool hasCommand() const;                      // true once tick() has run and the output is untaken
  Hal::DrivetrainToHardwareCommand takeCommand();     // clears hasCommand()

  // Assembled from getters. state().active mirrors active() (087-003) --
  // the authority-mode field a driveIn producer reads to check "who
  // currently has authority" without holding a Drivetrain* (see tick()'s
  // doc comment above).
  msg::DrivetrainState state() const;
  msg::DrivetrainCapabilities capabilities() const;

  // Records the two bound wheel motors' capabilities, needed only so
  // capabilities().onboard_position can report accurately without
  // Drivetrain holding a Hal::Motor reference. This is a plain data copy
  // (msg::MotorCapabilities is a POD message, not a handle) -- it does not
  // reintroduce the "no motor handles inside Drivetrain" constraint the
  // rest of this class observes. The wiring layer (ticket 5) calls this
  // once after binding a motor pair (DEV DT PORTS), querying each bound
  // Motor's own capabilities().
  void setMotorCapabilities(const msg::MotorCapabilities& left,
                             const msg::MotorCapabilities& right);

  // --- Port binding + authority arbitration (architecture-update.md
  // "Authority arbitration -- Drivetrain-owned, not DevLoopState-owned"). ---

  // The bound wheel-motor port pair, read from config_ (`DEV DT PORTS` ->
  // DrivetrainConfig.left_port/right_port, per sprint 079).
  DrivetrainPorts ports() const;

  // True if this Drivetrain is the one actually driving its bound pair right
  // now. setTwist()/setWheelTargets()/setNeutral() each (re)activate it.
  bool active() const;

  // The one audited "relinquish authority" path: active_ = false only --
  // never touches mode_ or the last commanded target. See the class
  // comment's "Authority arbitration" section for how a caller composes
  // this with a mode change in one apply() call.
  void standby();

 private:
  enum class Mode : uint8_t { NEUTRAL, TWIST, WHEELS };

  // Computes this Drivetrain's currently-commanded wheel velocity targets,
  // BEFORE the ratio governor -- kinematics for TWIST, a direct pass-through
  // for WHEELS, zero for NEUTRAL. Shared by tick() (which then governs the
  // result against live observations) and state() (which has no
  // observations to govern against, and reports the pre-governor commanded
  // targets as-is).
  void commandedWheelTargets(float* targetLeft, float* targetRight) const;

  // Ratio (sync) governor: if a wheel underachieves its target (bogged
  // down), lower the shared speed ceiling so left/right hold their
  // commanded ratio (curvature), instead of letting the healthy wheel run
  // away. Ported CONCEPT (not byte-for-byte -- architecture-update.md does
  // not require that here, unlike ticket 3's split-phase encoder
  // sequencing) from source_old/control/MotorController.cpp's syncGain
  // cross-wheel coupling, re-targeted at velocity TARGETS instead of the
  // duty-cycle PWM output it originally adjusted.
  // DrivetrainConfig.sync_gain is the tuning knob (0 = fully independent).
  void governRatio(float* targetLeft, float* targetRight,
                    const msg::MotorState& leftObs,
                    const msg::MotorState& rightObs) const;

  msg::DrivetrainConfig config_ = {};
  Mode mode_ = Mode::NEUTRAL;

  // TWIST-arm state.
  float v_x_ = 0.0f;      // [mm/s]
  float v_y_ = 0.0f;      // [mm/s] always ignored this sprint -- see setTwist()
  float omega_ = 0.0f;    // [rad/s]

  // WHEELS-arm state.
  float wheelTargetLeft_ = 0.0f;    // [mm/s]
  float wheelTargetRight_ = 0.0f;   // [mm/s]

  msg::Neutral neutralMode_ = msg::Neutral::BRAKE;

  msg::MotorCapabilities leftMotorCaps_ = {};
  msg::MotorCapabilities rightMotorCaps_ = {};

  // Authority + held-output state (sprint 079). See the class comment's
  // "Authority arbitration" section and tick()/hasCommand()/takeCommand().
  bool active_ = false;
  bool hasCommand_ = false;
  Hal::DrivetrainToHardwareCommand heldCommand_ = {};
};

}  // namespace Subsystems
