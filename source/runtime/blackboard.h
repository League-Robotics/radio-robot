// blackboard.h -- Rt::Blackboard: sprint 087's two-plane transport. Owns, as
// plain members, the committed state-plane snapshot x[k] (current-value
// cells: motors/drivetrain/pose/planner observations, current config) and
// every command-plane queue that connects each subsystem (commandsIn,
// driveIn, configIn, poseResetIn, otosSetPoseIn). Pure data -- no method
// computes anything; holds NO
// subsystem pointer of any kind (SUC-006). See
// clasi/sprints/087-two-plane-blackboard-synchronous-update-loop-
// configurator-and-command-queue-transport-greenfield/
// architecture-update-r1.md ("The blackboard" Reference code section) for
// the full design; this header is a direct port of that Reference code.
//
// Host-safe by construction (Decision 10). Every member type below is a
// host-safe POD:
//   - the eight state-cell msg::* types are auto-generated
//     (scripts/gen_messages.py) into source/messages/*.h, zero CODAL deps;
//   - Rt::PoseResetCommand / Rt::ConfigDelta are defined in the lightweight,
//     CODAL-free source/runtime/commands.h (an enum, a uint32_t, and one
//     msg::SetPose member -- also generated) -- NOT inline in this header,
//     since ticket 087-004 moved them out so
//     Subsystems::PoseEstimator::tick()'s poseResetIn parameter can name
//     Rt::PoseResetCommand without pose_estimator.h including this file
//     (the "subsystems never include blackboard.h" boundary rule);
//   - Subsystems::Hardware::kMotorCount is reachable via subsystems/
//     hardware.h alone, which includes only <stdint.h>, runtime/queue.h,
//     messages/motor.h, and the CODAL-free hal/capability/*.h interfaces;
//   - commandsIn's payload, Subsystems::
//     CommunicatorToCommandProcessorCommand, lives in the CODAL-free
//     source/subsystems/wire_command.h -- NOT subsystems/communicator.h,
//     which pulls in MicroBit.h/com/radio.h/com/serial_port.h.
//   - segmentIn's payload (100-007, THE CUTOVER: retyped from Motion::
//     Segment to Drive::Goal) is a plain POD with zero CODAL dependency --
//     source/drive/'s own isolation rule guarantees this (SUC-008); see
//     "Command plane" below.
// This is what makes Rt::Blackboard instantiable in a host test harness
// (tests/sim/unit/runtime_blackboard_harness.cpp) with the plain system
// C++ compiler -- no ARM toolchain, no MicroBit.h transitively included.
// Any FUTURE addition to Blackboard must be checked against this same
// host-safe-POD bar before being added (architecture-update-r1.md's
// Migration Concerns, Decision 10).
//
// (087-006) Six cells added beyond architecture-update-r1.md's own Reference
// code, all host-safe PODs per the same bar, needed so the pointerless
// command-family translators (source/commands/*.cpp) and the transitional
// loop (source/dev_loop.cpp/main.cpp/tests/_infra/sim/sim_api.cpp) have a
// bb-only path to facts/actions that previously required a held
// Subsystems::Hardware*/Hal::Odometer* reference:
//   - motorCaps[]/otosPresent -- boot-time, never-changing hardware-identity
//     facts (a motor's capability set, whether any Hal::Odometer exists at
//     all) snapshotted ONCE at boot by the loop (mirrors Subsystems::
//     Drivetrain's own setMotorCapabilities() cache precedent) so DEV M's
//     capability pre-validation gate and OI/OZ/OR/OV/OL/OA's "ERR nodev"
//     guard can run in the command family without a Hardware reference.
//   - devWatchdogWindow/streamWatchdogWindow (state) + devWatchdogWindowIn/
//     streamWatchdogWindowIn (command) -- the serial-silence watchdog
//     (`DEV WD`) and the streaming-drive watchdog (`SET sTimeout=`) are
//     loop-owned (ticket 007's "not one of the four Configurator-managed
//     subsystems" note on 087-007), so their window is a state cell the
//     loop publishes and a command mailbox the router posts to, the same
//     shape as configIn/*Config -- just for a scalar the Configurator does
//     not own.
//   - motionIn (command) -- S/T/D/R/TURN/RT/G/STOP's fan-out to the
//     loop-owned motion-executor step (drains into Subsystems::Planner::
//     apply(), ticket 007), carrying the msg::PlannerCommand plus the verb
//     disambiguation string the pre-087 MotionLoopState::activeVelocityVerb
//     field held (see runtime/commands.h's Rt::MotionCommand).
//   - otosCommandIn (command) -- OI/OZ/OR/OV's fan-out to the loop's direct
//     `hardware.odometer()->apply(...)` drain (mirrors otosSetPoseIn's own
//     "SI re-anchor -> odometer, drained by the loop directly" shape --
//     Hal::Odometer has no tick()-driven queue parameter of its own).
#pragma once

#include <array>
#include <cstdint>

#include "drive/drivetrain.h"
#include "messages/common.h"
#include "messages/drivetrain.h"
#include "messages/envelope.h"
#include "messages/motor.h"
#include "messages/odometer.h"
#include "messages/planner.h"
#include "runtime/commands.h"
#include "runtime/queue.h"
#include "subsystems/hardware.h"
#include "subsystems/wire_command.h"

namespace Rt {

constexpr uint32_t kMotorCount = Subsystems::Hardware::kMotorCount;  // 4

// Owned by the loop. Holds NO subsystem pointers -- only the committed
// snapshot x[k] (state plane) and the command queues (command plane).
struct Blackboard {
  // === State plane: committed snapshot x[k]. Written ONLY by the loop's
  //     commit step (from each subsystem's state()); read-only during a
  //     pass. ===
  //
  // (0-based motor indices, OOP refactor) motors[i] is motor index i's
  // state -- a std::array so a composition root can commit it in one shot,
  // `bb.motors = hardware.motorStates();` (Subsystems::Hardware::states()),
  // instead of a per-index copy loop.
  std::array<msg::MotorState, kMotorCount> motors;  // from Hardware
  msg::DrivetrainState drivetrain;    // from Drivetrain
  msg::PoseEstimate encoderPose;      // from PoseEstimator
  msg::PoseEstimate fusedPose;        // from PoseEstimator
  // bodyState (099-004, architecture-update.md Addition 2) -- reuses the
  // existing msg::PoseEstimate shape (pose+twist+stamp): pose from
  // fusedPose.pose, twist from BodyKinematics::forward() on the bound
  // pair's directly-read wheel velocities, stamp from fusedPose.stamp.
  // Published every pass by MainLoop::commit(); the ONE cell the follow-on
  // motion-v2 subsystem's thin adapter is designed to read directly.
  // Blackboard-only -- not on the wire this sprint (Decision 5).
  msg::PoseEstimate bodyState;
  // poseStepped (099-004, architecture-update.md Addition 1) -- the
  // magnitude of whatever pose correction (SI reset this sprint; a delayed
  // fix from 099-008 on) PoseEstimator applied on the immediately-prior
  // tick() call; zero on every other tick. Published every pass by
  // MainLoop::commit() from PoseEstimator::lastPoseStep(). Blackboard-only
  // -- not on the wire this sprint (Decision 5).
  msg::PoseStep poseStepped;

  // chainTail (100-007, THE CUTOVER) -- Drive::ChainTail: the predicted
  // world state at the end of everything currently admitted (executing +
  // queued), the issue's own "committed to the blackboard for queue-time
  // admission NACKs" cell. TWO cooperating writers, both documented at
  // their own call site: BinaryChannel's handleSegment()/handleReplace()
  // (commands/binary_channel.cpp) advance() it SYNCHRONOUSLY, at wire time,
  // on every successfully admitted segment/replace -- this is what lets a
  // stateless, pointerless wire handler (SUC-006: never a Subsystems::*
  // reference) NACK an infeasible ask against the REAL predicted queue tail
  // without needing a live Subsystems::Drivetrain reference; Subsystems::
  // Drivetrain::tick() (the adapter) re-anchors it to the current held/
  // measured pose on ABORT_*/an emptied ring, since a flush invalidates
  // whatever the wire handler had predicted. Not part of the "committed
  // snapshot x[k] written only by the loop's commit step" contract every
  // other state-plane cell above follows -- an explicit, narrow exception,
  // mirroring devWatchdogWindow/streamWatchdogWindow's own "loop-owned, not
  // strictly x[k]" precedent immediately below.
  Drive::ChainTail chainTail;

  // lastEvent (100-007, THE CUTOVER) -- the most recent msg::EventNotify
  // Subsystems::Drivetrain populated on an ABORT_* status (ring flushed,
  // ChainTail re-anchored -- see chainTail's own doc comment). Published
  // every pass by MainLoop::commit() from the adapter's own lastEvent()
  // getter, mirroring bb.drivetrain's publish shape exactly. Blackboard-
  // only this ticket -- wiring an actual unsolicited EVT reply onto the
  // wire is ticket 100-009's job (M9, "Trace/plan-dump wire arms"); no
  // loop-originated wire output exists yet post-093's removal (main_loop.h's
  // own "Safety supervision... and loop-originated wire output... stay gone
  // from the tick entirely" note) for this ticket to plug into.
  msg::EventNotify lastEvent;

  msg::PlannerState planner;          // from Planner
  // (090-003) odometer sample fusable -- derived from Hal::Odometer::
  // fusableThisPass(), never a device-presence (`!= nullptr`) test; always
  // false for a Hal::NullOdometer (no device). See main_loop.cpp's COMMIT
  // step for the exact derivation (reuses the SAME pass's one sanctioned
  // fusableThisPass() call, never a second one).
  bool otosValid = false;
  msg::PoseEstimate otos;             // from Hardware, when valid

  // otosConnected (092-002) -- Hal::Odometer::connected()'s live, per-pass
  // value, refreshed every pass in the loop's COMMIT step (main_loop.cpp),
  // straight off the SAME odometer pointer bb.otos/otosValid are sourced
  // from. Deliberately DISTINCT from otosValid (fusableThisPass()'s
  // reset-tracking flag) and from otos.stamp.valid (this ONE pass's read
  // freshness): this is "does a real device exist and answer at all" --
  // sticky/stable across many passes, unlike stamp.valid which flips on
  // Hal::OtosOdometer's own kReadPeriod rate-limit. Added as a diagnostic
  // surface for the frozen-fused-pose investigation
  // (clasi/issues/poseestimator-fused-pose-frozen-on-hardware.md): a bench
  // session could not previously tell, from the wire alone, whether
  // Hal::OtosOdometer had actually detected a chip (see otos_commands.cpp --
  // no existing verb surfaces connected()). Always false for a
  // Hal::NullOdometer/never-detected Hal::OtosOdometer.
  bool otosConnected = false;

  // Current config -- published by the Configurator on apply; read by
  // GET/telemetry. Replaces every shadow.
  msg::DrivetrainConfig drivetrainConfig;
  msg::MotorConfig motorConfig[kMotorCount];
  msg::PlannerConfig plannerConfig;
  msg::OdometerConfig odometerConfig;

  // (087-006) Boot-time hardware-identity snapshots -- never rewritten after
  // the loop's one-time boot seed (capabilities/device-presence do not
  // change at runtime for any current concrete Hardware leaf). See the file
  // header above.
  msg::MotorCapabilities motorCaps[kMotorCount];
  bool otosPresent = false;

  // (087-006) Loop-owned watchdog windows -- devWatchdogWindow is published
  // every pass by the loop from its own SerialSilenceWatchdog instance
  // (dev_commands.h; not one of the Configurator's four fold targets).
  // streamWatchdogWindow's own StreamingDriveWatchdog consumer was
  // already-dead code and was deleted outright (097-006, see
  // motion_commands.h's file header) -- the field below is still written
  // (by the binary config WATCHDOG patch, handleConfigWatchdog in
  // binary_channel.cpp) but has no live consumer.
  uint32_t devWatchdogWindow = 0;     // [ms] DEV WD's current window
  uint32_t streamWatchdogWindow = 0;  // [ms] binary config WATCHDOG patch's window (see above)

  // loopNow (2026-07-09 smooth-telemetry) -- the loop-pass time at which
  // this snapshot (bb.drivetrain, bb.motors) was committed, published by the
  // composition roots' commit step every pass. TLM surfaces it as `now=` so
  // per-pass values (cmd=) can be plotted at the instant they were COMPUTED
  // rather than at USB-CDC receive time, which batches/jitters replies by
  // tens of ms and renders a smooth commanded ramp as kinks.
  uint32_t loopNow = 0;               // [ms]

  // (087-006) STREAM/SNAP's own shared bookkeeping -- mirrors the pre-087
  // TelemetryState struct's periodMs/seq/replyFn+replyCtx/hasLastEmit/
  // lastEmitMs fields exactly, moved onto bb (as plain mutable scalars, not
  // strictly "committed state" in the x[k] sense -- nothing computes FROM
  // these) so telemetry_commands.cpp's STREAM handler can set them and the
  // loop's periodic-emission step can read them, neither holding a
  // Hardware/Drivetrain/PoseEstimator/Planner pointer. telemetryChannel
  // replaces the old raw ReplyFn/void* pair (a function pointer is not a
  // Blackboard-appropriate payload) -- the loop resolves it to its own
  // serial/radio reply sinks at emission time, the same way CommandRouter
  // resolves a command's Channel.
  uint32_t telemetryPeriod = 0;       // [ms] 0 = disabled; set (clamped) by STREAM
  uint16_t telemetrySeq = 0;          // shared by every STREAM-driven frame AND SNAP
  Subsystems::Channel telemetryChannel = Subsystems::Channel::NONE;
  bool telemetryHasLastEmit = false;
  uint32_t telemetryLastEmitMs = 0;   // [ms]

  // telemetryBinary (096-002, architecture-update.md M2) -- the branch point
  // tickTelemetry() (telemetry_commands.cpp) reads to choose text vs. binary
  // emission. Defaults false and stays inert this ticket: nothing sets it
  // true yet -- the binary formatter itself lands in ticket 003, and ticket
  // 005's binary `stream` arm is the first thing to ever set it true.
  bool telemetryBinary = false;

  // === Command plane: queues. Each drained by exactly ONE consumer. ===
  WorkQueue<Subsystems::CommunicatorToCommandProcessorCommand, 16>
      commandsIn;                            // Communicator -> router
  // driveIn: the S/STOP ESCAPE-HATCH input to Subsystems::Drivetrain ONLY --
  // drained (one command per tick, FIFO) and applied FIRST, ahead of
  // segmentIn (below), inside Drivetrain::tick() (see drivetrain.h's class
  // comment for the full precedence rules). UNCHANGED by the 100-007
  // cutover -- the DIRECT/escape-hatch path never went through the retired
  // Motion::SegmentExecutor and has no source/drive/ equivalent to migrate
  // to (architecture-update.md (100) "Impact on Existing Components").
  WorkQueue<msg::DrivetrainCommand, 8> driveIn;
  // segmentIn (100-007, THE CUTOVER: retyped from Motion::Segment to
  // Drive::Goal) -- ADMITTED primitive segments' fan-in. BinaryChannel's
  // handleSegment() (commands/binary_channel.cpp) is this queue's ONLY
  // producer: it runs wire admission (primitive-flag check +
  // Drive::Drivetrain::admit(), synchronously, against bb.chainTail) BEFORE
  // ever posting here -- an admission failure replies a typed ERR and never
  // reaches this queue at all ("queue untouched on rejection", ticket
  // 100-007's own acceptance criteria). An Rt::WorkQueue, NOT a latest-wins
  // Mailbox: multiple admitted segments can arrive between mandatory ticks
  // and must ALL apply, in order (the communicator issue's "no dropped
  // commands" requirement). Drained by Subsystems::Drivetrain::tick() into
  // its own internal ring_ every pass (see drivetrain.h) -- the actual
  // Drive::Drivetrain::plan() solve happens once a ring entry is POPPED to
  // become the active plan, not at admission time (admit() is the cheap,
  // conservative queue-time check; plan() is the exact, more expensive
  // Ruckig solve -- source/drive/drivetrain.cpp's own admit()/plan() doc
  // comments).
  WorkQueue<Drive::Goal, 8> segmentIn;
  // replaceIn (100-008: retyped from Drive::Goal to MoverRequest) -- a
  // latest-wins Mailbox ON PURPOSE, the exact dual of segmentIn's
  // no-dropped-commands WorkQueue: MOVER teleop's "each fresh MOVER
  // replaces the held plan" contract (SUC-010) IS Mailbox semantics, no new
  // queueing behavior. BinaryChannel::handleReplace() (commands/
  // binary_channel.cpp) is this queue's ONLY producer -- it decodes the
  // `replace`-arm MotionSegment's time/v/omega fields (primitive=true
  // required, ERR_UNIMPLEMENTED otherwise) straight into a MoverRequest, no
  // admit()/chainTail involvement (a velocity-mode plan has no pose goal to
  // admit against). Drained by Subsystems::Drivetrain::tick() into
  // Drive::Drivetrain::planVelocity(request.target, request.deadman,
  // current) every pass a fresh MOVER is pending -- see drivetrain.cpp's
  // own replaceIn-drain comment for the full swap-vs-keep-old-plan
  // contract on a SOLVE_FAILED/CEILING_INFEASIBLE verdict.
  Mailbox<MoverRequest> replaceIn;
  WorkQueue<ConfigDelta, 16> configIn;        // router -> Configurator
  WorkQueue<PoseResetCommand, 4> poseResetIn;  // router -> PoseEstimator
  // poseFixIn (099-008, architecture-update.md D7): a genuine timestamped
  // delayed camera fix -- BinaryChannel::handlePose()'s third branch
  // (neither `reset` nor `zero_encoders`) posts here. Latest-wins Mailbox
  // ON PURPOSE, the dual of poseResetIn's FIFO WorkQueue immediately
  // above: a newer camera frame supersedes an undrained older one, by
  // design -- there is no "queue every fix and apply them all in order"
  // requirement the way SI/ZERO resets have. Drained by
  // Subsystems::PoseEstimator::tick() itself (its own 7th parameter), not
  // by MainLoop directly.
  Mailbox<PoseFixCommand> poseFixIn;
  Mailbox<msg::SetPose> otosSetPoseIn;        // SI re-anchor -> odometer
  Mailbox<msg::OdometerCommand> otosCommandIn;  // OI/OZ/OR/OV -> odometer (loop-drained)
  Mailbox<uint32_t> devWatchdogWindowIn;       // DEV WD -> loop's SerialSilenceWatchdog
  Mailbox<uint32_t> streamWatchdogWindowIn;    // binary config WATCHDOG patch -> streamWatchdogWindow (StreamingDriveWatchdog deleted 097-006, no live consumer)
  Mailbox<MotionCommand> motionIn;             // S/T/D/R/TURN/RT/G/STOP -> Planner::apply()
};

}  // namespace Rt
