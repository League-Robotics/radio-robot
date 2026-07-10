#pragma once

// ---------------------------------------------------------------------------
// text_channel.h -- ALL remaining text-command source, consolidated from six
// file pairs into one (097-011, pure file consolidation, ZERO behavior
// change): motion_commands.{h,cpp} (STOP), system_commands.{h,cpp}
// (PING/HELLO + the identity helpers), pose_commands.{h,cpp} (SI/ZERO),
// otos_commands.{h,cpp} (OI/OZ/OR/OP/OV/OL/OA), and dev_commands.{h,cpp}
// (DEV *) all move here verbatim -- each section banner below reproduces the
// donor file's own original header comment, unabridged. Mirrors
// binary_channel.{h,cpp}'s naming: text_channel is the text-plane
// counterpart, binary_channel the binary-plane one -- source/commands/ now
// holds exactly 4 file pairs: arg_parse, binary_channel, command_processor,
// text_channel.
//
// Three sections, in order, each independently labeled below:
//   1. LIVE RUMP -- STOP + PING/HELLO, the only commands
//      `Rt::CommandRouter::buildTable()` (command_router.cpp) actually
//      registers, via this file's own `textCommands()` (097-011: folds the
//      donor files' separate `motionCommands()`/`systemCommands()` into one
//      call site) -- plus `formatDeviceAnnouncement()`/`deviceIdentity()`,
//      kept at external linkage exactly as before (binary_channel.cpp and
//      communicator.cpp call them).
//   2. DEAD SOURCE -- SPRINT 098 TRANSCRIPTION REFERENCE -- `poseCommands()`
//      (SI/ZERO) and `otosCommands()` (OI/OZ/OR/OP/OV/OL/OA): declared here
//      with external linkage, never called from `buildTable()` -- kept as
//      sprint 098's own field-shape/Blackboard-target reference for the
//      binary `pose`/`otos` `CommandEnvelope` arms. Do NOT trim or
//      summarize these two sections' doc comments -- they are 098's
//      transcription source.
//   3. DEV BENCH-DIAGNOSTIC -- `devCommands()` (DEV M/DEV DT/DEV STATE/
//      DEV STOP/DEV WD), `ROBOT_DEV_BUILD`-gated, declared here with
//      external linkage, never called from `buildTable()` -- raw per-port
//      motor control bypassing Drivetrain entirely, with no binary
//      counterpart planned.
//
// This is a mechanical move, not a rewrite: no handler logic, descriptor
// table, or wire behavior changed by this consolidation. The registered
// command table (`textCommands()`, Section 1) is byte-for-byte the same
// STOP/PING/HELLO set `motionCommands()`+`systemCommands()` used to
// produce; `poseCommands()`/`otosCommands()`/`devCommands()` remain
// declared-but-unregistered with external linkage exactly as before, so
// there is no `-Wunused` fallout and the registered-vs-dead distinction is
// unchanged.
// ---------------------------------------------------------------------------

#include <stdint.h>
#include <vector>

#include "command_types.h"
#include "hal/capability/hal_command.h"
#include "runtime/command_router.h"

// =============================================================================
// SECTION 1: LIVE RUMP -- STOP (from motion_commands.h) + PING/HELLO (from
// system_commands.h). `textCommands()` (below) is this sprint's own
// registered command table.
// =============================================================================

// --- motion_commands.h's original file header comment (097-006/097-008), verbatim ---
//
// motion_commands.h -- the STOP/TLM wire family (084-002..005, rewritten
// pointerless 087-006).
//
// 097-006 (architecture-update-r2.md Decision 9, Eric's 2026-07-10 redirect
// to a pure-binary firmware): DELETES the S/T/D/R/TURN/RT/G/MOVE/MOVER
// parser/handler pairs and QLEN outright -- unconditionally, not merely
// unregistered the way 093-001 left T/D/R/TURN/RT/G "source-unchanged but
// unwired" (see this file's git history for that prior state; 094-006
// un-registered D/T/RT into the segment path and added MOVE/MOVER on top of
// that same table). Every deleted verb's binary-plane parity
// (`drive`/`segment`/`replace` arms, source/commands/binary_channel.cpp)
// already carries its runtime behavior forward -- hardware-bench-smoke-
// tested (095, drive/stop) and sim-exhaustive (096, segment/replace) -- so
// no behavior is lost, only the SECOND (text) implementation of it. R/TURN/
// G had no binary arm at all and are deleted with no replacement --
// Decision 9's "no consumer-gating, no preservation" ethos explicitly
// overrides r1's earlier "keep until something proven replaces it"
// reasoning for exactly these three (see architecture-update-r2.md's own
// "forced consequence" note). The shared stop-clause text grammar
// (`parseStopClauseValue`/`collectStopClauses`/`packStopKVs`/
// `kMaxStopConds`/`replyStopBadarg`) and `copyCorrId()` had no callers left
// once those six/three handlers were gone and were deleted alongside them.
// `bb.motionIn`/`Rt::MotionCommand` (runtime/commands.h/blackboard.h) are
// now fully unreferenced plumbing as a result -- flagged as a future
// cleanup, explicitly OUT of this ticket's file scope (architecture-
// update-r2.md Open Question 1).
//
// STOP survives (093-001's fix, physical behavior updated by 094-004/006):
// `handleStop` builds a msg::DrivetrainCommand{NEUTRAL} inline (deliberately
// WITHOUT the standby side-channel -- see handleStop's own doc comment in
// motion_commands.cpp for why dev_commands.h's buildDrivetrainStop(), which
// sets standby=true, silently dropped the neutral instead of stopping the
// wheels) and posts it to the same bb.driveIn mailbox the binary `stop` arm
// also targets. STOP is one of this sprint's confirmed liveness-adjacent
// rump verbs (architecture-update-r2.md's 3-verb default: STOP/PING/HELLO)
// -- see system_commands.h for PING/HELLO's own rationale.
//
// TLM (094-006's one-shot pull-based telemetry read) was UNTOUCHED by
// ticket 006 -- its deletion was ticket 008's own scope (the text telemetry
// family), even though its source lived in this file. 097-008 has since
// deleted handleTlm/TLM outright (Decision 9's "no consumer-gating, no
// preservation" ethos, same as every other verb this file names above --
// see that ticket's own Description for why and for the file-edit
// coordination note with this ticket).
//
// StreamingDriveWatchdog -- DELETED (097-006): already-dead code, fed by
// nothing (confirmed in the 097 architecture research before this ticket
// ran; it fed the S-only streaming-drive-silence timeout, and S no longer
// exists). The text SET/GET config family (source/commands/
// config_commands.{h,cpp}) used to `#include "commands/motion_commands.h"`
// for a doc-comment-only reference to this type -- never an actual type use
// (it touched only `bb.streamWatchdogWindow`/`bb.streamWatchdogWindowIn`,
// both plain `uint32_t`/`Mailbox<uint32_t>`, not this class). That file was
// itself deleted outright by ticket 007, so the point is now moot.
// --- end motion_commands.h original header ----------------------------------

// --- system_commands.h's original file header comment (077-001/097-006), verbatim ---
//
// system_commands.h -- liveness command family (077-001).
//
// 097-006 (architecture-update-r2.md Decision 9, Eric's 2026-07-10 redirect
// to a pure-binary firmware): DELETES VER/HELP/ECHO/ID unconditionally --
// only PING and HELLO survive as this sprint's confirmed liveness rump
// (the 3-verb default alongside motion_commands.h's STOP -- see that
// file's header comment). Each deleted verb's information content is a
// strict subset of what the binary plane already carries: `echo`/`id`
// (095, hardware-bench-smoke-tested) are direct binary parity for ECHO/ID;
// VER's `fw`/`proto` pair is a strict subset of binary `id`'s `DeviceId`
// reply; HELP enumerated the live text table, which no longer has anything
// interesting left to enumerate. HELLO is the one exception kept beyond
// Eric's stated 2-verb (STOP+PING) default -- see the paragraph below.
//
// Handler bodies were ported from source_old/commands/SystemCommands.cpp,
// with the old Robot*/RobotSysCtx handlerCtx coupling removed -- this
// firmware has no Robot class. PING/HELLO read only free vendor functions
// (Types::systemClockNow(), microbit_friendly_name(), microbit_serial_number())
// and the wire constants in types/protocol.h.
//
// 088-005: HELLO re-requests the firmware's own identity banner (removed
// under v1->v2, re-added here). formatDeviceAnnouncement() is the single
// place that knows the `DEVICE:NEZHA2:robot:<name>:<serial>` wire format
// and the #ifdef HOST_BUILD identity source deviceIdentity() uses -- both
// main.cpp's boot-time announcement and handleHello call it, so the two
// call sites can never format the banner differently (architecture-update.md
// Decision 3: a shared free function here, not a revived Announcer class).
// 097-006 keeps HELLO specifically because communicator.cpp's own boot-
// announcement comment documents that a missed boot banner is "not a
// failure -- HELLO re-requests it", and host/robot_radio/io/serial_conn.py's
// connect()/_banner_classify() sends HELLO repeatedly on RECONNECT for
// exactly that reason -- deleting it would remove the one re-request path
// that handshake structurally depends on, a materially different risk than
// the other four (diagnostic-only) liveness verbs. See
// architecture-update-r2.md's "Open decision: the text safety rump" for the
// full evidence; flagged there as pending Eric's confirmation, not silently
// assumed.
//
// 095-007: deviceIdentity() has external linkage (declared here, defined in
// system_commands.cpp outside its anonymous namespace) so BinaryChannel's
// binary `id` reply (source/commands/binary_channel.cpp) can source
// model/name/serial/fw/proto from the SAME identity pair formatDeviceAnnouncement()
// uses, instead of duplicating the #ifdef HOST_BUILD branch a second time
// (architecture-update.md Decision 4). Zero behavior change to any existing
// caller.
// --- end system_commands.h original header ----------------------------------

// Formats the `DEVICE:NEZHA2:robot:<name>:<serial>` identity banner into
// buf[0..size-1]. <name>/<serial> are exactly microbit_friendly_name()/
// microbit_serial_number() -- the same identity pair deviceIdentity()
// exposes below (HOST_BUILD substitutes the same fixed "HOST-SIM"/0
// placeholder host-side). Buffer-writing convention, matching
// CommandProcessor::listVerbs(): returns the length written (snprintf
// semantics), truncates silently if buf is too small. Called once by
// main.cpp at boot (both channels) and once per HELLO request.
int formatDeviceAnnouncement(char* buf, int size);

// The raw identity pair formatDeviceAnnouncement() formats from: *name/
// *serial are exactly microbit_friendly_name()/microbit_serial_number() on-
// target, or the fixed "HOST-SIM"/0 placeholder under HOST_BUILD. External
// linkage (095-007) so BinaryChannel's binary `id` handler can build a
// msg::DeviceId from the SAME source, never a second #ifdef HOST_BUILD
// branch.
void deviceIdentity(const char** name, uint32_t* serial);

// Returns the text safety rump's command table: PING, HELLO (from
// system_commands.h's own systemCommands()), STOP (from motion_commands.h's
// own motionCommands()) -- 097-011 folds both prior per-family builders
// into this one call site, so `Rt::CommandRouter::buildTable()`
// (command_router.cpp) now calls `textCommands()` alone instead of
// `systemCommands()` + `motionCommands()` + `telemetryCommands()`. TLM
// (094-006's one-shot pull-based telemetry read) was deleted by 097-008 --
// see this section's motion_commands.h header comment above.
std::vector<CommandDescriptor> textCommands(Rt::CommandRouter& router);

// =============================================================================
// SECTION 2: DEAD SOURCE -- SPRINT 098 TRANSCRIPTION REFERENCE. Neither
// `poseCommands()` nor `otosCommands()` is called anywhere except its own
// definition (text_channel.cpp) -- kept as sprint 098's own field-shape/
// Blackboard-target reference for the binary `pose`/`otos` CommandEnvelope
// arms (envelope.proto fields 7/8). Do NOT trim or summarize either
// section's doc comments below.
// =============================================================================

// --- pose_commands.h's original file header comment (084-007/SUC-006/087-006), verbatim ---
//
// pose_commands.h -- the pose-set command family (084-007/SUC-006,
// rewritten pointerless 087-006): `SI` (re-anchor the believed world pose)
// and `ZERO enc` (rezero the bound pair's encoders).
//
// **Decision 7 (architecture-update-r1.md), router-half:** SI/ZERO are
// one-shot commands whose effects are entangled with PoseEstimator's own
// integration (the phantom-jump coherence problem) -- so, unlike SET's
// config-plane deltas (which flow through the Configurator), SI/ZERO POST
// directly to the target-drained reset queues PoseEstimator/Hardware
// themselves consume (bb.poseResetIn, bb.motorResetIn[]), plus the
// odometer-directed fan-out the loop drains directly (bb.otosSetPoseIn):
//   - SI posts Rt::PoseResetCommand{kind=kSetPose, pose} to bb.poseResetIn
//     (drained by Subsystems::PoseEstimator::tick(), ticket 004) AND the
//     SAME pose to bb.otosSetPoseIn (a Mailbox<msg::SetPose>, drained by the
//     loop directly against hardware.odometer() -- mirrors the pre-087
//     two-call handleSI(), just posted instead of called).
//   - `ZERO enc` posts Rt::PoseResetCommand{kind=kResetBaseline} to
//     bb.poseResetIn AND sets bb.motorResetIn[left-1]/[right-1] = true
//     (drained by Subsystems::Hardware::tick(), ticket 004) -- the port
//     binding is read from bb.drivetrainConfig.left_port/right_port (the
//     published snapshot), never a Drivetrain*.
//
// Neither handler holds or dereferences a Subsystems::* pointer (SUC-006).
// SI/ZERO's own reply text is built directly from the parsed wire input
// (never a bb read-back), matching today's wire text exactly.
// --- end pose_commands.h original header -------------------------------------

// Returns the pose-set command table (SI, ZERO), bound to `router`.
std::vector<CommandDescriptor> poseCommands(Rt::CommandRouter& router);

// --- otos_commands.h's original file header comment (084-008/SUC-007/087-006), verbatim ---
//
// otos_commands.h -- the seven-verb OTOS command family (084-008/SUC-007,
// rewritten pointerless 087-006): OI/OZ/OR/OP/OV/OL/OA, fully specified in
// docs/protocol-v2.md §11 (grammar, reply shapes, ERR nodev behavior).
//
// Never holds or dereferences a Subsystems::Hardware*/Hal::Odometer*
// (SUC-006). Device presence is read from bb.otosPresent -- a boot-time
// snapshot of `hardware.odometer() != nullptr` (see blackboard.h's file
// header): every current concrete Hardware leaf's odometer() either always
// returns the same non-null leaf or always returns nullptr for its whole
// lifetime, so a one-time boot snapshot is equivalent to the pre-087 "live
// resolution on every dispatch" for every build this tree actually produces
// (NezhaHardware always non-null since 086-006; SimHardware always
// non-null) -- flagged explicitly since the pre-087 file header's own
// rationale for LIVE resolution ("a future odometer swap... must not
// require touching this file") assumed a Hardware reference this rewrite
// can no longer hold; a genuine hot-swap capability, if ever added, would
// need bb.otosPresent refreshed by whatever performs the swap (the loop),
// not by this file.
//
// OI/OZ/OR/OV post one msg::OdometerCommand to bb.otosCommandIn (a
// Mailbox<msg::OdometerCommand>, drained by the loop directly against
// hardware.odometer() -- mirrors bb.otosSetPoseIn's own "the loop drains
// this against the odometer directly" shape, since Hal::Odometer has no
// tick()-driven queue parameter of its own). OP reads bb.otos/bb.otosPresent
// directly (a state-cell read, matching its pre-087 "reads Hal::Odometer::
// pose() directly... not tick()" CMD_NONE shape). OL/OA read/write
// bb.odometerConfig (the Configurator's own published config cell,
// replacing the pre-087 OtosCommandState::configShadow) and post a
// field-masked Rt::ConfigDelta (kOdometer) to bb.configIn on a set --
// mirrors the now-deleted text SET handler's own (config_commands, removed
// 097-007) and DEV *CFG's candidate-then-commit pattern exactly, since
// OL/OA are genuinely config-plane (read-modify-write persistent
// register), unlike the other five one-shot verbs.
// --- end otos_commands.h original header ---------------------------------

// Returns the OTOS command table (OI, OZ, OR, OP, OV, OL, OA), bound to
// `router`.
std::vector<CommandDescriptor> otosCommands(Rt::CommandRouter& router);

// =============================================================================
// SECTION 3: DEV BENCH-DIAGNOSTIC. `devCommands()` is `ROBOT_DEV_BUILD`-
// gated, declared here with external linkage, never called from
// `buildTable()`.
// =============================================================================

// --- dev_commands.h's original file header comment (077-005/087-006), verbatim ---
//
// dev_commands.h -- the DEV command family (077-005, rewritten pointerless
// 087-006): the only command family beyond bare liveness (PING/VER/HELP/
// ECHO/ID, system_commands.*) this dev-bench firmware registers. DEV drives
// individual motor ports and, through Subsystems::Drivetrain, a bound
// motor-port pair -- but now ONLY by reading Rt::Blackboard state cells and
// posting to its command-plane queues (Rt::CommandRouter::blackboard()),
// never by holding or dereferencing a Subsystems::* pointer (SUC-006).
//
// Full vocabulary (unchanged by this rewrite -- ported from the locked table
// in clasi/sprints/077-greenfield-faceplate-hal-drivetrain-and-dev-bench-
// system/issues/greenfield-rebuild-faceplate-hal-in-a-fresh-source-old-tree-
// parked.md, Step 5):
//
//   DEV M <n> DUTY <duty>       -- [%, -100..100]  motor duty-cycle setpoint
//   DEV M <n> VEL <velocity>    -- [mm/s] signed    embedded PID closes the loop
//   DEV M <n> POS <position>    -- [deg]            onboard absolute-angle move
//   DEV M <n> VOLT <voltage>    -- [V]              ERR unsupported (Nezha: no voltage mode)
//   DEV M <n> NEUTRAL <B|C>     -- brake or coast
//   DEV M <n> RESET             -- zero the encoder (MotorCommand.reset_position)
//   DEV M <n> STATE             -- OK DEV M <n> pos=.. vel=.. applied=.. wedged=.. conn=..
//   DEV M <n> CAPS              -- OK DEV M <n> duty=.. volt=.. vel=.. pos=.. enc=..
//   DEV M <n> CFG k=v ...       -- a Rt::ConfigDelta (kMotor) posted to bb.configIn
//   DEV DT PORTS <left> <right> -- bind Drivetrain to a motor-port pair (a Rt::ConfigDelta, kDrivetrain)
//   DEV DT VW <vx> <vy> <omega> -- [mm/s mm/s rad/s] body twist (ratio-governed)
//   DEV DT WHEELS <left> <right>-- [mm/s] per-wheel velocity targets (ratio-governed)
//   DEV DT NEUTRAL <B|C>
//   DEV DT STATE
//   DEV DT STOP                 -- drivetrain-scoped stop: bound pair + drivetrain idle
//   DEV DT CFG k=v ...          -- a Rt::ConfigDelta (kDrivetrain) posted to bb.configIn
//   DEV STATE                   -- everything: one line per motor + drivetrain
//   DEV STOP                    -- all four motors neutral, drivetrain idle
//   DEV WD <window>             -- [ms] set the serial-silence watchdog window
//
// <n> is always a motor PORT (1..4), matching how NezhaHardware instantiates one
// NezhaMotor per port (ticket 3) -- never an L/R role name.
//
// --- Open Question 3 (argument parsing mechanism) -- RESOLVED (unaffected
// by this rewrite; see dev_commands.cpp) ---
//
// --- 087-006 reshape: pointerless translators against Rt::Blackboard ---
// Every handler below is now a pure translator: it reads the bb state cells
// it needs (bb.motors[]/bb.motorCaps[]/bb.drivetrain/bb.drivetrainConfig) and
// posts a typed command onto the matching bb queue -- never calling
// Hal::Motor/Subsystems::Drivetrain/Subsystems::Hardware directly:
//   - DUTY/VEL/POS/VOLT/NEUTRAL/RESET post one msg::MotorCommand to
//     bb.motorIn[idx] (a per-port Mailbox, Decision 2, `idx` the 0-based
//     Hardware motor index converted from the wire `<n>` at handleDevM()'s
//     own handler boundary) ON ACCEPTANCE -- acceptance is still
//     pre-validated against bb.motorCaps[idx] (a boot-time snapshot of
//     Hal::Motor::capabilities(), since capabilities never change at
//     runtime for any current concrete leaf -- see blackboard.h's file
//     header), so a capability-rejected command (e.g. VOLT on Nezha) never
//     posts anything and so never steals authority, exactly as before.
//     091-002: DUTY/VEL/POS are ALSO pre-validated against
//     bb.motorConfig[idx].polled (the Configurator's published
//     NezhaHardware poll-set snapshot) BEFORE the capability gate --
//     `portIsPolled()` in dev_commands.cpp -- rejecting an unpolled port's
//     motion verb `ERR nodev <mode>` (mirrors OI/OZ/OR/OV's device-presence
//     convention). NEUTRAL/RESET/STATE/CAPS/CFG are never gated by poll
//     membership; a port opts into the poll set via `DEV M <n> CFG
//     polled=true` (see applyMotorCfgKey() in dev_commands.cpp).
//   - A bound-port DEV M motion verb ALSO posts a standby-only
//     msg::DrivetrainCommand ({control_kind=NONE, standby=true}) to
//     bb.driveIn (shared with DEV DT's own posts -- Decision 1's coalescing
//     mailbox) when the targeted port is one of Drivetrain's currently-bound
//     ports (read from bb.drivetrainConfig.left_port/right_port -- never a
//     Drivetrain* -- Decision 7's router-half pattern).
//   - DEV DT's motion verbs (VW/WHEELS/NEUTRAL) post a msg::DrivetrainCommand
//     to bb.driveIn UNCONDITIONALLY -- Decision 1's authority gate belongs to
//     whatever ELSE also posts to driveIn (Planner's own output, drained by
//     the loop's routeOutputs, ticket 007); DEV DT itself has never been
//     gated (today's contract is "always (re)activates authority" -- see
//     drivetrain.h's own "Authority arbitration" section) and this rewrite
//     preserves that contract exactly.
//   - DEV STOP's broadcast HAL neutral posts to bb.hardwareBroadcastIn (a
//     dedicated Mailbox<msg::MotorCommand> -- NOT bb.motorIn[], since
//     bb.motorIn[]'s per-port drain (NezhaHardware::tick()) has no
//     "broadcast to every port in one shot" shape at all -- a true
//     broadcast needs the allPorts=true Hal::CommandProcessorToHardwareCommand
//     forwarded through Hardware::apply(), a structurally different
//     distribution path than posting into 4 separate per-port mailboxes);
//     its Drivetrain-side {NEUTRAL,standby=true} posts to bb.driveIn like
//     any other DEV DT-shaped command. DEV DT STOP's narrower, addressed
//     (bound-pair-only) neutral posts to bb.motorIn[left-1]/
//     bb.motorIn[right-1] directly (the SAME per-port shape bb.motorIn[]'s
//     drain already handles), so no separate cell is needed for that case.
//   - DEV M <n> CFG / DEV DT CFG / DEV DT PORTS post one Rt::ConfigDelta
//     (kMotor / kDrivetrain) to bb.configIn -- the Configurator (ticket 005)
//     folds+applies it, exactly the same "config-plane, not command-plane"
//     shape SET already uses (config_commands.h), replacing the old
//     motorConfigShadow[]/drivetrainConfigShadow read-modify-write shadows
//     entirely (bb.motorConfig[]/bb.drivetrainConfig -- published by the
//     Configurator -- are the new read baseline).
//   - DEV WD posts the requested window to bb.devWatchdogWindowIn (a
//     Mailbox<uint32_t>, drained by the loop directly into its own
//     loop-owned SerialSilenceWatchdog instance -- the watchdog is not one of
//     the Configurator's four targets, architecture-update-r1.md/ticket
//     087-007's own note).
//   - STATE/CAPS/DEV STATE are pure reads against bb.motors[]/bb.motorCaps[]/
//     bb.drivetrain/bb.drivetrainConfig -- never touch any queue.
//
// --- Serial-silence watchdog -- NON-NEGOTIABLE (runaway history) ---
// SerialSilenceWatchdog itself is UNCHANGED by this rewrite (still a small,
// dependency-free value class) -- only its OWNER changes: it is no longer
// embedded in a deleted DevLoopState, it is a loop-owned instance (main.cpp/
// sim_api.cpp), fed every pass the loop ingests any command (regardless of
// channel/content) and checked every pass; on expiry the loop applies
// buildBroadcastNeutral()/buildDrivetrainStop() IMMEDIATELY (bypassing
// bb.driveIn/bb.motorIn/bb.hardwareBroadcastIn entirely -- the loop already
// holds Hardware&/Drivetrain& directly, so an emergency stop gains nothing
// from an extra pass of queue latency) and emits `EVT dev_watchdog`. See
// docs/protocol-v2.md's "Development commands" section for the wire
// contract, which this rewrite does not change.
//
// --- Build gating ---
// Unchanged: the whole family (and command_router.cpp's own registration of
// it) compiles only under ROBOT_DEV_BUILD.
// --- end dev_commands.h original header --------------------------------------

// ---------------------------------------------------------------------------
// SerialSilenceWatchdog -- see the file-level comment above. Unchanged in
// shape from before this rewrite; only its owner (the loop, not a deleted
// DevLoopState) changes.
// ---------------------------------------------------------------------------
class SerialSilenceWatchdog {
 public:
  static constexpr uint32_t kDefaultWindow = 1000;   // [ms]

  explicit SerialSilenceWatchdog(uint32_t window = kDefaultWindow)
      : windowMs_(window) {}

  // Call once at boot (so the window starts counting from power-on, not from
  // an uninitialized lastFeedMs_) and again every time a command line
  // arrives on either comms channel -- see the class comment.
  void feed(uint32_t now) { lastFeedMs_ = now; fired_ = false; }

  void setWindow(uint32_t window) { windowMs_ = window; }
  uint32_t window() const { return windowMs_; }

  // Returns true exactly once per silence episode: the first check() call at
  // or after the window has elapsed since the last feed(). Subsequent calls
  // return false until the next feed() re-arms it, so the caller
  // neutralizes exactly once per episode rather than every loop iteration.
  bool check(uint32_t now) {
    if (fired_) return false;
    if (now - lastFeedMs_ >= windowMs_) {
      fired_ = true;
      return true;
    }
    return false;
  }

 private:
  uint32_t windowMs_;
  uint32_t lastFeedMs_ = 0;
  bool fired_ = false;
};

// ---------------------------------------------------------------------------
// buildBroadcastNeutral / buildDrivetrainStop -- the one audited "make
// everything safe" construction path, unchanged in shape from before this
// rewrite. Shared by:
//   - DEV STOP's handler, which now POSTS the result (bb.hardwareBroadcastIn/
//     bb.driveIn) instead of writing into a deleted DevLoopState outbox.
//   - the loop's serial-silence watchdog-fire path, which still APPLIES the
//     result IMMEDIATELY (the loop is the top of the call tree, not a
//     subsystem; an emergency stop gains nothing from an extra pass of
//     queue latency).
// `DEV DT STOP` (the narrower, bound-pair-only stop) reuses
// buildDrivetrainStop() for its Drivetrain-side neutral+standby but posts
// its own addressed (bb.motorIn[left-1]/[right-1]) HAL neutral -- see
// handleDevDt's STOP case in dev_commands.cpp.
// ---------------------------------------------------------------------------
Hal::CommandProcessorToHardwareCommand buildBroadcastNeutral(msg::Neutral mode = msg::Neutral::BRAKE);
msg::DrivetrainCommand buildDrivetrainStop(msg::Neutral mode = msg::Neutral::BRAKE);

// Returns the DEV command table (DEV M, DEV DT, DEV STATE, DEV STOP, DEV WD),
// bound to `router` -- every handler reaches Rt::Blackboard exclusively via
// router.blackboard() (see command_router.h's class comment). Mirrors
// textCommands()'s free-function shape (this file, Section 1).
std::vector<CommandDescriptor> devCommands(Rt::CommandRouter& router);
