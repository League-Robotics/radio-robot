#pragma once

// ---------------------------------------------------------------------------
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
// it needs (bb.motor[]/bb.motorCaps[]/bb.drivetrain/bb.drivetrainConfig) and
// posts a typed command onto the matching bb queue -- never calling
// Hal::Motor/Subsystems::Drivetrain/Subsystems::Hardware directly:
//   - DUTY/VEL/POS/VOLT/NEUTRAL/RESET post one msg::MotorCommand to
//     bb.motorIn[port-1] (a per-port Mailbox, Decision 2) ON ACCEPTANCE --
//     acceptance is still pre-validated against bb.motorCaps[port-1] (a
//     boot-time snapshot of Hal::Motor::capabilities(), since capabilities
//     never change at runtime for any current concrete leaf -- see
//     blackboard.h's file header), so a capability-rejected command (e.g.
//     VOLT on Nezha) never posts anything and so never steals authority,
//     exactly as before.
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
//     dedicated Mailbox<msg::MotorCommand> -- NOT bb.motorIn[], since a
//     broadcast deliberately does NOT mark any port in-use, a semantic
//     bb.motorIn[]'s per-port drain does NOT preserve -- see
//     NezhaHardware::apply(const Hal::CommandProcessorToHardwareCommand&)'s
//     own "broadcast never marks a port in-use" branch); its Drivetrain-side
//     {NEUTRAL,standby=true} posts to bb.driveIn like any other DEV DT-shaped
//     command. DEV DT STOP's narrower, addressed (bound-pair-only) neutral
//     posts to bb.motorIn[left-1]/bb.motorIn[right-1] directly (both ports
//     ARE marked in-use for an addressed, non-broadcast neutral -- identical
//     to bb.motorIn[]'s own marking -- see NezhaHardware::apply()'s addressed
//     branch), so no separate cell is needed for that case.
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
//   - STATE/CAPS/DEV STATE are pure reads against bb.motor[]/bb.motorCaps[]/
//     bb.drivetrain/bb.drivetrainConfig -- never touch any queue.
//
// --- Serial-silence watchdog -- NON-NEGOTIABLE (runaway history) ---
// SerialSilenceWatchdog itself is UNCHANGED by this rewrite (still a small,
// dependency-free value class) -- only its OWNER changes: it is no longer
// embedded in a deleted DevLoopState, it is a loop-owned instance (main.cpp/
// sim_api.cpp), fed every pass the loop ingests any statement (regardless of
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
// ---------------------------------------------------------------------------

#if ROBOT_DEV_BUILD

#include <stdint.h>
#include <vector>

#include "command_types.h"
#include "hal/capability/hal_command.h"
#include "runtime/command_router.h"

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
  // an uninitialized lastFeedMs_) and again every time a statement line
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
// systemCommands()'s free-function shape (system_commands.h).
std::vector<CommandDescriptor> devCommands(Rt::CommandRouter& router);

#endif  // ROBOT_DEV_BUILD
