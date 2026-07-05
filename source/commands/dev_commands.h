#pragma once

// ---------------------------------------------------------------------------
// dev_commands.h -- the DEV command family (077-005): the only command
// family beyond bare liveness (PING/VER/HELP/ECHO/ID, system_commands.*)
// this dev-bench firmware registers. DEV drives individual Hal::Motors and,
// through Subsystems::Drivetrain, a bound motor-port pair, all the way
// through the message plane (apply()/configure()/state()/capabilities()) --
// never a primitive setter called directly from this file.
//
// Full vocabulary (ported from the locked table in clasi/sprints/077-
// greenfield-faceplate-hal-drivetrain-and-dev-bench-system/issues/greenfield-
// rebuild-faceplate-hal-in-a-fresh-source-old-tree-parked.md, Step 5):
//
//   DEV M <n> DUTY <duty>       -- [%, -100..100]  motor.apply(duty_cycle)
//   DEV M <n> VEL <velocity>    -- [mm/s] signed    embedded PID closes the loop
//   DEV M <n> POS <position>    -- [deg]            onboard absolute-angle move
//   DEV M <n> VOLT <voltage>    -- [V]              ERR unsupported (Nezha: no voltage mode)
//   DEV M <n> NEUTRAL <B|C>     -- brake or coast
//   DEV M <n> RESET             -- zero the encoder (MotorCommand.reset_position)
//   DEV M <n> STATE             -- OK DEV M <n> pos=.. vel=.. applied=.. wedged=.. conn=..
//   DEV M <n> CAPS              -- OK DEV M <n> duty=.. volt=.. vel=.. pos=.. enc=..
//   DEV M <n> CFG k=v ...       -- motor.configure() delta (kp=0.8 slew=400 ...)
//   DEV DT PORTS <left> <right> -- bind Drivetrain to a motor-port pair
//   DEV DT VW <vx> <vy> <omega> -- [mm/s mm/s rad/s] body twist (ratio-governed)
//   DEV DT WHEELS <left> <right>-- [mm/s] per-wheel velocity targets (ratio-governed)
//   DEV DT NEUTRAL <B|C>
//   DEV DT STATE
//   DEV DT STOP                 -- drivetrain-scoped stop: bound pair + drivetrain idle
//   DEV DT CFG k=v ...          -- drivetrain.configure() delta (sync_gain=0.8 trackwidth=128 ...)
//   DEV STATE                   -- everything: one line per motor + drivetrain
//   DEV STOP                    -- all four motors neutral, drivetrain idle
//   DEV WD <window>             -- [ms] set the serial-silence watchdog window
//
// <n> is always a motor PORT (1..4), matching how NezhaHardware instantiates one
// NezhaMotor per port (ticket 3) -- never an L/R role name.
//
// --- Open Question 3 (argument parsing mechanism) -- RESOLVED ---
// `DEV M <n> <mode> ...` and `DEV DT <mode> ...` mix a positional port/mode
// token with either a positional value or free-form `k=v` pairs, which does
// not fit the declarative, fixed-shape ArgSchema (the `<n>`/`<mode>` tokens
// sit BEFORE the registered literal prefix can end -- the port number varies,
// so the command table can only register the literal prefixes "DEV M" and
// "DEV DT"; everything after is parsed by hand). This file therefore
// hand-rolls one ParseFn per literal prefix ("DEV M", "DEV DT") that reads
// the port/mode tokens itself and shapes an ArgList for the shared handler,
// EXCEPT "DEV WD <window>", a pure fixed-shape `<verb> <int>` command, which
// uses ArgSchema (mixed approach explicitly sanctioned by the ticket's
// acceptance criteria). `DEV M <n> CFG k=v ...`'s arbitrary-count key=value
// pairs are threaded through by re-serializing each KVPair back into a
// "key=value" STR Argument (ArgList has no separate kv channel reaching the
// handler) -- see dev_commands.cpp's parseDevM(). `DEV DT CFG k=v ...`
// (077-007) reuses the identical re-serialization mechanism in parseDevDt().
//
// --- Open Question 4 (bench-rig port binding persistence) -- RESOLVED ---
// `DEV DT PORTS <left> <right>` persists across `DEV STOP` and a watchdog
// neutral event; it resets only on reboot. Neutralizing (STOP or watchdog)
// drops drivetrain AUTHORITY (Subsystems::Drivetrain::standby(), sprint 079)
// and commands hardware neutral, but never touches the port binding itself
// (`DrivetrainConfig.left_port`/`right_port`, read via `drivetrain->ports()`
// -- sprint 079 moved the binding off `DevLoopState` and into config, see
// below).
//
// --- Authority arbitration (sprint 079 reshape) ---
// This firmware runs only the dev loop (no planner) -- authority ("am I the
// one actually driving my bound pair right now") is now owned by
// `Subsystems::Drivetrain` itself (`active()`/`standby()`), not by a
// `DevLoopState` flag. `DEV M`'s motion verbs (DUTY/VEL/POS/VOLT/NEUTRAL/
// RESET) stage a standby-only `msg::DrivetrainCommand`
// ({control_kind=NONE, standby=true}) into the Drivetrain outbox ON
// ACCEPTANCE, but ONLY when the targeted port is one of the Drivetrain's
// currently-bound `ports()` (a capability-rejected command, e.g. VOLT on
// Nezha, never touched the motor and so never steals authority; an
// independent, unbound motor is unrelated to the Drivetrain and leaves its
// outbox untouched). `DEV DT`'s motion verbs (VW/WHEELS/NEUTRAL) stage a
// command that reactivates authority once `main.cpp` applies it (see
// `Subsystems::Drivetrain::apply()`'s oneof dispatch). Queries (STATE/CAPS)
// and CFG never touch authority or either outbox. `DEV STOP`/the watchdog
// fully drop authority (broadcast HAL neutral + `{NEUTRAL, standby=true}`);
// `DEV DT STOP` drops it too, but scoped to only the bound pair (addressed,
// not broadcast, HAL command). This is the ONLY authority conflict this
// firmware has, and is trivial to verify by reading main.cpp's loop plus the
// handlers below.
//
// --- Serial-silence watchdog -- NON-NEGOTIABLE (runaway history) ---
// SerialSilenceWatchdog tracks the wall-clock time of the last COMMAND LINE
// ARRIVAL on either comms channel (serial or radio) -- fed by main.cpp's
// pollComms() on every line, regardless of its content or whether it parsed
// to a known verb. This is deliberately NOT the legacy CMD_MOTION_WATCHDOG
// flag mechanism (which only certain command kinds reset): a silent host is
// a silent host, whether it stopped sending PING or DEV DT VW. Default
// window is kDefaultWindow (~1 s); `DEV WD <window>` is the mechanism that
// changes it at runtime (settable, not hardcoded-only, per the ticket's
// acceptance criteria). On expiry, main.cpp applies `buildBroadcastNeutral()`/
// `buildDrivetrainStop()` (see below) IMMEDIATELY -- not via the outbox,
// since main.cpp is the top of the call tree -- and emits `EVT dev_watchdog`
// on serial -- see docs/protocol-v2.md's "Development commands" section for
// the wire contract.
//
// --- Build gating ---
// The locked spec says DEV compiles out of production firmware. This sprint
// the new source/ tree IS the dev firmware (no production loop exists yet),
// so the whole family is gated behind the ROBOT_DEV_BUILD config define
// (codal.json's "config" object -> force-included into every TU as a
// preprocessor #define, same mechanism as MICROBIT_BLE_ENABLED) rather than
// physically deleted -- this ticket sets it to 1 (ON) for this tree. A
// future production firmware flips it to 0 and this whole file (and its
// wiring in main.cpp) compiles to nothing.
// ---------------------------------------------------------------------------

#if ROBOT_DEV_BUILD

#include <stdint.h>
#include <vector>

#include "command_types.h"
#include "hal/capability/hal_command.h"
#include "subsystems/nezha_hardware.h"
#include "subsystems/drivetrain.h"
#include "messages/drivetrain.h"
#include "messages/motor.h"

// ---------------------------------------------------------------------------
// SerialSilenceWatchdog -- see the file-level comment above.
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
  // return false until the next feed() re-arms it, so main.cpp's caller
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
// DevLoopState -- the dev loop's shared wiring, owned by main.cpp (as
// function-static instances, per this tree's existing main.cpp convention)
// and handed to devCommands() as the handlerCtx every DEV descriptor shares.
//
// Sprint 079 reshape (architecture-update.md "The processor is DevLoopState
// + CommandProcessor", "Config-plane vs. command-plane"): handlers no longer
// call Hal::Motor/Subsystems::Drivetrain write methods directly. Setpoint-
// shaped verbs (DEV M's DUTY/VEL/POS/VOLT/NEUTRAL/RESET; DEV DT's VW/WHEELS/
// NEUTRAL/STOP; DEV STOP) pre-validate against read-only capabilities() and
// STAGE into the outbox fields below; main.cpp (ticket 079-005) is the sole
// drainer, calling hardware.apply()/drivetrain.apply() once per pass -- see "The
// Part-2 loop". `leftPort`/`rightPort`/`drivetrainActive` are GONE -- the
// port binding moved into DrivetrainConfig (read via `drivetrain->ports()`)
// and authority arbitration moved into Drivetrain itself
// (`drivetrain->active()`/`->standby()`) per decision 8.
//
// hal/drivetrain/watchdog pointers, and motorConfigShadow[]/
// drivetrainConfigShadow, are UNCHANGED -- still needed for reads (STATE/
// CAPS), config-plane writes (CFG/PORTS/WD stay direct, parse-time calls --
// see the config-plane/command-plane table in architecture-update.md), and
// capability-cache refresh.
//
// motorConfigShadow[] exists because Hal::Motor exposes configure() (a full
// replace) but no getter for the CURRENTLY configured msg::MotorConfig --
// `DEV M <n> CFG k=v ...` is a DELTA (only the named keys change), so this
// per-port shadow copy is the read-modify-write staging area: main.cpp seeds
// it with the same configs passed to NezhaHardware's constructor, and every CFG
// command mutates shadow[port-1] in place before calling motor.configure().
//
// drivetrainConfigShadow exists for the identical reason, one level up:
// Subsystems::Drivetrain::configure() is also a full replace with no getter
// for the live msg::DrivetrainConfig, so `DEV DT CFG k=v ...` and
// `DEV DT PORTS <left> <right>` (both config-plane, sprint 079) are deltas
// against this single shared shadow (one Drivetrain instance, not
// per-port). main.cpp seeds it with the same msg::DrivetrainConfig passed to
// Drivetrain::configure() at boot.
// ---------------------------------------------------------------------------
struct DevLoopState {
  Subsystems::NezhaHardware* hardware = nullptr;
  Subsystems::Drivetrain* drivetrain = nullptr;
  SerialSilenceWatchdog* watchdog = nullptr;   // set by main.cpp; DEV WD's target

  // The outbox (sprint 079) -- setpoint-shaped DEV M/DEV DT verbs stage
  // here; main.cpp drains hasHardwareCommand/hasDrivetrainCommand once per pass.
  // Latest-wins: staging again before a drain overwrites the held value.
  bool hasHardwareCommand = false;
  Hal::CommandProcessorToHardwareCommand hardwareCommand = {};
  bool hasDrivetrainCommand = false;
  msg::DrivetrainCommand drivetrainCommand = {};

  msg::MotorConfig motorConfigShadow[Subsystems::NezhaHardware::kPortCount] = {};
  msg::DrivetrainConfig drivetrainConfigShadow = {};
};

// ---------------------------------------------------------------------------
// buildBroadcastNeutral / buildDrivetrainStop -- the one audited "make
// everything safe" construction path (architecture-update.md "The Part-2
// loop"), replacing 077's neutralizeAll()/neutralizeDrivetrain() free
// functions now that HAL/Drivetrain writes are staged through DevLoopState's
// outbox rather than called directly from a handler. Shared by:
//   - `DEV STOP`'s handler, which STAGES the result into the outbox (it
//     runs from inside a parsed statement and must respect the held/taken
//     discipline like every other setpoint-shaped verb).
//   - main.cpp's serial-silence watchdog-fire path, which APPLIES the
//     result IMMEDIATELY (main.cpp is the top of the call tree, not a
//     subsystem; an emergency stop gains nothing from an extra pass of
//     outbox latency -- see architecture-update.md's narrow, documented
//     exception to "never call apply() outside main/the HAL").
// `DEV DT STOP` (the narrower, bound-pair-only stop) reuses
// buildDrivetrainStop() for its Drivetrain-side neutral+standby (identical
// shape to DEV STOP's) but builds its own addressed, non-broadcast HAL
// command -- see handleDevDt's STOP case in dev_commands.cpp.
// ---------------------------------------------------------------------------
Hal::CommandProcessorToHardwareCommand buildBroadcastNeutral(msg::Neutral mode = msg::Neutral::BRAKE);
msg::DrivetrainCommand buildDrivetrainStop(msg::Neutral mode = msg::Neutral::BRAKE);

// Returns the DEV command table (DEV M, DEV DT, DEV STATE, DEV STOP, DEV WD),
// bound to the given shared state (state.watchdog must be set before this is
// called -- DEV WD dereferences it). Mirrors systemCommands()'s free-function
// shape (system_commands.h) -- state must outlive every call this table's
// handlers make (main.cpp holds it as a function-static, matching hal/comm).
std::vector<CommandDescriptor> devCommands(DevLoopState& state);

#endif  // ROBOT_DEV_BUILD
