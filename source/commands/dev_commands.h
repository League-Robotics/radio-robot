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
// <n> is always a motor PORT (1..4), matching how NezhaHal instantiates one
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
// drops drivetrain AUTHORITY (drivetrainActive=false) and commands hardware
// neutral, but never touches DevLoopState::leftPort/rightPort themselves.
//
// --- Authority arbitration ---
// This firmware runs only the dev loop (no planner) -- DEV M's motion verbs
// (DUTY/VEL/POS/VOLT/NEUTRAL/RESET) deactivate DevLoopState::drivetrainActive
// on ACCEPTANCE (a capability-rejected command, e.g. VOLT on Nezha, does not
// steal authority since it never actually touched the motor); DEV DT's
// motion verbs (VW/WHEELS/NEUTRAL) reactivate it. Queries (STATE/CAPS) and
// CFG never touch authority. `DEV STOP`/the watchdog fully drop authority;
// `DEV DT STOP` drops it too, but scoped to only the bound pair. This is the
// ONLY authority conflict this firmware has, and is trivial to verify by
// reading main.cpp's loop plus the handlers below.
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
// acceptance criteria). On expiry, main.cpp calls neutralizeAll() (ALL four
// motors -> neutral, drivetrain -> idle, drivetrainActive cleared) and emits
// `EVT dev_watchdog` on serial -- see docs/protocol-v2.md's "Development
// commands" section for the wire contract.
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
#include "hal/nezha/nezha_hal.h"
#include "subsystems/drivetrain.h"
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
// main.cpp's loop also reads leftPort/rightPort/drivetrainActive directly
// each iteration -- this struct IS the "which two ports does 'left'/'right'
// mean right now, and who has authority" single source of truth the issue's
// loop sketch refers to.
//
// motorConfigShadow[] exists because Hal::Motor exposes configure() (a full
// replace) but no getter for the CURRENTLY configured msg::MotorConfig --
// `DEV M <n> CFG k=v ...` is a DELTA (only the named keys change), so this
// per-port shadow copy is the read-modify-write staging area: main.cpp seeds
// it with the same configs passed to NezhaHal's constructor, and every CFG
// command mutates shadow[port-1] in place before calling motor.configure().
//
// drivetrainConfigShadow exists for the identical reason, one level up:
// Subsystems::Drivetrain::configure() is also a full replace with no getter
// for the live msg::DrivetrainConfig, so `DEV DT CFG k=v ...` (077-007,
// added to close a gap found in this ticket's HITL bench pass -- sync_gain
// booted at 0 with no live setter, so the ratio governor could never be
// turned on without a reflash) is a delta against this single shared shadow
// (one Drivetrain instance, not per-port). main.cpp seeds it with the same
// msg::DrivetrainConfig passed to Drivetrain::configure() at boot.
// ---------------------------------------------------------------------------
struct DevLoopState {
  Hal::NezhaHal* hal = nullptr;
  Subsystems::Drivetrain* drivetrain = nullptr;
  SerialSilenceWatchdog* watchdog = nullptr;   // set by main.cpp; DEV WD's target

  uint32_t leftPort = 1;    // DEV DT PORTS binding -- default drive pair
  uint32_t rightPort = 2;   // (coupled bench rig: DEV DT PORTS 3 4)
  bool drivetrainActive = false;

  msg::MotorConfig motorConfigShadow[Hal::NezhaHal::kPortCount] = {};
  msg::DrivetrainConfig drivetrainConfigShadow = {};
};

// ---------------------------------------------------------------------------
// neutralizeAll -- commands every HAL motor (not just the DT-bound pair) and
// the Drivetrain to neutral, and drops drivetrain authority. Shared by the
// top-level `DEV STOP` handler and main.cpp's serial-silence watchdog fire
// path so there is exactly one "make everything safe" code path to audit.
// ---------------------------------------------------------------------------
void neutralizeAll(DevLoopState& state, msg::Neutral mode = msg::Neutral::BRAKE);

// ---------------------------------------------------------------------------
// neutralizeDrivetrain -- the narrower `DEV DT STOP` action: neutrals only
// the Drivetrain and its currently-bound pair, drops drivetrain authority,
// but leaves any OTHER motor (independently under DEV M control) untouched.
// ---------------------------------------------------------------------------
void neutralizeDrivetrain(DevLoopState& state, msg::Neutral mode = msg::Neutral::BRAKE);

// Returns the DEV command table (DEV M, DEV DT, DEV STATE, DEV STOP, DEV WD),
// bound to the given shared state (state.watchdog must be set before this is
// called -- DEV WD dereferences it). Mirrors systemCommands()'s free-function
// shape (system_commands.h) -- state must outlive every call this table's
// handlers make (main.cpp holds it as a function-static, matching hal/comm).
std::vector<CommandDescriptor> devCommands(DevLoopState& state);

#endif  // ROBOT_DEV_BUILD
