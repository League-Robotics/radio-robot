// ---------------------------------------------------------------------------
// main.cpp -- the dev loop (077-005): Communicator tick -> dispatch -> HAL
// tick -> (if bound) Drivetrain tick/apply -> bound-motor tick ->
// serial-silence watchdog check.
//
// This supersedes 077-001/003/004's smoke wiring: the DEV command family
// (commands/dev_commands.*) is now registered alongside the liveness family
// (system_commands.*), and the HAL/Drivetrain instances built here are
// actually driven by DEV M / DEV DT rather than sitting untouched.
//
// Comms are a subsystem now (Subsystems::Communicator,
// subsystems/communicator.h): its tick(now) latches at most ONE complete
// statement per iteration (void return -- held/taken via
// hasStatement()/takeStatement(), the edge's returnPath routes the reply).
// The old free-function pollComms() and the stack line buffers it was
// threaded through are gone -- the Communicator internalizes the drivers
// and the line buffer. (079-002: tick() reshaped from returning the edge to
// this held/taken pair -- see communicator.h's held-output contract.)
//
// Loop shape matches the locked sketch in clasi/sprints/077-greenfield-
// faceplate-hal-drivetrain-and-dev-bench-system/issues/greenfield-rebuild-
// faceplate-hal-in-a-fresh-source-old-tree-parked.md, Step 5 (comms poll
// since folded into the Communicator subsystem's tick):
//
//   comm.tick(now);                // held/taken; at most one line latched
//   if (comm.hasStatement()) {
//       in = comm.takeStatement(); // dispatch via CommandProcessor
//   }
//   hal.tick(now);                // split-phase encoder schedule, all 4 ports
//   if (drivetrainActive) {
//       auto out = drivetrain.tick(now, left.state(), right.state());
//       left.apply(out.left);
//       right.apply(out.right);
//   }
//   left.tick(now);                // staged commands execute (PID runs here)
//   right.tick(now);
//   watchdog.check(now);          // silence -> all neutral
//
// `left`/`right` are whichever two NezhaMotors DEV DT PORTS last bound
// (DevLoopState::leftPort/rightPort, default 1/2) -- this loop never
// hardcodes which ports they are. Note the bound pair is ticked TWICE per
// iteration (once inside hal.tick()'s uniform 4-port sweep, once more
// explicitly here) so a freshly drivetrain-governed target actuates in the
// SAME cycle it was computed, rather than waiting for next iteration's
// hal.tick() -- this is the locked shape, not an oversight.
//
// Build gating: the DEV family (and the HAL/Drivetrain wiring it needs) is
// compiled in only when ROBOT_DEV_BUILD is set (codal.json's "config"
// object; see dev_commands.h's file header for the full rationale). This
// sprint's codal.json sets it to 1 for this tree -- there is no production
// loop yet, so the #else branch below is a minimal liveness-only fallback
// (PING/VER/HELP/ECHO/ID, no HAL) proving the gate is a real fork, not
// decoration.
// ---------------------------------------------------------------------------

#include "MicroBit.h"
#include "subsystems/communicator.h"
#include "commands/command_processor.h"
#include "commands/system_commands.h"

#if ROBOT_DEV_BUILD
#include "com/i2c_bus.h"
#include "hal/nezha/nezha_hal.h"
#include "subsystems/drivetrain.h"
#include "commands/dev_commands.h"
#include "messages/motor.h"
#include "messages/drivetrain.h"
#include <vector>
#endif

static MicroBit uBit;

// ---------------------------------------------------------------------------
// serialReply / radioReply -- reply adapters. CommandProcessor calls one of
// these per response line, routing the reply back out on whichever channel
// (serial or radio) the command arrived on. Both take the Communicator as
// ctx and build on its primitive sends.
// ---------------------------------------------------------------------------
static void serialReply(const char* msg, void* ctx) {
    static_cast<Subsystems::Communicator*>(ctx)->sendSerial(msg);
}

static void radioReply(const char* msg, void* ctx) {
    static_cast<Subsystems::Communicator*>(ctx)->sendRadio(msg);
}

#if ROBOT_DEV_BUILD

// ---------------------------------------------------------------------------
// initDefaultMotorConfigs -- one msg::MotorConfig per port (1..4). No
// source/robot/ (RobotConfig/ConfigRegistry) exists in this tree this sprint
// (architecture-update.md Step 1), so these are bench placeholders, not a
// per-robot calibration load: fwd_sign=+1 and travel_calib=0.487 mm/deg (the
// legacy firmware's ml/mr default, docs/protocol-v2.md's Named Key Table) so
// DEV M <n> STATE reports a plausible non-zero position/velocity out of the
// box. `DEV M <n> CFG` is exactly the mechanism to correct these live over
// the wire once a specific motor's real calibration is known (ticket 7).
//
// vel_gains/vel_filt_alpha (077-007): ticket 7's HITL bench pass found the
// embedded velocity PID (ticket 3) could never actually close the loop with
// an all-zero boot default -- not just because kp/ki/kff were 0 (expected,
// tunable live via `DEV M <n> CFG`), but because vel_filt_alpha was ALSO 0,
// which is the EMA coefficient in NezhaMotor::tick()'s
// `filteredVelocity_ = a * rawVel + (1 - a) * filteredVelocity_` --
// a=0 means filteredVelocity_ never incorporates a new sample and reports
// exactly 0 forever, regardless of real motion (confirmed live: position
// climbed under `DEV M 1 VEL 120` while `vel=` stayed pinned at 0.0 with
// vel_filt_alpha=0; setting vel_filt_alpha=0.3 over `DEV M 1 CFG` on the
// SAME motor immediately produced real, converging vel= readings). This is
// a silent-failure gap no unit test could have caught (ticket 3/4's scope
// never bench-tested a live velocity reading) -- exactly the kind of defect
// this ticket's bench pass exists to catch. Bench-tuned on the stand
// (Tovez, ports 1/3, targets 120/150/-100 mm/s): converges within ~1.5 s,
// small (~10%) overshoot, holds within the dev_exercise.py/pid_hold_speed.py
// tolerance bands (see this ticket's results section for the recorded step
// responses). Still bench placeholders like travel_calib above -- `DEV M
// <n> CFG` remains the live-correction mechanism for a specific motor's
// real tuning.
// ---------------------------------------------------------------------------
static msg::MotorConfig defaultMotorConfigs[Hal::NezhaHal::kPortCount];

static void initDefaultMotorConfigs() {
    msg::Gains velGains;
    velGains.kp = 0.0022f;
    velGains.ki = 0.0018f;
    velGains.kff = 0.0038f;
    velGains.i_max = 0.3f;

    // reversal_dwell / output_deadband (sprint 078) are left unset (.has ==
    // false) here on purpose -- Hal::Motor::configure() (078-002) applies
    // the real ship defaults (100 ms / 0.03) whenever a config arrives
    // unset; that is the one place those defaults live.
    for (uint32_t i = 0; i < Hal::NezhaHal::kPortCount; ++i) {
        defaultMotorConfigs[i] = msg::MotorConfig();
        defaultMotorConfigs[i].setPort(i + 1);
        defaultMotorConfigs[i].setFwdSign(1);
        defaultMotorConfigs[i].setTravelCalib(0.487f);   // [mm/deg] bench placeholder
        defaultMotorConfigs[i].setVelGains(velGains);
        defaultMotorConfigs[i].setVelFiltAlpha(0.3f);    // EMA coeff -- see comment above
    }
}

#endif  // ROBOT_DEV_BUILD

int main() {
    uBit.init();
    uBit.i2c.setFrequency(100000);

    // Comms: the Communicator subsystem (serial + radio, both enabled).
    // radio_channel.cpp (persisted boot channel selection, button-edit UI) is
    // not copied to this tree -- a default CommunicatorConfig's zero
    // radio_channel == radiochan::kDefault, so the radio simply comes up on
    // the relay's default channel every boot.
    static Subsystems::Communicator comm(uBit.serial, uBit.radio, uBit.messageBus);
    comm.configure(msg::CommunicatorConfig());
    comm.begin();

#if ROBOT_DEV_BUILD
    // --- HAL: one NezhaMotor per port (1-4) over the shared I2CBus. ---
    initDefaultMotorConfigs();
    static I2CBus i2cBus(uBit.i2c);
    static Hal::NezhaHal hal(i2cBus, defaultMotorConfigs);
    hal.begin();

    // --- Drivetrain: differential (Tovez), bench-placeholder trackwidth. ---
    // sync_gain is deliberately left at its zero default here (governor OFF
    // at boot) -- ticket 7's HITL bench pass found no live way to turn it on
    // short of a reflash and added `DEV DT CFG sync_gain=...`
    // (commands/dev_commands.cpp) for exactly that; bench scripts that need
    // the governor set it explicitly over the wire rather than this getting
    // a nonzero boot default.
    static Subsystems::Drivetrain drivetrain;
    msg::DrivetrainConfig dtConfig;
    dtConfig.setTrackwidth(128.0f);   // [mm] bench placeholder -- tuned in ticket 7
    drivetrain.configure(dtConfig);

    // --- Dev loop shared state: watchdog + DEV command wiring. ---
    static SerialSilenceWatchdog watchdog;

    static DevLoopState devState;
    devState.hal = &hal;
    devState.drivetrain = &drivetrain;
    devState.watchdog = &watchdog;
    // Seed the CFG-delta shadow so the first `DEV M <n> CFG kp=...` merges
    // onto the SAME calibration the motor was actually constructed with,
    // rather than an all-zero blank (see DevLoopState's field comment).
    for (uint32_t i = 0; i < Hal::NezhaHal::kPortCount; ++i) {
        devState.motorConfigShadow[i] = defaultMotorConfigs[i];
    }
    // Seed the drivetrain CFG-delta shadow the same way, so the first
    // `DEV DT CFG sync_gain=...` merges onto the SAME dtConfig the
    // Drivetrain was actually configured with above (trackwidth=128),
    // rather than an all-zero blank -- see DevLoopState's field comment.
    devState.drivetrainConfigShadow = dtConfig;
    // Prime the capabilities cache for the default DEV DT PORTS binding
    // (1, 2) -- see drivetrain.h's setMotorCapabilities() doc comment.
    drivetrain.setMotorCapabilities(hal.motor(devState.leftPort).capabilities(),
                                     hal.motor(devState.rightPort).capabilities());

    // --- Command table: liveness (PING/VER/HELP/ECHO/ID) + DEV. ---
    std::vector<CommandDescriptor> allCommands = systemCommands();
    std::vector<CommandDescriptor> dev = devCommands(devState);
    allCommands.insert(allCommands.end(), dev.begin(), dev.end());
    static CommandProcessor cmd(allCommands);
    cmd.setSerialReply(serialReply, &comm);

    // Start the serial-silence watchdog's window counting from boot (see
    // SerialSilenceWatchdog::feed()'s doc comment) rather than from an
    // uninitialized "last command" time.
    watchdog.feed(uBit.systemTime());

    while (true) {
        uint32_t now = uBit.systemTime();

        comm.tick(now);
        if (comm.hasStatement()) {
            Subsystems::CommunicatorToCommandProcessorStatement in = comm.takeStatement();
            // Feed on any line, either channel, regardless of content or
            // dispatch outcome -- see dev_commands.h's watchdog contract.
            watchdog.feed(now);
            cmd.process(in.line,
                        in.returnPath == Subsystems::Channel::RADIO ? radioReply
                                                                    : serialReply,
                        &comm);
        }

        hal.tick(now);

        if (devState.drivetrainActive) {
            Subsystems::DrivetrainToMotorCommand out = drivetrain.tick(
                now,
                hal.motor(devState.leftPort).state(),
                hal.motor(devState.rightPort).state());
            hal.motor(devState.leftPort).apply(out.left);
            hal.motor(devState.rightPort).apply(out.right);
        }

        // Bound pair ticks again here (on top of hal.tick()'s uniform sweep
        // above) so a fresh drivetrain-governed target actuates THIS cycle --
        // see the file header for why this is the locked shape, not a bug.
        hal.motor(devState.leftPort).tick(now);
        hal.motor(devState.rightPort).tick(now);

        if (watchdog.check(now)) {
            neutralizeAll(devState);
            char wbuf[32];
            CommandProcessor::replyEvt(wbuf, sizeof(wbuf), "dev_watchdog", nullptr,
                                       serialReply, &comm);
        }
    }
#else
    // ROBOT_DEV_BUILD == 0: no production loop exists yet this sprint (see
    // the file header) -- this minimal liveness-only fallback (no HAL, no
    // DEV, no watchdog) is what proves the ROBOT_DEV_BUILD gate is a real
    // fork rather than a decorative #if 1.
    static CommandProcessor cmd(systemCommands());
    cmd.setSerialReply(serialReply, &comm);

    while (true) {
        uint32_t now = uBit.systemTime();

        comm.tick(now);
        if (comm.hasStatement()) {
            Subsystems::CommunicatorToCommandProcessorStatement in = comm.takeStatement();
            cmd.process(in.line,
                        in.returnPath == Subsystems::Channel::RADIO ? radioReply
                                                                    : serialReply,
                        &comm);
        }
    }
#endif

    return 0;
}
