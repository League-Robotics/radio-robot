// ---------------------------------------------------------------------------
// main.cpp -- the dev loop (077-005): comms poll -> HAL tick -> (if bound)
// Drivetrain tick/apply -> bound-motor tick -> serial-silence watchdog check.
//
// This supersedes 077-001/003/004's smoke wiring: the DEV command family
// (commands/dev_commands.*) is now registered alongside the liveness family
// (system_commands.*), and the HAL/Drivetrain instances built here are
// actually driven by DEV M / DEV DT rather than sitting untouched.
//
// Loop shape matches the locked sketch in clasi/sprints/077-greenfield-
// faceplate-hal-drivetrain-and-dev-bench-system/issues/greenfield-rebuild-
// faceplate-hal-in-a-fresh-source-old-tree-parked.md, Step 5, exactly:
//
//   pollComms();                  // dispatch DEV/PING via CommandProcessor
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
#include "communicator.h"
#include "radio_channel.h"
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
// (serial or radio) the command arrived on.
// ---------------------------------------------------------------------------
static void serialReply(const char* msg, void* ctx) {
    static_cast<SerialPort*>(ctx)->send(msg);
}

static void radioReply(const char* msg, void* ctx) {
    static_cast<Radio*>(ctx)->send(msg);
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
// ---------------------------------------------------------------------------
static msg::MotorConfig defaultMotorConfigs[Hal::NezhaHal::kPortCount];

static void initDefaultMotorConfigs() {
    for (uint32_t i = 0; i < Hal::NezhaHal::kPortCount; ++i) {
        defaultMotorConfigs[i] = msg::MotorConfig();
        defaultMotorConfigs[i].setPort(i + 1);
        defaultMotorConfigs[i].setFwdSign(1);
        defaultMotorConfigs[i].setTravelCalib(0.487f);   // [mm/deg] bench placeholder
    }
}

// ---------------------------------------------------------------------------
// pollComms -- drains one line from each comms channel that has one ready,
// feeding the serial-silence watchdog on every line that arrives (regardless
// of content or dispatch outcome -- see dev_commands.h's watchdog contract)
// and dispatching it through the shared CommandProcessor.
// ---------------------------------------------------------------------------
static void pollComms(Communicator& comm, CommandProcessor& cmd,
                      SerialSilenceWatchdog& watchdog, uint32_t now,
                      char* serialLine, int serialLineSize,
                      char* radioLine, int radioLineSize) {
    if (comm.serial().readLine(serialLine, static_cast<uint16_t>(serialLineSize))) {
        watchdog.feed(now);
        cmd.process(serialLine, serialReply, &comm.serial());
    }
    if (comm.radio().poll(radioLine, static_cast<uint16_t>(radioLineSize))) {
        watchdog.feed(now);
        cmd.process(radioLine, radioReply, &comm.radio());
    }
}

#endif  // ROBOT_DEV_BUILD

int main() {
    uBit.init();
    uBit.i2c.setFrequency(100000);

    // Comms: serial + radio, both enabled. radio_channel.cpp (persisted boot
    // channel selection, button-edit UI) is not copied this ticket -- the
    // radio simply comes up on radiochan::kDefault every boot.
    static Communicator comm(uBit.serial, uBit.radio, uBit.messageBus);
    comm.begin(radiochan::kDefault);

#if ROBOT_DEV_BUILD
    // --- HAL: one NezhaMotor per port (1-4) over the shared I2CBus. ---
    initDefaultMotorConfigs();
    static I2CBus i2cBus(uBit.i2c);
    static Hal::NezhaHal hal(i2cBus, defaultMotorConfigs);
    hal.begin();

    // --- Drivetrain: differential (Tovez), bench-placeholder trackwidth. ---
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
    // Prime the capabilities cache for the default DEV DT PORTS binding
    // (1, 2) -- see drivetrain.h's setMotorCapabilities() doc comment.
    drivetrain.setMotorCapabilities(hal.motor(devState.leftPort).capabilities(),
                                     hal.motor(devState.rightPort).capabilities());

    // --- Command table: liveness (PING/VER/HELP/ECHO/ID) + DEV. ---
    std::vector<CommandDescriptor> allCommands = systemCommands();
    std::vector<CommandDescriptor> dev = devCommands(devState);
    allCommands.insert(allCommands.end(), dev.begin(), dev.end());
    static CommandProcessor cmd(allCommands);
    cmd.setSerialReply(serialReply, &comm.serial());

    char serialLine[256];
    char radioLine[256];

    // Start the serial-silence watchdog's window counting from boot (see
    // SerialSilenceWatchdog::feed()'s doc comment) rather than from an
    // uninitialized "last command" time.
    watchdog.feed(uBit.systemTime());

    while (true) {
        uint32_t now = uBit.systemTime();

        pollComms(comm, cmd, watchdog, now, serialLine, sizeof(serialLine),
                 radioLine, sizeof(radioLine));

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
                                       serialReply, &comm.serial());
        }
    }
#else
    // ROBOT_DEV_BUILD == 0: no production loop exists yet this sprint (see
    // the file header) -- this minimal liveness-only fallback (no HAL, no
    // DEV) is what proves the ROBOT_DEV_BUILD gate is a real fork rather
    // than a decorative #if 1.
    static CommandProcessor cmd(systemCommands());
    cmd.setSerialReply(serialReply, &comm.serial());

    char serialLine[256];
    char radioLine[256];

    while (true) {
        if (comm.serial().readLine(serialLine, sizeof(serialLine))) {
            cmd.process(serialLine, serialReply, &comm.serial());
        }
        if (comm.radio().poll(radioLine, sizeof(radioLine))) {
            cmd.process(radioLine, radioReply, &comm.radio());
        }
    }
#endif

    return 0;
}
