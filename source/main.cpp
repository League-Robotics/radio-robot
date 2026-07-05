// ---------------------------------------------------------------------------
// main.cpp -- the dev loop: sprint 079's three-beat "feed it, tick it, ask
// it" shape (architecture-update.md "The Part-2 loop"; issue's Part 2).
//
// This supersedes 077-001/003/004's smoke wiring and 079-004's minimal loop
// adaptation: CommandProcessor/DevLoopState are now a pure transformer
// (statements in, commands + replies out -- see dev_commands.h) and
// main.cpp is the SOLE caller of Subsystems::NezhaHardware::apply()/
// Subsystems::Drivetrain::apply() for anything DEV-sourced, draining
// DevLoopState's outbox once per pass instead of DEV handlers calling
// either subsystem's write methods directly.
//
// The loop, exactly per architecture-update.md's "The Part-2 loop":
//
//   hardware.tick(now);                          // slice 1: due collects land
//
//   comm.tick(now);
//   if (comm.hasStatement()) {
//       in = comm.takeStatement();          // feed (copies line + returnPath)
//       watchdog.feed(now);
//       cmd.process(in.line, ...);           // parse -> stage into devState's outbox + replies
//   }
//
//   if (devState.hasHardwareCommand)        hardware.apply(devState.hardwareCommand);
//   if (devState.hasDrivetrainCommand) drivetrain.apply(devState.drivetrainCommand);
//
//   if (drivetrain.active()) {
//       drivetrain.tick(now, hardware.motor(p.left).state(), hardware.motor(p.right).state());
//       if (drivetrain.hasCommand()) hardware.apply(drivetrain.takeCommand());
//   }
//
//   hardware.tick(now);                          // slice 2: requests/writes go out
//
//   if (watchdog.check(now)) {
//       hardware.apply(buildBroadcastNeutral(...));        // applied immediately --
//       drivetrain.apply(buildDrivetrainStop(...));   // main.cpp is top-of-tree
//   }
//
// Same-pass latency (the design sketch's own worked Case 1): a statement is
// fed, parsed, staged, the Drivetrain re-governs against fresh observations,
// and slice 2 actuates -- all in one pass. `hardware.tick()` is called TWICE per
// iteration (slice 1 lets any due collect land before this pass's dispatch
// reads state; slice 2 sends out whatever request/write this pass's
// dispatch just staged) -- decision 6, replacing the old explicit bound-pair
// re-tick hack with the sanctioned second call; NezhaHardware's own flip-flop now
// cycles every in-use port evenly, including whichever pair the Drivetrain
// is bound to, with no main.cpp-level special-casing.
//
// `p.left`/`p.right` (Subsystems::DrivetrainPorts, from `drivetrain.ports()`)
// are whichever two NezhaMotors `DEV DT PORTS` last bound -- this loop never
// hardcodes which ports they are, and never holds its own copy of the
// binding (DevLoopState's old leftPort/rightPort fields are gone; the
// binding lives in DrivetrainConfig -- sprint 079 decision 8).
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
#include "config/boot_config.h"
#include "subsystems/nezha_hardware.h"
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

// Per-port boot MotorConfig storage. The VALUES no longer live here: they are
// baked at build time into generated code (source/config/boot_config.cpp,
// rewritten from the active robot JSON by scripts/gen_boot_config.py each
// build) and supplied via Config::defaultMotorConfigs(), which main() calls to
// fill this array below. `DEV M <n> CFG` remains the live-correction mechanism
// for a specific motor's real tuning.
static msg::MotorConfig defaultMotorConfigs[Subsystems::NezhaHardware::kPortCount];

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
    // Boot calibration comes from generated code baked from the active robot
    // JSON, not from main -- see Config::defaultMotorConfigs (boot_config.h).
    static_assert(Config::kMotorConfigCount == Subsystems::NezhaHardware::kPortCount,
                  "boot_config motor count must match NezhaHardware::kPortCount");
    Config::defaultMotorConfigs(defaultMotorConfigs);
    static I2CBus i2cBus(uBit.i2c);
    static Subsystems::NezhaHardware hardware(i2cBus, defaultMotorConfigs);
    hardware.begin();

    // --- Drivetrain: differential (Tovez), bench-placeholder trackwidth. ---
    // sync_gain is deliberately left at its zero default here (governor OFF
    // at boot) -- ticket 7's HITL bench pass found no live way to turn it on
    // short of a reflash and added `DEV DT CFG sync_gain=...`
    // (commands/dev_commands.cpp) for exactly that; bench scripts that need
    // the governor set it explicitly over the wire rather than this getting
    // a nonzero boot default.
    static Subsystems::Drivetrain drivetrain;
    // Trackwidth + drive-pair port binding come from generated code baked from
    // the active robot JSON (Config::defaultDrivetrainConfig, boot_config.h),
    // not from main. The binding lives in DrivetrainConfig (sprint 079 decision
    // 8); the coupled bench rig re-binds via `DEV DT PORTS 3 4` at runtime.
    msg::DrivetrainConfig dtConfig = Config::defaultDrivetrainConfig();
    drivetrain.configure(dtConfig);

    // --- Dev loop shared state: watchdog + DEV command wiring. ---
    static SerialSilenceWatchdog watchdog;

    static DevLoopState devState;
    devState.hardware = &hardware;
    devState.drivetrain = &drivetrain;
    devState.watchdog = &watchdog;
    // Seed the CFG-delta shadow so the first `DEV M <n> CFG kp=...` merges
    // onto the SAME calibration the motor was actually constructed with,
    // rather than an all-zero blank (see DevLoopState's field comment).
    for (uint32_t i = 0; i < Subsystems::NezhaHardware::kPortCount; ++i) {
        devState.motorConfigShadow[i] = defaultMotorConfigs[i];
    }
    // Seed the drivetrain CFG-delta shadow the same way, so the first
    // `DEV DT CFG sync_gain=...`/`DEV DT PORTS ...` merges onto the SAME
    // dtConfig the Drivetrain was actually configured with above
    // (trackwidth=128, ports=1,2), rather than an all-zero blank -- see
    // DevLoopState's field comment.
    devState.drivetrainConfigShadow = dtConfig;
    // Prime the capabilities cache for the default DEV DT PORTS binding --
    // read back via ports() (not a local copy) since the binding now lives
    // in DrivetrainConfig -- see drivetrain.h's setMotorCapabilities() doc
    // comment.
    Subsystems::DrivetrainPorts bootPorts = drivetrain.ports();
    drivetrain.setMotorCapabilities(hardware.motor(bootPorts.left).capabilities(),
                                     hardware.motor(bootPorts.right).capabilities());

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

        hardware.tick(now);   // slice 1: any due collect lands before this pass's dispatch reads state

        comm.tick(now);
        if (comm.hasStatement()) {
            Subsystems::CommunicatorToCommandProcessorStatement in = comm.takeStatement();
            // Feed on any line, either channel, regardless of content or
            // dispatch outcome -- see dev_commands.h's watchdog contract.
            watchdog.feed(now);
            // Parse happens inside process(): replies go out the
            // statement's own return path directly (unaffected by this
            // sprint); setpoint-shaped DEV commands land in devState's
            // outbox instead of calling Hal/Drivetrain write methods --
            // see dev_commands.h/.cpp's pure-transformer reshape.
            cmd.process(in.line,
                        in.returnPath == Subsystems::Channel::RADIO ? radioReply
                                                                    : serialReply,
                        &comm);
        }

        // Drain the outbox: main.cpp is the sole caller of
        // hardware.apply()/drivetrain.apply() for anything DEV-sourced.
        if (devState.hasHardwareCommand) {
            hardware.apply(devState.hardwareCommand);
            devState.hasHardwareCommand = false;
        }
        if (devState.hasDrivetrainCommand) {
            drivetrain.apply(devState.drivetrainCommand);
            devState.hasDrivetrainCommand = false;
        }

        if (drivetrain.active()) {
            // Binding queried, not duplicated -- ports() reads straight from
            // DrivetrainConfig (sprint 079 decision 8; DevLoopState no
            // longer holds its own leftPort/rightPort copy).
            Subsystems::DrivetrainPorts p = drivetrain.ports();
            drivetrain.tick(now, hardware.motor(p.left).state(), hardware.motor(p.right).state());
            if (drivetrain.hasCommand()) {
                hardware.apply(drivetrain.takeCommand());
            }
        }

        // Slice 2: whatever request/write this pass's dispatch (or the
        // Drivetrain's own re-governed target) just staged goes out now --
        // the sanctioned second hardware.tick() call (architecture-update.md
        // decision 6) replaces the old explicit bound-pair re-tick hack.
        hardware.tick(now);

        if (watchdog.check(now)) {
            // Applied IMMEDIATELY, not staged via the outbox -- main.cpp is
            // the top of the call tree, already the visible mover of every
            // command; an emergency stop gains nothing from an extra pass
            // of outbox latency (architecture-update.md's narrow, deliberate
            // exception to "never call apply() outside main/the HAL"). The
            // SAME buildBroadcastNeutral()/buildDrivetrainStop() construction
            // path `DEV STOP`'s handler stages is used here directly.
            hardware.apply(buildBroadcastNeutral(msg::Neutral::BRAKE));
            drivetrain.apply(buildDrivetrainStop(msg::Neutral::BRAKE));
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
