// ---------------------------------------------------------------------------
// main.cpp -- the dev loop: sprint 079's three-beat "feed it, tick it, ask
// it" shape (architecture-update.md "The Part-2 loop"; issue's Part 2).
//
// 081-002: the loop body itself (the "tick it, ask it" beats) is no longer
// written inline here -- it moved, verbatim, into devLoopTick()
// (source/dev_loop.{h,cpp}), the shared function a future simulated caller
// (ticket 081-004's sim_api.cpp) can run too, with no hand-mirrored second
// copy. main.cpp's own loop now collapses to exactly the "feed it" beat plus
// one call into the shared body: read the clock, tick the Communicator
// (comms is Communicator's job ALONE -- it never enters the shared body,
// see dev_loop.h's file header and architecture-update.md (081) Decision 3),
// build a DevLoopStatement from whatever statement (if any) Communicator
// just handed back, and call devLoopTick(). See dev_loop.cpp for the
// line-by-line shape of what used to live here (the two hardware.tick()
// slices, the outbox drain, Drivetrain governance, and the watchdog check).
//
// This supersedes 077-001/003/004's smoke wiring and 079-004's minimal loop
// adaptation: CommandProcessor/DevLoopState are still a pure transformer
// (statements in, commands + replies out -- see dev_commands.h) and the
// SOLE caller of Subsystems::Hardware::apply()/Subsystems::Drivetrain::
// apply() for anything DEV-sourced is now devLoopTick() itself, draining
// DevLoopState's outbox once per pass instead of DEV handlers calling
// either subsystem's write methods directly.
//
// `p.left`/`p.right` (Subsystems::DrivetrainPorts, from `drivetrain.ports()`)
// are whichever two NezhaMotors `DEV DT PORTS` last bound -- devLoopTick()
// never hardcodes which ports they are, and never holds its own copy of the
// binding (DevLoopState's old leftPort/rightPort fields are gone; the
// binding lives in DrivetrainConfig -- sprint 079 decision 8).
//
// Build gating: the DEV family (and the HAL/Drivetrain/dev_loop wiring it
// needs) is compiled in only when ROBOT_DEV_BUILD is set (codal.json's
// "config" object; see dev_commands.h's file header for the full
// rationale). This sprint's codal.json sets it to 1 for this tree -- there
// is no production loop yet, so the #else branch below is a minimal
// liveness-only fallback (PING/VER/HELP/ECHO/ID, no HAL) proving the gate
// is a real fork, not decoration.
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
#include "subsystems/pose_estimator.h"
#include "commands/dev_commands.h"
#include "dev_loop.h"
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

    // --- Pose estimation (082-003): encoder dead-reckoning + OTOS (EkfTiny)
    // fusion, a Subsystems-tier peer of Drivetrain -- see
    // source/subsystems/pose_estimator.h's class comment. configure() reads
    // the SAME dtConfig drivetrain.configure() just took (one shared
    // boot-config source, no duplicated values) -- trackwidth/
    // rotational_slip plus the four EKF noise fields.
    static Subsystems::PoseEstimator poseEstimator;
    poseEstimator.configure(dtConfig);

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

    // --- Shared dev-loop wiring (081-002; source/dev_loop.h). defaultReply/
    // defaultReplyCtx is the loop-originated reply sink devLoopTick() uses
    // for the watchdog-fire EVT (not triggered by any inbound statement) --
    // byte-identical to this loop's pre-extraction serialReply/&comm.
    static DevLoop loop;
    loop.hardware = &hardware;
    loop.drivetrain = &drivetrain;
    loop.poseEstimator = &poseEstimator;
    loop.processor = &cmd;
    loop.watchdog = &watchdog;
    loop.devState = &devState;
    loop.defaultReply = serialReply;
    loop.defaultReplyCtx = &comm;

    // Start the serial-silence watchdog's window counting from boot (see
    // SerialSilenceWatchdog::feed()'s doc comment) rather than from an
    // uninitialized "last command" time.
    watchdog.feed(uBit.systemTime());

    while (true) {
        uint32_t now = uBit.systemTime();

        // Feed it: comms is Communicator's job alone -- it never enters the
        // shared body (dev_loop.h's file header; architecture-update.md
        // (081) Decision 3). A taken statement is copied into a
        // DevLoopStatement (a plain, caller-owned, single-call-lifetime
        // pointer -- Communicator's own line buffer is only valid until its
        // next tick(), so this copy must happen before devLoopTick() runs).
        comm.tick(now);
        DevLoopStatement stmt;
        const DevLoopStatement* stmtPtr = nullptr;
        if (comm.hasStatement()) {
            Subsystems::CommunicatorToCommandProcessorStatement in = comm.takeStatement();
            stmt.line = in.line;
            stmt.replyFn = in.returnPath == Subsystems::Channel::RADIO ? radioReply
                                                                        : serialReply;
            stmt.replyCtx = &comm;
            stmtPtr = &stmt;
        }

        // Tick it, ask it: the shared dev-loop body (source/dev_loop.cpp) --
        // the two hardware.tick() slices, statement dispatch, outbox drain,
        // Drivetrain governance, pose estimation (082-003), and the
        // watchdog check, byte-identical to this loop's pre-081-002 inline
        // body plus 082-003's one addition.
        devLoopTick(loop, now, stmtPtr);
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
