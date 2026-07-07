// ---------------------------------------------------------------------------
// main.cpp -- the dev loop: sprint 079's three-beat "feed it, tick it, ask
// it" shape (architecture-update.md "The Part-2 loop"), rewired for sprint
// 087 ticket 006's Rt::Blackboard/Rt::Configurator/Rt::CommandRouter
// transport (architecture-update-r1.md).
//
// 087-006: the six per-family `*State` structs (DevLoopState, TelemetryState,
// MotionLoopState, ConfigCommandState, PoseCommandState, OtosCommandState)
// and DevLoop are gone -- every command family now reads/posts against ONE
// Rt::Blackboard, reached from a command handler via Rt::CommandRouter (see
// runtime/command_router.h). main.cpp's own construction responsibility
// grows accordingly: it builds `bb`, the four subsystems, a Rt::Configurator
// (the one deliberate exception to "no subsystem pointers outside the
// loop" -- Decision 4), a Rt::CommandRouter, and dev_loop.h's LoopContext
// (the loop's own remaining subsystem references + the two loop-owned
// watchdogs), then runs runLoopPass() once per iteration -- the SAME shared
// function tests/_infra/sim/sim_api.cpp calls (mirrors ticket 081-002's
// "no hand-mirrored second copy" precedent, applied to the new transport).
//
// This is NOT ticket 007's real cyclic executive (no double-buffer commit,
// no mandatory/slack split, no uBit.sleep(1) yield) -- see dev_loop.h's file
// header for the full rationale; main.cpp's own loop shape (feed the
// Communicator, build a statement, call runLoopPass()) is otherwise
// unchanged from before this ticket.
//
// Build gating: unchanged -- the DEV family (and the HAL/Drivetrain/
// dev_loop wiring it needs) compiles in only when ROBOT_DEV_BUILD is set.
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
#include "subsystems/planner.h"
#include "subsystems/pose_estimator.h"
#include "dev_loop.h"
#include "messages/motor.h"
#include "messages/drivetrain.h"
#include "messages/planner.h"
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

// defaultPlannerConfig -- 084-002: unlike the motor/drivetrain configs above,
// there is no Config::defaultPlannerConfig() generator yet (no robot-JSON
// field maps onto msg::PlannerConfig today). These are fixed, conservative-
// but-workable ramp limits -- generous headroom above S/T/D's documented
// +-1000 mm/s wire range (docs/protocol-v2.md §10) so a max-speed command is
// never silently clamped.
msg::PlannerConfig defaultPlannerConfig() {
    msg::PlannerConfig cfg;
    cfg.a_max = 800.0f;              // [mm/s^2]
    cfg.a_decel = 800.0f;            // [mm/s^2]
    cfg.v_body_max = 1000.0f;        // [mm/s]
    cfg.yaw_rate_max = 6.0f;         // [rad/s]
    cfg.yaw_acc_max = 20.0f;         // [rad/s^2]
    cfg.j_max = 0.0f;                // trapezoid ramp, no S-curve, this sprint
    cfg.yaw_jerk_max = 0.0f;
    cfg.arrive_tol = 25.0f;          // [mm] matches docs/protocol-v2.md §10's G default
    cfg.turn_in_place_gate = 35.0f;  // matches docs/protocol-v2.md §10's G default
    cfg.min_speed = 0.0f;
    return cfg;
}

#endif  // ROBOT_DEV_BUILD

int main() {
    uBit.init();
    uBit.i2c.setFrequency(100000);

    // Comms: the Communicator subsystem (serial + radio, both enabled).
    static Subsystems::Communicator comm(uBit.serial, uBit.radio, uBit.messageBus);
    comm.configure(msg::CommunicatorConfig());
    comm.begin();

#if ROBOT_DEV_BUILD
    // --- HAL: one NezhaMotor per port (1-4) over the shared I2CBus. ---
    static_assert(Config::kMotorConfigCount == Subsystems::NezhaHardware::kPortCount,
                  "boot_config motor count must match NezhaHardware::kPortCount");
    Config::defaultMotorConfigs(defaultMotorConfigs);
    static I2CBus i2cBus(uBit.i2c);
    // 086-006: the real Hal::OtosOdometer leaf (I2C address 0x17) is
    // constructed alongside the four NezhaMotors, wired with ticket 086-005's
    // boot-config values.
    static Subsystems::NezhaHardware hardware(i2cBus, defaultMotorConfigs,
                                               Config::defaultOtosBootConfig());
    hardware.begin();

    // --- Drivetrain: differential (Tovez), bench-placeholder trackwidth. ---
    static Subsystems::Drivetrain drivetrain;
    msg::DrivetrainConfig dtConfig = Config::defaultDrivetrainConfig();
    drivetrain.configure(dtConfig);

    // --- Pose estimation (082-003): encoder dead-reckoning + OTOS (EkfTiny)
    // fusion. configure() reads the SAME dtConfig drivetrain.configure() just
    // took (one shared boot-config source).
    static Subsystems::PoseEstimator poseEstimator;
    poseEstimator.configure(dtConfig);

    // --- Motion executor (084-002): the goal-closure engine S/T/D/STOP
    // stage a msg::PlannerCommand into.
    static Subsystems::Planner planner;
    planner.configure(defaultPlannerConfig());

    // --- Rt::Blackboard (087-006): the single two-plane transport every
    // command family and the loop itself read/post against. Holds NO
    // subsystem pointers of any kind.
    static Rt::Blackboard bb;

    // --- Rt::Configurator (087-005): the single config-application
    // authority -- the ONE deliberate exception to "no subsystem pointers
    // outside the loop" (Decision 4).
    static Rt::Configurator configurator(drivetrain, poseEstimator, planner, hardware,
                                         dtConfig, defaultPlannerConfig());
    configurator.publish(bb);   // seed bb's current-config cells from boot config

    // --- Rt::CommandRouter (087-006): the pointerless command-tier
    // translator. Reply channels wired to the SAME serialReply/radioReply
    // adapters CommandProcessor used directly before this ticket.
    static Rt::CommandRouter router;
    router.setReplyChannels(serialReply, &comm, radioReply, &comm);

    // --- Boot-time hardware-identity snapshots (blackboard.h's file header):
    // never rewritten after this one-time seed -- capabilities/device
    // presence do not change at runtime for any current concrete Hardware
    // leaf.
    for (uint32_t port = 1; port <= Rt::kPortCount; ++port) {
        bb.motorCaps[port - 1] = hardware.motor(port).capabilities();
    }
    bb.otosPresent = (hardware.odometer() != nullptr);

    // Prime the capabilities cache for the default DEV DT PORTS binding --
    // read back via ports() (not a local copy), matching pre-087 boot wiring.
    Subsystems::DrivetrainPorts bootPorts = drivetrain.ports();
    drivetrain.setMotorCapabilities(hardware.motor(bootPorts.left).capabilities(),
                                     hardware.motor(bootPorts.right).capabilities());

    // --- LoopContext (dev_loop.h): the loop's own remaining subsystem
    // references + the two loop-owned watchdogs. serialReply/serialCtx
    // doubles as the loop-originated default reply sink (watchdog-fire EVT,
    // motion-done EVT), byte-identical to this loop's pre-087
    // serialReply/&comm.
    static LoopContext loop;
    loop.hardware = &hardware;
    loop.drivetrain = &drivetrain;
    loop.poseEstimator = &poseEstimator;
    loop.planner = &planner;
    loop.router = &router;
    loop.configurator = &configurator;
    loop.serialReply = serialReply;
    loop.serialCtx = &comm;
    loop.radioReply = radioReply;
    loop.radioCtx = &comm;

    // Start the serial-silence watchdog's window counting from boot (see
    // SerialSilenceWatchdog::feed()'s doc comment) rather than from an
    // uninitialized "last command" time.
    loop.watchdog.feed(uBit.systemTime());

    while (true) {
        uint32_t now = uBit.systemTime();

        // Feed it: comms is Communicator's job alone -- it never enters the
        // shared body. A taken statement is copied into a local
        // Subsystems::CommunicatorToCommandProcessorStatement (its line[]
        // is an OWNED buffer -- subsystems/statement.h -- so this copy is
        // safe past Communicator's own next tick()).
        comm.tick(now);
        Subsystems::CommunicatorToCommandProcessorStatement in;
        const Subsystems::CommunicatorToCommandProcessorStatement* stmtPtr = nullptr;
        if (comm.hasStatement()) {
            in = comm.takeStatement();
            stmtPtr = &in;
        }

        // Tick it, ask it: the shared, transitional loop body (source/
        // dev_loop.cpp) -- see dev_loop.h's file header.
        runLoopPass(loop, bb, now, stmtPtr);
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
