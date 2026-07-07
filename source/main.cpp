// ---------------------------------------------------------------------------
// main.cpp -- the cyclic executive (sprint 087 ticket 007's real design,
// architecture-update-r1.md's Reference code): mandatory control tick ->
// commit (clock edge) -> best-effort slack (ingest -> route -> apply
// config), replacing ticket 006's TRANSITIONAL same-pass sequential
// `runLoopPass()` (source/dev_loop.{h,cpp}, deleted by this ticket).
//
// Construction responsibility: builds one Rt::Blackboard (the two-plane
// transport every subsystem/command family reads/posts against, holding NO
// subsystem pointers of any kind), one Rt::Configurator (the ONE deliberate
// exception to "no subsystem pointers outside the loop" -- Decision 4), one
// Rt::CommandRouter (the pointerless command-tier translator), and one
// Rt::MainLoop (this loop's own composition-root state: the four
// subsystems it ticks every pass, the two loop-owned watchdogs, and the
// loop-originated reply sinks -- see runtime/main_loop.h).
//
// The slack loop's uBit.sleep(1) yield is REQUIRED, not pacing (Decision
// 9): CODAL's cooperative fiber scheduler only delivers a received radio
// datagram (Radio::onData, a MessageBus listener -- source/com/radio.cpp)
// when the main loop yields a fiber slice; serial RX is IRQ-driven and
// needs no yield. A busy-wait slack loop would starve radio ONLY, silently,
// while every serial-only test kept passing -- see architecture-update-r1.md
// Decision 9 for the full grounding.
//
// Build gating: unchanged -- the DEV family (and the HAL/Drivetrain/
// MainLoop wiring it needs) compiles in only when ROBOT_DEV_BUILD is set.
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
#include "runtime/blackboard.h"
#include "runtime/command_router.h"
#include "runtime/configurator.h"
#include "runtime/main_loop.h"
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

    // --- Rt::Blackboard (087-002/006): the single two-plane transport every
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

    // --- Rt::MainLoop (087-007): the cyclic executive's own composition-root
    // state -- the four subsystems above (ticked every mandatory pass), the
    // two loop-owned watchdogs, and the loop-originated reply sinks.
    // serialReply/&comm doubles as the loop-originated "default" reply sink
    // (watchdog-fire EVT, motion-done EVT, safety_stop EVT) AND the periodic-
    // telemetry-emission channel when bb.telemetryChannel is SERIAL/NONE --
    // byte-identical to this loop's pre-087 wiring, which was always bound to
    // serial.
    static Rt::MainLoop loop(hardware, drivetrain, poseEstimator, planner,
                             serialReply, &comm, radioReply, &comm);

    // Start the serial-silence watchdog's window counting from boot (see
    // SerialSilenceWatchdog::feed()'s doc comment) rather than from an
    // uninitialized "last command" time.
    loop.feedWatchdog(uBit.systemTime());

    constexpr uint32_t kPeriod = 20;   // [ms] target cadence -- best-effort, NOT a hard deadline

    for (;;) {
        uint32_t now = uBit.systemTime();

        // === MANDATORY + COMMIT: the one control pass. ===
        loop.tick(bb, now);

        // === SLACK: yield, then ingest -> route -> apply config, until the
        //     next period. uBit.sleep(1) is REQUIRED, not pacing (Decision 9,
        //     see this file's header) -- routing still wins over config
        //     application (Decision 8). ===
        uint32_t deadline = now + kPeriod;
        do {
            uBit.sleep(1);   // YIELD: radio delivery + other fibers run
            comm.tick(uBit.systemTime());
            if (comm.hasStatement()) {
                // A taken statement is copied into a local
                // Subsystems::CommunicatorToCommandProcessorStatement (its
                // line[] is an OWNED buffer -- subsystems/statement.h -- so
                // this copy is safe past Communicator's own next tick()).
                Subsystems::CommunicatorToCommandProcessorStatement statement = comm.takeStatement();
                // Feed BEFORE routing -- feeding must never be delayed by
                // routing/config-priority (the safety-watchdog contract).
                loop.feedWatchdog(uBit.systemTime());
                router.route(statement, bb);
            } else if (configurator.pending(bb)) {
                configurator.applyOne(bb);
            }
        } while (uBit.systemTime() < deadline);
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
