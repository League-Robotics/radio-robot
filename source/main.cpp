// ---------------------------------------------------------------------------
// main.cpp -- the cyclic executive (sprint 087 ticket 007's real design,
// architecture-update-r1.md's Reference code, gutted per sprint 093's
// architecture-update.md Step 5): mandatory control tick -> commit (clock
// edge) -> best-effort slack (ingest -> route), replacing ticket 006's
// TRANSITIONAL same-pass sequential `runLoopPass()` (source/dev_loop.{h,cpp},
// deleted by that ticket).
//
// Construction responsibility: builds one Rt::Blackboard (the two-plane
// transport every subsystem/command family reads/posts against, holding NO
// subsystem pointers of any kind), one Rt::CommandRouter (the pointerless
// command-tier translator), and one Rt::MainLoop (this loop's own
// composition-root state: the two subsystems -- Hardware, Drivetrain -- it
// ticks every pass; see runtime/main_loop.h). Boot config is applied once,
// directly, at construction (`drivetrain.configure(dtConfig)`) -- there is
// no runtime config-application authority left to wire in (093: the
// `SET`/`GET` runtime-config path it served is unregistered).
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
#include "runtime/blackboard.h"
#include "runtime/command_router.h"
#include "runtime/main_loop.h"
#include "messages/motor.h"
#include "messages/drivetrain.h"
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
    static Subsystems::Communicator comm(uBit.serial, uBit.radio, uBit.messageBus);
    comm.configure(msg::CommunicatorConfig());
    comm.begin();

    // 088-005: the DEVICE: identity banner is the first line out on BOTH
    // channels -- before anything else is sent, before the loop starts.
    // formatDeviceAnnouncement() (commands/system_commands.h) is the same
    // helper HELLO's handler calls, so a re-request always matches this
    // boot banner byte-for-byte. Radio is fire-and-forget: a missed boot
    // radio banner (no relay listening yet) is not a failure -- HELLO is
    // the reliable re-request path once a relay attaches.
    char deviceBanner[64];
    formatDeviceAnnouncement(deviceBanner, sizeof(deviceBanner));
    comm.sendSerial(deviceBanner);
    comm.sendRadio(deviceBanner);

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

    // --- Rt::Blackboard (087-002/006): the single two-plane transport every
    // command family and the loop itself read/post against. Holds NO
    // subsystem pointers of any kind.
    static Rt::Blackboard bb;

    // --- Rt::CommandRouter (087-006): the pointerless command-tier
    // translator. Reply channels wired to the SAME serialReply/radioReply
    // adapters CommandProcessor used directly before this ticket.
    static Rt::CommandRouter router;
    router.setReplyChannels(serialReply, &comm, radioReply, &comm);

    // Prime the capabilities cache for the default DEV DT PORTS binding --
    // read back via ports() (not a local copy), matching pre-087 boot wiring.
    Subsystems::DrivetrainPorts bootPorts = drivetrain.ports();
    drivetrain.setMotorCapabilities(hardware.motor(bootPorts.left).capabilities(),
                                     hardware.motor(bootPorts.right).capabilities());

    // --- Rt::MainLoop (093 gut): the cyclic executive's own composition-root
    // state -- just the two subsystems above, ticked every mandatory pass.
    // No watchdogs, no reply sinks, no pose/planner references left to wire.
    static Rt::MainLoop loop(hardware, drivetrain);

    constexpr uint32_t kPeriod = 20;   // [ms] target cadence -- best-effort, NOT a hard deadline

    for (;;) {
        uint32_t now = uBit.systemTime();

        // === MANDATORY + COMMIT: the one control pass. ===
        loop.tick(bb, now);

        // === SLACK: yield, then ingest -> route, until the next period.
        //     uBit.sleep(1) is REQUIRED, not pacing (Decision 9, see this
        //     file's header). ===
        uint32_t deadline = now + kPeriod;
        // 093: yield to the CODAL scheduler at most ONCE per slack window
        // instead of every pass. The per-pass uBit.sleep(1) added by 087-007
        // churns the scheduler ~1000x/s, and its brief IRQ-masked context
        // switches starve the DMA serial RX -- tripling serial command drops
        // (the 087-006 -> 087-007 regression, bisected). One yield per ~20ms
        // slack still lets CODAL deliver radio datagrams (Decision 9: radio RX
        // only runs when the loop yields a fiber slice) at ~50 Hz, far above
        // the radio's ~12 msg/s rate, while the rest of the slack busy-reads
        // serial so an inbound command is drained immediately.
        bool yieldedThisSlack = false;
        do {
            comm.tick(uBit.systemTime());
            if (comm.hasCommand()) {
                // A taken command is copied into a local
                // Subsystems::CommunicatorToCommandProcessorCommand (its
                // line[] is an OWNED buffer -- subsystems/wire_command.h -- so
                // this copy is safe past Communicator's own next tick()).
                Subsystems::CommunicatorToCommandProcessorCommand command = comm.takeCommand();
                router.route(command, bb);
            } else if (!yieldedThisSlack) {
                uBit.sleep(1);   // YIELD once/slack: radio delivery + other fibers
                yieldedThisSlack = true;
            }
        } while (uBit.systemTime() < deadline);
    }
#else
    // ROBOT_DEV_BUILD == 0: no production loop exists yet this sprint (see
    // the file header) -- this minimal liveness-only fallback (no HAL, no
    // DEV, no watchdog) is what proves the ROBOT_DEV_BUILD gate is a real
    // fork rather than a decorative #if 1.
    //
    // 088-003: systemCommands() now takes a Rt::CommandRouter& so HELP can
    // enumerate the live table (see system_commands.h). This fallback has
    // no Rt::MainLoop/Rt::Blackboard to route through -- `router` exists
    // solely to bind HELP's handlerCtx; `cmd` (built the same way, from the
    // same systemCommands(router) table) is what actually dispatches every
    // line, exactly as before. Since ROBOT_DEV_BUILD is 0 here,
    // buildTable() (command_router.cpp) registers only the liveness family
    // for `router` too, so HELP correctly reports just the five liveness
    // verbs this fallback actually serves.
    static Rt::CommandRouter router;
    static CommandProcessor cmd(systemCommands(router));
    cmd.setSerialReply(serialReply, &comm);

    while (true) {
        uint32_t now = uBit.systemTime();

        comm.tick(now);
        if (comm.hasCommand()) {
            Subsystems::CommunicatorToCommandProcessorCommand in = comm.takeCommand();
            cmd.process(in.line,
                        in.returnPath == Subsystems::Channel::RADIO ? radioReply
                                                                    : serialReply,
                        &comm);
        }
    }
#endif

    return 0;
}
