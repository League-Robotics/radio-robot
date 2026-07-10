// ---------------------------------------------------------------------------
// main.cpp -- sprint 094's bare-loop shape: the COMMUNICATION PLANE, plus
// the Drivetrain motion-planner connection, in main().
//
// main() constructs the Communicator (serial + radio), the I2C-bus-backed
// NezhaHardware container, the Drivetrain motion planner (holding a
// Hardware& -- ticket 094-004), one Rt::Blackboard (the queues commands
// post onto), and one Rt::CommandRouter (parse + dispatch), then runs a
// bare, explicit loop: tick the Communicator, route any arrived command,
// tick Hardware (pumps the I2C flip-flop -- timing unchanged from before
// this sprint), tick Drivetrain (drains bb.segmentIn/bb.driveIn, runs the
// executor/escape-hatch dispatch, stages this pass's wheel setpoints
// directly through Hardware's motor refs -- flushed the FOLLOWING pass by
// the next Hardware::tick()), then commit Drivetrain's measured state back
// onto the blackboard for TLM (094-006).
//
// This is the stakeholder's explicit harmonization decision (sprint 094):
// the Drivetrain connects into the bare main() loop DIRECTLY, as roughly
// one line calling tick on the drivetrain with the queues from the
// blackboard -- no `Rt::MainLoop` wrapper here (that class is kept for
// tests/_infra/sim/sim_api.cpp's SimHandle only, which shares one mandatory-
// tick implementation instead of hand-mirroring a second copy -- see
// runtime/main_loop.h's own file header). Minimalism is the point: this
// loop stays the explicit, wire-everything-here shape 093 established.
//
// Boot config is applied once, directly, at construction
// (`drivetrain.configure(dtConfig)`, plus `drivetrain.configureMotion(
// defaultMotionConfig())` for the Drivetrain-owned Motion::SegmentExecutor's
// jerk-limit defaults) -- there is no runtime config-application authority
// left to wire in (093: the `SET`/`GET` runtime-config path it served is
// unregistered).
//
// The DEVICE: identity banner is emitted by Communicator::begin() itself now
// (moved out of main()) -- the announcement is the Communicator's own job.
//
// uBit.sleep(1) yields to CODAL each pass so a received radio datagram
// (Radio::onData, a MessageBus listener) is delivered; serial RX is IRQ-driven
// and needs no yield.
// ---------------------------------------------------------------------------

#include "MicroBit.h"
#include "com/i2c_bus.h"
#include "config/boot_config.h"
#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "messages/planner.h"
#include "runtime/blackboard.h"
#include "runtime/command_router.h"
#include "subsystems/communicator.h"
#include "subsystems/drivetrain.h"
#include "subsystems/nezha_hardware.h"

static MicroBit uBit;

// ---------------------------------------------------------------------------
// serialReply / radioReply -- reply adapters. The CommandProcessor calls one
// of these per response line, routing the reply back out on whichever channel
// the command arrived on. Both take the Communicator as ctx and build on its
// primitive sends.
// ---------------------------------------------------------------------------
static void serialReply(const char* msg, void* ctx) {
    static_cast<Subsystems::Communicator*>(ctx)->sendSerial(msg);
}

static void radioReply(const char* msg, void* ctx) {
    static_cast<Subsystems::Communicator*>(ctx)->sendRadio(msg);
}

// defaultMotionConfig -- 094-005: a small, boot-only re-introduction of the
// jerk-limit defaults 093 deleted along with the whole `defaultPlannerConfig()`
// function (that function's other fields -- a_max/v_body_max/yaw_rate_max/
// yaw_acc_max/arrive_tol/turn_in_place_gate -- are NOT resurrected here: this
// sprint's Drivetrain-owned Motion::SegmentExecutor only needs the same
// four ramp-shape limits, applied once via drivetrain.configureMotion(),
// with jMax/yawJerkMax now nonzero instead of 093's `0.0` trapezoid
// sentinel -- see architecture-update.md Section 8). No runtime SET/GET
// path is revived; per-segment `MOVE j=`/`wj=` overrides (094-006) are the
// only live tuning surface this sprint ships.
static msg::PlannerConfig defaultMotionConfig() {
    msg::PlannerConfig cfg;
    cfg.a_max = 800.0f;         // [mm/s^2]
    cfg.a_decel = 800.0f;       // [mm/s^2]
    cfg.v_body_max = 1000.0f;   // [mm/s]
    cfg.yaw_rate_max = 6.0f;    // [rad/s]
    cfg.yaw_acc_max = 20.0f;    // [rad/s^2]
    cfg.j_max = 5000.0f;        // [mm/s^3] ~6x a_max -- ~0.16s jerk-limited edges
    cfg.yaw_jerk_max = 100.0f;  // [rad/s^3] ~5x yaw_acc_max -- ~0.2s
    return cfg;
}

int main() {
    uBit.init();

    // Comms: the Communicator subsystem (serial + radio, both enabled).
    // begin() brings up both transports AND emits the DEVICE: identity banner.
    static Subsystems::Communicator comm(uBit.serial, uBit.radio, uBit.messageBus);
    comm.configure(msg::CommunicatorConfig());
    comm.begin();

    // --- Hardware: the I2C brick flip-flop container (NezhaHardware). ---
    static I2CBus bus(uBit.i2c);
    static msg::MotorConfig motorConfigs[Subsystems::NezhaHardware::kMotorCount];
    Config::defaultMotorConfigs(motorConfigs);
    static Subsystems::NezhaHardware hardware(bus, motorConfigs,
                                               Config::defaultOtosBootConfig());
    hardware.begin();

    // --- Drivetrain: differential (Tovez), motion planner (094-004). Holds
    // a Hardware& -- `hardware` above must be constructed first (it is).
    // configureMotion() seeds the owned Motion::SegmentExecutor's boot
    // jerk-limit defaults exactly once; no runtime SET/GET path revived. ---
    static Subsystems::Drivetrain drivetrain(hardware);
    msg::DrivetrainConfig dtConfig = Config::defaultDrivetrainConfig();
    drivetrain.configure(dtConfig);
    drivetrain.configureMotion(defaultMotionConfig());
    drivetrain.setMotorCapabilities(hardware.motor(drivetrain.ports().left).capabilities(),
                                     hardware.motor(drivetrain.ports().right).capabilities());

    // The two-plane transport commands post onto, and the pointerless command
    // router that parses + dispatches inbound wire lines against it.
    static Rt::Blackboard bb;
    static Rt::CommandRouter router;
    router.setReplyChannels(serialReply, &comm, radioReply, &comm);

    // The whole loop: Communicator -> CommandRouter -> Blackboard + reply,
    // then Hardware -> Drivetrain -> commit (the Drivetrain connection --
    // see this file's own header comment).

    for (;;) {
        uint32_t now = uBit.systemTime();

        comm.tick(now);
        if (comm.hasCommand()) {
            router.route(comm.takeCommand(), bb); // Add the command to the router, which will parse and dispatch it, posting any command args onto the blackboard and replying through the appropriate channel.
        }

        hardware.tick(now);                                // pump the I2C flip-flop (timing unchanged)
        drivetrain.tick(now, 
            bb.segmentIn, 
            bb.driveIn);         

        bb.motors = hardware.motorStates();                // commit measured motor state (incl. I2C connected)
        bb.drivetrain = drivetrain.state();                // commit measured state for TLM (094-006)
        
        uBit.sleep(1);   // yield: radio RX delivery + other fibers
    }

    return 0;
}
