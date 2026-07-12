// ---------------------------------------------------------------------------
// main.cpp -- sprint 094's bare-loop shape: the COMMUNICATION PLANE, plus
// the Drivetrain motion-planner connection, in main().
//
// main() constructs the Communicator (serial + radio), the I2C-bus-backed
// NezhaHardware container, the Drivetrain motion planner (holding a
// Hardware& -- ticket 094-004), one Rt::Blackboard (the queues commands
// post onto), and one Rt::CommandRouter (parse + dispatch), then runs a
// bare, explicit loop: tick the Communicator, route any arrived command,
// tick telemetry (tickTelemetry() -- 096-002's loop-owned periodic STREAM
// emission, a no-op unless STREAM has armed bb.telemetryPeriod), tick
// Hardware (pumps the I2C flip-flop -- timing unchanged from before this
// sprint), tick the OTOS leaf directly (098-004/M6, Stage 2, optional --
// hardware.odometer()->tick(now), a separate I2C device from the flip-flop),
// tick Drivetrain (drains bb.segmentIn/bb.driveIn, runs the executor/
// escape-hatch dispatch -- reading the just-ticked OTOS pose internally for
// its heading source -- stages this pass's wheel setpoints directly through
// Hardware's motor refs -- flushed the FOLLOWING pass by the next
// Hardware::tick()), then commit Drivetrain's measured state (plus the OTOS
// pose/connected snapshot) back onto the blackboard for TLM (094-006).
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
// plannerConfig)` for the Drivetrain-owned Motion::SegmentExecutor's
// jerk-limit AND heading-loop-gain defaults, ticket 098-001) -- unchanged by
// the addition below.
//
// 098-005/M7: one Rt::Configurator is now ALSO constructed, seeded from the
// SAME dtConfig/plannerConfig, and ticked (`configurator.applyOne(bb)`) once
// per pass -- the loop's ONLY new runtime authority, and purely additive: it
// drains at most one already-queued `bb.configIn` delta per pass (a no-op
// whenever nothing has posted one -- e.g. every pass on a robot that never
// receives a `SET`), so boot behavior is unchanged. This does NOT reinstate
// 093/094-era full runtime config authority (no `SET`/`GET` text handler is
// revived); it only lets a delta ALREADY reaching `bb.configIn` via some
// other registered path (096-004's binary `config` command -- see
// commands/binary_channel.cpp) actually apply to the live Drivetrain/
// Hardware/PoseEstimator instead of sitting undrained forever.
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
#include "runtime/blackboard.h"
#include "runtime/command_router.h"
#include "runtime/configurator.h"
#include "subsystems/communicator.h"
#include "subsystems/drivetrain.h"
#include "subsystems/nezha_hardware.h"
#include "subsystems/pose_estimator.h"
#include "telemetry/telemetry_tick.h"

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

int hardware_main() {
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
    // jerk-limit/heading-gain defaults exactly once; a LIVE re-application
    // after boot is the Configurator's job now (098-005/M7, below), not a
    // revived text SET/GET handler. ---
    static Subsystems::Drivetrain drivetrain(hardware);
    msg::DrivetrainConfig dtConfig = Config::defaultDrivetrainConfig();
    drivetrain.configure(dtConfig);
    msg::PlannerConfig plannerConfig = Config::defaultPlannerConfig();
    drivetrain.configureMotion(plannerConfig);
    drivetrain.setMotorCapabilities(hardware.motor(drivetrain.ports().left).capabilities(),
                                     hardware.motor(drivetrain.ports().right).capabilities());

    // --- PoseEstimator: a Subsystems-tier peer of Drivetrain (never folded
    // into it -- pose_estimator.h's own file header). Constructed here ONLY
    // to satisfy Rt::Configurator's constructor (below) -- a kDrivetrain-
    // scoped delta re-propagates to it too (configurator.cpp). Holds no
    // hardware reference (pose_estimator.h: "holds NO Hal::Motor/
    // Hal::Odometer reference or pointer"), and stays UNTICKED by this loop
    // -- 098-004/M6's OTOS heading revival (below, and Drivetrain::tick()'s
    // own doc comment) reads hardware.odometer()->pose()/connected()
    // DIRECTLY, never through this PoseEstimator's fusion path (full
    // pose-fusion/EKF is sprint 099's scope, not this sprint's -- see
    // architecture-update.md M6's boundary), so this instance stays
    // constructed-but-inert -- it changes nothing about this loop's existing
    // behavior. ---
    static Subsystems::PoseEstimator poseEstimator;

    // --- Configurator (098-005/M7): the one live config-application
    // authority (source/runtime/configurator.h's class comment) -- seeded
    // from the SAME dtConfig/plannerConfig values already passed directly
    // to drivetrain.configure()/configureMotion() above, so a freshly
    // booted robot that never receives a SET behaves identically to today
    // (boot config still applies once, directly, at construction; this is
    // additive only -- see this file's own header comment). ---
    static Rt::Configurator configurator(drivetrain, poseEstimator, hardware, dtConfig,
                                          plannerConfig);

    // The two-plane transport commands post onto, and the pointerless command
    // router that parses + dispatches inbound wire lines against it.
    static Rt::Blackboard bb;
    // 096-002: no runtime Configurator is wired this sprint (093/094: "no
    // runtime config-application authority left" -- boot config is applied
    // once, directly, at construction), so bb.drivetrainConfig would
    // otherwise stay at its zero-valued default (left_port=0/right_port=0/
    // trackwidth=0) forever -- Configurator::applyOne() is normally the ONLY
    // thing that ever publishes it (configurator.cpp). Telemetry::tick()
    // (source/telemetry/tlm_frame.cpp) reads bb.drivetrainConfig.left_port/
    // right_port directly as a 0-based bb.motors[] index (leftIdx = left_port
    // - 1): left uninitialized, that underflows to UINT32_MAX and reads
    // wildly out of bounds -- STREAM/SNAP's first live emission crashed the
    // sim with a bus error before this line was added. A one-time direct
    // seed with the SAME dtConfig already handed to drivetrain.configure()
    // above, mirroring Configurator::applyOne()'s own "bb.drivetrainConfig =
    // drivetrainConfig_;" publish.
    bb.drivetrainConfig = dtConfig;
    // 098-005/M7: the Configurator's own boot-time publish (configurator.h:
    // "seeds all four bb.*Config cells... boot-time use, before the loop
    // starts") -- fills in bb.motorConfig[]/bb.plannerConfig/bb.odometerConfig
    // (previously always zero-valued here; nothing else in this loop ever
    // set them) with the SAME values the live subsystems were actually
    // configured with above. Harmless re-write of bb.drivetrainConfig with
    // the identical value the line above already set (mirrors
    // tests/_infra/sim/sim_api.cpp's SimHandle constructor, which keeps both
    // lines for the same reason). Purely a telemetry/GET-visibility fix --
    // no live subsystem is touched by this call (publish() never calls
    // configure() on anything), so it cannot change control-loop behavior.
    configurator.publish(bb);
    static Rt::CommandRouter router;
    router.setReplyChannels(serialReply, &comm, radioReply, &comm);

    // The whole loop: Communicator -> CommandRouter -> Blackboard + reply,
    // then Hardware -> Drivetrain -> commit (the Drivetrain connection --
    // see this file's own header comment).

    for (;;) {
        uint32_t now = uBit.systemTime();

        // Ticks
        ///

        comm.tick(now);
        if (comm.hasCommand()) {
            router.route(comm.takeCommand(), bb); // Add the command to the router, which will parse and dispatch it, posting any command args onto the blackboard and replying through the appropriate channel.
        }
        // 098-005/M7: drains AT MOST one bb.configIn delta per pass
        // (Configurator::applyOne()'s own documented one-delta-per-call
        // contract) -- placed here, right after a same-pass SET could have
        // just posted one via router.route() above, and BEFORE
        // hardware.tick()/drivetrain.tick() below, so a delta arriving THIS
        // pass is already live (e.g. reaches Drivetrain::configureMotion()
        // for a kPlanner delta) by the time this SAME pass's drivetrain.tick()
        // runs -- one tick sooner than draining it after the commit step
        // would. A no-op whenever bb.configIn is empty (every pass no SET
        // has ever arrived), so boot behavior is unchanged either way.
        configurator.applyOne(bb);
        tickTelemetry(bb, router, now); // Loop-owned periodic STREAM emission (096-002) -- a no-op unless STREAM has armed bb.telemetryPeriod.

        hardware.tick(now);                                // pump the I2C flip-flop (timing unchanged)
        // 098-004/M6 (Stage 2, optional): tick the OTOS leaf once per pass --
        // a separate I2C device (0x17) from the Nezha brick's own flip-flop
        // (0x10), so this is a standalone call, not folded into
        // hardware.tick() above. Placed AFTER hardware.tick() (this pass's
        // wheel/encoder collection) and BEFORE drivetrain.tick() below so a
        // fresh OTOS pose is available THIS SAME pass -- Drivetrain::tick()
        // reads hardware.odometer()->pose()/connected() directly, internally,
        // to feed its owned Motion::SegmentExecutor's heading source (see
        // drivetrain.h's own tick() doc comment). Subsystems::SimHardware
        // already ticks its own Hal::SimOdometer inside ITS OWN tick(), so
        // this call is what revives live OTOS sampling on REAL hardware
        // specifically (Subsystems::NezhaHardware::tick() does not tick its
        // otosOdometer_ member -- see nezha_hardware.cpp).
        hardware.odometer()->tick(now);
        drivetrain.tick(now,
            bb.segmentIn,
            bb.replaceIn,
            bb.driveIn);

        //
        // Commit state to the blackboard 
        // 

        bb.motors = hardware.motorStates();                // commit measured motor state (incl. I2C connected)
        bb.drivetrain = drivetrain.state();                // commit measured state for TLM (094-006)
        bb.loopNow = now;                                  // commit stamp for TLM now= (cmd='s true time)
        // 098-004/M6 (Stage 2, optional): commit this pass's OTOS pose/
        // connected snapshot for telemetry -- previously always false/
        // never-set on this bare loop (bb.otos/bb.otosConnected stayed at
        // their zero-valued defaults forever -- nothing else in this loop
        // ever wrote them). Deliberately NOT bb.otosValid: that field is
        // Hal::Odometer::fusableThisPass()'s own EKF-fusion-gate signal,
        // with exactly ONE sanctioned caller (Subsystems::PoseEstimator::
        // tick(), never ticked on this branch -- see poseEstimator's own
        // construction comment above); do not add a second
        // fusableThisPass() caller here (architecture-update.md Decision 4's
        // explicit note). connected() && pose().stamp.valid is this loop's
        // own freshness/validity signal instead, matching what
        // Drivetrain::tick() already derives internally for the executor.
        bb.otos = hardware.odometer()->pose();
        bb.otosConnected = hardware.odometer()->connected();

        uBit.sleep(1);   // yield: radio RX delivery + other fibers
    }

    return 0;
}

int main(){
    hardware_main();
}
