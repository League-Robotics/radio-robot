// bus_drain_api.cpp — extern "C" C-ABI shims for the bus drain layer (ticket 059-003).
//
// Provides a BusDrainHandle that owns a self-contained stack of
//   SimHardware + Drive + Planner
//   + CommandQueue
//   + CommandProcessor (empty command table)
//
// and exposes:
//   bus_drain_api_create / destroy
//   bus_drain_api_build_twist_batch — build a CommandBatch with one TWIST OutCommand
//   bus_drain_api_build_priority_batch — build a CommandBatch with one priority=true OutCommand
//   bus_drain_api_build_n_commands — build a CommandBatch with N identical OutCommands
//   bus_drain_api_drain — call drainCommandBatch(), return routed count
//   bus_drain_api_drive2_get_fused_x — read drive2 state after drain
//   bus_drain_api_queue_size — return queue.size() after drain
//   bus_drain_api_drain_returns_n — drain a batch, return the routed count
//
// Python tests (test_059_bus_drain.py) load this via ctypes.
//
// Construction order mirrors planner_api.cpp (061-004: motion_ctrl removed):
//   cfg / hal / mc_ctrl / bvc / est / drive / planner
//   + CommandQueue + CommandProcessor (empty)
//
// Heap allocation in BusDrainHandle is acceptable — the no-heap constraint
// applies to drainCommandBatch() itself, not the test harness.

// Sprint 050, Ticket 004: EKFTiny must be included BEFORE any header that
// transitively pulls in tinyekf.h.
#define EKF_N 5
#define EKF_M 2
#include "state/EKFTiny.h"

#include "types/Config.h"
#include "types/Inputs.h"
#include "hal/sim/SimHardware.h"
#include "control/MotorController.h"
#include "control/BodyVelocityController.h"
#include "state/PhysicalStateEstimate.h"
#include "control/Odometry.h"
#include "subsystems/drive/Drive.h"          // also declares toDriveConfig()
#include "superstructure/Planner.h"
#include "superstructure/PlannerConfig.h"
#include "commands/CommandQueue.h"
#include "commands/CommandProcessor.h"
#include "robot/BusDrain.h"
#include "messages/common.h"
#include "messages/drivetrain.h"
#include "messages/planner.h"
#include "messages/verb_ids.h"
#include <vector>
#include <cstring>

// ---------------------------------------------------------------------------
// BusDrainHandle — opaque handle owning the full subsystem stack + queue.
// ---------------------------------------------------------------------------
struct BusDrainHandle {
    RobotConfig             cfg;
    SimHardware             hal;
    MotorController         mc_ctrl;
    BodyVelocityController  bvc;
    PhysicalStateEstimate   est;
    subsystems::Drive       drive;
    Planner                 planner;
    CommandQueue            queue;
    CommandProcessor        cmd_proc;

    BusDrainHandle()
        : cfg(defaultRobotConfig())
        , hal(cfg)
        , mc_ctrl(hal.motorL(), hal.motorR(), cfg)
        , bvc(mc_ctrl, cfg)
        , est()
        , drive(hal.motorL(), hal.motorR(),
                mc_ctrl, bvc, est, est.odometry(),
                hal.otos(), cfg)
        , planner(mc_ctrl, est.odometry(), drive, cfg)
        , queue()
        , cmd_proc()
    {
        drive.configure(toDriveConfig(cfg));
        // 060-004: initialise the EKF so that Odometry::predict() runs the
        // full EKF step and populates _hw.fused.twist.vx_mmps.
        // Without this, _ekf.v() / _ekf.omega() are uninitialized and predict()
        // may skip the update, leaving fused.twist at zero even after encoder motion.
        // Mirrors Robot::Robot() which calls estimate.initEKF(…) during construction.
        est.initEKF(cfg.ekfQxy, cfg.ekfQtheta,
                    cfg.ekfQv,  cfg.ekfQomega,
                    cfg.ekfROtosXy, cfg.ekfROtosV,
                    cfg.ekfREncV,   cfg.ekfROtosTheta);
    }
};

extern "C" {

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void* bus_drain_api_create()
{
    return new BusDrainHandle();
}

void bus_drain_api_destroy(void* h)
{
    delete static_cast<BusDrainHandle*>(h);
}

// ---------------------------------------------------------------------------
// Batch builders
// ---------------------------------------------------------------------------

// Build a CommandBatch containing one TWIST OutCommand.
//   verb_id=1, args[0]=vx, args[1]=vy, args[2]=omega, priority=false.
// Returns the batch as a value through an output parameter.
void bus_drain_api_build_twist_batch(
    float vx, float vy, float omega,
    msg::CommandBatch* out_batch)
{
    if (!out_batch) return;
    msg::CommandBatch batch{};
    msg::OutCommand& oc = batch.cmds_[0];
    oc.verb_id    = msg::kVerbDrivetrainTwist;
    oc.args_[0]   = vx;
    oc.args_[1]   = vy;
    oc.args_[2]   = omega;
    oc.args_count = 3;
    oc.priority   = false;
    batch.cmds_count = 1;
    batch.count      = 1;
    *out_batch = batch;
}

// Build a CommandBatch with one OutCommand with priority=true (passthrough verb_id=99).
void bus_drain_api_build_priority_batch(msg::CommandBatch* out_batch)
{
    if (!out_batch) return;
    msg::CommandBatch batch{};
    msg::OutCommand& oc = batch.cmds_[0];
    oc.verb_id    = 99u;   // unrecognised verb → passthrough to queue
    oc.priority   = true;
    oc.args_count = 0;
    batch.cmds_count = 1;
    batch.count      = 1;
    *out_batch = batch;
}

// Build a CommandBatch with N identical passthrough commands (verb_id=99, priority=false).
// N is clamped to 8 (CommandBatch capacity).
void bus_drain_api_build_n_commands(uint8_t n, msg::CommandBatch* out_batch)
{
    if (!out_batch) return;
    msg::CommandBatch batch{};
    // CommandBatch.cmds_ has capacity 8; OutCommand array is [8].
    uint8_t cap = 8;
    uint8_t cnt = (n < cap) ? n : cap;
    for (uint8_t i = 0; i < cnt; ++i) {
        msg::OutCommand& oc = batch.cmds_[i];
        oc.verb_id    = 99u;   // passthrough
        oc.priority   = false;
        oc.args_count = 0;
    }
    batch.cmds_count = cnt;
    batch.count      = cnt;
    *out_batch = batch;
}

// ---------------------------------------------------------------------------
// Drain
// ---------------------------------------------------------------------------

// Call drainCommandBatch() with the provided batch.
// Returns the number of commands routed.
uint8_t bus_drain_api_drain(void* h, const msg::CommandBatch* batch)
{
    if (!h || !batch) return 0;
    BusDrainHandle* b = static_cast<BusDrainHandle*>(h);
    return drainCommandBatch(*batch, b->drive, b->planner, b->queue, b->cmd_proc);
}

// ---------------------------------------------------------------------------
// State reads
// ---------------------------------------------------------------------------

// Read drive2's fused X pose (mm/s) after drain (reflects applied TWIST).
float bus_drain_api_drive2_get_fused_x(void* h)
{
    if (!h) return 0.0f;
    BusDrainHandle* b = static_cast<BusDrainHandle*>(h);
    return b->drive.state().get_fused().get_pose().get_x();
}

// Read the command queue's current size.
int bus_drain_api_queue_size(void* h)
{
    if (!h) return -1;
    BusDrainHandle* b = static_cast<BusDrainHandle*>(h);
    return b->queue.size();
}

// ---------------------------------------------------------------------------
// Drive2 tick helpers (for verifying that an applied TWIST actually drives)
// ---------------------------------------------------------------------------

// Run one full sense + act cycle on Drive2 so the applied command takes effect.
void bus_drain_api_tick(void* h, uint32_t now_ms)
{
    if (!h) return;
    BusDrainHandle* b = static_cast<BusDrainHandle*>(h);
    b->hal.tick(now_ms, b->drive.outputs());
    b->hal.tick(now_ms);
    b->drive.tickUpdate(now_ms);
    b->drive.tickAction(now_ms);
}

// Read the encoder-derived vx from drive2 state.
//
// 060-004: In the ordered-tick path, Drive2::tickAction TWIST calls
// _mc.setTarget() directly (direct IK, no BVC ramp). The EKF velocity state
// (fused.twist.vx) starts at 0 and has a chi-square gate that requires
// P[3][3] to grow large enough to accept the full-speed step — which takes
// 100+ ticks. The encoder twist (_encVx = dCenter / dt_s) is NOT gated and
// reflects the actual measured encoder velocity directly.  The test's intent
// is to verify that the TWIST command was received and drive2 is actually
// driving; encoder vx is the right signal for that intent.
float bus_drain_api_drive2_get_vx(void* h)
{
    if (!h) return 0.0f;
    BusDrainHandle* b = static_cast<BusDrainHandle*>(h);
    // encoder.twist.v_x is set from dCenter/dt_s each tick — no chi-square gate.
    return b->drive.state().get_encoder().get_twist().get_v_x();
}

// ---------------------------------------------------------------------------
// kBusDrainMaxIters read — lets the test verify the constant from Python.
// ---------------------------------------------------------------------------
uint8_t bus_drain_api_max_iters()
{
    return kBusDrainMaxIters;
}

} // extern "C"
