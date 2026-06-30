// planner_api.cpp — extern "C" C-ABI shims for the MotionController2
// (Planner) subsystem (ticket 059-001).
//
// Provides an opaque PlannerHandle that owns a self-contained MotionController2
// constructed on top of SimHardware + Drive2, with its own local control
// components. Python tests (ticket 059-002) load this via ctypes and call these
// functions directly.
//
// Construction order (mirrors drive2_api.cpp / Robot.h dependency order):
//   1. cfg          — RobotConfig from defaultRobotConfig()
//   2. hal          — SimHardware(cfg)
//   3. mc_ctrl      — MotorController(hal.motorL(), hal.motorR(), cfg)
//   4. bvc          — BodyVelocityController(mc_ctrl, cfg)
//   5. est          — PhysicalStateEstimate
//   6. drive2       — Drive2(motorL, motorR, mc_ctrl, bvc, est, odo, otos, cfg)
//   7. odo_ctrl     — Odometry (owned by est; accessed via est.odometry())
//   8. motion_ctrl  — MotionController(mc_ctrl, est.odometry(), cfg)
//   9. mc2          — MotionController2(motion_ctrl, drive2, cfg)
//
// Heap allocation in PlannerHandle is acceptable — the no-heap constraint
// applies to MotionController2::tick() itself, not the test harness.

// Sprint 050, Ticket 004: EKFTiny must be included BEFORE any header that
// transitively pulls in tinyekf.h (e.g. Odometry.h → EKFTiny.h).
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
#include "subsystems/drive/Drive2.h"   // also declares toDriveConfig()
#include "superstructure/MotionController.h"
#include "superstructure/MotionController2.h"
#include "superstructure/PlannerConfig.h"
#include "messages/planner.h"
#include "messages/common.h"
#include "messages/drivetrain.h"

// ---------------------------------------------------------------------------
// PlannerHandle — opaque handle owning the full planner subsystem stack.
// ---------------------------------------------------------------------------
struct PlannerHandle {
    RobotConfig                 cfg;
    SimHardware                 hal;
    MotorController             mc_ctrl;
    BodyVelocityController      bvc;
    PhysicalStateEstimate       est;
    subsystems::Drive2          drive2;
    MotionController            motion_ctrl;
    MotionController2           mc2;

    PlannerHandle()
        : cfg(defaultRobotConfig())
        , hal(cfg)
        , mc_ctrl(hal.motorL(), hal.motorR(), cfg)
        , bvc(mc_ctrl, cfg)
        , est()
        , drive2(hal.motorL(), hal.motorR(),
                 mc_ctrl, bvc, est, est.odometry(),
                 hal.otos(), cfg)
        , motion_ctrl(mc_ctrl, est.odometry(), cfg)
        , mc2(motion_ctrl, drive2, cfg)
    {
        // Apply Drive2 config so it has live gains/lag.
        drive2.configure(toDriveConfig(cfg));
    }
};

extern "C" {

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void* planner_api_create()
{
    return new PlannerHandle();
}

void planner_api_destroy(void* h)
{
    delete static_cast<PlannerHandle*>(h);
}

// ---------------------------------------------------------------------------
// Tick
//
// Ordering mirrors the live loopTickOnce pattern:
//   hal.tick(now, drive2.outputs()) — integrate plant physics
//   hal.tick(now)                   — promote encoder into positionMm()
//   drive2.tickUpdate(now)          — SENSE: encoder collect + EKF predict
//   mc2.tick(now)                   — PLAN: driveAdvance + extract twist
//   drive2.apply(cmd)               — stage the twist command in Drive2
//   drive2.tickAction(now)          — ACT: BVC advance + motor output
// ---------------------------------------------------------------------------

// Run one full planner + drive tick. Returns the commanded vx (mm/s).
float planner_api_tick(void* h, uint32_t now_ms)
{
    PlannerHandle* p = static_cast<PlannerHandle*>(h);

    // Advance the physics plant.
    p->hal.tick(now_ms, p->drive2.outputs());
    p->hal.tick(now_ms);

    // SENSE phase.
    p->drive2.tickUpdate(now_ms);

    // PLAN phase: get the CommandBatch from MC2.
    msg::CommandBatch batch = p->mc2.tick(now_ms);

    // Stage the first TWIST command in Drive2 (if any).
    if (batch.cmds_count > 0) {
        const msg::OutCommand& oc = batch.cmds_[0];
        if (oc.verb_id == 1 && oc.args_count >= 3) {
            msg::DrivetrainCommand drvCmd;
            msg::BodyTwist3 twist{};
            twist.vx_mmps    = oc.args_[0];
            twist.vy_mmps    = oc.args_[1];
            twist.omega_rads = oc.args_[2];
            drvCmd.setTwist(twist);
            p->drive2.apply(drvCmd);
        }
    }

    // ACT phase.
    p->drive2.tickAction(now_ms);

    // Return the commanded vx for the caller to inspect.
    return p->mc2.state().get_body_twist().get_vx_mmps();
}

// ---------------------------------------------------------------------------
// Command application shims
// ---------------------------------------------------------------------------

// Apply a VELOCITY goal: vx_mmps, omega_rads.
void planner_api_apply_velocity(void* h, float vx, float omega)
{
    PlannerHandle* p = static_cast<PlannerHandle*>(h);
    msg::PlannerCommand cmd;
    msg::VelocityGoal g{};
    g.vx_mmps    = vx;
    g.omega_rads = omega;
    cmd.setVelocity(g);
    p->mc2.apply(cmd);
}

// Apply a STOP goal.
void planner_api_apply_stop(void* h)
{
    PlannerHandle* p = static_cast<PlannerHandle*>(h);
    msg::PlannerCommand cmd;
    cmd.setStop(true);
    p->mc2.apply(cmd);
}

// Apply a TURN goal: heading_rad.
void planner_api_apply_turn(void* h, float heading_rad)
{
    PlannerHandle* p = static_cast<PlannerHandle*>(h);
    msg::PlannerCommand cmd;
    msg::TurnGoal g{};
    g.heading_rad = heading_rad;
    cmd.setTurn(g);
    p->mc2.apply(cmd);
}

// Apply a TIMED goal: vx_mmps, omega_rads, duration_ms.
void planner_api_apply_timed(void* h, float vx, float omega, uint32_t duration_ms)
{
    PlannerHandle* p = static_cast<PlannerHandle*>(h);
    msg::PlannerCommand cmd;
    msg::TimedGoal g{};
    g.vx_mmps    = vx;
    g.omega_rads = omega;
    g.duration_ms = duration_ms;
    cmd.setTimed(g);
    p->mc2.apply(cmd);
}

// Apply a GOTO_GOAL: x_mm, y_mm, speed_mmps.
void planner_api_apply_goto(void* h, float x_mm, float y_mm, float speed_mmps)
{
    PlannerHandle* p = static_cast<PlannerHandle*>(h);
    msg::PlannerCommand cmd;
    msg::GotoGoal g{};
    g.x_mm       = x_mm;
    g.y_mm       = y_mm;
    g.speed_mmps = speed_mmps;
    cmd.setGotoGoal(g);
    p->mc2.apply(cmd);
}

// Apply a DISTANCE goal: distance_mm, speed_mmps.
void planner_api_apply_distance(void* h, float distance_mm, float speed_mmps)
{
    PlannerHandle* p = static_cast<PlannerHandle*>(h);
    msg::PlannerCommand cmd;
    msg::DistanceGoal g{};
    g.distance_mm = distance_mm;
    g.speed_mmps  = speed_mmps;
    cmd.setDistance(g);
    p->mc2.apply(cmd);
}

// Apply a ROTATION goal: angle_rad.
void planner_api_apply_rotation(void* h, float angle_rad)
{
    PlannerHandle* p = static_cast<PlannerHandle*>(h);
    msg::PlannerCommand cmd;
    msg::RotationGoal g{};
    g.angle_rad = angle_rad;
    cmd.setRotation(g);
    p->mc2.apply(cmd);
}

// ---------------------------------------------------------------------------
// State reads
// ---------------------------------------------------------------------------

int planner_api_get_active(void* h)
{
    return static_cast<PlannerHandle*>(h)->mc2.state().get_active() ? 1 : 0;
}

int planner_api_get_mode(void* h)
{
    return (int)static_cast<PlannerHandle*>(h)->mc2.state().get_mode();
}

float planner_api_get_body_twist_vx(void* h)
{
    return static_cast<PlannerHandle*>(h)->mc2.state().get_body_twist().get_vx_mmps();
}

float planner_api_get_body_twist_omega(void* h)
{
    return static_cast<PlannerHandle*>(h)->mc2.state().get_body_twist().get_omega_rads();
}

// ---------------------------------------------------------------------------
// Drive2 pose reads (for verifying motion)
// ---------------------------------------------------------------------------

float planner_api_get_fused_x(void* h)
{
    return static_cast<PlannerHandle*>(h)->drive2.state().get_fused().get_pose().get_x_mm();
}

float planner_api_get_fused_y(void* h)
{
    return static_cast<PlannerHandle*>(h)->drive2.state().get_fused().get_pose().get_y_mm();
}

float planner_api_get_fused_h(void* h)
{
    return static_cast<PlannerHandle*>(h)->drive2.state().get_fused().get_pose().get_h_rad();
}

// ---------------------------------------------------------------------------
// toPlannerConfig shim (for testing the projection function)
// ---------------------------------------------------------------------------

// Returns a_max from the default robot config projected through toPlannerConfig.
float planner_api_default_config_a_max()
{
    RobotConfig cfg = defaultRobotConfig();
    msg::PlannerConfig pcfg = toPlannerConfig(cfg);
    return pcfg.get_a_max();
}

float planner_api_default_config_v_body_max()
{
    RobotConfig cfg = defaultRobotConfig();
    msg::PlannerConfig pcfg = toPlannerConfig(cfg);
    return pcfg.get_v_body_max();
}

float planner_api_default_config_yaw_rate_max()
{
    RobotConfig cfg = defaultRobotConfig();
    msg::PlannerConfig pcfg = toPlannerConfig(cfg);
    return pcfg.get_yaw_rate_max();
}

float planner_api_default_config_arrive_tol_mm()
{
    RobotConfig cfg = defaultRobotConfig();
    msg::PlannerConfig pcfg = toPlannerConfig(cfg);
    return pcfg.get_arrive_tol_mm();
}

} // extern "C"
