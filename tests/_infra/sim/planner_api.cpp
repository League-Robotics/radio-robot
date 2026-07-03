// planner_api.cpp — extern "C" C-ABI shims for the Planner subsystem
// (ticket 059-001; updated 061-004: MotionController absorbed into Planner).
//
// Provides an opaque PlannerHandle that owns a self-contained Planner
// constructed on top of SimHardware + Drive, with its own local control
// components. Python tests (ticket 059-002) load this via ctypes and call these
// functions directly.
//
// Construction order (mirrors Robot.h dependency order):
//   1. cfg          — RobotConfig from defaultRobotConfig()
//   2. hal          — SimHardware(cfg)
//   3. mc_ctrl      — MotorController(hal.motorL(), hal.motorR(), cfg)
//   4. bvc          — BodyVelocityController(mc_ctrl, cfg)
//   5. est          — PhysicalStateEstimate
//   6. drive        — Drive(motorL, motorR, mc_ctrl, bvc, est, odo, otos, cfg)
//   7. planner      — Planner(mc_ctrl, est.odometry(), drive, cfg)
//
// Heap allocation in PlannerHandle is acceptable — the no-heap constraint
// applies to Planner::tick() itself, not the test harness.

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
#include "subsystems/drive/Drive.h"    // also declares toDriveConfig()
#include "superstructure/Planner.h"
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
    subsystems::Drive           drive;
    Planner                     planner;

    PlannerHandle()
        : cfg(defaultRobotConfig())
        , hal(cfg)
        , mc_ctrl(hal.motorL(), hal.motorR(), cfg)
        , bvc(mc_ctrl, cfg)
        , est()
        , drive(hal.motorL(), hal.motorR(),
                mc_ctrl, bvc, est, est.odometry(),
                hal.otos(), cfg)
        , planner(mc_ctrl, est.odometry(), drive, cfg)
    {
        // Apply Drive config so it has live gains/lag.
        drive.configure(toDriveConfig(cfg));
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
//   hal.tick(now, drive.outputs()) — integrate plant physics
//   hal.tick(now)                  — promote encoder into position()
//   drive.tickUpdate(now)          — SENSE: encoder collect + EKF predict
//   planner.tick(now)              — PLAN: driveAdvance + extract twist
//   drive.apply(cmd)               — stage the twist command in Drive
//   drive.tickAction(now)          — ACT: BVC advance + motor output
// ---------------------------------------------------------------------------

// Run one full planner + drive tick. Returns the commanded vx (mm/s).
float planner_api_tick(void* h, uint32_t now_ms)
{
    PlannerHandle* p = static_cast<PlannerHandle*>(h);

    // Advance the physics plant.
    p->hal.tick(now_ms, p->drive.outputs());
    p->hal.tick(now_ms);

    // SENSE phase.
    p->drive.tickUpdate(now_ms);

    // PLAN phase: get the CommandBatch from MC2.
    msg::CommandBatch batch = p->planner.tick(now_ms);

    // Stage the first TWIST command in Drive (if any).
    if (batch.cmds_count > 0) {
        const msg::OutCommand& oc = batch.cmds_[0];
        if (oc.verb_id == 1 && oc.args_count >= 3) {
            msg::DrivetrainCommand drvCmd;
            msg::BodyTwist3 twist{};
            twist.v_x    = oc.args_[0];
            twist.v_y    = oc.args_[1];
            twist.omega = oc.args_[2];
            drvCmd.setTwist(twist);
            p->drive.apply(drvCmd);
        }
    }

    // ACT phase.
    p->drive.tickAction(now_ms);

    // Return the commanded vx for the caller to inspect.
    return p->planner.state().get_body_twist().get_v_x();
}

// ---------------------------------------------------------------------------
// Command application shims
// ---------------------------------------------------------------------------

// Apply a VELOCITY goal: vx_mmps, omega_rads.
// now_ms: real system time at apply() (066-002 / CR-11) — baselines
// MotionBaseline.t0Ms via begin*() -> MotionCommand::start().
void planner_api_apply_velocity(void* h, float vx, float omega, uint32_t now_ms)
{
    PlannerHandle* p = static_cast<PlannerHandle*>(h);
    msg::PlannerCommand cmd;
    msg::VelocityGoal g{};
    g.v_x    = vx;
    g.omega = omega;
    cmd.setVelocity(g);
    p->planner.apply(cmd, now_ms);
}

// Apply a STOP goal.
void planner_api_apply_stop(void* h, uint32_t now_ms)
{
    PlannerHandle* p = static_cast<PlannerHandle*>(h);
    msg::PlannerCommand cmd;
    cmd.setStop(true);
    p->planner.apply(cmd, now_ms);
}

// Apply a TURN goal: heading_rad.
void planner_api_apply_turn(void* h, float heading_rad, uint32_t now_ms)
{
    PlannerHandle* p = static_cast<PlannerHandle*>(h);
    msg::PlannerCommand cmd;
    msg::TurnGoal g{};
    g.heading = heading_rad;
    cmd.setTurn(g);
    p->planner.apply(cmd, now_ms);
}

// Apply a TIMED goal: vx_mmps, omega_rads, duration_ms.
// now_ms: staged as the goal's t0Ms baseline — the guard test
// (test_planner_apply_now_ms.py) asserts the TIME stop does not fire until
// now_ms + duration_ms has elapsed, not on the tick immediately after apply().
void planner_api_apply_timed(void* h, float vx, float omega, uint32_t duration_ms,
                             uint32_t now_ms)
{
    PlannerHandle* p = static_cast<PlannerHandle*>(h);
    msg::PlannerCommand cmd;
    msg::TimedGoal g{};
    g.v_x    = vx;
    g.omega = omega;
    g.duration = duration_ms;
    cmd.setTimed(g);
    p->planner.apply(cmd, now_ms);
}

// Apply a GOTO_GOAL: x_mm, y_mm, speed_mmps.
void planner_api_apply_goto(void* h, float x_mm, float y_mm, float speed_mmps, uint32_t now_ms)
{
    PlannerHandle* p = static_cast<PlannerHandle*>(h);
    msg::PlannerCommand cmd;
    msg::GotoGoal g{};
    g.x       = x_mm;
    g.y       = y_mm;
    g.speed = speed_mmps;
    cmd.setGotoGoal(g);
    p->planner.apply(cmd, now_ms);
}

// Apply a DISTANCE goal: distance_mm, speed_mmps.
void planner_api_apply_distance(void* h, float distance_mm, float speed_mmps, uint32_t now_ms)
{
    PlannerHandle* p = static_cast<PlannerHandle*>(h);
    msg::PlannerCommand cmd;
    msg::DistanceGoal g{};
    g.distance = distance_mm;
    g.speed  = speed_mmps;
    cmd.setDistance(g);
    p->planner.apply(cmd, now_ms);
}

// Apply a ROTATION goal: angle_rad.
void planner_api_apply_rotation(void* h, float angle_rad, uint32_t now_ms)
{
    PlannerHandle* p = static_cast<PlannerHandle*>(h);
    msg::PlannerCommand cmd;
    msg::RotationGoal g{};
    g.angle = angle_rad;
    cmd.setRotation(g);
    p->planner.apply(cmd, now_ms);
}

// ---------------------------------------------------------------------------
// State reads
// ---------------------------------------------------------------------------

int planner_api_get_active(void* h)
{
    return static_cast<PlannerHandle*>(h)->planner.state().get_active() ? 1 : 0;
}

// Alias for planner_api_get_active — satisfies ticket 059-002 shim contract.
int planner_api_is_active(void* h)
{
    return static_cast<PlannerHandle*>(h)->planner.state().get_active() ? 1 : 0;
}

int planner_api_get_mode(void* h)
{
    return (int)static_cast<PlannerHandle*>(h)->planner.state().get_mode();
}

float planner_api_get_body_twist_vx(void* h)
{
    return static_cast<PlannerHandle*>(h)->planner.state().get_body_twist().get_v_x();
}

float planner_api_get_body_twist_omega(void* h)
{
    return static_cast<PlannerHandle*>(h)->planner.state().get_body_twist().get_omega();
}

// ---------------------------------------------------------------------------
// Drive pose reads (for verifying motion)
// ---------------------------------------------------------------------------

float planner_api_get_fused_x(void* h)
{
    return static_cast<PlannerHandle*>(h)->drive.state().get_fused().get_pose().get_x();
}

float planner_api_get_fused_y(void* h)
{
    return static_cast<PlannerHandle*>(h)->drive.state().get_fused().get_pose().get_y();
}

float planner_api_get_fused_h(void* h)
{
    return static_cast<PlannerHandle*>(h)->drive.state().get_fused().get_pose().get_h();
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
    return pcfg.get_arrive_tol();
}

} // extern "C"
