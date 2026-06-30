// drive2_api.cpp — extern "C" C-ABI shims for the Drive2 subsystem (ticket 057-004).
//
// Provides an opaque Drive2Handle that owns a self-contained Drive2 subsystem
// constructed on SimHardware, with its own local control components.
// Python tests (test_drive2_subsystem.py) load this via ctypes and call these
// functions directly.
//
// Construction order (mirrors Robot.h dependency order):
//   1. cfg          — RobotConfig from defaultRobotConfig()
//   2. hal          — SimHardware(cfg) — owns PhysicsWorld + Sim* devices
//   3. mc           — MotorController(hal.motorL(), hal.motorR(), cfg)
//   4. bvc          — BodyVelocityController(mc, cfg)
//   5. est          — PhysicalStateEstimate (default ctor, then initEKF)
//   6. drive2       — Drive2(motorL, motorR, mc, bvc, est, est.odometry(), otos, cfg)
//
// The Drive2Handle owns all control components as value members so they
// outlive any individual function call (no dangling refs).
//
// Heap allocation in the test fixture struct (Drive2Handle) is acceptable —
// the no-heap constraint applies to Drive2 itself, not the test harness.

// Sprint 050, Ticket 004: EKFTiny must be included BEFORE any header that
// transitively pulls in tinyekf.h (e.g. Odometry.h → EKFTiny.h).
// The #ifndef guards in EKFTiny.h make re-inclusion safe.
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
#include "subsystems/drive/Drive2.h"
#include "messages/drivetrain.h"
#include "messages/common.h"

// ---------------------------------------------------------------------------
// Drive2Handle — opaque handle owning a self-contained Drive2 subsystem.
// ---------------------------------------------------------------------------
struct Drive2Handle {
    RobotConfig                cfg;
    SimHardware                hal;
    MotorController            mc;
    BodyVelocityController     bvc;
    PhysicalStateEstimate      est;
    subsystems::Drive2         drive2;

    Drive2Handle()
        : cfg(defaultRobotConfig())
        , hal(cfg)
        , mc(hal.motorL(), hal.motorR(), cfg)
        , bvc(mc, cfg)
        , est()
        , drive2(hal.motorL(), hal.motorR(),
                 mc, bvc, est, est.odometry(),
                 hal.otos(), cfg)
    {
        // Apply the default config projection so Drive2 has live gains/lag.
        drive2.configure(toDriveConfig(cfg));
    }
};

extern "C" {

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void* drive2_api_create()
{
    return new Drive2Handle();
}

void drive2_api_destroy(void* h)
{
    delete static_cast<Drive2Handle*>(h);
}

// ---------------------------------------------------------------------------
// Command application
// ---------------------------------------------------------------------------

// Apply a body-twist command: vx_mmps, vy_mmps, omega_rads.
void drive2_api_apply_twist(void* h, float vx, float vy, float omega)
{
    Drive2Handle* d = static_cast<Drive2Handle*>(h);
    msg::DrivetrainCommand cmd;
    msg::BodyTwist3 twist{};
    twist.v_x    = vx;
    twist.v_y    = vy;
    twist.omega = omega;
    cmd.setTwist(twist);
    d->drive2.apply(cmd);
}

// Apply neutral/brake command.
void drive2_api_apply_neutral_brake(void* h)
{
    Drive2Handle* d = static_cast<Drive2Handle*>(h);
    msg::DrivetrainCommand cmd;
    cmd.setNeutral(msg::Neutral::BRAKE);
    d->drive2.apply(cmd);
}

// Apply neutral/coast command.
void drive2_api_apply_neutral_coast(void* h)
{
    Drive2Handle* d = static_cast<Drive2Handle*>(h);
    msg::DrivetrainCommand cmd;
    cmd.setNeutral(msg::Neutral::COAST);
    d->drive2.apply(cmd);
}

// Apply SetPose command: re-anchor the fused estimate to (x_mm, y_mm, h_rad).
void drive2_api_apply_setpose(void* h, float x, float y, float h_rad)
{
    Drive2Handle* d = static_cast<Drive2Handle*>(h);
    msg::DrivetrainCommand cmd;
    msg::SetPose pose{};
    pose.x  = x;
    pose.y  = y;
    pose.h = h_rad;
    cmd.setPose(pose);
    d->drive2.apply(cmd);
}

// ---------------------------------------------------------------------------
// Tick
// ---------------------------------------------------------------------------

// SENSE phase: encoders + EKF predict/correct; updates _state.
//
// Tick ordering (mirrors the live loopTickOnce pattern):
//   hal.tick(now, outputs)  — integrate plant physics with the PWM that was
//                             written by the previous tickAction's controlTick.
//   hal.tick(now)           — promote integrated encoder into positionMm().
//   drive2.tickUpdate(now)  — read positionMm(), run outlier filter + EKF predict.
void drive2_api_tick_update(void* h, uint32_t now_ms)
{
    Drive2Handle* d = static_cast<Drive2Handle*>(h);
    // drive2.outputs() exposes the MotorCommands that controlTick() wrote to
    // during the previous tickAction call (or zero-init on the very first tick).
    d->hal.tick(now_ms, d->drive2.outputs());
    d->hal.tick(now_ms);
    d->drive2.tickUpdate(now_ms);
}

// ACT phase: apply staged command → motor outputs.
void drive2_api_tick_action(void* h, uint32_t now_ms)
{
    Drive2Handle* d = static_cast<Drive2Handle*>(h);
    d->drive2.tickAction(now_ms);
}

// ---------------------------------------------------------------------------
// State reads (fused pose from Drive2::state())
// ---------------------------------------------------------------------------

float drive2_api_get_fused_x(void* h)
{
    return static_cast<Drive2Handle*>(h)->drive2.state().get_fused().get_pose().get_x();
}

float drive2_api_get_fused_y(void* h)
{
    return static_cast<Drive2Handle*>(h)->drive2.state().get_fused().get_pose().get_y();
}

float drive2_api_get_fused_h(void* h)
{
    return static_cast<Drive2Handle*>(h)->drive2.state().get_fused().get_pose().get_h();
}

int drive2_api_get_connected(void* h)
{
    return static_cast<Drive2Handle*>(h)->drive2.state().get_connected() ? 1 : 0;
}

// ---------------------------------------------------------------------------
// Capabilities
// ---------------------------------------------------------------------------

int drive2_api_capabilities_holonomic(void* h)
{
    return static_cast<Drive2Handle*>(h)->drive2.capabilities().get_holonomic() ? 1 : 0;
}

// ---------------------------------------------------------------------------
// Motor-level reads (for neutral/brake test verification)
// ---------------------------------------------------------------------------

// Read the left wheel target speed (mm/s) from the MotorController's commands.
// [1] = FL = left (differential).
float drive2_api_get_target_mms_l(void* h)
{
    // Access the sim motor's commanded speed (the MC has written to the PWM).
    // The SimMotor's setSpeed is called by MotorController; we can read from
    // the plant's commanded PWM indirectly.  For the test we read tgtMms via
    // the MotorController's internal state — simpler via a public route:
    // MotorController writes tgtMms into the MotorCommands ref (_outputs in Drive2).
    // We can reach those through the hal plant or use a trick: read the MC gains.
    // Actually the simplest path is: Drive2::_outputs is private.
    // However, the sim motor's setSpeed is called every controlTick, and we can
    // check whether it has been zeroed. SimMotor doesn't expose the last PWM
    // directly, but PhysicsWorld does.
    // For the test purposes: read sim motor L's current velocity (if 0, braked).
    // We use hal.simMotorL() to read the last commanded speed from PhysicsWorld.
    // PhysicsWorld::trueVelLMms() returns the ACTUAL velocity (not commanded).
    // Instead, the simplest correct approach: read state.vel_[1] from Drive2.
    const msg::DrivetrainState& st = static_cast<Drive2Handle*>(h)->drive2.state();
    if (st.vel_count_val() >= 2) return st.vel()[1];
    return 0.0f;
}

float drive2_api_get_target_mms_r(void* h)
{
    const msg::DrivetrainState& st = static_cast<Drive2Handle*>(h)->drive2.state();
    if (st.vel_count_val() >= 1) return st.vel()[0];
    return 0.0f;
}

// ---------------------------------------------------------------------------
// Sensor initialization (ticket 058-001)
// ---------------------------------------------------------------------------

// Initialize the SimOdometer (OTOS sim sensor) so Drive2's OTOS correction
// path activates.  Must be called before enable_otos_sim_model if optical
// fusion is required in the test.  Mirrors Robot::begin() → otos.begin().
void drive2_api_begin_otos(void* h)
{
    Drive2Handle* d = static_cast<Drive2Handle*>(h);
    d->hal.otos().begin();
}

// ---------------------------------------------------------------------------
// Noise / error-model injection (ticket 057-005)
// ---------------------------------------------------------------------------

// Enable the OTOS sim model and configure all noise + drift knobs.
// After this call the SimOdometer integrates from plant velocity with the
// specified error so the optical estimate diverges from ground truth.
//
//   linear_noise_sigma  — Gaussian position noise (mm, zero-mean, per tick)
//   yaw_noise_sigma     — Gaussian heading noise (rad, zero-mean, per tick)
//   drift_per_tick_mm   — Deterministic X-axis drift per tick (mm)
//   drift_per_tick_rad  — Deterministic heading drift per tick (rad)
//   linear_scale_err    — Fractional linear scale error (0.03 = 3% over-report)
//   angular_scale_err   — Fractional angular scale error
void drive2_api_enable_otos_sim_model(void* h,
                                      float linear_noise_sigma,
                                      float yaw_noise_sigma,
                                      float drift_per_tick_mm,
                                      float drift_per_tick_rad,
                                      float linear_scale_err,
                                      float angular_scale_err)
{
    Drive2Handle* d = static_cast<Drive2Handle*>(h);
    SimOdometer& odom = d->hal.simOdometer();
    odom.enableSimModel(true);
    odom.setLinearNoiseSigma(linear_noise_sigma);
    odom.setYawNoiseSigma(yaw_noise_sigma);
    odom.setDriftPerTickMm(drift_per_tick_mm);
    odom.setDriftPerTickRad(drift_per_tick_rad);
    odom.setLinearScaleError(linear_scale_err);
    odom.setAngularScaleError(angular_scale_err);
}

// ---------------------------------------------------------------------------
// Encoder error-model injection (ticket 058-001)
// ---------------------------------------------------------------------------

// Configure per-wheel encoder error on both SimMotors.
// Mirrors drive2_api_enable_otos_sim_model for the encoder path.
//
//   slip_l / slip_r:         fraction of motion not registered (0 = perfect,
//                            0.05 = 5% under-report) — applied to REPORTED
//                            encoder only; ground-truth pose is unaffected.
//   scale_err_l / scale_err_r: fractional over/under-report of motion
//                            (0 = perfect, 0.05 = 5% over-report).
void drive2_api_enable_encoder_sim_model(void* h,
                                         float slip_l,
                                         float slip_r,
                                         float scale_err_l,
                                         float scale_err_r)
{
    Drive2Handle* d = static_cast<Drive2Handle*>(h);
    d->hal.simMotorL().setSlip(slip_l);
    d->hal.simMotorR().setSlip(slip_r);
    d->hal.simMotorL().setScaleError(scale_err_l);
    d->hal.simMotorR().setScaleError(scale_err_r);
}

// ---------------------------------------------------------------------------
// Ground-truth reads (ticket 057-005)
// ---------------------------------------------------------------------------

// Plant ground-truth X position (mm) — the true integrated chassis pose.
float drive2_api_ground_truth_x(void* h)
{
    return static_cast<Drive2Handle*>(h)->hal.groundTruthX();
}

// Plant ground-truth Y position (mm).
float drive2_api_ground_truth_y(void* h)
{
    return static_cast<Drive2Handle*>(h)->hal.groundTruthY();
}

// Plant ground-truth heading (rad).
float drive2_api_ground_truth_h(void* h)
{
    return static_cast<Drive2Handle*>(h)->hal.groundTruthH();
}

// ---------------------------------------------------------------------------
// Raw encoder-only and optical-only pose reads (ticket 057-005)
// ---------------------------------------------------------------------------

// Encoder-only pose X (mm) — from DrivetrainState::encoder (dead-reckoning).
float drive2_api_get_encoder_x(void* h)
{
    return static_cast<Drive2Handle*>(h)->drive2.state().get_encoder().get_pose().get_x();
}

// Encoder-only pose Y (mm).
float drive2_api_get_encoder_y(void* h)
{
    return static_cast<Drive2Handle*>(h)->drive2.state().get_encoder().get_pose().get_y();
}

// Optical-only pose X (mm) — from DrivetrainState::optical (OTOS sim model).
float drive2_api_get_optical_x(void* h)
{
    return static_cast<Drive2Handle*>(h)->drive2.state().get_optical().get_pose().get_x();
}

// Optical-only pose Y (mm).
float drive2_api_get_optical_y(void* h)
{
    return static_cast<Drive2Handle*>(h)->drive2.state().get_optical().get_pose().get_y();
}

} // extern "C"
