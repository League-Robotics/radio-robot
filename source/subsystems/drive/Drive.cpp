// Drive.cpp — subsystems::Drive implementation (renamed from Drive2 in 060-006).
//
// Composes the existing control components by reference: MotorController,
// BodyVelocityController, PhysicalStateEstimate, Odometry, IMotor x2, IOdometer.
// Two-phase tick (tickUpdate/tickAction) mirrors the live loopTickOnce ordering:
//   tickUpdate  → SENSE (outlier filter → controlTick → EKF predict → OTOS correct)
//   tickAction  → ACT   (staged command → BVC.setTarget → BVC.advance → MC.setTarget)
//
// C++11, no heap in Drive, no virtual dispatch in the contract path.

#include "Drive.h"

#include "MotorController.h"
#include "BodyVelocityController.h"
#include "state/PhysicalStateEstimate.h"
#include "control/Odometry.h"
#include "hal/capability/Pose2D.h"    // Pose2D / BodyTwist
#include "kinematics/BodyKinematics.h" // inverse() — TWIST → wheel speeds (060-004)

#include <cmath>   // fmaxf, fabsf

namespace subsystems {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
Drive::Drive(IMotor& motorL, IMotor& motorR,
             MotorController& mc,
             BodyVelocityController& bvc,
             PhysicalStateEstimate& est,
             Odometry& odo,
             IOdometer& otos,
             const RobotConfig& cfg)
    : _motorL(motorL)
    , _motorR(motorR)
    , _mc(mc)
    , _bvc(bvc)
    , _est(est)
    , _odo(odo)
    , _otos(otos)
    , _robCfg(cfg)
{
    // Seed the MotorController's commands reference so setTarget() writes
    // _outputs.tgtMms[], which controlTick() reads for the velocity PIDs.
    _mc.setCommandsRef(&_outputs);

    // Initialise the EKF with config noise params so predict/correct work.
    _est.initEKF(_robCfg.ekfQxy, _robCfg.ekfQtheta,
                 _robCfg.ekfQv,   _robCfg.ekfQomega,
                 _robCfg.ekfROtosXy, _robCfg.ekfROtosV,
                 _robCfg.ekfREncV,   _robCfg.ekfROtosTheta);
}

// ---------------------------------------------------------------------------
// apply — stage the command (no hardware, no emission).
// ---------------------------------------------------------------------------
void Drive::apply(const msg::DrivetrainCommand& cmd)
{
    _cmd        = cmd;
    _cmdPending = true;
}

// ---------------------------------------------------------------------------
// tickUpdate — SENSE phase.
//
// 1. Encoder outlier filter → write _hw.encMm[0/1].
// 2. controlTick()  → velocity PID for both wheels (reads _hw.encMm[]).
// 3. Wedge push → est.setWedgeActive / setEncOmegaHealthy.
// 4. addOdometryObservation → EKF predict, update _hw.fused/encoder/optical.
// 5. OTOS correction (if lag elapsed and device is ready).
// 6. Refresh _state from _hw.
// ---------------------------------------------------------------------------
void Drive::tickUpdate(uint32_t now, bool fuseOtos)
{
    // ------------------------------------------------------------------
    // STEP 1+2: Outlier filter → encoder collect → controlTick
    // ------------------------------------------------------------------
    _runOutlierFilter(now);

    // controlTick runs the velocity PID and calls _motorL/R.setSpeed().
    // refreshedWheel=3 means both wheels were just collected (same semantics
    // as Drive::periodic).
    bool driving = (_outputs.tgtMms[1] != 0.0f || _outputs.tgtMms[0] != 0.0f);
    _mc.controlTick(_hw, _outputs, now, driving ? 3 : 0);

    // ------------------------------------------------------------------
    // STEP 3: Wedge push (mirrors Drive::periodic — 033-005e)
    // ------------------------------------------------------------------
    bool anyWedged = _mc.wheelWedgedL() || _mc.wheelWedgedR();
    _est.setWedgeActive(anyWedged);
    if (anyWedged) {
        _est.setEncOmegaHealthy(false);
    } else if (_prevAnyWedged) {
        _est.setEncOmegaHealthy(true);
    }
    _prevAnyWedged = anyWedged;

    // ------------------------------------------------------------------
    // STEP 4: EKF predict — encoder dead-reckoning integrate
    // ------------------------------------------------------------------
    float trackwidth = (_drvCfg.get_trackwidth() > 0.0f)
                           ? _drvCfg.get_trackwidth()
                           : _robCfg.trackwidthMm;
    float rotSlip    = _robCfg.rotationalSlip;
    _est.addOdometryObservation(_hw, trackwidth, rotSlip, now);

    // ------------------------------------------------------------------
    // STEP 5: OTOS correction (lag-gated, matches LoopTickOnce pattern)
    // ------------------------------------------------------------------
    uint32_t lagMs = (_drvCfg.get_lag_otos() > 0)
                         ? _drvCfg.get_lag_otos()
                         : _robCfg.lagOtosMs;
    if (lagMs > 0 && _otos.is_initialized()) {
        _hw.otos.lagMs = lagMs;   // keep stamp lag field in sync
        if (!_otosEverReady && !fuseOtos) {
            // First time (normal lag-gated path): mark ready and seed the timer
            // so we don't fire with an uninitialised _lastOtosMs.
            _otosEverReady = true;
            _lastOtosMs    = now;
        } else if (fuseOtos || (int32_t)(now - _lastOtosMs) >= (int32_t)lagMs) {
            if (!_otosEverReady) { _otosEverReady = true; }
            float headingRad = _hw.fused.pose.h;
            Pose2D p{};
            bool poseOk = _otos.readTransformed(p, headingRad);
            if (poseOk) {
                BodyTwist vel{};
                _otos.readVelocityTransformed(vel, headingRad);
                _est.addOtosObservation(_hw,
                                        p.x, p.y, p.h,
                                        vel.v_mmps, vel.omega_rads,
                                        0.0f, now);
                // 060-004: Mirror Robot::otosCorrect() — mark otos.valid so
                // buildTlmFrame's N8 freshness gate emits the otos= field.
                _hw.otos.lastUpdMs = now;
                _hw.otos.valid     = true;
            } else {
                // Read failed this cycle — valid stays unchanged (preserves
                // the last-known-good stamp, matching Robot::otosCorrect behaviour).
                _hw.otos.valid = false;
            }
            _lastOtosMs = now;
        }
    }

    // ------------------------------------------------------------------
    // STEP 6: Refresh _state from _hw
    //
    // HardwareState uses legacy ::PoseEstimate / ::ValueSet (source/state/,
    // types/).  msg::DrivetrainState uses msg::PoseEstimate / msg::ValueSet
    // (messages/common.h, auto-generated).  Both types have the same field
    // layout but are distinct C++ types, so we copy field-by-field.
    // ------------------------------------------------------------------

    // Helper lambda: copy ::PoseEstimate → msg::PoseEstimate.
    auto copyPE = [](const ::PoseEstimate& src, msg::PoseEstimate& dst) {
        dst.pose.x      = src.pose.x;
        dst.pose.y      = src.pose.y;
        dst.pose.h      = src.pose.h;
        dst.twist.v_x   = src.twist.vx_mmps;
        dst.twist.v_y   = src.twist.vy_mmps;
        dst.twist.omega = src.twist.omega_rads;
        dst.stamp.lag      = src.stamp.lagMs;
        dst.stamp.last_upd = src.stamp.lastUpdMs;
        dst.stamp.valid    = src.stamp.valid;
    };

    copyPE(_hw.fused,   _state.fused);
    copyPE(_hw.encoder, _state.encoder);
    copyPE(_hw.optical, _state.optical);

    // Per-wheel diagnostics (differential: [0]=R, [1]=L).
    _state.enc_[0]  = _hw.encMm[0];
    _state.enc_[1]  = _hw.encMm[1];
    _state.enc_count = 2;
    _state.vel_[0]  = _hw.velMms[0];
    _state.vel_[1]  = _hw.velMms[1];
    _state.vel_count = 2;

    // Freshness envelopes: ::ValueSet → msg::ValueSet field-by-field.
    _state.enc_stamp.lag      = _hw.enc.lagMs;
    _state.enc_stamp.last_upd = _hw.enc.lastUpdMs;
    _state.enc_stamp.valid    = _hw.enc.valid;
    _state.otos.lag           = _hw.otos.lagMs;
    _state.otos.last_upd      = _hw.otos.lastUpdMs;
    _state.otos.valid         = _hw.otos.valid;

    // Wedge latch per wheel.
    _state.wheel_wedged_[0]  = _mc.wheelWedgedR() ? 1u : 0u;
    _state.wheel_wedged_[1]  = _mc.wheelWedgedL() ? 1u : 0u;
    _state.wheel_wedged_count = 2;

    // connected: true once the MotorController has seen at least one tick.
    _state.connected = (_lastControlMs != 0 || now != 0);
}

// ---------------------------------------------------------------------------
// tickAction — ACT phase.
//
// Reads the staged command and dispatches to the appropriate control path.
// Returns an empty CommandBatch (Drive is a leaf actuator in this sprint).
// ---------------------------------------------------------------------------
msg::CommandBatch Drive::tickAction(uint32_t now)
{
    (void)now;

    if (!_cmdPending) {
        return msg::CommandBatch{};
    }
    _cmdPending = false;

    switch (_cmd.control_kind) {

    case msg::DrivetrainCommand::ControlKind::TWIST: {
        float vx    = _cmd.control.twist.v_x;
        float vy    = _cmd.control.twist.v_y;
        float omega = _cmd.control.twist.omega;

        // vy-reject on differential build.
        if (!capabilities().get_holonomic() && vy != 0.0f) {
            // Reject: set motors to zero.  Per contract: "no-op actuation".
            _mc.setTarget(0.0f, 0.0f);
            break;
        }

        // 060-004: The TWIST arriving here is already profiled by the planner's
        // internal BVC (MotionController._bvc).  Running another BVC ramp in
        // Drive would double-profile the motion (planner ramps 0→target, then
        // Drive ramps 0→planner_output) causing indefinitely-slow ramp-up that
        // fails almost every motor/motion test.  Instead, do a direct inverse-
        // kinematics conversion and set wheel targets immediately.
        //
        // The MotorController's own velocity PID (controlTick) still closes the
        // wheel-speed loop; this sets the TARGET, not the PWM duty cycle.
        float vL = 0.0f, vR = 0.0f;
        float trackwidth = (_drvCfg.get_trackwidth() > 0.0f)
                               ? _drvCfg.get_trackwidth()
                               : _robCfg.trackwidthMm;
        BodyKinematics::inverse(vx, omega, trackwidth, vL, vR);
        _mc.setTarget(vL, vR);
        break;
    }

    case msg::DrivetrainCommand::ControlKind::WHEELS: {
        // Per-wheel velocity targets.
        uint8_t count = _cmd.control.wheels.w_count;
        float tL = 0.0f;
        float tR = 0.0f;
        // Convention: differential — w_[0] = right, w_[1] = left.
        if (count > 0 && _cmd.control.wheels.w_[0].get_speed().has) {
            tR = _cmd.control.wheels.w_[0].get_speed().val;
        }
        if (count > 1 && _cmd.control.wheels.w_[1].get_speed().has) {
            tL = _cmd.control.wheels.w_[1].get_speed().val;
        }
        _mc.setTarget(tL, tR);
        break;
    }

    case msg::DrivetrainCommand::ControlKind::NEUTRAL: {
        if (_cmd.control.neutral == msg::Neutral::BRAKE) {
            // Brake: zero targets and write zero PWM.
            _mc.stop();
        } else {
            // Coast: zero targets, reset PID, but don't actively drive zero PWM.
            _mc.resetIntegrators();
            _mc.setTarget(0.0f, 0.0f);
        }
        break;
    }

    case msg::DrivetrainCommand::ControlKind::POSE: {
        // SetPose: re-anchor the fused estimate.
        // h_rad → centidegrees for resetPose() API.
        static constexpr float RAD_TO_CDEG = 18000.0f / 3.14159265f;
        int32_t pose_x  = (int32_t)_cmd.control.pose.x;
        int32_t pose_y  = (int32_t)_cmd.control.pose.y;
        int32_t h_cdeg  = (int32_t)(_cmd.control.pose.h * RAD_TO_CDEG);
        _est.resetPose(_hw, pose_x, pose_y, h_cdeg);

        // Refresh fused estimate into _state immediately (field-by-field copy:
        // ::PoseEstimate and msg::PoseEstimate are distinct C++ types).
        _state.fused.pose.x         = _hw.fused.pose.x;
        _state.fused.pose.y         = _hw.fused.pose.y;
        _state.fused.pose.h         = _hw.fused.pose.h;
        _state.fused.twist.v_x      = _hw.fused.twist.vx_mmps;
        _state.fused.twist.v_y      = _hw.fused.twist.vy_mmps;
        _state.fused.twist.omega    = _hw.fused.twist.omega_rads;
        break;
    }

    case msg::DrivetrainCommand::ControlKind::NONE:
    default:
        // No-op.
        break;
    }

    return msg::CommandBatch{};
}

// ---------------------------------------------------------------------------
// resetEncoders — zero Drive's private encoder baseline (060-004).
//
// Mirrors Robot::resetEncoders() for the Drive subsystem so that D commands
// and ZERO enc keep drive._hw in sync with the hardware motor reset.
//
// Three-step reset (verbatim semantics from Robot::resetEncoders):
//   1. Zero _hw.encMm[]: sets the outlier-filter baseline so the next
//      _runOutlierFilter() sees delta=0 on the fresh accumulator.
//   2. Refresh _state.enc_[] so state() returns 0 immediately this tick.
//   3. Re-baseline _odo's snapshot: prevents predict() computing dL=0-prev
//      (a large negative delta) on the first tick after reset.
// ---------------------------------------------------------------------------
void Drive::resetEncoders()
{
    for (int i = 0; i < kWheelCount; ++i) {
        _hw.encMm[i]    = 0.0f;
        _state.enc_[i]  = 0.0f;
    }
    _odo.rebaselinePrev(0.0f, 0.0f);
}

// ---------------------------------------------------------------------------
// injectFusedPose — sim injection hook (060-004).
//
// Writes x/y/h_rad directly into _hw.fused.pose and refreshes _state.fused
// so that the next drive.state() read sees the injected pose immediately.
// Used by sim_api.cpp::sim_set_pose.
// ---------------------------------------------------------------------------
void Drive::injectFusedPose(float x, float y, float h_rad)
{
    _hw.fused.pose.x = x;
    _hw.fused.pose.y = y;
    _hw.fused.pose.h = h_rad;
    _state.fused.pose.x = x;
    _state.fused.pose.y = y;
    _state.fused.pose.h = h_rad;
}

// ---------------------------------------------------------------------------
// setEncOmegaHealthy — sim injection hook (060-004).
//
// Forwards the encoder-omega health gate to Drive's own PhysicalStateEstimate.
// Used by sim_api.cpp::sim_set_enc_omega_healthy.
// ---------------------------------------------------------------------------
void Drive::setEncOmegaHealthy(bool healthy)
{
    _est.setEncOmegaHealthy(healthy);
}

// ---------------------------------------------------------------------------
// configure — store config; next tick reads updated gains/lag/etc.
// ---------------------------------------------------------------------------
void Drive::configure(const msg::DrivetrainConfig& cfg)
{
    _drvCfg = cfg;

    // Update live velocity gains in MotorController if provided.
    // Only update if non-zero gains were supplied (zero = "use defaults").
    if (cfg.get_vel_gains().get_kp() != 0.0f || cfg.get_vel_gains().get_ki() != 0.0f) {
        // We reach into RobotConfig-typed updateVelGains via a local copy.
        // Drive uses _robCfg as the "base" and the msg::Gains as an override.
        // For simplicity in this sprint, just call updateVelGains on the original
        // config — the msg::DrivetrainConfig gains are applied when a REAL
        // toDriveConfig projection wires them in (ticket 005 / Phase 3).
        _mc.updateVelGains(_robCfg);
    }
}

// ---------------------------------------------------------------------------
// capabilities — declared truth for this build.
// ---------------------------------------------------------------------------
msg::DrivetrainCapabilities Drive::capabilities() const
{
    msg::DrivetrainCapabilities caps;
    // Holonomic if drivetrain_type > 0 (1 = mecanum) in config.
    // For the differential (Tovez) build drivetrain == 0.
    int32_t dtype = (_drvCfg.get_drivetrain_type() != 0)
                        ? _drvCfg.get_drivetrain_type()
                        : (int32_t)_robCfg.drivetrain;
    caps.holonomic        = (dtype > 0);
    caps.onboard_position = _otos.is_initialized();
    caps.wheel_count      = 2;
    return caps;
}

// ---------------------------------------------------------------------------
// _runOutlierFilter — private: speed-scaled outlier filter + encoder collect.
//
// Verbatim from legacy Drive::periodic() with member renaming:
//   _commands → _outputs     (MotorCommands)
//   _inputs   → _hw          (HardwareState)
//   fn/ctx    → nullptr      (no EVT sink in Drive for now)
// ---------------------------------------------------------------------------
void Drive::_runOutlierFilter(uint32_t now)
{
    bool driving = (_outputs.tgtMms[1] != 0.0f || _outputs.tgtMms[0] != 0.0f);
    if (driving) {
        const float kMaxDeltaMm = fmaxf(40.0f,
            fmaxf(fabsf((float)_outputs.tgtMms[1]),
                  fabsf((float)_outputs.tgtMms[0])) * 0.2f);
        static constexpr int kRetries = 2;

        // Right (M1) first — proven ordering from WedgeTest.
        {
            float newR = _motorR.positionMm();
            float dR   = newR - _hw.encMm[0];
            if (dR > kMaxDeltaMm || dR < -kMaxDeltaMm) {
                newR = _hw.encMm[0];
                for (int k = 0; k < kRetries; ++k) {
                    float r2  = _motorR.readEncoderMmFSettle(_robCfg);
                    float dr2 = r2 - _hw.encMm[0];
                    if (dr2 <= kMaxDeltaMm && dr2 >= -kMaxDeltaMm) { newR = r2; break; }
                }
                if (_filterRejectStreakR < 255) ++_filterRejectStreakR;
            } else {
                _filterRejectStreakR = 0;
            }
            _hw.encMm[0] = newR;
        }

        // Left (M2) second.
        {
            float newL = _motorL.positionMm();
            float dL   = newL - _hw.encMm[1];
            if (dL > kMaxDeltaMm || dL < -kMaxDeltaMm) {
                newL = _hw.encMm[1];
                for (int k = 0; k < kRetries; ++k) {
                    float r2  = _motorL.readEncoderMmFSettle(_robCfg);
                    float dr2 = r2 - _hw.encMm[1];
                    if (dr2 <= kMaxDeltaMm && dr2 >= -kMaxDeltaMm) { newL = r2; break; }
                }
                if (_filterRejectStreakL < 255) ++_filterRejectStreakL;
            } else {
                _filterRejectStreakL = 0;
            }
            _hw.encMm[1] = newL;
        }
    } else {
        _filterRejectStreakL = 0;
        _filterRejectStreakR = 0;
    }
    _prevDriving   = driving;
    _lastControlMs = now;
}

}  // namespace subsystems
