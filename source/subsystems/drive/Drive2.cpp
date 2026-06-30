// Drive2.cpp — subsystems::Drive2 implementation (ticket 057-004).
//
// Composes the existing control components by reference: MotorController,
// BodyVelocityController, PhysicalStateEstimate, Odometry, IMotor x2, IOdometer.
// Two-phase tick (tickUpdate/tickAction) mirrors the live loopTickOnce ordering:
//   tickUpdate  → SENSE (outlier filter → controlTick → EKF predict → OTOS correct)
//   tickAction  → ACT   (staged command → BVC.setTarget → BVC.advance → MC.setTarget)
//
// ADDITIVE — Drive::periodic() and the live loopTickOnce wiring are UNTOUCHED.
//
// C++11, no heap in Drive2, no virtual dispatch in the contract path.

#include "Drive2.h"

#include "MotorController.h"
#include "BodyVelocityController.h"
#include "state/PhysicalStateEstimate.h"
#include "control/Odometry.h"
#include "hal/capability/Pose2D.h"    // Pose2D / BodyTwist

#include <cmath>   // fmaxf, fabsf

namespace subsystems {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
Drive2::Drive2(IMotor& motorL, IMotor& motorR,
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
void Drive2::apply(const msg::DrivetrainCommand& cmd)
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
void Drive2::tickUpdate(uint32_t now)
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
    float trackwidth = (_drvCfg.get_trackwidth_mm() > 0.0f)
                           ? _drvCfg.get_trackwidth_mm()
                           : _robCfg.trackwidthMm;
    float rotSlip    = _robCfg.rotationalSlip;
    _est.addOdometryObservation(_hw, trackwidth, rotSlip, now);

    // ------------------------------------------------------------------
    // STEP 5: OTOS correction (lag-gated, matches LoopTickOnce pattern)
    // ------------------------------------------------------------------
    uint32_t lagMs = (_drvCfg.get_lag_otos_ms() > 0)
                         ? _drvCfg.get_lag_otos_ms()
                         : _robCfg.lagOtosMs;
    if (lagMs > 0 && _otos.is_initialized()) {
        if (!_otosEverReady) {
            // First time: mark ready and seed the timer so we don't fire
            // with an uninitialised _lastOtosMs.
            _otosEverReady = true;
            _lastOtosMs    = now;
        } else if ((int32_t)(now - _lastOtosMs) >= (int32_t)lagMs) {
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
        dst.pose.x_mm      = src.pose.x;
        dst.pose.y_mm      = src.pose.y;
        dst.pose.h_rad     = src.pose.h;
        dst.twist.vx_mmps  = src.twist.vx_mmps;
        dst.twist.vy_mmps  = src.twist.vy_mmps;
        dst.twist.omega_rads = src.twist.omega_rads;
        dst.stamp.lag_ms      = src.stamp.lagMs;
        dst.stamp.last_upd_ms = src.stamp.lastUpdMs;
        dst.stamp.valid       = src.stamp.valid;
    };

    copyPE(_hw.fused,   _state.fused);
    copyPE(_hw.encoder, _state.encoder);
    copyPE(_hw.optical, _state.optical);

    // Per-wheel diagnostics (differential: [0]=R, [1]=L).
    _state.enc_mm_[0]  = _hw.encMm[0];
    _state.enc_mm_[1]  = _hw.encMm[1];
    _state.enc_mm_count = 2;
    _state.vel_mms_[0]  = _hw.velMms[0];
    _state.vel_mms_[1]  = _hw.velMms[1];
    _state.vel_mms_count = 2;

    // Freshness envelopes: ::ValueSet → msg::ValueSet field-by-field.
    _state.enc.lag_ms      = _hw.enc.lagMs;
    _state.enc.last_upd_ms = _hw.enc.lastUpdMs;
    _state.enc.valid       = _hw.enc.valid;
    _state.otos.lag_ms      = _hw.otos.lagMs;
    _state.otos.last_upd_ms = _hw.otos.lastUpdMs;
    _state.otos.valid       = _hw.otos.valid;

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
// Returns an empty CommandBatch (Drive2 is a leaf actuator in this sprint).
// ---------------------------------------------------------------------------
msg::CommandBatch Drive2::tickAction(uint32_t now)
{
    (void)now;

    if (!_cmdPending) {
        return msg::CommandBatch{};
    }
    _cmdPending = false;

    switch (_cmd.control_kind) {

    case msg::DrivetrainCommand::ControlKind::TWIST: {
        float vx    = _cmd.control.twist.vx_mmps;
        float vy    = _cmd.control.twist.vy_mmps;
        float omega = _cmd.control.twist.omega_rads;

        // vy-reject on differential build.
        if (!capabilities().get_holonomic() && vy != 0.0f) {
            // Reject: set motors to zero.  Per contract: "no-op actuation".
            _mc.setTarget(0.0f, 0.0f);
            break;
        }

        // Use BVC profiler: setTarget then advance with a 1-tick dt.
        // dt_s: estimate from config controlPeriodMs; fall back to 20 ms.
        float dt_s = (_robCfg.controlPeriodMs > 0)
                         ? (float)_robCfg.controlPeriodMs / 1000.0f
                         : 0.020f;
        _bvc.setTarget(vx, omega);
        _bvc.advance(dt_s);
        break;
    }

    case msg::DrivetrainCommand::ControlKind::WHEELS: {
        // Per-wheel velocity targets.
        uint8_t count = _cmd.control.wheels.w_count;
        float tL = 0.0f;
        float tR = 0.0f;
        // Convention: differential — w_[0] = right, w_[1] = left.
        if (count > 0 && _cmd.control.wheels.w_[0].get_speed_mmps().has) {
            tR = _cmd.control.wheels.w_[0].get_speed_mmps().val;
        }
        if (count > 1 && _cmd.control.wheels.w_[1].get_speed_mmps().has) {
            tL = _cmd.control.wheels.w_[1].get_speed_mmps().val;
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
        int32_t x_mm  = (int32_t)_cmd.control.pose.x_mm;
        int32_t y_mm  = (int32_t)_cmd.control.pose.y_mm;
        int32_t h_cdeg = (int32_t)(_cmd.control.pose.h_rad * RAD_TO_CDEG);
        _est.resetPose(_hw, x_mm, y_mm, h_cdeg);

        // Refresh fused estimate into _state immediately (field-by-field copy:
        // ::PoseEstimate and msg::PoseEstimate are distinct C++ types).
        _state.fused.pose.x_mm       = _hw.fused.pose.x;
        _state.fused.pose.y_mm       = _hw.fused.pose.y;
        _state.fused.pose.h_rad      = _hw.fused.pose.h;
        _state.fused.twist.vx_mmps   = _hw.fused.twist.vx_mmps;
        _state.fused.twist.vy_mmps   = _hw.fused.twist.vy_mmps;
        _state.fused.twist.omega_rads = _hw.fused.twist.omega_rads;
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
// configure — store config; next tick reads updated gains/lag/etc.
// ---------------------------------------------------------------------------
void Drive2::configure(const msg::DrivetrainConfig& cfg)
{
    _drvCfg = cfg;

    // Update live velocity gains in MotorController if provided.
    // Only update if non-zero gains were supplied (zero = "use defaults").
    if (cfg.get_vel_gains().get_kp() != 0.0f || cfg.get_vel_gains().get_ki() != 0.0f) {
        // We reach into RobotConfig-typed updateVelGains via a local copy.
        // Drive2 uses _robCfg as the "base" and the msg::Gains as an override.
        // For simplicity in this sprint, just call updateVelGains on the original
        // config — the msg::DrivetrainConfig gains are applied when a REAL
        // toDriveConfig projection wires them in (ticket 005 / Phase 3).
        _mc.updateVelGains(_robCfg);
    }
}

// ---------------------------------------------------------------------------
// capabilities — declared truth for this build.
// ---------------------------------------------------------------------------
msg::DrivetrainCapabilities Drive2::capabilities() const
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
// Verbatim from Drive::periodic() with member renaming:
//   _commands → _outputs     (MotorCommands)
//   _inputs   → _hw          (HardwareState)
//   fn/ctx    → nullptr      (no EVT sink in Drive2 for now)
// ---------------------------------------------------------------------------
void Drive2::_runOutlierFilter(uint32_t now)
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
