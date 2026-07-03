// Drive.cpp — subsystems::Drive implementation (de-scaffolded in 060-006).
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
             Hardware& hal,
             const RobotConfig& cfg)
    : _motorL(motorL)
    , _motorR(motorR)
    , _mc(mc)
    , _bvc(bvc)
    , _est(est)
    , _odo(odo)
    , _hal(hal)
    , _robCfg(cfg)
{
    // Seed the MotorController's commands reference so setTarget() writes
    // _outputs.tgtSpeed[], which controlTick() reads for the velocity PIDs.
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
// 1. Encoder outlier filter → write _hw.encPos[0/1].
// 2. controlTick()  → velocity PID for both wheels (reads _hw.encPos[]).
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
    bool driving = (_outputs.tgtSpeed[1] != 0.0f || _outputs.tgtSpeed[0] != 0.0f);
    _mc.controlTick(_hw, _outputs, now, driving ? 3 : 0);

    // ------------------------------------------------------------------
    // STEP 3: Wedge push (mirrors Drive::periodic — 033-005e)
    // ------------------------------------------------------------------
    bool anyWedged = _mc.wheelWedgedL() || _mc.wheelWedgedR();
    _est.setWedgeActive(anyWedged);
    if (anyWedged) {
        _est.setEncOmegaHealthy(false);
        // (064-004) Auto re-prime at idle: a wedge latch that persists while
        // the drivetrain is genuinely at rest is worth exactly one automatic
        // hardware re-prime attempt per episode -- this is the same at-rest
        // atomic reset that self-heals a transient latch elsewhere (next D
        // from idle, ZERO enc). A persistent latch needs a full power cycle
        // (see the KB doc), so hammering resetEncoderAccumulators() every
        // idle tick would not help and would just add needless I2C traffic
        // -- hence the one-shot gate. Reuses MotorController's own at-rest
        // decision (064-003, isAtRest()) instead of duplicating the
        // epsilon/commanded-vs-measured logic here.
        //
        // resetStuckCounters() is required alongside the reset: while idle,
        // Drive calls controlTick() with refreshedWheel=0 (see `driving`
        // below), so the wedge-check block in controlTick() never runs
        // again until a new command starts driving -- nothing would
        // otherwise observe the reset and clear the latch, so the one-shot
        // flag below would never re-arm for a future, separate episode.
        if (!_wedgeReprimeAttempted && _mc.isAtRest()) {
            _mc.resetEncoderAccumulators();
            _mc.resetStuckCounters();
            _wedgeReprimeAttempted = true;
        }
    } else {
        if (_prevAnyWedged) {
            _est.setEncOmegaHealthy(true);
        }
        // Re-arm for the next episode once the latch actually clears.
        _wedgeReprimeAttempted = false;
    }
    _prevAnyWedged = anyWedged;

    // ------------------------------------------------------------------
    // STEP 4: EKF predict — encoder dead-reckoning integrate
    // ------------------------------------------------------------------
    float trackwidth = _robCfg.trackwidth;
    float rotSlip    = _robCfg.rotationalSlip;
    _est.setKinematics(trackwidth, rotSlip);
    _est.addOdometryObservation(_hw.encPos[1], _hw.encPos[0], now,
                                _hw.encoder, _hw.fused);

    // ------------------------------------------------------------------
    // STEP 5: OTOS correction (lag-gated, matches LoopTickOnce pattern)
    //
    // Two independent warn sources feed the single _updateOtosFusionGate
    // state machine below: the CR-06 STATUS-bit check (065-006) and the
    // (074-003) pose-VALUE staleness check. Either one blocking fusion is
    // surfaced on the wire via otos_health=<status>,<blocked> (074-004).
    // ------------------------------------------------------------------
    uint32_t lagMs = _robCfg.lagOtos;
    if (lagMs > 0 && _hal.otos().is_initialized()) {
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
            bool poseOk = _hal.otos().readTransformed(p, headingRad);
            if (poseOk) {
                // CR-06 (065-006): WARNING-bit persistence gate. poseOk above
                // is the READABLE tier (I2C burst succeeded); readStatus()
                // adds the HEALTHY tier so a persistently degraded-but-
                // readable reading (lifted / on-stand / freshly-placed
                // robot) is not fused into the EKF, mirroring
                // Robot::otosCorrect()'s two-tier D9 gate. A failed status
                // read is treated the same as a WARNING tick (conservative —
                // do not count it toward re-admission).
                uint8_t otosStatus = 0;
                bool statusOk = _hal.otos().readStatus(otosStatus);

                // (074-003) Pose-VALUE staleness check. STATUS byte alone
                // cannot catch a reading that is READABLE, STATUS-clean, and
                // simply stops updating -- the field symptom of a frozen
                // otos= alongside a climbing ekf_rej (the stuck value keeps
                // getting fused, keeps disagreeing with the encoder
                // prediction, keeps getting rejected). encMotion uses the
                // per-wheel velocity already computed by controlTick() in
                // STEP 1+2 (above) as the "commanded to move" signal, so a
                // legitimately stationary robot with an unchanging reading
                // is never flagged. Comparison is against the PREVIOUS
                // tick's successfully-read pose (captured before it is
                // overwritten below), not the first-ever value.
                bool encMotion = (fabsf(_hw.vel[0]) > kOtosStuckEncMotionMmps) ||
                                 (fabsf(_hw.vel[1]) > kOtosStuckEncMotionMmps);
                bool otosStuck = _prevOtosValid && encMotion &&
                                 (fabsf(p.x - _prevOtosX) < kOtosStuckPosEpsMm) &&
                                 (fabsf(p.y - _prevOtosY) < kOtosStuckPosEpsMm) &&
                                 (fabsf(p.h - _prevOtosH) < kOtosStuckHeadEpsRad);
                _prevOtosX     = p.x;
                _prevOtosY     = p.y;
                _prevOtosH     = p.h;
                _prevOtosValid = true;

                _updateOtosFusionGate(!statusOk || (otosStatus != 0) || otosStuck);

                BodyTwist vel{};
                _hal.otos().readVelocityTransformed(vel, headingRad);
                if (!_otosFusionBlocked) {
                    _est.addOtosObservation(p.x, p.y, p.h,
                                            vel.v_mmps, vel.omega_rads,
                                            0.0f, now,
                                            _hw.optical, _hw.fused);
                }
                // 060-004: Mirror Robot::otosCorrect() — mark otos.valid so
                // buildTlmFrame's N8 freshness gate emits the otos= field.
                // Raw telemetry visibility is unaffected by the fusion gate
                // above (matches Robot::otosCorrect's contract).
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
    _state.enc_[0]  = _hw.encPos[0];
    _state.enc_[1]  = _hw.encPos[1];
    _state.enc_count = 2;
    _state.vel_[0]  = _hw.vel[0];
    _state.vel_[1]  = _hw.vel[1];
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
        // internal BVC (Planner::_bvc).  Running another BVC ramp in
        // Drive would double-profile the motion (planner ramps 0→target, then
        // Drive ramps 0→planner_output) causing indefinitely-slow ramp-up that
        // fails almost every motor/motion test.  Instead, do a direct inverse-
        // kinematics conversion and set wheel targets immediately.
        //
        // The MotorController's own velocity PID (controlTick) still closes the
        // wheel-speed loop; this sets the TARGET, not the PWM duty cycle.
        //
        // 067-004: this ternary used to read _drvCfg.get_trackwidth() first,
        // falling back to _robCfg.trackwidth only while _drvCfg's cached
        // value was <=0.0f -- the same shadow-cache disease Ticket 002 fixed
        // in tickUpdate()'s EKF-predict step. Robot::Robot() calls
        // drive.configure(toDriveConfig(config)) once at boot, which
        // populates _drvCfg.trackwidth with a positive snapshot; since `tw`
        // is not "drive"-annotated, _drvCfg is never refreshed for it again,
        // so this read was frozen at the boot-time trackwidth forever
        // regardless of any later `SET tw=<x>`. Read _robCfg.trackwidth
        // directly -- the same live source Drive's tickUpdate() now uses
        // (067-002) and the pattern every other plain-key consumer in this
        // file already follows.
        float vL = 0.0f, vR = 0.0f;
        float trackwidth = _robCfg.trackwidth;
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
        static constexpr float kAngleScale = 18000.0f / 3.14159265f;  // [cdeg/rad]
        int32_t pose_x  = (int32_t)_cmd.control.pose.x;
        int32_t pose_y  = (int32_t)_cmd.control.pose.y;
        int32_t pose_h  = (int32_t)(_cmd.control.pose.h * kAngleScale);  // [cdeg]
        _est.resetPose(_hw.encPos[1], _hw.encPos[0], pose_x, pose_y, pose_h,
                       _hw.encoder, _hw.fused);

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
//   1. Zero _hw.encPos[]: sets the outlier-filter baseline so the next
//      _runOutlierFilter() sees delta=0 on the fresh accumulator.
//   2. Refresh _state.enc_[] so state() returns 0 immediately this tick.
//   3. Re-baseline _odo's snapshot: prevents predict() computing dL=0-prev
//      (a large negative delta) on the first tick after reset.
// ---------------------------------------------------------------------------
void Drive::resetEncoders()
{
    for (int i = 0; i < kWheelCount; ++i) {
        _hw.encPos[i]    = 0.0f;
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
// _updateOtosFusionGate — CR-06 (065-006) WARNING-bit persistence gate.
//
// Mirrors Robot::otosCorrect()'s state machine exactly: a warn tick
// accumulates _otosWarnStreak and drops any partial re-admission progress; a
// clean tick resets the warn streak and, once blocked, counts toward
// re-admission. Called once per STEP 5 OTOS read (readable ticks only —
// unreadable ticks skip fusion entirely via the existing poseOk branch and
// do not touch this gate, same as Robot::otosCorrect's unreadable path).
// ---------------------------------------------------------------------------
void Drive::_updateOtosFusionGate(bool warnBit)
{
    if (warnBit) {
        if (_otosWarnStreak < 0xFF) ++_otosWarnStreak;
        _otosCleanStreak = 0;
        if (_otosWarnStreak > kOtosWarnPersistK) {
            _otosFusionBlocked = true;
        }
    } else {
        _otosWarnStreak = 0;
        if (_otosFusionBlocked) {
            if (++_otosCleanStreak >= kOtosCleanReadmitN) {
                _otosFusionBlocked = false;
                _otosCleanStreak   = 0;
            }
        }
    }
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

    // Push a live EKF noise update (does NOT reset fused pose/covariance —
    // see EKFTiny::setNoise()). Sourced from the live _robCfg, which already
    // reflects the just-committed SET, not the `cfg` parameter above (which
    // only carries the eleven msg::DrivetrainConfig-projected fields).
    // Fires whenever any "drive"-annotated key is SET; today that's only
    // ekfRHead (ekfROtosTheta), but the signature already accepts the seven
    // not-yet-registered EKF noise fields so a future sprint can expose them
    // via the registry with no further plumbing changes here.
    // Sprint 067, Ticket 003.
    _est.setNoise(_robCfg.ekfQxy, _robCfg.ekfQtheta,
                  _robCfg.ekfQv,   _robCfg.ekfQomega,
                  _robCfg.ekfROtosXy, _robCfg.ekfROtosV,
                  _robCfg.ekfREncV,   _robCfg.ekfROtosTheta);
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
    caps.onboard_position = _hal.otos().is_initialized();
    caps.wheel_count      = 2;
    return caps;
}

// ---------------------------------------------------------------------------
// _runOutlierFilter — private: speed-scaled outlier filter + encoder collect.
//
// Originally verbatim from legacy Drive::periodic() with member renaming:
//   _commands → _outputs     (MotorCommands)
//   _inputs   → _hw          (HardwareState)
//   fn/ctx    → nullptr      (no EVT sink in Drive for now)
//
// (064-006) Restores the reject-streak rebaseline that was lost in the
// sprint-060 cutover: kFilterRejectStreakThreshold consecutive rejected
// ticks now accept the already-computed fresh reading as the new baseline
// instead of holding a stale one forever (CR-02). Also refreshes _hw.encPos[]
// unconditionally while idle (architecture-update.md Design Rationale 5) so
// a hand-rolled wheel's baseline is absorbed before the next command starts,
// rather than relying solely on the in-drive streak escape hatch.
// ---------------------------------------------------------------------------
void Drive::_runOutlierFilter(uint32_t now)
{
    bool driving = (_outputs.tgtSpeed[1] != 0.0f || _outputs.tgtSpeed[0] != 0.0f);
    if (driving) {
        const float kMaxDelta = fmaxf(40.0f,   // [mm]
            fmaxf(fabsf((float)_outputs.tgtSpeed[1]),
                  fabsf((float)_outputs.tgtSpeed[0])) * 0.2f);
        static constexpr int kRetries = 2;

        // Right (M1) first — proven ordering from WedgeTest.
        {
            float freshR = _motorR.position();   // already read this tick, no extra I2C
            float newR   = freshR;
            float dR     = newR - _hw.encPos[0];
            if (dR > kMaxDelta || dR < -kMaxDelta) {
                newR = _hw.encPos[0];
                for (int k = 0; k < kRetries; ++k) {
                    float r2  = _motorR.readEncoderMmFSettle(_robCfg);
                    float dr2 = r2 - _hw.encPos[0];
                    if (dr2 <= kMaxDelta && dr2 >= -kMaxDelta) { newR = r2; break; }
                }
                if (_filterRejectStreakR < 255) ++_filterRejectStreakR;
                if (_filterRejectStreakR >= kFilterRejectStreakThreshold) {
                    // Persistent (3+ consecutive) rejection: the baseline is
                    // stale, not the reading. Rebaseline to the fresh read.
                    newR = freshR;
                    _filterRejectStreakR = 0;
                }
            } else {
                _filterRejectStreakR = 0;
            }
            _hw.encPos[0] = newR;
        }

        // Left (M2) second.
        {
            float freshL = _motorL.position();   // already read this tick, no extra I2C
            float newL   = freshL;
            float dL     = newL - _hw.encPos[1];
            if (dL > kMaxDelta || dL < -kMaxDelta) {
                newL = _hw.encPos[1];
                for (int k = 0; k < kRetries; ++k) {
                    float r2  = _motorL.readEncoderMmFSettle(_robCfg);
                    float dr2 = r2 - _hw.encPos[1];
                    if (dr2 <= kMaxDelta && dr2 >= -kMaxDelta) { newL = r2; break; }
                }
                if (_filterRejectStreakL < 255) ++_filterRejectStreakL;
                if (_filterRejectStreakL >= kFilterRejectStreakThreshold) {
                    // Persistent (3+ consecutive) rejection: the baseline is
                    // stale, not the reading. Rebaseline to the fresh read.
                    newL = freshL;
                    _filterRejectStreakL = 0;
                }
            } else {
                _filterRejectStreakL = 0;
            }
            _hw.encPos[1] = newL;
        }
    } else {
        // Idle: refresh the baseline unconditionally, no outlier gate.
        // PWM is 0 here so no PID/EKF stability is at stake, and this
        // absorbs a hand-rolled wheel's new position before the next
        // command starts (architecture-update.md Design Rationale 5).
        _hw.encPos[0] = _motorR.position();
        _hw.encPos[1] = _motorL.position();
        _filterRejectStreakL = 0;
        _filterRejectStreakR = 0;
    }
    _prevDriving   = driving;
    _lastControlMs = now;
}

}  // namespace subsystems
