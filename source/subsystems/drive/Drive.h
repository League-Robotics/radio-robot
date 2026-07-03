#pragma once
// =============================================================================
// Drive.h — subsystems::Drive
//
// Message-contract Drive subsystem: composes the existing control components
// by reference (MotorController, BodyVelocityController, PhysicalStateEstimate,
// Odometry, two IVelocityMotor, one Hardware&) and exposes the 4-verb contract
// plus two-phase tick (tickUpdate / tickAction) per SubsystemContract.h.
//
// De-scaffolded in ticket 060-006 (name stabilized).
//
// OTOS is resolved LIVE through Hardware every tick (074-002), not bound at
// construction: STEP 5 of tickUpdate() calls `_hal.otos()` fresh on each read
// instead of caching an `IOdometer&`, so a runtime `DBG OTOS BENCH` swap of
// the active odometer is observed on the very next tick. Same live-indirection
// idiom Robot::otosCorrect() already uses (Robot.cpp) for the identical reason.
//
// Constraints: C++11, no heap/STL/RTTI/exceptions, no virtual in the contract.
// =============================================================================

#include "messages/drivetrain.h"   // msg::DrivetrainCommand/State/Config/Capabilities
#include "messages/common.h"       // msg::CommandBatch
#include "subsystems/SubsystemContract.h"  // FluentBuilder<>
#include "hal/capability/IVelocityMotor.h" // IMotor alias
#include "hal/Hardware.h"          // Hardware& — live otos() indirection (074-002)
#include "types/Config.h"          // RobotConfig
#include "types/Inputs.h"          // HardwareState (= ActualState), MotorCommands

// Forward declarations — resolved at link time by including the .h in Drive.cpp.
class MotorController;
class BodyVelocityController;
class PhysicalStateEstimate;
class Odometry;

namespace subsystems {

// ---------------------------------------------------------------------------
// Drive — message-driven drivetrain subsystem.
//
// Construction order mirrors Robot.h: motorL/motorR → MotorController →
// BodyVelocityController → PhysicalStateEstimate → RobotConfig.
//
// Inherits FluentBuilder so call sites can use the fluent idiom:
//   drive.newCommand().msg().setTwist({200,0,0});
//   drive.apply(drive._pending_cmd);
// (Or the direct apply(cmd) path for wire-sourced commands.)
//
// No virtual dispatch in the control path. No heap allocation inside Drive.
// All component references are live for the lifetime of the owning object.
// ---------------------------------------------------------------------------
class Drive : public FluentBuilder<Drive,
                                   msg::DrivetrainCommand,
                                   msg::DrivetrainConfig>
{
public:
    // Constructor — holds references to all control components.
    // `hw` is the HardwareState that the Odometry/Estimator writes into; Drive
    // owns its own private HardwareState slice (`_hw`) for isolation from the
    // live Robot state.
    // `hal` (074-002): Drive resolves the ACTIVE odometer through `_hal.otos()`
    // fresh on every STEP-5 read, rather than caching a boot-time `IOdometer&`.
    // A plain reference cannot be re-seated, so a runtime `DBG OTOS BENCH` swap
    // of Hardware's active pointer would otherwise never reach the live
    // fusion/telemetry path (mirrors Robot::otosCorrect()'s existing live
    // `hal.otos()` indirection — see that function's header comment).
    Drive(IMotor& motorL, IMotor& motorR,
          MotorController& mc,
          BodyVelocityController& bvc,
          PhysicalStateEstimate& est,
          Odometry& odo,
          Hardware& hal,
          const RobotConfig& cfg);

    // ---- 4-verb contract (no virtual dispatch) ----

    // Stage the command (no hardware I/O, no emission).
    void apply(const msg::DrivetrainCommand& cmd);

    // SENSE phase: read encoders, run EKF predict; optionally OTOS correct.
    // Updates _state. Call before tickAction().
    // fuseOtos: when true, bypass the internal OTOS lag gate and run OTOS
    //   correction every tick (mirrors ts.fuseOtos in LoopTickOnce — used by
    //   sim_set_otos_fusion which forces sub-lag-period OTOS updates in tests).
    void tickUpdate(uint32_t now, bool fuseOtos = false);

    // ACT phase: apply staged command via kinematics → wheel PID → motor output.
    // Returns (currently empty) CommandBatch — Drive is a leaf actuator.
    msg::CommandBatch tickAction(uint32_t now);

    // Read-only state snapshot — no I/O, no copy.
    const msg::DrivetrainState& state() const { return _state; }

    // Read-only access to the internal actuator outputs (for sim tick ordering).
    // The MotorController writes pwm[] here each controlTick; the sim plant needs
    // these to advance the physics model.
    const MotorCommands& outputs() const { return _outputs; }

    // Store config; next tick picks it up.
    void configure(const msg::DrivetrainConfig& cfg);

    // Declared capability set.
    msg::DrivetrainCapabilities capabilities() const;

    // resetEncoders — atomically zero Drive's private encoder baseline and
    // re-anchor the Odometry snapshot so the next tickUpdate sees delta=0.
    // Called by Robot::resetEncoders() in the ordered-tick path so that D and
    // ZERO enc commands keep drive._hw in sync with the hardware reset.
    void resetEncoders();

    // ---- Sim injection hooks (060-004) ----
    // Used by sim_api.cpp to synchronise the Drive private state with direct
    // plant injections (e.g. sim_set_enc_l, sim_set_pose, sim_set_enc_omega_healthy).
    // These are compile-time thin — zero overhead in firmware (never called there).

    // Inject encoder position directly into _hw (mirrors sim_set_enc_l/r for Drive).
    // Does NOT reset the plant or MotorController — just aligns the private baseline.
    void injectEncL(float mm) { _hw.encPos[1] = mm; _state.enc_[1] = mm; }
    void injectEncR(float mm) { _hw.encPos[0] = mm; _state.enc_[0] = mm; }

    // Inject a fused pose directly into _hw (mirrors sim_set_pose for Drive).
    // Refreshes _state.fused too so the next state() read sees it immediately.
    void injectFusedPose(float x, float y, float h_rad);

    // Forward the encoder-omega health gate to drive's own estimator.
    void setEncOmegaHealthy(bool healthy);

private:
    // ---- Component references ----
    IMotor&                 _motorL;
    IMotor&                 _motorR;
    MotorController&        _mc;
    BodyVelocityController& _bvc;
    PhysicalStateEstimate&  _est;
    Odometry&               _odo;
    Hardware&               _hal;   // live otos() indirection (074-002)
    const RobotConfig&      _robCfg;

    // ---- Private state ----
    msg::DrivetrainConfig   _drvCfg  = {};   // live config slice (from configure())
    msg::DrivetrainState    _state   = {};   // owned state snapshot (tickUpdate writes)
    msg::DrivetrainCommand  _cmd     = {};   // staged command (apply → tickAction)
    bool                    _cmdPending = false;

    // Internal hardware-state slice owned by Drive (isolates from live Robot state).
    HardwareState           _hw      = {};
    MotorCommands           _outputs = {};

    // Outlier-filter streak counters (verbatim from legacy Drive — same initial values).
    uint32_t _lastControlMs       = 0;
    bool     _prevDriving         = false;
    bool     _prevAnyWedged       = false;
    uint8_t  _filterRejectStreakL = 0;
    uint8_t  _filterRejectStreakR = 0;

    // (064-004) Auto re-prime at idle: one-shot flag so a wedge latch that
    // persists while the drivetrain is at rest gets exactly one automatic
    // resetEncoderAccumulators() attempt per episode, not one every idle
    // tick. Clears when anyWedged next goes false (mirrors _prevAnyWedged).
    bool     _wedgeReprimeAttempted = false;

    // OTOS timing for the lag gate.
    uint32_t _lastOtosMs = 0;
    bool     _otosEverReady = false;

    // ---- OTOS WARNING-bit persistence gate (CR-06 — 065-006) ----
    // This is the LIVE ordered-tick OTOS-fusion path (STEP 5 of tickUpdate,
    // exercised by both real firmware and the sim via loopTickOnce ->
    // drive.tickUpdate); Robot::otosCorrect() carries the same gate for
    // documentation/API parity but has no caller post-060 cutover (dead
    // code -- the legacy loop was deleted in 060-005). Mirrors
    // Robot::otosCorrect()'s state machine and constants exactly: fuse
    // through <= kOtosWarnPersistK consecutive WARNING ticks (transient),
    // block once the streak persists, re-admit after kOtosCleanReadmitN
    // consecutive clean ticks. See architecture-update.md Step 4-5 item 6 /
    // Design Rationale Decision 5.
    uint8_t _otosWarnStreak    = 0;
    uint8_t _otosCleanStreak   = 0;
    bool    _otosFusionBlocked = false;
    static constexpr uint8_t kOtosWarnPersistK  = 3;
    static constexpr uint8_t kOtosCleanReadmitN = 5;

    // ---- OTOS health telemetry (074-004) ----
    // Raw OTOS STATUS byte from the most recent SUCCESSFUL readStatus() call
    // (STEP 5 below). Left UNCHANGED on a read failure -- same "preserve
    // last-known-good" convention as _hw.otos.valid and _prevOtosValid above.
    // Copied into _state.otos_status every tick (STEP 6) so buildTlmFrame can
    // emit it unconditionally as otos_health=<status>,<blocked>, independent
    // of otos='s own freshness gate -- see RobotTelemetry.cpp.
    uint8_t _lastOtosStatus = 0;

    // ---- OTOS pose-VALUE staleness check (074-003) ----
    // The STATUS-bit gate above catches a chip that self-reports degraded;
    // it has no signal for a chip that reports READABLE + STATUS-clean but
    // simply stops updating its pose register (the field symptom: frozen
    // otos= alongside a climbing ekf_rej, since a stuck-but-clean reading
    // sails through the STATUS gate and gets fused/rejected every tick).
    // _prevOtos{X,Y,H}/_prevOtosValid hold the previous SUCCESSFUL read only
    // (a read failure leaves them unchanged -- same "preserve last-known-
    // good" convention as _hw.otos.valid on the same STEP-5 branch) so the
    // comparison is always tick-to-tick, never against a stale failed read.
    // otosStuck (computed in Drive.cpp) ORs into the same warnBit passed to
    // _updateOtosFusionGate above -- this is an additional input to that
    // already-correct, already-tested state machine, not a second gate.
    float _prevOtosX = 0.0f;
    float _prevOtosY = 0.0f;
    float _prevOtosH = 0.0f;
    bool  _prevOtosValid = false;

    // Position/heading epsilon below which two consecutive OTOS reads are
    // considered "the same value" (i.e. not evidence of a live sensor).
    // Heading epsilon ~0.01 rad (~0.57 deg) and position epsilon 0.5 mm are
    // well below the real OTOS's typical tick-to-tick sensor noise floor, so
    // a healthy, live-updating sensor should never trip this by chance.
    // Per-wheel velocity threshold above which the robot is considered
    // "commanded to move" (encoder-evidenced motion) -- 5 mm/s is well above
    // encoder-idle jitter but well below any real driven speed. All three
    // are HIL-tunable starting points (architecture-update.md Open Question
    // 4), not yet bench-validated against real sensor noise.
    static constexpr float kOtosStuckPosEpsMm     = 0.5f;
    static constexpr float kOtosStuckHeadEpsRad   = 0.01f;
    static constexpr float kOtosStuckEncMotionMmps = 5.0f;

    // Update the WARNING-bit persistence gate for one tick's readStatus()
    // result. warnBit: true when the reading is degraded (a WARNING bit is
    // set), the status read itself failed, or the pose value is stuck while
    // the robot is commanded to move; false for a fully clean, live tick.
    void _updateOtosFusionGate(bool warnBit);

    // Per-wheel outlier-filter hold threshold (same as kFilterRejectStreakThreshold).
    static constexpr uint8_t kFilterRejectStreakThreshold = 3;

    // ---- Helpers ----
    void _runOutlierFilter(uint32_t now);
};

}  // namespace subsystems

// ---------------------------------------------------------------------------
// toDriveConfig — project RobotConfig → msg::DrivetrainConfig.
// Declared in the global namespace (matches the DriveConfig.cpp translation unit).
// Motion limits (aMax/vBodyMax/yawRateMax) are NOT mapped here (PlannerConfig scope).
// ---------------------------------------------------------------------------
msg::DrivetrainConfig toDriveConfig(const RobotConfig& rc);
