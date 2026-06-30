#pragma once
// =============================================================================
// Drive2.h — subsystems::Drive2
//
// Message-contract Drive subsystem: composes the existing control components
// by reference (MotorController, BodyVelocityController, PhysicalStateEstimate,
// Odometry, two IVelocityMotor, one IOdometer) and exposes the 4-verb contract
// plus two-phase tick (tickUpdate / tickAction) per SubsystemContract.h.
//
// ADDITIVE — does NOT modify Drive::periodic() or the live loopTickOnce wiring.
// Phase 3 does the swap; Drive2 is a NEW class. (Ticket 057-004.)
//
// Constraints: C++11, no heap/STL/RTTI/exceptions, no virtual in the contract.
// =============================================================================

#include "messages/drivetrain.h"   // msg::DrivetrainCommand/State/Config/Capabilities
#include "messages/common.h"       // msg::CommandBatch
#include "subsystems/SubsystemContract.h"  // FluentBuilder<>
#include "hal/capability/IVelocityMotor.h" // IMotor alias
#include "hal/capability/IOdometer.h"
#include "types/Config.h"          // RobotConfig
#include "types/Inputs.h"          // HardwareState (= ActualState), MotorCommands

// Forward declarations — resolved at link time by including the .h in Drive2.cpp.
class MotorController;
class BodyVelocityController;
class PhysicalStateEstimate;
class Odometry;

namespace subsystems {

// ---------------------------------------------------------------------------
// Drive2 — message-driven drivetrain subsystem.
//
// Construction order mirrors Robot.h: motorL/motorR → MotorController →
// BodyVelocityController → PhysicalStateEstimate → RobotConfig.
//
// Inherits FluentBuilder so call sites can use the fluent idiom:
//   drive2.newCommand().msg().setTwist({200,0,0});
//   drive2.apply(drive2._pending_cmd);
// (Or the direct apply(cmd) path for wire-sourced commands.)
//
// No virtual dispatch in the control path. No heap allocation inside Drive2.
// All component references are live for the lifetime of the owning object.
// ---------------------------------------------------------------------------
class Drive2 : public FluentBuilder<Drive2,
                                    msg::DrivetrainCommand,
                                    msg::DrivetrainConfig>
{
public:
    // Constructor — holds references to all control components.
    // `hw` is the HardwareState that the Odometry/Estimator writes into; Drive2
    // owns its own private HardwareState slice (`_hw`) for isolation from the
    // live Robot state.
    Drive2(IMotor& motorL, IMotor& motorR,
           MotorController& mc,
           BodyVelocityController& bvc,
           PhysicalStateEstimate& est,
           Odometry& odo,
           IOdometer& otos,
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
    // Returns (currently empty) CommandBatch — Drive2 is a leaf actuator.
    msg::CommandBatch tickAction(uint32_t now);

    // Read-only state snapshot — no I/O, no copy.
    const msg::DrivetrainState& state() const { return _state; }

    // projectFromLegacy — copy encoder/pose/vel/twist/otos fields from a
    // legacy HardwareState snapshot into _state, without running any motor
    // control or EKF logic.  Used by the legacy loopTickOnce path so that
    // buildTlmFrame (which reads drive2.state()) sees live values during the
    // transitional period before USE_ORDERED_TICK is the default.
    // Deleted together with the legacy loop branch in ticket 060-005.
    void projectFromLegacy(const HardwareState& hw);

    // Read-only access to the internal actuator outputs (for sim tick ordering).
    // The MotorController writes pwm[] here each controlTick; the sim plant needs
    // these to advance the physics model.
    const MotorCommands& outputs() const { return _outputs; }

    // Store config; next tick picks it up.
    void configure(const msg::DrivetrainConfig& cfg);

    // Declared capability set.
    msg::DrivetrainCapabilities capabilities() const;

    // resetEncoders — atomically zero Drive2's private encoder baseline and
    // re-anchor the Odometry snapshot so the next tickUpdate sees delta=0.
    // Called by Robot::resetEncoders() in the ordered-tick path so that D and
    // ZERO enc commands keep drive2._hw in sync with the hardware reset.
    void resetEncoders();

    // ---- Sim injection hooks (060-004) ----
    // Used by sim_api.cpp to synchronise the Drive2 private state with direct
    // plant injections (e.g. sim_set_enc_l, sim_set_pose, sim_set_enc_omega_healthy).
    // These are compile-time thin — zero overhead in firmware (never called there).

    // Inject encoder position directly into _hw (mirrors sim_set_enc_l/r for Drive2).
    // Does NOT reset the plant or MotorController — just aligns the private baseline.
    void injectEncL(float mm) { _hw.encMm[1] = mm; _state.enc_[1] = mm; }
    void injectEncR(float mm) { _hw.encMm[0] = mm; _state.enc_[0] = mm; }

    // Inject a fused pose directly into _hw (mirrors sim_set_pose for Drive2).
    // Refreshes _state.fused too so the next state() read sees it immediately.
    void injectFusedPose(float x, float y, float h_rad);

    // Forward the encoder-omega health gate to drive2's own estimator.
    void setEncOmegaHealthy(bool healthy);

private:
    // ---- Component references ----
    IMotor&                 _motorL;
    IMotor&                 _motorR;
    MotorController&        _mc;
    BodyVelocityController& _bvc;
    PhysicalStateEstimate&  _est;
    Odometry&               _odo;
    IOdometer&              _otos;
    const RobotConfig&      _robCfg;

    // ---- Private state ----
    msg::DrivetrainConfig   _drvCfg  = {};   // live config slice (from configure())
    msg::DrivetrainState    _state   = {};   // owned state snapshot (tickUpdate writes)
    msg::DrivetrainCommand  _cmd     = {};   // staged command (apply → tickAction)
    bool                    _cmdPending = false;

    // Internal hardware-state slice owned by Drive2 (isolates from live Robot state).
    HardwareState           _hw      = {};
    MotorCommands           _outputs = {};

    // Outlier-filter streak counters (verbatim from Drive — same initial values).
    uint32_t _lastControlMs       = 0;
    bool     _prevDriving         = false;
    bool     _prevAnyWedged       = false;
    uint8_t  _filterRejectStreakL = 0;
    uint8_t  _filterRejectStreakR = 0;

    // OTOS timing for the lag gate.
    uint32_t _lastOtosMs = 0;
    bool     _otosEverReady = false;

    // Per-wheel outlier-filter hold threshold (same as Drive::kFilterRejectStreakThreshold).
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
