#pragma once
#include <stdint.h>
#include "io/capability/IVelocityMotor.h"
#include "MotorController.h"
#include "PhysicalStateEstimate.h"
#include "Inputs.h"          // HardwareState, MotorCommands (RobotStateContainer slices)
#include "Config.h"              // RobotConfig
#include "Protocol.h"            // ReplyFn

// ---------------------------------------------------------------------------
// Drive — thin subsystem wrapper for the per-wheel velocity control +
// encoder-filter concern (the CONTROL COLLECT block).
//
// Phase E (043-002): owns the CONTROL COLLECT block that previously lived inline
// in loopTickOnce — the speed-scaled outlier filter, the motorController
// controlTick() call, and the wedge push into PhysicalStateEstimate.  The body
// is moved VERBATIM — no numeric or ordering change.  golden-TLM is the
// byte-exact oracle.
//
// The five filter-streak members (_filterRejectStreakL/R, _prevDriving,
// _prevAnyWedged, _lastControlMs) move here from Robot as value members.  Their
// initial values and update order are identical to the former Robot fields.
//
// No virtual dispatch, no SubsystemBase: a standalone value-type class held by
// Robot.  Holds references into the device interfaces (motorL/motorR), the
// MotorController, PhysicalStateEstimate, HardwareState, MotorCommands, and
// config — all live for the lifetime of the owning Robot.
//
// No printf / telemetryEmit inside any method (Phase F logging-contract pre-cut).
// The one EVT enc_filter_hold emission uses the Robot TLM sink (fn/ctx), which is
// passed into periodic() as parameters (architecture-update.md OQ-2) rather than
// stored — Drive does not own the TLM channel.
//
// Annotation (architecture-update.md is source of truth where ticket wording
// conflicts): the Module Definition lists the deps as MotorController&,
// PhysicalStateEstimate&, HardwareState&, const RobotConfig&.  The verbatim block
// ALSO reaches r.motorR.positionMm() / r.motorL.readEncoderMmFSettle(cfg) on the
// IMotor& device refs, and r.state.commands (MotorCommands) for the drive gate.
// To preserve byte-exactness Drive therefore additionally holds IMotor& motorL,
// IMotor& motorR, and MotorCommands& — these are the exact lvalues the inline
// block used (r.motorL / r.motorR / r.state.commands).  No behavior change.
//
// Namespaced under `subsystems` for consistency with the sensor subsystems and
// to keep the global namespace clear; Robot exposes it as the member `drive`, so
// call sites read robot.drive.periodic(...).
// ---------------------------------------------------------------------------
namespace subsystems {

class Drive {
public:
    Drive(IMotor& motorL, IMotor& motorR, MotorController& mc,
          PhysicalStateEstimate& est, HardwareState& inputs,
          MotorCommands& commands, const RobotConfig& cfg)
        : _motorL(motorL), _motorR(motorR), _mc(mc), _est(est),
          _inputs(inputs), _commands(commands), _cfg(cfg) {}

    // updateInputs — writes encLMm/encRMm via the outlier filter.  The encoder
    // writes are inlined into periodic() (they live inside the per-wheel filter
    // blocks); this method is the conceptual seam documented for Phase F.  It is
    // not called separately today — periodic() performs the writes verbatim.
    void updateInputs();

    // periodic — VERBATIM CONTROL COLLECT block: outlier filter -> controlTick()
    // -> wedge push.  fn/ctx are the Robot TLM sink (_tlmBoundFn/_tlmBoundCtx),
    // threaded through so the EVT enc_filter_hold emission is byte-identical.
    void periodic(uint32_t now, ReplyFn fn, void* ctx);

    // Per-wheel outlier-filter hold threshold (was Robot::kFilterRejectStreakThreshold).
    // Same value (3); the EVT enc_filter_hold emits once per episode at onset.
    static constexpr uint8_t kFilterRejectStreakThreshold = 3;

private:
    IMotor&                _motorL;
    IMotor&                _motorR;
    MotorController&       _mc;
    PhysicalStateEstimate& _est;
    HardwareState&         _inputs;
    MotorCommands&         _commands;
    const RobotConfig&     _cfg;

    // Filter-streak state (moved VERBATIM from Robot — same initial values).
    uint32_t _lastControlMs       = 0;
    bool     _prevDriving         = false;
    bool     _prevAnyWedged       = false;
    uint8_t  _filterRejectStreakL = 0;
    uint8_t  _filterRejectStreakR = 0;
};

}  // namespace subsystems
