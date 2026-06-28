#pragma once
#include <stdint.h>
#include "types/ValueSet.h"          // ValueSet (freshness envelope)
#include "types/Config.h"            // RobotConfig, DriveMode
#include "types/Protocol.h"          // ReplyFn
#include "control/MotionEventSink.h" // MotionEventSink

// ---------------------------------------------------------------------------
// Sprint 047-001: RobotStateContainer restructured from
//   { MotorCommands commands; HardwareState inputs; TargetState target; }
// to
//   { ActualState actual; DesiredState desired; OutputState outputs; }
//
// ActualState, DesiredState, and OutputState are defined in source/state/ and
// export backward-compat type aliases:
//   using HardwareState  = ActualState;
//   using MotorCommands  = OutputState;
//   using TargetState    = DesiredState;
//
// All existing function signatures that accept HardwareState& / MotorCommands& /
// TargetState& continue to compile and bind to state.actual / state.outputs /
// state.desired without any call-site changes.
//
// Reference-member approach was rejected: reference members cannot appear in
// structs using aggregate `= {}` zero-initialisation (C++ requires that every
// reference member be explicitly bound in a user-provided constructor, and
// `RobotStateContainer s{}` would not compile).
// ---------------------------------------------------------------------------
#include "state/ActualState.h"   // ActualState + using HardwareState = ActualState
#include "state/DesiredState.h"  // DesiredState + using TargetState  = DesiredState
#include "state/OutputState.h"   // OutputState  + using MotorCommands = OutputState

// ---------------------------------------------------------------------------
// RobotStateContainer — single authoritative state blob passed through the
// cooperative main loop (replaces per-subsystem private caches).
//
// Exactly three top-level fields (047-001):
//   actual  — all measured/estimated robot state (replaces `inputs`)
//   desired — all commanded/planned state (replaces `target`)
//   outputs — actuator PWM outputs (replaces `commands`)
// ---------------------------------------------------------------------------
struct RobotStateContainer {
    ActualState  actual;
    DesiredState desired;
    OutputState  outputs;
};

// ---------------------------------------------------------------------------
// defaultInputs — zero-initialise the container, then seed each ValueSet's
// lagMs from the corresponding RobotConfig lag field.
// ---------------------------------------------------------------------------
inline RobotStateContainer defaultInputs(const RobotConfig& cfg) {
    RobotStateContainer s{};
    s.actual.otos.lagMs    = cfg.lagOtosMs;
    s.actual.lineVS.lagMs  = cfg.lagLineMs;
    s.actual.colorVS.lagMs = cfg.lagColorMs;
    s.actual.portsVS.lagMs = cfg.lagPortsMs;
    // enc lag: encoder readings are synchronous in the control loop, so lagMs
    // is left at 0 (zero-initialised).
    return s;
}
