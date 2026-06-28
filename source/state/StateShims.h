#pragma once
#include "types/Inputs.h"  // RobotStateContainer (actual/desired/outputs)

// ---------------------------------------------------------------------------
// StateShims — inline free functions mapping legacy field names to new paths.
//
// Sprint 047-001 Phase A: RobotStateContainer changed from
//   { MotorCommands commands; HardwareState inputs; TargetState target; }
// to
//   { ActualState actual; DesiredState desired; OutputState outputs; }
//
// These shims let call sites that can't be changed in Phase A still compile.
// All shims are inline free functions returning references — NOT reference
// members (which break = {} aggregate init; see Inputs.h design note).
//
// Consumer migration (Phase C) replaces shim calls with direct new-path access.
// ---------------------------------------------------------------------------

// ---- Actual / sensor (via state.actual) ----

inline float& poseX(RobotStateContainer& s)     { return s.actual.poseX; }
inline float& poseY(RobotStateContainer& s)     { return s.actual.poseY; }
inline float& poseHrad(RobotStateContainer& s)  { return s.actual.poseHrad; }

inline float& encLMm(RobotStateContainer& s)    { return s.actual.encLMm; }
inline float& encRMm(RobotStateContainer& s)    { return s.actual.encRMm; }

inline float& velLMms(RobotStateContainer& s)   { return s.actual.velLMms; }
inline float& velRMms(RobotStateContainer& s)   { return s.actual.velRMms; }

inline float& fusedV(RobotStateContainer& s)    { return s.actual.fusedV; }
inline float& fusedOmega(RobotStateContainer& s){ return s.actual.fusedOmega; }
inline float& fusedVy(RobotStateContainer& s)   { return s.actual.fusedVy; }

inline float& otosX(RobotStateContainer& s)     { return s.actual.otosX; }
inline float& otosY(RobotStateContainer& s)     { return s.actual.otosY; }
inline float& otosH(RobotStateContainer& s)     { return s.actual.otosH; }
inline float& otosAccelX(RobotStateContainer& s){ return s.actual.otosAccelX; }
inline float& otosAccelY(RobotStateContainer& s){ return s.actual.otosAccelY; }

// ---- Outputs / actuator (via state.outputs) ----

inline int16_t& pwmL(RobotStateContainer& s)    { return s.outputs.pwmL; }
inline int16_t& pwmR(RobotStateContainer& s)    { return s.outputs.pwmR; }
inline float&   tgtLMms(RobotStateContainer& s) { return s.outputs.tgtLMms; }
inline float&   tgtRMms(RobotStateContainer& s) { return s.outputs.tgtRMms; }

// ---- Desired / commanded (via state.desired) ----

inline DriveMode& mode(RobotStateContainer& s)              { return s.desired.mode; }
inline float& targetXWorld(RobotStateContainer& s)          { return s.desired.targetXWorld; }
inline float& targetYWorld(RobotStateContainer& s)          { return s.desired.targetYWorld; }
inline float& targetSpeedMms(RobotStateContainer& s)        { return s.desired.targetSpeedMms; }
inline float& distanceTargetMm(RobotStateContainer& s)      { return s.desired.distanceTargetMm; }
inline uint32_t& deadlineMs(RobotStateContainer& s)         { return s.desired.deadlineMs; }
inline ReplyFn& replyFn(RobotStateContainer& s)             { return s.desired.replyFn; }
inline void*& replyCtx(RobotStateContainer& s)              { return s.desired.replyCtx; }
inline MotionEventSink& sink(RobotStateContainer& s)        { return s.desired.sink; }
