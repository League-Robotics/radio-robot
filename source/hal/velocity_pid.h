// velocity_pid.h — Hal::MotorVelocityPid: the shared closed-loop velocity
// control law, extracted byte-for-byte (sprint 081-001) out of what used to
// be NezhaMotor::runVelocityPid() (source/hal/nezha/nezha_motor.cpp) so a
// future simulated leaf (Hal::SimMotor, ticket 003) can run the IDENTICAL
// control law rather than a re-derived approximation — the design write-
// up's own highest-flagged correction ("the sim must run the real PID, not
// a re-derived approximation").
//
// Pure host-clean math: no MicroBit.h, no I2C, no CODAL dependency — only
// messages/common.h's Gains struct. compute() takes gains/minDuty per call
// rather than caching a second copy of MotorConfig inside this class, so
// NezhaMotor's own config_ stays the single source of truth for
// calibration (architecture-update.md; this ticket's Implementation Plan
// step 1).
//
// See velocity_pid.cpp's compute() for the full control-law derivation
// comment (VelocityController::update()'s reduced discrete-PI-with-back-
// calculation-anti-windup form, the iOld-ordering rationale, and the
// documented source_old ReInit() stale-D divergence note) — carried
// forward verbatim from the pre-extraction NezhaMotor::runVelocityPid().
#pragma once

#include "messages/common.h"

namespace Hal {

class MotorVelocityPid {
 public:
  // Runs one control-law update and returns the duty fraction [-1, 1] to
  // apply this tick. target/measured are signed velocities [mm/s]; dt is
  // the elapsed time since the last update, substituted with kNominalDt
  // when <= 0 (e.g. the very first tick, or a clock glitch). gains/minDuty
  // are supplied fresh every call — see the file header for why this class
  // caches no calibration state of its own.
  float compute(float target, float measured, float dt,
                const msg::Gains& gains, float minDuty);   // [mm/s] [mm/s] [s] [-1,1] [-1,1] -> duty [-1,1]

 private:
  // Nominal loop period used before the first real dt measurement exists
  // (ported from VelocityController.cpp's kNominalDt, via NezhaMotor's
  // former file-local constant of the same name).
  static constexpr float kNominalDt = 0.024f;   // [s]

  // Persistent integrator state — the only state this class owns.
  float integral_ = 0.0f;

  // Edge-detector for the integrator-freeze deadband (086-002 fix — see
  // compute()'s own comment at the reset site for the invariant this
  // preserves): true whenever the PREVIOUS call was already inside the
  // deadband, so compute() can tell a fresh entry (reset the integrator)
  // apart from continuing to sit in it (stay frozen, as before).
  bool wasInDeadband_ = false;
};

}  // namespace Hal
