// velocity_pid.h — Devices::MotorVelocityPid: the shared closed-loop
// velocity control law.
//
// `velDeadband` (not `minDuty`) is the parameter name: it gates the
// VELOCITY TARGET magnitude for integrator-freeze, not a duty — see
// device_config.h's MotorConfig::velDeadband comment (naming-and-style.md
// rule 5: name the quantity, not the misleading wire-field name it mirrors).
//
// Pure host-clean math: no MicroBit.h, no I2C, no CODAL dependency — only
// devices/device_config.h's Gains struct. compute() takes gains/velDeadband
// per call rather than caching a second copy of MotorConfig inside this
// class, so NezhaMotor's own config_ stays the single source of truth for
// calibration.
//
// See velocity_pid.cpp's compute() for the full control-law derivation.
// Design/rationale: DESIGN.md.
#pragma once

#include "devices/device_config.h"

namespace Devices {

class MotorVelocityPid {
 public:
  // Runs one control-law update and returns the duty fraction [-1, 1] to
  // apply this tick. target/measured are signed velocities [mm/s]; dt is
  // the elapsed time since the last update, substituted with kNominalDt
  // when <= 0 (e.g. the very first tick, or a clock glitch). gains/
  // velDeadband are supplied fresh every call — see the file header for why
  // this class caches no calibration state of its own beyond the
  // integrator.
  float compute(float target, float measured, float dt, const Gains& gains,
                float velDeadband);   // [mm/s] [mm/s] [s] [-1,1] -> duty [-1,1]

  // True iff the most recent compute() call hit the exact-zero-target /
  // near-rest-measured exemption (c98be2e9's `target == 0.0f && fabsf(
  // measured) <= restThreshold` early-return, which hard-zeros the
  // returned duty). The caller (NezhaMotor::tick()) uses this to also snap
  // its own reported velocity() to 0.0f the same tick — see that call
  // site's own comment for why the estimator needs a second, explicit
  // reset rather than just letting its EMA/line-fit tail decay on its own.
  // Reflects ONLY the last compute() call; a caller that skips compute()
  // entirely (Duty-mode passthrough, PID disabled) must not consult this —
  // it is not re-derived independently of that call.
  bool restGateEngaged() const { return restGateEngaged_; }

 private:
  // Nominal loop period used before the first real dt measurement exists.
  static constexpr float kNominalDt = 0.024f;   // [s]

  // Persistent integrator state — the only state this class owns.
  float integral_ = 0.0f;

  // Edge-detector for the integrator-freeze deadband — see compute()'s own
  // comment at the reset site for the invariant this preserves: true
  // whenever the PREVIOUS call was already inside the deadband, so
  // compute() can tell a fresh entry (reset the integrator) apart from
  // continuing to sit in it (stay frozen, as before).
  bool wasInDeadband_ = false;

  // Set every compute() call — true iff THIS call's exact-zero-target/
  // near-rest exemption fired. See the public restGateEngaged() getter's
  // own comment.
  bool restGateEngaged_ = false;
};

}  // namespace Devices
