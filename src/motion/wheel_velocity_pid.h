// wheel_velocity_pid.h — Motion::WheelVelocityPid: the shared closed-loop
// velocity control law.
//
// RELOCATED (125-003, sprint 125 Decision 2): this class used to be
// Devices::MotorVelocityPid, embedded one-per-channel inside
// Devices::NezhaMotor. The stakeholder's rate argument (issue REVISION,
// 2026-07-24) settled that encoder freshness (~80ms) bounds the PID's
// effective update rate at or below the loop rate regardless of WHERE it
// runs, so motor residency bought nothing — the control DECISION belongs in
// motion (tunable/testable in `motion_tests`, no hardware), while
// `NezhaMotor` narrows to protocol + bus hygiene + dwell/deadband + clamp
// (Decision 2's "protection vs. control" reframing of the older "armor
// lives in the base; PID stays leaf" rule). This move is otherwise
// VERBATIM — same control law, same integrator/anti-windup/rest-gate
// behavior, byte-identical `compute()` body (wheel_velocity_pid.cpp) — only
// the namespace and the Gains parameter type changed (see below).
//
// `Gains` is a NEW, motion-local plain aggregate (kp/ki/kff/iMax/kaw),
// mirroring `Devices::Gains` (device_config.h) field-for-field — NOT a
// reuse of that type, because `src/motion` may not `#include "devices/..."`
// (the base/motion isolation invariant: motion depends on nothing in
// src/firm except messages/ and firm/types/). Whoever supplies gains to
// this class (125-003: App::Drive's own interim closed loop; a later
// ticket: Motion::MoveQueue, once it owns two of these instances per
// Decision 1) constructs a Motion::Gains from whatever its own source of
// truth is.
//
// `velDeadband` (not `minDuty`) is the parameter name: it gates the
// VELOCITY TARGET magnitude for integrator-freeze, not a duty — see the
// coding-standards.md rule 5 this name follows (name the quantity, not the
// misleading wire-field name it mirrors).
//
// Pure host-clean math: no MicroBit.h, no I2C, no CODAL dependency, no
// src/firm dependency at all. compute() takes gains/velDeadband per call
// rather than caching a second copy of any config inside this class, so the
// caller's own config stays the single source of truth for calibration.
//
// See wheel_velocity_pid.cpp's compute() for the full control-law
// derivation. Design/rationale: DESIGN.md (src/firm/devices/, historical —
// this class's control-law history predates the move).
#pragma once

namespace Motion {

// Gains — a generic PI(+feedforward+anti-windup) gain set. Motion-local
// counterpart to the pre-125-003 Devices::Gains — see this file's own
// header for why it is not a reuse of that type. Dimensionless (gain
// coefficients carry no `// [unit]` tag).
struct Gains {
  float kp = 0.0f;
  float ki = 0.0f;
  float kff = 0.0f;
  float iMax = 0.0f;
  float kaw = 0.0f;
};

class WheelVelocityPid {
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
  // returned duty). The caller uses this to also snap its own reported
  // velocity to 0.0f the same tick, if it maintains its own estimator (the
  // pre-125-003 NezhaMotor call site's own rationale — see git history).
  // Reflects ONLY the last compute() call; a caller that skips compute()
  // entirely (raw-duty passthrough) must not consult this — it is not
  // re-derived independently of that call.
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

}  // namespace Motion
