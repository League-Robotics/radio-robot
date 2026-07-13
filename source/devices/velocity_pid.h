// velocity_pid.h — Devices::MotorVelocityPid: the shared closed-loop
// velocity control law.
//
// Ticket DB-004 (device-bus-tickets.md). Ported from
// source/hal/velocity_pid.{h,cpp} (Hal::MotorVelocityPid, itself extracted
// byte-for-byte in sprint 081-001 out of the pre-extraction
// NezhaMotor::runVelocityPid()) into the greenfield `source/devices/`
// subsystem (namespace `Devices`) per clasi/issues/device-bus-fiber-owned-
// self-contained-device-subsystem.md's "Shape" ("Proven policy code is
// *ported, not re-derived*"). The control law itself is carried forward
// byte-for-byte; only the surrounding types change:
//
//   - `msg::Gains` (messages/common.h) -> `Devices::Gains`
//     (source/devices/device_config.h) — the isolation invariant
//     (device-bus-tickets.md's "Standing isolation invariant") forbids
//     `#include "messages/..."` from source/devices/.
//   - the `minDuty` parameter -> `velDeadband` — DB-001 renamed
//     msg::MotorConfig's `min_duty` wire field to `Devices::MotorConfig::
//     velDeadband` in device_config.h because it gates the VELOCITY TARGET
//     magnitude for integrator-freeze, not a duty (see that field's own
//     comment, and the original hal/velocity_pid.cpp compute() comment this
//     port carries forward below). This DB-004 port finishes that rename by
//     retyping compute()'s own parameter to match, rather than propagating
//     the misleading `minDuty` name into new code (naming-and-style.md
//     rule 5).
//
// Pure host-clean math: no MicroBit.h, no I2C, no CODAL dependency — only
// devices/device_config.h's Gains struct. compute() takes gains/velDeadband
// per call rather than caching a second copy of MotorConfig inside this
// class, so NezhaMotor's own config_ stays the single source of truth for
// calibration (matches the original's own rationale).
//
// See velocity_pid.cpp's compute() for the full control-law derivation
// comment (carried forward verbatim from the pre-port file).
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

}  // namespace Devices
