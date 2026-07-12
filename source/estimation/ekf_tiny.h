// ekf_tiny.h — EkfTiny: a minimal Extended Kalman Filter over a 3-state pose
// (x, y, heading), built as a thin wrapper over vendored TinyEKF
// (libraries/tinyekf/tinyekf.h).
//
// This is a SIMPLIFIED DERIVATIVE of the parked source_old/state/EKFTiny.*
// (sprint 050's 5-state wrapper: x, y, theta, v, omega). Sprint 082, ticket
// 001 trims that 515-line class down to the 3-state (x, y, heading) shape
// Subsystems::PoseEstimator (ticket 002) needs. Deliberately DROPPED
// (architecture-update.md Decision 2 — do not reintroduce without a fresh,
// acceptance-bar-driven reason):
//   - The velocity/omega state sub-block and its updateVelocity() channel.
//     `twist=` (TLM) is populated from directly-measured/derived rates
//     elsewhere (ticket 004), not filtered EKF state.
//   - setNoise() (live noise re-tune, independent of init()'s boot-only
//     reset) and the old class's Python-oracle test-harness accessors
//     (pEntry()/xEntry()/setXEntry()/setPEntry()/setRejPosStreak()/
//     setRejHeadStreak()) — not this sprint's scope to re-validate.
//
// Sprint 099, ticket 006 RE-ADDS a bounded innovation-consistency gate +
// rejection-streak P-inflation recovery on updatePosition()/updateHeading()
// (architecture-update.md sprint 099 Decision 4) — but as fully PRIVATE,
// internal-only state. The old class's PUBLIC test-harness accessors for
// this (rejHeadStreak()/rejPosStreak()/rejectedCount()/getRejectCount())
// stay dropped: PoseEstimator and the harness only ever need to observe
// accept/reject behavior externally (does x()/y()/theta() move or not?),
// never to read the streak counters directly — the gate is invisible to
// every caller, by design (ticket 006's AC).
//
// Namespace/location choice: EkfTiny is deliberately left UN-namespaced (not
// under Hal::, despite the ticket title's "Hal::EkfTiny" shorthand — this
// class does not own any device/leaf, so it does not belong under a
// device-owning namespace). This mirrors source/types/command_types.h's
// Commandable, which is likewise a pure, non-device-owning type living
// un-namespaced alongside device code. It is not a byte-for-byte copy of
// source/kinematics/body_kinematics.h's own convention (that file exposes a
// namespace, BodyKinematics, of stateless free functions with no per-call
// instance) because this class must own per-instance filter state (the
// ekf_t and the process-noise matrix) — but it follows the same underlying
// rule that pure-math code sibling to kinematics/ does not live under a
// Hal::/Subsystems:: device-owning namespace. Lives at
// source/estimation/ekf_tiny.{h,cpp} — a new top-level directory, sibling to
// source/kinematics/, per the Implementation Plan.
//
// Pure math: no MicroBit.h, no I2C, no CODAL dependency — only <math.h>,
// <stdint.h>, and vendored <tinyekf.h> (already on the include path; see
// root CMakeLists.txt's `include_directories(.../libraries/tinyekf)`).
//
// Kept from the old class, adapted to the 3-state shape (lowerCamelCase
// methods/params, no unit-suffixed identifiers — units live in `// [unit]`
// comment tags per .claude/rules/coding-standards.md):
//   - predict(dCenter, dTheta, thetaBefore, dt) — arc-segment motion model.
//   - updatePosition(xOtos, yOtos) — 2-observation (M=2) position channel,
//     analytic 2x2 S-inverse (same numerical approach as the old class; no
//     Cholesky ekf_update()/invert() needed for a 2x2).
//   - updateHeading(thetaOtos) — scalar (1-DOF) heading channel, wrap-safe
//     innovation, applied manually (no ekf_update() call).
//   - setPose(x, y, theta) — overwrite state with a known pose; sane
//     diagonal P-prior instead of zeroing P.
//   - Plain accessors: x(), y(), theta(), pDiag(idx) (idx in [0..2], not
//     [0..4]).
//   - init(qXy, qTheta, rOtosXy, rOtosTheta) — 4 noise parameters, not 8
//     (only the position/heading channels this class implements).
//
// This class's public surface is plain floats, never msg::Pose2D/BodyTwist3
// (source/messages/common.h) — there is no pose-shaped struct anywhere in
// the "kept" API above, so none is introduced here. It never uses the
// parallel, unit-suffixed Pose2D/BodyTwist3 family that used to live at
// source/kinematics/pose2d.h; that file was deleted pre-082 (commit
// f5fd7dde) and must not be recreated.
#pragma once

#include <math.h>
#include <stdint.h>

// EKF_N and EKF_M must be defined before including tinyekf.h. If a consuming
// TU already set them to different values this will fail at the #error
// check below. ekf_tiny.cpp defines them itself before including this
// header, which is the canonical path (mirrors source_old/state/EKFTiny.h's
// precedent, adapted from EKF_N=5 to EKF_N=3). Other TUs that only
// #include "estimation/ekf_tiny.h" (never calling any TinyEKF static
// function directly) are fine because tinyekf.h is header-only and all its
// functions are file-static.
#ifndef EKF_N
#define EKF_N 3
#endif
#ifndef EKF_M
#define EKF_M 2
#endif

#if EKF_N != 3
#error "ekf_tiny.h requires EKF_N == 3"
#endif
#if EKF_M != 2
#error "ekf_tiny.h requires EKF_M == 2"
#endif

#include <tinyekf.h>

// EkfTiny — 3-state (x, y, heading) EKF core. See the file header above for
// exactly what was dropped/kept relative to source_old/state/EKFTiny.*.
class EkfTiny {
 public:
  EkfTiny();

  // Initialize noise parameters and reset state/covariance to zero.
  //   qXy        — process noise variance for x and y       [mm^2]
  //   qTheta     — process noise variance for heading        [rad^2]
  //   rOtosXy    — OTOS measurement noise variance, x and y  [mm^2]
  //   rOtosTheta — OTOS measurement noise variance, heading  [rad^2]
  //
  // BOOT-ONLY: also resets state and covariance to zero. Not safe to call
  // mid-mission — doing so would teleport the fused pose back to the
  // origin. This trimmed class has no live setNoise() equivalent (see file
  // header); add one only with a fresh acceptance-bar-driven reason.
  void init(float qXy, float qTheta, float rOtosXy, float rOtosTheta);

  // Overwrite state with a known pose; sets a sane diagonal P-prior instead
  // of zeroing P.
  void setPose(float x, float y, float theta);

  // Predict step: arc-segment motion model. Uses ekf_predict() (TinyEKF) for
  // the F*P*F^T+Q covariance propagation.
  //   dCenter     — wheel-center arc length traveled this tick  [mm]
  //   dTheta      — heading change this tick                    [rad]
  //   thetaBefore — heading at the start of this tick (caller supplies the
  //                 pre-tick theta(), since predict() mutates state)
  //                                                              [rad]
  //   dt          — elapsed time this tick; clamped to [0, 0.5]  [s]
  void predict(float dCenter, float dTheta, float thetaBefore, float dt);

  // Update step: 2D position-only observation (e.g. OTOS x, y).
  // S^-1 is computed analytically (2x2) — no Cholesky ekf_update()/invert()
  // path.
  //
  // Sprint 099 D4 bounded innovation-consistency gate: the Mahalanobis
  // statistic d^2 = y^T S^-1 y is computed by REUSING this same analytic
  // S^-1 (never recomputed) and compared against kChiSquare2Dof99. If
  // d^2 exceeds it, the observation is REJECTED — the call returns without
  // modifying state — and the position channel's private rejection-streak
  // counter is incremented; on ACCEPT the streak resets to 0. Every
  // kRejectStreakThreshold-th consecutive rejection inflates this channel's
  // own P[0][0]/P[1][1] diagonal entries by a small, capped bump (never a
  // hard reset) so a genuinely-shifted sensor is eventually re-trusted
  // rather than locked out forever. This gate protects THIS method only —
  // it never applies to a delayed camera-fix update (ticket 008, D5), which
  // uses its own, separate, ungated update path.
  void updatePosition(float xOtos, float yOtos);   // [mm] [mm]

  // Update step: heading-only observation (e.g. OTOS heading), scalar
  // (1-DOF). Wrap-safe innovation: y = wrapPi(thetaOtos - theta()). Applied
  // manually (no ekf_update() call), same numerical path as the old class.
  //
  // Sprint 099 D4 bounded innovation-consistency gate: rejects when
  // |y| > kHeadingSigma * sqrt(S). Same reject/streak/P-inflation-recovery
  // shape as updatePosition() above, using the heading channel's own
  // private streak counter and P[2][2]. See updatePosition()'s doc comment
  // for the full behavior; this gate likewise protects THIS method only,
  // never the delayed camera-fix path.
  void updateHeading(float thetaOtos);   // [rad]

  // Accessors.
  float x() const;       // [mm]
  float y() const;       // [mm]
  float theta() const;   // [rad]

  // Return P[idx][idx] for idx in [0..2]; -1 if idx is out of range.
  float pDiag(int idx) const {
    return (idx >= 0 && idx < EKF_N) ? ekf_.P[idx * EKF_N + idx] : -1.0f;
  }

 private:
  ekf_t ekf_;
  float q_[EKF_N][EKF_N];   // process noise (diagonal)
  float rOtosXy_;           // [mm^2] OTOS position noise variance (x and y)
  float rOtosTheta_;        // [rad^2] OTOS heading noise variance

  // Wrap angle to (-pi, pi] using the atan2f identity — same as the old
  // class's EKFTiny::wrapPi / EKF::wrapPi.
  static float wrapPi(float theta);

  // Sane P-prior diagonal values used by setPose() — mirrors the old
  // class's kPriorXY/kPriorTheta exactly.
  static constexpr float kPriorXY = 100.0f;       // [mm^2]
  static constexpr float kPriorTheta = 0.00762f;  // [rad^2] (5 deg)^2

  // --- Bounded innovation-consistency gate (architecture-update.md sprint
  // 099 Decision 4) — protects updatePosition()/updateHeading() (the OTOS
  // channel) ONLY. Never applies to a delayed camera-fix update (ticket
  // 008, D5's separate ungated path). Starting values per D4, characterized
  // (not freehanded) in tests/sim/unit/ekf_tiny_harness.cpp — see that
  // file's gate scenarios for the derivation of every constant below and
  // ticket 006's completion notes for what evidence (or lack of it, at the
  // time this ticket ran) informed them. Not stakeholder-locked; retune
  // only with fresh sim/bench evidence, per D4's "characterize, don't
  // freehand" mandate.

  // 2-DOF chi-square critical value at p=0.99 — the position channel's
  // Mahalanobis d^2 = y^T S^-1 y gate. D4's own documented starting value
  // ("~9.21").
  static constexpr float kChiSquare2Dof99 = 9.21f;

  // Heading channel gate: reject when |y| > kHeadingSigma * sqrt(S). D4's
  // own documented starting value ("e.g. 3.0").
  static constexpr float kHeadingSigma = 3.0f;

  // Consecutive rejections (on a single channel) before that channel's own
  // P diagonal is inflated. D4's own documented starting value ("e.g. 10").
  // The streak counter keeps counting past this value if rejections
  // continue — the widening is applied again every further
  // kRejectStreakThreshold-th consecutive reject (10, 20, 30, ...) — and
  // only resets to 0 on the next accept, per the AC's "streak counter
  // resets once the gate re-opens" requirement.
  static constexpr int kRejectStreakThreshold = 10;

  // Position-channel P-inflation: additive bump to P[0][0]/P[1][1] applied
  // each time the streak crosses a kRejectStreakThreshold multiple, capped
  // so a permanently-disagreeing sensor cannot inflate P without bound.
  // Sized (see ekf_tiny_harness.cpp's recovery scenario) so a ~30mm
  // genuinely-shifted observation against a settled P is re-accepted after
  // two widenings (~20 ticks), not on the very first one — a gradual, not
  // abrupt, recovery.
  static constexpr float kPInflationBumpXY = 50.0f;     // [mm^2]
  static constexpr float kPInflationCapXY = 500.0f;     // [mm^2]

  // Heading-channel P-inflation: same shape as the position channel above,
  // applied to P[2][2] only.
  static constexpr float kPInflationBumpTheta = 0.001f;  // [rad^2]
  static constexpr float kPInflationCapTheta = 0.01f;    // [rad^2]

  int rejPosStreak_ = 0;    // consecutive updatePosition() rejections
  int rejHeadStreak_ = 0;   // consecutive updateHeading() rejections
};
