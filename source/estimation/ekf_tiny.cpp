// ekf_tiny.cpp — EkfTiny: 3-state (x, y, heading) EKF core. See
// ekf_tiny.h's file header for the full derivation note (what this class
// keeps/drops relative to source_old/state/EKFTiny.*) and the
// namespace/location rationale.
//
// EKF_N and EKF_M must be defined before including tinyekf.h (and before
// including ekf_tiny.h, which pulls in tinyekf.h). This TU is the canonical
// definition site; ekf_tiny.h's #ifndef guards defer to these.
//
// Sprint 082, Ticket 001.

#define EKF_N 3
#define EKF_M 2

#include "estimation/ekf_tiny.h"

#include <string.h>

// ===========================================================================
// EkfTiny() — default constructor: zero all state and noise matrices.
// ===========================================================================
EkfTiny::EkfTiny() {
  for (int i = 0; i < EKF_N * EKF_N; ++i) ekf_.P[i] = 0.0f;
  for (int i = 0; i < EKF_N; ++i) ekf_.x[i] = 0.0f;

  for (int i = 0; i < EKF_N; ++i)
    for (int j = 0; j < EKF_N; ++j) q_[i][j] = 0.0f;

  rOtosXy_ = 0.0f;
  rOtosTheta_ = 0.0f;
}

// ===========================================================================
// init — set noise parameters; reset state and covariance.
// ===========================================================================
void EkfTiny::init(float qXy, float qTheta, float rOtosXy, float rOtosTheta) {
  for (int i = 0; i < EKF_N; ++i)
    for (int j = 0; j < EKF_N; ++j) q_[i][j] = 0.0f;

  q_[0][0] = qXy;
  q_[1][1] = qXy;
  q_[2][2] = qTheta;

  rOtosXy_ = rOtosXy;
  rOtosTheta_ = rOtosTheta;

  for (int i = 0; i < EKF_N; ++i) ekf_.x[i] = 0.0f;
  for (int i = 0; i < EKF_N * EKF_N; ++i) ekf_.P[i] = 0.0f;

  // Boot-only reset also clears the gate's per-channel streak state (099
  // D4) -- a fresh init() means a fresh gate, same as a fresh filter.
  rejPosStreak_ = 0;
  rejHeadStreak_ = 0;
}

// ===========================================================================
// setPose — overwrite state with known pose; sane diagonal P-prior.
// ===========================================================================
void EkfTiny::setPose(float x, float y, float theta) {
  ekf_.x[0] = x;
  ekf_.x[1] = y;
  ekf_.x[2] = theta;

  for (int i = 0; i < EKF_N * EKF_N; ++i) ekf_.P[i] = 0.0f;
  ekf_.P[0 * EKF_N + 0] = kPriorXY;
  ekf_.P[1 * EKF_N + 1] = kPriorXY;
  ekf_.P[2 * EKF_N + 2] = kPriorTheta;
}

// ===========================================================================
// predict — arc-segment motion model. Uses ekf_predict() (TinyEKF) for the
// F*P*F^T+Q covariance propagation.
//
// Motion equations:
//   thetaMid = thetaBefore + dTheta/2
//   x[0] += dCenter * cos(thetaMid)
//   x[1] += dCenter * sin(thetaMid)
//   x[2]  = wrapPi(x[2] + dTheta)
//
// Jacobian f is EKF_N x EKF_N identity except:
//   f[0][2] = -dCenter * sin(thetaMid)
//   f[1][2] =  dCenter * cos(thetaMid)
//
// Process noise is scaled by dt (Q_scaled = Q * dt), mirroring the old
// class. dt is clamped to [0, 0.5].
// ===========================================================================
void EkfTiny::predict(float dCenter, float dTheta, float thetaBefore, float dt) {
  if (dt < 0.0f) dt = 0.0f;
  if (dt > 0.5f) dt = 0.5f;

  float thetaMid = thetaBefore + dTheta * 0.5f;
  float ct = cosf(thetaMid);
  float st = sinf(thetaMid);

  float fx[EKF_N];
  fx[0] = ekf_.x[0] + dCenter * ct;
  fx[1] = ekf_.x[1] + dCenter * st;
  fx[2] = wrapPi(ekf_.x[2] + dTheta);

  float a = -dCenter * st;   // f[0][2]
  float b = dCenter * ct;    // f[1][2]

  float f[EKF_N * EKF_N];
  memset(f, 0, sizeof(f));
  f[0 * EKF_N + 0] = 1.0f;
  f[1 * EKF_N + 1] = 1.0f;
  f[2 * EKF_N + 2] = 1.0f;
  f[0 * EKF_N + 2] = a;
  f[1 * EKF_N + 2] = b;

  float qScaled[EKF_N * EKF_N];
  for (int i = 0; i < EKF_N; ++i)
    for (int j = 0; j < EKF_N; ++j) qScaled[i * EKF_N + j] = q_[i][j] * dt;

  // Delegate F*P*F^T + Q to TinyEKF. ekf_predict() also sets ekf_.x = fx.
  ekf_predict(&ekf_, fx, f, qScaled);
}

// ===========================================================================
// computePositionGain / applyPositionGain — the shared position-channel
// Kalman-update core (099-008, D5). See ekf_tiny.h's doc comment on these
// two private helpers for why the math is split this way.
//
// Observation model: H is 2xEKF_N with H[0][0]=1, H[1][1]=1, rest zero.
// S^-1 is computed analytically (det = s00*s11 - s01*s10) — same as the old
// class — to avoid the Cholesky-based ekf_update()/invert() path for a 2x2.
// ===========================================================================
bool EkfTiny::computePositionGain(float xObs, float yObs, float r, PositionGain* out) const {
  // Innovation.
  float yi0 = xObs - ekf_.x[0];
  float yi1 = yObs - ekf_.x[1];

  // Innovation covariance S (2x2). H selects rows/cols 0 and 1.
  float s00 = ekf_.P[0 * EKF_N + 0] + r;
  float s01 = ekf_.P[0 * EKF_N + 1];
  float s10 = ekf_.P[1 * EKF_N + 0];
  float s11 = ekf_.P[1 * EKF_N + 1] + r;

  // Numerical safety only (NOT the Mahalanobis gate, which lives in the
  // gated caller): skip a genuinely singular S rather than divide by ~0.
  float det = s00 * s11 - s01 * s10;
  if (det > -1e-9f && det < 1e-9f) {
    return false;
  }
  float invDet = 1.0f / det;
  float si00 = s11 * invDet;
  float si01 = -s01 * invDet;
  float si10 = -s10 * invDet;
  float si11 = s00 * invDet;

  out->yi0 = yi0;
  out->yi1 = yi1;
  // Mahalanobis d^2 = y^T S^-1 y — computed here (reusing the analytic S^-1
  // above) unconditionally, whether or not the caller ends up gating on it;
  // updatePositionUngated() simply never reads it.
  out->d2 = yi0 * (si00 * yi0 + si01 * yi1) + yi1 * (si10 * yi0 + si11 * yi1);

  // Kalman gain K = P*H^T * S^-1 (EKF_N x 2). P*H^T selects columns 0/1 of P.
  out->k00 = ekf_.P[0 * EKF_N + 0] * si00 + ekf_.P[0 * EKF_N + 1] * si10;
  out->k01 = ekf_.P[0 * EKF_N + 0] * si01 + ekf_.P[0 * EKF_N + 1] * si11;
  out->k10 = ekf_.P[1 * EKF_N + 0] * si00 + ekf_.P[1 * EKF_N + 1] * si10;
  out->k11 = ekf_.P[1 * EKF_N + 0] * si01 + ekf_.P[1 * EKF_N + 1] * si11;
  out->k20 = ekf_.P[2 * EKF_N + 0] * si00 + ekf_.P[2 * EKF_N + 1] * si10;
  out->k21 = ekf_.P[2 * EKF_N + 0] * si01 + ekf_.P[2 * EKF_N + 1] * si11;
  return true;
}

void EkfTiny::applyPositionGain(const PositionGain& g) {
  // State update: x += K * yi.
  ekf_.x[0] += g.k00 * g.yi0 + g.k01 * g.yi1;
  ekf_.x[1] += g.k10 * g.yi0 + g.k11 * g.yi1;
  ekf_.x[2] += g.k20 * g.yi0 + g.k21 * g.yi1;
  ekf_.x[2] = wrapPi(ekf_.x[2]);

  // Covariance update: P = (I - K*H) * P.
  float p00 = ekf_.P[0 * EKF_N + 0];
  float p01 = ekf_.P[0 * EKF_N + 1];
  float p02 = ekf_.P[0 * EKF_N + 2];
  float p10 = ekf_.P[1 * EKF_N + 0];
  float p11 = ekf_.P[1 * EKF_N + 1];
  float p12 = ekf_.P[1 * EKF_N + 2];
  float p20 = ekf_.P[2 * EKF_N + 0];
  float p21 = ekf_.P[2 * EKF_N + 1];
  float p22 = ekf_.P[2 * EKF_N + 2];

  ekf_.P[0 * EKF_N + 0] = p00 - g.k00 * p00 - g.k01 * p10;
  ekf_.P[0 * EKF_N + 1] = p01 - g.k00 * p01 - g.k01 * p11;
  ekf_.P[0 * EKF_N + 2] = p02 - g.k00 * p02 - g.k01 * p12;

  ekf_.P[1 * EKF_N + 0] = p10 - g.k10 * p00 - g.k11 * p10;
  ekf_.P[1 * EKF_N + 1] = p11 - g.k10 * p01 - g.k11 * p11;
  ekf_.P[1 * EKF_N + 2] = p12 - g.k10 * p02 - g.k11 * p12;

  ekf_.P[2 * EKF_N + 0] = p20 - g.k20 * p00 - g.k21 * p10;
  ekf_.P[2 * EKF_N + 1] = p21 - g.k20 * p01 - g.k21 * p11;
  ekf_.P[2 * EKF_N + 2] = p22 - g.k20 * p02 - g.k21 * p12;
}

// ===========================================================================
// updatePosition — 2D position-only Kalman update (e.g. OTOS x, y). GATED
// (099 D4): computes the shared core's gain/statistic first, then rejects
// (no state mutation) before ever calling applyPositionGain() when the
// Mahalanobis statistic exceeds the documented starting 2-DOF chi-square
// critical value. See ekf_tiny.h's doc comment on this method for the full
// gate/streak/P-inflation-recovery behavior.
// ===========================================================================
void EkfTiny::updatePosition(float xOtos, float yOtos) {
  PositionGain g;
  if (!computePositionGain(xOtos, yOtos, rOtosXy_, &g)) {
    return;
  }

  // --- Innovation-consistency gate (D4): reject (state untouched) when the
  // Mahalanobis statistic exceeds the documented starting critical value. ---
  if (g.d2 > kChiSquare2Dof99) {
    ++rejPosStreak_;
    if (rejPosStreak_ % kRejectStreakThreshold == 0) {
      // Gradual, bounded widening — never a hard reset — so a genuinely-
      // shifted OTOS is eventually re-trusted rather than locked out
      // forever. Capped so a permanently-disagreeing sensor cannot inflate
      // P without bound.
      float p00 = ekf_.P[0 * EKF_N + 0] + kPInflationBumpXY;
      float p11 = ekf_.P[1 * EKF_N + 1] + kPInflationBumpXY;
      ekf_.P[0 * EKF_N + 0] = (p00 < kPInflationCapXY) ? p00 : kPInflationCapXY;
      ekf_.P[1 * EKF_N + 1] = (p11 < kPInflationCapXY) ? p11 : kPInflationCapXY;
    }
    return;
  }
  rejPosStreak_ = 0;
  applyPositionGain(g);
}

// ===========================================================================
// updatePositionUngated — 099-008, D5: the delayed camera-fix's position
// update. Routes through the IDENTICAL computePositionGain()/
// applyPositionGain() pair updatePosition() above uses -- no gate, no
// streak counter touched. Returns (no state change) only on the same
// numerically-singular-S safety guard computePositionGain() already
// applies for every caller.
// ===========================================================================
void EkfTiny::updatePositionUngated(float xFix, float yFix, float rFixXy) {
  PositionGain g;
  if (!computePositionGain(xFix, yFix, rFixXy, &g)) {
    return;
  }
  applyPositionGain(g);
}

// ===========================================================================
// computeHeadingGain / applyHeadingGain — the shared heading-channel
// Kalman-update core (099-008, D5). Same split rationale as
// computePositionGain()/applyPositionGain() above. Applied manually (no
// ekf_update() call) — same numerical path as the old class.
// ===========================================================================
bool EkfTiny::computeHeadingGain(float thetaObs, float r, HeadingGain* out) const {
  // Wrap-safe innovation.
  float y = wrapPi(thetaObs - ekf_.x[2]);
  float s = ekf_.P[2 * EKF_N + 2] + r;

  // Numerical safety only (NOT the sigma gate, which lives in the gated
  // caller): skip a degenerate S.
  if (s <= 1e-12f) {
    return false;
  }

  out->y = y;
  out->s = s;
  out->k0 = ekf_.P[0 * EKF_N + 2] / s;
  out->k1 = ekf_.P[1 * EKF_N + 2] / s;
  out->k2 = ekf_.P[2 * EKF_N + 2] / s;
  return true;
}

void EkfTiny::applyHeadingGain(const HeadingGain& g) {
  ekf_.x[0] += g.k0 * g.y;
  ekf_.x[1] += g.k1 * g.y;
  ekf_.x[2] += g.k2 * g.y;
  ekf_.x[2] = wrapPi(ekf_.x[2]);

  // Covariance update: P[i][k] -= K[i] * P[2][k].
  float p2k0 = ekf_.P[2 * EKF_N + 0];
  float p2k1 = ekf_.P[2 * EKF_N + 1];
  float p2k2 = ekf_.P[2 * EKF_N + 2];

  ekf_.P[0 * EKF_N + 0] -= g.k0 * p2k0;
  ekf_.P[0 * EKF_N + 1] -= g.k0 * p2k1;
  ekf_.P[0 * EKF_N + 2] -= g.k0 * p2k2;

  ekf_.P[1 * EKF_N + 0] -= g.k1 * p2k0;
  ekf_.P[1 * EKF_N + 1] -= g.k1 * p2k1;
  ekf_.P[1 * EKF_N + 2] -= g.k1 * p2k2;

  ekf_.P[2 * EKF_N + 0] -= g.k2 * p2k0;
  ekf_.P[2 * EKF_N + 1] -= g.k2 * p2k1;
  ekf_.P[2 * EKF_N + 2] -= g.k2 * p2k2;
}

// ===========================================================================
// updateHeading — fuse a heading observation (e.g. OTOS heading) as a scalar
// (1-DOF) Kalman update. GATED (099 D4): see ekf_tiny.h's doc comment on
// this method for the full gate/streak/P-inflation-recovery behavior.
// ===========================================================================
void EkfTiny::updateHeading(float thetaOtos) {
  HeadingGain g;
  if (!computeHeadingGain(thetaOtos, rOtosTheta_, &g)) {
    return;
  }

  // --- Innovation-consistency gate (D4): reject when |y| exceeds
  // kHeadingSigma standard deviations of S. ---
  if (fabsf(g.y) > kHeadingSigma * sqrtf(g.s)) {
    ++rejHeadStreak_;
    if (rejHeadStreak_ % kRejectStreakThreshold == 0) {
      // Gradual, bounded widening — never a hard reset (mirrors
      // updatePosition()'s recovery mechanism exactly).
      float p22 = ekf_.P[2 * EKF_N + 2] + kPInflationBumpTheta;
      ekf_.P[2 * EKF_N + 2] = (p22 < kPInflationCapTheta) ? p22 : kPInflationCapTheta;
    }
    return;
  }
  rejHeadStreak_ = 0;
  applyHeadingGain(g);
}

// ===========================================================================
// updateHeadingUngated — 099-008, D5: the delayed camera-fix's heading
// update. Routes through the IDENTICAL computeHeadingGain()/
// applyHeadingGain() pair updateHeading() above uses -- no gate, no streak
// counter touched.
// ===========================================================================
void EkfTiny::updateHeadingUngated(float thetaFix, float rFixTheta) {
  HeadingGain g;
  if (!computeHeadingGain(thetaFix, rFixTheta, &g)) {
    return;
  }
  applyHeadingGain(g);
}

// ===========================================================================
// Accessors
// ===========================================================================

float EkfTiny::x() const { return ekf_.x[0]; }
float EkfTiny::y() const { return ekf_.x[1]; }
float EkfTiny::theta() const { return ekf_.x[2]; }

// ===========================================================================
// wrapPi — wrap angle to (-pi, pi] using the atan2f identity.
// ===========================================================================
float EkfTiny::wrapPi(float theta) {
  return atan2f(sinf(theta), cosf(theta));
}
