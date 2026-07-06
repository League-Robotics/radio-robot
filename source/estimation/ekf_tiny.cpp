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
// updatePosition — 2D position-only Kalman update (e.g. OTOS x, y).
//
// Observation model: H is 2xEKF_N with H[0][0]=1, H[1][1]=1, rest zero.
// S^-1 is computed analytically (det = s00*s11 - s01*s10) — same as the old
// class — to avoid the Cholesky-based ekf_update()/invert() path for a 2x2.
// Gain and update are applied manually (no ekf_update() call) so the
// analytic S^-1 is used end-to-end. No gating/rejection/P-inflation logic —
// see ekf_tiny.h's file header for what was deliberately dropped.
// ===========================================================================
void EkfTiny::updatePosition(float xOtos, float yOtos) {
  // Innovation.
  float yi0 = xOtos - ekf_.x[0];
  float yi1 = yOtos - ekf_.x[1];

  // Innovation covariance S (2x2). H selects rows/cols 0 and 1.
  float s00 = ekf_.P[0 * EKF_N + 0] + rOtosXy_;
  float s01 = ekf_.P[0 * EKF_N + 1];
  float s10 = ekf_.P[1 * EKF_N + 0];
  float s11 = ekf_.P[1 * EKF_N + 1] + rOtosXy_;

  // Numerical safety only (NOT Mahalanobis gating): skip a genuinely
  // singular S rather than divide by ~0.
  float det = s00 * s11 - s01 * s10;
  if (det > -1e-9f && det < 1e-9f) {
    return;
  }
  float invDet = 1.0f / det;
  float si00 = s11 * invDet;
  float si01 = -s01 * invDet;
  float si10 = -s10 * invDet;
  float si11 = s00 * invDet;

  // Kalman gain K = P*H^T * S^-1 (EKF_N x 2). P*H^T selects columns 0/1 of P.
  float k00 = ekf_.P[0 * EKF_N + 0] * si00 + ekf_.P[0 * EKF_N + 1] * si10;
  float k01 = ekf_.P[0 * EKF_N + 0] * si01 + ekf_.P[0 * EKF_N + 1] * si11;
  float k10 = ekf_.P[1 * EKF_N + 0] * si00 + ekf_.P[1 * EKF_N + 1] * si10;
  float k11 = ekf_.P[1 * EKF_N + 0] * si01 + ekf_.P[1 * EKF_N + 1] * si11;
  float k20 = ekf_.P[2 * EKF_N + 0] * si00 + ekf_.P[2 * EKF_N + 1] * si10;
  float k21 = ekf_.P[2 * EKF_N + 0] * si01 + ekf_.P[2 * EKF_N + 1] * si11;

  // State update: x += K * yi.
  ekf_.x[0] += k00 * yi0 + k01 * yi1;
  ekf_.x[1] += k10 * yi0 + k11 * yi1;
  ekf_.x[2] += k20 * yi0 + k21 * yi1;
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

  ekf_.P[0 * EKF_N + 0] = p00 - k00 * p00 - k01 * p10;
  ekf_.P[0 * EKF_N + 1] = p01 - k00 * p01 - k01 * p11;
  ekf_.P[0 * EKF_N + 2] = p02 - k00 * p02 - k01 * p12;

  ekf_.P[1 * EKF_N + 0] = p10 - k10 * p00 - k11 * p10;
  ekf_.P[1 * EKF_N + 1] = p11 - k10 * p01 - k11 * p11;
  ekf_.P[1 * EKF_N + 2] = p12 - k10 * p02 - k11 * p12;

  ekf_.P[2 * EKF_N + 0] = p20 - k20 * p00 - k21 * p10;
  ekf_.P[2 * EKF_N + 1] = p21 - k20 * p01 - k21 * p11;
  ekf_.P[2 * EKF_N + 2] = p22 - k20 * p02 - k21 * p12;
}

// ===========================================================================
// updateHeading — fuse a heading observation (e.g. OTOS heading) as a scalar
// (1-DOF) Kalman update. Applied manually (no ekf_update() call) — same
// numerical path as the old class. No gating/rejection/P-inflation logic —
// see ekf_tiny.h's file header for what was deliberately dropped.
// ===========================================================================
void EkfTiny::updateHeading(float thetaOtos) {
  // Wrap-safe innovation.
  float y = wrapPi(thetaOtos - ekf_.x[2]);
  float s = ekf_.P[2 * EKF_N + 2] + rOtosTheta_;

  // Numerical safety only (NOT chi-square gating): skip a degenerate S.
  if (s <= 1e-12f) {
    return;
  }

  float k0 = ekf_.P[0 * EKF_N + 2] / s;
  float k1 = ekf_.P[1 * EKF_N + 2] / s;
  float k2 = ekf_.P[2 * EKF_N + 2] / s;

  ekf_.x[0] += k0 * y;
  ekf_.x[1] += k1 * y;
  ekf_.x[2] += k2 * y;
  ekf_.x[2] = wrapPi(ekf_.x[2]);

  // Covariance update: P[i][k] -= K[i] * P[2][k].
  float p2k0 = ekf_.P[2 * EKF_N + 0];
  float p2k1 = ekf_.P[2 * EKF_N + 1];
  float p2k2 = ekf_.P[2 * EKF_N + 2];

  ekf_.P[0 * EKF_N + 0] -= k0 * p2k0;
  ekf_.P[0 * EKF_N + 1] -= k0 * p2k1;
  ekf_.P[0 * EKF_N + 2] -= k0 * p2k2;

  ekf_.P[1 * EKF_N + 0] -= k1 * p2k0;
  ekf_.P[1 * EKF_N + 1] -= k1 * p2k1;
  ekf_.P[1 * EKF_N + 2] -= k1 * p2k2;

  ekf_.P[2 * EKF_N + 0] -= k2 * p2k0;
  ekf_.P[2 * EKF_N + 1] -= k2 * p2k1;
  ekf_.P[2 * EKF_N + 2] -= k2 * p2k2;
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
