// EKFTiny.cpp — thin wrapper over TinyEKF's ekf_t for the 5-state EKF.
//
// EKF_N and EKF_M must be defined before including tinyekf.h (and before
// including EKFTiny.h, which pulls in tinyekf.h). This TU is the canonical
// definition site; the header uses #ifndef guards so these take precedence.
//
// Sprint 050, Ticket 003.

#define EKF_N 5
#define EKF_M 2

#include "EKFTiny.h"
#include <math.h>
#include <string.h>

// ===========================================================================
// EKFTiny() — default constructor: zero all state and noise matrices.
// ===========================================================================

EKFTiny::EKFTiny()
{
    for (int i = 0; i < EKF_N * EKF_N; ++i)
        _ekf.P[i] = 0.0f;
    for (int i = 0; i < EKF_N; ++i)
        _ekf.x[i] = 0.0f;

    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j)
            _Q[i][j] = 0.0f;

    _rOtosXy = 0.0f;
    _rOtosV  = 0.0f;
    _rEncV   = 0.0f;
    _rejected = 0;
    _rejHead_streak = 0;
    _rejPos_streak  = 0;
}

// ===========================================================================
// init — set noise parameters; reset state and covariance.
// ===========================================================================

void EKFTiny::init(float q_xy, float q_theta, float q_v, float q_omega,
                   float r_otos_xy, float r_otos_v, float r_enc_v)
{
    // Zero _Q then set diagonal.
    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j)
            _Q[i][j] = 0.0f;

    _Q[0][0] = q_xy;
    _Q[1][1] = q_xy;
    _Q[2][2] = q_theta;
    _Q[3][3] = q_v;
    _Q[4][4] = q_omega;

    _rOtosXy = r_otos_xy;
    _rOtosV  = r_otos_v;
    _rEncV   = r_enc_v;
    _rejected = 0;
    _rejHead_streak = 0;
    _rejPos_streak  = 0;

    // Reset state and covariance.
    for (int i = 0; i < EKF_N; ++i)
        _ekf.x[i] = 0.0f;
    for (int i = 0; i < EKF_N * EKF_N; ++i)
        _ekf.P[i] = 0.0f;
}

// ===========================================================================
// setNoise — update noise parameters ONLY; leaves state, covariance, and the
// rejection-streak counters untouched. See EKFTiny.h for the boot-only vs.
// live-update contract distinguishing this from init(). Sprint 067, Ticket 003.
// ===========================================================================

void EKFTiny::setNoise(float q_xy, float q_theta, float q_v, float q_omega,
                       float r_otos_xy, float r_otos_v, float r_enc_v)
{
    // Zero _Q then set diagonal (mirrors init()'s _Q update exactly).
    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j)
            _Q[i][j] = 0.0f;

    _Q[0][0] = q_xy;
    _Q[1][1] = q_xy;
    _Q[2][2] = q_theta;
    _Q[3][3] = q_v;
    _Q[4][4] = q_omega;

    _rOtosXy = r_otos_xy;
    _rOtosV  = r_otos_v;
    _rEncV   = r_enc_v;

    // Deliberately NOT touched: _ekf.x[], _ekf.P[], _rejected,
    // _rejHead_streak, _rejPos_streak.
}

// ===========================================================================
// setPose — overwrite state with known pose; zero v and omega; set sane P-prior.
//
// Sane diagonal P-prior (mirrors EKF::setPose exactly):
//   P[0][0] = P[1][1] = kPriorXY    (100 mm^2)
//   P[2][2]           = kPriorTheta (~(5 deg)^2)
//   P[3][3]           = kPriorV     (100 (mm/s)^2)
//   P[4][4]           = kPriorOmega (0.01 (rad/s)^2)
//   Off-diagonal entries zero.
// ===========================================================================

void EKFTiny::setPose(float x, float y, float theta)
{
    _ekf.x[0] = x;
    _ekf.x[1] = y;
    _ekf.x[2] = theta;
    _ekf.x[3] = 0.0f;  // v
    _ekf.x[4] = 0.0f;  // omega

    // Zero all of P first, then set sane diagonal entries.
    for (int i = 0; i < EKF_N * EKF_N; ++i)
        _ekf.P[i] = 0.0f;

    _ekf.P[0 * 5 + 0] = kPriorXY;
    _ekf.P[1 * 5 + 1] = kPriorXY;
    _ekf.P[2 * 5 + 2] = kPriorTheta;
    _ekf.P[3 * 5 + 3] = kPriorV;
    _ekf.P[4 * 5 + 4] = kPriorOmega;
}

// ===========================================================================
// predict — arc-segment motion model (position block) + random-walk (velocity
// block). Uses ekf_predict() for the F*P*F^T+Q computation.
//
// Motion equations (position block):
//   theta_mid = theta_before + dTheta/2
//   x[0] += dCenter * cos(theta_mid)
//   x[1] += dCenter * sin(theta_mid)
//   x[2]  = wrapPi(x[2] + dTheta)
//
// Jacobian F is 5x5 identity except:
//   F[0][2] = a = -dCenter*sin(theta_mid)
//   F[1][2] = b =  dCenter*cos(theta_mid)
//
// Process noise is scaled by dt_s (same as EKF.cpp: Q_scaled = Q * dt_s).
// dt_s is clamped to [0, 0.5].
// ===========================================================================

void EKFTiny::predict(float dCenter, float dTheta, float theta_before, float dt_s)
{
    // Clamp dt_s (same as EKF.cpp).
    if (dt_s < 0.0f) dt_s = 0.0f;
    if (dt_s > 0.5f) dt_s = 0.5f;

    float theta_mid = theta_before + dTheta * 0.5f;
    float ct = cosf(theta_mid);
    float st = sinf(theta_mid);

    // Build fx (predicted state) — same as EKF.cpp position state update.
    float fx[EKF_N];
    fx[0] = _ekf.x[0] + dCenter * ct;
    fx[1] = _ekf.x[1] + dCenter * st;
    fx[2] = wrapPi(_ekf.x[2] + dTheta);
    fx[3] = _ekf.x[3];  // random-walk: unchanged
    fx[4] = _ekf.x[4];  // random-walk: unchanged

    // Jacobian non-identity entries for the position block.
    float a = -dCenter * st;   // F[0][2]
    float b =  dCenter * ct;   // F[1][2]

    // Build F (5x5, row-major flat array). Identity with F[0][2]=a, F[1][2]=b.
    float F[EKF_N * EKF_N];
    memset(F, 0, sizeof(F));
    // Identity diagonal.
    F[0 * 5 + 0] = 1.0f;
    F[1 * 5 + 1] = 1.0f;
    F[2 * 5 + 2] = 1.0f;
    F[3 * 5 + 3] = 1.0f;
    F[4 * 5 + 4] = 1.0f;
    // Non-identity entries.
    F[0 * 5 + 2] = a;
    F[1 * 5 + 2] = b;

    // Build Q_scaled = Q * dt_s (same as EKF.cpp N15 fix).
    float Q_scaled[EKF_N * EKF_N];
    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j)
            Q_scaled[i * 5 + j] = _Q[i][j] * dt_s;

    // Delegate F*P*F^T + Q to TinyEKF. ekf_predict also sets ekf->x = fx.
    ekf_predict(&_ekf, fx, F, Q_scaled);

    // EKF.cpp explicitly zeros cross-block P entries after the predict to
    // maintain the block-decoupling invariant (they should already be zero if
    // the invariant held, but zeroing is explicit for robustness). Mirror that.
    _ekf.P[0 * 5 + 3] = 0.0f; _ekf.P[0 * 5 + 4] = 0.0f;
    _ekf.P[1 * 5 + 3] = 0.0f; _ekf.P[1 * 5 + 4] = 0.0f;
    _ekf.P[2 * 5 + 3] = 0.0f; _ekf.P[2 * 5 + 4] = 0.0f;
    _ekf.P[3 * 5 + 0] = 0.0f; _ekf.P[3 * 5 + 1] = 0.0f; _ekf.P[3 * 5 + 2] = 0.0f;
    _ekf.P[4 * 5 + 0] = 0.0f; _ekf.P[4 * 5 + 1] = 0.0f; _ekf.P[4 * 5 + 2] = 0.0f;
    _ekf.P[3 * 5 + 4] = 0.0f;
    _ekf.P[4 * 5 + 3] = 0.0f;
}

// ===========================================================================
// updatePosition — 2D position-only Kalman update (OTOS x, y observation).
//
// Observation model: H is 2x5 with H[0][0]=1, H[1][1]=1, rest zero.
// S^-1 is computed analytically (det = s00*s11 - s01*s10) — same as EKF.cpp
// and the Python oracle — to guarantee numerical parity (avoids Cholesky path).
// After gating, the gain and update are applied manually (same math as EKF.cpp)
// rather than calling ekf_update, so that the analytic S^-1 is used end-to-end.
//
// D3 gate recovery: after 10 consecutive position rejections, inflate P[0][0]
// and P[1][1] to kRebaselineP and re-run the standard update. (See EKF.cpp for
// the derivation: R×10 cannot pass a 200mm gate at steady-state P.)
// ===========================================================================

void EKFTiny::updatePosition(float x_otos, float y_otos)
{
    // Innovation.
    float yi0 = x_otos - _ekf.x[0];
    float yi1 = y_otos - _ekf.x[1];

    // Innovation covariance S (2x2).
    // H selects rows 0 and 1: S[i][j] = P[i][j] + R*delta(i,j).
    float s00 = _ekf.P[0 * 5 + 0] + _rOtosXy;
    float s01 = _ekf.P[0 * 5 + 1];
    float s10 = _ekf.P[1 * 5 + 0];
    float s11 = _ekf.P[1 * 5 + 1] + _rOtosXy;

    // Analytic 2x2 inverse of S (matches Python oracle exactly).
    float det = s00 * s11 - s01 * s10;
    if (det > -1e-9f && det < 1e-9f) {
        return;  // singular — skip update
    }
    float inv_det = 1.0f / det;
    float si00 =  s11 * inv_det;
    float si01 = -s01 * inv_det;
    float si10 = -s10 * inv_det;
    float si11 =  s00 * inv_det;

    // Mahalanobis gating: d2 = yi^T * S_inv * yi; chi-square 2-DOF threshold = 5.99.
    float d2 = yi0 * (si00 * yi0 + si01 * yi1) + yi1 * (si10 * yi0 + si11 * yi1);
    bool accepted = (d2 <= 5.99f);

    if (!accepted) {
        ++_rejected;
        ++_rejPos_streak;
        // D3 gate recovery (sprint 024-005): after 10 consecutive position
        // rejections, inflate position block of P and re-run update. K≈1 path.
        if (_rejPos_streak >= 10) {
            _rejPos_streak = 0;
            static constexpr float kRebaselineP = 1.0e6f;  // mm² — K≈1
            _ekf.P[0 * 5 + 0] = kRebaselineP;
            _ekf.P[0 * 5 + 1] = 0.0f;
            _ekf.P[1 * 5 + 0] = 0.0f;
            _ekf.P[1 * 5 + 1] = kRebaselineP;
            // Zero position cross-covariances with heading and velocity blocks.
            _ekf.P[0 * 5 + 2] = 0.0f; _ekf.P[0 * 5 + 3] = 0.0f; _ekf.P[0 * 5 + 4] = 0.0f;
            _ekf.P[1 * 5 + 2] = 0.0f; _ekf.P[1 * 5 + 3] = 0.0f; _ekf.P[1 * 5 + 4] = 0.0f;
            _ekf.P[2 * 5 + 0] = 0.0f; _ekf.P[2 * 5 + 1] = 0.0f;
            _ekf.P[3 * 5 + 0] = 0.0f; _ekf.P[3 * 5 + 1] = 0.0f;
            _ekf.P[4 * 5 + 0] = 0.0f; _ekf.P[4 * 5 + 1] = 0.0f;
            // Recompute S and S_inv with inflated P.
            s00 = _ekf.P[0 * 5 + 0] + _rOtosXy;
            s01 = 0.0f;
            s10 = 0.0f;
            s11 = _ekf.P[1 * 5 + 1] + _rOtosXy;
            float detR = s00 * s11;   // s01=s10=0
            if (detR < 1e-9f) {
                return;  // degenerate after inflation — skip
            }
            inv_det = 1.0f / detR;
            si00 =  s11 * inv_det;
            si01 = 0.0f;
            si10 = 0.0f;
            si11 =  s00 * inv_det;
            accepted = true;  // K≈1 path: gate trivially satisfied
        }
        if (!accepted) {
            return;
        }
    } else {
        // Normal accept — reset streak.
        _rejPos_streak = 0;
    }

    // Kalman gain K = P*H^T * S_inv  (5x2).
    // P*H^T selects columns 0 and 1 of P (H is [I2x2 | 0]).
    float k00 = _ekf.P[0 * 5 + 0] * si00 + _ekf.P[0 * 5 + 1] * si10;
    float k01 = _ekf.P[0 * 5 + 0] * si01 + _ekf.P[0 * 5 + 1] * si11;
    float k10 = _ekf.P[1 * 5 + 0] * si00 + _ekf.P[1 * 5 + 1] * si10;
    float k11 = _ekf.P[1 * 5 + 0] * si01 + _ekf.P[1 * 5 + 1] * si11;
    float k20 = _ekf.P[2 * 5 + 0] * si00 + _ekf.P[2 * 5 + 1] * si10;
    float k21 = _ekf.P[2 * 5 + 0] * si01 + _ekf.P[2 * 5 + 1] * si11;
    float k30 = _ekf.P[3 * 5 + 0] * si00 + _ekf.P[3 * 5 + 1] * si10;
    float k31 = _ekf.P[3 * 5 + 0] * si01 + _ekf.P[3 * 5 + 1] * si11;
    float k40 = _ekf.P[4 * 5 + 0] * si00 + _ekf.P[4 * 5 + 1] * si10;
    float k41 = _ekf.P[4 * 5 + 0] * si01 + _ekf.P[4 * 5 + 1] * si11;

    // State update: x += K * yi.
    _ekf.x[0] += k00 * yi0 + k01 * yi1;
    _ekf.x[1] += k10 * yi0 + k11 * yi1;
    _ekf.x[2] += k20 * yi0 + k21 * yi1;
    _ekf.x[2]  = wrapPi(_ekf.x[2]);
    _ekf.x[3] += k30 * yi0 + k31 * yi1;
    _ekf.x[4] += k40 * yi0 + k41 * yi1;

    // Covariance update: P = (I - K*H) * P.
    // H[0] has 1 at col 0, H[1] has 1 at col 1, rest zero.
    // P_new[i][j] = P[i][j] - K[i][0]*P[0][j] - K[i][1]*P[1][j]
    float p00 = _ekf.P[0*5+0]; float p01 = _ekf.P[0*5+1]; float p02 = _ekf.P[0*5+2]; float p03 = _ekf.P[0*5+3]; float p04 = _ekf.P[0*5+4];
    float p10 = _ekf.P[1*5+0]; float p11 = _ekf.P[1*5+1]; float p12 = _ekf.P[1*5+2]; float p13 = _ekf.P[1*5+3]; float p14 = _ekf.P[1*5+4];
    float p20 = _ekf.P[2*5+0]; float p21 = _ekf.P[2*5+1]; float p22 = _ekf.P[2*5+2]; float p23 = _ekf.P[2*5+3]; float p24 = _ekf.P[2*5+4];
    float p30 = _ekf.P[3*5+0]; float p31 = _ekf.P[3*5+1]; float p32 = _ekf.P[3*5+2]; float p33 = _ekf.P[3*5+3]; float p34 = _ekf.P[3*5+4];
    float p40 = _ekf.P[4*5+0]; float p41 = _ekf.P[4*5+1]; float p42 = _ekf.P[4*5+2]; float p43 = _ekf.P[4*5+3]; float p44 = _ekf.P[4*5+4];

    _ekf.P[0*5+0] = p00 - k00*p00 - k01*p10;
    _ekf.P[0*5+1] = p01 - k00*p01 - k01*p11;
    _ekf.P[0*5+2] = p02 - k00*p02 - k01*p12;
    _ekf.P[0*5+3] = p03 - k00*p03 - k01*p13;
    _ekf.P[0*5+4] = p04 - k00*p04 - k01*p14;

    _ekf.P[1*5+0] = p10 - k10*p00 - k11*p10;
    _ekf.P[1*5+1] = p11 - k10*p01 - k11*p11;
    _ekf.P[1*5+2] = p12 - k10*p02 - k11*p12;
    _ekf.P[1*5+3] = p13 - k10*p03 - k11*p13;
    _ekf.P[1*5+4] = p14 - k10*p04 - k11*p14;

    _ekf.P[2*5+0] = p20 - k20*p00 - k21*p10;
    _ekf.P[2*5+1] = p21 - k20*p01 - k21*p11;
    _ekf.P[2*5+2] = p22 - k20*p02 - k21*p12;
    _ekf.P[2*5+3] = p23 - k20*p03 - k21*p13;
    _ekf.P[2*5+4] = p24 - k20*p04 - k21*p14;

    _ekf.P[3*5+0] = p30 - k30*p00 - k31*p10;
    _ekf.P[3*5+1] = p31 - k30*p01 - k31*p11;
    _ekf.P[3*5+2] = p32 - k30*p02 - k31*p12;
    _ekf.P[3*5+3] = p33 - k30*p03 - k31*p13;
    _ekf.P[3*5+4] = p34 - k30*p04 - k31*p14;

    _ekf.P[4*5+0] = p40 - k40*p00 - k41*p10;
    _ekf.P[4*5+1] = p41 - k40*p01 - k41*p11;
    _ekf.P[4*5+2] = p42 - k40*p02 - k41*p12;
    _ekf.P[4*5+3] = p43 - k40*p03 - k41*p13;
    _ekf.P[4*5+4] = p44 - k40*p04 - k41*p14;
}

// ===========================================================================
// updateVelocity — fuse v and omega as two sequential scalar 1-DOF updates.
//
// Applied manually (no ekf_update call) — same numerical path as EKF.cpp.
// Sequential: the omega update sees the post-v-update P (correct behaviour).
// ===========================================================================

void EKFTiny::updateVelocity(float v_meas, float omega_meas, float r_v, float r_omega)
{
    // --- Fuse linear velocity (state index 3) ---
    {
        float yv  = v_meas - _ekf.x[3];
        float s_v = _ekf.P[3 * 5 + 3] + r_v;

        if (s_v > 1e-12f && (yv * yv / s_v) <= 3.84f) {
            float kv0 = _ekf.P[0 * 5 + 3] / s_v;
            float kv1 = _ekf.P[1 * 5 + 3] / s_v;
            float kv2 = _ekf.P[2 * 5 + 3] / s_v;
            float kv3 = _ekf.P[3 * 5 + 3] / s_v;
            float kv4 = _ekf.P[4 * 5 + 3] / s_v;

            _ekf.x[0] += kv0 * yv;
            _ekf.x[1] += kv1 * yv;
            _ekf.x[2] += kv2 * yv;
            _ekf.x[2]  = wrapPi(_ekf.x[2]);
            _ekf.x[3] += kv3 * yv;
            _ekf.x[4] += kv4 * yv;

            // Covariance update: P[i][k] -= K[i] * P[3][k].
            float p3k0 = _ekf.P[3*5+0]; float p3k1 = _ekf.P[3*5+1]; float p3k2 = _ekf.P[3*5+2];
            float p3k3 = _ekf.P[3*5+3]; float p3k4 = _ekf.P[3*5+4];

            _ekf.P[0*5+0] -= kv0*p3k0; _ekf.P[0*5+1] -= kv0*p3k1; _ekf.P[0*5+2] -= kv0*p3k2; _ekf.P[0*5+3] -= kv0*p3k3; _ekf.P[0*5+4] -= kv0*p3k4;
            _ekf.P[1*5+0] -= kv1*p3k0; _ekf.P[1*5+1] -= kv1*p3k1; _ekf.P[1*5+2] -= kv1*p3k2; _ekf.P[1*5+3] -= kv1*p3k3; _ekf.P[1*5+4] -= kv1*p3k4;
            _ekf.P[2*5+0] -= kv2*p3k0; _ekf.P[2*5+1] -= kv2*p3k1; _ekf.P[2*5+2] -= kv2*p3k2; _ekf.P[2*5+3] -= kv2*p3k3; _ekf.P[2*5+4] -= kv2*p3k4;
            _ekf.P[3*5+0] -= kv3*p3k0; _ekf.P[3*5+1] -= kv3*p3k1; _ekf.P[3*5+2] -= kv3*p3k2; _ekf.P[3*5+3] -= kv3*p3k3; _ekf.P[3*5+4] -= kv3*p3k4;
            _ekf.P[4*5+0] -= kv4*p3k0; _ekf.P[4*5+1] -= kv4*p3k1; _ekf.P[4*5+2] -= kv4*p3k2; _ekf.P[4*5+3] -= kv4*p3k3; _ekf.P[4*5+4] -= kv4*p3k4;
        } else if (s_v <= 1e-12f) {
            // Degenerate: skip.
        } else {
            ++_rejected;
        }
    }

    // --- Fuse angular velocity (state index 4) ---
    {
        float yw  = omega_meas - _ekf.x[4];
        float s_w = _ekf.P[4 * 5 + 4] + r_omega;

        if (s_w > 1e-12f && (yw * yw / s_w) <= 3.84f) {
            float kw0 = _ekf.P[0 * 5 + 4] / s_w;
            float kw1 = _ekf.P[1 * 5 + 4] / s_w;
            float kw2 = _ekf.P[2 * 5 + 4] / s_w;
            float kw3 = _ekf.P[3 * 5 + 4] / s_w;
            float kw4 = _ekf.P[4 * 5 + 4] / s_w;

            _ekf.x[0] += kw0 * yw;
            _ekf.x[1] += kw1 * yw;
            _ekf.x[2] += kw2 * yw;
            _ekf.x[2]  = wrapPi(_ekf.x[2]);
            _ekf.x[3] += kw3 * yw;
            _ekf.x[4] += kw4 * yw;

            // Covariance update: P[i][k] -= K[i] * P[4][k].
            float p4k0 = _ekf.P[4*5+0]; float p4k1 = _ekf.P[4*5+1]; float p4k2 = _ekf.P[4*5+2];
            float p4k3 = _ekf.P[4*5+3]; float p4k4 = _ekf.P[4*5+4];

            _ekf.P[0*5+0] -= kw0*p4k0; _ekf.P[0*5+1] -= kw0*p4k1; _ekf.P[0*5+2] -= kw0*p4k2; _ekf.P[0*5+3] -= kw0*p4k3; _ekf.P[0*5+4] -= kw0*p4k4;
            _ekf.P[1*5+0] -= kw1*p4k0; _ekf.P[1*5+1] -= kw1*p4k1; _ekf.P[1*5+2] -= kw1*p4k2; _ekf.P[1*5+3] -= kw1*p4k3; _ekf.P[1*5+4] -= kw1*p4k4;
            _ekf.P[2*5+0] -= kw2*p4k0; _ekf.P[2*5+1] -= kw2*p4k1; _ekf.P[2*5+2] -= kw2*p4k2; _ekf.P[2*5+3] -= kw2*p4k3; _ekf.P[2*5+4] -= kw2*p4k4;
            _ekf.P[3*5+0] -= kw3*p4k0; _ekf.P[3*5+1] -= kw3*p4k1; _ekf.P[3*5+2] -= kw3*p4k2; _ekf.P[3*5+3] -= kw3*p4k3; _ekf.P[3*5+4] -= kw3*p4k4;
            _ekf.P[4*5+0] -= kw4*p4k0; _ekf.P[4*5+1] -= kw4*p4k1; _ekf.P[4*5+2] -= kw4*p4k2; _ekf.P[4*5+3] -= kw4*p4k3; _ekf.P[4*5+4] -= kw4*p4k4;
        } else if (s_w <= 1e-12f) {
            // Degenerate: skip.
        } else {
            ++_rejected;
        }
    }
}

// ===========================================================================
// updateHeading — fuse OTOS heading as a scalar (1-DOF) Kalman update.
//
// Applied manually (no ekf_update call) — same numerical path as EKF.cpp.
// D3 recovery: at 10 consecutive rejections, inflate P[2][2] to kRebaselinePTheta
// and zero cross-terms so K≈1 and heading snaps to measurement.
// ===========================================================================

void EKFTiny::updateHeading(float theta_meas, float r_theta)
{
    // Wrap-safe innovation.
    float y = wrapPi(theta_meas - _ekf.x[2]);
    float s = _ekf.P[2 * 5 + 2] + r_theta;

    if (s <= 1e-12f) {
        return;  // degenerate — skip silently
    }

    bool accepted = (y * y / s) <= 3.84f;

    if (!accepted) {
        ++_rejected;
        ++_rejHead_streak;
        if (_rejHead_streak >= 10) {
            _rejHead_streak = 0;
            static constexpr float kRebaselinePTheta = 1.0e5f;  // rad² — K≈1
            _ekf.P[2 * 5 + 2] = kRebaselinePTheta;
            _ekf.P[2 * 5 + 0] = 0.0f; _ekf.P[2 * 5 + 1] = 0.0f;
            _ekf.P[2 * 5 + 3] = 0.0f; _ekf.P[2 * 5 + 4] = 0.0f;
            _ekf.P[0 * 5 + 2] = 0.0f; _ekf.P[1 * 5 + 2] = 0.0f;
            _ekf.P[3 * 5 + 2] = 0.0f; _ekf.P[4 * 5 + 2] = 0.0f;
            s = _ekf.P[2 * 5 + 2] + r_theta;
            accepted = (s > 1e-12f);
        }
        if (!accepted) {
            return;
        }
    }

    // Accepted (normal or recovery path).
    _rejHead_streak = 0;

    float k0 = _ekf.P[0 * 5 + 2] / s;
    float k1 = _ekf.P[1 * 5 + 2] / s;
    float k2 = _ekf.P[2 * 5 + 2] / s;
    float k3 = _ekf.P[3 * 5 + 2] / s;
    float k4 = _ekf.P[4 * 5 + 2] / s;

    _ekf.x[0] += k0 * y;
    _ekf.x[1] += k1 * y;
    _ekf.x[2] += k2 * y;
    _ekf.x[2]  = wrapPi(_ekf.x[2]);
    _ekf.x[3] += k3 * y;
    _ekf.x[4] += k4 * y;

    // Covariance update: P[i][k] -= K[i] * P[2][k].
    float p2k0 = _ekf.P[2*5+0]; float p2k1 = _ekf.P[2*5+1]; float p2k2 = _ekf.P[2*5+2];
    float p2k3 = _ekf.P[2*5+3]; float p2k4 = _ekf.P[2*5+4];

    _ekf.P[0*5+0] -= k0*p2k0; _ekf.P[0*5+1] -= k0*p2k1; _ekf.P[0*5+2] -= k0*p2k2; _ekf.P[0*5+3] -= k0*p2k3; _ekf.P[0*5+4] -= k0*p2k4;
    _ekf.P[1*5+0] -= k1*p2k0; _ekf.P[1*5+1] -= k1*p2k1; _ekf.P[1*5+2] -= k1*p2k2; _ekf.P[1*5+3] -= k1*p2k3; _ekf.P[1*5+4] -= k1*p2k4;
    _ekf.P[2*5+0] -= k2*p2k0; _ekf.P[2*5+1] -= k2*p2k1; _ekf.P[2*5+2] -= k2*p2k2; _ekf.P[2*5+3] -= k2*p2k3; _ekf.P[2*5+4] -= k2*p2k4;
    _ekf.P[3*5+0] -= k3*p2k0; _ekf.P[3*5+1] -= k3*p2k1; _ekf.P[3*5+2] -= k3*p2k2; _ekf.P[3*5+3] -= k3*p2k3; _ekf.P[3*5+4] -= k3*p2k4;
    _ekf.P[4*5+0] -= k4*p2k0; _ekf.P[4*5+1] -= k4*p2k1; _ekf.P[4*5+2] -= k4*p2k2; _ekf.P[4*5+3] -= k4*p2k3; _ekf.P[4*5+4] -= k4*p2k4;
}

// ===========================================================================
// Accessors
// ===========================================================================

float    EKFTiny::x()             const { return _ekf.x[0]; }
float    EKFTiny::y()             const { return _ekf.x[1]; }
float    EKFTiny::theta()         const { return _ekf.x[2]; }
float    EKFTiny::v()             const { return _ekf.x[3]; }
float    EKFTiny::omega()         const { return _ekf.x[4]; }
uint32_t EKFTiny::rejectedCount() const { return _rejected; }
int      EKFTiny::getRejectCount() const { return (int)_rejected; }
int      EKFTiny::rejHeadStreak()  const { return _rejHead_streak; }
int      EKFTiny::rejPosStreak()   const { return _rejPos_streak; }

// ===========================================================================
// wrapPi — wrap angle to (-pi, pi] using atan2f identity.
// ===========================================================================

float EKFTiny::wrapPi(float theta)
{
    return atan2f(sinf(theta), cosf(theta));
}
