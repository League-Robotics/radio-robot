#include "EKF.h"
#include <math.h>

// ===========================================================================
// EKF — 3-state Extended Kalman Filter for pose fusion
// ===========================================================================

// ---------------------------------------------------------------------------
// EKF() — default constructor: zero all state and noise matrices.
// ---------------------------------------------------------------------------

EKF::EKF()
{
    _x[0] = 0.0f; 
    _x[1] = 0.0f; 
    _x[2] = 0.0f;

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            _P[i][j] = _Q[i][j] = 0.0f;

    _r = 0.0f;
}

// ---------------------------------------------------------------------------
// init — set noise parameters; reset state and covariance.
// ---------------------------------------------------------------------------

void EKF::init(float q_xy, float q_theta, float r_otos_xy)
{
    // Zero _Q then set diagonal.
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            _Q[i][j] = 0.0f;

    _Q[0][0] = q_xy;
    _Q[1][1] = q_xy;
    _Q[2][2] = q_theta;

    _r = r_otos_xy;

    // Reset state and covariance.
    _x[0] = 0.0f; _x[1] = 0.0f; _x[2] = 0.0f;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            _P[i][j] = 0.0f;
}

// ---------------------------------------------------------------------------
// setPose — overwrite state with known pose; reset covariance to zero.
// ---------------------------------------------------------------------------

void EKF::setPose(float x, float y, float theta)
{
    _x[0] = x;
    _x[1] = y;
    _x[2] = theta;

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            _P[i][j] = 0.0f;
}

// ---------------------------------------------------------------------------
// predict — arc-segment motion model.
//
// Motion equations:
//   theta_mid = theta_before + dTheta/2
//   dx = dCenter * cosf(theta_mid)
//   dy = dCenter * sinf(theta_mid)
//   _x[0] += dx
//   _x[1] += dy
//   _x[2]  = wrapPi(_x[2] + dTheta)
//
// Jacobian F is 3x3 identity with:
//   F[0][2] = a = -dCenter*sinf(theta_mid)
//   F[1][2] = b =  dCenter*cosf(theta_mid)
//
// Covariance update: P = F*P*F^T + Q  (fully unrolled).
// ---------------------------------------------------------------------------

void EKF::predict(float dCenter, float dTheta, float theta_before)
{
    float theta_mid = theta_before + dTheta * 0.5f;
    float ct = cosf(theta_mid);
    float st = sinf(theta_mid);

    float dx = dCenter * ct;
    float dy = dCenter * st;

    _x[0] += dx;
    _x[1] += dy;
    _x[2]  = wrapPi(_x[2] + dTheta);

    // Jacobian non-identity entries.
    float a = -dCenter * st;   // F[0][2]
    float b =  dCenter * ct;   // F[1][2]

    // Compute new P = F*P*F^T + Q, fully unrolled.
    //
    // Derivation: T = F*P (F is identity with F[0][2]=a, F[1][2]=b):
    //   T[0][j] = P[0][j] + a*P[2][j]
    //   T[1][j] = P[1][j] + b*P[2][j]
    //   T[2][j] = P[2][j]
    //
    // Result = T*F^T (F^T is identity with FT[2][0]=a, FT[2][1]=b):
    //   Result[i][0] = T[i][0] + T[i][2]*a
    //   Result[i][1] = T[i][1] + T[i][2]*b
    //   Result[i][2] = T[i][2]

    float p00 = _P[0][0]; float p01 = _P[0][1]; float p02 = _P[0][2];
    float p10 = _P[1][0]; float p11 = _P[1][1]; float p12 = _P[1][2];
    float p20 = _P[2][0]; float p21 = _P[2][1]; float p22 = _P[2][2];

    // T rows
    float t00 = p00 + a*p20;  float t01 = p01 + a*p21;  float t02 = p02 + a*p22;
    float t10 = p10 + b*p20;  float t11 = p11 + b*p21;  float t12 = p12 + b*p22;
    float t20 = p20;          float t21 = p21;           float t22 = p22;

    // New P = T*F^T + Q
    _P[0][0] = t00 + t02*a + _Q[0][0];
    _P[0][1] = t01 + t02*b;
    _P[0][2] = t02;
    _P[1][0] = t10 + t12*a;
    _P[1][1] = t11 + t12*b + _Q[1][1];
    _P[1][2] = t12;
    _P[2][0] = t20 + t22*a;
    _P[2][1] = t21 + t22*b;
    _P[2][2] = t22 + _Q[2][2];
}

// ---------------------------------------------------------------------------
// update — 2D position-only Kalman update (OTOS x, y observation).
//
// Observation model: H = [[1,0,0],[0,1,0]]
// Innovation:        y_inn[i] = otos_meas[i] - _x[i]
// Innovation cov:    S = H*P*H^T + R  (2x2)
//                    S[0][0] = P[0][0] + _r,  S[0][1] = P[0][1]
//                    S[1][0] = P[1][0],        S[1][1] = P[1][1] + _r
// Kalman gain:       K = P*H^T * S_inv  (3x2)
// State update:      _x += K * y_inn
// Covariance update: P = (I - K*H) * P
// ---------------------------------------------------------------------------

void EKF::update(float x_otos, float y_otos)
{
    // Innovation
    float yi0 = x_otos - _x[0];
    float yi1 = y_otos - _x[1];

    // Innovation covariance S (2x2)
    float s00 = _P[0][0] + _r;
    float s01 = _P[0][1];
    float s10 = _P[1][0];
    float s11 = _P[1][1] + _r;

    // Analytic 2x2 inverse
    float det = s00 * s11 - s01 * s10;
    if (det > -1e-9f && det < 1e-9f) {
        return;  // singular — skip update
    }
    float inv_det = 1.0f / det;
    float si00 =  s11 * inv_det;
    float si01 = -s01 * inv_det;
    float si10 = -s10 * inv_det;
    float si11 =  s00 * inv_det;

    // Kalman gain K = P*H^T * S_inv  (3x2)
    // P*H^T selects columns 0 and 1 of P (since H rows pick x and y):
    //   (P*H^T)[i][0] = P[i][0]
    //   (P*H^T)[i][1] = P[i][1]
    // K[i][j] = P[i][0]*si[0][j] + P[i][1]*si[1][j]
    float k00 = _P[0][0]*si00 + _P[0][1]*si10;
    float k01 = _P[0][0]*si01 + _P[0][1]*si11;
    float k10 = _P[1][0]*si00 + _P[1][1]*si10;
    float k11 = _P[1][0]*si01 + _P[1][1]*si11;
    float k20 = _P[2][0]*si00 + _P[2][1]*si10;
    float k21 = _P[2][0]*si01 + _P[2][1]*si11;

    // State update: _x += K * y_inn
    _x[0] += k00*yi0 + k01*yi1;
    _x[1] += k10*yi0 + k11*yi1;
    _x[2] += k20*yi0 + k21*yi1;
    _x[2]  = wrapPi(_x[2]);

    // Covariance update: P = (I - K*H) * P
    // I - K*H is 3x3: the only non-identity columns are 0 and 1
    // (because H has non-zero entries only in cols 0 and 1).
    //
    // (I - K*H)[i][j]:
    //   col 0: delta(i,0) - K[i][0]*H[0][0] - K[i][1]*H[1][0]
    //         = delta(i,0) - K[i][0]    (H[0][0]=1, H[1][0]=0)
    //   col 1: delta(i,1) - K[i][0]*H[0][1] - K[i][1]*H[1][1]
    //         = delta(i,1) - K[i][1]    (H[0][1]=0, H[1][1]=1)
    //   col 2: delta(i,2)               (H has no col-2 entries)
    //
    // So (I-KH)*P[i][j] = P[i][j] - K[i][0]*P[0][j] - K[i][1]*P[1][j]

    float p00 = _P[0][0]; float p01 = _P[0][1]; float p02 = _P[0][2];
    float p10 = _P[1][0]; float p11 = _P[1][1]; float p12 = _P[1][2];
    float p20 = _P[2][0]; float p21 = _P[2][1]; float p22 = _P[2][2];

    _P[0][0] = p00 - k00*p00 - k01*p10;
    _P[0][1] = p01 - k00*p01 - k01*p11;
    _P[0][2] = p02 - k00*p02 - k01*p12;
    _P[1][0] = p10 - k10*p00 - k11*p10;
    _P[1][1] = p11 - k10*p01 - k11*p11;
    _P[1][2] = p12 - k10*p02 - k11*p12;
    _P[2][0] = p20 - k20*p00 - k21*p10;
    _P[2][1] = p21 - k20*p01 - k21*p11;
    _P[2][2] = p22 - k20*p02 - k21*p12;
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

float EKF::x()     const { return _x[0]; }
float EKF::y()     const { return _x[1]; }
float EKF::theta() const { return _x[2]; }

// ---------------------------------------------------------------------------
// wrapPi — wrap angle to (-π, π] using atan2f identity.
// ---------------------------------------------------------------------------

float EKF::wrapPi(float theta)
{
    return atan2f(sinf(theta), cosf(theta));
}
