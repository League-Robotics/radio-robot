#pragma once
#include <math.h>
#include <stdint.h>

// ===========================================================================
// EKFTiny — thin wrapper over TinyEKF's ekf_t for the 5-state EKF
//
// State vector: x = [x_mm, y_mm, theta_rad, v_mmps, omega_rads]
//
// Drop-in API replacement for EKF. The linear-algebra core (F*P*F^T+Q for
// predict; P*H^T*S^-1 gain and (I-KH)*P update for the M=2 position channel)
// is delegated to ekf_t from TinyEKF. All robustness layers are preserved:
//
//   - arc-segment motion model (predict)
//   - Mahalanobis chi-squared gating for all three update channels
//   - D3 P-inflation gate-recovery for position and heading
//   - scalar manual updates for M=1 channels (heading, velocity)
//   - wedge-aware omega suppression (updateVelocity omega_obs=0 path)
//
// For the M=2 position update, S^-1 is computed analytically (same formula
// as EKF.cpp: det = s00*s11 - s01*s10) to guarantee numerical parity with the
// Python oracle in test_ekf.py, which also uses analytic 2x2 inversion.
// ekf_update's Cholesky invert() is NOT used for the position channel.
//
// Sprint 050, Ticket 003.
// ===========================================================================

// EKF_N and EKF_M must be defined before including tinyekf.h. If a consuming
// TU already set them to different values this will fail at the #error check
// below. EKFTiny.cpp defines them itself before including this header, which is
// the canonical path. Other TUs that only #include "EKFTiny.h" (never calling
// any TinyEKF static functions directly) are fine because tinyekf.h is
// header-only and all its functions are file-static.
#ifndef EKF_N
#define EKF_N 5
#endif
#ifndef EKF_M
#define EKF_M 2
#endif

#if EKF_N != 5
#error "EKFTiny.h requires EKF_N == 5"
#endif
#if EKF_M != 2
#error "EKFTiny.h requires EKF_M == 2"
#endif

#include <tinyekf.h>

class EKFTiny {
public:
    EKFTiny();

    // Initialize noise parameters and reset state to origin.
    //   q_xy      — process noise variance for x and y (mm^2)
    //   q_theta   — process noise variance for heading (rad^2)
    //   q_v       — process noise variance for linear velocity (mm/s)^2
    //   q_omega   — process noise variance for angular velocity (rad/s)^2
    //   r_otos_xy — OTOS measurement noise variance for x and y (mm^2)
    //   r_otos_v  — OTOS measurement noise variance for linear velocity (mm/s)^2
    //   r_enc_v   — encoder measurement noise variance for linear velocity (mm/s)^2
    //
    // BOOT-ONLY: also resets state and covariance (_ekf.x[]/_ekf.P[]) to zero.
    // Not safe to call mid-mission — doing so would teleport the fused pose
    // back to the origin. Its only caller is Drive's constructor, where
    // x[]/P[] are already zero and the reset is a no-op in effect. A live
    // runtime noise update (e.g. from a SET command) must use setNoise()
    // instead (Sprint 067, Ticket 003).
    void init(float q_xy, float q_theta, float q_v, float q_omega,
              float r_otos_xy, float r_otos_v, float r_enc_v);

    // Update noise parameters ONLY — does NOT touch _ekf.x[], _ekf.P[], or
    // the rejection-streak counters. Safe to call at any time, including
    // mid-mission, to live-tune the filter's noise model without disturbing
    // its current pose/velocity belief or gate-recovery state.
    //   q_xy/q_theta/q_v/q_omega/r_otos_xy/r_otos_v/r_enc_v — same meaning
    //   as init()'s parameters of the same name.
    // Sprint 067, Ticket 003.
    void setNoise(float q_xy, float q_theta, float q_v, float q_omega,
                  float r_otos_xy, float r_otos_v, float r_enc_v);

    // Overwrite state with a known pose; zeroes v and omega; sets sane diagonal
    // P-prior (100 mm^2, 100 mm^2, (5 deg)^2, v-vars) instead of zeroing P.
    void setPose(float x, float y, float theta);

    // Predict step: arc-segment motion model for position block; random-walk
    // for velocity block. Uses ekf_predict() for the F*P*F^T+Q computation.
    void predict(float dCenter, float dTheta, float theta_before, float dt_s);

    // Update step: 2D position-only observation from OTOS.
    // Applies Mahalanobis gating (chi-square 2-DOF threshold 5.99).
    // S^-1 computed analytically (parity with Python oracle).
    void updatePosition(float x_otos, float y_otos);

    // Update step: fuse linear and angular velocity measurements.
    // Two sequential scalar (1-DOF) Kalman updates applied manually.
    // Each is independently gated (chi-square 1-DOF threshold 3.84).
    void updateVelocity(float v_meas, float omega_meas, float r_v, float r_omega);

    // Update step: fuse OTOS heading measurement (scalar 1-DOF update).
    // H = [0,0,1,0,0]; innovation is wrap-safe: y = wrapPi(theta_meas - x[2]).
    // Applied manually (no ekf_update call) for numerical parity.
    void updateHeading(float theta_meas, float r_theta);

    // Accessors
    float    x()             const;
    float    y()             const;
    float    theta()         const;
    float    v()             const;
    float    omega()         const;
    uint32_t rejectedCount() const;
    int      getRejectCount() const;
    int      rejHeadStreak()  const;
    int      rejPosStreak()   const;

    // Return P[idx][idx] for idx in [0..4].
    float pDiag(int idx) const
    {
        return (idx >= 0 && idx < 5) ? _ekf.P[idx * 5 + idx] : -1.0f;
    }

    // --- Test-harness accessors (sim_api.cpp / test_ekf.py parity gate) ---
    // These expose internal state needed by the EKFTiny parity tests in
    // tests/simulation/unit/test_ekf.py. They mirror the Python EKF mirror's
    // direct attribute access (_x[i], _P[i][j], _rej_pos_streak, etc.).
    // Sprint 050, Ticket 004.

    // P[row][col] — full covariance matrix entry read.
    float pEntry(int row, int col) const
    {
        return (row >= 0 && row < 5 && col >= 0 && col < 5)
               ? _ekf.P[row * 5 + col] : 0.0f;
    }

    // x[idx] — state vector read.
    float xEntry(int idx) const
    {
        return (idx >= 0 && idx < 5) ? _ekf.x[idx] : 0.0f;
    }

    // Direct state injection — needed by tests that pre-set v, omega, theta.
    void setXEntry(int idx, float val)
    {
        if (idx >= 0 && idx < 5) _ekf.x[idx] = val;
    }

    // Direct covariance injection — needed by tests that set P[2][2]=0.1 etc.
    void setPEntry(int row, int col, float val)
    {
        if (row >= 0 && row < 5 && col >= 0 && col < 5)
            _ekf.P[row * 5 + col] = val;
    }

    // Direct streak injection — needed by pre-005-logic tests that reset streaks.
    void setRejPosStreak(int s) { _rejPos_streak = s; }
    void setRejHeadStreak(int s) { _rejHead_streak = s; }

private:
    ekf_t    _ekf;
    float    _Q[5][5];    // process noise (diagonal)
    float    _rOtosXy;    // OTOS position noise variance (same for x and y)
    float    _rOtosV;     // OTOS velocity noise variance
    float    _rEncV;      // encoder velocity noise variance
    uint32_t _rejected;       // cumulative count of gated (rejected) observations
    int      _rejHead_streak; // consecutive heading-update rejection streak
    int      _rejPos_streak;  // consecutive position-update rejection streak

    // Wrap angle to (-pi, pi] using atan2f identity — same as EKF::wrapPi.
    static float wrapPi(float theta);

    // Sane P-prior diagonal values used by setPose() — must match EKF.h exactly.
    static constexpr float kPriorXY    = 100.0f;      // mm^2
    static constexpr float kPriorTheta = 0.00762f;    // (5 deg in rad)^2
    static constexpr float kPriorV     = 100.0f;      // (mm/s)^2
    static constexpr float kPriorOmega = 0.01f;       // (rad/s)^2
};
