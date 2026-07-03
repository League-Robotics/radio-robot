#include "Odometry.h"
#include <math.h>

Odometry::Odometry()
    : _prevEncL(0.0f), _prevEncR(0.0f)
    , _encPoseX(0.0f), _encPoseY(0.0f), _encPoseH(0.0f)
    , _encVx(0.0f), _encVy(0.0f), _encOmega(0.0f)
    , _otosRejected(0)
    , _lastPredict(0)
    , _rOtosV(0.0f), _rEncV(0.0f), _rOtosTheta(0.0f)
    , _lastEncV(0.0f), _lastEncOmega(0.0f)
{
}

// ---------------------------------------------------------------------------
// predict — midpoint (exact-arc) integration (docs/kinematics-model.md §2.4)
//
// Reads encLeft/encRight (raw cumulative encoder readings) and the
// trackwidth/rotationalSlip set by the most recent setKinematics() call;
// writes fusedOut.pose.{x,y,h} and the encoder-only accumulator into
// encoderOut.
// ---------------------------------------------------------------------------

void Odometry::predict(float encLeft, float encRight, uint32_t now,
                       PoseEstimate& encoderOut, PoseEstimate& fusedOut)
{
    // Compute dt — use signed cast to avoid uint32 underflow on rollover.
    // (See watchdog-uint32-underflow project finding: never plain-subtract
    //  two uint32 ms stamps without a signed cast.)
    float dt_s = 0.0f;
    if (_lastPredict == 0) {
        // First call: seed the timestamp, skip velocity update this tick.
        _lastPredict = now;
    } else {
        dt_s = (int32_t)(now - _lastPredict) * 0.001f;
    }

    float theta_before = fusedOut.pose.h;   // heading before this step — MUST be first

    float dL = encLeft  - _prevEncL;
    float dR = encRight - _prevEncR;
    _prevEncL = encLeft;
    _prevEncR = encRight;

    float dCenter   = (dL + dR) * 0.5f;
    // Apply rotational-slip correction: encoder arc over-reports body rotation
    // (wheel scrub during turns).  slip factor in [0.5, 1.0]; 0/unset → 1.0.
    // (024-006: rotationalSlip is now active — was dead before this sprint.)
    float slip      = effectiveSlip(_rotationalSlip);
    float dTheta    = ((dR - dL) / _trackwidth) * slip;

    // (033-005e) Wedge defense: while a wheel is wedged its encoder is frozen,
    // so the differential (dR - dL) contains phantom heading rotation.  Suppress
    // dTheta entirely — hold heading — to prevent pose and EKF corruption.
    // _encOmegaHealthy is also suppressed by the same event (Robot wires both).
    if (_wedgeActive) {
        dTheta = 0.0f;
    }

    float thetaMid  = fusedOut.pose.h + dTheta * 0.5f;

    // 047-002: arc-integrate encoder deltas into the encoder-only accumulator FIRST.
    // This accumulator is never touched by the EKF so it provides a pure
    // dead-reckoning baseline for fusion validation.
    float encThetaMid = _encPoseH + dTheta * 0.5f;
    _encPoseX += dCenter * cosf(encThetaMid);
    _encPoseY += dCenter * sinf(encThetaMid);
    _encPoseH  = wrapPi(_encPoseH + dTheta);

    // Dead-reckoning advance of the fused pose (EKF seed before EKF overwrites below).
    fusedOut.pose.x += dCenter * cosf(thetaMid);
    fusedOut.pose.y += dCenter * sinf(thetaMid);
    fusedOut.pose.h  = wrapPi(fusedOut.pose.h + dTheta);

    // Compute encoder-rate velocity for this tick.
    // Guard against dt_s == 0 (first tick or duplicate timestamp): skip velocity
    // computation and retain previous value (which is 0 on the very first tick).
    if (dt_s > 0.0f) {
        _lastEncV     = dCenter / dt_s;        // body linear speed (mm/s)
        _lastEncOmega = dTheta  / dt_s;        // yaw rate (rad/s)
        _encVx        = _lastEncV;
        _encVy        = 0.0f;                  // differential: no lateral encoder obs
        _encOmega     = _lastEncOmega;
    }
    // else: retain previous _lastEncV/_lastEncOmega / _encV* (0 on first tick)

    // 047-002: populate encoderOut from the private accumulator.
    // EKF has NOT run yet — these fields reflect pure dead-reckoning.
    encoderOut.pose.x          = _encPoseX;
    encoderOut.pose.y          = _encPoseY;
    encoderOut.pose.h          = _encPoseH;
    encoderOut.twist.vx_mmps   = _encVx;
    encoderOut.twist.vy_mmps   = _encVy;
    encoderOut.twist.omega_rads = _encOmega;
    encoderOut.stamp.lastUpdMs = now;
    encoderOut.stamp.valid     = true;

    // EKF predict — propagate state and covariance using encoder-derived arc segment.
    _ekf.predict(dCenter, dTheta, theta_before, dt_s);

    // Fuse encoder-derived velocity into the EKF EVERY tick, regardless of OTOS
    // health (033-003).  The EKF velocity states (v, omega) are a random walk in
    // predict() — they only change via updateVelocity().  Previously that was
    // called ONLY inside correctEKF() (the OTOS-gated path), so fusedV/fusedOmega
    // were stuck at 0 whenever OTOS was invalid (lifted stand, real-world
    // dropout): twist read 0 even while the wheels turned.  Encoder velocity is
    // now the always-available velocity source; OTOS pose/heading/velocity fusion
    // stays gated in correctEKF() and is no longer the only writer of v/omega.
    //
    // Suppress the omega observation when an encoder is wedged — a frozen wheel
    // injects phantom yaw rate into the fused heading.  _encOmegaHealthy is driven
    // by the wedge detector (033-005) and defaults true; linear v still fuses (a
    // wedged wheel only corrupts the differential term).  Guard on dt_s > 0: on
    // the first tick _lastEncV/_lastEncOmega are still 0 and there is no rate to
    // fuse.
    if (dt_s > 0.0f) {
        float omega_obs = _encOmegaHealthy ? _lastEncOmega : 0.0f;
        _ekf.updateVelocity(_lastEncV, omega_obs, _rEncV, _rEncV);
    }

    // 047-002: populate fusedOut from EKF output (after enc-velocity fusion).
    fusedOut.pose.x           = _ekf.x();
    fusedOut.pose.y           = _ekf.y();
    fusedOut.pose.h           = _ekf.theta();
    fusedOut.twist.vx_mmps    = _ekf.v();
    fusedOut.twist.vy_mmps    = 0.0f;          // updated in correctEKF for mecanum
    fusedOut.twist.omega_rads = _ekf.omega();
    fusedOut.stamp.lastUpdMs  = now;
    fusedOut.stamp.valid      = true;

    _lastPredict = now;
}

// ---------------------------------------------------------------------------
// correct — OTOS complementary correction (docs/kinematics-model.md §2.4)
//
// Reads and writes s.fused.pose.{x,y,h}.
// ---------------------------------------------------------------------------

void Odometry::correct(HardwareState& s,
                       float x_otos, float y_otos, float thetaOtos,
                       float alphaPos, float alphaYaw, float otosGate)
{
    // Outlier gate: reject if OTOS position disagrees with predicted pose
    // by more than the gate threshold.
    float dx = x_otos - s.fused.pose.x;
    float dy = y_otos - s.fused.pose.y;
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist > otosGate) {
        ++_otosRejected;
        return;
    }

    // Accepted: complementary blend of position.
    s.fused.pose.x += alphaPos * dx;
    s.fused.pose.y += alphaPos * dy;

    // Heading blend: angle-wrap-safe — blend on the angular difference,
    // not on the raw angle, to avoid crossing the ±π discontinuity.
    float dh = wrapPi(thetaOtos - s.fused.pose.h);
    s.fused.pose.h = wrapPi(s.fused.pose.h + alphaYaw * dh);
}

// ---------------------------------------------------------------------------
// getPose — read pose from fused.pose and convert to integer mm + centidegrees.
// ---------------------------------------------------------------------------

void Odometry::getPose(const PoseEstimate& fused,
                       int32_t& x, int32_t& y, int32_t& h)
{
    x = static_cast<int32_t>(fused.pose.x);
    y = static_cast<int32_t>(fused.pose.y);

    float cdeg = fused.pose.h * RAD_TO_CDEG;
    if (cdeg >  18000.0f) cdeg =  18000.0f;
    if (cdeg < -18000.0f) cdeg = -18000.0f;
    h = static_cast<int32_t>(cdeg);
}

// ---------------------------------------------------------------------------
// setPose — write pose into fusedOut/encoderOut; also reset prev-encoder snapshot.
// ---------------------------------------------------------------------------

void Odometry::setPose(float encLeft, float encRight,
                       int32_t x, int32_t y, int32_t h,
                       PoseEstimate& encoderOut, PoseEstimate& fusedOut)
{
    float newX = static_cast<float>(x);
    float newY = static_cast<float>(y);
    float newH = static_cast<float>(h) * CDEG_TO_RAD;

    fusedOut.pose.x = newX;
    fusedOut.pose.y = newY;
    fusedOut.pose.h = newH;

    // Re-baseline encoder snapshot to current encoder values (not 0.0f).
    // This prevents a spurious encoder-delta jump on the very next predict()
    // call after a camera fix (SI command) when encoders are non-zero.
    // Note: zero() calls setPose(encL, encR, 0, 0, 0, ...) at startup when
    // encoders read 0, so _prevEncL = encLeft = 0 there — identical to the
    // old behaviour on boot.
    _prevEncL  = encLeft;
    _prevEncR  = encRight;

    // 047-002: reset the encoder-only accumulator to the new pose value so the
    // dead-reckoning baseline stays consistent with the absolute fix.
    _encPoseX  = newX;
    _encPoseY  = newY;
    _encPoseH  = newH;
    _encVx     = 0.0f;
    _encVy     = 0.0f;
    _encOmega  = 0.0f;

    // Also update encoder estimate pose for consistency.
    encoderOut.pose.x = newX;
    encoderOut.pose.y = newY;
    encoderOut.pose.h = newH;

    _ekf.setPose(newX, newY, newH);
}

// ---------------------------------------------------------------------------
// zero — reset pose to origin; reset prev-encoder snapshot.
// ---------------------------------------------------------------------------

void Odometry::zero(float encLeft, float encRight,
                    PoseEstimate& encoderOut, PoseEstimate& fusedOut)
{
    setPose(encLeft, encRight, 0, 0, 0, encoderOut, fusedOut);
}

// ---------------------------------------------------------------------------
// wrapPi — keep heading in (-π, π]
// ---------------------------------------------------------------------------

float Odometry::wrapPi(float theta)
{
    return atan2f(sinf(theta), cosf(theta));
}

// ---------------------------------------------------------------------------
// initEKF — set EKF process and measurement noise parameters.
// ---------------------------------------------------------------------------

void Odometry::initEKF(float q_xy, float q_theta, float q_v, float q_omega,
                       float r_otos_xy, float r_otos_v, float r_enc_v,
                       float r_otos_theta)
{
    _ekf.init(q_xy, q_theta, q_v, q_omega, r_otos_xy, r_otos_v, r_enc_v);
    // Cache the velocity noise params for use in correctEKF() calls.
    // _rOtosV is used for both v and omega of the OTOS source (symmetric
    // simplification — v1 design; separate v/omega noise is a future extension).
    _rOtosV     = r_otos_v;
    _rEncV      = r_enc_v;
    _rOtosTheta = r_otos_theta;  // OTOS heading noise (sprint 024-004)
}

// ---------------------------------------------------------------------------
// setNoise — live noise update; does NOT reset EKF state/covariance.
// Sprint 067, Ticket 003.
// ---------------------------------------------------------------------------

void Odometry::setNoise(float q_xy, float q_theta, float q_v, float q_omega,
                        float r_otos_xy, float r_otos_v, float r_enc_v,
                        float r_otos_theta)
{
    _ekf.setNoise(q_xy, q_theta, q_v, q_omega, r_otos_xy, r_otos_v, r_enc_v);
    // Refresh the cached velocity/heading noise params used by correctEKF(),
    // mirroring initEKF()'s caching exactly.
    _rOtosV     = r_otos_v;
    _rEncV      = r_enc_v;
    _rOtosTheta = r_otos_theta;
}

// ---------------------------------------------------------------------------
// correctEKF — apply OTOS position, heading, and velocity observations to
// the EKF (sprint 024-004: heading fusion added).
//
// Update order: position → heading → velocity(OTOS).
// All channels are Mahalanobis-gated inside EKF methods.
//
// 033-003: encoder-derived velocity is NO LONGER fused here.  It is fused
// unconditionally in predict() every tick so that fusedV/fusedOmega stay live
// even when OTOS is invalid.  Fusing it here too would double-count the same
// encoder observation per OTOS tick.
// ---------------------------------------------------------------------------

void Odometry::correctEKF(float x_otos, float y_otos,
                          float thetaOtos,
                          float vOtos, float omegaOtos,
                          float vyOtos, uint32_t now,
                          PoseEstimate& opticalOut, PoseEstimate& fusedOut)
{
    // 047-002: capture raw OTOS observation into opticalOut BEFORE EKF update.
    // pose: store the raw OTOS reading (do NOT differentiate; Q4 resolved).
    // twist: reuse the v/omega values passed in as the optical twist estimate.
    // stamp: mark valid with the current now.
    opticalOut.pose.x       = x_otos;
    opticalOut.pose.y       = y_otos;
    opticalOut.pose.h       = thetaOtos;
    opticalOut.twist.vx_mmps   = vOtos;
    opticalOut.twist.vy_mmps   = vyOtos;
    opticalOut.twist.omega_rads = omegaOtos;
    opticalOut.stamp.lastUpdMs = now;
    opticalOut.stamp.valid     = true;

    // 1. Fuse OTOS position (Mahalanobis-gated inside EKF).
    _ekf.updatePosition(x_otos, y_otos);

    // 2. Fuse OTOS heading (sprint 024-004). H=[0,0,1,0,0]; wrap-safe innovation.
    _ekf.updateHeading(thetaOtos, _rOtosTheta);

    // 3. Fuse OTOS velocity (v, omega). Single scalar _rOtosV used for both
    //    v and omega noise (symmetric simplification — v1 design).
    _ekf.updateVelocity(vOtos, omegaOtos, _rOtosV, _rOtosV);

    // 047-002: write structured fused estimate from EKF output.
    fusedOut.pose.x          = _ekf.x();
    fusedOut.pose.y          = _ekf.y();
    fusedOut.pose.h          = _ekf.theta();
    fusedOut.twist.vx_mmps   = _ekf.v();
    fusedOut.twist.vy_mmps   = 0.0f;   // differential: no lateral velocity; vyOtos captured in optical only
    fusedOut.twist.omega_rads = _ekf.omega();
    fusedOut.stamp.lastUpdMs = now;
    fusedOut.stamp.valid     = true;
    // vyOtos is captured into opticalOut.twist above (before EKF update);
    // on the differential build it is always 0.0f and is not fused into fused.twist.
}
