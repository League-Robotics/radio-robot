#pragma once
#include <stdint.h>

/**
 * Odometry — differential-drive dead-reckoning pose tracker.
 *
 * Internal state is float for accuracy; protocol output is integer.
 * Heading convention: 0 = +X axis, positive = CCW (standard math).
 * Output convention: centidegrees (360 degrees = 36000 cdeg).
 *
 * Primary API: call predict() once per fast tick with current encoder
 * positions (absolute mm). Odometry owns the previous-encoder snapshot
 * and computes deltas internally, then applies midpoint (exact-arc)
 * integration per docs/kinematics-model.md §2.4:
 *
 *   dC = (dL + dR)/2 ;  dθ = (dR − dL)/b
 *   θ_mid = θ + dθ/2
 *   x += dC·cos(θ_mid) ;  y += dC·sin(θ_mid) ;  θ = wrapπ(θ + dθ)
 *
 * Sprint 010, Ticket 005.
 * Sprint 010, Ticket 006: adds correct() for OTOS complementary fusion
 * (predict/correct interface per docs/kinematics-model.md §2.4).
 */
class Odometry {
public:
    Odometry();

    // Midpoint (exact-arc) integration — primary predict step.
    // encLMm, encRMm: current absolute encoder positions in mm.
    // trackwidthMm: distance between wheel contact points in mm.
    // Computes dL/dR against stored _prevEncL/_prevEncR, updates them,
    // then advances pose using midpoint heading.
    void predict(float encLMm, float encRMm, float trackwidthMm);

    // OTOS complementary correction — correct step of predict/correct.
    // (docs/kinematics-model.md §2.4; EKF upgrade path replaces this later.)
    //
    // Outlier gate: if distance(otos, predicted) > otosGate, the sample is
    // rejected; _otosRejected is incremented and pose is unchanged.
    //
    // If accepted:
    //   _x        += alphaPos * (x_otos - _x)
    //   _y        += alphaPos * (y_otos - _y)
    //   _heading  += alphaYaw * wrapπ(θ_otos - _heading)   [angle-wrap safe]
    //
    // alphaPos, alphaYaw: blend gains in [0, 1] (from RobotConfig).
    // otosGate: rejection distance threshold in mm (from RobotConfig).
    void correct(float x_otos, float y_otos, float theta_otos_rad,
                 float alphaPos, float alphaYaw, float otosGate);

    // Telemetry: number of OTOS samples rejected by the outlier gate since boot.
    uint32_t otosRejectedCount() const { return _otosRejected; }

    // Legacy forward-Euler integrate (deprecated; prefer predict()).
    // dL_mm, dR_mm: signed mm traveled by left and right wheels this tick.
    // trackwidthMm: distance between wheel contact points in mm.
    void update(float dL_mm, float dR_mm, float trackwidthMm);

    // Read current pose. x_mm and y_mm are integer mm; h_cdeg is
    // centidegrees (-18000..+18000 clamped).
    void getPose(int32_t& x_mm, int32_t& y_mm, int32_t& h_cdeg) const;

    // Overwrite pose (used by SI command).
    // h_cdeg is centidegrees; stored internally as radians.
    // Resets _prevEncL/_prevEncR to 0; caller must snapshot current
    // encoder positions before the next predict() call or pass them
    // here if desired (zero() resets to 0 and DriveController snapshots).
    void setPose(int32_t x_mm, int32_t y_mm, int32_t h_cdeg);

    // Zero pose: equivalent to setPose(0, 0, 0).
    // Resets _prevEncL/_prevEncR to 0.
    void zero();

private:
    float    _x;          // mm, float internal
    float    _y;          // mm, float internal
    float    _headingRad; // radians

    float    _prevEncL;   // last encoder snapshot, mm (owned by Odometry)
    float    _prevEncR;   // last encoder snapshot, mm (owned by Odometry)

    uint32_t _otosRejected; // count of OTOS samples rejected by outlier gate

    // Wrap heading to (-π, π] using atan2f identity.
    static float wrapPi(float theta);

    static constexpr float PI_F        = 3.14159265f;
    static constexpr float RAD_TO_CDEG = 18000.0f / 3.14159265f;
    static constexpr float CDEG_TO_RAD = 3.14159265f / 18000.0f;
};
