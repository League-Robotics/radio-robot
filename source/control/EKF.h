#pragma once
#include <math.h>

// ===========================================================================
// EKF — 3-state Extended Kalman Filter for pose fusion
//
// State vector: x = [x_mm, y_mm, theta_rad]
//
// Motion model: arc-segment (midpoint integration), same geometry as Odometry.
// Observation model: 2D position-only (OTOS x,y; heading not observed).
//
// All matrix operations (3x3, 3x2) are unrolled as plain float arithmetic.
// No heap allocation, no STL, no Eigen.
//
// Sprint 022, Ticket 001.
// ===========================================================================

class EKF {
public:
    EKF();

    // Initialize noise parameters and reset state to origin.
    //   q_xy      — process noise variance for x and y (mm^2)
    //   q_theta   — process noise variance for heading (rad^2)
    //   r_otos_xy — OTOS measurement noise variance for x and y (mm^2)
    void init(float q_xy, float q_theta, float r_otos_xy);

    // Overwrite state with a known pose; reset covariance to zero.
    void setPose(float x, float y, float theta);

    // Predict step: arc-segment motion model.
    //   dCenter     — distance traveled by the center point (mm)
    //   dTheta      — change in heading (rad)
    //   theta_before — heading at the start of this step (rad)
    void predict(float dCenter, float dTheta, float theta_before);

    // Update step: 2D position-only observation from OTOS.
    void update(float x_otos, float y_otos);

    // Accessors
    float x()     const;
    float y()     const;
    float theta() const;

private:
    float _x[3];      // state: [x_mm, y_mm, theta_rad]
    float _P[3][3];   // covariance matrix
    float _Q[3][3];   // process noise (diagonal)
    float _r;         // OTOS position noise variance (scalar, same for x and y)

    // Wrap angle to (-π, π] using atan2f identity.
    static float wrapPi(float theta);
};
