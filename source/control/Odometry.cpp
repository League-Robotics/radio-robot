#include "Odometry.h"
#include <math.h>

Odometry::Odometry()
    : _x(0.0f), _y(0.0f), _headingRad(0.0f)
    , _prevEncL(0.0f), _prevEncR(0.0f)
{
}

// ---------------------------------------------------------------------------
// predict — midpoint (exact-arc) integration (docs/kinematics-model.md §2.4)
// ---------------------------------------------------------------------------

void Odometry::predict(float encLMm, float encRMm, float trackwidthMm)
{
    float dL = encLMm - _prevEncL;
    float dR = encRMm - _prevEncR;
    _prevEncL = encLMm;
    _prevEncR = encRMm;

    float dCenter   = (dL + dR) * 0.5f;
    float dTheta    = (dR - dL) / trackwidthMm;
    float thetaMid  = _headingRad + dTheta * 0.5f;

    _x          += dCenter * cosf(thetaMid);
    _y          += dCenter * sinf(thetaMid);
    _headingRad  = wrapPi(_headingRad + dTheta);
}

// ---------------------------------------------------------------------------
// update — legacy forward-Euler (deprecated; callers should use predict())
// ---------------------------------------------------------------------------

void Odometry::update(float dL_mm, float dR_mm, float trackwidthMm)
{
    float dCenter = (dL_mm + dR_mm) * 0.5f;
    float dTheta  = (dR_mm - dL_mm) / trackwidthMm;

    _x          += dCenter * cosf(_headingRad);
    _y          += dCenter * sinf(_headingRad);
    _headingRad += dTheta;
}

void Odometry::getPose(int32_t& x_mm, int32_t& y_mm, int32_t& h_cdeg) const
{
    x_mm = static_cast<int32_t>(_x);
    y_mm = static_cast<int32_t>(_y);

    float cdeg = _headingRad * RAD_TO_CDEG;
    if (cdeg >  18000.0f) cdeg =  18000.0f;
    if (cdeg < -18000.0f) cdeg = -18000.0f;
    h_cdeg = static_cast<int32_t>(cdeg);
}

void Odometry::setPose(int32_t x_mm, int32_t y_mm, int32_t h_cdeg)
{
    _x          = static_cast<float>(x_mm);
    _y          = static_cast<float>(y_mm);
    _headingRad = static_cast<float>(h_cdeg) * CDEG_TO_RAD;
    _prevEncL   = 0.0f;
    _prevEncR   = 0.0f;
}

void Odometry::zero()
{
    setPose(0, 0, 0);
}

// ---------------------------------------------------------------------------
// wrapPi — keep heading in (-π, π]
// ---------------------------------------------------------------------------

float Odometry::wrapPi(float theta)
{
    return atan2f(sinf(theta), cosf(theta));
}
