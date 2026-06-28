#include "PhysicalStateEstimate.h"

PhysicalStateEstimate::PhysicalStateEstimate() {}

void PhysicalStateEstimate::addOdometryObservation(
        HardwareState& s, float trackwidthMm,
        float rotationalSlip, uint32_t now_ms) {
    _odometry.predict(s, trackwidthMm, rotationalSlip, now_ms);
}

void PhysicalStateEstimate::addOtosObservation(
        HardwareState& s,
        float x_otos, float y_otos, float theta_otos_rad,
        float v_otos_mmps, float omega_otos_rads,
        float vy_otos_mmps, uint32_t now_ms) {
    _odometry.correctEKF(s, x_otos, y_otos, theta_otos_rad,
                         v_otos_mmps, omega_otos_rads, vy_otos_mmps, now_ms);
}

void PhysicalStateEstimate::resetPose(
        HardwareState& s, int32_t x_mm, int32_t y_mm, int32_t h_cdeg) {
    _odometry.setPose(s, x_mm, y_mm, h_cdeg);
}

void PhysicalStateEstimate::zero(HardwareState& s) {
    _odometry.zero(s);
}

void PhysicalStateEstimate::getPose(const HardwareState& s,
        int32_t& x_mm, int32_t& y_mm, int32_t& h_cdeg) {
    Odometry::getPose(s, x_mm, y_mm, h_cdeg);
}

void PhysicalStateEstimate::getVelocity(const HardwareState& s,
        float& v_mmps, float& omega_rads) {
    // Read from canonical fused.twist fields (written by Odometry — 047-002).
    v_mmps     = s.fused.twist.vx_mmps;
    omega_rads = s.fused.twist.omega_rads;
}

void PhysicalStateEstimate::initEKF(
        float q_xy, float q_theta, float q_v, float q_omega,
        float r_otos_xy, float r_otos_v, float r_enc_v, float r_otos_theta) {
    _odometry.initEKF(q_xy, q_theta, q_v, q_omega,
                      r_otos_xy, r_otos_v, r_enc_v, r_otos_theta);
}

void PhysicalStateEstimate::setCtx(IOdometer* otos,
                                   const HardwareState* hwState) {
    _odometry.setCtx(otos, hwState);
}

uint32_t PhysicalStateEstimate::otosRejectedCount() const {
    return _odometry.otosRejectedCount();
}
int PhysicalStateEstimate::ekfRejectCount() const {
    return _odometry.ekfRejectCount();
}
float PhysicalStateEstimate::ekfPDiag(int idx) const {
    return _odometry.ekfPDiag(idx);
}
float PhysicalStateEstimate::lastEncV() const {
    return _odometry.lastEncV();
}
float PhysicalStateEstimate::lastEncOmega() const {
    return _odometry.lastEncOmega();
}
bool PhysicalStateEstimate::encOmegaHealthy() const {
    return _odometry.encOmegaHealthy();
}
void PhysicalStateEstimate::setEncOmegaHealthy(bool healthy) {
    _odometry.setEncOmegaHealthy(healthy);
}
bool PhysicalStateEstimate::wedgeActive() const {
    return _odometry.wedgeActive();
}
void PhysicalStateEstimate::setWedgeActive(bool active) {
    _odometry.setWedgeActive(active);
}
void PhysicalStateEstimate::rebaselinePrev(float encL, float encR) {
    _odometry.rebaselinePrev(encL, encR);
}

#ifdef ROBOT_DRIVETRAIN_MECANUM
void PhysicalStateEstimate::setOtosAlphaVy(float alpha) {
    _odometry.setOtosAlphaVy(alpha);
}
#endif  // ROBOT_DRIVETRAIN_MECANUM
