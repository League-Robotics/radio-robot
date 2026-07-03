#include "PhysicalStateEstimate.h"

PhysicalStateEstimate::PhysicalStateEstimate() {}

void PhysicalStateEstimate::setKinematics(float trackwidthMm, float rotationalSlip) {
    _odometry.setKinematics(trackwidthMm, rotationalSlip);
}

void PhysicalStateEstimate::addOdometryObservation(
        float encLeftMm, float encRightMm, uint32_t now_ms,
        PoseEstimate& encoderOut, PoseEstimate& fusedOut) {
    _odometry.predict(encLeftMm, encRightMm, now_ms, encoderOut, fusedOut);
}

void PhysicalStateEstimate::addOtosObservation(
        float x_otos, float y_otos, float theta_otos_rad,
        float v_otos_mmps, float omega_otos_rads,
        float vy_otos_mmps, uint32_t now_ms,
        PoseEstimate& opticalOut, PoseEstimate& fusedOut) {
    _odometry.correctEKF(x_otos, y_otos, theta_otos_rad,
                         v_otos_mmps, omega_otos_rads, vy_otos_mmps, now_ms,
                         opticalOut, fusedOut);
}

void PhysicalStateEstimate::resetPose(
        float encLeftMm, float encRightMm,
        int32_t x_mm, int32_t y_mm, int32_t h_cdeg,
        PoseEstimate& encoderOut, PoseEstimate& fusedOut) {
    _odometry.setPose(encLeftMm, encRightMm, x_mm, y_mm, h_cdeg, encoderOut, fusedOut);
}

void PhysicalStateEstimate::zero(float encLeftMm, float encRightMm,
        PoseEstimate& encoderOut, PoseEstimate& fusedOut) {
    _odometry.zero(encLeftMm, encRightMm, encoderOut, fusedOut);
}

void PhysicalStateEstimate::getPose(const PoseEstimate& fused,
        int32_t& x_mm, int32_t& y_mm, int32_t& h_cdeg) {
    Odometry::getPose(fused, x_mm, y_mm, h_cdeg);
}

void PhysicalStateEstimate::initEKF(
        float q_xy, float q_theta, float q_v, float q_omega,
        float r_otos_xy, float r_otos_v, float r_enc_v, float r_otos_theta) {
    _odometry.initEKF(q_xy, q_theta, q_v, q_omega,
                      r_otos_xy, r_otos_v, r_enc_v, r_otos_theta);
}

void PhysicalStateEstimate::setNoise(
        float q_xy, float q_theta, float q_v, float q_omega,
        float r_otos_xy, float r_otos_v, float r_enc_v, float r_otos_theta) {
    _odometry.setNoise(q_xy, q_theta, q_v, q_omega,
                       r_otos_xy, r_otos_v, r_enc_v, r_otos_theta);
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
