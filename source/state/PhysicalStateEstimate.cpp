#include "PhysicalStateEstimate.h"

PhysicalStateEstimate::PhysicalStateEstimate() {}

void PhysicalStateEstimate::setKinematics(float trackwidth, float rotationalSlip) {
    _odometry.setKinematics(trackwidth, rotationalSlip);
}

void PhysicalStateEstimate::addOdometryObservation(
        float encLeft, float encRight, uint32_t now,
        PoseEstimate& encoderOut, PoseEstimate& fusedOut) {
    _odometry.predict(encLeft, encRight, now, encoderOut, fusedOut);
}

void PhysicalStateEstimate::addOtosObservation(
        float x_otos, float y_otos, float thetaOtos,
        float vOtos, float omegaOtos,
        float vyOtos, uint32_t now,
        PoseEstimate& opticalOut, PoseEstimate& fusedOut) {
    _odometry.correctEKF(x_otos, y_otos, thetaOtos,
                         vOtos, omegaOtos, vyOtos, now,
                         opticalOut, fusedOut);
}

void PhysicalStateEstimate::resetPose(
        float encLeft, float encRight,
        int32_t x, int32_t y, int32_t h,
        PoseEstimate& encoderOut, PoseEstimate& fusedOut) {
    _odometry.setPose(encLeft, encRight, x, y, h, encoderOut, fusedOut);
}

void PhysicalStateEstimate::zero(float encLeft, float encRight,
        PoseEstimate& encoderOut, PoseEstimate& fusedOut) {
    _odometry.zero(encLeft, encRight, encoderOut, fusedOut);
}

void PhysicalStateEstimate::getPose(const PoseEstimate& fused,
        int32_t& x, int32_t& y, int32_t& h) {
    Odometry::getPose(fused, x, y, h);
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
