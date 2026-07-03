// DriveConfig.cpp — toDriveConfig projection (ticket 057-004).
//
// Projects a RobotConfig into a msg::DrivetrainConfig for use by Drive::configure()
// and the fluent builder idiom (drive.newConfig().msg() = toDriveConfig(cfg)).
//
// Motion limits (aMax, vBodyMax, yawRateMax) are NOT mapped here — those belong
// to PlannerConfig scope (Phase 3, ticket 006).

#include "subsystems/drive/Drive.h"  // declares toDriveConfig (global namespace)
#include "types/Config.h"
#include "messages/drivetrain.h"
#include "messages/common.h"

msg::DrivetrainConfig toDriveConfig(const RobotConfig& rc)
{
    msg::DrivetrainConfig cfg;

    // --- Wheel forward signs ---
    cfg.setFwdSignL((int32_t)rc.fwdSignL);
    cfg.setFwdSignR((int32_t)rc.fwdSignR);

    // --- Encoder calibration (mm per degree of motor rotation) ---
    cfg.setTravelCalibL(rc.wheelTravelCalibL);
    cfg.setTravelCalibR(rc.wheelTravelCalibR);

    // --- Geometry ---
    cfg.setTrackwidth(rc.trackwidth);
    cfg.setHalfTrack(rc.halfTrack);
    cfg.setHalfWheelbase(rc.halfWheelbase);

    // --- Wheel saturation / steering headroom ---
    cfg.setVWheelMax(rc.vWheelMax);
    cfg.setSteerHeadroom(rc.steerHeadroom);

    // --- Velocity PID gains ---
    msg::Gains vg{};
    vg.kp    = rc.velKp;
    vg.ki    = rc.velKi;
    vg.kff   = rc.velKff;
    vg.i_max = rc.velIMax;
    vg.kaw   = rc.velKaw;
    cfg.setVelGains(vg);

    // --- Velocity filter and sync ---
    cfg.setVelFiltAlpha(rc.velFiltAlpha);
    cfg.setSyncGain(rc.syncGain);
    cfg.setMinWheel(rc.minWheelSpeed);

    // --- OTOS complementary fusion ---
    cfg.setAlphaPos(rc.alphaPos);
    cfg.setAlphaYaw(rc.alphaYaw);
    cfg.setOtosGate(rc.otosGate);
    cfg.setOtosLinearScale(rc.otosLinearScale);
    cfg.setOtosAngularScale(rc.otosAngularScale);

    // --- Rotational slip ---
    cfg.setRotationalSlip(rc.rotationalSlip);

    // --- OTOS mounting offsets ---
    cfg.setOdomOffX(rc.odomOffX);
    cfg.setOdomOffY(rc.odomOffY);
    cfg.setOdomYaw(rc.odomYaw);
    cfg.setOdomUpsideDown(rc.odomUpsideDown);

    // --- EKF noise parameters ---
    cfg.setEkfQXy(rc.ekfQxy);
    cfg.setEkfQTheta(rc.ekfQtheta);
    cfg.setEkfROtosXy(rc.ekfROtosXy);
    cfg.setEkfROtosTheta(rc.ekfROtosTheta);
    cfg.setEkfQV(rc.ekfQv);
    cfg.setEkfQOmega(rc.ekfQomega);
    cfg.setEkfROtosV(rc.ekfROtosV);
    cfg.setEkfREncV(rc.ekfREncV);

    // --- OTOS lag budget (ms) ---
    cfg.setLagOtos(rc.lagOtos);

    // --- Drivetrain type (0 = differential, 1 = mecanum) ---
    cfg.setDrivetrainType((int32_t)rc.drivetrain);

    // --- Rotation asymmetry correction ---
    cfg.setRotationGainPos(rc.rotationGainPos);
    cfg.setRotationGainNeg(rc.rotationGainNeg);
    cfg.setRotationOffset(rc.rotationOffset);
    cfg.setRotationOffsetNeg(rc.rotationOffsetNeg);

    return cfg;
}
