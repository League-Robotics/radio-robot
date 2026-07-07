// lever_arm.h — LeverArm: OTOS lever-arm (mounting-offset) compensation math
// (ticket 086-005, porting source_old/hal/capability/OtosLeverArm.h — ticket
// 066-001, CR-07/CR-08 lineage).
//
// The OTOS chip's REG_OFFSET register is unwritable on this hardware
// (verified in source_old: the write ACKs but the register reads back 0 —
// see OtosSensor::begin()'s own comment), so the mounting-offset
// compensation is done HOST-SIDE instead: the chip reports the SENSOR's own
// pose (its physical position on the chassis, offset from the robot's
// centre of rotation by offsetX/offsetY), and these two pure functions
// convert between that sensor pose and the chassis CENTRE pose the rest of
// the firmware (and the EKF) actually wants:
//
//   sensor = centre + R(centreHeading) * offset
//   centre = sensor  - R(sensorHeading) * offset   (exact inverse, SAME-
//                                                     INSTANT heading — see
//                                                     below)
//
// Ported from OtosSensor::readTransformed() (the sensor->centre direction)
// and OtosSensor::setWorldPose() (the centre->sensor direction), so a future
// real-hardware driver (source/hal/otos/, ticket 086-006) and any simulated
// leaf that grows the same lever-arm model exercise the EXACT SAME
// compensation code — no behaviour change to either caller, and no second,
// independently-drifting implementation for a sim to "prove" against itself.
//
// *** SAME-INSTANT-HEADING CONTRACT — READ BEFORE CALLING ***
// sensorToCentre()'s sensorHeading parameter MUST be the heading read in the
// SAME I2C burst/sample as sensorX/sensorY — never a heading left over from
// a previous tick or a separately-fused estimate. A past regression (commit
// db11b7c, pre-rebuild tree) produced ~433 mm of phantom translation on a
// pure spin on hardware because the offset rotation used a heading that
// lagged the live spin by a constant ~omega*dt: the residual is a lever-arm
// circle proportional to spin rate, invisible at rest and severe during a
// fast turn. Passing the same-instant heading makes the arc cancel exactly,
// regardless of spin rate. Do not reintroduce this bug — see
// centreToSensor()'s doc comment for the exact-inverse relationship this
// relies on.
//
// Pure math, no state, no I/O, no CODAL dependency — a stateless top-level
// namespace, matching this tree's existing pure-math-helper precedent
// (MotorSlew — source/hal/nezha/motor_slew.h; BodyKinematics —
// source/kinematics/body_kinematics.h).
#pragma once

#include <cmath>

namespace LeverArm {

// sensor -> centre. sensorX/sensorY: the sensor's own reported position
// (already mount-yaw-rotated / upside-down-flip corrected into a world-
// oriented frame, but NOT yet lever-arm-compensated). sensorHeading: the
// SAME-INSTANT heading [rad] the sensor reading was taken at — see this
// file's header comment; using a stale/lagged heading here is exactly the
// db11b7c failure mode. offsetX/offsetY: mounting offset from the chassis
// centre to the sensor [mm] (RobotConfig geometry.odometry_offset_mm).
inline void sensorToCentre(float sensorX, float sensorY, float sensorHeading,
                            float offsetX, float offsetY,
                            float& centreXOut, float& centreYOut)
{
    float c = cosf(sensorHeading);
    float s = sinf(sensorHeading);
    float offsetXWorld = c * offsetX - s * offsetY;
    float offsetYWorld = s * offsetX + c * offsetY;
    centreXOut = sensorX - offsetXWorld;
    centreYOut = sensorY - offsetYWorld;
}

// centre -> sensor (the exact inverse of sensorToCentre() — same rotation
// angle, offset added instead of subtracted). centreX/centreY/centreHeading:
// the chassis centre pose (world frame); centreHeading [rad] and
// sensorHeading above are the SAME value (the mounting offset never affects
// heading, only position), so a caller round-tripping through both functions
// passes one heading reading straight through both calls. offsetX/offsetY:
// same mounting offset as sensorToCentre(). Returns the sensor's own
// world-frame position at that centre pose/heading.
inline void centreToSensor(float centreX, float centreY, float centreHeading,
                            float offsetX, float offsetY,
                            float& sensorXOut, float& sensorYOut)
{
    float c = cosf(centreHeading);
    float s = sinf(centreHeading);
    sensorXOut = centreX + (c * offsetX - s * offsetY);
    sensorYOut = centreY + (s * offsetX + c * offsetY);
}

}  // namespace LeverArm
