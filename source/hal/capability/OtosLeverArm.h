#pragma once
#include <cmath>

/**
 * OtosLeverArm.h — shared OTOS lever-arm (mounting-offset) compensation math
 * (ticket 066-001, CR-07/CR-08).
 *
 * The OTOS chip's REG_OFFSET is unwritable on this hardware (verified: the
 * write ACKs but the register reads back 0 — see OtosSensor::begin()), so the
 * mounting-offset compensation is done HOST-SIDE instead: the chip reports the
 * SENSOR's own pose (its physical position on the chassis, offset from the
 * robot's centre of rotation by `odomOffX`/`odomOffY`), and these two pure
 * functions convert between that sensor pose and the chassis CENTRE pose the
 * rest of the firmware (and the EKF) actually wants:
 *
 *   sensor = centre + R(centreHrad) * offset
 *   centre = sensor  - R(sensorHrad) * offset     (exact inverse, same-instant
 *                                                   heading — see below)
 *
 * Extracted VERBATIM from OtosSensor::readTransformed() (the sensor->centre
 * direction) and OtosSensor::setWorldPose() (the centre->sensor direction) so
 * BOTH the real hardware driver (source/hal/real/OtosSensor.cpp) and the
 * simulated sensor (source/hal/sim/SimOdometer.cpp) exercise the EXACT SAME
 * compensation code — no behaviour change to either caller, and no second,
 * independently-drifting implementation for the sim to "prove" against itself.
 *
 * A past regression in this exact math (commit db11b7c) produced 433 mm of
 * phantom translation on a pure spin on hardware, because the offset rotation
 * used a heading that lagged the live spin. Sharing this header means a future
 * regression here fails identically in both hardware and sim tests.
 *
 * Pure math, no state, no I/O, no CODAL dependency — a stateless leaf
 * dependency of both source/hal/real/ and source/hal/sim/ (architecturally
 * equivalent to source/control/Odometry.h's effectiveSlip() shared helper).
 */

// sensor -> centre.  sensorX/sensorY: the sensor's own reported position
// (already mount-yaw-rotated / upside-down-flip corrected into a world-
// oriented frame, but NOT yet lever-arm-compensated). sensorHrad: the
// SAME-INSTANT heading (radians) the sensor reading was taken at — using a
// stale/lagged heading here is exactly the db11b7c failure mode (a residual
// lever-arm circle proportional to spin rate), so callers must pass the
// heading read in the same burst/sample as sensorX/sensorY, not a heading
// from a previous tick. offX/offY: mounting offset from the chassis centre
// to the sensor, mm (RobotConfig odomOffX/odomOffY).
inline void sensorToCentre(float sensorX, float sensorY, float sensorHrad,
                            float offX, float offY,
                            float& centreXOut, float& centreYOut)
{
    float ch = cosf(sensorHrad);
    float sh = sinf(sensorHrad);
    float offXWorld = ch * offX - sh * offY;
    float offYWorld = sh * offX + ch * offY;
    centreXOut = sensorX - offXWorld;
    centreYOut = sensorY - offYWorld;
}

// centre -> sensor (the exact inverse of sensorToCentre — same rotation angle,
// offset added instead of subtracted). centreX/centreY/centreHrad: the
// chassis centre pose (world frame). Returns the sensor's own world-frame
// position at that centre pose/heading.
inline void centreToSensor(float centreX, float centreY, float centreHrad,
                            float offX, float offY,
                            float& sensorXOut, float& sensorYOut)
{
    float ch = cosf(centreHrad);
    float sh = sinf(centreHrad);
    sensorXOut = centreX + (ch * offX - sh * offY);
    sensorYOut = centreY + (sh * offX + ch * offY);
}
