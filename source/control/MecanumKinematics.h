#pragma once
#include "Pose2D.h"
#include <stdint.h>

/**
 * MecanumKinematics — stateless kinematic maps for a 4-wheel X-roller
 * mecanum drivetrain (046-002).
 *
 * All functions are pure: no I/O, no global state, no heap allocation.
 *
 * Wheel index order (canonical, matching Nezha ports 1–4):
 *   [0] = FR (Front-Right)
 *   [1] = FL (Front-Left)
 *   [2] = BR (Back-Right)
 *   [3] = BL (Back-Left)
 *
 * Combined geometry constant:
 *   k = halfTrackMm + halfWheelbaseMm
 *
 * Inverse kinematics (body twist → raw wheel speeds, before sign correction):
 *   FR_raw =  vx - vy - k * omega
 *   FL_raw =  vx + vy + k * omega
 *   BR_raw =  vx + vy - k * omega
 *   BL_raw =  vx - vy + k * omega
 *   wheels[i] = raw[i] * signs[i]
 *
 * Forward kinematics (wheel speeds → body twist, after dividing out signs):
 *   w[i] = wheels[i] * signs[i]   (since signs[i] == ±1, multiply == divide)
 *   vx    = ( w[0] + w[1] + w[2] + w[3]) / 4
 *   vy    = (-w[0] + w[1] + w[2] - w[3]) / 4        (BL contributes positively)
 *   omega = (-w[0] + w[1] - w[2] + w[3]) / (4 * k)
 *
 * Unit conventions:
 *   vx, vy  : mm/s (body frame; vx = forward, vy = left-positive)
 *   omega   : rad/s (CCW-positive)
 *   wheels  : mm/s (signed, after sign application)
 *   signs   : ±1 (from RobotConfig.fwdSign{FR,FL,BR,BL}; FL=+1 is primary ref)
 *   k       : mm (halfTrackMm + halfWheelbaseMm)
 */
namespace MecanumKinematics {

/**
 * inverse — body twist → 4 wheel speeds.
 *
 * @param t      3-DOF body twist (vx_mmps, vy_mmps, omega_rads)
 * @param geom   robot geometry (halfTrackMm, halfWheelbaseMm)
 * @param signs  per-wheel forward signs [FR, FL, BR, BL] (from RobotConfig)
 * @param wheels output wheel speeds [FR, FL, BR, BL], mm/s
 */
void inverse(BodyTwist3 t, const RobotGeometry& geom,
             const int8_t signs[4], float wheels[4]);

/**
 * forward — 4 wheel speeds → body twist.
 *
 * signs are applied in reverse (multiply by ±1 to undo the forward-sign
 * encoding baked into wheels[]).
 *
 * @param wheels  input wheel speeds [FR, FL, BR, BL], mm/s
 * @param geom    robot geometry (halfTrackMm, halfWheelbaseMm)
 * @param signs   per-wheel forward signs [FR, FL, BR, BL] (from RobotConfig)
 * @param t_out   output 3-DOF body twist (vx_mmps, vy_mmps, omega_rads)
 */
void forward(const float wheels[4], const RobotGeometry& geom,
             const int8_t signs[4], BodyTwist3& t_out);

/**
 * saturate — uniform scale when any |wheel| > vWheelMax.
 *
 * Preserves twist direction (no per-wheel clipping). Unlike the differential
 * saturate, there is no steerHeadroom parameter — the caller may pre-subtract
 * headroom from vWheelMax before calling.
 *
 * When max(|wheels[i]|) > vWheelMax, all outputs are scaled by
 *   s = vWheelMax / max(|wheels[i]|)
 * so the fastest wheel sits exactly at vWheelMax. Otherwise outputs == inputs.
 *
 * @param wheels    input wheel speeds [FR, FL, BR, BL], mm/s
 * @param vWheelMax wheel speed ceiling, mm/s (must be > 0)
 * @param out       output (scaled) wheel speeds [FR, FL, BR, BL], mm/s
 */
void saturate(float wheels[4], float vWheelMax, float out[4]);

} // namespace MecanumKinematics
