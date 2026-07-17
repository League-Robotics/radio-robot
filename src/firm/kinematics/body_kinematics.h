#pragma once
#include "messages/common.h"

/**
 * BodyKinematics — stateless differential-drive kinematic maps and saturation.
 *
 * All functions are pure: no I2C, no global state, no heap allocation.
 *
 * See DESIGN.md (this directory) for the subsystem contract and
 * docs/kinematics-model.md §1.3/§1.7 for the math this implements.
 */
namespace BodyKinematics {

/**
 * inverse — map body twist (v, omega) to wheel speeds (vL, vR).
 *
 * Equations (§1.3):
 *   vL = v - omega * (b / 2)
 *   vR = v + omega * (b / 2)
 *
 * @param v       body forward speed, mm/s
 * @param omega   body yaw rate, rad/s (CCW-positive)
 * @param b       track width, mm
 * @param vL_out  left wheel speed output, mm/s
 * @param vR_out  right wheel speed output, mm/s
 */
void inverse(float v, float omega, float b, float& vL_out, float& vR_out);

/**
 * forward — map wheel speeds (vL, vR) to body twist (v, omega).
 *
 * Equations (§1.3):
 *   v     = (vR + vL) / 2
 *   omega = (vR - vL) / b
 *
 * @param vL       left wheel speed, mm/s
 * @param vR       right wheel speed, mm/s
 * @param b        track width, mm
 * @param v_out    body forward speed output, mm/s
 * @param omega_out body yaw rate output, rad/s (CCW-positive)
 */
void forward(float vL, float vR, float b, float& v_out, float& omega_out);

/**
 * saturate — curvature-preserving wheel speed saturation (§1.7).
 *
 * Effective ceiling: ceiling = vWheelMax - steerHeadroom
 *
 * When max(|vL|, |vR|) > ceiling, both wheel speeds are scaled by:
 *   s = ceiling / max(|vL|, |vR|)
 * so the faster wheel sits exactly at the ceiling and the wheel-speed
 * ratio (and therefore arc curvature) is preserved.
 *
 * If max(|vL|, |vR|) <= ceiling, outputs equal inputs (pass-through).
 *
 * @param vL            left wheel speed input, mm/s
 * @param vR            right wheel speed input, mm/s
 * @param vWheelMax     absolute wheel speed ceiling, mm/s (must be > 0)
 * @param steerHeadroom headroom below vWheelMax for steering authority, mm/s
 * @param vL_out        scaled left wheel speed output, mm/s
 * @param vR_out        scaled right wheel speed output, mm/s
 */
void saturate(float vL, float vR,
              float vWheelMax, float steerHeadroom,
              float& vL_out, float& vR_out);

/**
 * Array-form overloads — wheels[2] = {vL, vR} (same sign convention as the
 * scalar forms above). See DESIGN.md for why this API shape exists.
 * v_y is always 0 for a differential drivetrain; inverse ignores t.v_y and
 * forward sets t_out.v_y = 0.
 *
 * @param t          body twist input (v_y ignored)
 * @param b          track width, mm
 * @param wheels     wheel speed array [vL, vR], mm/s (in/out)
 */
void inverse(msg::BodyTwist3 t, float b, float wheels[2]);
void forward(const float wheels[2], float b, msg::BodyTwist3& t_out);

/**
 * Array-form saturate — uniform scale when any |wheel| > (vWheelMax - steerHeadroom).
 *
 * @param wheels        wheel speed array [vL, vR] (in/out, modified in-place)
 * @param vWheelMax     absolute wheel speed ceiling, mm/s (must be > 0)
 * @param steerHeadroom headroom below vWheelMax for steering authority, mm/s
 * @param out           scaled wheel speed output [vL, vR], mm/s
 */
void saturate(float wheels[2], float vWheelMax, float steerHeadroom, float out[2]);

} // namespace BodyKinematics
