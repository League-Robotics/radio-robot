// interpolation.h — lerp helpers a reading type's sampleAt() composes to
// interpolate between the two Sample<T>s a MeasurementRing<T>::bracket()
// call returns.
//
// Ticket DB-002 (device-bus-tickets.md). Part of the greenfield
// `source/devices/` subsystem (namespace `Devices`) described in
// clasi/issues/device-bus-fiber-owned-self-contained-device-subsystem.md's
// "Measurement rings" section: "sampleAt(t) brackets a past instant and
// linearly interpolates ... Each reading type supplies its own lerp; OTOS
// heading needs wrap-aware angular lerp — naive linear interpolation across
// ±180° is a known trap."
//
// This ticket supplies the two primitives every later sampleAt()
// implementation (DB-007's DeviceBus handle classes are where a reading
// type's own sampleAt() will pick, per field, which helper applies) is
// built from:
//   - lerpFraction()/lerp() — ordinary linear interpolation, for every
//     non-angular field (MotorReading::position/velocity, PoseReading::
//     x/y/v_x/v_y/omega — device_types.h).
//   - wrapAngle()/lerpAngle() — wrap-aware angular interpolation, for
//     PoseReading::heading specifically. otos.h's own "Wrap-aware heading"
//     note establishes every heading this subsystem stores is already in
//     (-pi, pi] range (the OTOS chip's own int16 HEADING register wraps at
//     exactly that point) — lerpAngle() assumes that same range on input
//     and preserves it on output.
//
// Deliberately generic (float in, float out), not reading-type-aware: which
// helper applies to which struct field is a decision that belongs to the
// reading type's own sampleAt(), not to this file — device_types.h stays
// free of any dependency on this header, and this header stays free of any
// dependency on device_types.h.
//
// Pure host-clean C++: <cmath> (atan2/sin/cos) and <cstdint> (the
// microsecond stamp width lerpFraction() takes) only.
#pragma once

#include <cmath>
#include <cstdint>

namespace Devices {

// lerpFraction — the dimensionless [0,1] position of `t` between
// `olderStamp` and `newerStamp`. Clamped to [0,1] if `t` falls outside
// [olderStamp, newerStamp] — defensive only: a `t` produced by a successful
// MeasurementRing<T>::bracket() call already satisfies olderStamp <= t <=
// newerStamp by construction, so the clamp is belt-and-suspenders, not the
// normal path. `newerStamp <= olderStamp` (degenerate/equal stamps) returns
// 0 — snaps to the older sample instead of dividing by zero.
inline float lerpFraction(uint64_t olderStamp, uint64_t newerStamp,
                           uint64_t t) {  // [us] [us] [us] -> dimensionless [0,1]
  if (newerStamp <= olderStamp) return 0.0f;
  if (t <= olderStamp) return 0.0f;
  if (t >= newerStamp) return 1.0f;
  return static_cast<float>(t - olderStamp) /
         static_cast<float>(newerStamp - olderStamp);
}

// lerp — ordinary linear interpolation between two scalar/position values.
// frac is the dimensionless [0,1] fraction lerpFraction() computes: frac==0
// returns `older`, frac==1 returns `newer`.
inline float lerp(float older, float newer, float frac) {
  return older + (newer - older) * frac;
}

// wrapAngle — wrap `angle` into (-pi, pi], via the same atan2(sin, cos)
// identity source/subsystems/pose_estimator.cpp's wrapPi() uses. Reimplemented
// locally rather than shared: the isolation invariant forbids
// source/devices/ from including it (device-bus-tickets.md's
// "Standing isolation invariant").
inline float wrapAngle(float angle) {  // [rad] -> [rad] in (-pi, pi]
  return std::atan2(std::sin(angle), std::cos(angle));
}

// lerpAngle — wrap-aware angular interpolation. Radians in, radians out,
// (-pi, pi] both ways. This is the issue's flagged trap: naive
// lerp(older, newer, frac) across the ±pi seam goes the LONG way around
// (170°->-170° naively interpolated toward the midpoint drifts DOWN through
// 0°, not up through ±180° — the wrong direction and 9x the true angular
// distance). Instead this takes the SHORTEST signed angular delta from
// older to newer — wrapAngle(newer - older), itself always in (-pi, pi] —
// and steps `frac` of THAT delta from `older`, re-wrapping the result. For
// the issue's own worked example (170°->-170°), the shortest delta is +20°
// (continuing past +180° into -170°, not the 340° the long way would
// cover), so the frac=0.5 midpoint lands at ~180° (the short way), not 0°.
inline float lerpAngle(float older, float newer,
                        float frac) {  // [rad] [rad] dimensionless -> [rad]
  const float delta = wrapAngle(newer - older);
  return wrapAngle(older + delta * frac);
}

}  // namespace Devices
