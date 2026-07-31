#include "shape.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Motion {

namespace {

// Below this a commanded wheel speed is indistinguishable from "not
// commanded" and the Move cannot be shaped.
constexpr float kMinSpeed = 1e-3f;  // [mm/s]

// Ratio-identity tolerance for the lookahead compatibility test. Generous
// relative to float noise (a unit is in [-1,1]) and tight relative to any
// ratio difference a host would deliberately command.
constexpr float kShapeTol = 1e-3f;  // [1]

bool finite(float v) { return std::isfinite(v); }

}  // namespace

MoveShape shapeOf(const Move& move, float trackWidth) {
  MoveShape out;

  // A planned stop has no shape: it is a forced ramp to zero, and the
  // profiler never sees it. Leaving valid = false also makes it an
  // automatic lookahead barrier -- nothing hands off at speed INTO a stop.
  if (move.kind == Move::Kind::Stop) return out;

  if (!finite(trackWidth) || trackWidth <= 0.0f) return out;

  // 1. Decompose to a wheel-velocity pair.
  float vLeft = 0.0f;   // [mm/s]
  float vRight = 0.0f;  // [mm/s]
  if (move.velocityKind == Move::VelocityKind::Wheels) {
    vLeft = move.vLeft;
    vRight = move.vRight;
  } else {
    const float half = 0.5f * trackWidth * move.omega;  // [mm/s]
    vLeft = move.v_x - half;
    vRight = move.v_x + half;
  }
  if (!finite(vLeft) || !finite(vRight)) return out;

  // 2. Normalize by the dominant wheel.
  const float peak = std::max(std::fabs(vLeft), std::fabs(vRight));  // [mm/s]
  if (peak < kMinSpeed) return out;  // commanded to stand still
  out.unitLeft = vLeft / peak;
  out.unitRight = vRight / peak;
  out.cruise = peak;

  // Snap the dominant wheel to exactly +-1: the division above is exact for
  // the wheel whose magnitude IS the peak, but say so structurally rather
  // than relying on it, because the ratio lock's tie-break depends on the
  // dominant unit being a literal 1.0f.
  if (std::fabs(vLeft) >= std::fabs(vRight)) {
    out.unitLeft = std::copysign(1.0f, vLeft);
  } else {
    out.unitRight = std::copysign(1.0f, vRight);
  }

  // 3. Duration at cruise, from the stop condition. threshold is a positive
  // magnitude; direction lives in the velocity sign.
  const float threshold = std::fabs(move.threshold);
  switch (move.kind) {
    case Move::Kind::Time: {
      out.duration = threshold * 0.001f;  // [ms] -> [s]
      out.hasTimeBudget = true;
      out.valid = out.duration > 0.0f;
      return out;  // no distance target: a Time Move ends on the clock
    }
    case Move::Kind::Distance: {
      // Net translation is the MEAN wheel speed. A pure pivot has none, so
      // a distance stop can never be reached -- reject rather than run
      // forever.
      const float mean = std::fabs(0.5f * (vLeft + vRight));  // [mm/s]
      if (mean < kMinSpeed) return out;
      out.duration = threshold / mean;  // [s]
      break;
    }
    case Move::Kind::Angle: {
      // Net rotation is the wheel DIFFERENCE over the track. A straight has
      // none, so an angle stop can never be reached.
      const float diff = std::fabs(vRight - vLeft);  // [mm/s]
      if (diff < kMinSpeed) return out;
      out.duration = threshold * trackWidth / diff;  // [s]
      break;
    }
    case Move::Kind::Stop:
      return out;  // unreachable; handled above
  }

  if (!finite(out.duration) || out.duration <= 0.0f) return out;

  // 4. Per-wheel travel at cruise over that duration.
  out.distanceLeft = vLeft * out.duration;    // [mm]
  out.distanceRight = vRight * out.duration;  // [mm]
  out.hasDistanceTarget = true;
  out.valid = true;
  return out;
}

AxisLimits shapeLimits(const MoveShape& shape, const PlannerLimits& limits) {
  AxisLimits out;
  if (!shape.valid) return out;  // fail closed: all-zero ceilings

  // How much body translation / body rotation one unit of lambda produces.
  // A straight has mean 1, diff 0 (linear ceilings only); a pivot has mean
  // 0, diff 2 (angular ceilings only, scaled by half the track); an arc has
  // both, and whichever binds first wins.
  const float mean = std::fabs(0.5f * (shape.unitLeft + shape.unitRight));
  const float diff = std::fabs(shape.unitRight - shape.unitLeft);

  // Start UNSET (0 = "no bound from any contributing axis") rather than
  // seeded with the linear ceilings: seeding would silently impose the
  // LINEAR accel ceiling on a pure pivot, which the pre-existing per-Kind
  // cases never did. A 0 ceiling reaching profileStep() means unconfigured,
  // which is the correct fail-closed answer for a shape no axis bounds.
  float vMax = 0.0f;
  float aMax = 0.0f;
  float aDecel = 0.0f;
  float jMax = 0.0f;

  auto tighten = [](float current, float candidate) {
    if (candidate <= 0.0f) return current;  // unconfigured: contributes nothing
    if (current <= 0.0f) return candidate;
    return std::min(current, candidate);
  };

  if (mean > 0.0f) {
    // lambda * mean is the body's forward speed, so a body-frame linear
    // ceiling C bounds lambda at C/mean.
    vMax = tighten(vMax, limits.vMax / mean);
    aMax = tighten(aMax, limits.aMax / mean);
    aDecel = tighten(aDecel, limits.aDecel / mean);
    jMax = tighten(jMax, limits.jerkMax / mean);
  }
  if (diff > 0.0f) {
    // lambda * diff / trackWidth is the body's yaw rate, so an angular
    // ceiling C bounds lambda at C * trackWidth / diff. For a pivot
    // (diff == 2) that is exactly C * halfTrack -- the same half-track
    // scaling the per-Kind Angle case applied after profiling.
    vMax = tighten(vMax, limits.omegaMax * limits.trackWidth / diff);
    aMax = tighten(aMax, limits.alphaMax * limits.trackWidth / diff);
    aDecel = tighten(aDecel, limits.alphaDecel * limits.trackWidth / diff);
    jMax = tighten(jMax, limits.yawJerkMax * limits.trackWidth / diff);
  }

  // lambda IS the dominant wheel's speed, so no shape may ever plan a wheel
  // past the wheel-speed ceiling however the axis arithmetic came out. (On
  // both shipped configs this never binds -- omegaMax*halfTrack is 192 mm/s
  // on the robot and 400 mm/s in sim, against a 400/600 vMax -- so it
  // changes no current behavior; it is a guard, not a retune.)
  if (limits.vMax > 0.0f) vMax = tighten(vMax, limits.vMax);

  out.vMax = vMax;
  out.aMax = aMax;
  out.aDecel = aDecel;
  out.jMax = jMax;
  // Decel leeway: plan the brake START against a fraction of the ceiling,
  // keeping the rest in reserve for a plant that cannot follow a
  // full-authority ramp down. 0 (and 1) mean "plan at full authority".
  //
  // SELF-LIMITING PER MOVE. A leeway the Move cannot afford is worse than
  // no leeway at all: if the planned decel cannot stop `cruise` within the
  // Move's own distance, profileStep() finds itself infeasible on the very
  // FIRST tick and brakes at full authority forever -- the Move never
  // accelerates, and a tour built from such Moves collapses (measured: at
  // fraction 0.2 a square tour completed 3 of 4 turns and finished 467 mm
  // out). So floor the planned decel at what this Move actually needs,
  // with margin for the discrete steps and the jerk-limited entry. Long
  // Moves get the full requested leeway; short ones get only what they can
  // pay for, and the knob degrades smoothly instead of falling off a cliff.
  out.aDecelPlan = 0.0f;
  if (limits.decelPlanFraction > 0.0f && limits.decelPlanFraction < 1.0f &&
      aDecel > 0.0f) {
    float planned = aDecel * limits.decelPlanFraction;
    if (shape.hasDistanceTarget) {
      const float dominant = std::max(std::fabs(shape.distanceLeft),
                                      std::fabs(shape.distanceRight));
      if (dominant > 0.0f) {
        constexpr float kFeasibilityMargin = 2.0f;
        const float needed =
            kFeasibilityMargin * shape.cruise * shape.cruise / (2.0f * dominant);
        planned = std::max(planned, needed);
      }
    }
    out.aDecelPlan = std::min(planned, aDecel);
  }
  return out;
}

bool shapesCompatible(const MoveShape& current, const MoveShape& next) {
  if (!current.valid || !next.valid) return false;
  return std::fabs(current.unitLeft - next.unitLeft) <= kShapeTol &&
         std::fabs(current.unitRight - next.unitRight) <= kShapeTol;
}

bool shapeDirectionsAgree(const MoveShape& current, const MoveShape& next) {
  if (!current.valid || !next.valid) return false;
  const float currentUnit[2] = {current.unitLeft, current.unitRight};
  const float nextUnit[2] = {next.unitLeft, next.unitRight};
  for (int w = 0; w < 2; ++w) {
    // Negligible in either shape: nothing to conflict with.
    if (std::fabs(currentUnit[w]) <= kShapeTol ||
        std::fabs(nextUnit[w]) <= kShapeTol) {
      continue;
    }
    if ((currentUnit[w] > 0.0f) != (nextUnit[w] > 0.0f)) return false;
  }
  return true;
}

float curvatureHandoffLambdaCap(const MoveShape& current, const MoveShape& next,
                                float wheelDecelCeiling, float dt,
                                int blendCycles) {
  constexpr float kNoCap = std::numeric_limits<float>::infinity();
  if (!current.valid || !next.valid || dt <= 0.0f || blendCycles <= 0) {
    return 0.0f;
  }
  // Fail closed: no declared per-wheel decel authority to absorb a
  // differential hand-off with, same posture as boundaryLambda()'s own
  // aDecel <= 0 guard.
  if (wheelDecelCeiling <= 0.0f) return 0.0f;

  const float currentUnit[2] = {current.unitLeft, current.unitRight};
  const float nextUnit[2] = {next.unitLeft, next.unitRight};
  const float budget =
      wheelDecelCeiling * dt * static_cast<float>(blendCycles);  // [mm/s]
  float cap = kNoCap;
  for (int w = 0; w < 2; ++w) {
    const float delta = std::fabs(nextUnit[w] - currentUnit[w]);
    if (delta <= kShapeTol) continue;  // this wheel's ratio does not change
    cap = std::min(cap, budget / delta);
  }
  return cap;
}

}  // namespace Motion
