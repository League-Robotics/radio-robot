#include "profile.h"

#include <algorithm>
#include <cmath>

namespace Motion {

namespace {
// Slack for the closing-step reachability comparison only -- absorbs float
// rounding in remaining/dt vs. the accel/decel window edges. Never part of
// the feasibility accounting itself.
constexpr float kTiny = 1e-6f;
}  // namespace

float brakeDistance(float velocity, float boundary, float decel, float dt) {
  if (decel <= 0.0f || dt <= 0.0f) return 0.0f;
  const float delta = decel * dt;  // per-interval velocity step
  if (velocity <= boundary + delta) return 0.0f;  // one allowed step away
  // Steps strictly above boundary: after m steps we sit within (boundary,
  // boundary+delta], from where one more allowed step reaches boundary.
  const int m =
      static_cast<int>(std::ceil((velocity - boundary) / delta)) - 1;
  const float mf = static_cast<float>(m);
  return (mf * velocity - delta * (mf * (mf + 1.0f) * 0.5f)) * dt;
}

float planDecel(const AxisLimits& limits) {
  if (limits.aDecelPlan <= 0.0f) return limits.aDecel;
  return std::min(limits.aDecelPlan, limits.aDecel);
}

float maxEntryVelocity(float distance, float boundary, const AxisLimits& limits,
                       float dt) {
  if (distance <= 0.0f || dt <= 0.0f) return 0.0f;
  const float decelPlan = planDecel(limits);
  auto fits = [&](float v) {
    return v * dt + brakeDistance(v, boundary, decelPlan, dt) <= distance;
  };
  float hi = limits.vMax;
  if (fits(hi)) return hi;
  float lo = 0.0f;
  for (int i = 0; i < 48; ++i) {
    const float mid = 0.5f * (lo + hi);
    (fits(mid) ? lo : hi) = mid;
  }
  return lo;
}

ProfileResult profileStep(float remaining, float previous, float cruise,
                          float boundary, const AxisLimits& limits, float dt,
                          float previousAccel) {
  const float r = std::max(0.0f, remaining);
  // Accel authority this step. Plain trapezoid: aMax. With a jerk ceiling:
  // accel may grow from the previous step's accel by at most jMax*dt, and
  // is tapered to hit zero exactly at cruise (v(a=0) matches cruise when
  // a = sqrt(2*jMax*(cruise - v))) -- the S-curve's two accel corners
  // rounded, no instant jump into or out of the ramp.
  float accelAllow = limits.aMax;
  if (limits.jMax > 0.0f) {
    const float grown = std::max(0.0f, previousAccel) + limits.jMax * dt;
    const float headroom = std::max(0.0f, std::min(cruise, limits.vMax) -
                                              previous);
    const float taper = std::sqrt(2.0f * limits.jMax * headroom);
    accelAllow = std::min(limits.aMax, std::min(grown, taper));
  }
  const float cruiseCap = std::min(cruise, limits.vMax);
  float ceiling = std::min(cruiseCap, previous + accelAllow * dt);
  const float floor = std::max(0.0f, previous - limits.aDecel * dt);
  // Cruise below the reachable window: brake toward it. That is a decel
  // regime even though the feasibility test below will accept the ceiling.
  const bool brakingToCruise = ceiling < floor;
  if (brakingToCruise) ceiling = floor;

  // Exact terminal step: one interval at remaining/dt closes the sum to
  // `remaining` exactly, when that velocity is inside the reachable window
  // AND within one decel step of the boundary -- the command AFTER landing
  // is `boundary`, so that hand-off must itself respect the decel limit
  // (a zero-boundary landing then stops dead, with no post-completion
  // drift adding unaccounted distance).
  const float landing = r / dt;
  if (landing <= ceiling + kTiny && landing >= floor - kTiny &&
      landing <= boundary + limits.aDecel * dt + kTiny) {
    return {std::clamp(landing, 0.0f, ceiling), true, StepPhase::Closing};
  }

  // Brake-START decision rides aDecelPlan (<= aDecel); the floor above and
  // the landing test just made both keep the full aDecel authority.
  const float decelPlan = planDecel(limits);
  auto feasible = [&](float v) {
    return r - v * dt >= brakeDistance(v, boundary, decelPlan, dt);
  };
  if (feasible(ceiling)) {
    const StepPhase phase =
        brakingToCruise
            ? StepPhase::Decel
            : ((ceiling >= cruiseCap - kTiny && previous >= cruiseCap - kTiny)
                   ? StepPhase::Hold
                   : StepPhase::Accel);
    return {ceiling, false, phase};
  }
  // The floor choice is always feasible when last tick's choice was (the
  // staircase from v-delta is exactly the tail of the staircase from v);
  // if we were handed an infeasible state (e.g. a replace at speed into a
  // short Move), brake as hard as allowed and let re-measurement recover.
  if (!feasible(floor)) return {floor, false, StepPhase::Decel};
  float lo = floor;
  float hi = ceiling;
  for (int i = 0; i < 48; ++i) {
    const float mid = 0.5f * (lo + hi);
    (feasible(mid) ? lo : hi) = mid;
  }
  return {lo, false, StepPhase::Decel};
}

}  // namespace Motion
