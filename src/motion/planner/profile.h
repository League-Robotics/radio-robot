// profile.h -- the discrete-exact trapezoid velocity policy (motion-planner
// sketch §3). Pure free functions, one axis at a time, all in the POSITIVE
// frame (the caller normalizes by the cruise sign and re-applies it).
//
// The model: the planner emits one velocity per control interval, held ZOH
// for the whole interval, so distance advanced in a perfect plant is exactly
// velocity*dt. A Move of distance D is exact iff the emitted sequence sums
// to D -- an accounting problem. Feasibility ("can we still land at the
// boundary velocity within the remaining distance?") is computed as the
// exact discrete braking staircase sum, NOT the continuous v^2/2a formula --
// this is what replaces the swept land-at-zero margin constants.
#pragma once

namespace Motion {

// One axis's ceilings, positive frame. Units are [mm/s]/[mm/s^2] for the
// linear axis, [rad/s]/[rad/s^2] for the angular axis.
struct AxisLimits {
  float vMax = 0.0f;
  float aMax = 0.0f;
  float aDecel = 0.0f;
};

struct ProfileResult {
  float velocity = 0.0f;  // positive-frame command for the next interval
  bool closing = false;   // true: this command lands remaining exactly at zero
};

// Minimum distance consumed by the max-decel staircase that takes the
// commanded velocity from `velocity` down to within one decel step of
// `boundary` (from where a single allowed step reaches `boundary` itself).
// Exact discrete sum: velocities velocity-i*decel*dt for the steps strictly
// above boundary, each held for dt. Returns 0 when already within one step.
float brakeDistance(float velocity, float boundary, float decel, float dt);  // [s]

// Largest velocity from which `distance` still suffices to reach `boundary`
// under `limits` (one interval at that velocity plus its braking staircase).
// Used for lookahead: how fast may we hand off into the NEXT Move without
// making ITS landing infeasible.
float maxEntryVelocity(float distance, float boundary, const AxisLimits& limits,
                       float dt);  // [s]

// One policy step. `remaining` is distance-to-go (>= 0), `previous` the
// last commanded velocity, `cruise` the Move's cruise ceiling, `boundary`
// the velocity to land at (0, or the next Move's carried entry velocity).
// Picks the largest velocity that keeps landing feasible; emits the exact
// terminal step (remaining/dt) when it is reachable within the accel/decel
// window, closing the sum to `remaining` exactly.
ProfileResult profileStep(float remaining, float previous, float cruise,
                          float boundary, const AxisLimits& limits,
                          float dt);  // [s]

}  // namespace Motion
