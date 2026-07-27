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

#include <cstdint>

namespace Motion {

// One axis's ceilings, positive frame. Units are [mm/s]/[mm/s^2] for the
// linear axis, [rad/s]/[rad/s^2] for the angular axis.
struct AxisLimits {
  float vMax = 0.0f;
  float aMax = 0.0f;
  float aDecel = 0.0f;
  // Jerk ceiling (0 = disabled, plain trapezoid). When set, the ACCEL
  // side becomes an S-curve: per-step acceleration may grow by at most
  // jMax*dt, and is tapered as sqrt(2*jMax*(cruise - v)) approaching
  // cruise so it reaches zero exactly at cruise -- no instant accel
  // corner. The BRAKING side deliberately keeps its instant-aDecel
  // authority: the discrete feasibility/braking accounting stays exact
  // and the landing guarantee is untouched.
  float jMax = 0.0f;
  // Decel used for the brake-START decision only -- the feasibility
  // accounting that answers "may we hold this velocity one more interval?".
  // 0 means "same as aDecel", so every pre-existing brace-init behaves
  // exactly as before; a value above aDecel is clamped down to it, so the
  // plan can never promise more than the authority.
  //
  // Setting it BELOW aDecel is the leeway: the profile commits to braking
  // sooner and rides a gentler ramp, holding (aDecel - aDecelPlan) in
  // reserve for the plant to fall behind into. The terminal closing step
  // and the per-step floor still use the FULL aDecel, so the landing stays
  // discrete-exact and a state that HAS gone infeasible still brakes at
  // full authority. This is decel-AUTHORITY headroom, never
  // command-authority headroom: the profile always commands the wheel all
  // the way down to the boundary and never stops commanding early to let
  // friction finish -- plant tau ~230 ms, so coasting overshoots.
  float aDecelPlan = 0.0f;
};

// Which regime produced a step. The planner folds both wheels' phases into
// the Move's own MovePhase; the velocity trim gates its integrator on it.
enum class StepPhase : uint8_t {
  Accel,    // commanded velocity is climbing toward cruise
  Hold,     // sitting at the cruise ceiling, unchanged
  Decel,    // braking -- either to land, or down toward a lower cruise
  Closing,  // the exact terminal step; lands remaining at zero
};

struct ProfileResult {
  float velocity = 0.0f;  // positive-frame command for the next interval
  bool closing = false;   // true: this command lands remaining exactly at zero
  StepPhase phase = StepPhase::Accel;
};

// The decel the brake-START decision plans against: aDecelPlan when set,
// else aDecel, never more than aDecel. Exposed because the lookahead has to
// ask the same feasibility question profileStep() asks internally.
float planDecel(const AxisLimits& limits);

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
// `previousAccel` is the accel the caller's LAST emitted step implied
// ((v_k - v_{k-1})/dt, signed, positive frame); only consulted when
// limits.jMax > 0.
ProfileResult profileStep(float remaining, float previous, float cruise,
                          float boundary, const AxisLimits& limits,
                          float dt, float previousAccel = 0.0f);  // [s] [mm/s^2]

}  // namespace Motion
