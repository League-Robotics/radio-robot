// profile_test.cpp -- unit tests for the discrete-exact trapezoid policy
// (profile.h): brakeDistance staircase values, the feasibility invariant
// under a pure-profile simulation sweep, exact terminal landing, and limit
// respect at every step.
#include <algorithm>
#include <cmath>
#include <cstdio>

#include "profile.h"
#include "tests/test_support.h"

using Motion::AxisLimits;
using Motion::brakeDistance;
using Motion::maxEntryVelocity;
using Motion::ProfileResult;
using Motion::profileStep;

namespace {

constexpr float kDt = 0.05f;  // [s]

// Pure-profile closed loop: perfect accounting, r -= v*dt. Returns ticks
// used; asserts limits at every step and exact landing.
int runProfile(double distance, float cruise, float boundary,
               const AxisLimits& limits, float entryVelocity) {
  double remaining = distance;
  float previous = entryVelocity;
  int ticks = 0;
  bool closed = false;
  while (ticks < 100000) {
    const ProfileResult step = profileStep(static_cast<float>(remaining),
                                           previous, cruise, boundary,
                                           limits, kDt);
    // Limits hold at every step (small slack for float rounding).
    CHECK(step.velocity <= limits.vMax + 1e-3f);
    CHECK(step.velocity <= previous + limits.aMax * kDt + 1e-3f);
    CHECK(step.velocity >= previous - limits.aDecel * kDt - 1e-3f);
    remaining -= static_cast<double>(step.velocity) * kDt;
    previous = step.velocity;
    ++ticks;
    if (step.closing) {
      closed = true;
      break;
    }
  }
  CHECK(closed);
  // Exact landing: the closing step consumed remaining exactly, to the
  // float rounding floor (all planner math is float, firmware-typed;
  // 1e-4 mm = 0.1 um. The end-to-end gate is the scenario tests' 1e-3 mm).
  CHECK_NEAR(remaining, 0.0, 1e-4);
  return ticks;
}

void testBrakeDistanceValues() {
  // One allowed step away: no staircase.
  CHECK_NEAR(brakeDistance(10.0f, 0.0f, 300.0f, kDt), 0.0, 1e-6);
  CHECK_NEAR(brakeDistance(15.0f, 0.0f, 300.0f, kDt), 0.0, 1e-6);
  // v=30, delta=15: one step at 15 -> 0.75 mm.
  CHECK_NEAR(brakeDistance(30.0f, 0.0f, 300.0f, kDt), 15.0 * 0.05, 1e-4);
  // v=150, delta=15: steps 135,120,...,15 -> sum = (135+15)*9/2 * dt.
  CHECK_NEAR(brakeDistance(150.0f, 0.0f, 300.0f, kDt), 675.0 * 0.05, 1e-3);
  // Nonzero boundary: v=150 -> 100, delta=15: ceil(50/15)=4 -> m=3:
  // steps 135,120,105 -> 360*dt.
  CHECK_NEAR(brakeDistance(150.0f, 100.0f, 300.0f, kDt), 360.0 * 0.05, 1e-3);
  // Already at/below boundary.
  CHECK_NEAR(brakeDistance(100.0f, 100.0f, 300.0f, kDt), 0.0, 1e-6);
  CHECK_NEAR(brakeDistance(50.0f, 100.0f, 300.0f, kDt), 0.0, 1e-6);
}

void testMaxEntryVelocity() {
  const AxisLimits lim{600.0f, 400.0f, 300.0f};
  // A long runway admits full speed.
  const float vLong = maxEntryVelocity(10000.0f, 0.0f, lim, kDt);
  CHECK_NEAR(vLong, 600.0f, 1e-3);
  // A short runway admits less; entering at the returned velocity must be
  // brakeable within the distance.
  const float vShort = maxEntryVelocity(20.0f, 0.0f, lim, kDt);
  CHECK(vShort < 600.0f);
  CHECK(vShort * kDt + brakeDistance(vShort, 0.0f, lim.aDecel, kDt) <=
        20.0f + 1e-3f);
  // Zero runway admits nothing.
  CHECK_NEAR(maxEntryVelocity(0.0f, 0.0f, lim, kDt), 0.0, 1e-6);
}

void testSweepExactLanding() {
  // Broad sweep: distances x cruises x limit sets x boundaries, from rest.
  const float distances[] = {5.0f, 37.0f, 100.0f, 333.3f, 500.0f, 2000.0f};
  const float cruises[] = {60.0f, 150.0f, 400.0f, 600.0f};
  const AxisLimits limitSets[] = {
      {600.0f, 400.0f, 300.0f},
      {600.0f, 200.0f, 500.0f},
      {600.0f, 1000.0f, 1000.0f},
  };
  for (const AxisLimits& lim : limitSets) {
    for (float d : distances) {
      for (float cruise : cruises) {
        runProfile(d, cruise, 0.0f, lim, 0.0f);
      }
    }
  }
}

void testShortMoveFromSpeed() {
  // Entering a short Move already at speed (replace-at-speed shape): exact
  // landing is physically impossible -- the policy must brake at the full
  // decel ceiling every tick and overshoot by no more than the entry
  // velocity's own braking distance.
  const AxisLimits lim{600.0f, 400.0f, 300.0f};
  const float entry = 300.0f;
  double remaining = 30.0;
  float previous = entry;
  for (int i = 0; i < 100 && previous > 0.0f; ++i) {
    const ProfileResult step = profileStep(static_cast<float>(remaining),
                                           previous, entry, 0.0f, lim, kDt);
    // Max braking, every tick, until stopped.
    CHECK_NEAR(step.velocity,
               std::max(0.0f, previous - lim.aDecel * kDt), 1e-3);
    remaining -= static_cast<double>(step.velocity) * kDt;
    previous = step.velocity;
  }
  CHECK(previous == 0.0f);
  const double overshoot = -remaining;
  CHECK(overshoot >= 0.0);
  CHECK(overshoot <=
        entry * kDt + brakeDistance(entry, 0.0f, lim.aDecel, kDt));
}

void testCruiseHold() {
  // Long move: velocity must actually reach and hold cruise (the policy is
  // max-feasible, not just any-feasible).
  const AxisLimits lim{600.0f, 400.0f, 300.0f};
  double remaining = 5000.0;
  float previous = 0.0f;
  int atCruise = 0;
  for (int i = 0; i < 1000 && remaining > 0.0; ++i) {
    const ProfileResult step = profileStep(static_cast<float>(remaining),
                                           previous, 150.0f, 0.0f, lim, kDt);
    if (std::fabs(step.velocity - 150.0f) < 1e-3f) ++atCruise;
    remaining -= static_cast<double>(step.velocity) * kDt;
    previous = step.velocity;
    if (step.closing) break;
  }
  CHECK(atCruise > 500);  // the vast majority of a 5 m run cruises
}

void testPhaseReported() {
  // A long move reports Accel* Hold* Decel* Closing, in that order and with
  // no regression: the phase is what the velocity trim gates its integrator
  // on, so a stray Hold inside a ramp would wind the integrator up.
  const AxisLimits lim{600.0f, 400.0f, 300.0f};
  double remaining = 5000.0;
  float previous = 0.0f;
  int accel = 0, hold = 0, decel = 0, closing = 0;
  int lastRank = -1;
  auto rank = [](Motion::StepPhase p) {
    switch (p) {
      case Motion::StepPhase::Accel: return 0;
      case Motion::StepPhase::Hold: return 1;
      case Motion::StepPhase::Decel: return 2;
      case Motion::StepPhase::Closing: return 3;
    }
    return 0;
  };
  for (int i = 0; i < 1000; ++i) {
    const ProfileResult step = profileStep(static_cast<float>(remaining),
                                           previous, 150.0f, 0.0f, lim, kDt);
    const int r = rank(step.phase);
    CHECK(r >= lastRank);  // monotone: never back to Accel/Hold after Decel
    lastRank = r;
    switch (step.phase) {
      case Motion::StepPhase::Accel: ++accel; break;
      case Motion::StepPhase::Hold: ++hold; break;
      case Motion::StepPhase::Decel: ++decel; break;
      case Motion::StepPhase::Closing: ++closing; break;
    }
    // Hold means the command genuinely is not changing.
    if (step.phase == Motion::StepPhase::Hold) {
      CHECK_NEAR(step.velocity, previous, 1e-3);
    }
    remaining -= static_cast<double>(step.velocity) * kDt;
    previous = step.velocity;
    if (step.closing) break;
  }
  CHECK(accel > 0);
  CHECK(hold > 0);
  CHECK(decel > 0);
  CHECK(closing == 1);

  // A short move never cruises: Accel/Decel only, still landing exactly.
  remaining = 20.0;
  previous = 0.0f;
  hold = 0;
  for (int i = 0; i < 1000; ++i) {
    const ProfileResult step = profileStep(static_cast<float>(remaining),
                                           previous, 600.0f, 0.0f, lim, kDt);
    if (step.phase == Motion::StepPhase::Hold) ++hold;
    remaining -= static_cast<double>(step.velocity) * kDt;
    previous = step.velocity;
    if (step.closing) break;
  }
  CHECK(hold == 0);
  CHECK_NEAR(remaining, 0.0, 1e-4);
}

void testDecelLeewayStartsBrakingEarlier() {
  // aDecelPlan below aDecel must (a) leave landing exactness untouched,
  // (b) begin braking no later than the full-authority profile, and
  // (c) keep the reserve: the emitted ramp never uses more than the PLAN
  // decel while it is riding the taper down.
  AxisLimits tight{600.0f, 400.0f, 300.0f};
  AxisLimits leeway = tight;
  leeway.aDecelPlan = 200.0f;  // [mm/s^2] hold 100 in reserve

  auto firstBrakeTick = [&](const AxisLimits& lim) {
    double remaining = 2000.0;
    float previous = 0.0f;
    for (int i = 0; i < 10000; ++i) {
      const ProfileResult step = profileStep(static_cast<float>(remaining),
                                             previous, 400.0f, 0.0f, lim, kDt);
      if (step.phase == Motion::StepPhase::Decel ||
          step.phase == Motion::StepPhase::Closing) {
        return i;
      }
      remaining -= static_cast<double>(step.velocity) * kDt;
      previous = step.velocity;
    }
    return -1;
  };
  const int tightTick = firstBrakeTick(tight);
  const int leewayTick = firstBrakeTick(leeway);
  CHECK(tightTick > 0);
  CHECK(leewayTick > 0);
  CHECK(leewayTick < tightTick);  // braking starts strictly earlier

  // Landing stays exact, and the ramp respects the plan decel while
  // braking (the full aDecel stays available but is not spent).
  double remaining = 2000.0;
  float previous = 0.0f;
  bool closed = false;
  for (int i = 0; i < 10000; ++i) {
    const ProfileResult step = profileStep(static_cast<float>(remaining),
                                           previous, 400.0f, 0.0f, leeway, kDt);
    if (step.phase == Motion::StepPhase::Decel) {
      CHECK(step.velocity >= previous - leeway.aDecelPlan * kDt - 1e-3f);
    }
    // Full authority is always respected, plan or no plan.
    CHECK(step.velocity >= previous - leeway.aDecel * kDt - 1e-3f);
    remaining -= static_cast<double>(step.velocity) * kDt;
    previous = step.velocity;
    if (step.closing) {
      closed = true;
      break;
    }
  }
  CHECK(closed);
  CHECK_NEAR(remaining, 0.0, 1e-4);

  // aDecelPlan above aDecel is clamped down, never up.
  AxisLimits over = tight;
  over.aDecelPlan = 9999.0f;
  CHECK_NEAR(Motion::planDecel(over), tight.aDecel, 1e-6);
  CHECK_NEAR(Motion::planDecel(tight), tight.aDecel, 1e-6);
  CHECK_NEAR(Motion::planDecel(leeway), 200.0f, 1e-6);
}

}  // namespace

int main() {
  testBrakeDistanceValues();
  testMaxEntryVelocity();
  testSweepExactLanding();
  testShortMoveFromSpeed();
  testCruiseHold();
  testPhaseReported();
  testDecelLeewayStartsBrakingEarlier();
  std::printf("profile_test: all checks passed\n");
  return 0;
}
