// kinematics_harness.cpp -- off-hardware acceptance harness for
// src/firm/kinematics/: Kinematics::Model and its two implementations.
//
// Three things to prove, in order of how badly a regression would hurt:
//
//   1. Differential is the OLD BodyKinematics, bit for bit. The
//      reorganization that created this directory claims "unchanged math";
//      every real robot in this fleet drives through these equations, so
//      that claim gets checked against the round-trip identities and the
//      closed-form values, not taken on faith.
//   2. Mecanum is correct. Nothing in the firmware constructs it
//      yet (Control::DifferentialDrive is still differential-specific end to end), so this
//      harness is its ONLY coverage -- and an interface whose second
//      implementation is untested is not a proven seam.
//   3. Model::saturate() -- the one concrete method on the base -- behaves
//      identically for N=2 and N=4, since it is what makes a path survive
//      a speed ceiling.
//
// Compiled with -DHOST_BUILD for consistency with every other
// src/tests/sim/unit harness. These sources have zero platform dependency
// (pure math, no bus, no clock), so the define changes nothing here.

#include "kinematics/differential.h"
#include "kinematics/kinematics.h"
#include "kinematics/mecanum.h"

#include <cmath>
#include <cstdio>

namespace {

int failures = 0;

void checkNear(float got, float want, float tol, const char* what) {
  if (!(std::fabs(got - want) <= tol)) {
    std::printf("  FAIL: %s -- expected %.6f, got %.6f\n", what, want, got);
    ++failures;
  }
}

void checkInt(int got, int want, const char* what) {
  if (got != want) {
    std::printf("  FAIL: %s -- expected %d, got %d\n", what, want, got);
    ++failures;
  }
}

void beginScenario(const char* name) { std::printf("--- %s\n", name); }

constexpr float kTol = 1e-4f;
constexpr float kTrackWidth = 148.0f;  // [mm]
constexpr float kWheelBase = 120.0f;   // [mm]

// ---------------------------------------------------------------------------
// 1. Differential -- the closed forms from docs/kinematics-model.md §1.3.
// ---------------------------------------------------------------------------

void scenarioDifferentialClosedForm() {
  beginScenario("differential: inverse/forward match the closed forms exactly");
  using K = Kinematics::Differential;

  // Straight: both wheels at v, no yaw.
  float vL = 0.0f, vR = 0.0f;
  K::inverse(200.0f, 0.0f, kTrackWidth, vL, vR);
  checkNear(vL, 200.0f, kTol, "straight drive leaves the left wheel at v");
  checkNear(vR, 200.0f, kTol, "straight drive leaves the right wheel at v");

  // Pure spin: vL = -omega*b/2, vR = +omega*b/2.
  K::inverse(0.0f, 1.5f, kTrackWidth, vL, vR);
  checkNear(vL, -1.5f * kTrackWidth * 0.5f, kTol, "spin left wheel is -omega*b/2");
  checkNear(vR, 1.5f * kTrackWidth * 0.5f, kTol, "spin right wheel is +omega*b/2");

  // forward() is inverse()'s exact inverse.
  float v = 0.0f, omega = 0.0f;
  K::inverse(137.0f, -0.8f, kTrackWidth, vL, vR);
  K::forward(vL, vR, kTrackWidth, v, omega);
  checkNear(v, 137.0f, kTol, "forward(inverse(v)) round-trips v");
  checkNear(omega, -0.8f, kTol, "forward(inverse(omega)) round-trips omega");

  // CCW-positive: right wheel faster than left means positive omega.
  K::forward(100.0f, 140.0f, kTrackWidth, v, omega);
  checkNear(omega, 40.0f / kTrackWidth, kTol,
            "omega is (vR - vL) / b, CCW-positive");
  checkNear(v, 120.0f, kTol, "v is (vR + vL) / 2");
}

void scenarioDifferentialModel() {
  beginScenario("differential: the Model& overrides agree with the statics");
  const Kinematics::Differential model(kTrackWidth);
  const Kinematics::Model& m = model;

  checkInt(m.wheelCount(), 2, "a differential drive has 2 wheels");

  Kinematics::Twist in;
  in.v_x = 180.0f;
  in.v_y = 55.0f;  // deliberately nonzero -- must be IGNORED, not an error
  in.omega = 0.6f;

  float wheels[Kinematics::kMaxWheels] = {};
  m.inverse(in, wheels);

  float vL = 0.0f, vR = 0.0f;
  Kinematics::Differential::inverse(in.v_x, in.omega, kTrackWidth, vL, vR);
  checkNear(wheels[0], vL, kTol, "Model::inverse left matches the static");
  checkNear(wheels[1], vR, kTol, "Model::inverse right matches the static");

  Kinematics::Twist out;
  m.forward(wheels, out);
  checkNear(out.v_x, in.v_x, kTol, "round-trip preserves v_x");
  checkNear(out.omega, in.omega, kTol, "round-trip preserves omega");
  checkNear(out.v_y, 0.0f, kTol,
            "v_y is reported 0 -- a differential drive cannot strafe");
}

// ---------------------------------------------------------------------------
// 2. Mecanum -- this class's only coverage anywhere.
// ---------------------------------------------------------------------------

void scenarioMecanumPureMotions() {
  beginScenario("mecanum: the three pure body motions hit the expected wheels");
  const Kinematics::Mecanum model(kTrackWidth, kWheelBase);
  const Kinematics::Model& m = model;
  const float k = 0.5f * (kWheelBase + kTrackWidth);  // [mm]

  checkInt(m.wheelCount(), 4, "a mecanum drive has 4 wheels");

  float w[Kinematics::kMaxWheels] = {};
  Kinematics::Twist t;

  // Pure forward: all four wheels the same speed, same sign.
  t = Kinematics::Twist{};
  t.v_x = 200.0f;
  m.inverse(t, w);
  for (int i = 0; i < 4; ++i) {
    checkNear(w[i], 200.0f, kTol, "pure forward drives every wheel at v_x");
  }

  // Pure strafe (+y, left): FL and BR go negative, FR and BL positive.
  t = Kinematics::Twist{};
  t.v_y = 150.0f;
  m.inverse(t, w);
  checkNear(w[0], -150.0f, kTol, "strafe: FL is -v_y");
  checkNear(w[1], 150.0f, kTol, "strafe: FR is +v_y");
  checkNear(w[2], 150.0f, kTol, "strafe: BL is +v_y");
  checkNear(w[3], -150.0f, kTol, "strafe: BR is -v_y");

  // Pure spin: left side back, right side forward (CCW-positive).
  t = Kinematics::Twist{};
  t.omega = 1.0f;
  m.inverse(t, w);
  checkNear(w[0], -k, kTol, "spin: FL is -(lx+ly)*omega");
  checkNear(w[1], k, kTol, "spin: FR is +(lx+ly)*omega");
  checkNear(w[2], -k, kTol, "spin: BL is -(lx+ly)*omega");
  checkNear(w[3], k, kTol, "spin: BR is +(lx+ly)*omega");
}

void scenarioMecanumRoundTrip() {
  beginScenario("mecanum: forward(inverse(twist)) is the identity");
  const Kinematics::Mecanum model(kTrackWidth, kWheelBase);
  const Kinematics::Model& m = model;

  Kinematics::Twist in;
  in.v_x = 123.0f;
  in.v_y = -87.0f;
  in.omega = 0.42f;

  float w[Kinematics::kMaxWheels] = {};
  m.inverse(in, w);

  Kinematics::Twist out;
  m.forward(w, out);
  checkNear(out.v_x, in.v_x, kTol, "round-trip preserves v_x");
  checkNear(out.v_y, in.v_y, kTol, "round-trip preserves v_y (holonomic)");
  checkNear(out.omega, in.omega, kTol, "round-trip preserves omega");
}

// ---------------------------------------------------------------------------
// 3. saturate() -- the base class's one concrete method.
// ---------------------------------------------------------------------------

void scenarioSaturate() {
  beginScenario("saturate: uniform scaling preserves the wheel-speed ratio");
  const Kinematics::Differential diff(kTrackWidth);
  const Kinematics::Model& m2 = diff;

  // Below the ceiling: pass-through.
  const float under[2] = {100.0f, 200.0f};
  float out2[Kinematics::kMaxWheels] = {};
  m2.saturate(under, 500.0f, 0.0f, out2);
  checkNear(out2[0], 100.0f, kTol, "under the ceiling, left passes through");
  checkNear(out2[1], 200.0f, kTol, "under the ceiling, right passes through");

  // Over the ceiling: fastest wheel lands exactly on it, ratio preserved.
  const float over[2] = {300.0f, 600.0f};
  m2.saturate(over, 500.0f, 100.0f, out2);  // ceiling 400
  checkNear(out2[1], 400.0f, kTol, "the fastest wheel lands on the ceiling");
  checkNear(out2[0] / out2[1], 300.0f / 600.0f, kTol,
            "the wheel-speed ratio -- hence the arc -- survives saturation");

  // The N=2 result is identical to the old two-wheel static, which is what
  // "no behavior change" has to mean here.
  float staticL = 0.0f, staticR = 0.0f;
  Kinematics::Differential::saturate(over[0], over[1], 500.0f, 100.0f,
                                               staticL, staticR);
  checkNear(out2[0], staticL, kTol, "Model::saturate matches the 2-wheel static (L)");
  checkNear(out2[1], staticR, kTol, "Model::saturate matches the 2-wheel static (R)");

  // Same rule over 4 wheels, including a negative one.
  const Kinematics::Mecanum mec(kTrackWidth, kWheelBase);
  const Kinematics::Model& m4 = mec;
  const float over4[4] = {200.0f, -800.0f, 400.0f, 100.0f};
  float out4[Kinematics::kMaxWheels] = {};
  m4.saturate(over4, 400.0f, 0.0f, out4);
  checkNear(out4[1], -400.0f, kTol,
            "the fastest wheel by MAGNITUDE lands on the ceiling, sign kept");
  checkNear(out4[0] / out4[1], 200.0f / -800.0f, kTol,
            "4-wheel ratios survive saturation too");

  // Aliasing: out may be the same array as wheels.
  float alias[4] = {200.0f, -800.0f, 400.0f, 100.0f};
  m4.saturate(alias, 400.0f, 0.0f, alias);
  checkNear(alias[1], -400.0f, kTol, "saturate() is safe when out aliases wheels");
}

}  // namespace

int main() {
  scenarioDifferentialClosedForm();
  scenarioDifferentialModel();
  scenarioMecanumPureMotions();
  scenarioMecanumRoundTrip();
  scenarioSaturate();

  if (failures != 0) {
    std::printf("FAILED: %d assertion(s) across the Kinematics scenarios\n",
                failures);
    return 1;
  }
  std::printf("OK: all Kinematics::Model scenarios passed\n");
  return 0;
}
