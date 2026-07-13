// ekf_tiny_harness.cpp — off-hardware acceptance harness for ticket 082-001
// (SUC-001): exercises EkfTiny (source/estimation/ekf_tiny.{h,cpp}) — the
// 3-state (x, y, heading) EKF core ported and trimmed from the parked
// source_old/state/EKFTiny.* (5-state x, y, theta, v, omega) — in isolation,
// with no encoder/odometer wiring (that wiring is ticket 002's job).
//
// Per motor_policy_harness.cpp / velocity_pid_harness.cpp's precedent
// (078-004 / 081-001), this #includes only estimation/ekf_tiny.h plus its
// own translation unit (ekf_tiny.cpp), so it compiles with the plain system
// C++ compiler — no CMake, no ARM toolchain — as long as
// libraries/tinyekf/ is also on the include path (tinyekf.h is header-only).
//
// Two required scenarios (see ticket 082-001's Acceptance Criteria):
//   (a) predict-only matches a hand-computed arc-integration reference
//       within floating-point tolerance.
//   (b) predict+correct demonstrably pulls the estimate TOWARD a
//       deliberately-offset observation (proves the correction step is not
//       a no-op).
//
// Plain C++ program, hand-rolled assertions (mirrors the existing harnesses'
// shape) — prints a PASS/FAIL line per scenario and exits nonzero if any
// assertion failed.
//
// Verification command (see ticket 082-001's Testing plan — no `uv run
// pytest` involvement for this ticket, pure C++, no Python surface yet):
//   c++ -std=c++11 -Wall -Wextra \
//       -I source -I libraries/tinyekf \
//       -o /tmp/ekf_tiny_harness \
//       tests/sim/unit/ekf_tiny_harness.cpp source/estimation/ekf_tiny.cpp
//   /tmp/ekf_tiny_harness

#include <cmath>
#include <cstdio>
#include <string>

#include "estimation/ekf_tiny.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors sim_hardware_harness.cpp /
// velocity_pid_harness.cpp) ---

int g_failureCount = 0;
std::string g_scenarioName;

void beginScenario(const std::string& name) {
  g_scenarioName = name;
  std::printf("--- %s\n", name.c_str());
}

void fail(const std::string& what) {
  ++g_failureCount;
  std::printf("  FAIL [%s]: %s\n", g_scenarioName.c_str(), what.c_str());
}

void checkTrue(bool condition, const std::string& what) {
  if (!condition) fail(what + " — expected true, got false");
}

void checkNear(float actual, float expected, float tol, const std::string& what) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s — expected %.6f +/- %.6f, got %.6f",
                  what.c_str(), static_cast<double>(expected),
                  static_cast<double>(tol), static_cast<double>(actual));
    fail(buf);
  }
}

// Wrap angle to (-pi, pi], same identity EkfTiny itself uses — kept
// independent here (not a call into the class under test) so the reference
// pose computed below is a genuinely separate re-derivation of the
// arc-segment motion model's equations, not a reuse of the implementation.
float wrapPiRef(float theta) {
  return std::atan2(std::sin(theta), std::cos(theta));
}

// One arc-segment motion-model step, hand-computed independently of
// EkfTiny::predict() — mirrors the equations documented in ekf_tiny.h /
// ekf_tiny.cpp (thetaMid = thetaBefore + dTheta/2; x += dCenter*cos(thetaMid);
// y += dCenter*sin(thetaMid); theta = wrapPi(theta + dTheta)).
struct RefPose {
  float x;
  float y;
  float theta;
};

RefPose referencePredictStep(RefPose prev, float dCenter, float dTheta) {
  float thetaMid = prev.theta + dTheta * 0.5f;
  RefPose next;
  next.x = prev.x + dCenter * std::cos(thetaMid);
  next.y = prev.y + dCenter * std::sin(thetaMid);
  next.theta = wrapPiRef(prev.theta + dTheta);
  return next;
}

// --- Scenarios ----------------------------------------------------------

// (a) predict-only: drive the filter through a short, varied sequence of
// arc segments (straight run, a turn, a reverse-curving turn) with no
// corrections at all, and assert the filter's (x, y, theta) matches an
// independently hand-computed arc-integration reference within tight
// floating-point tolerance.
void scenarioPredictOnlyMatchesArcIntegrationReference() {
  beginScenario("predict-only matches hand-computed arc-integration reference");

  EkfTiny filter;
  filter.init(/*qXy=*/4.0f, /*qTheta=*/0.001f, /*rOtosXy=*/25.0f,
              /*rOtosTheta=*/0.00076f);

  struct Step {
    float dCenter;   // [mm]
    float dTheta;    // [rad]
    float dt;        // [s]
  };
  const Step steps[] = {
      {100.0f, 0.0f, 0.02f},                  // straight run
      {50.0f, static_cast<float>(M_PI) / 2.0f, 0.02f},    // 90 deg turn
      {80.0f, -static_cast<float>(M_PI) / 4.0f, 0.03f},   // partial reverse turn
      {30.0f, static_cast<float>(M_PI) / 6.0f, 0.015f},   // small turn
  };

  RefPose ref{0.0f, 0.0f, 0.0f};
  for (const Step& s : steps) {
    // thetaBefore is the filter's own current theta() — the caller's
    // contract per ekf_tiny.h — read BEFORE calling predict(), which
    // mutates state.
    float thetaBefore = filter.theta();
    filter.predict(s.dCenter, s.dTheta, thetaBefore, s.dt);
    ref = referencePredictStep(ref, s.dCenter, s.dTheta);
  }

  checkNear(filter.x(), ref.x, 1e-3f, "x() matches arc-integration reference after 4 predict-only ticks");
  checkNear(filter.y(), ref.y, 1e-3f, "y() matches arc-integration reference after 4 predict-only ticks");
  checkNear(filter.theta(), ref.theta, 1e-4f, "theta() matches arc-integration reference after 4 predict-only ticks");
}

// (b1) predict+correct (position channel): drive the filter forward with a
// few predict() ticks, then apply updatePosition() with an observation
// deliberately offset from the filter's own belief. Assert the post-update
// state moved TOWARD the observation (strictly closer on both axes than the
// pre-update state), and that it actually moved (not a no-op).
void scenarioUpdatePositionPullsTowardOffsetObservation() {
  beginScenario("updatePosition() pulls the estimate toward an offset observation");

  EkfTiny filter;
  filter.init(/*qXy=*/4.0f, /*qTheta=*/0.001f, /*rOtosXy=*/25.0f,
              /*rOtosTheta=*/0.00076f);

  // A few straight-line predict ticks so P has grown off zero (P starts at
  // zero out of init(); a zero-P Kalman gain is identically zero, which
  // would make updatePosition() a trivial no-op for the wrong reason — the
  // predict ticks below are what make the correction step meaningful).
  for (int i = 0; i < 5; ++i) {
    float thetaBefore = filter.theta();
    filter.predict(/*dCenter=*/40.0f, /*dTheta=*/0.0f, thetaBefore, /*dt=*/0.02f);
  }

  float preX = filter.x();
  float preY = filter.y();

  // Deliberate offset: observation sits away from the filter's belief in
  // both x and y, but INSIDE the ticket 006 innovation gate (D4's
  // chi-square threshold) so the correction is accepted, not rejected --
  // after these 5 straight predict ticks, P[0][0]=0.4/P[1][1]=1.36 exactly
  // (hand-derivable: dTheta=0 throughout keeps x decoupled from theta, so
  // P[0][0] is a pure qXy*dt sum; see the gate scenarios below for the same
  // derivation applied to the gate's own boundary tests), giving
  // s00=25.4/s11=26.36 and a chi-square(9.21) boundary near 15-16mm per
  // axis -- (10, -8) stays safely under that on both axes.
  const float kOffsetX = 10.0f;   // [mm]
  const float kOffsetY = -8.0f;   // [mm]
  float xOtos = preX + kOffsetX;
  float yOtos = preY + kOffsetY;

  filter.updatePosition(xOtos, yOtos);

  float postX = filter.x();
  float postY = filter.y();

  checkTrue(postX != preX, "updatePosition() actually moved x() (not a no-op)");
  checkTrue(postY != preY, "updatePosition() actually moved y() (not a no-op)");

  checkTrue(std::fabs(postX - xOtos) < std::fabs(preX - xOtos),
            "post-update x() is strictly closer to the observation than pre-update x()");
  checkTrue(std::fabs(postY - yOtos) < std::fabs(preY - yOtos),
            "post-update y() is strictly closer to the observation than pre-update y()");

  // Moved in the correct direction (toward the observation, not away).
  checkTrue((postX - preX) * kOffsetX > 0.0f,
            "x() moved in the same direction as the offset (toward the observation)");
  checkTrue((postY - preY) * kOffsetY > 0.0f,
            "y() moved in the same direction as the offset (toward the observation)");
}

// (b2) predict+correct (heading channel): same shape as (b1) but for
// updateHeading() — a deliberately offset heading observation must pull
// theta() toward it, not away from it or leave it unchanged.
void scenarioUpdateHeadingPullsTowardOffsetObservation() {
  beginScenario("updateHeading() pulls the estimate toward an offset observation");

  EkfTiny filter;
  filter.init(/*qXy=*/4.0f, /*qTheta=*/0.001f, /*rOtosXy=*/25.0f,
              /*rOtosTheta=*/0.00076f);

  // A few turning predict ticks so P[2][2] has grown off zero.
  for (int i = 0; i < 5; ++i) {
    float thetaBefore = filter.theta();
    filter.predict(/*dCenter=*/20.0f, /*dTheta=*/0.05f, thetaBefore, /*dt=*/0.02f);
  }

  float preTheta = filter.theta();

  // Deliberate offset: ~2.9 deg away from the filter's current belief --
  // INSIDE the ticket 006 innovation gate (D4's kHeadingSigma=3.0 bound).
  // After these 5 predict ticks P[2][2]=0.0001 exactly (P[2][2] evolves
  // completely decoupled from x/y in the arc-motion Jacobian -- it is a
  // pure qTheta*dt sum regardless of dCenter/dTheta; see the gate scenarios
  // below), giving s=0.00086 and a 3-sigma boundary near 0.088 rad --
  // 0.05 stays safely under that.
  const float kOffsetTheta = 0.05f;   // [rad]
  float thetaOtos = wrapPiRef(preTheta + kOffsetTheta);

  filter.updateHeading(thetaOtos);

  float postTheta = filter.theta();

  checkTrue(postTheta != preTheta, "updateHeading() actually moved theta() (not a no-op)");

  float preErr = std::fabs(wrapPiRef(thetaOtos - preTheta));
  float postErr = std::fabs(wrapPiRef(thetaOtos - postTheta));
  checkTrue(postErr < preErr,
            "post-update theta() is strictly closer to the observation than pre-update theta()");

  // Moved in the correct direction (toward the observation).
  float delta = wrapPiRef(postTheta - preTheta);
  checkTrue(delta * kOffsetTheta > 0.0f,
            "theta() moved in the same direction as the offset (toward the observation)");
}

// --- Ticket 099-006: bounded innovation-consistency gate + rejection-streak
// P-inflation recovery -------------------------------------------------
//
// This IS the characterization work D4/ticket 006 requires -- every
// constant below is an INDEPENDENT re-declaration (not a reach into
// EkfTiny's private state) of the documented starting values from
// architecture-update.md's Decision 4, kept in lock-step with
// ekf_tiny.h's actual private kChiSquare2Dof99/kHeadingSigma/
// kRejectStreakThreshold/kPInflationBumpXY/kPInflationCapXY/
// kPInflationBumpTheta/kPInflationCapTheta constants (same "independent
// reference" pattern wrapPiRef()/referencePredictStep() already use above
// for the predict-only scenario). If EkfTiny's own constants are ever
// retuned, these must be updated to match -- that is the intended coupling
// (this file is where D4's thresholds get bench/sim-characterized).
namespace gate {

// Only the constants actually referenced by an assertion below are kept
// here (the rest of D4's starting values -- kChiSquare2Dof99=9.21,
// kHeadingSigma=3.0, kBumpXY=50.0mm^2, kBumpTheta=0.001rad^2,
// kCapTheta=0.01rad^2 -- are exercised implicitly through the hand-computed
// literal offsets/expected boundaries documented inline in each scenario's
// comment, per this file's existing "independent reference" convention).
constexpr int kStreakThreshold = 10;
constexpr float kCapXY = 500.0f;      // [mm^2]

// Shared setup used by every gate scenario below: init with the same noise
// parameters the pull-toward-observation scenarios above use, then drive 5
// straight predict ticks (dCenter=40mm, dTheta=0, dt=0.02s) so P has grown
// off zero. Because dTheta=0 for every step, thetaMid=thetaBefore=0
// throughout, which makes f[0][2] = -dCenter*sin(0) = 0 for every step --
// x() is therefore fully decoupled from theta() and P[0][0] is a pure
// qXy*dt sum: 5 * 4.0 * 0.02 = 0.4 exactly. P[2][2] evolves independently
// of dCenter/dTheta entirely (row 2 of F is always [0,0,1], so
// F*P*F^T's row/col 2 pass P[2][2] through untouched) -- it is a pure
// qTheta*dt sum too: 5 * 0.001 * 0.02 = 0.0001 exactly, regardless of the
// motion path. Both are hand-derivable exactly, not empirically read back,
// mirroring scenarioPredictOnlyMatchesArcIntegrationReference()'s own
// "independent reference" discipline.
EkfTiny makeSettledFilter() {
  EkfTiny filter;
  filter.init(/*qXy=*/4.0f, /*qTheta=*/0.001f, /*rOtosXy=*/25.0f,
              /*rOtosTheta=*/0.00076f);
  for (int i = 0; i < 5; ++i) {
    float thetaBefore = filter.theta();
    filter.predict(/*dCenter=*/40.0f, /*dTheta=*/0.0f, thetaBefore, /*dt=*/0.02f);
  }
  return filter;
}

// rOtosXy=25/rOtosTheta=0.00076 (init()'s own arguments above) combine with
// these settled P values into s00=25.4/s=0.00086, the exact figures every
// scenario's comment below derives its hand-computed boundaries from.
constexpr float kSettledP00 = 0.4f;  // [mm^2] see makeSettledFilter()'s doc comment

}  // namespace gate

// (c1) position channel accept/reject boundary: with P[0][0]=0.4 (settled,
// see makeSettledFilter()) and rOtosXy=25, s00=25.4 exactly, so the
// chi-square(9.21) boundary sits at sqrt(9.21*25.4) ~= 15.29mm. A 12mm
// x-only innovation (d^2=144/25.4=5.67 < 9.21) must be ACCEPTED (state
// moves); a 20mm x-only innovation (d^2=400/25.4=15.75 > 9.21) must be
// REJECTED (state unchanged) -- both against known, hand-computed
// innovation values per the AC.
void scenarioPositionGateAcceptRejectBoundary() {
  beginScenario("updatePosition() gate: accept/reject against known innovation values");

  {
    EkfTiny filter = gate::makeSettledFilter();
    float preX = filter.x();
    float preY = filter.y();
    const float kAcceptOffsetX = 12.0f;  // [mm] d^2 = 144/25.4 = 5.67 < 9.21
    filter.updatePosition(preX + kAcceptOffsetX, preY);
    checkTrue(filter.x() != preX,
              "a 12mm innovation (d^2=5.67 < chi-square 9.21) is ACCEPTED -- x() moves");
  }

  {
    EkfTiny filter = gate::makeSettledFilter();
    float preX = filter.x();
    float preY = filter.y();
    const float kRejectOffsetX = 20.0f;  // [mm] d^2 = 400/25.4 = 15.75 > 9.21
    filter.updatePosition(preX + kRejectOffsetX, preY);
    checkNear(filter.x(), preX, 0.0f,
              "a 20mm innovation (d^2=15.75 > chi-square 9.21) is REJECTED -- x() unchanged");
    checkNear(filter.y(), preY, 0.0f, "y() unchanged too on a rejected update");
  }
}

// (c2) heading channel accept/reject boundary: with P[2][2]=0.0001
// (settled) and rOtosTheta=0.00076, s=0.00086 exactly, sqrt(s)~=0.02933,
// so the kHeadingSigma=3.0 boundary sits at ~0.08797 rad. A 0.05 rad
// innovation (< boundary) must be ACCEPTED; a 0.15 rad innovation
// (> boundary) must be REJECTED.
void scenarioHeadingGateAcceptRejectBoundary() {
  beginScenario("updateHeading() gate: accept/reject against known innovation values");

  {
    EkfTiny filter = gate::makeSettledFilter();
    float preTheta = filter.theta();
    const float kAcceptOffsetTheta = 0.05f;  // [rad] 0.05 < 3*sqrt(0.00086)=0.088
    filter.updateHeading(wrapPiRef(preTheta + kAcceptOffsetTheta));
    checkTrue(filter.theta() != preTheta,
              "a 0.05 rad innovation (< 3-sigma bound 0.088) is ACCEPTED -- theta() moves");
  }

  {
    EkfTiny filter = gate::makeSettledFilter();
    float preTheta = filter.theta();
    const float kRejectOffsetTheta = 0.15f;  // [rad] 0.15 > 3*sqrt(0.00086)=0.088
    filter.updateHeading(wrapPiRef(preTheta + kRejectOffsetTheta));
    checkNear(filter.theta(), preTheta, 0.0f,
              "a 0.15 rad innovation (> 3-sigma bound 0.088) is REJECTED -- theta() unchanged");
  }
}

// (c3) position channel rejection-streak recovery: a FIXED, genuinely-
// shifted 30mm x-only observation is repeatedly re-applied to the SAME
// filter (no predict() interleaved -- an isolated, deterministic streak).
// Hand-traced (see ticket 006's completion notes): calls 1-9 reject at
// constant d^2=900/25.4=35.4; call 10 crosses the streak threshold and
// inflates P[0][0] by kBumpXY (0.4 -> 50.4), still rejecting at
// d^2=900/75.4=11.9 > 9.21; calls 11-19 keep rejecting at that same P;
// call 20 crosses the threshold a second time (P[0][0] 50.4 -> 100.4),
// still rejecting at d^2=900/125.4=7.18 -- wait, that IS below 9.21, so
// call 21 (the first call evaluated against the twice-inflated P) is
// ACCEPTED. Documented tick bound: recovery must land within
// kDocumentedTickBound (25) calls -- comfortably above the measured 21,
// and strictly more than kStreakThreshold (10) to prove the gate did not
// just cave on the very first widening.
void scenarioPositionRejectionStreakRecoversViaPInflation() {
  beginScenario("updatePosition(): rejection streak recovers via P-inflation within a documented tick bound");

  EkfTiny filter = gate::makeSettledFilter();
  float preX = filter.x();
  float preY = filter.y();
  const float kShiftedOffsetX = 30.0f;  // [mm] genuinely-shifted, fixed observation
  const float xOtos = preX + kShiftedOffsetX;

  const int kDocumentedTickBound = 25;
  int acceptedAtCall = -1;
  for (int call = 1; call <= kDocumentedTickBound; ++call) {
    filter.updatePosition(xOtos, preY);
    if (filter.x() != preX) {
      acceptedAtCall = call;
      break;
    }
  }

  checkTrue(acceptedAtCall > 0,
            "the fixed, genuinely-shifted observation is EVENTUALLY accepted after "
            "streak-triggered P-inflation");
  checkTrue(acceptedAtCall > gate::kStreakThreshold,
            "recovery required MORE than one streak cycle -- proves the gate did not "
            "cave on the very first widening (initial d^2=35.4 is far past threshold)");
  checkTrue(acceptedAtCall <= kDocumentedTickBound,
            "recovery lands within the documented tick bound (measured: call 21)");
}

// (c4) heading channel rejection-streak recovery: same shape as (c3), on
// the heading channel's own independent streak counter/P[2][2]. A fixed
// 0.15 rad innovation starts rejected (0.15 > 0.088), inflates P[2][2] by
// kBumpTheta at streak 10 (still rejected: 3*sqrt(0.00186)=0.129 < 0.15)
// and again at streak 20 (3*sqrt(0.00286)=0.160 > 0.15 -- accepted on
// call 21).
void scenarioHeadingRejectionStreakRecoversViaPInflation() {
  beginScenario("updateHeading(): rejection streak recovers via P-inflation within a documented tick bound");

  EkfTiny filter = gate::makeSettledFilter();
  float preTheta = filter.theta();
  const float kShiftedOffsetTheta = 0.15f;  // [rad] genuinely-shifted, fixed observation
  const float thetaOtos = wrapPiRef(preTheta + kShiftedOffsetTheta);

  const int kDocumentedTickBound = 25;
  int acceptedAtCall = -1;
  for (int call = 1; call <= kDocumentedTickBound; ++call) {
    filter.updateHeading(thetaOtos);
    if (filter.theta() != preTheta) {
      acceptedAtCall = call;
      break;
    }
  }

  checkTrue(acceptedAtCall > 0,
            "the fixed, genuinely-shifted heading observation is EVENTUALLY accepted "
            "after streak-triggered P-inflation");
  checkTrue(acceptedAtCall > gate::kStreakThreshold,
            "recovery required MORE than one streak cycle on the heading channel too");
  checkTrue(acceptedAtCall <= kDocumentedTickBound,
            "recovery lands within the documented tick bound (measured: call 21)");
}

// (c5) a single accepted, noisy-but-not-shifted observation does not trip
// the streak counter toward inflation: seed a PARTIAL streak (5 rejects,
// below the 10-call threshold, so P is still untouched), then feed ONE
// small, in-gate observation (accepted), then feed 9 MORE reject-level
// observations. If (and only if) the single accept correctly reset the
// streak counter to 0, these 9 further rejects total only 9 (< 10) and
// never trigger inflation -- P[0][0] stays exactly where it was. A buggy
// implementation that failed to reset the streak on accept would carry the
// earlier 5 forward, cross the streak=10 threshold partway through these 9,
// and inflate P -- which this scenario would catch via the pDiag(0)
// equality check below.
void scenarioSingleAcceptedNoisyObservationDoesNotPreloadStreak() {
  beginScenario("a single accepted, noisy-but-not-shifted observation does not "
                "preload the rejection streak toward inflation");

  EkfTiny filter = gate::makeSettledFilter();
  float preX = filter.x();
  float preY = filter.y();
  const float kRejectOffsetX = 30.0f;  // [mm] d^2 = 900/25.4 = 35.4 > 9.21 -- reject

  // 5 partial rejects (below the streak threshold -- P must stay untouched).
  for (int i = 0; i < 5; ++i) {
    filter.updatePosition(preX + kRejectOffsetX, preY);
  }
  checkNear(filter.x(), preX, 0.0f, "5 partial rejects (< streak threshold) leave x() unchanged");
  float pAfterPartialStreak = filter.pDiag(0);
  checkNear(pAfterPartialStreak, gate::kSettledP00, 1e-4f,
            "5 partial rejects (< streak threshold) leave P[0][0] uninflated");

  // One small, in-gate observation -- accepted, and (if correctly
  // implemented) resets the streak counter to 0.
  const float kAcceptOffsetX = 12.0f;  // [mm] d^2 = 144/25.4 = 5.67 < 9.21 -- accept
  float xBeforeAccept = filter.x();
  filter.updatePosition(xBeforeAccept + kAcceptOffsetX, filter.y());
  checkTrue(filter.x() != xBeforeAccept, "the single noisy observation is accepted -- x() moves");

  // 9 more reject-level observations -- if the streak correctly reset to 0
  // above, this totals only 9 (< 10) and must NOT trigger inflation.
  float xBeforeFurtherRejects = filter.x();
  float yBeforeFurtherRejects = filter.y();
  float pBeforeFurtherRejects = filter.pDiag(0);
  for (int i = 0; i < 9; ++i) {
    filter.updatePosition(xBeforeFurtherRejects + kRejectOffsetX, yBeforeFurtherRejects);
  }
  checkNear(filter.x(), xBeforeFurtherRejects, 0.0f,
            "9 further rejects (streak reset by the intervening accept) leave x() unchanged");
  checkNear(filter.pDiag(0), pBeforeFurtherRejects, 1e-4f,
            "9 further rejects (streak reset by the intervening accept, total 9 < "
            "threshold 10) do NOT inflate P[0][0] -- proves the single accept did not "
            "preload the streak counter toward inflation");
}

// (c6) P-inflation is bounded, not unbounded: a persistently, extremely
// disagreeing observation (10000mm -- far beyond any plausible re-trust
// point even at the capped P) keeps rejecting and keeps inflating every
// kStreakThreshold-th call, but P[0][0]/P[1][1] never exceed kCapXY --
// this is the "gradual, BOUNDED widening" half of the AC, not just the
// eventual-recovery half proven by (c3).
void scenarioPInflationIsBoundedByCap() {
  beginScenario("rejection-streak P-inflation is bounded by a documented cap, never unbounded");

  EkfTiny filter = gate::makeSettledFilter();
  float preX = filter.x();
  float preY = filter.y();
  const float kExtremeOffsetX = 10000.0f;  // [mm] rejected even at the capped P

  const int kCalls = 130;  // 13 full streak cycles -- well past saturation (~10 cycles)
  for (int call = 1; call <= kCalls; ++call) {
    filter.updatePosition(preX + kExtremeOffsetX, preY);
    checkTrue(filter.pDiag(0) <= gate::kCapXY + 1e-3f,
              "P[0][0] never exceeds the documented cap, even under a persistent, "
              "extreme, never-re-accepted disagreement");
    checkTrue(filter.pDiag(1) <= gate::kCapXY + 1e-3f,
              "P[1][1] never exceeds the documented cap either");
  }
  checkTrue(filter.x() == preX,
            "an extreme, permanently-disagreeing observation is never re-accepted just "
            "because P saturated at its cap");
  checkNear(filter.pDiag(0), gate::kCapXY, 1e-2f,
            "P[0][0] has saturated at (not merely approached) the documented cap after "
            "well over 10 streak cycles");
}

}  // namespace

int main() {
  scenarioPredictOnlyMatchesArcIntegrationReference();
  scenarioUpdatePositionPullsTowardOffsetObservation();
  scenarioUpdateHeadingPullsTowardOffsetObservation();

  // Ticket 099-006: bounded innovation-consistency gate + rejection-streak
  // P-inflation recovery.
  scenarioPositionGateAcceptRejectBoundary();
  scenarioHeadingGateAcceptRejectBoundary();
  scenarioPositionRejectionStreakRecoversViaPInflation();
  scenarioHeadingRejectionStreakRecoversViaPInflation();
  scenarioSingleAcceptedNoisyObservationDoesNotPreloadStreak();
  scenarioPInflationIsBoundedByCap();

  if (g_failureCount == 0) {
    std::printf("OK: all EkfTiny scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the EkfTiny scenarios\n", g_failureCount);
  return 1;
}
