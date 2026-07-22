// app_state_estimator_harness.cpp -- off-hardware acceptance harness for
// sprint 117 ticket 002 (SUC-057), App::StateEstimator
// (src/firm/app/state_estimator.{h,cpp}). Proves: ZOH distance/velocity
// wheel extrapolation, ZOH body pose extrapolation (straight-line and
// rotating), per-peer valid semantics (false before the first update(),
// true after, independently for each wheel and the body), reset()'s
// body-only re-anchor (wheel peers untouched, basisTime/valid untouched),
// innovations() residual computation (even at weight 0, diagnostic only),
// and the v1 complementary-blend weight cases (0.0 pure encoder, 1.0 pure
// OTOS, intermediate proportional blend) -- including the staleness gate
// (a too-old or absent OTOS reading blends nothing and leaves innovations()
// unchanged).
//
// Unlike app_odometry_harness.cpp's sibling scenarios, this module has NO
// I2C bus/Devices:: leaf dependency at all (state_estimator.h's own file
// header: "no I2C bus access, no sleeping, no owned Devices::Clock&
// collaborator") -- every scenario below hand-builds a
// App::Telemetry::Frame directly (the same struct RobotLoop's real
// updateTlm()/applyOtosSample()/odom_.integrate() calls stage each cycle)
// rather than scripting a SimPlant/ScriptedI2CHook bus. Compiled by
// test_app_state_estimator.py with -DHOST_BUILD against state_estimator.cpp
// alone -- no other .cpp dependency is needed (this harness never calls a
// Comms/Telemetry member function, only references the nested
// Telemetry::Frame struct type, so the class BODIES declared in comms.h/
// telemetry.h need not be linked).
//
// Mirrors app_odometry_harness.cpp's exact hand-rolled assertion plumbing
// (beginScenario/fail/checkTrue/checkFalse/checkNear), PASS/FAIL printf,
// exit nonzero on failure.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "app/state_estimator.h"
#include "app/telemetry.h"

namespace {

// --- Hand-rolled assertion plumbing (see app_odometry_harness.cpp) ------

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
  if (!condition) fail(what + " -- expected true, got false");
}

void checkFalse(bool condition, const std::string& what) {
  if (condition) fail(what + " -- expected false, got true");
}

void checkUintEq(uint32_t actual, uint32_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %u, got %u", what.c_str(),
                  static_cast<unsigned>(expected), static_cast<unsigned>(actual));
    fail(buf);
  }
}

void checkNear(float actual, float expected, float tol, const std::string& what) {
  if (std::fabs(static_cast<double>(actual - expected)) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g (tol %g)", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual),
                  static_cast<double>(tol));
    fail(buf);
  }
}

// --- Telemetry::Frame builder -- explicit field assignment (not aggregate
// braces) so a future field reorder in the generated messages/*.h structs
// can't silently swap two same-typed fields under this harness. ---------

App::Telemetry::Frame makeFrame(float encLeftPos, float encLeftVel, uint32_t encLeftTime,
                                 float encRightPos, float encRightVel, uint32_t encRightTime,
                                 float poseX, float poseY, float poseH, float twistVx,
                                 float twistVy, float twistOmega, bool otosPresent,
                                 float otosHeading = 0.0f, float otosOmega = 0.0f,
                                 uint32_t otosTime = 0) {
  App::Telemetry::Frame frame;
  frame.encLeft.position = encLeftPos;
  frame.encLeft.velocity = encLeftVel;
  frame.encLeft.time = encLeftTime;
  frame.encRight.position = encRightPos;
  frame.encRight.velocity = encRightVel;
  frame.encRight.time = encRightTime;
  frame.pose.x = poseX;
  frame.pose.y = poseY;
  frame.pose.h = poseH;
  frame.twist.v_x = twistVx;
  frame.twist.v_y = twistVy;
  frame.twist.omega = twistOmega;
  frame.otosPresent = otosPresent;
  frame.otos.heading = otosHeading;
  frame.otos.omega = otosOmega;
  frame.otos.time = otosTime;
  return frame;
}

// ===========================================================================
// 1. wheelAt()/wheelNow(): ZOH distance extrapolation, velocity held
//    constant, original basisTime preserved in the returned estimate.
// ===========================================================================

void scenarioWheelZohExtrapolation() {
  beginScenario("StateEstimator::wheelAt(): ZOH distance extrapolation, velocity held constant");

  App::StateEstimator estimator;
  App::Telemetry::Frame frame = makeFrame(/*encLeftPos=*/100.0f, /*encLeftVel=*/50.0f,
                                           /*encLeftTime=*/1000,
                                           /*encRightPos=*/-40.0f, /*encRightVel=*/-20.0f,
                                           /*encRightTime=*/1000,
                                           /*poseX=*/0, /*poseY=*/0, /*poseH=*/0,
                                           /*twistVx=*/0, /*twistVy=*/0, /*twistOmega=*/0,
                                           /*otosPresent=*/false);
  estimator.update(frame, /*now=*/1000);

  App::WheelEstimate leftNow = estimator.wheelNow(App::Wheel::Left);
  checkTrue(leftNow.valid, "wheelNow(Left) valid after update()");
  checkNear(leftNow.distance, 100.0f, 1e-6f, "wheelNow(Left) distance == basis position");
  checkNear(leftNow.velocity, 50.0f, 1e-6f, "wheelNow(Left) velocity == basis velocity");
  checkUintEq(leftNow.basisTime, 1000, "wheelNow(Left) basisTime == encLeft.time");

  App::WheelEstimate leftAtBasis = estimator.wheelAt(App::Wheel::Left, 1000);
  checkNear(leftAtBasis.distance, 100.0f, 1e-6f, "wheelAt(Left, basisTime) == wheelNow (age 0)");

  App::WheelEstimate leftAt3000 = estimator.wheelAt(App::Wheel::Left, 3000);
  checkNear(leftAt3000.distance, 200.0f, 1e-3f,
            "wheelAt(Left, +2000ms): 100 + 50mm/s * 2s == 200mm");
  checkNear(leftAt3000.velocity, 50.0f, 1e-6f, "wheelAt() holds velocity constant under ZOH");
  checkUintEq(leftAt3000.basisTime, 1000,
              "wheelAt()'s returned basisTime stays the ORIGINAL basis time, not the query time");

  App::WheelEstimate rightAt1500 = estimator.wheelAt(App::Wheel::Right, 1500);
  checkNear(rightAt1500.distance, -50.0f, 1e-3f,
            "wheelAt(Right, +500ms): -40 + (-20mm/s * 0.5s) == -50mm");
}

// ===========================================================================
// 2. bodyAt(): straight-line world-frame extrapolation (heading == 0).
// ===========================================================================

void scenarioBodyZohStraightLine() {
  beginScenario("StateEstimator::bodyAt(): straight-line world-frame extrapolation");

  App::StateEstimator estimator;
  App::Telemetry::Frame frame =
      makeFrame(0, 0, 5000, 0, 0, 5000, /*poseX=*/10.0f, /*poseY=*/20.0f, /*poseH=*/0.0f,
                /*twistVx=*/100.0f, /*twistVy=*/0.0f, /*twistOmega=*/0.0f,
                /*otosPresent=*/false);
  estimator.update(frame, /*now=*/5000);

  App::BodyEstimate atBasis = estimator.bodyAt(5000);
  checkTrue(atBasis.valid, "bodyAt() valid after update()");
  checkNear(atBasis.x, 10.0f, 1e-6f, "bodyAt(basisTime) x == basis x (age 0)");
  checkNear(atBasis.y, 20.0f, 1e-6f, "bodyAt(basisTime) y == basis y (age 0)");

  App::BodyEstimate at6000 = estimator.bodyAt(6000);
  checkNear(at6000.x, 110.0f, 1e-3f, "bodyAt(+1000ms): 10 + 100mm/s * 1s == 110mm (heading 0)");
  checkNear(at6000.y, 20.0f, 1e-3f, "bodyAt(+1000ms): y unchanged -- zero heading, zero v_y");
  checkNear(at6000.heading, 0.0f, 1e-6f, "bodyAt(+1000ms): heading unchanged -- omega == 0");
  checkUintEq(at6000.basisTime, 5000, "bodyAt()'s returned basisTime stays the ORIGINAL basis time");
}

// ===========================================================================
// 3. bodyAt(): rotating extrapolation -- basis heading rotates the
//    held-constant body-frame velocity into world frame; heading itself
//    extrapolates via basis.heading + basis.omega * age (headingLead(),
//    generalized).
// ===========================================================================

void scenarioBodyZohRotating() {
  beginScenario("StateEstimator::bodyAt(): rotating extrapolation -- world-frame projection + heading rate");

  const float kPi = 3.14159265358979323846f;
  App::StateEstimator estimator;
  App::Telemetry::Frame frame =
      makeFrame(0, 0, 0, 0, 0, 0, /*poseX=*/0.0f, /*poseY=*/0.0f, /*poseH=*/kPi / 2.0f,
                /*twistVx=*/50.0f, /*twistVy=*/0.0f, /*twistOmega=*/1.0f,
                /*otosPresent=*/false);
  estimator.update(frame, /*now=*/0);

  App::BodyEstimate at1000 = estimator.bodyAt(1000);
  // basis heading == pi/2: cos(pi/2)~=0, sin(pi/2)==1 -- the body-frame
  // forward velocity (50mm/s along +x_body) projects onto WORLD +y.
  checkNear(at1000.x, 0.0f, 1e-3f, "bodyAt(+1s) x: (50*cos(pi/2) - 0*sin(pi/2))*1s ~= 0");
  checkNear(at1000.y, 50.0f, 1e-3f, "bodyAt(+1s) y: (50*sin(pi/2) + 0*cos(pi/2))*1s == 50mm");
  checkNear(at1000.heading, kPi / 2.0f + 1.0f, 1e-6f,
            "bodyAt(+1s) heading == basis.heading + basis.omega * age (headingLead(), generalized)");
}

// ===========================================================================
// 4. whereAmI() == bodyAt(now) exactly; wheelNow() == raw basis, zero
//    extrapolation.
// ===========================================================================

void scenarioWhereAmIAndWheelNow() {
  beginScenario("whereAmI(now) == bodyAt(now); wheelNow() == raw basis (zero extrapolation)");

  App::StateEstimator estimator;
  App::Telemetry::Frame frame =
      makeFrame(30.0f, 15.0f, 2000, 30.0f, 15.0f, 2000, 5.0f, 5.0f, 0.3f, 40.0f, 0.0f, 0.0f,
                /*otosPresent=*/false);
  estimator.update(frame, /*now=*/2000);

  App::BodyEstimate viaWhereAmI = estimator.whereAmI(9000);
  App::BodyEstimate viaBodyAt = estimator.bodyAt(9000);
  checkNear(viaWhereAmI.x, viaBodyAt.x, 1e-9f, "whereAmI(now).x == bodyAt(now).x exactly");
  checkNear(viaWhereAmI.y, viaBodyAt.y, 1e-9f, "whereAmI(now).y == bodyAt(now).y exactly");
  checkNear(viaWhereAmI.heading, viaBodyAt.heading, 1e-9f,
            "whereAmI(now).heading == bodyAt(now).heading exactly");

  App::WheelEstimate viaWheelNow = estimator.wheelNow(App::Wheel::Left);
  checkNear(viaWheelNow.distance, 30.0f, 1e-6f, "wheelNow() returns the raw basis, no extrapolation");
  checkUintEq(viaWheelNow.basisTime, 2000, "wheelNow() basisTime is the raw basis time");
}

// ===========================================================================
// 5. valid semantics: false before the first update(), true after --
//    verified independently for both wheels and the body peer.
// ===========================================================================

void scenarioValidBeforeAndAfterFirstUpdate() {
  beginScenario("valid: false before the first update(), true after -- independent per peer");

  App::StateEstimator estimator;
  checkFalse(estimator.wheelNow(App::Wheel::Left).valid, "wheelNow(Left) invalid before any update()");
  checkFalse(estimator.wheelNow(App::Wheel::Right).valid, "wheelNow(Right) invalid before any update()");
  checkFalse(estimator.wheelAt(App::Wheel::Left, 1000).valid, "wheelAt(Left, t) invalid before any update()");
  checkFalse(estimator.whereAmI(0).valid, "whereAmI() invalid before any update()");
  checkFalse(estimator.bodyAt(0).valid, "bodyAt(t) invalid before any update()");

  App::Telemetry::Frame frame =
      makeFrame(1.0f, 1.0f, 100, 1.0f, 1.0f, 100, 0, 0, 0, 0, 0, 0, /*otosPresent=*/false);
  estimator.update(frame, /*now=*/100);

  checkTrue(estimator.wheelNow(App::Wheel::Left).valid, "wheelNow(Left) valid after update()");
  checkTrue(estimator.wheelNow(App::Wheel::Right).valid, "wheelNow(Right) valid after update()");
  checkTrue(estimator.whereAmI(100).valid, "whereAmI() valid after update()");
}

// ===========================================================================
// 6. reset(): re-anchors ONLY the body peer's world pose -- wheel-peer
//    state (distance/velocity/basisTime/valid) is untouched, and the
//    body's own basisTime/valid are untouched too (mirrors Odometry::
//    reset()'s own teleport semantics -- no `now` argument, the next
//    update() naturally re-baselines).
// ===========================================================================

void scenarioResetReanchorsBodyOnlyWheelsUntouched() {
  beginScenario("reset(): re-anchors body pose only -- wheel peers and body basisTime/valid untouched");

  App::StateEstimator estimator;
  App::Telemetry::Frame frame =
      makeFrame(/*encLeftPos=*/77.0f, /*encLeftVel=*/3.0f, /*encLeftTime=*/1000,
                /*encRightPos=*/-22.0f, /*encRightVel=*/-1.0f, /*encRightTime=*/1000,
                /*poseX=*/1.0f, /*poseY=*/2.0f, /*poseH=*/0.4f, /*twistVx=*/9.0f,
                /*twistVy=*/0.0f, /*twistOmega=*/0.1f, /*otosPresent=*/false);
  estimator.update(frame, /*now=*/1000);

  estimator.reset(100.0f, 200.0f, 1.5f);

  App::BodyEstimate atBasis = estimator.bodyAt(1000);  // age 0 -- pure re-anchored pose
  checkTrue(atBasis.valid, "reset() does not clear an already-true valid flag");
  checkNear(atBasis.x, 100.0f, 1e-6f, "reset() snaps x to the given pose");
  checkNear(atBasis.y, 200.0f, 1e-6f, "reset() snaps y to the given pose");
  checkNear(atBasis.heading, 1.5f, 1e-6f, "reset() snaps heading to the given pose");
  checkUintEq(atBasis.basisTime, 1000, "reset() leaves basisTime untouched -- no `now` argument, by design");
  checkNear(atBasis.v_x, 9.0f, 1e-6f, "reset() leaves v_x untouched");
  checkNear(atBasis.omega, 0.1f, 1e-6f, "reset() leaves omega untouched");

  App::WheelEstimate leftAfterReset = estimator.wheelNow(App::Wheel::Left);
  checkNear(leftAfterReset.distance, 77.0f, 1e-6f, "reset() leaves the LEFT wheel peer completely untouched");
  App::WheelEstimate rightAfterReset = estimator.wheelNow(App::Wheel::Right);
  checkNear(rightAfterReset.distance, -22.0f, 1e-6f, "reset() leaves the RIGHT wheel peer completely untouched");

  // reset() before ANY update() -- still writes the pose, but valid stays
  // false (valid only ever flips true via update(), per this class's own
  // contract -- state_estimator.h's own reset() doc comment).
  App::StateEstimator fresh;
  fresh.reset(9.0f, 9.0f, 9.0f);
  checkFalse(fresh.bodyAt(0).valid, "reset() alone (no prior update()) does NOT make the body peer valid");
}

// ===========================================================================
// 7. innovations(): OTOS-vs-predicted heading/omega residual, computed
//    whenever a fresh OTOS reading is blended -- even at weight 0.0.
// ===========================================================================

void scenarioInnovationsComputedEvenAtWeightZero() {
  beginScenario("innovations(): computed from a fresh OTOS reading even at weight 0.0 (diagnostic only)");

  App::FusionWeights zeroWeights;  // headingOtos=0, omegaOtos=0 -- pure encoder output
  App::StateEstimator estimator(zeroWeights);

  App::Telemetry::Frame frame =
      makeFrame(0, 0, 1000, 0, 0, 1000, 0, 0, /*poseH=*/0.2f, 0, 0, /*twistOmega=*/0.5f,
                /*otosPresent=*/true, /*otosHeading=*/1.0f, /*otosOmega=*/2.0f,
                /*otosTime=*/1000);
  estimator.update(frame, /*now=*/1000);

  App::Innovations innov = estimator.innovations();
  checkTrue(innov.valid, "innovations() valid after a fresh OTOS reading is blended");
  checkNear(innov.heading, 0.8f, 1e-4f, "innovations().heading == otos.heading - predicted heading (1.0 - 0.2)");
  checkNear(innov.omega, 1.5f, 1e-4f, "innovations().omega == otos.omega - predicted omega (2.0 - 0.5)");

  // Weight is 0.0 -- the residual is diagnostic ONLY, never fed back into
  // the body estimate itself.
  App::BodyEstimate body = estimator.whereAmI(1000);
  checkNear(body.heading, 0.2f, 1e-6f, "weight=0.0 -- body heading stays PURE encoder-derived, unaffected by innovations()");
  checkNear(body.omega, 0.5f, 1e-6f, "weight=0.0 -- body omega stays PURE encoder-derived, unaffected by innovations()");
}

// ===========================================================================
// 8. setWeights(): weight=0.0 pure encoder, weight=1.0 pure OTOS (fresh),
//    intermediate weight blends proportionally.
// ===========================================================================

void scenarioSetWeightsPureEncoderPureOtosAndBlend() {
  beginScenario("setWeights(): weight=0.0 pure encoder, weight=1.0 pure OTOS, intermediate blends proportionally");

  App::StateEstimator estimator;  // starts at default FusionWeights{} == 0.0/0.0

  App::Telemetry::Frame frame =
      makeFrame(0, 0, 1000, 0, 0, 1000, 0, 0, /*poseH=*/0.2f, 0, 0, /*twistOmega=*/0.5f,
                /*otosPresent=*/true, /*otosHeading=*/1.0f, /*otosOmega=*/2.0f,
                /*otosTime=*/1000);

  // weight == 0.0 -- pure encoder-derived heading/omega.
  estimator.update(frame, 1000);
  App::BodyEstimate w0 = estimator.whereAmI(1000);
  checkNear(w0.heading, 0.2f, 1e-6f, "weight=0.0 -- heading == pure encoder frame.pose.h");
  checkNear(w0.omega, 0.5f, 1e-6f, "weight=0.0 -- omega == pure encoder frame.twist.omega");

  // weight == 1.0 -- pure OTOS heading/omega (fresh reading).
  estimator.setWeights(App::FusionWeights{/*headingOtos=*/1.0f, /*omegaOtos=*/1.0f, /*staleness=*/200});
  estimator.update(frame, 1000);
  App::BodyEstimate w1 = estimator.whereAmI(1000);
  checkNear(w1.heading, 1.0f, 1e-6f, "weight=1.0 -- heading == pure otos.heading");
  checkNear(w1.omega, 2.0f, 1e-6f, "weight=1.0 -- omega == pure otos.omega");

  // Intermediate weight -- proportional blend: encoder + weight*(otos - encoder).
  estimator.setWeights(App::FusionWeights{/*headingOtos=*/0.25f, /*omegaOtos=*/0.75f, /*staleness=*/200});
  estimator.update(frame, 1000);
  App::BodyEstimate wMid = estimator.whereAmI(1000);
  checkNear(wMid.heading, 0.2f + 0.25f * (1.0f - 0.2f), 1e-5f,
            "intermediate weight -- heading blends proportionally (0.25)");
  checkNear(wMid.omega, 0.5f + 0.75f * (2.0f - 0.5f), 1e-5f,
            "intermediate weight -- omega blends proportionally (0.75)");

  checkTrue(estimator.weights().headingOtos == 0.25f, "weights() reads back the live headingOtos weight");
}

// ===========================================================================
// 9. Staleness gate: an OTOS reading past the live staleness window (or an
//    absent frame.otosPresent) blends nothing and leaves innovations()
//    unchanged from its prior value.
// ===========================================================================

void scenarioStaleOrAbsentOtosBlendsNothing() {
  beginScenario("a stale (past staleness window) or absent OTOS reading blends nothing, leaves innovations() unchanged");

  App::StateEstimator estimator(App::FusionWeights{/*headingOtos=*/1.0f, /*omegaOtos=*/1.0f,
                                                     /*staleness=*/50});

  // First: a genuinely fresh OTOS reading DOES blend and sets innovations().
  App::Telemetry::Frame freshFrame =
      makeFrame(0, 0, 1000, 0, 0, 1000, 0, 0, /*poseH=*/0.2f, 0, 0, /*twistOmega=*/0.5f,
                /*otosPresent=*/true, /*otosHeading=*/1.0f, /*otosOmega=*/2.0f,
                /*otosTime=*/1000);
  estimator.update(freshFrame, /*now=*/1000);
  App::Innovations afterFresh = estimator.innovations();
  checkTrue(afterFresh.valid, "setup: a fresh OTOS reading blends and sets innovations()");
  checkNear(estimator.whereAmI(1000).heading, 1.0f, 1e-6f, "setup: weight=1.0 fresh OTOS -- pure OTOS heading");

  // Next cycle: frame.otosPresent is FALSE this cycle (no fresh burst) --
  // heading/omega revert to pure encoder-derived, and innovations() keeps
  // its PRIOR value (mirrors Telemetry's own "last staged snapshot" -- see
  // Innovations' own doc comment).
  App::Telemetry::Frame noOtosFrame =
      makeFrame(0, 0, 1020, 0, 0, 1020, 0, 0, /*poseH=*/0.3f, 0, 0, /*twistOmega=*/0.6f,
                /*otosPresent=*/false);
  estimator.update(noOtosFrame, /*now=*/1020);
  checkNear(estimator.whereAmI(1020).heading, 0.3f, 1e-6f,
            "otosPresent=false this cycle -- heading falls back to pure encoder-derived");
  App::Innovations afterAbsent = estimator.innovations();
  checkNear(afterAbsent.heading, afterFresh.heading, 1e-6f, "innovations() unchanged when this cycle has no fresh OTOS reading");

  // Next cycle: frame.otosPresent is TRUE but the reading's age exceeds
  // the live staleness window (now - otos.time > staleness=50ms) --
  // treated the SAME as absent: no blend, innovations() unchanged again.
  App::Telemetry::Frame staleFrame =
      makeFrame(0, 0, 1200, 0, 0, 1200, 0, 0, /*poseH=*/0.4f, 0, 0, /*twistOmega=*/0.7f,
                /*otosPresent=*/true, /*otosHeading=*/5.0f, /*otosOmega=*/6.0f,
                /*otosTime=*/1000);  // 200ms old at now=1200, staleness window is 50ms
  estimator.update(staleFrame, /*now=*/1200);
  checkNear(estimator.whereAmI(1200).heading, 0.4f, 1e-6f,
            "a stale OTOS reading (age > staleness) blends nothing -- heading stays pure encoder-derived");
  App::Innovations afterStale = estimator.innovations();
  checkNear(afterStale.heading, afterFresh.heading, 1e-6f, "innovations() unchanged when this cycle's OTOS reading is stale");
}

}  // namespace

int main() {
  scenarioWheelZohExtrapolation();
  scenarioBodyZohStraightLine();
  scenarioBodyZohRotating();
  scenarioWhereAmIAndWheelNow();
  scenarioValidBeforeAndAfterFirstUpdate();
  scenarioResetReanchorsBodyOnlyWheelsUntouched();
  scenarioInnovationsComputedEvenAtWeightZero();
  scenarioSetWeightsPureEncoderPureOtosAndBlend();
  scenarioStaleOrAbsentOtosBlendsNothing();

  if (g_failureCount == 0) {
    std::printf("OK: all App::StateEstimator scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the App::StateEstimator scenarios\n", g_failureCount);
  return 1;
}
