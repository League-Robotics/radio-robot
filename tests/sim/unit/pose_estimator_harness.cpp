// pose_estimator_harness.cpp — off-hardware acceptance harness for ticket
// 082-002 (SUC-002): exercises Subsystems::PoseEstimator
// (source/subsystems/pose_estimator.{h,cpp}) — encoder-only dead-reckoning
// plus EkfTiny (ticket 001) fusion — in isolation, with synthetic
// msg::MotorState/msg::PoseEstimate observations (no real HAL, no CMake, no
// ARM toolchain).
//
// Per ekf_tiny_harness.cpp's precedent (ticket 001) and the project's wider
// motor_policy_harness.cpp / velocity_pid_harness.cpp convention (078-004 /
// 081-001), this #includes only its own translation units so it compiles
// with the plain system C++ compiler, as long as libraries/tinyekf/ is also
// on the include path (tinyekf.h is header-only).
//
// Three required scenarios (see ticket 082-002's Acceptance Criteria):
//   (a) otosObs always nullptr -> fusedPose() equals encoderPose() exactly
//       at every tick (no correction applied when there is nothing to
//       correct with).
//   (b) a synthetic otosObs diverging from the encoder-only path ->
//       fusedPose() differs measurably from encoderPose() after several
//       ticks (proves the correction step actually executes).
//   (c) a DrivetrainConfig with all four EKF noise fields at 0.0f still
//       produces a finite, non-NaN, non-degenerate fusedPose() after several
//       predict+correct ticks (the zero-as-unset sentinel-default fallback
//       prevents the degenerate Q=0, R=0 case — proven here by requiring the
//       correction to still measurably pull fusedPose() toward the offset
//       OTOS observation, not just by checking finiteness: a literal
//       Q=0,R=0 EKF would leave P singular and updatePosition()/
//       updateHeading() would silently no-op forever, which
//       std::isfinite() alone would NOT catch).
//
// Plain C++ program, hand-rolled assertions (mirrors ekf_tiny_harness.cpp's
// shape) — prints a PASS/FAIL line per scenario and exits nonzero if any
// assertion failed.
//
// Verification command (see ticket 082-002's Testing plan):
//   c++ -std=c++11 -Wall -Wextra \
//       -I source -I libraries/tinyekf \
//       -o /tmp/pose_estimator_harness \
//       tests/sim/unit/pose_estimator_harness.cpp \
//       source/subsystems/pose_estimator.cpp source/estimation/ekf_tiny.cpp
//   /tmp/pose_estimator_harness

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "runtime/commands.h"
#include "runtime/queue.h"
#include "subsystems/pose_estimator.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors ekf_tiny_harness.cpp) ---

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

// Asserts |actual - expected| <= tol. tol == 0.0f is an exact (bit-for-bit,
// modulo IEEE754 +/-0) equality check.
void checkNear(float actual, float expected, float tol, const std::string& what) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s — expected %.9g (tol %.3g), got %.9g",
                  what.c_str(), static_cast<double>(expected),
                  static_cast<double>(tol), static_cast<double>(actual));
    fail(buf);
  }
}

// --- Synthetic observation builders -------------------------------------

// motorStateAt — a connected MotorState reporting only a cumulative wheel
// position (the only field PoseEstimator::tick() reads).
msg::MotorState motorStateAt(float position) {
  msg::MotorState state;
  state.connected = true;
  state.position.has = true;
  state.position.val = position;
  return state;
}

// otosAt — a fresh (stamp.valid = true) odometer observation at the given
// pose.
msg::PoseEstimate otosAt(float x, float y, float h, uint32_t now) {
  msg::PoseEstimate obs;
  obs.pose.x = x;
  obs.pose.y = y;
  obs.pose.h = h;
  obs.stamp.valid = true;
  obs.stamp.last_upd = now;
  return obs;
}

// makeConfig — a DrivetrainConfig carrying only the six fields
// PoseEstimator::configure() reads.
msg::DrivetrainConfig makeConfig(float trackwidth, float rotationalSlip,
                                  float qXy, float qTheta, float rOtosXy,
                                  float rOtosTheta) {
  msg::DrivetrainConfig cfg;
  cfg.trackwidth = trackwidth;
  cfg.rotational_slip = rotationalSlip;
  cfg.ekf_q_xy = qXy;
  cfg.ekf_q_theta = qTheta;
  cfg.ekf_r_otos_xy = rOtosXy;
  cfg.ekf_r_otos_theta = rOtosTheta;
  return cfg;
}

// --- Scenarios ------------------------------------------------------------

// (a) otosObs always nullptr: fusedPose() must equal encoderPose() exactly
// (not just "close") at every tick, across a varied multi-tick sequence
// (straight run, a turn each way, another straight run) — proving predict()
// alone reproduces the encoder-only accumulator's own arc-integration
// arithmetic bit-for-bit, and that no correction is silently sneaking in.
void scenarioNoOtosFusedMatchesEncoderExactly() {
  beginScenario(
      "otosObs always nullptr -> fusedPose() equals encoderPose() exactly "
      "every tick");

  Subsystems::PoseEstimator pe;
  pe.configure(makeConfig(/*trackwidth=*/128.0f, /*rotationalSlip=*/0.92f,
                           /*qXy=*/800.0f, /*qTheta=*/4.0f,
                           /*rOtosXy=*/50.0f, /*rOtosTheta=*/0.01f));

  // 087-004: tick() gained a poseResetIn parameter (Rt::WorkQueue<
  // Rt::PoseResetCommand,4>, source/runtime/commands.h) -- never posted to
  // in this scenario, so it is a no-op, matching today's exact behavior
  // (see the queue-driven scenarios below for coverage of a non-empty one).
  Rt::WorkQueue<Rt::PoseResetCommand, 4> poseResetIn;
  Rt::Mailbox<msg::SetPose> otosSetPoseOut;

  struct Step {
    float dLeft;
    float dRight;
  };
  const Step steps[] = {
      {40.0f, 40.0f},   // straight
      {30.0f, 50.0f},   // gentle turn
      {50.0f, 20.0f},   // turn the other way
      {45.0f, 45.0f},   // straight again
      {0.0f, 60.0f},    // sharp turn
  };

  float cumLeft = 0.0f;
  float cumRight = 0.0f;
  uint32_t now = 0;
  int tickIndex = 0;
  for (const Step& s : steps) {
    now += 20;
    cumLeft += s.dLeft;
    cumRight += s.dRight;
    pe.tick(now, motorStateAt(cumLeft), motorStateAt(cumRight), nullptr, poseResetIn, otosSetPoseOut);

    msg::PoseEstimate enc = pe.encoderPose();
    msg::PoseEstimate fused = pe.fusedPose();

    char label[96];
    std::snprintf(label, sizeof(label), "tick %d", tickIndex++);

    checkTrue(fused.pose.x == enc.pose.x,
              std::string(label) + ": fusedPose().pose.x == encoderPose().pose.x");
    checkTrue(fused.pose.y == enc.pose.y,
              std::string(label) + ": fusedPose().pose.y == encoderPose().pose.y");
    checkTrue(fused.pose.h == enc.pose.h,
              std::string(label) + ": fusedPose().pose.h == encoderPose().pose.h");
  }

  // Sanity: the sequence actually moved the robot (not a trivially-passing
  // all-zero test).
  msg::PoseEstimate finalEnc = pe.encoderPose();
  checkTrue(finalEnc.pose.x > 50.0f || finalEnc.pose.y > 50.0f ||
                std::fabs(finalEnc.pose.h) > 0.01f,
            "sanity: the synthetic drive sequence actually produced motion");
}

// (b) a synthetic otosObs persistently offset ahead of the encoder-only
// path in x: fusedPose() must diverge measurably from encoderPose() after
// several correction ticks — proving the correction step actually executes
// (and keeps executing) when an odometer is present.
void scenarioOtosDivergesFusedFromEncoder() {
  beginScenario(
      "synthetic OTOS observation pulls fusedPose() measurably away from "
      "encoderPose()");

  Subsystems::PoseEstimator pe;
  pe.configure(makeConfig(128.0f, 0.92f, /*qXy=*/800.0f, /*qTheta=*/4.0f,
                           /*rOtosXy=*/50.0f, /*rOtosTheta=*/0.01f));

  // 087-004: never posted to in this scenario -- see scenario (a)'s comment.
  Rt::WorkQueue<Rt::PoseResetCommand, 4> poseResetIn;
  Rt::Mailbox<msg::SetPose> otosSetPoseOut;

  uint32_t now = 0;
  float cumLeft = 0.0f;
  float cumRight = 0.0f;

  // Seed: a few straight ticks with no OTOS so the EKF's P has grown off
  // zero (P starts at zero out of init() -- a zero-P Kalman gain would make
  // the correction step below a no-op for the wrong reason).
  for (int i = 0; i < 5; ++i) {
    now += 20;
    cumLeft += 40.0f;
    cumRight += 40.0f;
    pe.tick(now, motorStateAt(cumLeft), motorStateAt(cumRight), nullptr, poseResetIn, otosSetPoseOut);
  }

  msg::PoseEstimate encBefore = pe.encoderPose();
  msg::PoseEstimate fusedBefore = pe.fusedPose();
  checkTrue(fusedBefore.pose.x == encBefore.pose.x,
            "sanity: fusedPose() still matches encoderPose() before any "
            "OTOS observation");

  // Feed a run of OTOS observations persistently offset ahead of the
  // encoder-only path in x.
  //
  // 099-006: this offset was 150mm before ticket 006 added EkfTiny's
  // bounded innovation-consistency gate (D4). 150mm is now a genuinely-
  // shifted-sensor-class disagreement (d^2 far past the chi-square
  // threshold at this scenario's noise level) that the gate correctly
  // rejects tick after tick within these 8 ticks -- proving the NEW,
  // correct behavior, not the OLD unconditional-accept one. 60mm keeps
  // this scenario testing what it always meant to test ("the correction
  // step actually executes") by staying inside the gate at every one of
  // these 8 ticks (d^2 <= ~2.7, comfortably under the 9.21 threshold), and
  // still produces a >10mm measurable divergence (see the assertion below)
  // once the one-tick lag between `refEnc` and the correction settles out.
  const float kOffsetX = 60.0f;  // [mm]
  for (int i = 0; i < 8; ++i) {
    now += 20;
    cumLeft += 40.0f;
    cumRight += 40.0f;

    // Offset is computed from the PRE-tick encoderPose() (i.e. the previous
    // tick's pure dead-reckoning pose) -- a synthetic sensor that reads
    // consistently kOffsetX ahead of where the wheels alone say the robot
    // is.
    msg::PoseEstimate refEnc = pe.encoderPose();
    msg::PoseEstimate otos =
        otosAt(refEnc.pose.x + kOffsetX, refEnc.pose.y, refEnc.pose.h, now);

    pe.tick(now, motorStateAt(cumLeft), motorStateAt(cumRight), &otos, poseResetIn, otosSetPoseOut);
  }

  msg::PoseEstimate encAfter = pe.encoderPose();
  msg::PoseEstimate fusedAfter = pe.fusedPose();

  checkTrue(fusedAfter.pose.x > encAfter.pose.x + 10.0f,
            "fusedPose().pose.x is measurably ahead of encoderPose().pose.x "
            "after repeated OTOS correction (proves the correction step "
            "executes when an odometer is present)");
}

// (c) a DrivetrainConfig with all four EKF noise fields at 0.0f: the
// zero-as-unset sentinel must substitute non-zero fallback defaults before
// EkfTiny::init(), so fusedPose() both (i) stays finite/non-NaN and (ii)
// still gets measurably corrected by a fresh OTOS observation exactly like
// scenario (b) above -- a literal Q=0,R=0 EKF would leave P singular
// forever, silently no-op'ing every updatePosition()/updateHeading() call
// (finite, but degenerate: fusedPose() would just equal the uncorrected
// dead-reckoning path). Requiring the SAME measurable-divergence proof as
// scenario (b) is what actually exercises the fallback, not just its
// absence of a crash.
void scenarioZeroConfigSentinelKeepsFusionFiniteAndCorrected() {
  beginScenario(
      "all-zero EKF config fields still produce a finite, corrected "
      "fusedPose() via the sentinel-default fallback");

  Subsystems::PoseEstimator pe;
  pe.configure(makeConfig(128.0f, 0.92f, /*qXy=*/0.0f, /*qTheta=*/0.0f,
                           /*rOtosXy=*/0.0f, /*rOtosTheta=*/0.0f));

  // 087-004: never posted to in this scenario -- see scenario (a)'s comment.
  Rt::WorkQueue<Rt::PoseResetCommand, 4> poseResetIn;
  Rt::Mailbox<msg::SetPose> otosSetPoseOut;

  uint32_t now = 0;
  float cumLeft = 0.0f;
  float cumRight = 0.0f;

  for (int i = 0; i < 5; ++i) {
    now += 20;
    cumLeft += 40.0f;
    cumRight += 40.0f;
    pe.tick(now, motorStateAt(cumLeft), motorStateAt(cumRight), nullptr, poseResetIn, otosSetPoseOut);
  }

  // 099-006: see scenario (b)'s matching comment -- this offset was 150mm
  // before ticket 006's gate; 60mm stays inside it at every tick (same
  // noise config as (b): kDefaultQXy/kDefaultQTheta/kDefaultROtosXy/
  // kDefaultROtosTheta are the sentinel-default fallback for this
  // all-zero-config scenario, numerically identical to (b)'s explicit
  // 800/4/50/0.01).
  const float kOffsetX = 60.0f;  // [mm]
  for (int i = 0; i < 8; ++i) {
    now += 20;
    cumLeft += 40.0f;
    cumRight += 40.0f;

    msg::PoseEstimate refEnc = pe.encoderPose();
    msg::PoseEstimate otos =
        otosAt(refEnc.pose.x + kOffsetX, refEnc.pose.y, refEnc.pose.h, now);

    pe.tick(now, motorStateAt(cumLeft), motorStateAt(cumRight), &otos, poseResetIn, otosSetPoseOut);
  }

  msg::PoseEstimate enc = pe.encoderPose();
  msg::PoseEstimate fused = pe.fusedPose();

  checkTrue(std::isfinite(fused.pose.x) && std::isfinite(fused.pose.y) &&
                std::isfinite(fused.pose.h),
            "fusedPose() stays finite after several predict+correct ticks "
            "with all-zero EKF config");
  checkTrue(std::isfinite(enc.pose.x) && std::isfinite(enc.pose.y) &&
                std::isfinite(enc.pose.h),
            "encoderPose() stays finite too");

  checkTrue(fused.pose.x > enc.pose.x + 10.0f,
            "fusedPose() is still measurably corrected toward the offset "
            "OTOS observation despite an all-zero config -- proves the "
            "sentinel-default fallback substituted non-zero Q/R (a literal "
            "Q=0,R=0 EKF would leave P singular and silently no-op every "
            "correction, leaving fused == encoder)");
}

// (d) 087-004 AC1: configure()/config() round-trip a DrivetrainConfig value
// verbatim -- kills the config-shadow this sprint's design removes
// elsewhere (a caller, e.g. the Configurator, ticket 005, can read back
// what was configured without a separate cache).
void scenarioConfigureConfigRoundTrip() {
  beginScenario("configure()/config() round-trip a DrivetrainConfig value (087-004)");

  Subsystems::PoseEstimator pe;
  msg::DrivetrainConfig cfg = makeConfig(/*trackwidth=*/150.0f, /*rotationalSlip=*/0.85f,
                                          /*qXy=*/500.0f, /*qTheta=*/3.0f,
                                          /*rOtosXy=*/40.0f, /*rOtosTheta=*/0.02f);
  pe.configure(cfg);

  msg::DrivetrainConfig readBack = pe.config();
  checkNear(readBack.trackwidth, cfg.trackwidth, 0.0f, "config() returns the configured trackwidth");
  checkNear(readBack.rotational_slip, cfg.rotational_slip, 0.0f,
            "config() returns the configured rotational_slip");
  checkNear(readBack.ekf_q_xy, cfg.ekf_q_xy, 0.0f, "config() returns the configured ekf_q_xy");
  checkNear(readBack.ekf_q_theta, cfg.ekf_q_theta, 0.0f, "config() returns the configured ekf_q_theta");
  checkNear(readBack.ekf_r_otos_xy, cfg.ekf_r_otos_xy, 0.0f,
            "config() returns the configured ekf_r_otos_xy");
  checkNear(readBack.ekf_r_otos_theta, cfg.ekf_r_otos_theta, 0.0f,
            "config() returns the configured ekf_r_otos_theta");
}

// (e) 087-004 AC5: posting a kSetPose Rt::PoseResetCommand to poseResetIn and
// ticking dispatches to the EXISTING setPose() -- same phantom-jump-free
// re-anchor contract setPose() already documents (pose_estimator.h): both
// encoderPose() and fusedPose() jump cleanly to the commanded pose, and
// prevEncLeft_/prevEncRight_/haveEncBaseline_ (the encoder-delta baseline)
// are left untouched, so a FOLLOWING tick's motion continues smoothly off
// the re-anchored pose rather than fabricating a jump of its own.
void scenarioPoseResetInDrainsKSetPoseMatchesDirectSetPose() {
  beginScenario(
      "poseResetIn: kSetPose drains to setPose(), re-anchoring both readings "
      "with no phantom jump (087-004)");

  Subsystems::PoseEstimator pe;
  pe.configure(makeConfig(128.0f, 0.92f, 800.0f, 4.0f, 50.0f, 0.01f));

  Rt::WorkQueue<Rt::PoseResetCommand, 4> poseResetIn;
  Rt::Mailbox<msg::SetPose> otosSetPoseOut;

  uint32_t now = 0;
  float cumLeft = 0.0f;
  float cumRight = 0.0f;
  for (int i = 0; i < 5; ++i) {
    now += 20;
    cumLeft += 40.0f;
    cumRight += 40.0f;
    pe.tick(now, motorStateAt(cumLeft), motorStateAt(cumRight), nullptr, poseResetIn, otosSetPoseOut);
  }

  // SI arrives: post kSetPose re-anchoring to a known world pose, delivered
  // at the SAME `now` as the last real tick with the SAME cumulative
  // encoder reading (no wheel motion between the command's arrival and this
  // tick) -- setPose() re-anchors ONLY the believed pose, deliberately
  // leaving prevEncLeft_/prevEncRight_/haveEncBaseline_ untouched (see that
  // method's own doc comment), so this tick's own encoder delta is exactly
  // zero regardless.
  msg::SetPose target;
  target.x = 500.0f;
  target.y = -200.0f;
  target.h = 1.0f;
  Rt::PoseResetCommand setPoseCmd;
  setPoseCmd.kind = Rt::PoseResetCommand::kSetPose;
  setPoseCmd.pose = target;
  checkTrue(poseResetIn.post(setPoseCmd), "post() succeeds");

  pe.tick(now, motorStateAt(cumLeft), motorStateAt(cumRight), nullptr, poseResetIn, otosSetPoseOut);
  checkTrue(poseResetIn.empty(), "tick() drained the posted kSetPose command");

  msg::PoseEstimate afterReanchor = pe.encoderPose();
  checkNear(afterReanchor.pose.x, target.x, 0.0f,
            "encoderPose().pose.x re-anchored to exactly the commanded SetPose (zero wheel motion this pass)");
  checkNear(afterReanchor.pose.y, target.y, 0.0f,
            "encoderPose().pose.y re-anchored to exactly the commanded SetPose");
  checkNear(afterReanchor.pose.h, target.h, 1e-5f,
            "encoderPose().pose.h re-anchored to (wrapped) the commanded SetPose");

  msg::PoseEstimate fusedAfterReanchor = pe.fusedPose();
  checkNear(fusedAfterReanchor.pose.x, target.x, 0.0f,
            "fusedPose().pose.x re-anchored too (EkfTiny::setPose())");
  checkNear(fusedAfterReanchor.pose.y, target.y, 0.0f, "fusedPose().pose.y re-anchored too");
  checkNear(fusedAfterReanchor.pose.h, target.h, 1e-4f, "fusedPose().pose.h re-anchored too");

  // A further tick with real new motion off the RE-ANCHORED pose produces a
  // normal, bounded delta relative to the commanded pose -- proves
  // prevEncLeft_/prevEncRight_ were left untouched (continuous tracking)
  // while encX_/encY_/encTheta_ jumped cleanly to the commanded pose (no
  // phantom jump on the FOLLOWING tick either).
  now += 20;
  cumLeft += 40.0f;
  cumRight += 40.0f;
  pe.tick(now, motorStateAt(cumLeft), motorStateAt(cumRight), nullptr, poseResetIn, otosSetPoseOut);
  msg::PoseEstimate afterFreshMotion = pe.encoderPose();
  checkTrue(afterFreshMotion.pose.x > afterReanchor.pose.x - 1.0f &&
                afterFreshMotion.pose.x < afterReanchor.pose.x + 60.0f,
            "motion off the re-anchored pose advances encoderPose() by a normal, "
            "bounded delta -- no phantom jump on the tick following the re-anchor");
}

// (f) 087-004 AC5: posting a kResetBaseline Rt::PoseResetCommand to
// poseResetIn and ticking dispatches to the EXISTING resetEncoderBaseline()
// -- preserving that method's deferred dt>0 phantom-jump guard exactly
// (pose_estimator.h's doc comment): a same-pass (dt==0) tick after posting
// must NOT apply the reset yet (it stays armed), and the LATER,
// genuinely-time-advancing tick where a staged hardware encoder zero has
// landed (the encoder reading snapping from a large cumulative value to 0)
// must produce ZERO delta, not a large negative phantom jump.
void scenarioPoseResetInDrainsKResetBaselineNoPhantomJump() {
  beginScenario(
      "poseResetIn: kResetBaseline drains to resetEncoderBaseline(), "
      "preserving the deferred dt>0 phantom-jump guard (087-004)");

  Subsystems::PoseEstimator pe;
  pe.configure(makeConfig(128.0f, 0.92f, 800.0f, 4.0f, 50.0f, 0.01f));

  Rt::WorkQueue<Rt::PoseResetCommand, 4> poseResetIn;
  Rt::Mailbox<msg::SetPose> otosSetPoseOut;

  uint32_t now = 0;
  float cumLeft = 0.0f;
  float cumRight = 0.0f;
  // Build up real motion across a few ticks so there's a nonzero baseline
  // (a large cumulative encoder value) to rebaseline away from.
  for (int i = 0; i < 5; ++i) {
    now += 20;
    cumLeft += 40.0f;
    cumRight += 40.0f;
    pe.tick(now, motorStateAt(cumLeft), motorStateAt(cumRight), nullptr, poseResetIn, otosSetPoseOut);
  }
  msg::PoseEstimate beforeReset = pe.encoderPose();

  // ZERO enc arrives: post kResetBaseline. The hardware encoder zero
  // (Hal::Motor::resetPosition()) is itself STAGED -- it has not landed
  // yet, so THIS SAME-PASS tick() (dt == 0, same `now`, encoder reading
  // still the STALE pre-reset cumulative value) must NOT apply the reset
  // immediately -- it must stay armed.
  Rt::PoseResetCommand resetCmd;
  resetCmd.kind = Rt::PoseResetCommand::kResetBaseline;
  checkTrue(poseResetIn.post(resetCmd), "post() succeeds");

  pe.tick(now, motorStateAt(cumLeft), motorStateAt(cumRight), nullptr, poseResetIn, otosSetPoseOut);   // dt == 0 (same now)
  checkTrue(poseResetIn.empty(), "tick() drained the posted kResetBaseline command");

  msg::PoseEstimate afterSamePassTick = pe.encoderPose();
  checkNear(afterSamePassTick.pose.x, beforeReset.pose.x, 0.0f,
            "same-pass (dt==0) tick after posting kResetBaseline leaves encoderPose() "
            "unchanged -- reset still armed, not yet applied");
  checkNear(afterSamePassTick.pose.y, beforeReset.pose.y, 0.0f, "encoderPose().pose.y unchanged too");
  checkNear(afterSamePassTick.pose.h, beforeReset.pose.h, 0.0f, "encoderPose().pose.h unchanged too");

  // Next tick: a GENUINELY time-advancing pass (dt > 0) where the staged
  // hardware reset has now landed -- the encoder reading snaps to 0 (a huge
  // absolute change from the stale cumulative value). If the reset were
  // applied eagerly (the bug this mechanism prevents), this pass would diff
  // the fresh zero against the STALE baseline and fabricate a large phantom
  // jump. With the deferred guard, this pass's own delta is treated as zero
  // motion (the first reading after a fresh baseline).
  now += 20;
  pe.tick(now, motorStateAt(0.0f), motorStateAt(0.0f), nullptr, poseResetIn, otosSetPoseOut);
  msg::PoseEstimate afterRebaselineTick = pe.encoderPose();
  checkNear(afterRebaselineTick.pose.x, afterSamePassTick.pose.x, 0.0f,
            "the rebaseline-landing tick produces ZERO delta -- no phantom jump "
            "despite the encoder reading jumping from a large cumulative value to 0");
  checkNear(afterRebaselineTick.pose.y, afterSamePassTick.pose.y, 0.0f, "no phantom jump in y either");
  checkNear(afterRebaselineTick.pose.h, afterSamePassTick.pose.h, 0.0f, "no phantom jump in heading either");

  // A further tick with real new motion off the FRESH (rezeroed) baseline
  // produces a normal, bounded delta -- proves the class is still tracking
  // correctly afterward, not just frozen.
  now += 20;
  pe.tick(now, motorStateAt(40.0f), motorStateAt(40.0f), nullptr, poseResetIn, otosSetPoseOut);
  msg::PoseEstimate afterFreshMotion = pe.encoderPose();
  checkTrue(afterFreshMotion.pose.x > afterRebaselineTick.pose.x,
            "motion off the fresh rezeroed baseline advances encoderPose() normally");
}

// (g) 099-004: otosSetPoseOut is posted EXACTLY ONCE per applied kSetPose
// (never on an ordinary tick, never on a kResetBaseline), and lastPoseStep()
// reports the correct |Δp|/|Δθ| magnitude of a known setPose() re-anchor,
// resetting to {0, 0} on every other tick (including the very next one).
void scenarioOtosSetPoseOutAndLastPoseStepMagnitude() {
  beginScenario(
      "otosSetPoseOut posted exactly once per applied kSetPose (never on "
      "kResetBaseline); lastPoseStep() reports the correct magnitude for a "
      "known setPose() and zero on every other tick (099-004)");

  Subsystems::PoseEstimator pe;
  pe.configure(makeConfig(128.0f, 0.92f, 800.0f, 4.0f, 50.0f, 0.01f));

  Rt::WorkQueue<Rt::PoseResetCommand, 4> poseResetIn;
  Rt::Mailbox<msg::SetPose> otosSetPoseOut;

  uint32_t now = 0;
  float cumLeft = 0.0f;
  float cumRight = 0.0f;

  // A handful of ordinary ticks (a mild turn, no queued reset): both new
  // signals must stay at their inert defaults every single tick.
  for (int i = 0; i < 5; ++i) {
    now += 20;
    cumLeft += 40.0f;
    cumRight += 55.0f;
    pe.tick(now, motorStateAt(cumLeft), motorStateAt(cumRight), nullptr, poseResetIn, otosSetPoseOut);
    checkTrue(otosSetPoseOut.empty(), "otosSetPoseOut stays empty when no reset is queued");
    msg::PoseStep step = pe.lastPoseStep();
    checkNear(step.pos, 0.0f, 0.0f, "lastPoseStep().pos is zero on an ordinary tick");
    checkNear(step.theta, 0.0f, 0.0f, "lastPoseStep().theta is zero on an ordinary tick");
  }

  // kSetPose: re-anchor to a known target, same encoder reading (no wheel
  // motion this pass) so this tick's own encoder-delta step contributes
  // nothing further to fusedPose() beyond the reset itself -- the SAME
  // "zero wheel motion this pass" setup scenario (e) above uses. `before` is
  // read directly off the live object (not hand-derived) so the expected
  // magnitudes below hold regardless of the exact accumulated heading;
  // target.h is chosen close enough to `before.pose.h` that the delta never
  // needs wrap-around handling (kept well under +/-pi).
  msg::PoseEstimate before = pe.fusedPose();
  msg::SetPose target;
  target.x = before.pose.x + 321.0f;
  target.y = before.pose.y - 87.0f;
  target.h = before.pose.h + 0.3f;

  Rt::PoseResetCommand cmd;
  cmd.kind = Rt::PoseResetCommand::kSetPose;
  cmd.pose = target;
  checkTrue(poseResetIn.post(cmd), "post() succeeds");

  float expectedDx = target.x - before.pose.x;
  float expectedDy = target.y - before.pose.y;
  float expectedPos = std::sqrt(expectedDx * expectedDx + expectedDy * expectedDy);
  float expectedTheta = std::fabs(target.h - before.pose.h);

  pe.tick(now, motorStateAt(cumLeft), motorStateAt(cumRight), nullptr, poseResetIn, otosSetPoseOut);

  checkTrue(poseResetIn.empty(), "tick() drained the posted kSetPose command");
  checkTrue(!otosSetPoseOut.empty(), "otosSetPoseOut received exactly one post for the applied kSetPose");
  msg::SetPose posted = otosSetPoseOut.take();
  checkTrue(otosSetPoseOut.empty(), "otosSetPoseOut.take() drained the one posted value -- posted exactly once");
  checkNear(posted.x, target.x, 0.0f, "otosSetPoseOut carries the re-anchored fusedPose().pose.x");
  checkNear(posted.y, target.y, 0.0f, "otosSetPoseOut carries the re-anchored fusedPose().pose.y");
  checkNear(posted.h, target.h, 1e-4f, "otosSetPoseOut carries the re-anchored fusedPose().pose.h");

  msg::PoseStep step = pe.lastPoseStep();
  checkNear(step.pos, expectedPos, 1e-3f, "lastPoseStep().pos matches the SI re-anchor's position magnitude");
  checkNear(step.theta, expectedTheta, 1e-4f, "lastPoseStep().theta matches the SI re-anchor's heading magnitude");

  // The VERY NEXT tick, with no queued reset: lastPoseStep_ resets to {0, 0}
  // at the top of every tick() call -- it must not keep echoing the
  // previous tick's step, and otosSetPoseOut must not be posted to again.
  now += 20;
  cumLeft += 40.0f;
  cumRight += 40.0f;
  pe.tick(now, motorStateAt(cumLeft), motorStateAt(cumRight), nullptr, poseResetIn, otosSetPoseOut);
  checkTrue(otosSetPoseOut.empty(),
            "otosSetPoseOut stays empty the tick after the reset -- posted exactly once, not every tick");
  msg::PoseStep stepAfter = pe.lastPoseStep();
  checkNear(stepAfter.pos, 0.0f, 0.0f, "lastPoseStep() resets to zero on the tick following the applied reset");
  checkNear(stepAfter.theta, 0.0f, 0.0f, "lastPoseStep().theta resets to zero too");

  // kResetBaseline: must NOT post to otosSetPoseOut and must NOT produce a
  // nonzero lastPoseStep() (no PoseStep, no otosSetPoseOut post -- ticket
  // 099-004's AC).
  Rt::PoseResetCommand resetCmd;
  resetCmd.kind = Rt::PoseResetCommand::kResetBaseline;
  checkTrue(poseResetIn.post(resetCmd), "post() succeeds");
  now += 20;
  pe.tick(now, motorStateAt(cumLeft), motorStateAt(cumRight), nullptr, poseResetIn, otosSetPoseOut);
  checkTrue(poseResetIn.empty(), "tick() drained the posted kResetBaseline command");
  checkTrue(otosSetPoseOut.empty(), "kResetBaseline never posts to otosSetPoseOut");
  msg::PoseStep stepReset = pe.lastPoseStep();
  checkNear(stepReset.pos, 0.0f, 0.0f, "lastPoseStep().pos stays zero on a kResetBaseline-only tick");
  checkNear(stepReset.theta, 0.0f, 0.0f, "lastPoseStep().theta stays zero too");
}

}  // namespace

int main() {
  scenarioNoOtosFusedMatchesEncoderExactly();
  scenarioOtosDivergesFusedFromEncoder();
  scenarioZeroConfigSentinelKeepsFusionFiniteAndCorrected();
  scenarioConfigureConfigRoundTrip();
  scenarioPoseResetInDrainsKSetPoseMatchesDirectSetPose();
  scenarioPoseResetInDrainsKResetBaselineNoPhantomJump();
  scenarioOtosSetPoseOutAndLastPoseStepMagnitude();

  if (g_failureCount == 0) {
    std::printf("OK: all PoseEstimator scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the PoseEstimator scenarios\n",
              g_failureCount);
  return 1;
}
