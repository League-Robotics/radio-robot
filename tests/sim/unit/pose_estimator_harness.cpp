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

// motorStateAt — a connected MotorState reporting a cumulative wheel
// position plus (099-005) the sampled_at timestamp PoseEstimator::tick()'s
// paired-freshness gate reads. Every scenario in this file predating 099-005
// passes sampledAt == the SAME `now` as the enclosing tick() call at every
// one of its call sites — i.e. both wheels are always synchronously fresh,
// exactly the "tick cadence == flip-flop cadence" world the pre-fix code
// implicitly assumed — so the paired-freshness gate fires every tick for
// them and their expected results are unaffected by this ticket. See
// scenarioStaggeredSampleTimingMatchesSynchronousTotalNoLocalMisattribution()
// below for the new staggered-timing (sampledAt != now) coverage this
// ticket adds.
msg::MotorState motorStateAt(float position, uint32_t sampledAt) {
  msg::MotorState state;
  state.connected = true;
  state.position.has = true;
  state.position.val = position;
  state.sampled_at.has = true;
  state.sampled_at.val = sampledAt;
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

// driveStraightTicks -- 099-008 convenience helper: ticks `pe` straight
// (dTheta == 0 -- both wheels advance by the SAME `stepMm` each tick, so
// encX_(t) is EXACTLY linear in t: encX_ == t - firstTickMs, since the very
// first application is the pre-existing zero-delta warm-up tick) for
// `count` ticks of `tickMs` each, mutating now/cumLeft/cumRight IN PLACE so
// a caller can interleave multiple driveStraightTicks() calls with fix/
// reset dispatches and keep accumulating from where it left off. Posts
// nothing to poseResetIn/poseFixIn itself -- pure encoder motion, matching
// every straight-line scenario elsewhere in this file.
void driveStraightTicks(Subsystems::PoseEstimator& pe, int count, float stepMm, uint32_t tickMs,
                         uint32_t& now, float& cumLeft, float& cumRight,
                         Rt::WorkQueue<Rt::PoseResetCommand, 4>& poseResetIn,
                         Rt::Mailbox<msg::SetPose>& otosSetPoseOut,
                         Rt::Mailbox<Rt::PoseFixCommand>& poseFixIn) {
  for (int i = 0; i < count; ++i) {
    now += tickMs;
    cumLeft += stepMm;
    cumRight += stepMm;
    pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), nullptr, poseResetIn,
            otosSetPoseOut, poseFixIn);
  }
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
  Rt::Mailbox<Rt::PoseFixCommand> poseFixIn;

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
    pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), nullptr, poseResetIn, otosSetPoseOut, poseFixIn);

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
  Rt::Mailbox<Rt::PoseFixCommand> poseFixIn;

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
    pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), nullptr, poseResetIn, otosSetPoseOut, poseFixIn);
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

    pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), &otos, poseResetIn, otosSetPoseOut, poseFixIn);
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
  Rt::Mailbox<Rt::PoseFixCommand> poseFixIn;

  uint32_t now = 0;
  float cumLeft = 0.0f;
  float cumRight = 0.0f;

  for (int i = 0; i < 5; ++i) {
    now += 20;
    cumLeft += 40.0f;
    cumRight += 40.0f;
    pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), nullptr, poseResetIn, otosSetPoseOut, poseFixIn);
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

    pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), &otos, poseResetIn, otosSetPoseOut, poseFixIn);
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
  Rt::Mailbox<Rt::PoseFixCommand> poseFixIn;

  uint32_t now = 0;
  float cumLeft = 0.0f;
  float cumRight = 0.0f;
  for (int i = 0; i < 5; ++i) {
    now += 20;
    cumLeft += 40.0f;
    cumRight += 40.0f;
    pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), nullptr, poseResetIn, otosSetPoseOut, poseFixIn);
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

  pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), nullptr, poseResetIn, otosSetPoseOut, poseFixIn);
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
  pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), nullptr, poseResetIn, otosSetPoseOut, poseFixIn);
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
  Rt::Mailbox<Rt::PoseFixCommand> poseFixIn;

  uint32_t now = 0;
  float cumLeft = 0.0f;
  float cumRight = 0.0f;
  // Build up real motion across a few ticks so there's a nonzero baseline
  // (a large cumulative encoder value) to rebaseline away from.
  for (int i = 0; i < 5; ++i) {
    now += 20;
    cumLeft += 40.0f;
    cumRight += 40.0f;
    pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), nullptr, poseResetIn, otosSetPoseOut, poseFixIn);
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

  pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), nullptr, poseResetIn, otosSetPoseOut, poseFixIn);   // dt == 0 (same now)
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
  pe.tick(now, motorStateAt(0.0f, now), motorStateAt(0.0f, now), nullptr, poseResetIn, otosSetPoseOut, poseFixIn);
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
  pe.tick(now, motorStateAt(40.0f, now), motorStateAt(40.0f, now), nullptr, poseResetIn, otosSetPoseOut, poseFixIn);
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
  Rt::Mailbox<Rt::PoseFixCommand> poseFixIn;

  uint32_t now = 0;
  float cumLeft = 0.0f;
  float cumRight = 0.0f;

  // A handful of ordinary ticks (a mild turn, no queued reset): both new
  // signals must stay at their inert defaults every single tick.
  for (int i = 0; i < 5; ++i) {
    now += 20;
    cumLeft += 40.0f;
    cumRight += 55.0f;
    pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), nullptr, poseResetIn, otosSetPoseOut, poseFixIn);
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

  pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), nullptr, poseResetIn, otosSetPoseOut, poseFixIn);

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
  pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), nullptr, poseResetIn, otosSetPoseOut, poseFixIn);
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
  pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), nullptr, poseResetIn, otosSetPoseOut, poseFixIn);
  checkTrue(poseResetIn.empty(), "tick() drained the posted kResetBaseline command");
  checkTrue(otosSetPoseOut.empty(), "kResetBaseline never posts to otosSetPoseOut");
  msg::PoseStep stepReset = pe.lastPoseStep();
  checkNear(stepReset.pos, 0.0f, 0.0f, "lastPoseStep().pos stays zero on a kResetBaseline-only tick");
  checkNear(stepReset.theta, 0.0f, 0.0f, "lastPoseStep().theta stays zero too");
}

// (h) 099-005: staggered wheel sample timing -- only one wheel's sampled_at
// advances per tick (matching bb.motors[]'s real ~40-80ms-per-motor
// flip-flop cadence vs. the 20ms tick cadence) -- proves two things:
//   (i)  the TOTAL accumulated encoderPose() displacement across the whole
//        sequence is IDENTICAL to a synchronous-pair baseline that delivers
//        the SAME five physical arc segments one-tick-per-segment, both
//        wheels fresh together (the telescoping-sum total is unaffected by
//        staleness -- architecture-update.md Decision 6). This holds exactly
//        (not just approximately) because the paired-freshness gate defers
//        each joint step until BOTH sides are fresh, at which point it
//        captures the FULL delta since the last joint step for both wheels
//        in one shot -- the identical (dCenter, dTheta) the synchronous
//        baseline computes for that same segment, applied against the
//        identical running heading, segment by segment.
//   (ii) the PER-TICK intermediate value on a tick where only one side is
//        fresh is UNCHANGED from the tick before it (encoderPose() does not
//        move at all) -- proving no local pivot is misattributed while the
//        other side is stale, in contrast to the pre-fix (naive) formula
//        (dCenter = dL/2 with the stale side's contribution silently
//        treated as 0 that same tick), which is shown to predict a
//        materially different, nonzero local pose change on the exact same
//        raw inputs.
void scenarioStaggeredSampleTimingMatchesSynchronousTotalNoLocalMisattribution() {
  beginScenario(
      "099-005: staggered sampled_at (one wheel fresh per tick) matches a "
      "synchronous-pair baseline's TOTAL displacement exactly, with no "
      "per-tick local misattribution while a side is stale");

  struct Segment {
    float dL;
    float dR;
  };
  const Segment segments[] = {
      {40.0f, 40.0f},   // straight
      {30.0f, 50.0f},   // gentle turn
      {50.0f, 20.0f},   // turn the other way
      {45.0f, 45.0f},   // straight again
      {0.0f, 60.0f},    // sharp turn
  };
  const float kTrackwidth = 128.0f;
  const float kSlip = 0.92f;   // in [0.5, 1.0] -- effectiveSlip() passes it
                                // through unchanged (see pose_estimator.cpp).

  // --- Synchronous-pair baseline: an explicit warm-up tick (both sides
  // fresh, zero motion) establishes haveEncBaseline_ FIRST -- matching the
  // staggered run's own warm-up below -- so segment 0 is a genuine,
  // fully-integrated joint step in BOTH runs, not silently absorbed as the
  // zero-delta "first application" by whichever run happens to reach it
  // first (haveEncBaseline_'s own pre-existing warm-up convention -- see
  // pose_estimator.h -- otherwise makes the very first tick asymmetric
  // between the two constructions). Each segment thereafter is delivered as
  // ONE tick, both wheels' sampled_at advancing together -- the SAME
  // pattern every other scenario in this file uses (motorStateAt(...,
  // now) at every call site).
  Subsystems::PoseEstimator baseline;
  baseline.configure(makeConfig(kTrackwidth, kSlip, 800.0f, 4.0f, 50.0f, 0.01f));
  Rt::WorkQueue<Rt::PoseResetCommand, 4> baselineResetIn;
  Rt::Mailbox<msg::SetPose> baselineSetPoseOut;
  Rt::Mailbox<Rt::PoseFixCommand> baselineFixIn;

  uint32_t now = 0;
  float cumLeft = 0.0f;
  float cumRight = 0.0f;
  baseline.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now),
                nullptr, baselineResetIn, baselineSetPoseOut, baselineFixIn);   // warm-up
  for (const Segment& seg : segments) {
    now += 20;
    cumLeft += seg.dL;
    cumRight += seg.dR;
    baseline.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now),
                  nullptr, baselineResetIn, baselineSetPoseOut, baselineFixIn);
  }
  msg::PoseEstimate baselineFinal = baseline.encoderPose();

  // Sanity: the sequence actually moved the robot (not a trivially-passing
  // all-zero comparison).
  checkTrue(baselineFinal.pose.x > 50.0f || baselineFinal.pose.y > 50.0f ||
                std::fabs(baselineFinal.pose.h) > 0.01f,
            "sanity: the segment sequence actually produced motion");

  // --- Staggered run: the SAME five segments, but each delivered across TWO
  // ticks -- an explicit warm-up tick first (both sides fresh, zero motion)
  // cleanly establishes the paired baseline, then each segment alternates a
  // left-only tick (right's position/sampled_at UNCHANGED -- still stale)
  // and a right-only tick (left's UNCHANGED, closing the pair). ---
  Subsystems::PoseEstimator staggered;
  staggered.configure(makeConfig(kTrackwidth, kSlip, 800.0f, 4.0f, 50.0f, 0.01f));
  Rt::WorkQueue<Rt::PoseResetCommand, 4> stagResetIn;
  Rt::Mailbox<msg::SetPose> stagSetPoseOut;
  Rt::Mailbox<Rt::PoseFixCommand> stagFixIn;

  uint32_t stagNow = 20;
  float stagLeft = 0.0f;
  float stagRight = 0.0f;
  uint32_t stagLeftSampledAt = stagNow;
  uint32_t stagRightSampledAt = stagNow;
  staggered.tick(stagNow, motorStateAt(stagLeft, stagLeftSampledAt),
                 motorStateAt(stagRight, stagRightSampledAt), nullptr,
                 stagResetIn, stagSetPoseOut, stagFixIn);

  int segIndex = 0;
  for (const Segment& seg : segments) {
    // Left-only tick: left's position AND sampled_at advance; right's stay
    // exactly at their last-fresh values (still stale) -- the
    // paired-freshness gate must NOT fire.
    stagNow += 10;
    stagLeft += seg.dL;
    stagLeftSampledAt = stagNow;
    msg::PoseEstimate beforeLeftOnly = staggered.encoderPose();
    staggered.tick(stagNow, motorStateAt(stagLeft, stagLeftSampledAt),
                   motorStateAt(stagRight, stagRightSampledAt), nullptr,
                   stagResetIn, stagSetPoseOut, stagFixIn);
    msg::PoseEstimate afterLeftOnly = staggered.encoderPose();

    char label[96];
    std::snprintf(label, sizeof(label), "segment %d left-only tick", segIndex);

    checkNear(afterLeftOnly.pose.x, beforeLeftOnly.pose.x, 0.0f,
              std::string(label) +
                  ": encoderPose().pose.x unchanged -- no local "
                  "misattribution while right is stale");
    checkNear(afterLeftOnly.pose.y, beforeLeftOnly.pose.y, 0.0f,
              std::string(label) + ": encoderPose().pose.y unchanged too");
    checkNear(afterLeftOnly.pose.h, beforeLeftOnly.pose.h, 0.0f,
              std::string(label) +
                  ": encoderPose().pose.h unchanged too (no phantom pivot)");

    // Confirm the fix's zero-change result above is a REAL divergence from
    // the pre-fix (naive) formula, not a vacuous "nothing happened": a naive
    // implementation computes this tick's delta as (this tick's raw
    // position - the immediately-prior tick's raw position) regardless of
    // freshness -- i.e. dL_naive = seg.dL, dR_naive = 0 (right's raw
    // reading did not change) -- and would have integrated a materially
    // different local pose right here.
    if (seg.dL != 0.0f) {
      float naiveDCenter = (seg.dL + 0.0f) * 0.5f;
      float naiveDTheta = ((0.0f - seg.dL) / kTrackwidth) * kSlip;
      float naiveThetaMid = beforeLeftOnly.pose.h + naiveDTheta * 0.5f;
      float naiveX = beforeLeftOnly.pose.x + naiveDCenter * std::cos(naiveThetaMid);
      float naiveY = beforeLeftOnly.pose.y + naiveDCenter * std::sin(naiveThetaMid);
      bool naiveDiverges = std::fabs(naiveX - afterLeftOnly.pose.x) > 1.0f ||
                            std::fabs(naiveY - afterLeftOnly.pose.y) > 1.0f;
      checkTrue(naiveDiverges,
                std::string(label) +
                    ": the naive pre-fix formula would have computed a "
                    "materially different (misattributed) local pose here "
                    "-- confirms the fix's zero-change result is a genuine "
                    "divergence, not a coincidence");
    }

    // Right-only tick: right's position AND sampled_at advance, closing the
    // pair (left has been fresh since the tick above) -- the joint step
    // fires NOW, capturing the FULL segment's (dL, dR) in one shot.
    stagNow += 10;
    stagRight += seg.dR;
    stagRightSampledAt = stagNow;
    staggered.tick(stagNow, motorStateAt(stagLeft, stagLeftSampledAt),
                   motorStateAt(stagRight, stagRightSampledAt), nullptr,
                   stagResetIn, stagSetPoseOut, stagFixIn);

    ++segIndex;
  }

  msg::PoseEstimate staggeredFinal = staggered.encoderPose();

  checkNear(staggeredFinal.pose.x, baselineFinal.pose.x, 1e-3f,
            "staggered run's TOTAL encoderPose().pose.x matches the "
            "synchronous-pair baseline exactly (telescoping-sum total is "
            "unaffected by staleness)");
  checkNear(staggeredFinal.pose.y, baselineFinal.pose.y, 1e-3f,
            "staggered run's TOTAL encoderPose().pose.y matches the "
            "synchronous-pair baseline exactly");
  checkNear(staggeredFinal.pose.h, baselineFinal.pose.h, 1e-4f,
            "staggered run's TOTAL encoderPose().pose.h matches the "
            "synchronous-pair baseline exactly");
}

// ===========================================================================
// 099-008 (architecture-update.md D5-D8): delayed camera-fix scenarios --
// pose-history ring, interpolate/compose, ungated EKF update.
//
// Every scenario below drives STRAIGHT (dTheta == 0, both wheels advancing
// by the SAME kStepMm every kTickMs) so encX_(t) is EXACTLY linear in
// (post-warm-up) tick time: encX_(now) == now - kTickMs, independent of
// the ring's own recording cadence. This makes "what SHOULD enc(T) be"
// hand-computable by simple arithmetic (t - kTickMs) rather than requiring
// a second copy of the arc-integration math, while still genuinely
// exercising the ring/interpolate/compose pipeline: a broken interpolation
// (e.g. returning the wrong ring entry, or not interpolating at all) would
// make the observed post-fix fusedPose() diverge from the hand-computed
// expectation by an amount far larger than these scenarios' tight
// tolerances allow (see each scenario's own comment for the specific
// divergence a bug would produce).
//
// A near-total-trust noise pair (kTinyR) is used for the interpolation/
// compose/clamp scenarios specifically so the Kalman gain is close enough
// to 1.0 that fusedPose() converges to the ALGEBRAICALLY exact composed
// target within a tight tolerance -- proving the compose math itself, not
// just "some correction happened." The default-fallback (sentinelOr) noise
// is exercised instead where the scenario's own point is mechanism
// (otosSetPoseOut/lastPoseStep/ring survival), not exact convergence.
// ===========================================================================

namespace {
constexpr float kStepMm = 20.0f;     // [mm] both wheels, every tick -- dTheta == 0
constexpr uint32_t kTickMs = 20;     // [ms]
constexpr float kTinyR = 1e-6f;      // [mm^2] / [rad^2] -- near-total-trust noise
}  // namespace

// (i) interpolation BETWEEN two ring entries, vs. a hand-computed oracle.
void scenarioFixInterpolationBetweenRingEntries() {
  beginScenario(
      "099-008: delayed fix interpolates BETWEEN two ring entries -- "
      "composed correction matches a hand-computed oracle");

  Subsystems::PoseEstimator pe;
  msg::DrivetrainConfig cfg = makeConfig(128.0f, 0.92f, 800.0f, 4.0f, 50.0f, 0.01f);
  cfg.ekf_r_fix_xy = kTinyR;
  cfg.ekf_r_fix_theta = kTinyR;
  pe.configure(cfg);

  Rt::WorkQueue<Rt::PoseResetCommand, 4> poseResetIn;
  Rt::Mailbox<msg::SetPose> otosSetPoseOut;
  Rt::Mailbox<Rt::PoseFixCommand> poseFixIn;

  uint32_t now = 0;
  float cumLeft = 0.0f;
  float cumRight = 0.0f;
  // 8 ticks -> ring entries land at t=20 (first, immediate), 80, 140 (the
  // 50ms-vs-20ms-tick interaction documented in pose_estimator.h's own
  // class comment); encX_(160) == 140 (== 160 - 20, the warm-up offset).
  driveStraightTicks(pe, 8, kStepMm, kTickMs, now, cumLeft, cumRight, poseResetIn,
                      otosSetPoseOut, poseFixIn);

  msg::PoseEstimate encNow = pe.encoderPose();
  checkNear(encNow.pose.x, 140.0f, 1e-3f, "sanity: encoderPose().pose.x == 140 after 8 ticks");

  // T=100 falls strictly between the ring entries at t=80 (x=60) and
  // t=140 (x=120) -- hand-computed oracle (linear, since motion is
  // straight): enc(100) == 80.
  const uint32_t kT = 100;
  const float kOracleEncAtT = 80.0f;
  const float kOffsetX = 300.0f;   // [mm] the "camera" claims the robot was this far ahead

  Rt::PoseFixCommand fix;
  fix.x = kOracleEncAtT + kOffsetX;
  fix.y = 0.0f;
  fix.h = 0.0f;
  fix.t = kT;
  poseFixIn.post(fix);
  // Same-pass (dt==0) re-dispatch, mirroring every other scenario in this
  // file's "SI/reset arrives, apply on the next tick() at the SAME now"
  // pattern (e.g. scenario (e)/(g) above).
  pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), nullptr, poseResetIn,
          otosSetPoseOut, poseFixIn);

  // implied.x = fix.x + (encNow.x - enc(T).x) = encNow.x + kOffsetX,
  // REGARDLESS of the exact value of enc(T), PROVIDED the internal
  // interpolation computes the SAME enc(T) this oracle does -- if it
  // computed a different enc(T) (e.g. returned the wrong ring entry, or
  // didn't interpolate at all), the result would differ from
  // encNow.x + kOffsetX by exactly that error, which this tight tolerance
  // catches.
  float expectedX = encNow.pose.x + kOffsetX;
  msg::PoseEstimate fused = pe.fusedPose();
  checkNear(fused.pose.x, expectedX, 0.5f,
            "fusedPose().pose.x converges to the composed target implied by "
            "the CORRECT between-entries interpolation of enc(T)");
  checkNear(fused.pose.y, encNow.pose.y, 0.5f,
            "fusedPose().pose.y unaffected (no y offset injected; straight "
            "motion keeps enc(T).y == encNow.y == 0)");
  checkNear(pe.encoderPose().pose.x, encNow.pose.x, 0.0f,
            "encoderPose() itself is untouched by applying a delayed fix");
  checkTrue(pe.fixDropped() == 0, "the fix was applied, not dropped");
}

// (ii) interpolation from the ring's NEWEST entry to "now" (T newer than
// every recorded ring entry), vs. a hand-computed oracle.
void scenarioFixInterpolationNewestToNow() {
  beginScenario(
      "099-008: delayed fix interpolates from the ring's NEWEST entry to "
      "\"now\" -- composed correction matches a hand-computed oracle");

  Subsystems::PoseEstimator pe;
  msg::DrivetrainConfig cfg = makeConfig(128.0f, 0.92f, 800.0f, 4.0f, 50.0f, 0.01f);
  cfg.ekf_r_fix_xy = kTinyR;
  cfg.ekf_r_fix_theta = kTinyR;
  pe.configure(cfg);

  Rt::WorkQueue<Rt::PoseResetCommand, 4> poseResetIn;
  Rt::Mailbox<msg::SetPose> otosSetPoseOut;
  Rt::Mailbox<Rt::PoseFixCommand> poseFixIn;

  uint32_t now = 0;
  float cumLeft = 0.0f;
  float cumRight = 0.0f;
  // 6 ticks -> the ring's newest (and only second) entry lands at t=80
  // (x=60); no further entry records before now=120 (80+40 < 130, the next
  // recording threshold) -- encX_(120) == 100.
  driveStraightTicks(pe, 6, kStepMm, kTickMs, now, cumLeft, cumRight, poseResetIn,
                      otosSetPoseOut, poseFixIn);

  msg::PoseEstimate encNow = pe.encoderPose();
  checkNear(encNow.pose.x, 100.0f, 1e-3f, "sanity: encoderPose().pose.x == 100 after 6 ticks");

  // T=110 is newer than the ring's newest entry (t=80) but older than now
  // (120) -- hand-computed oracle (linear): enc(110) == 90.
  const uint32_t kT = 110;
  const float kOracleEncAtT = 90.0f;
  const float kOffsetX = 300.0f;   // [mm]

  Rt::PoseFixCommand fix;
  fix.x = kOracleEncAtT + kOffsetX;
  fix.y = 0.0f;
  fix.h = 0.0f;
  fix.t = kT;
  poseFixIn.post(fix);
  pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), nullptr, poseResetIn,
          otosSetPoseOut, poseFixIn);

  float expectedX = encNow.pose.x + kOffsetX;
  msg::PoseEstimate fused = pe.fusedPose();
  checkNear(fused.pose.x, expectedX, 0.5f,
            "fusedPose().pose.x converges to the composed target implied by "
            "the CORRECT newest-to-now interpolation of enc(T)");
  checkTrue(pe.fixDropped() == 0, "the fix was applied, not dropped");
}

// (iii) a future t (t > now) clamps to now -- enc(T) collapses to encNow
// exactly, so implied.x == fix.x with NO offset contribution at all.
void scenarioFixFutureTimestampClamps() {
  beginScenario("099-008: a future-t (t > now) fix clamps T to now");

  Subsystems::PoseEstimator pe;
  msg::DrivetrainConfig cfg = makeConfig(128.0f, 0.92f, 800.0f, 4.0f, 50.0f, 0.01f);
  cfg.ekf_r_fix_xy = kTinyR;
  cfg.ekf_r_fix_theta = kTinyR;
  pe.configure(cfg);

  Rt::WorkQueue<Rt::PoseResetCommand, 4> poseResetIn;
  Rt::Mailbox<msg::SetPose> otosSetPoseOut;
  Rt::Mailbox<Rt::PoseFixCommand> poseFixIn;

  uint32_t now = 0;
  float cumLeft = 0.0f;
  float cumRight = 0.0f;
  driveStraightTicks(pe, 5, kStepMm, kTickMs, now, cumLeft, cumRight, poseResetIn,
                      otosSetPoseOut, poseFixIn);
  msg::PoseEstimate encNow = pe.encoderPose();

  // A wildly future t -- if the clamp is missing, interpolateEncAt() would
  // extrapolate the newest-ring-entry-to-now slope far past "now", making
  // enc(T) diverge sharply from encNow and implied.x diverge sharply from
  // fix.x. With the clamp, T collapses to exactly "now" -- frac == 1.0 in
  // lerpEncPose() -- so enc(T) == encNow exactly and implied.x == fix.x
  // exactly (the (encNow.x - enc(T).x) term vanishes).
  Rt::PoseFixCommand fix;
  fix.x = encNow.pose.x + 250.0f;
  fix.y = encNow.pose.y;
  fix.h = encNow.pose.h;
  fix.t = now + 1000;   // [ms] far future
  poseFixIn.post(fix);
  pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), nullptr, poseResetIn,
          otosSetPoseOut, poseFixIn);

  msg::PoseEstimate fused = pe.fusedPose();
  checkNear(fused.pose.x, fix.x, 0.5f,
            "a future-t fix converges to EXACTLY fix.x (the clamp collapses "
            "enc(T) to encNow, cancelling the (encNow - enc(T)) term) -- a "
            "missing clamp would extrapolate far past this");
  checkTrue(pe.fixDropped() == 0, "a future t is clamped, never dropped");
}

// (iv) SI (reset=true) clears the ring; zero_encoders does NOT; applying a
// delayed fix does NOT either -- verified against the actual code (via
// fixDropped()'s observable behavior), not just asserted.
void scenarioRingClearedBySIOnlyNotZeroNotFix() {
  beginScenario(
      "099-008: SI clears the pose-history ring; zero_encoders does not; "
      "applying a delayed fix does not either");

  Subsystems::PoseEstimator pe;
  pe.configure(makeConfig(128.0f, 0.92f, 800.0f, 4.0f, 50.0f, 0.01f));

  Rt::WorkQueue<Rt::PoseResetCommand, 4> poseResetIn;
  Rt::Mailbox<msg::SetPose> otosSetPoseOut;
  Rt::Mailbox<Rt::PoseFixCommand> poseFixIn;

  uint32_t now = 0;
  float cumLeft = 0.0f;
  float cumRight = 0.0f;
  // Ring after 5 ticks: entries at t=20, t=80 -- oldest.t == 20.
  driveStraightTicks(pe, 5, kStepMm, kTickMs, now, cumLeft, cumRight, poseResetIn,
                      otosSetPoseOut, poseFixIn);

  // A fix timestamped well before the ring's oldest entry (t=20) -- used
  // repeatedly below as a PROBE: it is dropped (fixDropped_ increments)
  // exactly when the ring still has an entry older than "now" to compare
  // against, and NOT dropped once the ring is empty.
  auto postStaleProbe = [&]() {
    Rt::PoseFixCommand fix;
    fix.x = 99999.0f;
    fix.y = 99999.0f;
    fix.h = 3.0f;
    fix.t = 1;   // [ms] -- older than the ring's oldest entry (t=20)
    poseFixIn.post(fix);
    pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), nullptr, poseResetIn,
            otosSetPoseOut, poseFixIn);
  };

  postStaleProbe();
  checkTrue(pe.fixDropped() == 1, "probe #1: dropped -- the ring's oldest entry (t=20) rejects t=1");

  // zero_encoders (kResetBaseline) -- must NOT clear the ring.
  Rt::PoseResetCommand zeroCmd;
  zeroCmd.kind = Rt::PoseResetCommand::kResetBaseline;
  checkTrue(poseResetIn.post(zeroCmd), "post() succeeds");
  pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), nullptr, poseResetIn,
          otosSetPoseOut, poseFixIn);

  postStaleProbe();
  checkTrue(pe.fixDropped() == 2,
            "probe #2 (after zero_encoders): STILL dropped -- zero_encoders "
            "did not clear the ring");

  // A VALID fix (t=50, within the ring's range) -- applying it must NOT
  // clear the ring either.
  Rt::PoseFixCommand validFix;
  validFix.x = 500.0f;
  validFix.y = 0.0f;
  validFix.h = 0.0f;
  validFix.t = 50;
  poseFixIn.post(validFix);
  pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), nullptr, poseResetIn,
          otosSetPoseOut, poseFixIn);
  checkTrue(pe.fixDropped() == 2, "the valid fix (t=50) was applied, not dropped");

  postStaleProbe();
  checkTrue(pe.fixDropped() == 3,
            "probe #3 (after applying a valid fix): STILL dropped -- "
            "applying a delayed fix did not clear the ring either "
            "(consecutive fixes compose without ring invalidation)");

  // SI (kSetPose) -- MUST clear the ring.
  Rt::PoseResetCommand siCmd;
  siCmd.kind = Rt::PoseResetCommand::kSetPose;
  siCmd.pose.x = 1000.0f;
  siCmd.pose.y = -500.0f;
  siCmd.pose.h = 0.5f;
  checkTrue(poseResetIn.post(siCmd), "post() succeeds");
  pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), nullptr, poseResetIn,
          otosSetPoseOut, poseFixIn);
  // NOTE: clearRing() runs INSIDE this same tick() call (from setPose()'s
  // own dispatch), and this SAME call's own end-of-tick ring-record step
  // (haveRingRecord_ now false) immediately re-seeds ONE fresh entry at
  // THIS call's `now` -- the ring is never observably empty from OUTSIDE a
  // tick() call. The correct external proof of clearing is therefore: a
  // timestamp that WAS valid against the pre-SI ring (t=50, accepted by
  // the "valid fix" step above, whose ring covered [20, ~100]) must now be
  // REJECTED, because the post-SI ring's sole entry is `now` itself (100),
  // strictly newer than 50 -- proving the pre-SI history is genuinely gone,
  // not merely quiet.

  Rt::PoseFixCommand probeAtOldValidT;
  probeAtOldValidT.x = 0.0f;
  probeAtOldValidT.y = 0.0f;
  probeAtOldValidT.h = 0.0f;
  probeAtOldValidT.t = 50;   // was ACCEPTED pre-SI (the "valid fix" step above)
  poseFixIn.post(probeAtOldValidT);
  pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), nullptr, poseResetIn,
          otosSetPoseOut, poseFixIn);
  checkTrue(pe.fixDropped() == 4,
            "t=50 -- valid BEFORE SI -- is now dropped AFTER SI: the ring's "
            "pre-SI history (oldest t=20) is genuinely gone, replaced by a "
            "ring whose only entry is SI's own `now`");

  // A fix timestamped at (or after) the NEW post-SI baseline is still
  // accepted -- the ring is alive and correct post-SI, not just perpetually
  // rejecting everything.
  Rt::PoseFixCommand probeAtNewBaseline;
  probeAtNewBaseline.x = 1000.0f;
  probeAtNewBaseline.y = -500.0f;
  probeAtNewBaseline.h = 0.5f;
  probeAtNewBaseline.t = now;   // == SI's own re-seeded ring entry's timestamp
  poseFixIn.post(probeAtNewBaseline);
  pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), nullptr, poseResetIn,
          otosSetPoseOut, poseFixIn);
  checkTrue(pe.fixDropped() == 4,
            "a fix at (or after) the NEW post-SI baseline is accepted -- "
            "the ring works correctly post-SI, not merely empty/broken");

  msg::PoseEstimate fused = pe.fusedPose();
  checkTrue(std::isfinite(fused.pose.x) && std::isfinite(fused.pose.y) &&
                std::isfinite(fused.pose.h),
            "fusedPose() stays finite throughout -- no crash, no NaN");
}

// (v) two consecutive valid fixes both compose correctly (neither dropped),
// proving the ring survives across repeated fix applications without
// invalidation.
void scenarioConsecutiveFixesComposeWithoutRingInvalidation() {
  beginScenario(
      "099-008: two consecutive delayed fixes both compose correctly -- "
      "the ring is never invalidated by applying one");

  Subsystems::PoseEstimator pe;
  msg::DrivetrainConfig cfg = makeConfig(128.0f, 0.92f, 800.0f, 4.0f, 50.0f, 0.01f);
  cfg.ekf_r_fix_xy = kTinyR;
  cfg.ekf_r_fix_theta = kTinyR;
  pe.configure(cfg);

  Rt::WorkQueue<Rt::PoseResetCommand, 4> poseResetIn;
  Rt::Mailbox<msg::SetPose> otosSetPoseOut;
  Rt::Mailbox<Rt::PoseFixCommand> poseFixIn;

  uint32_t now = 0;
  float cumLeft = 0.0f;
  float cumRight = 0.0f;
  driveStraightTicks(pe, 8, kStepMm, kTickMs, now, cumLeft, cumRight, poseResetIn,
                      otosSetPoseOut, poseFixIn);
  msg::PoseEstimate encNow1 = pe.encoderPose();
  checkNear(encNow1.pose.x, 140.0f, 1e-3f, "sanity: encoderPose().pose.x == 140 after 8 ticks");

  // Fix #1: T=100 (between the ring entries at t=80/t=140), oracle enc(100)
  // == 80 -- same derivation as scenarioFixInterpolationBetweenRingEntries().
  Rt::PoseFixCommand fix1;
  fix1.x = 80.0f + 300.0f;
  fix1.y = 0.0f;
  fix1.h = 0.0f;
  fix1.t = 100;
  poseFixIn.post(fix1);
  pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), nullptr, poseResetIn,
          otosSetPoseOut, poseFixIn);

  checkTrue(pe.fixDropped() == 0, "fix #1 was applied, not dropped");
  checkNear(pe.fusedPose().pose.x, encNow1.pose.x + 300.0f, 0.5f,
            "fix #1 converges to its own composed target");

  // More motion after fix #1 -- encX_/encY_/encTheta_ (the ring's OWN
  // source series) are completely untouched by applying a fix, so they
  // keep accumulating normally.
  driveStraightTicks(pe, 4, kStepMm, kTickMs, now, cumLeft, cumRight, poseResetIn,
                      otosSetPoseOut, poseFixIn);
  msg::PoseEstimate encNow2 = pe.encoderPose();
  checkNear(encNow2.pose.x, 220.0f, 1e-3f, "sanity: encoderPose().pose.x == 220 after 12 ticks");

  // Fix #2: T=210 (newest-to-now against the ring entry recorded at
  // t=200), oracle enc(210) == 190.
  Rt::PoseFixCommand fix2;
  fix2.x = 190.0f + 300.0f;
  fix2.y = 0.0f;
  fix2.h = 0.0f;
  fix2.t = 210;
  poseFixIn.post(fix2);
  pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), nullptr, poseResetIn,
          otosSetPoseOut, poseFixIn);

  checkTrue(pe.fixDropped() == 0, "fix #2 was ALSO applied, not dropped -- fix #1 did not invalidate the ring");
  checkNear(pe.fusedPose().pose.x, encNow2.pose.x + 300.0f, 0.5f,
            "fix #2 converges to its OWN composed target, computed fresh "
            "against the ring's current state");
}

// (vi) otosSetPoseOut is posted EXACTLY ONCE per applied fix (never on a
// tick with no fix), and lastPoseStep() reports a nonzero magnitude for the
// tick a fix was applied, resetting to {0, 0} on every other tick -- the
// SAME mechanism ticket 004 proved for SI (scenario (g) above), now proven
// for a delayed fix. Uses the DEFAULT (unconfigured, sentinelOr()-
// substituted) ekf_r_fix_xy/theta -- proves the fallback produces a REAL,
// nonzero, finite correction, not a silent no-op.
void scenarioFixOtosSetPoseOutPostedOnceLastPoseStepMagnitude() {
  beginScenario(
      "099-008: otosSetPoseOut posted exactly once per applied delayed fix; "
      "lastPoseStep() reports its magnitude (default sentinel-fallback noise)");

  Subsystems::PoseEstimator pe;
  pe.configure(makeConfig(128.0f, 0.92f, 800.0f, 4.0f, 50.0f, 0.01f));   // ekf_r_fix_* left 0 (unset)

  Rt::WorkQueue<Rt::PoseResetCommand, 4> poseResetIn;
  Rt::Mailbox<msg::SetPose> otosSetPoseOut;
  Rt::Mailbox<Rt::PoseFixCommand> poseFixIn;

  uint32_t now = 0;
  float cumLeft = 0.0f;
  float cumRight = 0.0f;
  for (int i = 0; i < 5; ++i) {
    driveStraightTicks(pe, 1, kStepMm, kTickMs, now, cumLeft, cumRight, poseResetIn,
                        otosSetPoseOut, poseFixIn);
    checkTrue(otosSetPoseOut.empty(), "otosSetPoseOut stays empty when no fix is queued");
    msg::PoseStep step = pe.lastPoseStep();
    checkNear(step.pos, 0.0f, 0.0f, "lastPoseStep().pos is zero on an ordinary tick");
    checkNear(step.theta, 0.0f, 0.0f, "lastPoseStep().theta is zero on an ordinary tick");
  }
  // now == 100, encoderPose().pose.x == 80; ring entries at t=20, t=80.

  Rt::PoseFixCommand fix;
  fix.x = 30.0f + 150.0f;   // enc(50) == 30 (between t=20/t=80); +150mm offset
  fix.y = 0.0f;
  fix.h = 0.0f;
  fix.t = 50;
  poseFixIn.post(fix);
  pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), nullptr, poseResetIn,
          otosSetPoseOut, poseFixIn);

  checkTrue(pe.fixDropped() == 0, "the fix was applied, not dropped");
  checkTrue(!otosSetPoseOut.empty(), "otosSetPoseOut received exactly one post for the applied fix");
  msg::SetPose posted = otosSetPoseOut.take();
  checkTrue(otosSetPoseOut.empty(), "otosSetPoseOut.take() drained the one posted value -- posted exactly once");
  msg::PoseEstimate fusedAfter = pe.fusedPose();
  checkNear(posted.x, fusedAfter.pose.x, 0.0f, "otosSetPoseOut carries the corrected fusedPose().pose.x");
  checkNear(posted.y, fusedAfter.pose.y, 0.0f, "otosSetPoseOut carries the corrected fusedPose().pose.y");
  checkNear(posted.h, fusedAfter.pose.h, 0.0f, "otosSetPoseOut carries the corrected fusedPose().pose.h");

  msg::PoseStep step = pe.lastPoseStep();
  checkTrue(std::isfinite(step.pos) && step.pos > 0.5f,
            "lastPoseStep().pos is a real, nonzero, finite correction -- the "
            "default sentinel-fallback ekf_r_fix_xy did NOT silently no-op");
  checkTrue(step.pos < 150.0f + 50.0f,
            "lastPoseStep().pos is a bounded fraction of the injected offset "
            "(Kalman gain <= 1), not a runaway value");

  // The VERY NEXT tick, no queued fix: both signals reset to inert.
  driveStraightTicks(pe, 1, kStepMm, kTickMs, now, cumLeft, cumRight, poseResetIn,
                      otosSetPoseOut, poseFixIn);
  checkTrue(otosSetPoseOut.empty(),
            "otosSetPoseOut stays empty the tick after the fix -- posted exactly once, not every tick");
  msg::PoseStep stepAfter = pe.lastPoseStep();
  checkNear(stepAfter.pos, 0.0f, 0.0f, "lastPoseStep() resets to zero on the tick following the applied fix");
  checkNear(stepAfter.theta, 0.0f, 0.0f, "lastPoseStep().theta resets to zero too");
}

// (vii) a fix timestamped older than the ring's oldest entry is dropped:
// fixDropped_ increments, fusedPose()/encoderPose() are EXACTLY unchanged
// (no jump), lastPoseStep() stays {0, 0}, otosSetPoseOut stays empty, and
// the estimator keeps working normally afterward (no crash).
void scenarioStaleFixDroppedNoJumpNoCrash() {
  beginScenario(
      "099-008: a stale-timestamp fix (t older than the ring) is dropped -- "
      "no jump, counted, no crash");

  Subsystems::PoseEstimator pe;
  pe.configure(makeConfig(128.0f, 0.92f, 800.0f, 4.0f, 50.0f, 0.01f));

  Rt::WorkQueue<Rt::PoseResetCommand, 4> poseResetIn;
  Rt::Mailbox<msg::SetPose> otosSetPoseOut;
  Rt::Mailbox<Rt::PoseFixCommand> poseFixIn;

  uint32_t now = 0;
  float cumLeft = 0.0f;
  float cumRight = 0.0f;
  driveStraightTicks(pe, 8, kStepMm, kTickMs, now, cumLeft, cumRight, poseResetIn,
                      otosSetPoseOut, poseFixIn);

  msg::PoseEstimate fusedBefore = pe.fusedPose();
  msg::PoseEstimate encBefore = pe.encoderPose();
  checkTrue(pe.fixDropped() == 0, "sanity: nothing dropped yet");

  // t=1 is older than the ring's oldest entry (t=20) -- a wildly different
  // x/y/h so an incorrectly-applied fix would produce an unmistakable jump.
  Rt::PoseFixCommand staleFix;
  staleFix.x = -99999.0f;
  staleFix.y = 99999.0f;
  staleFix.h = 2.5f;
  staleFix.t = 1;
  poseFixIn.post(staleFix);
  pe.tick(now, motorStateAt(cumLeft, now), motorStateAt(cumRight, now), nullptr, poseResetIn,
          otosSetPoseOut, poseFixIn);

  checkTrue(pe.fixDropped() == 1, "the stale fix was dropped and counted exactly once");

  msg::PoseEstimate fusedAfter = pe.fusedPose();
  msg::PoseEstimate encAfter = pe.encoderPose();
  checkNear(fusedAfter.pose.x, fusedBefore.pose.x, 0.0f, "fusedPose().pose.x EXACTLY unchanged -- no jump");
  checkNear(fusedAfter.pose.y, fusedBefore.pose.y, 0.0f, "fusedPose().pose.y EXACTLY unchanged too");
  checkNear(fusedAfter.pose.h, fusedBefore.pose.h, 0.0f, "fusedPose().pose.h EXACTLY unchanged too");
  checkNear(encAfter.pose.x, encBefore.pose.x, 0.0f, "encoderPose() EXACTLY unchanged too");

  msg::PoseStep step = pe.lastPoseStep();
  checkNear(step.pos, 0.0f, 0.0f, "lastPoseStep().pos stays zero -- nothing was applied");
  checkNear(step.theta, 0.0f, 0.0f, "lastPoseStep().theta stays zero too");
  checkTrue(otosSetPoseOut.empty(), "otosSetPoseOut was never posted to -- the drop path never reaches step 6");

  checkTrue(std::isfinite(fusedAfter.pose.x) && std::isfinite(fusedAfter.pose.y) &&
                std::isfinite(fusedAfter.pose.h),
            "fusedPose() stays finite -- no crash");

  // The estimator keeps working normally afterward.
  driveStraightTicks(pe, 1, kStepMm, kTickMs, now, cumLeft, cumRight, poseResetIn,
                      otosSetPoseOut, poseFixIn);
  checkTrue(std::isfinite(pe.fusedPose().pose.x), "still finite after a further normal tick");
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
  scenarioStaggeredSampleTimingMatchesSynchronousTotalNoLocalMisattribution();

  // 099-008: delayed camera-fix -- history ring, transport-compose, ungated
  // EKF update.
  scenarioFixInterpolationBetweenRingEntries();
  scenarioFixInterpolationNewestToNow();
  scenarioFixFutureTimestampClamps();
  scenarioRingClearedBySIOnlyNotZeroNotFix();
  scenarioConsecutiveFixesComposeWithoutRingInvalidation();
  scenarioFixOtosSetPoseOutPostedOnceLastPoseStepMagnitude();
  scenarioStaleFixDroppedNoJumpNoCrash();

  if (g_failureCount == 0) {
    std::printf("OK: all PoseEstimator scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the PoseEstimator scenarios\n",
              g_failureCount);
  return 1;
}
