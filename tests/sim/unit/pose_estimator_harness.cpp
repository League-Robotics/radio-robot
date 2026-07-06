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
    pe.tick(now, motorStateAt(cumLeft), motorStateAt(cumRight), nullptr);

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
    pe.tick(now, motorStateAt(cumLeft), motorStateAt(cumRight), nullptr);
  }

  msg::PoseEstimate encBefore = pe.encoderPose();
  msg::PoseEstimate fusedBefore = pe.fusedPose();
  checkTrue(fusedBefore.pose.x == encBefore.pose.x,
            "sanity: fusedPose() still matches encoderPose() before any "
            "OTOS observation");

  // Feed a run of OTOS observations persistently offset ahead of the
  // encoder-only path in x.
  const float kOffsetX = 150.0f;  // [mm]
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

    pe.tick(now, motorStateAt(cumLeft), motorStateAt(cumRight), &otos);
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

  uint32_t now = 0;
  float cumLeft = 0.0f;
  float cumRight = 0.0f;

  for (int i = 0; i < 5; ++i) {
    now += 20;
    cumLeft += 40.0f;
    cumRight += 40.0f;
    pe.tick(now, motorStateAt(cumLeft), motorStateAt(cumRight), nullptr);
  }

  const float kOffsetX = 150.0f;  // [mm]
  for (int i = 0; i < 8; ++i) {
    now += 20;
    cumLeft += 40.0f;
    cumRight += 40.0f;

    msg::PoseEstimate refEnc = pe.encoderPose();
    msg::PoseEstimate otos =
        otosAt(refEnc.pose.x + kOffsetX, refEnc.pose.y, refEnc.pose.h, now);

    pe.tick(now, motorStateAt(cumLeft), motorStateAt(cumRight), &otos);
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

}  // namespace

int main() {
  scenarioNoOtosFusedMatchesEncoderExactly();
  scenarioOtosDivergesFusedFromEncoder();
  scenarioZeroConfigSentinelKeepsFusionFiniteAndCorrected();

  if (g_failureCount == 0) {
    std::printf("OK: all PoseEstimator scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the PoseEstimator scenarios\n",
              g_failureCount);
  return 1;
}
