// dev_loop_pose_estimator_harness.cpp — off-hardware acceptance harness for
// ticket 082-003: proves devLoopTick() (source/dev_loop.{h,cpp}) advances the
// wired Subsystems::PoseEstimator by EXACTLY one PoseEstimator::tick() call
// per pass — the ticket's single most important correctness property (see
// dev_loop.h's own doc comment on the new step).
//
// --- How "exactly once" is tested here, and what is deliberately NOT
// attempted ---
//
// PoseEstimator::tick() is a plain (non-virtual) method and
// DevLoop::poseEstimator is declared as a concrete `Subsystems::
// PoseEstimator*` (082-003's own acceptance criterion) — there is no seam to
// intercept a call through (no vtable, no mock). A literal "call tick()
// twice in immediate succession with the SAME cached (now, leftObs, rightObs,
// otosObs) arguments" mutant is therefore not just untestable here, it is
// PROVABLY A MATHEMATICAL NO-OP for this exact implementation: the second
// call's encoder delta is (left - prevEncLeft_) == 0 and (right -
// prevEncRight_) == 0 (the first call already rebaselined prevEncLeft_/
// prevEncRight_ to left/right), so dCenter == dTheta == 0; and its dt is
// (now - lastTick_) == 0 (the first call already advanced lastTick_ to now).
// EkfTiny::predict() with dCenter == dTheta == 0 has Jacobian f == the exact
// identity (f[0][2] = -dCenter*sin(...) == 0, f[1][2] = dCenter*cos(...) ==
// 0) and qScaled == q_ * dt == 0 (ekf_tiny.cpp's predict()), so the EKF's
// state AND covariance are left EXACTLY unchanged too. Same-instant repeat
// calls are inert by construction — not a bug this class can have, so this
// harness does not chase it.
//
// What IS a real, observable correctness property — and what this harness
// proves — is that devLoopTick() invokes poseEstimator->tick() UNCONDITIONALLY
// once per pass, with the CORRECT (freshest, correctly-bound) observations,
// regardless of the drivetrain's active()/inactive() state or which port
// pair is bound. Proven by running the REAL devLoopTick() (instance "A")
// alongside an independently, manually-driven reference pipeline (instance
// "REF") that calls the equivalent steps — two hardware.tick(now) calls,
// drivetrain.ports(), hardware.motor(...).state(), hardware.odometer()->
// tick()+pose() — and poseEstimator.tick() EXACTLY ONCE, per pass, by
// construction. Both pipelines are fed byte-identical motor commands and
// `now` sequences, so their SimHardware/PoseEstimator state is deterministic
// and must match bit-for-bit at every pass IF devLoopTick() calls tick()
// exactly once, with the right arguments, every pass:
//   - if devLoopTick() skipped the call some pass (e.g. gated on
//     drivetrain.active() the way the OLDER Drivetrain-governance block is),
//     A would lag REF's accumulated motion — caught (scenario 1: drivetrain
//     left inactive throughout; scenario 2: drivetrain active throughout).
//   - if devLoopTick() used the wrong port pair (e.g. hardcoding ports 1/2
//     instead of querying drivetrain.ports()), A would read the WRONG
//     motors' state whenever the bound pair is rebound away from {1,2} —
//     caught (scenario 3: ports rebound to {3,4}).
//
// Same ad hoc-compile convention as the other tests/sim/unit/*_harness.cpp
// files (hand-rolled assertions, PASS/FAIL per scenario, nonzero exit on any
// failure) — compiled by test_dev_loop_pose_estimator.py together with the
// real source/dev_loop.cpp, source/subsystems/{drivetrain,sim_hardware,
// pose_estimator}.cpp, source/estimation/ekf_tiny.cpp,
// source/commands/{arg_parse,command_processor,dev_commands}.cpp,
// source/kinematics/body_kinematics.cpp, source/hal/sim/*.cpp, and
// source/hal/velocity_pid.cpp, with -DHOST_BUILD -DROBOT_DEV_BUILD=1 (dev_loop.cpp
// is gated behind ROBOT_DEV_BUILD — see dev_loop.h's file header) and
// libraries/tinyekf/ on the include path (ekf_tiny.h's tinyekf.h is
// header-only).
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "commands/command_processor.h"
#include "commands/dev_commands.h"
#include "commands/telemetry_commands.h"
#include "dev_loop.h"
#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "runtime/commands.h"
#include "runtime/queue.h"
#include "subsystems/drivetrain.h"
#include "subsystems/hardware.h"
#include "subsystems/pose_estimator.h"
#include "subsystems/sim_hardware.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors sim_hardware_harness.cpp /
// pose_estimator_harness.cpp) ---

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

// tol == 0.0f is an exact (bit-for-bit) equality check -- both pipelines
// below are pure functions of identical inputs, so exact equality is the
// right bar (mirrors sim_hardware_harness.cpp's own dt=0 re-entry guard
// scenario, which makes the same "exact, not approximate" argument).
void checkNear(float actual, float expected, float tol, const std::string& what) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %.9g (tol %.3g), got %.9g",
                  what.c_str(), static_cast<double>(expected),
                  static_cast<double>(tol), static_cast<double>(actual));
    fail(buf);
  }
}

// --- Fixture helpers ---

// fillDefaultConfigs -- fwd_sign=1, travel_calib=1.0, no dwell/deadband, so a
// commanded duty cycle passes through unmodified -- PLUS real velocity-PID
// gains, needed because scenario 2 (drivetrain ACTIVE) governs the wheels via
// a VELOCITY-mode command: all-zero gains would leave that command's applied
// duty permanently at 0, producing no motion regardless of this harness's
// own correctness. Uses sim_api.cpp's own defaultMotorConfigSet() gains
// (kp=0.0022/ki=0.0018/kff=0.0038/i_max=0.3) rather than sim_hardware_
// harness.cpp's more aggressive kp=0.01 set: at this harness's 20ms step and
// PhysicsWorld's instant (no-lag) velocity response, kp=0.01 bang-bangs
// between +/-1.0 duty every pass (a stable, but NET-ZERO, limit cycle) --
// sim_api.cpp's gentler gains converge smoothly, which this harness's
// multi-pass "did it actually move" sanity check needs.
void fillDefaultConfigs(msg::MotorConfig configs[Subsystems::Hardware::kPortCount]) {
  msg::Gains gains;
  gains.kp = 0.0022f;
  gains.ki = 0.0018f;
  gains.kff = 0.0038f;
  gains.i_max = 0.3f;

  for (uint32_t i = 0; i < Subsystems::Hardware::kPortCount; ++i) {
    configs[i] = msg::MotorConfig{};
    configs[i].setPort(i + 1)
        .setFwdSign(1)
        .setTravelCalib(1.0f)
        .setVelGains(gains)
        .setVelFiltAlpha(1.0f)
        .setOutputDeadband(0.0f)
        .setReversalDwell(0.0f);
  }
}

msg::DrivetrainConfig makeDtConfig(uint32_t leftPort, uint32_t rightPort) {
  msg::DrivetrainConfig cfg;
  cfg.setTrackwidth(128.0f)
      .setRotationalSlip(0.92f)
      .setLeftPort(leftPort)
      .setRightPort(rightPort)
      .setEkfQXy(800.0f)
      .setEkfQTheta(4.0f)
      .setEkfROtosXy(50.0f)
      .setEkfROtosTheta(0.01f);
  return cfg;
}

// oneReferencePass -- devLoopTick()'s FULL relevant body, replicated by hand
// against an INDEPENDENT SimHardware/Drivetrain/PoseEstimator trio: slice 1,
// the Drivetrain governance block (only when active() -- byte-identical to
// dev_loop.cpp's own `if (drivetrain.active())` block, so a driveActive
// scenario's REF motors are governed exactly like A's are), slice 2, then
// the new pose-estimation step -- ports() queried unconditionally, freshest
// motor().state() reads, an odometer tick()+pose() sample if one exists, and
// EXACTLY ONE poseEstimator.tick() call. Every call site here mirrors
// dev_loop.cpp's own structure 1:1 (statement dispatch/outbox drain are
// omitted -- this harness never feeds a statement or stages a DevLoopState
// command, so both are no-ops in the real devLoopTick() too).
void oneReferencePass(Subsystems::SimHardware& hardware, Subsystems::Drivetrain& drivetrain,
                       Subsystems::PoseEstimator& poseEstimator, uint32_t now) {
  // 087-004: mirrors dev_loop.cpp's own always-empty/all-false local
  // motorIn[]/motorResetIn[] pair (see that file's comment at its matching
  // call site) -- this harness's REF pipeline never stages a motorIn[]/
  // motorResetIn[] post either, so an empty/all-false pair here keeps REF
  // byte-identical to dev_loop.cpp's real devLoopTick().
  Rt::Mailbox<msg::MotorCommand> noMotorInYet[Subsystems::Hardware::kPortCount];
  bool noMotorResetInYet[Subsystems::Hardware::kPortCount] = {false, false, false, false};

  hardware.tick(now, noMotorInYet, noMotorResetInYet);   // slice 1

  if (drivetrain.active()) {
    Subsystems::DrivetrainPorts governedPorts = drivetrain.ports();
    // 087-003: mirrors dev_loop.cpp's own always-empty local driveIn Mailbox
    // (see that file's comment at its matching call site) -- this harness's
    // REF pipeline never stages a driveIn post either, so an empty Mailbox
    // here keeps REF byte-identical to dev_loop.cpp's real devLoopTick().
    Rt::Mailbox<msg::DrivetrainCommand> noDriveInYet;
    drivetrain.tick(now, hardware.motor(governedPorts.left).state(),
                     hardware.motor(governedPorts.right).state(), noDriveInYet);
    if (drivetrain.hasCommand()) {
      hardware.apply(drivetrain.takeCommand());
    }
  }

  hardware.tick(now, noMotorInYet, noMotorResetInYet);   // slice 2 (same now -- SimHardware's own dt=0 guard, 081-003)

  Subsystems::DrivetrainPorts p = drivetrain.ports();
  msg::MotorState leftObs = hardware.motor(p.left).state();
  msg::MotorState rightObs = hardware.motor(p.right).state();

  Hal::Odometer* odometer = hardware.odometer();
  msg::PoseEstimate sampledPose = {};
  if (odometer != nullptr) {
    odometer->tick(now);
    sampledPose = odometer->pose();
  }
  // 087-004: mirrors dev_loop.cpp's own always-empty local poseResetIn
  // queue (see that file's comment at its matching call site).
  Rt::WorkQueue<Rt::PoseResetCommand, 4> noPoseResetInYet;
  poseEstimator.tick(now, leftObs, rightObs, odometer != nullptr ? &sampledPose : nullptr,
                      noPoseResetInYet);
}

// runComparison -- drives instance "A" (the REAL devLoopTick(), via a fully
// wired DevLoop) and instance "REF" (oneReferencePass(), called by hand) in
// lockstep over `passCount` passes, 20ms apart, asserting A's poseEstimator
// exactly matches REF's after every single pass. leftPort/rightPort bind
// both drivetrains' pair; leftDuty/rightDuty are applied once, up front, to
// BOTH pipelines' bound motors (straight or curved motion, asymmetric so
// dTheta != 0 too); if driveActive, both drivetrains are additionally put
// into WHEELS-arm active() authority (setWheelTargets) so the governance
// block that runs BETWEEN the two hardware.tick() slices genuinely executes
// this pass, proving the new step is unconditional regardless.
void runComparison(uint32_t leftPort, uint32_t rightPort, float leftDuty, float rightDuty,
                    bool driveActive, int passCount, const std::string& scenarioName) {
  beginScenario(scenarioName);

  msg::MotorConfig configs[Subsystems::Hardware::kPortCount];
  fillDefaultConfigs(configs);
  msg::DrivetrainConfig dtConfig = makeDtConfig(leftPort, rightPort);

  // --- Instance A: driven ENTIRELY through the real devLoopTick(). ---
  Subsystems::SimHardware hardwareA(configs);
  hardwareA.begin();
  Subsystems::Drivetrain drivetrainA;
  drivetrainA.configure(dtConfig);
  Subsystems::PoseEstimator poseA;
  poseA.configure(dtConfig);
  SerialSilenceWatchdog watchdogA;   // default 1000ms window; never fed, never fires (see below)
  DevLoopState devStateA;
  // 082-004: TelemetryState's periodMs defaults to 0 (STREAM never issued in
  // this harness), so devLoopTick()'s periodic-emission step is a no-op --
  // wired only to satisfy DevLoop::telemetry's non-null contract (dev_loop.h),
  // never affecting the encoder/fused-pose comparison this harness makes.
  TelemetryState telemetryA;
  telemetryA.hardware = &hardwareA;
  telemetryA.drivetrain = &drivetrainA;
  telemetryA.poseEstimator = &poseA;
  // 084-002: devLoopTick() now dereferences DevLoop::planner/motionState
  // unconditionally every pass (the new motion-executor step) -- wired here
  // only to satisfy that non-null contract; this harness never stages an
  // S/T/D/STOP command (statement is always nullptr, see below), so Planner
  // is never actually engaged and any config is fine.
  Subsystems::Planner plannerA;
  plannerA.configure(msg::PlannerConfig());
  MotionLoopState motionStateA;
  motionStateA.poseEstimator = &poseA;
  CommandProcessor processorA;   // empty table -- never dispatched (statement is always nullptr)
  DevLoop loopA;
  loopA.hardware = &hardwareA;
  loopA.drivetrain = &drivetrainA;
  loopA.poseEstimator = &poseA;
  loopA.telemetry = &telemetryA;
  loopA.processor = &processorA;
  loopA.watchdog = &watchdogA;
  loopA.devState = &devStateA;
  loopA.planner = &plannerA;
  loopA.motionState = &motionStateA;
  loopA.defaultReply = nullptr;   // never invoked -- the watchdog never fires this test
  loopA.defaultReplyCtx = nullptr;

  hardwareA.motor(leftPort).apply(msg::MotorCommand{}.setDutyCycle(leftDuty));
  hardwareA.motor(rightPort).apply(msg::MotorCommand{}.setDutyCycle(rightDuty));
  if (driveActive) {
    drivetrainA.setWheelTargets(100.0f, 100.0f);   // any nonzero target -- just to flip active()
  }

  // --- Instance REF: driven by hand, ONE poseEstimator.tick() per pass. ---
  Subsystems::SimHardware hardwareREF(configs);
  hardwareREF.begin();
  Subsystems::Drivetrain drivetrainREF;
  drivetrainREF.configure(dtConfig);
  Subsystems::PoseEstimator poseREF;
  poseREF.configure(dtConfig);

  hardwareREF.motor(leftPort).apply(msg::MotorCommand{}.setDutyCycle(leftDuty));
  hardwareREF.motor(rightPort).apply(msg::MotorCommand{}.setDutyCycle(rightDuty));
  if (driveActive) {
    drivetrainREF.setWheelTargets(100.0f, 100.0f);
  }

  uint32_t now = 0;
  for (int i = 0; i < passCount; ++i) {
    now += 20;   // [ms] -- passCount*20 stays well under the watchdog's 1000ms window

    devLoopTick(loopA, now, nullptr);
    oneReferencePass(hardwareREF, drivetrainREF, poseREF, now);

    msg::PoseEstimate encA = poseA.encoderPose();
    msg::PoseEstimate encREF = poseREF.encoderPose();
    msg::PoseEstimate fusedA = poseA.fusedPose();
    msg::PoseEstimate fusedREF = poseREF.fusedPose();

    char label[96];
    std::snprintf(label, sizeof(label), "pass %d", i);

    checkNear(encA.pose.x, encREF.pose.x, 0.0f, std::string(label) + ": encoderPose().pose.x matches the once-per-pass reference");
    checkNear(encA.pose.y, encREF.pose.y, 0.0f, std::string(label) + ": encoderPose().pose.y matches the once-per-pass reference");
    checkNear(encA.pose.h, encREF.pose.h, 0.0f, std::string(label) + ": encoderPose().pose.h matches the once-per-pass reference");
    checkNear(fusedA.pose.x, fusedREF.pose.x, 0.0f, std::string(label) + ": fusedPose().pose.x matches the once-per-pass reference");
    checkNear(fusedA.pose.y, fusedREF.pose.y, 0.0f, std::string(label) + ": fusedPose().pose.y matches the once-per-pass reference");
    checkNear(fusedA.pose.h, fusedREF.pose.h, 0.0f, std::string(label) + ": fusedPose().pose.h matches the once-per-pass reference");
  }

  // Sanity: the sequence actually moved the robot -- a trivially-passing
  // all-zero test (e.g. both instances stuck at zero because neither ever
  // advanced) would satisfy every checkNear() call above without proving
  // anything.
  msg::PoseEstimate finalEnc = poseA.encoderPose();
  checkTrue(std::fabs(finalEnc.pose.x) > 5.0f || std::fabs(finalEnc.pose.y) > 5.0f ||
                std::fabs(finalEnc.pose.h) > 0.01f,
            "sanity: the driven sequence actually produced motion");
}

}  // namespace

int main() {
  // Scenario 1: drivetrain left INACTIVE throughout (ports() must still be
  // queried and the new step must still run every pass, unconditionally --
  // the OLDER Drivetrain-governance block, by contrast, is gated on
  // active() and does NOT run here).
  runComparison(/*leftPort=*/1, /*rightPort=*/2, /*leftDuty=*/0.4f, /*rightDuty=*/0.55f,
                /*driveActive=*/false, /*passCount=*/20,
                "devLoopTick() advances PoseEstimator exactly once per pass "
                "(drivetrain inactive -- default ports 1/2)");

  // Scenario 2: drivetrain ACTIVE throughout (the mid-pass governance block
  // between the two hardware.tick() slices genuinely runs) -- the new step
  // must still fire exactly once per pass and land on the SAME once-per-pass
  // reference trajectory.
  runComparison(/*leftPort=*/1, /*rightPort=*/2, /*leftDuty=*/0.4f, /*rightDuty=*/0.55f,
                /*driveActive=*/true, /*passCount=*/20,
                "devLoopTick() advances PoseEstimator exactly once per pass "
                "(drivetrain ACTIVE -- governance block runs mid-pass)");

  // Scenario 3: the bound pair is rebound away from the default {1, 2} to
  // {3, 4} -- proving devLoopTick() genuinely QUERIES drivetrain.ports()
  // rather than hardcoding the default pair.
  runComparison(/*leftPort=*/3, /*rightPort=*/4, /*leftDuty=*/0.5f, /*rightDuty=*/0.3f,
                /*driveActive=*/false, /*passCount=*/20,
                "devLoopTick() respects a rebound port pair (3/4), not a "
                "hardcoded default");

  if (g_failureCount == 0) {
    std::printf("OK: devLoopTick() advances Subsystems::PoseEstimator exactly once per pass\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the devLoopTick()/PoseEstimator scenarios\n",
              g_failureCount);
  return 1;
}
