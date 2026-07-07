// dev_loop_pose_estimator_harness.cpp — off-hardware acceptance harness for
// ticket 082-003, rewired for sprint 087 ticket 007's real cyclic executive
// (Rt::MainLoop, source/runtime/main_loop.{h,cpp} -- replaces ticket 006's
// transitional LoopContext/runLoopPass(), source/dev_loop.{h,cpp}, deleted
// by ticket 007). Proves Rt::MainLoop::tick() advances the wired
// Subsystems::PoseEstimator by EXACTLY one PoseEstimator::tick() call per
// pass, with the CORRECT (correctly-bound, correctly-latency-staged)
// observations — this ticket's single most important correctness property
// for the pose-estimation step specifically.
//
// --- What changed from the pre-007 version of this harness ---
//
// The pre-007 harness's REF pipeline read the FRESHEST same-pass
// hardware/drivetrain state (ticket 006's transitional same-pass
// feed-forward, matching devLoopTick()'s own same-pass reads). Ticket 007's
// REAL cyclic executive is synchronous-update (architecture-update-r1.md
// Decision 6): PoseEstimator::tick() now reads bb.motor[]/bb.otos as
// COMMITTED at the end of the PREVIOUS pass (x[k]), never a same-pass-fresh
// read — and Decision 2 adds a SECOND one-pass hop for Drivetrain's own
// output (Drivetrain::takeCommand() -> bb.motorIn[], drained by
// Hardware::tick() the NEXT pass, not applied to hardware directly the same
// pass the way ticket 006's transitional loop did). REF below is rewritten
// to mirror this exactly: a persistent, committed "x[k]" snapshot (motor
// state, otos state) carried between calls, and a persistent motorIn[]
// mailbox pair standing in for bb.motorIn[] — reproducing the SAME
// one-pass-per-hop latency instance A (driven through the real
// Rt::MainLoop::tick()) now has, so the two pipelines' PoseEstimator state
// still matches bit-for-bit every pass IF Rt::MainLoop::tick() calls
// poseEstimator.tick() correctly.
//
// --- How "exactly once, with the right (committed) arguments" is tested
// here, and what is deliberately NOT attempted ---
//
// PoseEstimator::tick() is a plain (non-virtual) method with no seam to
// intercept a call through (no vtable, no mock). A literal "call tick()
// twice in immediate succession with the SAME cached (now, leftObs,
// rightObs, otosObs) arguments" mutant is therefore not just untestable
// here, it is PROVABLY A MATHEMATICAL NO-OP for this exact implementation
// (see the pre-007 version of this comment, still true: zero encoder delta,
// zero dt, EkfTiny::predict() at identity) — same-instant repeat calls are
// inert by construction, so this harness does not chase it.
//
// What IS a real, observable correctness property — and what this harness
// proves — is that Rt::MainLoop::tick() invokes poseEstimator.tick()
// UNCONDITIONALLY once per pass, with the CORRECT bound-port, CORRECT-
// latency-staged observations, regardless of the drivetrain's
// active()/inactive() state or which port pair is bound. Proven by running
// the REAL Rt::MainLoop (instance "A") alongside an independently,
// manually-driven reference pipeline (instance "REF") that mirrors
// Rt::MainLoop::tick()'s own documented sequencing by hand. Both pipelines
// are fed byte-identical motor commands and `now` sequences, so their
// SimHardware/PoseEstimator state is deterministic and must match
// bit-for-bit after every single pass IF Rt::MainLoop::tick() calls
// PoseEstimator::tick() exactly once, with the right arguments, every pass:
//   - if Rt::MainLoop::tick() skipped the call some pass, A would lag REF's
//     accumulated motion — caught (scenario 1: drivetrain left inactive
//     throughout; scenario 2: drivetrain active throughout).
//   - if Rt::MainLoop::tick() used the wrong port pair (e.g. hardcoding
//     ports 1/2 instead of querying drivetrain.ports()), A would read the
//     WRONG motors' state whenever the bound pair is rebound away from
//     {1,2} — caught (scenario 3: ports rebound to {3,4}).
//
// Same ad hoc-compile convention as the other tests/sim/unit/*_harness.cpp
// files (hand-rolled assertions, PASS/FAIL per scenario, nonzero exit on any
// failure) — compiled by test_dev_loop_pose_estimator.py together with the
// real source/runtime/main_loop.cpp, source/subsystems/{drivetrain,
// sim_hardware,pose_estimator}.cpp, source/estimation/ekf_tiny.cpp,
// source/commands/{arg_parse,command_processor,dev_commands}.cpp,
// source/kinematics/body_kinematics.cpp, source/hal/sim/*.cpp, and
// source/hal/velocity_pid.cpp, with -DHOST_BUILD -DROBOT_DEV_BUILD=1
// (main_loop.cpp is gated behind ROBOT_DEV_BUILD — see main_loop.h's file
// header) and libraries/tinyekf/ on the include path (ekf_tiny.h's
// tinyekf.h is header-only).
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "runtime/blackboard.h"
#include "runtime/commands.h"
#include "runtime/main_loop.h"
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

// RefPipeline -- the independently, by-hand-driven reference pipeline.
// Mirrors Rt::MainLoop::tick()'s pose-estimation-relevant sequencing
// EXACTLY (main_loop.cpp): hardware.tick() draining a persistent motorIn[]
// (standing in for bb.motorIn[] -- Decision 2's per-port unpack), Drivetrain
// governance (posting its OWN output back into that SAME motorIn[] --
// gated on active(), reproducing the routeOutputs discard-when-standby
// rule), a COMMITTED motor-state snapshot (standing in for bb.motor[]),
// and a COMMITTED otos snapshot (standing in for bb.otos/bb.otosValid) --
// both read by poseEstimator.tick() ONE PASS STALE relative to this same
// pass's own fresh samples (Decision 6), exactly like instance A.
struct RefPipeline {
  Subsystems::SimHardware& hardware;
  Subsystems::Drivetrain& drivetrain;
  Subsystems::PoseEstimator& poseEstimator;

  Rt::Mailbox<msg::MotorCommand> motorIn[Subsystems::Hardware::kPortCount];
  msg::MotorState committedLeft;
  msg::MotorState committedRight;
  msg::PoseEstimate committedOtos;
  bool committedOtosValid = false;

  RefPipeline(Subsystems::SimHardware& hw, Subsystems::Drivetrain& dt,
              Subsystems::PoseEstimator& pe)
      : hardware(hw), drivetrain(dt), poseEstimator(pe) {}

  void tick(uint32_t now) {
    bool noMotorResetInYet[Subsystems::Hardware::kPortCount] = {false, false, false, false};
    hardware.tick(now, motorIn, noMotorResetInYet);

    Subsystems::DrivetrainPorts p = drivetrain.ports();
    Rt::Mailbox<msg::DrivetrainCommand> noDriveInYet;   // this harness never posts driveIn
    // Both drivetrain.tick() and poseEstimator.tick() (below) read
    // committedLeft/committedRight as x[k] -- i.e. whatever was committed at
    // the END of the PREVIOUS call, BEFORE this pass's own commit (further
    // down) refreshes them. This mirrors Rt::MainLoop::tick()'s own Decision
    // 6 ordering exactly (main_loop.cpp commits bb.motor[] only AFTER both
    // drivetrain_.tick() and poseEstimator_.tick() have already run this
    // pass) -- committing here BEFORE poseEstimator.tick() would give REF an
    // extra same-pass freshness instance A does not have, permanently
    // one-pass-ahead of A (caught empirically while implementing this
    // ticket: encoderPose()'s pure, unsmoothed accumulator showed an exact,
    // permanent one-pass divergence until this ordering was corrected).
    drivetrain.tick(now, committedLeft, committedRight, noDriveInYet);
    if (drivetrain.hasCommand()) {
      Hal::DrivetrainToHardwareCommand cmd = drivetrain.takeCommand();
      if (drivetrain.active()) {
        motorIn[cmd.wheel[0].port - 1].post(cmd.wheel[0].command);
        motorIn[cmd.wheel[1].port - 1].post(cmd.wheel[1].command);
      }
      // else: discard -- mirrors routeOutputs()'s own standby-discard rule.
    }

    Hal::Odometer* odometer = hardware.odometer();
    Rt::WorkQueue<Rt::PoseResetCommand, 4> noPoseResetInYet;
    poseEstimator.tick(now, committedLeft, committedRight,
                        committedOtosValid ? &committedOtos : nullptr, noPoseResetInYet);

    // COMMIT x[k+1] -- AFTER both reads above, exactly like
    // Rt::MainLoop::tick()'s own commit step.
    committedLeft = hardware.motor(p.left).state();
    committedRight = hardware.motor(p.right).state();
    if (odometer != nullptr) {
      odometer->tick(now);
      committedOtos = odometer->pose();
      committedOtosValid = true;
    } else {
      committedOtosValid = false;
    }
  }
};

// runComparison -- drives instance "A" (the REAL Rt::MainLoop::tick(), via a
// fully wired Rt::MainLoop) and instance "REF" (RefPipeline::tick(), by
// hand) in lockstep over `passCount` passes, 20ms apart, asserting A's
// poseEstimator exactly matches REF's after every single pass. leftPort/
// rightPort bind both drivetrains' pair; leftDuty/rightDuty are applied
// once, up front, to BOTH pipelines' bound motors (straight or curved
// motion, asymmetric so dTheta != 0 too); if driveActive, both drivetrains
// are additionally put into WHEELS-arm active() authority
// (setWheelTargets) so the governance block genuinely executes this pass,
// proving the pose-estimation step is unconditional regardless.
void runComparison(uint32_t leftPort, uint32_t rightPort, float leftDuty, float rightDuty,
                    bool driveActive, int passCount, const std::string& scenarioName) {
  beginScenario(scenarioName);

  msg::MotorConfig configs[Subsystems::Hardware::kPortCount];
  fillDefaultConfigs(configs);
  msg::DrivetrainConfig dtConfig = makeDtConfig(leftPort, rightPort);

  // --- Instance A: driven ENTIRELY through the real Rt::MainLoop::tick(). ---
  Subsystems::SimHardware hardwareA(configs);
  hardwareA.begin();
  Subsystems::Drivetrain drivetrainA;
  drivetrainA.configure(dtConfig);
  Subsystems::PoseEstimator poseA;
  poseA.configure(dtConfig);
  // 084-002: Rt::MainLoop::tick() dereferences its own Planner reference
  // unconditionally every pass (the motion-executor step) -- wired here
  // only to satisfy that non-null contract; this harness never stages an
  // S/T/D/STOP command (bb.motionIn is never posted to), so Planner is
  // never actually engaged and any config is fine.
  Subsystems::Planner plannerA;
  plannerA.configure(msg::PlannerConfig());
  Rt::Blackboard bbA;
  // Never actually dispatched through (this harness never routes a
  // command) -- Rt::MainLoop::tick() itself needs no CommandRouter/
  // Configurator reference at all (087-007: those stay top-level objects
  // the SLACK phase alone calls -- see main_loop.h's class comment), unlike
  // ticket 006's transitional LoopContext, which needed both to link.
  Rt::MainLoop loopA(hardwareA, drivetrainA, poseA, plannerA,
                     /*serialReply=*/nullptr, /*serialCtx=*/nullptr,
                     /*radioReply=*/nullptr, /*radioCtx=*/nullptr);

  hardwareA.motor(leftPort).apply(msg::MotorCommand{}.setDutyCycle(leftDuty));
  hardwareA.motor(rightPort).apply(msg::MotorCommand{}.setDutyCycle(rightDuty));
  if (driveActive) {
    drivetrainA.setWheelTargets(100.0f, 100.0f);   // any nonzero target -- just to flip active()
  }

  // --- Instance REF: driven by hand, ONE poseEstimator.tick() per pass,
  // mirroring Rt::MainLoop::tick()'s own one-pass-per-hop latency. ---
  Subsystems::SimHardware hardwareREF(configs);
  hardwareREF.begin();
  Subsystems::Drivetrain drivetrainREF;
  drivetrainREF.configure(dtConfig);
  Subsystems::PoseEstimator poseREF;
  poseREF.configure(dtConfig);
  RefPipeline ref(hardwareREF, drivetrainREF, poseREF);

  hardwareREF.motor(leftPort).apply(msg::MotorCommand{}.setDutyCycle(leftDuty));
  hardwareREF.motor(rightPort).apply(msg::MotorCommand{}.setDutyCycle(rightDuty));
  if (driveActive) {
    drivetrainREF.setWheelTargets(100.0f, 100.0f);
  }

  uint32_t now = 0;
  for (int i = 0; i < passCount; ++i) {
    now += 20;   // [ms] -- passCount*20 stays well under the watchdog's 1000ms window

    loopA.tick(bbA, now);
    ref.tick(now);

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
                "Rt::MainLoop::tick() advances PoseEstimator exactly once per pass "
                "(drivetrain inactive -- default ports 1/2)");

  // Scenario 2: drivetrain ACTIVE throughout (the mid-pass governance block
  // between the two hardware.tick() slices genuinely runs) -- the new step
  // must still fire exactly once per pass and land on the SAME once-per-pass
  // reference trajectory.
  runComparison(/*leftPort=*/1, /*rightPort=*/2, /*leftDuty=*/0.4f, /*rightDuty=*/0.55f,
                /*driveActive=*/true, /*passCount=*/20,
                "Rt::MainLoop::tick() advances PoseEstimator exactly once per pass "
                "(drivetrain ACTIVE -- governance block runs mid-pass)");

  // Scenario 3: the bound pair is rebound away from the default {1, 2} to
  // {3, 4} -- proving Rt::MainLoop::tick() genuinely QUERIES
  // drivetrain.ports() rather than hardcoding the default pair.
  runComparison(/*leftPort=*/3, /*rightPort=*/4, /*leftDuty=*/0.5f, /*rightDuty=*/0.3f,
                /*driveActive=*/false, /*passCount=*/20,
                "Rt::MainLoop::tick() respects a rebound port pair (3/4), not a "
                "hardcoded default");

  if (g_failureCount == 0) {
    std::printf("OK: Rt::MainLoop::tick() advances Subsystems::PoseEstimator exactly once per pass\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Rt::MainLoop/PoseEstimator scenarios\n",
              g_failureCount);
  return 1;
}
