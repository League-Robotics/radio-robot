// stop_path_safety_harness.cpp -- off-hardware acceptance harness for ticket
// 133-001, App::RobotLoop::zeroUnownedMotion(): the derived-idle safety
// arbitration step that runs immediately ahead of drive_.tick(), the single
// actuation path.
//
// THE DEFECT THESE SCENARIOS EXIST FOR (measured on vevov, 2026-08-03,
// 16/16 reproductions). A host that issues a stop ONCE and then goes quiet
// got 936mm of continued travel with no decay -- still going when the
// capture ended -- and estop() failed 5 of 6 attempts. The Nezha brick
// physically latches its last commanded speed and does not reset on an
// nRF52 reset, so a lost zero write is permanent, not transient. Above the
// two encoder-gated leaf defences, the ownership handoff left a gap nothing
// covered: App::Drive::update() publishes ONE zero pair on the expiry cycle
// and then returns early forever after (`if (!owned) return;`), and
// Motion::Planner::update() runs unconditionally but republishes only ITS
// OWN idea of the command. Nothing re-stated "no one is driving, so the
// speed is zero" on the cycles in between.
//
// WHY THESE ARE PER-CYCLE ASSERTIONS, NOT END-STATE ASSERTIONS. The whole
// shape of the defect is "correct on the transition cycle, wrong on every
// cycle after it." A test that samples the end state, or only the expiry
// cycle, passes against the defect -- which is precisely why the defect
// shipped. Every scenario below therefore asserts on EVERY cycle of the
// tail and reports the FIRST offending cycle index, not a final value.
//
// Built on TestSim::SimHarness (src/firm/platform/host/sim_harness.h) -- the composition
// root that mirrors src/firm/main.cpp via App::composeRobot(), so the
// App::RobotLoop under test here is the shipped graph, not a hand-wired
// stand-in. Same choice, and for the same reason, as
// robot_loop_tlm_harness.cpp (see that file's header for why
// src/tests/sim/unit/app_robot_loop_harness.cpp is not an option).
//
// NOT COVERED HERE, deliberately: the App::Drive half of 133-001 (arming
// the stop re-assertion window on the commanded nonzero->zero transition of
// the duty pair) is a Drive-internal contract with the motor leaves, proven
// at that layer by app_drive_harness.cpp's own
// scenarioCommandedStopArmsReassertionWithEncoderAtRest() -- it needs a
// motor double whose velocity() can be pinned at rest independent of any
// plant physics, which is exactly what this system-level harness cannot
// give. Hardware verification of both halves together is sprint 133 ticket
// 004's job, on tovez.
//
// WHAT THESE SCENARIOS CAN AND CANNOT FALSIFY TODAY -- stated plainly so no
// one later reads a green run here as proof the runaway is reproduced and
// fixed in sim. It is not. MEASURED 2026-08-03 by compiling this harness
// against a build with zeroUnownedMotion()'s call site commented out: all
// three scenarios below still PASS. The reason is specific and worth
// knowing: in this sim, nothing ever leaves a stale nonzero on the
// blackboard for the guard to catch. App::RobotLoop::handleWheels() calls
// planner_.estop() (zeroing Motion::Planner's own cmdLeft_/cmdRight_)
// before arming Drive, so the planner's unconditional republish after a
// WHEELS expiry republishes a ZERO; and a Move that times out mid-motion
// deactivates and lands its staged pair on zero in the SAME cycle (verified
// directly -- there is no lingering nonzero drain window either). Both
// halves of the software handoff are already clean here.
//
// So what these scenarios are: standing, per-cycle regression guards that
// PIN the invariant -- they fail the moment any future change lets a
// nonzero survive into an unowned window, which is the class of change that
// produced the defect. What they are not: a reproduction of the vevov
// runaway. That defect's live mechanism is a LOST WRITE at the bus/leaf
// layer (the brick latches its last speed and never sees the zero), which
// TestSim::SimPlant does not model at all -- every I2C write it accepts
// succeeds. The two tests that genuinely bite against the pre-fix code are
// app_drive_harness.cpp's scenario (fix b, via a MockMotor that can hold
// velocity at rest) and test_stop_path_safety.py's own structural
// test_arbitration_step_writes_only_zero() (the monotone contract).
//
// Hand-rolled assertions, PASS/FAIL per scenario, nonzero exit on any
// failure -- mirrors every other src/tests/sim/{unit,system} harness.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "bench_test_config.h"
#include "sim_harness.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors every other tests/sim harness
// in this codebase) ---------------------------------------------------------

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

// --- Shared fixture constants ---------------------------------------------

// [mm/s] a plainly-nonzero teleop speed, comfortably above App::Drive's own
// kRestVelocity and any configured speed floor, so "the wheels really were
// turning" is never in doubt.
constexpr float kTeleopSpeed = 150.0f;

// Steps needed to cover a wall-clock duration at the CURRENT loop period.
//
// Never hardcode a cycle count in this file. kCycle has moved three times
// (40 -> 50 -> 32) and every hardcoded count silently changed what it
// covered: "12 cycles" was 600ms of a 500ms command at kCycle=50 and became
// 384ms at kCycle=32, so the command had NOT expired when the scenario
// asserted that it had -- the harness reported a runaway that was really
// just a test that stopped waiting long enough.
constexpr int cyclesFor(uint32_t duration) {  // [ms]
  return static_cast<int>((duration + App::RobotLoop::kCycle - 1) / App::RobotLoop::kCycle);
}

// [ms] one WHEELS command's bounded lifetime -- long enough that the plant
// is genuinely in motion when it expires.
constexpr float kWheelsDuration = 500.0f;

// Cycles to run after expiry with the host SILENT -- no further command of
// any kind. The vevov capture ended, still moving, well inside this window.
constexpr int kSilentTailCycles = cyclesFor(6000);  // 6s

// [mm] residual travel allowed across a stretch of cycles that must be
// motionless. The plant is a first-order lag, so a settle window is given
// before any motionless assertion; this is the noise floor after it, not a
// decay allowance.
constexpr float kMotionlessTolerance = 1.0f;

// ===========================================================================
// 1. THE REGRESSION. A WHEELS command runs to expiry with a silent host;
//    cmdVelocity must read exactly zero on EVERY cycle after expiry, not
//    merely on the expiry cycle itself, and the robot must actually stop
//    travelling and stay stopped.
// ===========================================================================

void scenarioSilentHostAfterExpiryHoldsZeroOnEveryCycle() {
  beginScenario("133-001: after a WHEELS command expires with the host silent, cmdVelocity is "
                "exactly zero on EVERY subsequent cycle and travel stops");

  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  sim.step(3);  // settle: both leaves' own one-time zero-duty activation writes land

  sim.injectWheels(kTeleopSpeed, kTeleopSpeed, kWheelsDuration, /*id=*/1, /*corrId=*/1);

  // --- Command phase: prove the setup is real before asserting anything
  // about the tail. A tail-only assertion would pass trivially against a
  // robot that never moved at all.
  const float startX = sim.trueX();  // [mm]
  int ownedCycles = 0;
  int nonzeroCycles = 0;
  // Cover the command's WHOLE bounded lifetime plus margin -- derived, so
  // the phase still outlasts the command at any kCycle.
  const int kCommandPhaseCycles = cyclesFor(static_cast<uint32_t>(kWheelsDuration)) + 4;
  for (int i = 0; i < kCommandPhaseCycles; ++i) {
    sim.step(1);
    if (sim.drive().owns()) ++ownedCycles;
    if (sim.driveTargetVelLeft() != 0.0f && sim.driveTargetVelRight() != 0.0f) ++nonzeroCycles;
  }
  checkTrue(ownedCycles >= 8, "setup: App::Drive genuinely owned motion for most of the command");
  checkTrue(nonzeroCycles >= 8, "setup: a nonzero cmdVelocity really was staged while it owned");
  checkTrue(!sim.drive().owns(), "setup: the command has expired by the end of the command phase");
  const float commandedTravel = std::fabs(sim.trueX() - startX);  // [mm]
  checkTrue(commandedTravel > 20.0f,
            "setup: the robot really was travelling when the command expired (measured " +
                std::to_string(commandedTravel) + "mm)");

  // --- The tail. The host is silent from here on: not one further command,
  // not a stop, not an estop. This is the exact vevov condition.
  int firstNonzeroCycle = -1;
  float firstNonzeroLeft = 0.0f;
  float firstNonzeroRight = 0.0f;
  int idleOwnershipViolations = 0;
  for (int i = 0; i < kSilentTailCycles; ++i) {
    sim.step(1);
    if (sim.planner().active() || sim.drive().owns()) ++idleOwnershipViolations;
    const float left = sim.driveTargetVelLeft();    // [mm/s]
    const float right = sim.driveTargetVelRight();  // [mm/s]
    if ((left != 0.0f || right != 0.0f) && firstNonzeroCycle < 0) {
      firstNonzeroCycle = i;
      firstNonzeroLeft = left;
      firstNonzeroRight = right;
    }
  }
  checkTrue(idleOwnershipViolations == 0,
            "the tail really is the unowned case the guard exists for -- neither decider claimed "
            "motion on any of its cycles");
  if (firstNonzeroCycle >= 0) {
    fail("cmdVelocity went nonzero on tail cycle " + std::to_string(firstNonzeroCycle) + " (left " +
         std::to_string(firstNonzeroLeft) + ", right " + std::to_string(firstNonzeroRight) +
         ") -- a stale target survived the expiry, which is the runaway");
  }

  // --- And the physical consequence, which is what the defect was measured
  // in: travel stops and STAYS stopped. Sampled after a settle window (the
  // plant is a first-order lag; coasting to rest is legitimate) and then
  // held across a long motionless stretch.
  sim.step(cyclesFor(2000));  // 2s settle
  const float settledX = sim.trueX();  // [mm]
  const float settledY = sim.trueY();  // [mm]
  sim.step(cyclesFor(4000));           // 4s of nothing whatsoever
  const float drift = std::hypot(sim.trueX() - settledX, sim.trueY() - settledY);  // [mm]
  checkTrue(drift < kMotionlessTolerance,
            "the robot stays stopped through 4s of host silence (measured " +
                std::to_string(drift) + "mm of residual travel)");
}

// ===========================================================================
// 2. NEITHER DECIDER OWNS AT BOOT. The very first actuation of a freshly
//    booted robot happens before any command has ever arrived -- planner
//    inactive, Drive never armed. The commanded speed must be zero there
//    too, and stay zero for as long as nobody asks for motion.
// ===========================================================================

void scenarioNeitherDeciderOwnsAtBootYieldsZeroBeforeFirstActuation() {
  beginScenario("133-001: at boot, with the planner inactive and Drive never armed, cmdVelocity "
                "is zero before the first actuation and the wheels never move");

  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();

  // The guard's precondition, stated before it is relied on -- otherwise
  // every assertion below could pass vacuously on a robot that happened to
  // have an owner.
  checkTrue(!sim.planner().active(), "setup: the planner is inactive at boot");
  checkTrue(!sim.drive().owns(), "setup: App::Drive has never been armed");
  checkTrue(sim.driveTargetVelLeft() == 0.0f && sim.driveTargetVelRight() == 0.0f,
            "cmdVelocity is zero BEFORE the first cycle, i.e. before the first actuation");

  const float startX = sim.trueX();  // [mm]
  const float startY = sim.trueY();  // [mm]
  int firstNonzeroCycle = -1;
  int firstNonzeroDutyCycle = -1;
  for (int i = 0; i < 60; ++i) {
    sim.step(1);
    if ((sim.driveTargetVelLeft() != 0.0f || sim.driveTargetVelRight() != 0.0f) &&
        firstNonzeroCycle < 0) {
      firstNonzeroCycle = i;
    }
    if ((sim.motorLeft().appliedDuty() != 0.0f || sim.motorRight().appliedDuty() != 0.0f) &&
        firstNonzeroDutyCycle < 0) {
      firstNonzeroDutyCycle = i;
    }
  }
  if (firstNonzeroCycle >= 0) {
    fail("cmdVelocity went nonzero on idle cycle " + std::to_string(firstNonzeroCycle) +
         " with no command ever issued");
  }
  if (firstNonzeroDutyCycle >= 0) {
    fail("a nonzero duty reached a wheel on idle cycle " + std::to_string(firstNonzeroDutyCycle) +
         " with no command ever issued");
  }
  const float drift = std::hypot(sim.trueX() - startX, sim.trueY() - startY);  // [mm]
  checkTrue(drift < kMotionlessTolerance,
            "an unowned, uncommanded robot does not move (measured " + std::to_string(drift) +
                "mm)");
}

// ===========================================================================
// 3. THE MONOTONE CONTRACT. The arbitration step may write ONLY 0.0f. Two
//    halves, because "never writes a nonzero" fails in two different
//    directions:
//      (a) it must never ORIGINATE motion -- covered by scenario 2's idle
//          stretch and re-proven here across command boundaries;
//      (b) it must never OVERWRITE a decider that does own motion -- so
//          while a decider owns, cmdVelocity must be exactly the value that
//          decider staged, bit for bit.
//    Run over several arm/expire cycles, the observed cmdVelocity values
//    are therefore drawn from exactly two: 0.0f, and the commanded pair.
//    Any third value could only have come from the arbiter.
// ===========================================================================

void scenarioArbitrationStepNeverWritesNonzero() {
  beginScenario("133-001: across repeated arm/expire boundaries, cmdVelocity is only ever exactly "
                "0.0f or exactly the decider's own commanded value -- the arbiter writes no "
                "third value");

  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  sim.step(3);

  int unexplainedValueCycles = 0;
  int ownedNonzeroCycles = 0;
  int unownedZeroCycles = 0;
  float firstUnexplainedLeft = 0.0f;
  float firstUnexplainedRight = 0.0f;

  for (int leg = 0; leg < 3; ++leg) {
    // Alternate the commanded direction so a stale value from the previous
    // leg is a DIFFERENT number from this leg's own -- a guard that merely
    // held the last value would otherwise be indistinguishable from one
    // that re-derived it.
    const float speed = (leg % 2 == 0) ? kTeleopSpeed : -kTeleopSpeed;  // [mm/s]
    sim.injectWheels(speed, speed, kWheelsDuration, /*id=*/static_cast<uint32_t>(leg + 1),
                     /*corrId=*/static_cast<uint32_t>(leg + 1));

    // Cover the command, then an unowned tail of the same length again
    // before the next leg arms -- both derived from the loop period.
    const int kLegCycles = cyclesFor(static_cast<uint32_t>(kWheelsDuration)) + cyclesFor(1500);
    for (int i = 0; i < kLegCycles; ++i) {
      sim.step(1);
      const float left = sim.driveTargetVelLeft();    // [mm/s]
      const float right = sim.driveTargetVelRight();  // [mm/s]
      const bool owned = sim.drive().owns();
      const bool zeroPair = (left == 0.0f && right == 0.0f);
      const bool commandedPair = (left == speed && right == speed);

      if (owned && commandedPair) ++ownedNonzeroCycles;
      if (!owned && zeroPair) ++unownedZeroCycles;

      // The one thing the arbiter must never produce: a value that is
      // neither zero nor what the owning decider asked for.
      if (!zeroPair && !commandedPair) {
        if (unexplainedValueCycles == 0) {
          firstUnexplainedLeft = left;
          firstUnexplainedRight = right;
        }
        ++unexplainedValueCycles;
      }
      // ... and, symmetrically, it must never zero a decider that DOES own
      // motion. While Drive owns an armed, unexpired command, the staged
      // pair is its own target, untouched.
      if (owned && zeroPair && sim.robotLoop().state().command.moveActive) {
        fail("cmdVelocity was zero while App::Drive owned an ACTIVE command -- the arbiter is "
             "not permitted to override a live decider");
      }
    }
  }

  if (unexplainedValueCycles > 0) {
    fail("cmdVelocity took a value neither zero nor the commanded pair on " +
         std::to_string(unexplainedValueCycles) + " cycle(s); first was (" +
         std::to_string(firstUnexplainedLeft) + ", " + std::to_string(firstUnexplainedRight) +
         ") -- App::RobotLoop originated a value no decider asked for");
  }
  checkTrue(ownedNonzeroCycles >= 20,
            "setup: the owned/nonzero half was genuinely exercised across the three legs (" +
                std::to_string(ownedNonzeroCycles) + " cycles)");
  checkTrue(unownedZeroCycles >= 60,
            "setup: the unowned/zero half was genuinely exercised across the three legs (" +
                std::to_string(unownedZeroCycles) + " cycles)");
}

}  // namespace

int main() {
  scenarioSilentHostAfterExpiryHoldsZeroOnEveryCycle();
  scenarioNeitherDeciderOwnsAtBootYieldsZeroBeforeFirstActuation();
  scenarioArbitrationStepNeverWritesNonzero();

  if (g_failureCount == 0) {
    std::printf("OK: all stop-path safety scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the stop-path safety scenarios\n", g_failureCount);
  return 1;
}
