// plant_harness.cpp -- off-hardware acceptance harness for ticket 105-003
// (SUC-020): proves TestSim::WheelPlant + TestSim::OtosPlant (this
// directory) satisfy the ticket's own acceptance criteria --
//   1. a velocity-step scenario shows a visible RAMP (not an instantaneous
//      step) with a time constant in the 120-140ms range;
//   2. two runs of the SAME command script with the SAME (implicit --
//      there is no RNG in this plant, see wheel_plant.h's file header)
//      seed produce bit-identical trajectories;
//   3. a pivot (differential-duty, turn-in-place) scenario's resulting
//      heading, read ENTIRELY through App::Odometry's own integration over
//      the plant's two wheel positions (never read from the plant
//      directly), is sane -- the sprint's own "B3 doesn't reappear" check.
//
// Drives the REAL Devices::NezhaMotor x2 + Devices::Otos + App::Odometry
// against a real Devices::I2CBus implementation, exactly as
// devices_motor_harness.cpp scenario 6 and app_odometry_harness.cpp already
// do for their own narrower scopes -- this harness generalizes that same
// proven idiom across the whole loop (both motors + OTOS), per
// architecture-update.md Decision 2. No App::RobotLoop involvement -- that
// composition is TestSim::SimHarness's own job (architecture-update.md Step
// 3's "Plant" boundary: "the plant is driven BY the harness, between
// cycles, never inside a runAndWait block").
//
// Bus: TestSim::SimPlant (tests/_infra/sim/sim_plant.{h,cpp}, ticket
// 108-002) -- sprint 108 ticket 001 reduced Devices::I2CBus to a pure
// interface and deleted its old scripted-FIFO HOST_BUILD fake
// (queueWrite()/queueRead()/errCount(), and WheelPlant/OtosPlant's own
// scriptEncoderResponse()/scriptPoseResponse() helpers that targeted it --
// see wheel_plant.h's/otos_plant.h's own file headers), so this file no
// longer scripts exact per-cycle bus responses; it drives SimPlant live
// (bus.tick(dt) each cycle, motor/otos calls read back whatever SimPlant's
// OWN WheelPlant/OtosPlant instances actually computed). SimPlant itself
// has zero App::RobotLoop dependency -- it is purely a wire-protocol
// responder over these same plant classes -- so reusing it here does not
// pull in the RobotLoop/sim_api composition layer this file's own header
// has always disclaimed; it only replaces the deleted scripted-FIFO
// plumbing with a real (if more capable) I2CBus implementation.
//
// Hand-rolled assertions, PASS/FAIL per scenario, nonzero exit on any
// failure -- mirrors every other src/tests/sim/unit harness's own shape. Run by
// test_plant.py, which compiles this file together with wheel_plant.cpp,
// otos_plant.cpp, sim_plant.cpp, and the HOST_BUILD Devices/App/Kinematics
// sources it needs, then runs the resulting binary via subprocess.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "app/odometry.h"
#include "devices/device_config.h"
#include "devices/device_types.h"
#include "devices/i2c_bus.h"
#include "devices/nezha_motor.h"
#include "devices/otos.h"
#include "sim_plant.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors every other src/tests/sim/unit
// harness in this codebase) -------------------------------------------------

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

void checkFloatEq(float actual, float expected, const std::string& what,
                   float tol = 1e-3f) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g (tol %g)",
                  what.c_str(), static_cast<double>(expected),
                  static_cast<double>(actual), static_cast<double>(tol));
    fail(buf);
  }
}

// --- Devices::NezhaMotor / Devices::Otos fixture helpers --------------------

// wheelTravelCalib [mm/deg]: the INVERSE of sim_plant.cpp's own
// kEncoderCountsPerMm (1.4187f, "counts = mm * 360/(pi*80.77) for the tovez
// wheel") -- duplicated here per this codebase's established per-file
// fixture-duplication convention (kEncoderCountsPerMm has internal linkage,
// not importable). sim_plant.cpp's handleMotorRead() now packs the
// simulated 0x46 encoder register as raw motor-shaft-degree COUNTS (matching
// the real Nezha register semantics), not bare millimetres -- NezhaMotor's
// own tick() decodes counts back to mm via `wheelTravelCalib`, exactly like
// a real robot's own per-wheel calibration. 1.0f (this fixture's own
// original value, from before commit 172a429d fixed handleMotorRead() to
// pack real counts instead of bare mm) was only lossless while the plant
// still packed raw mm; leaving it at 1.0f after that fix makes every
// position/velocity this harness reads through
// NezhaMotor/Odometry over-report by 1.4187x relative to WheelPlant's own
// ground truth (the scale mismatch that broke this file's own
// "OtosPlant tracks Odometry closely" pivot assertion -- OtosPlant reads
// WheelPlant's true position directly, bypassing this encoding entirely).
// Matching it here restores the round-trip a properly calibrated real robot
// gets for free.
constexpr float kWheelTravelCalib = 1.0f / 1.4187f;  // [mm/deg]

Devices::MotorConfig baseMotorConfig(uint32_t port) {
  Devices::MotorConfig cfg;
  cfg.port = port;
  cfg.fwdSign = 1;
  cfg.wheelTravelCalib = kWheelTravelCalib;
  cfg.velFiltAlpha = 1.0f;
  return cfg;
}

// ===========================================================================
// 1. Velocity-step scenario: a single wheel/motor, isolated on its own
//    SimPlant (nothing else attached to the bus). Proves the plant's own
//    simulated velocity RAMPS (not steps) toward the commanded target, with
//    a time constant in the 120-140ms range.
// ===========================================================================

void scenarioVelocityStepShowsRampWithTauInRange() {
  beginScenario("velocity step: plant velocity ramps, not steps, with tau in the 120-140ms range");

  checkTrue(TestSim::kDefaultTau >= 0.12f && TestSim::kDefaultTau <= 0.14f,
            "kDefaultTau itself sits in the bench-characterized 120-140ms range");

  TestSim::SimPlant bus;   // owns its own left/right WheelPlant + OtosPlant internally --
                            // this scenario only ever drives port 1 (left).

  Devices::MotorConfig cfg = baseMotorConfig(1);
  Devices::NezhaMotor motor(bus, cfg);
  motor.setPidEnabled(false);
  const float duty = 0.6f;
  motor.setDuty(duty);

  const float finalTargetVel = TestSim::kDefaultDutyVelMax * duty;   // 300 mm/s

  const float dtS = 0.02f;          // [s]
  const uint64_t dtUs = 20000;      // [us]
  uint64_t nowUs = 50000;           // starts >=35ms (118 ticket 003's jitter
                                     // margin, kMinWriteIntervalUs) so cycle
                                     // 0's first duty write is not
                                     // write-rate-throttled -- see
                                     // nezha_motor.cpp's writeRawDuty() and
                                     // every other harness's identical
                                     // convention (e.g.
                                     // scenarioNakedStopWriteIsRetriedNextTickNotLatched()).

  const int kCycles = 250;   // 5s of virtual time -- >>30*tau, fully converges
  std::vector<float> velTrace;
  velTrace.reserve(static_cast<size_t>(kCycles));

  for (int i = 0; i < kCycles; ++i) {
    // bus.tick(dt) integrates SimPlant's own left WheelPlant off whatever
    // duty motor.tick() last actually WROTE to the wire -- 0 until this
    // motor's own first duty write lands (mirrors the deleted scripted
    // harness's identical "plant.step(motor.appliedDuty(), dtS)" call,
    // just against the live wire-parsed duty instead of appliedDuty()'s
    // own getter directly).
    bus.tick(dtS);
    motor.requestSample();
    motor.tick(nowUs);
    velTrace.push_back(bus.wheelPlant(1).velocity());
    nowUs += dtUs;
  }

  checkTrue(velTrace[0] == 0.0f,
            "cycle 0: appliedDuty() was still 0 (no write had landed yet) -- plant starts at rest");

  // Not an instantaneous step: one dt after the duty first lands (cycle 1),
  // velocity is still a small fraction of the final target -- a true "step"
  // plant would already be at (or very near) finalTargetVel here.
  checkTrue(velTrace[1] < 0.3f * finalTargetVel,
            "one cycle after the duty write lands, velocity is still well below the final target (a ramp, not a step)");

  // Converges to the commanded target well within the run.
  checkFloatEq(velTrace.back(), finalTargetVel, "velocity converges to dutyVelMax*duty after many cycles", 1.0f);

  // Monotonic, no overshoot -- the signature of a first-order lag with no
  // oscillatory/derivative term.
  bool monotonic = true;
  for (size_t i = 1; i < velTrace.size(); ++i) {
    if (velTrace[i] + 1e-4f < velTrace[i - 1] || velTrace[i] > finalTargetVel + 1e-3f) {
      monotonic = false;
      break;
    }
  }
  checkTrue(monotonic, "velocity rises monotonically toward the target with no overshoot");

  // Time-constant check: the analytic first-order step response crosses
  // (1 - 1/e) ~= 63.2% of its final value at t == tau after the step
  // begins. The step begins being APPLIED at cycle 1 (cycle 0's plant.step()
  // still saw appliedDuty()==0 -- see above), so "time since step" for the
  // sample recorded at index i (i >= 1) is (i - 1) * dtS + dtS = i * dtS
  // measured from the start of cycle 1's OWN integration window -- i.e.
  // velTrace[i] is the value AFTER i total ramping steps starting at
  // index 1. Find the first index whose value crosses the 63.2% mark and
  // check its time falls within one dt of tau.
  const float crossing = finalTargetVel * (1.0f - std::exp(-1.0f));
  int crossingIndex = -1;
  for (size_t i = 1; i < velTrace.size(); ++i) {
    if (velTrace[i] >= crossing) {
      crossingIndex = static_cast<int>(i);
      break;
    }
  }
  checkTrue(crossingIndex > 0, "velocity trace actually crosses the 63.2%-of-final mark within the run");
  if (crossingIndex > 0) {
    // Elapsed ramping time since the duty first landed (cycle 1's own
    // integration is the first ramping step, at t=dtS since the step).
    float elapsedSinceStep = static_cast<float>(crossingIndex) * dtS;
    checkFloatEq(elapsedSinceStep, TestSim::kDefaultTau,
                 "the 63.2%-of-final crossing time matches tau (proves the time constant, within one cycle's dt)",
                 dtS + 1e-3f);
  }

}

// ===========================================================================
// Shared multi-device fixture: two NezhaMotor + one Otos + one App::Odometry,
// all sharing ONE SimPlant (its own left/right WheelPlant + OtosPlant).
// Used by BOTH the pivot scenario and the determinism scenario below --
// exercising "the WHOLE plant (both motors + OTOS)", not one leaf in
// isolation, per architecture-update.md's own framing of this ticket.
// ===========================================================================

constexpr float kTrackWidth = 130.0f;   // [mm]

struct CycleSample {
  float posLeft;
  float posRight;
  float velLeft;
  float velRight;
  float otosX;
  float otosY;
  float otosHeading;
  float odomX;
  float odomY;
  float odomTheta;
};

// Runs a fresh two-motor + OTOS + Odometry scenario for `cycles` cycles at a
// constant differential duty target, returning the full per-cycle trace.
// Every instance (bus, motors, otos, odometry) is constructed LOCAL to this
// call -- two calls with identical arguments are two fully independent
// runs, which the determinism scenario relies on.
std::vector<CycleSample> runScenario(float dutyLeft, float dutyRight, int cycles) {
  TestSim::SimPlant bus(kTrackWidth);   // trackWidth MUST match the App::Odometry instance
                                         // below -- see otos_plant.h's own "MUST match" comment.

  Devices::NezhaMotor motorLeft(bus, baseMotorConfig(1));
  Devices::NezhaMotor motorRight(bus, baseMotorConfig(2));
  motorLeft.setPidEnabled(false);
  motorRight.setPidEnabled(false);
  motorLeft.setDuty(dutyLeft);
  motorRight.setDuty(dutyRight);

  Devices::OtosConfig otosCfg;   // identity mounting (offsetX=offsetY=offsetYaw=0) --
                                  // see otos_plant.h's own "Identity-mounting assumption".
  Devices::RealOtos otos(bus, otosCfg);
  otos.begin();   // SimPlant answers the product-ID probe + init/config writes live --
                   // no bus scripting needed (the deleted scripted-FIFO fake's own
                   // exact-write-count bookkeeping does not apply to a real responder).

  App::Odometry odom(motorLeft, motorRight, kTrackWidth);

  const float dtS = 0.02f;        // [s]
  const uint64_t dtUs = 20000;    // [us] == Devices::Otos's own kReadPeriod, so
                                   // readDue() is true every single cycle.
  uint64_t nowUs = 50000;         // avoid the first-write throttle edge (see
                                   // the ramp scenario's identical comment).

  std::vector<CycleSample> trace;
  trace.reserve(static_cast<size_t>(cycles));

  for (int i = 0; i < cycles; ++i) {
    // Integrate SimPlant's own left/right WheelPlant + OtosPlant off
    // whatever duty each motor's own last tick() actually wrote to the
    // wire (0 until each motor's own first duty write lands) -- mirrors
    // the deleted scripted harness's identical per-cycle
    // plantLeft.step()/plantRight.step()/otosPlant.step() call sequence,
    // just against the live wire-parsed duty instead of appliedDuty()'s
    // own getter directly.
    bus.tick(dtS);

    // Call order: each motor's requestSample() (the 0x46 encoder-select
    // write) MUST be immediately followed by that SAME motor's own tick()
    // (the read that consumes the selection) before the OTHER motor
    // touches the bus -- unlike the deleted scripted-FIFO fake (a plain
    // response queue, order-insensitive across devices), SimPlant tracks
    // "which port is currently selected" as one piece of live protocol
    // state (sim_plant.h's own selectedPort_); interleaving both
    // requestSample() calls before either tick() would let the second
    // select overwrite the first before its own read lands. This also
    // matches App::RobotLoop::cycle()'s own real schedule (robot_loop.cpp):
    // motorL_.requestSample() -> ... -> motorL_.tick() -> motorR_.
    // requestSample() -> ... -> motorR_.tick().
    motorLeft.requestSample();
    motorLeft.tick(nowUs);
    motorRight.requestSample();
    motorRight.tick(nowUs);
    otos.tick(nowUs);

    odom.integrate();

    trace.push_back(CycleSample{
        motorLeft.position(), motorRight.position(),
        motorLeft.velocity(), motorRight.velocity(),
        otos.pose().x, otos.pose().y, otos.pose().heading,
        odom.x(), odom.y(), odom.theta(),
    });

    nowUs += dtUs;
  }

  return trace;
}

// ===========================================================================
// 2. Pivot scenario: equal-and-opposite duty targets on the two wheels (a
//    turn-in-place). Heading is asserted ENTIRELY through App::Odometry's
//    own integrate()/theta() -- never read from OtosPlant or WheelPlant
//    directly -- the sprint's own "B3 doesn't reappear" re-verification
//    (architecture-update.md Decision 3).
// ===========================================================================

void scenarioPivotHeadingSaneViaOdometry() {
  beginScenario("pivot: differential duty produces a sane heading, read entirely through Odometry");

  const float dutyMag = 0.25f;
  const int kCycles = 50;   // 1.0s of virtual time

  std::vector<CycleSample> trace = runScenario(/*dutyLeft=*/-dutyMag, /*dutyRight=*/dutyMag, kCycles);
  const CycleSample& last = trace.back();

  // BodyKinematics::forward(): omega = (vR - vL) / b. vR > 0, vL < 0 here,
  // so omega > 0 -- a positive (CCW) turn.
  checkTrue(last.odomTheta > 0.3f,
            "Odometry::theta() is a significant, positive (CCW) rotation after the pivot run");
  checkTrue(last.odomTheta < 3.0f,
            "Odometry::theta() stays well under a half-turn -- a plausible, non-runaway result "
            "(this ticket does not exercise the +/-pi register-wrap boundary; that is a later, "
            "system-level scenario's job)");

  // Equal-and-opposite wheel duties are an exact pivot: each cycle's
  // BodyKinematics::forward() distance term is (vR + vL)/2 == 0 exactly
  // (vL == -vR by construction), so Odometry's x_/y_ never accumulate any
  // translation.
  checkFloatEq(last.odomX, 0.0f, "pivot: Odometry::x() shows no translation", 1e-2f);
  checkFloatEq(last.odomY, 0.0f, "pivot: Odometry::y() shows no translation", 1e-2f);

  // Sanity cross-check (NOT the primary assertion -- Decision 3's own
  // "will always agree closely, by design" consequence): the plant's own
  // OTOS pose derives from the SAME two wheel positions via the SAME
  // BodyKinematics::forward() call, so it should land close to Odometry's
  // independently-integrated heading.
  checkFloatEq(last.otosHeading, last.odomTheta,
               "OtosPlant's simulated heading tracks Odometry's own heading closely (same wheel positions, same kinematics)",
               0.05f);
}

// ===========================================================================
// 3. Determinism: two independent runs of the SAME command script (same
//    duty targets, same cycle count -- and implicitly the same "seed", since
//    this plant has no RNG anywhere, see wheel_plant.h's file header)
//    produce bit-identical trajectories across every recorded field, every
//    cycle. Reuses the pivot scenario's own script (both motors + OTOS +
//    Odometry) so this proves determinism of "the WHOLE plant", not one
//    leaf.
// ===========================================================================

void scenarioDeterminismAcrossTwoRuns() {
  beginScenario("determinism: two runs of the same command script produce bit-identical trajectories");

  const float dutyMag = 0.25f;
  const int kCycles = 50;

  std::vector<CycleSample> runA = runScenario(-dutyMag, dutyMag, kCycles);
  std::vector<CycleSample> runB = runScenario(-dutyMag, dutyMag, kCycles);

  checkTrue(runA.size() == runB.size(), "both runs recorded the same number of cycles");

  bool identical = (runA.size() == runB.size());
  for (size_t i = 0; identical && i < runA.size(); ++i) {
    const CycleSample& a = runA[i];
    const CycleSample& b = runB[i];
    if (a.posLeft != b.posLeft || a.posRight != b.posRight ||
        a.velLeft != b.velLeft || a.velRight != b.velRight ||
        a.otosX != b.otosX || a.otosY != b.otosY || a.otosHeading != b.otosHeading ||
        a.odomX != b.odomX || a.odomY != b.odomY || a.odomTheta != b.odomTheta) {
      identical = false;
      char buf[256];
      std::snprintf(buf, sizeof(buf), "cycle %zu diverged between the two runs", i);
      fail(buf);
      break;
    }
  }
  checkTrue(identical, "every recorded field, every cycle, is bit-identical across both runs");
}

// ===========================================================================
// 4. Mount-orientation correction (114-007, sprint.md Revision 2 Decision 7):
//    a mirror-mounted motor pair's real fwd_sign (+1 left / -1 right,
//    tovez_nocal.json, issue 088-002) means firmware's own straight-forward
//    write drives the two wheel shafts in OPPOSITE physical (wire-frame)
//    directions -- exactly what a real mirrored pair's shafts do. This
//    scenario drives the wire-level Nezha 0x60 frame DIRECTLY via
//    SimPlant::write() (bypassing Devices::NezhaMotor entirely, so this is a
//    test of SimPlant alone, independent of firmware's own fwdSign encode
//    logic) with the SAME duty magnitude, OPPOSITE wire-level sign on the
//    two ports -- exactly what handleMotorWrite() (sim_plant.cpp) decodes
//    off a real firmware write for a straight command under this fwd_sign
//    profile (nezha_motor.cpp's own writeRawDuty() "Apply fwdSign" comment:
//    effective = fwdSign * written). Without SimPlant::setFwdSign()
//    correcting the OtosPlant-feeding boundary, the plant naively combines
//    the two raw wheel positions and reports a SPIN (nonzero heading, ~zero
//    x); with it, ground truth TRANSLATES (nonzero x, ~zero heading),
//    matching what the real robot does.
// ===========================================================================

// Writes one wire-level Nezha 0x60 RUN frame directly to `bus` -- the exact
// 8-byte layout nezha_motor.cpp's writeMotorRun() sends
// ([0xFF, 0xF9, motorId, direction, 0x60, speed, 0xF5, 0x00]) -- bypassing
// Devices::NezhaMotor so this scenario exercises SimPlant's own wire
// protocol handling directly, per this ticket's Approach step 6.
void writeStraightDutyFrame(TestSim::SimPlant& bus, uint8_t port, uint8_t dir, uint8_t speedPct) {
  const uint16_t motorWireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr) << 1;
  uint8_t frame[8] = {0xFF, 0xF9, port, dir, 0x60, speedPct, 0xF5, 0x00};
  int status = bus.write(motorWireAddr, frame, 8);
  checkTrue(status == 0, "wire-level motor-run write ACKed");
}

void scenarioMountOrientationCorrectionTranslatesNotSpins() {
  beginScenario("mount orientation: mirrored fwd_sign straight command translates ground truth, not spins it");

  constexpr uint8_t kDirCw = 1;    // matches sim_plant.cpp's own kNezhaDirCw
  constexpr uint8_t kDirCcw = 2;   // matches sim_plant.cpp's own kNezhaDirCcw
  constexpr uint8_t kSpeedPct = 60;

  TestSim::SimPlant bus(kTrackWidth);
  bus.setFwdSign(1, 1);    // left: mount-neutral
  bus.setFwdSign(2, -1);   // right: mirror-mounted (tovez_nocal.json's real 088-002 fwd_sign)

  // A real firmware straight command under this fwd_sign profile writes CW
  // to the left port (effective = +1 * written) and CCW to the right port
  // (effective = -1 * written) for the SAME logical-forward duty -- same
  // magnitude, opposite wire-level sign.
  writeStraightDutyFrame(bus, /*port=*/1, kDirCw, kSpeedPct);
  writeStraightDutyFrame(bus, /*port=*/2, kDirCcw, kSpeedPct);

  const float dtS = 0.02f;   // [s]
  const int kCycles = 100;   // 2.0s of virtual time -- >>10*tau, fully converges
  for (int i = 0; i < kCycles; ++i) {
    bus.tick(dtS);
  }

  const TestSim::OtosPlant& otos = bus.otosPlant();
  checkTrue(otos.x() > 400.0f,
            "ground truth shows meaningful forward TRANSLATION (nonzero x()), not a spin");
  checkFloatEq(otos.heading(), 0.0f,
               "ground truth heading stays near zero -- a straight command must not turn the robot",
               0.02f);
  checkFloatEq(otos.y(), 0.0f, "no lateral drift for a straight command", 1e-2f);
}

}  // namespace

int main() {
  scenarioVelocityStepShowsRampWithTauInRange();
  scenarioPivotHeadingSaneViaOdometry();
  scenarioDeterminismAcrossTwoRuns();
  scenarioMountOrientationCorrectionTranslatesNotSpins();

  if (g_failureCount == 0) {
    std::printf("OK: all plant scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the plant scenarios\n", g_failureCount);
  return 1;
}
