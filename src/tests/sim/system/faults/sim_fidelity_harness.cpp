// sim_fidelity_harness.cpp -- ticket 109-007 (sim-honors-otos-calibration.md):
// off-hardware acceptance proof that TestSim::SimPlant honors a firmware-
// pushed OTOS calibration scalar against an injected raw scale-error fault
// knob, and that the encoder error model's new tick-quantization/slip knobs
// behave as documented, alongside a zero-error exactness regression for
// both the WheelPlant and OtosPlant sides.
//
// Unlike fault_knobs_harness.cpp / sim_api_harness.cpp (which run the FULL
// TestSim::SimHarness -- the real App::RobotLoop against SimPlant), this
// harness exercises the REAL Devices::Otos leaf (src/firm/devices/otos.cpp)
// directly against a bare TestSim::SimPlant -- no ScriptedI2CHook, no
// RobotLoop -- so it is SimPlant's own defaultRead()/defaultWrite() fidelity
// under test, exactly the thing devices_otos_harness.cpp's ScriptedI2CHook-
// based scenarios do NOT exercise (that harness scripts arbitrary register
// payloads; this one drives the real wire protocol both directions). The
// encoder scenarios drive TestSim::WheelPlant directly -- no bus needed at
// all for those.
//
// Hand-rolled assertions, PASS/FAIL per scenario, nonzero exit on any
// failure -- mirrors every other tests/sim harness in this codebase. Run by
// test_sim_fidelity.py, which compiles this file together with
// otos.cpp/sim_plant.cpp/otos_plant.cpp/wheel_plant.cpp only -- a much
// lighter dependency graph than the full RobotLoop-based harnesses need,
// since Devices::Otos/TestSim::SimPlant/TestSim::WheelPlant/TestSim::OtosPlant
// have no messages/app dependency (the devices/ isolation invariant).
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "devices/device_config.h"
#include "devices/otos.h"
#include "sim_plant.h"
#include "wheel_plant.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors every other tests/sim harness) ---

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

void checkNear(float actual, float expected, float tol, const std::string& what) {
  if (std::fabs(static_cast<double>(actual - expected)) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g (tol %g)", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual),
                  static_cast<double>(tol));
    fail(buf);
  }
}

Devices::OtosConfig makeConfig(float linearScale, float angularScale) {
  Devices::OtosConfig cfg;
  cfg.offsetX = 0.0f;
  cfg.offsetY = 0.0f;
  cfg.offsetYaw = 0.0f;
  cfg.linearScale = linearScale;
  cfg.angularScale = angularScale;
  return cfg;
}

// scaleToRegister() -- the EXACT same conversion Devices::Otos::
// scaleToRegister() performs (otos.cpp, private) -- duplicated here per
// this codebase's established per-file fixture-duplication convention
// (devices_otos_harness.cpp's own kPosMmPerLsb/kHdgRadPerLsb precedent),
// used by this harness to compute the compensating register value a real
// OL/OA calibration push would send.
int8_t scaleToRegister(float scale) {
  float raw = std::round((scale - 1.0f) / 0.001f);
  if (raw > 127.0f) raw = 127.0f;
  if (raw < -127.0f) raw = -127.0f;
  return static_cast<int8_t>(raw);
}

// ===========================================================================
// 1. Zero-error exactness (109-007 AC): with every new knob left at its
//    default (off), WheelPlant::reportedPosition() must match position()
//    EXACTLY -- the sim gate's "with sim OTOS error/noise disabled, turns
//    are exact" requirement, at the plant level.
// ===========================================================================

void scenarioZeroErrorWheelEncoderExactness() {
  beginScenario("zero-error: WheelPlant::reportedPosition() matches truth exactly");

  TestSim::WheelPlant wheel(TestSim::kDefaultDutyVelMax, TestSim::kDefaultTau);
  wheel.step(/*appliedDuty=*/0.73f, /*dt=*/0.5f);

  float truth = wheel.position();
  float reported = wheel.reportedPosition();

  checkNear(reported, truth, 1e-6f,
            "no fault knob touched -- reportedPosition() must equal position() bit-for-bit");
}

// ===========================================================================
// 2. Zero-error exactness -- OTOS side: with rawScaleErr and the
//    calibration register both at their defaults (0), OtosPlant's
//    reportedX/Y/Heading() must equal x()/y()/heading() exactly.
// ===========================================================================

void scenarioZeroErrorOtosExactness() {
  beginScenario("zero-error: OtosPlant reportedX/Y/Heading() match truth exactly");

  TestSim::SimPlant plant;
  plant.setTruePose(1234.5f, -678.9f, 0.42f);

  const TestSim::OtosPlant& otos = plant.otosPlant();
  checkNear(otos.reportedX(), otos.x(), 1e-4f, "reportedX() == x() with no fault knob touched");
  checkNear(otos.reportedY(), otos.y(), 1e-4f, "reportedY() == y() with no fault knob touched");
  checkNear(otos.reportedHeading(), otos.heading(), 1e-6f,
            "reportedHeading() == heading() with no fault knob touched");
}

// ===========================================================================
// 3. SUC-005's second acceptance criterion, made concrete (AC #1/#2): a raw
//    OTOS linear scale error makes the REAL Devices::Otos leaf's decoded
//    pose diverge from truth; pushing the compensating calibration scalar
//    via the SAME OL wire path (Otos::setLinearScalar()) converges it back.
//    Runs through SimPlant's own defaultRead()/defaultWrite() -- no hook, no
//    ScriptedI2CHook -- so this is SimPlant's real wire-protocol fidelity
//    under test, not a scripted stand-in.
// ===========================================================================

void scenarioOtosRawScaleErrorDivergesThenCalibrationConverges() {
  beginScenario("OTOS raw scale error diverges pose; OtosConfigPatch calibration converges it");

  TestSim::SimPlant plant;
  // Neutral calibration baked at boot (scaleToRegister(1.0) == 0 -- an
  // un-calibrated chip) -- begin() pushes it via the REAL setLinearScalar()/
  // setAngularScalar() wire path, exercising handleOtosWrite()'s register
  // capture from the very first boot.
  Devices::Otos odom(plant, makeConfig(/*linearScale=*/1.0f, /*angularScale=*/1.0f));
  odom.begin();

  constexpr float kTrueX = 1000.0f;   // [mm]
  constexpr float kRawErrorLinear = 0.05f;   // 5% over-report -- a plausible mis-calibration
  plant.setTruePose(kTrueX, 0.0f, 0.0f);
  plant.setOtosRawScaleErr(kRawErrorLinear, 0.0f);

  uint64_t nowUs = 1000000;
  odom.tick(nowUs);

  float uncalibrated = odom.pose().x;
  checkNear(uncalibrated, kTrueX * (1.0f + kRawErrorLinear), 1.0f,
            "uncalibrated: SimPlant's OTOS burst-read response is truth*rawError");
  if (std::fabs(static_cast<double>(uncalibrated - kTrueX)) < 5.0) {
    fail("uncalibrated pose must have MEASURABLY diverged from truth (test would be vacuous otherwise)");
  }

  // Push the compensating calibration scalar -- the SAME OL wire verb / a
  // live OtosConfigPatch drives (RobotLoop::handleConfig -> Otos::
  // setLinearScalar()), computed exactly the way calibration_commands()
  // (push.py) derives it: scale = 1/(1+rawError), then scaleToRegister().
  float compensatingScale = 1.0f / (1.0f + kRawErrorLinear);
  odom.setLinearScalar(static_cast<float>(scaleToRegister(compensatingScale)));

  nowUs += 20000;  // kReadPeriod -- otos.h's own rate-limit window
  odom.tick(nowUs);

  float calibrated = odom.pose().x;
  // Tolerance: the register is a quantized 0.1%-per-LSB int8, so the
  // correction is not bit-exact -- within ~1% of true (well inside a single
  // register LSB's worth of residual error) proves genuine convergence.
  checkNear(calibrated, kTrueX, kTrueX * 0.01f,
            "calibrated: pushing the compensating OL scalar converges pose back to truth");
}

// ===========================================================================
// 4. Encoder tick quantization (109-007): reportedPosition() rounds to the
//    nearest multiple of the configured tick size.
// ===========================================================================

void scenarioEncoderTickQuantizationRounds() {
  beginScenario("encoder tick quantization: reportedPosition() rounds to the nearest tick");

  TestSim::WheelPlant wheel(TestSim::kDefaultDutyVelMax, TestSim::kDefaultTau);
  wheel.setTickQuantization(1.0f);  // [mm] -- coarse enough to see rounding clearly
  wheel.step(/*appliedDuty=*/0.5f, /*dt=*/0.777f);  // an arbitrary, non-tick-aligned duration

  float truth = wheel.position();
  float expected = std::round(truth / 1.0f) * 1.0f;
  float reported = wheel.reportedPosition();

  checkNear(reported, expected, 1e-4f, "reportedPosition() quantizes to the nearest 1mm tick");
  if (std::fabs(static_cast<double>(truth - expected)) < 1e-4) {
    fail("test setup must produce a truth position NOT already tick-aligned "
         "(otherwise quantization is untested)");
  }
}

// ===========================================================================
// 5. Encoder slip events (109-007): a deterministic accumulator injects a
//    PERMANENT signed offset every time it crosses 1.0 -- proven by driving
//    a rate that fires exactly every other call and checking the offset
//    accumulates, persists, and stacks across multiple firings.
// ===========================================================================

void scenarioEncoderSlipInjectsPermanentOffset() {
  beginScenario("encoder slip: a fired slip event injects a permanent offset that persists and stacks");

  TestSim::WheelPlant wheel(TestSim::kDefaultDutyVelMax, TestSim::kDefaultTau);
  // No motion -- isolates the slip offset from position()'s own drift so
  // reportedPosition() - position() is exactly the accumulated slip.
  constexpr float kSlipMagnitude = 2.0f;  // [mm] per fired event
  wheel.setSlip(/*rate=*/0.5f, kSlipMagnitude);  // fires on every 2nd call

  float truth = wheel.position();  // 0.0 -- no step() called

  float r1 = wheel.reportedPosition();  // accum 0.5 -- no fire yet
  checkNear(r1, truth, 1e-6f, "call 1: accumulator below 1.0 -- no slip offset yet");

  float r2 = wheel.reportedPosition();  // accum 1.0 -- fires once
  checkNear(r2, truth + kSlipMagnitude, 1e-4f, "call 2: accumulator crosses 1.0 -- one slip event fires");

  float r3 = wheel.reportedPosition();  // accum 1.5 -- offset persists, no second fire yet
  checkNear(r3, truth + kSlipMagnitude, 1e-4f, "call 3: the fired offset PERSISTS (does not decay)");

  float r4 = wheel.reportedPosition();  // accum 2.0 -- fires again, offset STACKS
  checkNear(r4, truth + 2.0f * kSlipMagnitude, 1e-4f, "call 4: a second slip event STACKS onto the first");
}

}  // namespace

int main() {
  scenarioZeroErrorWheelEncoderExactness();
  scenarioZeroErrorOtosExactness();
  scenarioOtosRawScaleErrorDivergesThenCalibrationConverges();
  scenarioEncoderTickQuantizationRounds();
  scenarioEncoderSlipInjectsPermanentOffset();

  if (g_failureCount == 0) {
    std::printf("OK: all sim-fidelity scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the sim-fidelity scenarios\n", g_failureCount);
  return 1;
}
