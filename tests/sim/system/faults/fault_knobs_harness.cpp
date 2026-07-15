// fault_knobs_harness.cpp -- off-hardware acceptance harness for ticket
// 105-005 (SUC-022), TestSim::WheelPlant's three fault-injection knobs
// (setDisconnected()/freezePosition()/setDropoutRate(), tests/sim/plant/
// wheel_plant.h) driven through the REAL TestSim::SimApi (105-004) and
// asserted against the FIRMWARE's own observable reaction in decoded
// telemetry -- exactly the retargeted issue's own ask
// (clasi/sprints/105-sim-rebuild-around-the-steppable-loop/issues/
// sim-hardware-fault-injection.md): "a thin steppable-loop sim over the
// devices layer's HOST_BUILD fakes, whose scripted I2CBus can natively fake
// NAKs, stale reads, and wedge latch-ups -- a better fault-injection seam
// than SimMotor ever was."
//
// Three independent scenarios, ONE knob active at a time on ONE plant (the
// other motor's plant, and the other two knobs, left at their default/
// inactive state) -- per this ticket's own "keep failure attribution
// unambiguous" testing plan.
//
// Hand-rolled assertions, PASS/FAIL per scenario, nonzero exit on any
// failure -- mirrors sim_api_harness.cpp's own shape exactly (same
// SimApi/DecodedLine plumbing). Run by test_fault_knobs.py, which compiles
// this file together with sim_api.cpp and the same full HOST_BUILD
// dependency graph test_sim_api.py already compiles.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "messages/envelope.h"
#include "messages/planner.h"
#include "sim_api.h"
#include "wire_test_codec.h"

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

// --- Small local helpers -----------------------------------------------

using TestSupport::DecodedKind;
using TestSupport::DecodedLine;

std::vector<DecodedLine> onlyTelemetry(const std::vector<DecodedLine>& lines) {
  std::vector<DecodedLine> out;
  for (const auto& l : lines) {
    if (l.kind == DecodedKind::kTelemetry) out.push_back(l);
  }
  return out;
}

// Same fiber-level wedge-latch threshold Devices::MotorArmor's own private
// kWedgeThreshold declares (source/devices/motor_armor.h) -- duplicated
// here by citation, this codebase's established per-file fixture-
// duplication convention (devices_motor_harness.cpp scenario 4's own
// identical precedent: "064-004 hardening -- do not reintroduce...").
constexpr int kWedgeThreshold = 10;

// ===========================================================================
// 1. Motor disconnect (AC #1): WheelPlant::setDisconnected(true) on the
//    LEFT plant NAKs every I2CBus transaction for that motor's wire address
//    while active. Right stays connected throughout -- proves the fault is
//    per-motor, not bus-wide. Held for fewer than kWedgeThreshold cycles so
//    this scenario's own signal (connLeft) is never contaminated by a
//    side-effect wedge latch (position necessarily holds at its last-good
//    value while disconnected -- collectEncoder()'s own "return
//    lastGoodRawEnc_" path, nezha_motor.cpp).
// ===========================================================================

void scenarioMotorDisconnectFlipsConnLeftAndRecovers() {
  beginScenario("motor disconnect: connLeft flips false while active, recovers once cleared");

  TestSim::SimApi sim;
  sim.step(1);  // boot
  sim.step(3);  // settle: both leaves' own one-time activation writes land
  (void)sim.drainTelemetry();

  sim.plantLeft().setDisconnected(true);
  sim.step(5);  // well under kWedgeThreshold -- isolates the connLeft signal

  std::vector<DecodedLine> disconnectedFrames = onlyTelemetry(sim.drainTelemetry());
  checkTrue(!disconnectedFrames.empty(), "telemetry decoded while disconnected");

  bool sawDisconnected = false;
  bool rightStayedConnectedThroughout = true;
  for (const auto& f : disconnectedFrames) {
    if (!f.telemetry.conn_left) sawDisconnected = true;
    if (!f.telemetry.conn_right) rightStayedConnectedThroughout = false;
  }
  checkTrue(sawDisconnected, "decoded telemetry shows conn_left=false while the knob is active");
  checkTrue(rightStayedConnectedThroughout,
            "conn_right stays true throughout -- the fault is per-motor, not bus-wide");

  sim.plantLeft().setDisconnected(false);
  sim.step(5);  // recovery window

  std::vector<DecodedLine> recoveredFrames = onlyTelemetry(sim.drainTelemetry());
  checkTrue(!recoveredFrames.empty(), "telemetry decoded after clearing the knob");

  bool sawRecovered = false;
  for (const auto& f : recoveredFrames) {
    if (f.telemetry.conn_left) sawRecovered = true;
  }
  checkTrue(sawRecovered, "decoded telemetry shows conn_left=true again once the knob is cleared "
                          "(connected() is recomputed fresh every collectEncoder() call, never latched)");
}

// ===========================================================================
// 2. Encoder wedge (AC #2): WheelPlant::freezePosition(true) on the LEFT
//    plant, WHILE driving (a twist keeps appliedDuty() nonzero throughout --
//    the "moving-but-stuck" flavor devices_motor_harness.cpp scenario 4(b)
//    already proves in isolation), freezes the REPORTED encoder value while
//    the plant's own internal velocity/position state keeps advancing
//    underneath (WheelPlant::step() runs every cycle in SimApi regardless
//    of the knob -- see wheel_plant.h/.cpp). Asserts kFaultWedgeLatch sets
//    in decoded telemetry within kWedgeThreshold cycles, and -- the set/
//    clear semantics robot_loop.cpp's own live
//    `tlm_.setFault(kFaultWedgeLatch, motorL_.wedged() || motorR_.wedged())`
//    call re-evaluates fresh every cycle, never a one-shot latch at the wire
//    level -- clears again once the knob is released and the reported
//    position resumes advancing.
// ===========================================================================

void scenarioEncoderWedgeSetsFaultBitAndClearsOnRelease() {
  beginScenario("encoder wedge: kFaultWedgeLatch sets while frozen, clears once released");

  TestSim::SimApi sim;
  sim.step(1);  // boot
  sim.step(3);  // settle
  (void)sim.drainTelemetry();

  sim.injectTwist(/*v_x=*/1000.0f, /*omega=*/0.0f, /*duration=*/100000.0f, /*corrId=*/11);
  sim.step(5);  // ramp a bit -- appliedDuty() is genuinely nonzero once frozen
  (void)sim.drainTelemetry();

  sim.plantLeft().freezePosition(true);
  sim.step(kWedgeThreshold + 5);  // comfortably past the threshold

  std::vector<DecodedLine> frozenFrames = onlyTelemetry(sim.drainTelemetry());
  checkTrue(!frozenFrames.empty(), "telemetry decoded while frozen");

  bool sawWedgeLatch = false;
  for (const auto& f : frozenFrames) {
    if (f.telemetry.fault_bits & App::kFaultWedgeLatch) sawWedgeLatch = true;
  }
  checkTrue(sawWedgeLatch, "kFaultWedgeLatch sets in decoded telemetry within the wedge threshold");

  sim.plantLeft().freezePosition(false);
  sim.step(kWedgeThreshold + 5);  // enough cycles for the changed reading to clear the latch

  std::vector<DecodedLine> releasedFrames = onlyTelemetry(sim.drainTelemetry());
  checkTrue(!releasedFrames.empty(), "telemetry decoded after release");

  bool sawClear = false;
  for (const auto& f : releasedFrames) {
    if (!(f.telemetry.fault_bits & App::kFaultWedgeLatch)) sawClear = true;
  }
  checkTrue(sawClear,
            "kFaultWedgeLatch clears again once the frozen reading resumes advancing "
            "(robot_loop.cpp's own live, never-sticky re-evaluation)");
}

// ===========================================================================
// 3. Encoder dropout (AC #3): WheelPlant::setDropoutRate() on the LEFT
//    plant holds a moderate fraction (25%) of scripted encoder reads at the
//    last value instead of a fresh one -- the exact stale-vs-fresh pattern
//    devices_motor_harness.cpp scenario 8 already proves NezhaMotor's own
//    freshness gate survives in isolation, now driven through the full
//    loop. Asserts telemetry stays sane throughout: no false wedge latch,
//    velocity never starved to ~0 despite the held reads.
// ===========================================================================

void scenarioEncoderDropoutStaysSaneUnderModerateLoss() {
  beginScenario("encoder dropout: telemetry stays sane under moderate (25%) sample loss");

  TestSim::SimApi sim;
  sim.step(1);  // boot
  sim.step(3);  // settle
  (void)sim.drainTelemetry();

  sim.injectTwist(/*v_x=*/1000.0f, /*omega=*/0.0f, /*duration=*/100000.0f, /*corrId=*/22);
  sim.step(15);  // ramp to steady state BEFORE dropout starts -- matches sim_api_harness.cpp's
                 // own scenarioTwistDrivesRealPlantRamp() timing (>=300mm/s within 15 cycles)
  (void)sim.drainTelemetry();

  sim.plantLeft().setDropoutRate(0.25f);
  sim.step(40);  // sustained run under dropout -- several dropout holds AND several fresh samples

  std::vector<DecodedLine> frames = onlyTelemetry(sim.drainTelemetry());
  checkTrue(!frames.empty(), "telemetry decoded under sustained dropout");

  bool sawWedgeLatch = false;
  bool sawStarvedVelocity = false;
  bool sawHealthyVelocity = false;
  for (const auto& f : frames) {
    if (f.telemetry.fault_bits & App::kFaultWedgeLatch) sawWedgeLatch = true;
    if (!f.telemetry.has_vel) continue;
    if (std::fabs(f.telemetry.vel_left) < 50.0f) sawStarvedVelocity = true;
    if (f.telemetry.vel_left > 300.0f) sawHealthyVelocity = true;
  }
  checkTrue(!sawWedgeLatch, "no false kFaultWedgeLatch across sustained moderate dropout");
  checkTrue(!sawStarvedVelocity, "velLeft never starved to ~0 by the held/stale reads");
  checkTrue(sawHealthyVelocity, "velLeft still reaches/holds a healthy value despite the dropout");
}

}  // namespace

int main() {
  scenarioMotorDisconnectFlipsConnLeftAndRecovers();
  scenarioEncoderWedgeSetsFaultBitAndClearsOnRelease();
  scenarioEncoderDropoutStaysSaneUnderModerateLoss();

  if (g_failureCount == 0) {
    std::printf("OK: all fault-knob scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the fault-knob scenarios\n", g_failureCount);
  return 1;
}
