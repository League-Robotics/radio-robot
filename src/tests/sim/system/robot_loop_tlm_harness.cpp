// robot_loop_tlm_harness.cpp -- off-hardware, end-to-end acceptance harness
// for the TLM wire-command surface driven through the REAL App::RobotLoop
// (125-006, telemetry-emit-policy-rebuild-spec.md Part 8, sim criteria
// #8/#9/#10).
//
// Criterion #2 (the line/color-alternation regression) is deliberately NOT
// re-proven here at the RobotLoop/SimPlant level: sim_plant.cpp's own
// defaultWrite()/defaultRead() say plainly that "Color/line sensors ...
// are never simulated devices (no plant models them); every transaction to
// one of these wire addresses NAKs, matching the real bus's own behavior
// for an absent/uninitialized device." Confirmed empirically while writing
// this file: a SimHarness's line_/color_ leaves never leave
// present()==false under any boot/step sequence, so
// state().perception.lineFresh/colorFresh never toggle at all in this
// environment -- driving that scenario here would silently test "an
// unconnected robot has no fresh sensor bits" (true, uninteresting, and
// already covered by every OTHER parked-robot scenario in this codebase),
// not the regression ("a robot WITH connected line/color sensors"). Adding
// real color/line simulation to SimPlant is a standalone SimPlant feature,
// out of this telemetry ticket's scope. The regression's real proof is
// app_telemetry_harness.cpp's own scenarioParkedRobotWithAlternatingFresh
// LineColorStaysSilent(), which drives Telemetry::update() with the EXACT
// per-cycle alternation robot_loop.cpp's publishLineColor(tickedLine)
// produces (verified against that function's own source) -- Telemetry is
// the class that held the defect, so a Telemetry-level proof driven by the
// real production alternation pattern is a complete, honest proof of the
// fix; a RobotLoop/SimPlant-level rerun would need real (not NAK'd) sensor
// physics to add anything, which does not exist in this sim today.
//
// Origin (125-006's own KNOWN OBSTACLE): 125-003 originally wrote these
// scenarios directly into src/tests/sim/unit/app_robot_loop_harness.cpp,
// but that harness was ALREADY broken before sprint 125 even started --
// an unrelated, independent motion-library rework ("Planner integration",
// dated 2026-07-26 per src/sim/sim_harness.h's own comments) changed
// App::RobotLoop's constructor from 15 to 16 arguments (adding
// App::Configurator, and replacing the Motion::MoveQueue& slot with
// Motion::Planner&), and removed App::Drive::gainsLeft()/gainsRight().
// app_robot_loop_harness.cpp hand-builds its own RobotLoop graph inline in
// ~20 places and was never updated for that change -- it is currently
// -DHOST_BUILD compile-broken (~28 errors) and quarantined behind
// `@pytest.mark.xfail(strict=False)` in test_app_robot_loop.py, for a
// REASON UNRELATED to telemetry (a stale reorder-experiment note that
// predates the Planner rework too). Fully repairing that 2785-line file --
// ~20 RobotLoop construction call sites plus every Drive-gains assertion --
// is a substantial, unrelated motion-library-alignment undertaking, not
// telemetry-emit-policy work; 125-006 explicitly permits relocating
// scenarios that cannot execute rather than leaving acceptance resting on
// dead code.
//
// This file re-homes 125-003's 4 TLM scenarios onto TestSim::SimHarness
// (src/sim/sim_harness.h) instead -- the SAME composition root
// src/tests/sim/system/sim_api_harness.cpp
// already uses successfully (confirmed compiling and passing today), which
// IS current on the Planner/Configurator constructor shape because it is
// main.cpp's own mirror, kept in lockstep with robot_loop.h by construction
// (see sim_harness.h's file header). Testing through SimHarness's public
// wire-level surface (injectCommand()/drainReliable()) rather than a
// direct Telemetry& accessor is also a STRICTER end-to-end proof than the
// original scenarios attempted: it proves the wire line really reaches
// Telemetry::setMode() through RobotLoop::cycle()'s consume point and back
// out again as an observable STATUS/HELP reply, with no test-only escape
// hatch into internals.
//
// Hand-rolled assertions, PASS/FAIL per scenario, nonzero exit on any
// failure -- mirrors every other src/tests/sim/{unit,system} harness (see
// sim_api_harness.cpp).
#include <cstdio>
#include <string>
#include <vector>

#include "bench_test_config.h"
#include "sim_harness.h"
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

using TestSupport::DecodedKind;
using TestSupport::DecodedLine;

std::vector<DecodedLine> onlyTelemetry(const std::vector<DecodedLine>& lines) {
  std::vector<DecodedLine> out;
  for (const auto& l : lines) {
    if (l.kind == DecodedKind::kTelemetry) out.push_back(l);
  }
  return out;
}

// ===========================================================================
// TLM: command surface (originally 125-003, relocated 125-006 -- see this
// file's own header). End-to-end proof that a wire `TLM:` line reaches
// Telemetry::setMode() through RobotLoop::cycle()'s own consume point, and
// that the STATUS/HELP reply RobotLoop triggers actually appears on the
// wire. app_comms_harness.cpp already proves the Comms-level parse/reply
// halves in isolation (Comms holds no Telemetry&); only RobotLoop can prove
// the FULL wire-to-Telemetry path, since it is the one collaborator that
// holds both.
// ===========================================================================

void scenarioTlmOnStartsStreamingOnAParkedRobotAndStatusReportsIt() {
  beginScenario("issue Part 8 #8: \"TLM:ON\" over the wire sets Telemetry's mode to kOn and streams frames "
                "while parked, reported via STATUS");

  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  sim.step(3);
  std::vector<DecodedLine> idle = onlyTelemetry(sim.drainTelemetry());
  checkTrue(idle.empty(), "setup: parked + kAuto -> zero unsolicited frames before TLM:ON");

  sim.injectCommand("TLM:ON");
  sim.step(1);

  std::vector<std::string> raw = sim.drainReliable();
  bool sawStatusOn = false;
  for (const std::string& line : raw) {
    if (line.rfind("STATUS:", 0) == 0 && line.find("tlm=on") != std::string::npos) sawStatusOn = true;
  }
  checkTrue(sawStatusOn, "TLM:ON's reply is a STATUS line with tlm=on");

  (void)sim.drainTelemetry();  // discard the ON command's own first cadence tick
  sim.step(5);
  std::vector<DecodedLine> more = onlyTelemetry(sim.drainTelemetry());
  checkTrue(!more.empty(), "TLM:ON: a parked robot streams unsolicited frames at cadence");
}

void scenarioTlmOffThenAutoRestoresDefaultBehavior() {
  beginScenario("issue Part 8 #8: \"TLM:OFF\" stops the stream, \"TLM:AUTO\" restores the default mode's "
                "behavior");

  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  sim.step(3);
  (void)sim.drainTelemetry();
  (void)sim.drainReliable();

  sim.injectCommand("TLM:ON");
  sim.step(5);
  std::vector<DecodedLine> onFrames = onlyTelemetry(sim.drainTelemetry());
  checkTrue(!onFrames.empty(), "setup: streaming while kOn");
  (void)sim.drainReliable();

  sim.injectCommand("TLM:OFF");
  sim.step(1);
  bool sawStatusOff = false;
  for (const std::string& line : sim.drainReliable()) {
    if (line.rfind("STATUS:", 0) == 0 && line.find("tlm=off") != std::string::npos) sawStatusOff = true;
  }
  checkTrue(sawStatusOff, "TLM:OFF's reply is a STATUS line with tlm=off");

  sim.step(5);
  std::vector<DecodedLine> offFrames = onlyTelemetry(sim.drainTelemetry());
  checkTrue(offFrames.empty(), "kOff: no further unsolicited frames on a parked robot");

  sim.injectCommand("TLM:AUTO");
  sim.step(1);
  bool sawStatusAuto = false;
  for (const std::string& line : sim.drainReliable()) {
    if (line.rfind("STATUS:", 0) == 0 && line.find("tlm=auto") != std::string::npos) sawStatusAuto = true;
  }
  checkTrue(sawStatusAuto, "TLM:AUTO's reply is a STATUS line with tlm=auto");

  sim.step(5);
  std::vector<DecodedLine> autoFrames = onlyTelemetry(sim.drainTelemetry());
  checkTrue(autoFrames.empty(), "kAuto restored: still silent while parked");
}

void scenarioTlmGarbageArgumentChangesNothingAndRepliesHelp() {
  beginScenario("issue Part 8 #9: \"TLM:<garbage>\" changes no mode and replies with the HELP line");

  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  sim.step(3);
  (void)sim.drainTelemetry();
  (void)sim.drainReliable();

  sim.injectCommand("TLM:BOGUS");
  sim.step(1);
  bool sawHelp = false;
  for (const std::string& line : sim.drainReliable()) {
    if (line.rfind("HELP:", 0) == 0 && line.find("TLM[:NOW|ON|AUTO|OFF]") != std::string::npos) sawHelp = true;
  }
  checkTrue(sawHelp, "TLM:BOGUS's reply is the HELP line, listing TLM's argument grammar");

  // Mode unchanged (still kAuto/silent-while-parked) -- confirmed via
  // STATUS, the same black-box observable every other scenario here uses.
  sim.injectCommand("STATUS");
  sim.step(1);
  bool sawStatusAuto = false;
  for (const std::string& line : sim.drainReliable()) {
    if (line.rfind("STATUS:", 0) == 0 && line.find("tlm=auto") != std::string::npos) sawStatusAuto = true;
  }
  checkTrue(sawStatusAuto, "TLM:BOGUS changed no mode -- STATUS still reports tlm=auto");
}

void scenarioTlmModeNeverPersistsAcrossAFreshBoot() {
  beginScenario("issue Part 8 #10: TLM:ON never persists -- a fresh SimHarness (simulated reboot) always "
                "starts kAuto");

  TestSim::SimHarness first;
  TestSupport::configureSimForBenchTest(first);
  first.boot();
  first.step(1);
  (void)first.drainReliable();
  first.injectCommand("TLM:ON");
  first.step(1);
  bool sawStatusOn = false;
  for (const std::string& line : first.drainReliable()) {
    if (line.rfind("STATUS:", 0) == 0 && line.find("tlm=on") != std::string::npos) sawStatusOn = true;
  }
  checkTrue(sawStatusOn, "setup: TLM:ON took effect on the first instance");

  // A genuinely SEPARATE SimHarness -- a fresh construction, exactly like a
  // real power cycle: nothing is written to config, nothing carried over
  // (no shared static/global state exists to carry it).
  TestSim::SimHarness rebooted;
  TestSupport::configureSimForBenchTest(rebooted);
  rebooted.boot();
  rebooted.step(1);
  (void)rebooted.drainReliable();
  rebooted.injectCommand("STATUS");
  rebooted.step(1);
  bool sawStatusAuto = false;
  for (const std::string& line : rebooted.drainReliable()) {
    if (line.rfind("STATUS:", 0) == 0 && line.find("tlm=auto") != std::string::npos) sawStatusAuto = true;
  }
  checkTrue(sawStatusAuto,
            "a fresh boot always starts kAuto, regardless of any prior session's TLM:ON/OFF");
}

}  // namespace

int main() {
  scenarioTlmOnStartsStreamingOnAParkedRobotAndStatusReportsIt();
  scenarioTlmOffThenAutoRestoresDefaultBehavior();
  scenarioTlmGarbageArgumentChangesNothingAndRepliesHelp();
  scenarioTlmModeNeverPersistsAcrossAFreshBoot();

  if (g_failureCount == 0) {
    std::printf("OK: all robot_loop TLM scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the robot_loop TLM scenarios\n", g_failureCount);
  return 1;
}
