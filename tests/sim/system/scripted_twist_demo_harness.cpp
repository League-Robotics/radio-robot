// scripted_twist_demo_harness.cpp -- this sprint's own Definition of Done
// (105-006, SUC-023): a headless, readable, end-to-end "run one command and
// see the sim loop move" narrative built ENTIRELY on already-shipped
// primitives -- TestSim::SimApi (105-004) and its plant (105-003) -- boot,
// twist forward, watch the plant's real first-order velocity ramp, stop,
// watch velocity reverse its ramp and head back toward zero.
//
// Hand-rolled assertions, PASS/FAIL per phase, nonzero exit on any failure,
// PLUS a human-readable cycle-by-cycle trace printed to stdout -- mirrors
// sim_api_harness.cpp's/fault_knobs_harness.cpp's own shape exactly (same
// SimApi/DecodedLine plumbing), with the trace printing added on top as
// this ticket's own "stakeholder-visible proof" requirement. Run by
// test_scripted_twist_demo.py, which compiles this file together with
// sim_api.cpp, wire_test_codec.cpp, the plant sources, and the same full
// HOST_BUILD Devices/App/messages/kinematics dependency graph
// test_sim_api.py already compiles.
//
// --- Why the post-STOP window is exactly 3 cycles (a verified, not
// arbitrary, bound) ---
// SimApi's own scripted-I2CBus-FIFO contract (sim_api.cpp's
// scriptCycleBusResponses(), and sim_api.h's "Plant/PID tuning" section)
// pre-provisions EXACTLY ONE extra ("mode-activation" or "fresh command")
// duty write per injected command, at a single hand-derived cycle index --
// correct because every EXISTING scenario in this codebase (ticket 004's
// own ramp/stop/deadman scenarios, ticket 005's fault-knob scenarios) keeps
// the commanded |v_x| far enough above the plant's achievable ceiling
// (TestSim::kDefaultDutyVelMax) that the velocity-PID output stays
// saturated at +-1.0 duty FOREVER once set -- so "one write, then never
// again" is always true for THEM. STOP is different: it commands a
// velocity TARGET of exactly 0 while the plant is still doing ~500mm/s, so
// the PID's saturated output eventually (after roughly one time constant,
// ~0.13s) crosses back into its UNSATURATED region as the error shrinks --
// and once unsaturated, NezhaMotor's write-on-change gate (nezha_motor.cpp)
// legitimately issues SEVERAL more duty writes as the quantized percent
// counts down toward 0, none of which SimApi's single-transition script
// provisions for.
//
// This was not theorized -- it was verified empirically with a throwaway
// probe harness before this file was written: stepping the REAL RobotLoop
// past cycle pendingEventCycle_+4 after an injectStop() desyncs the shared
// I2CBus script FIFO (an unscripted write consumes an entry meant for a
// DIFFERENT device's encoder request), producing directly OBSERVABLE
// corruption in decoded telemetry -- connRight flipping false, velLeft
// freezing at a wrong value, and a FALSE kFaultWedgeLatch trip a few cycles
// later. Cycles pendingEventCycle_+1 through +4 (R's write, L's lagged
// write, two clean decay cycles) are provably clean; +5 is where the first
// unscripted write lands. This harness therefore observes STOP for exactly
// 4 cycles -- one more than `sim_api_harness.cpp`'s own
// scenarioStopAcksAndClearsActive() (its own step(3) after injectStop(),
// which never asserts on velocity at all and so never needed to find this
// exact boundary) -- and asserts the STRONGEST TRUE claim that window
// supports: velocity has unambiguously reversed the ramp's own trend and
// dropped well below its peak (empirically ~500 -> ~230, a >50% drop, in 4
// cycles), not that it has reached exactly zero. A literal
// "settles to exactly zero" assertion would require SimApi to script a
// VARIABLE number of post-transition duty writes (a real capability gap,
// not a scenario bug) -- out of this integration ticket's own stated scope
// ("consumes sim_api and fault knobs as ALREADY-BUILT primitives", ticket
// 105-006's own Implementation Plan) to add. See this ticket's completion
// notes for the same finding, and
// clasi/issues/sim-api-multi-write-decay-window.md for the deferred
// follow-up.
//
// --- Why "fault bits stay quiet" is phrased as "no NEW fault types" ---
// kFaultI2CSafetyNet (bit 0, telemetry.h) trips once during SimApi's own
// boot sequence (bus_.clearanceSafetyNetCount() ticks up from a timing
// artifact of the harness's own clock-jump boot priming, sim_api.cpp's
// driveBootToDone()) and, because Telemetry re-evaluates `count() > 0`
// every emit() (never resets the underlying counter mid-scenario), stays
// SET for the rest of any SimApi run -- verified with the same probe
// harness above, present from this demo's very first decoded frame, well
// before any twist is ever injected. This is a known, pre-existing
// artifact of ticket 004's own boot sequence, not something this demo's
// own actions provoke -- so this demo asserts the narratively-honest
// claim: no OTHER fault bit (kFaultWedgeLatch/kFaultI2CNak/
// kFaultCommsMalformed) ever sets, i.e. nothing THIS demo does introduces
// a new fault, rather than the false claim that fault_bits stays 0
// throughout.
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

void checkFloatGe(float actual, float bound, const std::string& what) {
  if (!(actual >= bound)) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected >= %g, got %g", what.c_str(),
                  static_cast<double>(bound), static_cast<double>(actual));
    fail(buf);
  }
}

void checkFloatLe(float actual, float bound, const std::string& what) {
  if (!(actual <= bound)) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected <= %g, got %g", what.c_str(),
                  static_cast<double>(bound), static_cast<double>(actual));
    fail(buf);
  }
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

bool anyAckMatches(const std::vector<DecodedLine>& frames, uint32_t corrId, msg::AckStatus status) {
  for (const auto& f : frames) {
    for (uint8_t i = 0; i < f.telemetry.acks_count; ++i) {
      const msg::AckEntry& e = f.telemetry.acks_[i];
      if (e.corr_id == corrId && e.status == status) return true;
    }
  }
  return false;
}

// bit 0 (kFaultI2CSafetyNet) is a known, pre-existing SimApi boot artifact
// -- see this file's own header comment. Every OTHER declared fault bit is
// something this demo's own actions should never provoke.
constexpr uint32_t kWatchedFaultMask =
    App::kFaultWedgeLatch | App::kFaultI2CNak | App::kFaultCommsMalformed;

void printTraceHeader() {
  std::printf("%6s  %8s %8s  %8s %8s  %8s %8s\n", "cycle", "cmd_v_x", "cmd_om", "encL", "encR", "velL", "velR");
}

void printTraceRow(int cycle, float cmdVx, float cmdOmega, const msg::Telemetry& t) {
  std::printf("%6d  %8.1f %8.1f  %8.1f %8.1f  %8.1f %8.1f\n", cycle, static_cast<double>(cmdVx),
              static_cast<double>(cmdOmega), static_cast<double>(t.enc_left), static_cast<double>(t.enc_right),
              static_cast<double>(t.vel_left), static_cast<double>(t.vel_right));
}

}  // namespace

int main() {
  std::printf("=== Scripted-Twist Demo (105-006, SUC-023) ===\n");
  std::printf("A readable, headless, off-hardware proof: boot -> twist forward -> the REAL plant's\n");
  std::printf("first-order velocity ramp -> stop -> velocity reverses the ramp and heads back to zero.\n\n");

  TestSim::SimApi sim;
  bool anyWatchedFaultEver = false;
  bool connHealthyThroughout = true;

  // ===========================================================================
  // Phase 1: BOOT -- drives the REAL App::RobotLoop::boot(), motors + OTOS
  // connect, kEventBootReady becomes visible in decoded telemetry.
  // ===========================================================================
  beginScenario("boot: motors + OTOS connect, kEventBootReady observed");

  sim.step(1);  // boot phase -- see SimApi::step()'s own doc comment
  checkTrue(sim.booted(), "booted() true after the first step() call");
  checkTrue(sim.motorLeft().connected(), "left motor connected after boot");
  checkTrue(sim.motorRight().connected(), "right motor connected after boot");

  sim.step(3);  // settle: emits kEventBootReady + both leaves' own activation writes land
  {
    std::vector<DecodedLine> bootFrames = onlyTelemetry(sim.drainTelemetry());
    checkTrue(!bootFrames.empty(), "telemetry decoded during boot settle");
    bool sawBootReady = false;
    for (const auto& f : bootFrames) {
      if (f.telemetry.event_bits & App::kEventBootReady) sawBootReady = true;
      if (f.telemetry.fault_bits & kWatchedFaultMask) anyWatchedFaultEver = true;
      if (!f.telemetry.conn_left || !f.telemetry.conn_right) connHealthyThroughout = false;
    }
    checkTrue(sawBootReady, "kEventBootReady observed in decoded telemetry");
    std::printf("  BOOT OK: motors + OTOS connected, kEventBootReady observed\n\n");
  }

  // ===========================================================================
  // Phase 2: TWIST FORWARD -- an injected twist drives the REAL plant's
  // duty->velocity->position first-order response, visible cycle by cycle
  // in decoded telemetry. v_x is chosen far above the plant's own
  // achievable ceiling (TestSim::kDefaultDutyVelMax) so the PID saturates
  // immediately and stays saturated for the whole ramp -- see sim_api.h's
  // own "Plant/PID tuning" section for the full derivation.
  // ===========================================================================
  constexpr float kCmdVx = 1000.0f;      // [mm/s] -- commanded; the plant's own ceiling is 500mm/s
  constexpr float kCmdOmega = 0.0f;      // [rad/s]
  constexpr uint32_t kTwistCorrId = 1;
  constexpr int kRampCycles = 20;        // ~1s of virtual ramp time -- comfortably >> kDefaultTau (130ms)

  beginScenario("twist: injected command drives the REAL plant's velocity ramp");
  std::printf("  TWIST commanded: v_x=%.1f mm/s omega=%.1f rad/s (corrId=%u)\n", static_cast<double>(kCmdVx),
              static_cast<double>(kCmdOmega), kTwistCorrId);
  sim.injectTwist(kCmdVx, kCmdOmega, /*duration=*/100000.0f, kTwistCorrId);

  printTraceHeader();
  bool twistAcked = false;
  bool sawRampData = false;
  float firstVelLeft = 0.0f, peakVelLeft = 0.0f, peakVelRight = 0.0f;
  float firstEncLeft = 0.0f, lastEncLeft = 0.0f;
  for (int i = 0; i < kRampCycles; ++i) {
    sim.step(1);
    std::vector<DecodedLine> frames = onlyTelemetry(sim.drainTelemetry());
    if (anyAckMatches(frames, kTwistCorrId, msg::AckStatus::ACK_STATUS_OK)) twistAcked = true;
    for (const auto& f : frames) {
      if (f.telemetry.fault_bits & kWatchedFaultMask) anyWatchedFaultEver = true;
      if (!f.telemetry.conn_left || !f.telemetry.conn_right) connHealthyThroughout = false;
      if (!f.telemetry.has_vel || !f.telemetry.has_enc) continue;
      if (!sawRampData) {
        firstVelLeft = f.telemetry.vel_left;
        firstEncLeft = f.telemetry.enc_left;
        sawRampData = true;
      }
      peakVelLeft = f.telemetry.vel_left;
      peakVelRight = f.telemetry.vel_right;
      lastEncLeft = f.telemetry.enc_left;
      printTraceRow(sim.cycleCount(), kCmdVx, kCmdOmega, f.telemetry);
    }
  }
  std::printf("\n");

  checkTrue(twistAcked, "the twist's corrId was acked OK");
  checkTrue(sawRampData, "at least one frame carried vel/enc data during the ramp");
  checkFloatGe(peakVelLeft, 300.0f, "velLeft ramped well above its starting value toward the plant's ceiling");
  checkFloatGe(peakVelRight, 300.0f, "velRight ramped well above its starting value toward the plant's ceiling");
  checkTrue(peakVelLeft > firstVelLeft, "velLeft increased over the ramp (the commanded direction)");
  checkTrue(lastEncLeft > firstEncLeft, "encLeft advanced over the ramp (real encoder movement)");
  std::printf("  RAMP OK: velLeft/velRight reached ~%.0f/%.0f mm/s (plant ceiling 500mm/s)\n\n",
              static_cast<double>(peakVelLeft), static_cast<double>(peakVelRight));

  // ===========================================================================
  // Phase 3: STOP -- an explicit STOP command acks OK, `active` clears, and
  // velocity unambiguously reverses the ramp's own trend, dropping well
  // below its peak within the harness's own verified-safe 3-cycle
  // post-transition window -- see this file's own header comment for the
  // derivation of why 3, not "until it reaches zero."
  // ===========================================================================
  constexpr uint32_t kStopCorrId = 2;
  constexpr int kStopCycles = 4;

  beginScenario("stop: STOP acks OK, active clears, velocity reverses the ramp and heads toward zero");
  std::printf("  STOP commanded (corrId=%u)\n", kStopCorrId);
  sim.injectStop(kStopCorrId);

  bool stopAcked = false;
  bool sawInactive = false;
  bool sawIdleMode = false;
  float lastVelLeft = peakVelLeft, lastVelRight = peakVelRight;
  for (int i = 0; i < kStopCycles; ++i) {
    sim.step(1);
    std::vector<DecodedLine> frames = onlyTelemetry(sim.drainTelemetry());
    if (anyAckMatches(frames, kStopCorrId, msg::AckStatus::ACK_STATUS_OK)) stopAcked = true;
    for (const auto& f : frames) {
      if (f.telemetry.fault_bits & kWatchedFaultMask) anyWatchedFaultEver = true;
      if (!f.telemetry.conn_left || !f.telemetry.conn_right) connHealthyThroughout = false;
      if (!f.telemetry.active) sawInactive = true;
      if (f.telemetry.mode == msg::DriveMode::IDLE) sawIdleMode = true;
      if (f.telemetry.has_vel) {
        lastVelLeft = f.telemetry.vel_left;
        lastVelRight = f.telemetry.vel_right;
      }
      printTraceRow(sim.cycleCount(), 0.0f, 0.0f, f.telemetry);
    }
  }
  std::printf("\n");

  checkTrue(stopAcked, "the stop's corrId was acked OK");
  checkTrue(sawInactive, "decoded telemetry shows active=false after STOP");
  checkTrue(sawIdleMode, "decoded telemetry shows mode=IDLE after STOP");
  checkFloatLe(lastVelLeft, 0.6f * peakVelLeft, "velLeft dropped well below its ramp peak within the safe window");
  checkFloatLe(lastVelRight, 0.6f * peakVelRight, "velRight dropped well below its ramp peak within the safe window");
  std::printf("  STOP OK: velLeft/velRight dropped from ~%.0f/%.0f to ~%.0f/%.0f mm/s in %d cycles"
              " (still descending toward zero -- see this file's header for why this demo stops observing here)\n\n",
              static_cast<double>(peakVelLeft), static_cast<double>(peakVelRight), static_cast<double>(lastVelLeft),
              static_cast<double>(lastVelRight), kStopCycles);

  // ===========================================================================
  // Phase 4: no NEW fault bit ever set, connLeft/connRight stayed healthy
  // throughout -- this demo injects no fault knob at all (105-005's own
  // scope), so the firmware's own health signals should stay clean save
  // for the pre-existing boot-time kFaultI2CSafetyNet artifact.
  // ===========================================================================
  beginScenario("health: no new fault bit set, conn_left/conn_right healthy throughout");
  checkTrue(!anyWatchedFaultEver,
            "no kFaultWedgeLatch/kFaultI2CNak/kFaultCommsMalformed ever set (no fault knob used in this demo)");
  checkTrue(connHealthyThroughout, "conn_left/conn_right stayed true across the whole run");
  std::printf("  HEALTH OK: no new fault bit set, connections healthy throughout\n\n");

  if (g_failureCount == 0) {
    std::printf("OK: scripted-twist demo complete\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the scripted-twist demo\n", g_failureCount);
  return 1;
}
