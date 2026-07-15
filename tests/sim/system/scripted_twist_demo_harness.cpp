// scripted_twist_demo_harness.cpp -- 105-006's own Definition of Done
// (SUC-023): a headless, readable, end-to-end "run one command and see the
// sim loop move" narrative built ENTIRELY on already-shipped primitives --
// TestSim::SimApi (105-004) and its plant (105-003) -- boot, twist forward,
// watch the plant's real first-order velocity ramp, stop, watch velocity
// converge to (approximately) zero. The STOP phase's convergence assertion
// was strengthened 106-003 (SUC-026) once SimApi::DutyPredictor lifted the
// old ~4-cycle safe-observation bound -- see this file's own header comment
// below for the full derivation.
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
// --- Why the post-STOP window is 12 cycles, and asserts FULL convergence
// (106-003, SUC-026 -- supersedes the ~4-cycle bound this comment used to
// document) ---
// Ticket 105-006's original version of this file bounded the post-STOP
// observation window to 4 cycles: `SimApi::scriptCycleBusResponses()` used
// to pre-provision exactly ONE extra ("mode-activation" or "fresh command")
// duty write per injected command, at a single hand-derived cycle index --
// correct for every OTHER scenario in this codebase (their commanded
// |v_x| stays far enough above the plant's ceiling that the PID output
// stays saturated forever), but wrong for STOP, whose target (0) the plant
// can actually reach: once the PID's saturated output crosses back into its
// UNSATURATED region as the error shrinks, NezhaMotor's write-on-change
// gate (nezha_motor.cpp) legitimately issues SEVERAL more duty writes as
// the quantized percent counts down toward 0 -- more writes than the old
// single-transition script provisioned for, desyncing the shared I2CBus
// script FIFO a few cycles in (verified empirically at the time: connRight
// flipping false, velLeft freezing at a wrong value, a false
// kFaultWedgeLatch trip). See `clasi/issues/sim-api-multi-write-decay-
// window.md` for the original finding.
//
// 106-003 generalized the scripting mechanism (`SimApi::DutyPredictor`,
// sim_api.{h,cpp}) to predict, per leaf, per cycle, whether NezhaMotor's
// tick() will actually emit a duty write THIS cycle -- including replicating
// `Devices::MotorArmor::armoredWrite()`'s own reversal-dwell/output-deadband
// gate (motor_armor.h), which forces a same-cycle-or-later duty SIGN FLIP to
// zero for a 100ms dwell window before letting the new direction reach the
// bus -- discovered only by comparing this predictor's own per-cycle state
// against decoded telemetry, since a naive write-on-change-only replica
// (ignoring the dwell) mis-predicts by exactly one write at the STOP
// transition. That generalization is what makes a MULTI-CYCLE, ARBITRARY-
// LENGTH decay observable at all -- this demo's own STOP phase is the first
// consumer.
//
// 12 cycles (not 4, not "until true zero") is itself a verified, not
// arbitrary, bound: the plant's residual velocity keeps shrinking under
// closed-loop control until it drops below `Devices::MotorVelocityPid`'s own
// effective deadband-equivalent (~3mm/s here, this harness's own
// `kOutputDeadband` in `DutyPredictor` -- mirrors `MotorArmor`'s
// `kDefaultOutputDeadband`), at which point duty locks at EXACTLY 0 and the
// plant's own further decay (now open-loop, duty=0) becomes too slow per
// 50ms cycle to move the encoder's own tenths-of-mm quantization -- the
// SAME "boundary-latch" flavor `.clasi/knowledge/encoder-wedge-boundary-
// latch.md` documents for the real hardware. Left unobserved long enough,
// this scenario's own converged residual would eventually accumulate
// `Devices::MotorArmor`'s own `kWedgeThreshold` (10) consecutive identical
// raw reads and trip a REAL (not false) `kFaultWedgeLatch` -- verified by
// stepping this same scenario out to 60+ cycles during this ticket's own
// implementation. 12 cycles lands comfortably inside the converged-and-
// clean window (velocity already within ~2mm/s of zero by cycle 5-6 post-
// STOP, consecutive-identical-read count still single digits by cycle 12)
// while being 3x the OLD 4-cycle bound -- unambiguous proof the bound was
// lifted, not a coincidental pass.
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

int countSecondary(const std::vector<DecodedLine>& lines) {
  int n = 0;
  for (const auto& l : lines) {
    if (l.kind == DecodedKind::kSecondary) ++n;
  }
  return n;
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
  // 106-002 own "sim-assert both cadences" requirement (drive-by fix for
  // `secondary-telemetry-starved-by-106-001-cadence-retarget.md`): tallied
  // across the whole run below (boot settle + ramp + stop), alongside the
  // primary-frame trace this demo already prints -- proves secondary is NOT
  // stuck at 0 Hz under the real ~40ms/cycle schedule this sim drives
  // RobotLoop against (the exact regime that starved it pre-106-002).
  int secondaryFrameCount = 0;

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
    std::vector<DecodedLine> bootLines = sim.drainTelemetry();
    secondaryFrameCount += countSecondary(bootLines);
    std::vector<DecodedLine> bootFrames = onlyTelemetry(bootLines);
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
    std::vector<DecodedLine> rampLines = sim.drainTelemetry();
    secondaryFrameCount += countSecondary(rampLines);
    std::vector<DecodedLine> frames = onlyTelemetry(rampLines);
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
  // velocity converges to (approximately) zero within the harness's own
  // verified-safe 12-cycle post-transition window -- 3x the pre-106-003
  // 4-cycle bound, and a full-convergence assertion rather than a partial-
  // drop one now that DutyPredictor (sim_api.{h,cpp}) scripts the shared
  // I2CBus FIFO correctly for however many duty writes the decay actually
  // needs. See this file's own header comment for the verified derivation
  // of both the 12-cycle window and the ~3mm/s convergence bound.
  // ===========================================================================
  constexpr uint32_t kStopCorrId = 2;
  constexpr int kStopCycles = 12;
  constexpr float kConvergedVelocity = 5.0f;  // [mm/s] "approximately zero" -- see header comment

  beginScenario("stop: STOP acks OK, active clears, velocity converges to (approximately) zero");
  std::printf("  STOP commanded (corrId=%u)\n", kStopCorrId);
  sim.injectStop(kStopCorrId);

  bool stopAcked = false;
  bool sawInactive = false;
  bool sawIdleMode = false;
  float lastVelLeft = peakVelLeft, lastVelRight = peakVelRight;
  for (int i = 0; i < kStopCycles; ++i) {
    sim.step(1);
    std::vector<DecodedLine> stopLines = sim.drainTelemetry();
    secondaryFrameCount += countSecondary(stopLines);
    std::vector<DecodedLine> frames = onlyTelemetry(stopLines);
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
  // Full convergence, not a partial-drop bound (106-003 lifts the old
  // ~4-cycle ceiling on how far this demo can safely observe): the last
  // decoded velocity sample in the 12-cycle window must sit within
  // kConvergedVelocity of zero on BOTH sides (fabs, not just an upper
  // bound -- the closed loop can legitimately overshoot slightly negative
  // on its way to zero, see this file's header's own per-cycle trace).
  checkFloatLe(std::fabs(lastVelLeft), kConvergedVelocity, "velLeft converged to (approximately) zero within the window");
  checkFloatLe(std::fabs(lastVelRight), kConvergedVelocity, "velRight converged to (approximately) zero within the window");
  std::printf("  STOP OK: velLeft/velRight converged from ~%.0f/%.0f to ~%.1f/%.1f mm/s in %d cycles"
              " (within %.0fmm/s of zero -- full convergence, not a partial drop)\n\n",
              static_cast<double>(peakVelLeft), static_cast<double>(peakVelRight), static_cast<double>(lastVelLeft),
              static_cast<double>(lastVelRight), kStopCycles, static_cast<double>(kConvergedVelocity));

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

  // ===========================================================================
  // Phase 5 (106-002): secondary telemetry is NOT starved to 0 Hz over this
  // run's real ~40ms/cycle schedule -- `secondary-telemetry-starved-by-106-
  // 001-cadence-retarget.md`'s own regime, reproduced here for real by the
  // REAL App::RobotLoop (via SimApi), not just the App::Telemetry unit
  // harness. This run covers 1 (boot) + 3 (settle) + 20 (ramp) + 12 (stop,
  // 106-003's widened window) = 36 real cycles at ~40ms each, ~1.4s of
  // virtual time -- comfortably more than 5x kSecondaryPeriod (200ms), so a
  // healthy fix should show several secondary frames, not zero.
  // ===========================================================================
  beginScenario("secondary telemetry: not starved to 0 Hz over the run's real schedule");
  checkTrue(secondaryFrameCount > 0,
            "at least one TelemetrySecondary frame decoded across the whole run "
            "(0 would reproduce the pre-106-002 starvation bug)");
  std::printf("  SECONDARY OK: %d TelemetrySecondary frame(s) decoded across the run\n\n", secondaryFrameCount);

  if (g_failureCount == 0) {
    std::printf("OK: scripted-twist demo complete\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the scripted-twist demo\n", g_failureCount);
  return 1;
}
