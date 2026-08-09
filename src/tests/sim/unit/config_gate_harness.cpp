// config_gate_harness.cpp -- ticket 114-001's own acceptance proof: the
// configuration-completeness gate (Core::RobotLoop::configured_/
// markConfigured()/isConfigured(), robot_loop.h/.cpp) makes "unconfigured" a
// real, refusable state on TestSim::SimHarness, and the configured-then-
// accepted transition actually produces real, measured wheel motion -- not
// merely an ACK_STATUS_OK (the gap this ticket's own thrown-and-resolved
// exception found: MotorArmor::configure() used to forward nothing to the
// wrapped NezhaMotor, so a "configured" motor was still functionally dead;
// see sprint.md's Architecture Revision 1 / Decision 6).
//
// Mirrors sim_harness_configure_harness.cpp's own shape (fresh SimHarness
// per scenario, hand-rolled PASS/FAIL assertion plumbing, one file compiled
// standalone by its own pytest wrapper).
//
// Scenarios:
//   1. A freshly-constructed (and booted) SimHarness starts unconfigured.
//   2. MOVE against an unconfigured harness: ack_err == ERR_NOT_CONFIGURED
//      (flags bit 5 fresh); Core::DifferentialDrive's own staged twist is never touched
//      (driveTargetVelLeft/Right() stay 0) and no real (nonzero-speed) duty
//      byte ever reaches the simulated bus (write-hook duty-history check).
//   3. STOP against an unconfigured harness: still ack_err == 0/OK
//      (handleStop() is unconditional, unaffected by the gate).
//   4. CONFIG{motor} against an unconfigured harness: still ack_err == 0/OK
//      (handleConfig() is unconditional, unaffected by the gate).
//   5. Both configureMotor() calls flip isConfigured() to true, and a
//      subsequent MOVE is accepted AND produces real, nonzero measured
//      wheel motion on BOTH motors, with each port driving its OWN distinct
//      simulated wheel (a pure-rotation move drives the two WheelPlants to
//      OPPOSITE-sign velocities -- impossible if both ports were aliased
//      onto the same simulated wheel).
//
// 115-009 (gut S1's own test-sweep/green-bar ticket): a former Scenario 3
// ("MOVE against an unconfigured harness") is DELETED, not ported --
// `injectMove()`/`pilotQueueDepth()`/`pilotState()`/`Motion::State` are all
// Motion::Executor/Core::Pilot-era `TestSim::SimHarness` API deleted
// wholesale by 115-002/115-006 (gut S1 motion-stack excision); there is no
// MOVE command in the S1 minimal firmware to refuse. `findAck()` below is
// also rewritten for the single ack slot (`Telemetry.ack_corr`/`ack_err`,
// valid iff `flags` bit 5) that replaced the deleted depth-3 `AckEntry`
// ring/`AckStatus` enum (115-003 frame v2).
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "bench_test_config.h"
#include "hardware/nezha/nezha_motor.h"
#include "messages/envelope.h"
#include "messages/wire_runtime.h"
#include "sim_harness.h"
#include "wire_test_codec.h"

namespace {

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

void checkFloatEq(float actual, float expected, const std::string& what, float tol = 1e-3f) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual));
    fail(buf);
  }
}

// Finds `corrId`'s own entry in the bounded ack ring (Telemetry.acks,
// 124-008: packed uint32, corr_id<<4|err -- the single "freshest ack"
// scalar slot / kFlagAckFresh this function used to scan is DELETED, issue
// §B4; ring membership alone means "really acked," no freshness gate
// needed) across every decoded telemetry frame in `lines`. Returns true
// and fills *errCode if found (0 == OK, nonzero == the msg::ErrCode value).
bool findAck(const std::vector<TestSupport::DecodedLine>& lines, uint32_t corrId, uint32_t* errCode) {
  constexpr uint32_t kAckErrBits = 4;
  constexpr uint32_t kAckErrMask = (1u << kAckErrBits) - 1;
  for (const auto& line : lines) {
    if (line.kind != TestSupport::DecodedKind::kTelemetry) continue;
    for (uint8_t i = 0; i < line.telemetry.acks_count; ++i) {
      const uint32_t packed = line.telemetry.acks_[i];
      if ((packed >> kAckErrBits) == corrId) {
        *errCode = packed & kAckErrMask;
        return true;
      }
    }
  }
  return false;
}

// --- Hand-rolled CommandEnvelope{config: SetConfigGroup{WHEEL_CONTROL, ...}} encoder --
// (mirrors app_robot_loop_harness.cpp's own Buf/putVarintField/
// putFloatField/putMessageField/armorLine helpers -- no encode(CommandEnvelope)
// codec exists, firmware only ever decodes one; a host/test builds commands
// by hand against the same WireRuntime primitives the generated codec uses).
using WireRuntime::WireType;

struct Buf {
  uint8_t data[256] = {};
  size_t len = 0;
};

bool putVarintField(Buf& b, uint32_t number, uint64_t v) {
  return WireRuntime::encodeTag(number, WireType::kVarint, b.data, sizeof(b.data), &b.len) &&
         WireRuntime::encodeVarint(v, b.data, sizeof(b.data), &b.len);
}

bool putFloatField(Buf& b, uint32_t number, float v) {
  return WireRuntime::encodeTag(number, WireType::kFixed32, b.data, sizeof(b.data), &b.len) &&
         WireRuntime::encodeFloat(v, b.data, sizeof(b.data), &b.len);
}

bool putBytesField(Buf& b, uint32_t number, const uint8_t* payload, size_t payloadLen) {
  if (!WireRuntime::encodeTag(number, WireType::kLengthDelimited, b.data, sizeof(b.data), &b.len)) return false;
  if (!WireRuntime::encodeVarint(payloadLen, b.data, sizeof(b.data), &b.len)) return false;
  if (b.len + payloadLen > sizeof(b.data)) return false;
  std::memcpy(b.data + b.len, payload, payloadLen);
  b.len += payloadLen;
  return true;
}

bool putMessageField(Buf& b, uint32_t number, const Buf& nested) {
  return putBytesField(b, number, nested.data, nested.len);
}

// armorLine() -- 124-005 (protocol v5 Part A, "framing grammar cutover"):
// builds the COMPLETE wire LINE, `<command>':'<COBS+CRC bytes>` (CRC-then-
// COBS, delimiter 0x0A), byte-for-byte the same composition as
// Core::Comms::sendReply()/decodeBinaryFrame() / TestSupport::armor()
// (wire_test_codec.cpp). `command` is REQUIRED -- every dispatched binary
// command needs a real registry verb name (messages/commands.h); this
// file's only caller (armorMotorConfigCommand() below) always passes
// "CONFIG". The trailing '\n' terminator is a transport concern, not
// included here -- SimHarness::injectCommand() (this file's only caller)
// takes an EXPLICIT length, never strlen()-recovered: COBS is keyed on
// 0x0A now, not 0x00, so the line may legitimately contain an embedded
// 0x00 byte.
std::string armorLine(const uint8_t* raw, size_t rawLen, const char* command) {
  uint8_t combined[256];
  if (rawLen > sizeof(combined) - 2) return std::string();
  std::memcpy(combined, raw, rawLen);
  size_t combinedLen = rawLen;
  const size_t commandLen = std::strlen(command);
  uint16_t crc = WireRuntime::crcInit();
  crc = WireRuntime::crcUpdate(crc, reinterpret_cast<const uint8_t*>(command), commandLen);
  const uint8_t sep = ':';
  crc = WireRuntime::crcUpdate(crc, &sep, 1);
  crc = WireRuntime::crcUpdate(crc, raw, rawLen);
  if (!WireRuntime::encodeCrc16(crc, combined, sizeof(combined), &combinedLen)) return std::string();
  uint8_t framed[300];
  size_t framedLen = 0;
  if (!WireRuntime::cobsEncode(combined, combinedLen, framed, sizeof(framed), &framedLen, /*delimiter=*/0x0A)) {
    return std::string();
  }
  std::string line(command, commandLen);
  line += ':';
  line.append(reinterpret_cast<const char*>(framed), framedLen);
  return line;
}

// Builds an armored CommandEnvelope{corr_id, config: SetConfigGroup{
// target=WHEEL_CONTROL, body=WheelControl{..., pid_kp=kp}}} line --
// WHEEL_CONTROL (132-013's replacement for the deleted MOTOR patch that
// used to carry kp -- see Configurator::isLiveConfigurable(),
// configurator.cpp) is live (Configurator::install(WHEEL_CONTROL) runs
// unconditionally, unaffected by the configuration-completeness gate this
// file tests) and demonstrates "CONFIG stays unconditional" the same way
// the deleted MOTOR patch used to. Every OTHER WheelControl field is
// encoded at its own real baked default (config/boot_config.cpp's
// defaultWheelControllerConfig()) -- applyGroup() is a WHOLE-GROUP
// replace, no merge (132-008), so leaving them unset would silently zero
// them in config_.wheelControl instead of only changing kp.
std::string armorWheelControlConfigCommand(float kp, uint32_t corrId) {
  Buf wheelControl;
  putFloatField(wheelControl, 1, 99.7f);   // v_min [mm/s]
  putFloatField(wheelControl, 2, 23.8f);   // bias_max [mm/s]
  putFloatField(wheelControl, 3, 30.0f);   // tau_adapt [s]
  putFloatField(wheelControl, 4, 30.0f);   // a_steady [mm/s^2]
  putFloatField(wheelControl, 5, 0.0f);    // deficit_threshold [mm/s]
  putFloatField(wheelControl, 6, 0.0f);    // deficit_window [ms]
  putFloatField(wheelControl, 7, kp);      // pid_kp -- the field under test
  putFloatField(wheelControl, 8, 0.0f);    // pid_ki [1/s]
  putFloatField(wheelControl, 9, 0.0f);    // pid_i_max [mm/s]
  putFloatField(wheelControl, 10, 0.0f);   // pid_kaff [s]
  putFloatField(wheelControl, 11, 0.0f);   // pid_max [mm/s]
  Buf setConfigGroup;
  putVarintField(setConfigGroup, 1, 4);  // SetConfigGroup.target = WHEEL_CONTROL (4)
  putMessageField(setConfigGroup, 2, wheelControl);  // SetConfigGroup.body, field 2
  Buf env;
  putVarintField(env, 1, corrId);            // CommandEnvelope.corr_id
  putMessageField(env, 6, setConfigGroup);   // CommandEnvelope.config, field 6
  return armorLine(env.data, env.len, "CONFIG");
}

}  // namespace

int main() {
  std::printf("=== Config-Completeness Gate (114-001, SUC-001/SUC-004) ===\n\n");

  const uint16_t kNezhaWireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);

  // --- Scenario 1: fresh + booted SimHarness starts unconfigured ---------
  {
    beginScenario("fresh SimHarness starts unconfigured, even after boot()");
    TestSim::SimHarness sim;
    checkTrue(!sim.isConfigured(), "isConfigured() is false immediately after construction");
    sim.boot();
    sim.step(3);
    checkTrue(!sim.isConfigured(), "isConfigured() stays false after boot() + a few cycles (boot never configures)");
  }

  // --- Scenario 2: MOVE refused while unconfigured -------------------------
  {
    beginScenario("MOVE against an unconfigured harness: ERR_NOT_CONFIGURED, drive_ untouched, no real duty write");
    TestSim::SimHarness sim;
    sim.boot();
    sim.step(3);
    (void)sim.drainTelemetry();

    uint8_t maxSpeedSeen = 0;
    sim.plant().setWriteHook([&](uint16_t address, uint8_t* data, int len) -> int {
      if (address == kNezhaWireAddr && len >= 6 && data[4] == 0x60) {
        if (data[5] > maxSpeedSeen) maxSpeedSeen = data[5];
      }
      return sim.plant().defaultWrite(address, data, len);
    });

    checkTrue(!sim.isConfigured(), "setup: still unconfigured before the move");
    sim.injectMove(/*v_x=*/1000.0f, /*v_y=*/0.0f, /*omega=*/0.0f, TestSupport::MoveStopKind::kTime,
                    /*stopValue=*/100000.0f, /*timeout=*/100000.0f, /*replace=*/true, /*id=*/501,
                    /*corrId=*/501);
    sim.step(10);

    sim.plant().clearWriteHook();

    std::vector<TestSupport::DecodedLine> lines = sim.drainTelemetry();
    uint32_t errCode = 0;
    checkTrue(findAck(lines, 501, &errCode), "an ack for corrId=501 was seen");
    checkTrue(errCode == static_cast<uint32_t>(msg::ErrCode::ERR_NOT_CONFIGURED),
              "the err_code is ERR_NOT_CONFIGURED");

    checkFloatEq(sim.driveTargetVelLeft(), 0.0f,
                 "drive_'s own staged left target stays 0 -- handleMove() never called drive_.setTwist()");
    checkFloatEq(sim.driveTargetVelRight(), 0.0f,
                 "drive_'s own staged right target stays 0 -- handleMove() never called drive_.setTwist()");
    checkTrue(maxSpeedSeen == 0,
              "no nonzero-speed 0x60 duty byte ever reached the bus during the refused move's window "
              "(plant write-hook duty-history check)");
  }

  // --- Scenario 3: STOP still works while unconfigured ---------------------
  {
    beginScenario("STOP against an unconfigured harness: still ack_err==0/OK (unconditional, unaffected by the gate)");
    TestSim::SimHarness sim;
    sim.boot();
    sim.step(3);
    (void)sim.drainTelemetry();

    checkTrue(!sim.isConfigured(), "setup: still unconfigured before the stop");
    sim.injectStop(/*corrId=*/503);
    sim.step(3);

    std::vector<TestSupport::DecodedLine> lines = sim.drainTelemetry();
    uint32_t errCode = 1;  // any nonzero sentinel -- overwritten by findAck() on a match
    checkTrue(findAck(lines, 503, &errCode), "an ack for corrId=503 was seen");
    checkTrue(errCode == 0,
              "STOP still acks ack_err==0/OK even though the harness is unconfigured");
  }

  // --- Scenario 4: CONFIG still works while unconfigured -------------------
  {
    beginScenario("CONFIG{WHEEL_CONTROL} against an unconfigured harness: still ack_err==0/OK "
                  "(unconditional, unaffected by the gate)");
    TestSim::SimHarness sim;
    sim.boot();
    sim.step(3);
    (void)sim.drainTelemetry();

    checkTrue(!sim.isConfigured(), "setup: still unconfigured before the config push");
    std::string line = armorWheelControlConfigCommand(/*kp=*/0.05f, /*corrId=*/504);
    checkTrue(!line.empty(), "armor() of the CONFIG{WHEEL_CONTROL} envelope succeeds");
    sim.injectCommand(line);
    sim.step(3);

    std::vector<TestSupport::DecodedLine> lines = sim.drainTelemetry();
    uint32_t errCode = 1;  // any nonzero sentinel -- overwritten by findAck() on a match
    checkTrue(findAck(lines, 504, &errCode), "an ack for corrId=504 was seen");
    checkTrue(errCode == 0,
              "CONFIG{WHEEL_CONTROL} still acks ack_err==0/OK even though the harness is unconfigured");
    // pid_kp routes to Core::DifferentialDrive's unified wheel-speed controller (Stage
    // B) via Configurator::install(WHEEL_CONTROL) -> Drive::configure() --
    // the ONE wheel controller every cmdVelocity writer shares (drive.h's
    // own header). 132-013 (patch-surface retirement): this used to be
    // MotorConfigPatch.kp/the MOTOR arm; now it is WheelControl.pid_kp/the
    // WHEEL_CONTROL group, applyGroup()'s whole-group push replacing the
    // deleted merge-patch, same live-apply-while-unconfigured property.
    checkFloatEq(sim.drive().controlGains().kp, 0.05f,
                 "the pushed group's pid_kp actually landed live on Drive's controller -- "
                 "CONFIG ran, unaffected by the gate");
  }

  // --- Scenario 5: configured-then-accepted -- real motion, no port
  //     aliasing (the exact gap this ticket's own thrown exception found:
  //     a "configured" motor that was still functionally dead) ------------
  {
    beginScenario("both configureMotor() calls: isConfigured() becomes true, a subsequent "
                  "MOVE is accepted and produces real, nonzero measured wheel motion on BOTH motors, "
                  "each port driving its own distinct simulated wheel");
    TestSim::SimHarness sim;
    sim.boot();
    sim.step(3);
    checkTrue(!sim.isConfigured(), "setup: unconfigured immediately after boot, before configuring");

    TestSupport::configureSimForBenchTest(sim);
    checkTrue(sim.isConfigured(),
              "isConfigured() == true once both configureMotor() calls have landed");

    (void)sim.drainTelemetry();

    // Pure rotation (v_x=0, omega!=0): a correctly-ported drivetrain drives
    // the left and right wheels in OPPOSITE directions. If configureMotor()
    // had failed to propagate `port` (the exact aliasing bug Revision 1
    // fixed -- both motors constructed with Hal::MotorConfig{}'s
    // port=0, never corrected), both simulated writes would land on the
    // SAME WheelPlant and the other port's own plant would stay stone dead
    // at velocity 0 for the whole run -- impossible to produce opposite-sign
    // motion on BOTH plants simultaneously.
    sim.injectMove(/*v_x=*/0.0f, /*v_y=*/0.0f, /*omega=*/1.5f, TestSupport::MoveStopKind::kTime,
                    /*stopValue=*/100000.0f, /*timeout=*/100000.0f, /*replace=*/true, /*id=*/505,
                    /*corrId=*/505);

    std::vector<TestSupport::DecodedLine> lines;
    uint32_t errCode = 1;  // any nonzero sentinel -- overwritten by findAck() on a match
    // The ack for this injection may not land until the NEXT drain -- step a
    // little first, matching every other scenario in this codebase's own
    // "ack rides the next emitted frame" convention.
    sim.step(5);
    lines = sim.drainTelemetry();
    checkTrue(findAck(lines, 505, &errCode), "an ack for corrId=505 was seen");
    checkTrue(errCode == 0,
              "the move is accepted (ack_err==0/OK) now that the harness is configured");

    // Give the PID a few more cycles to actually ramp real duty onto the bus
    // and for the plant's own first-order response to become measurable --
    // this is the acceptance criterion the original exception found
    // unsatisfiable: velL/velR must become NONZERO, not stay frozen at 0.00.
    sim.step(15);

    float velLeft = sim.motorLeft().velocity();
    float velRight = sim.motorRight().velocity();
    std::printf("  velLeft=%.3f velRight=%.3f (mm/s, measured via Hardware::NezhaMotor::velocity())\n",
                static_cast<double>(velLeft), static_cast<double>(velRight));
    checkTrue(std::fabs(velLeft) > 5.0f,
              "left wheel measured velocity is genuinely nonzero -- NOT frozen at 0.00 (the original "
              "exception's own symptom)");
    checkTrue(std::fabs(velRight) > 5.0f,
              "right wheel measured velocity is genuinely nonzero -- NOT frozen at 0.00 (the original "
              "exception's own symptom)");
    checkTrue((velLeft > 0.0f) != (velRight > 0.0f),
              "left and right measured velocities have OPPOSITE sign, as a pure rotation demands");

    // No port aliasing: read the PLANT's own ground truth per port directly
    // -- an aliased setup could never show both simulated WheelPlants
    // independently spinning in opposite directions (one port's own writes
    // would silently land on the OTHER port's physical wheel, leaving that
    // second WheelPlant dead at 0 the whole run).
    float plantVelLeft = sim.plant().wheelPlant(1).velocity();
    float plantVelRight = sim.plant().wheelPlant(2).velocity();
    std::printf("  plant wheelPlant(1)=%.3f wheelPlant(2)=%.3f (mm/s, ground truth, per port)\n",
                static_cast<double>(plantVelLeft), static_cast<double>(plantVelRight));
    checkTrue(std::fabs(plantVelLeft) > 5.0f, "plant port 1 (left) is genuinely spinning");
    checkTrue(std::fabs(plantVelRight) > 5.0f, "plant port 2 (right) is genuinely spinning");
    checkTrue((plantVelLeft > 0.0f) != (plantVelRight > 0.0f),
              "plant ports 1 and 2 spin in OPPOSITE directions -- each port drives its OWN distinct "
              "simulated wheel, no port aliasing");
  }

  std::printf("\n");
  if (g_failureCount == 0) {
    std::printf("OK: all config-completeness gate scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the config-completeness gate scenarios\n", g_failureCount);
  return 1;
}
