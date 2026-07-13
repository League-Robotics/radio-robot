// devices_motor_harness.cpp — off-hardware acceptance harness for ticket
// DB-004 (device-bus-tickets.md): exercises Devices::MotorArmor's shared
// armor policy (zero-dwell reversal, output deadband, standstill-guarded
// resets, motion-qualified wedge reporting) through a dependency-free
// MockDeviceMotor leaf, AND Devices::NezhaMotor's own request/collect
// encoder pairing, embedded PID, and PID-on/off dispatch through the real
// leaf against the scripted Devices::I2CBus (DB-003).
//
// Modeled on tests/sim/unit/motor_policy_harness.cpp (the MockMotor-style
// armor scenarios) + tests/sim/unit/velocity_pid_harness.cpp (the PID
// convergence-scenario style), per device-bus-tickets.md's DB-004
// acceptance criteria. Unlike those pre-port harnesses, this one #includes
// ONLY source/devices/ headers (isolation invariant) plus plain C/C++
// stdlib -- no messages/*.h, no com/i2c_bus.h.
//
// Plain C++ program, hand-rolled assertions -- prints a PASS/FAIL line per
// scenario and exits nonzero if any assertion failed. Run by the pytest
// wrapper in test_devices_motor.py, which compiles this file together with
// source/devices/i2c_bus_host.cpp, source/devices/velocity_pid.cpp, and
// source/devices/nezha_motor.cpp under -DHOST_BUILD, then runs the
// resulting binary via subprocess and asserts exit code 0.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "devices/device_config.h"
#include "devices/device_types.h"
#include "devices/i2c_bus.h"
#include "devices/motor_armor.h"
#include "devices/nezha_motor.h"
#include "devices/velocity_pid.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors motor_policy_harness.cpp) ---

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
  if (!condition) fail(what + " — expected true, got false");
}

void checkFloatEq(float actual, float expected, const std::string& what,
                   float tol = 1e-3f) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s — expected %g, got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual));
    fail(buf);
  }
}

void checkUintEq(uint32_t actual, uint32_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s — expected %u, got %u", what.c_str(),
                  static_cast<unsigned>(expected),
                  static_cast<unsigned>(actual));
    fail(buf);
  }
}

void checkVecEq(const std::vector<float>& actual,
                 const std::vector<float>& expected, const std::string& what) {
  bool ok = actual.size() == expected.size();
  if (ok) {
    for (size_t i = 0; i < actual.size(); ++i) {
      if (std::fabs(actual[i] - expected[i]) > 1e-6f) { ok = false; break; }
    }
  }
  if (!ok) {
    fail(what + " — write-call sequence mismatch");
  }
}

// --- MockDeviceMotor --------------------------------------------------
//
// Implements only the three device-specific protected virtuals
// Devices::MotorArmor requires (writeRawDuty/hardReset/softRebaseline) plus
// the three public leaf getters (position/velocity/appliedDuty) -- no I2C,
// no CODAL, no dependency beyond devices/motor_armor.h and
// devices/device_config.h. Every recorded call goes into a plain member
// vector/counter the scenarios assert on directly. Mirrors
// motor_policy_harness.cpp's MockMotor.
class MockDeviceMotor : public Devices::MotorArmor {
 public:
  // --- Test-driving surface (beyond the MotorArmor faceplate) ---

  void setMockPosition(float position) { mockPosition_ = position; }
  void setMockVelocity(float velocity) { mockVelocity_ = velocity; }

  // Drives the write gate directly with an arbitrary (duty, now) pair, as
  // if it were the output of any upstream control law.
  void requestDuty(float duty, uint32_t now) { armoredWrite(duty, now); }

  void stageDuty(float duty) { stagedDuty_ = duty; }

  bool isDwelling() const { return dwelling_; }

  // Drives the same 5-step call order NezhaMotor::tick() documents:
  // standstill-guarded reset dispatch, wedge detector (reads last tick's
  // appliedDuty()), a dispatch of whatever stageDuty() last staged, then
  // rest tracking.
  void tick(uint32_t now) {
    processResetIfPending(now);
    updateWedgeDetector();
    armoredWrite(stagedDuty_, now);
    updateRestTracking();
  }

  float position() const override { return mockPosition_; }
  float velocity() const override { return mockVelocity_; }
  float appliedDuty() const override { return lastWrittenDuty_; }

  // --- Call recording (scenarios assert on these directly) ---
  std::vector<float> writeRawDutyCalls;
  int hardResetCalls = 0;
  int softRebaselineCalls = 0;

 protected:
  void writeRawDuty(float duty) override {
    writeRawDutyCalls.push_back(duty);
    lastWrittenDuty_ = duty;
  }
  void hardReset() override { ++hardResetCalls; }
  void softRebaseline() override {
    ++softRebaselineCalls;
    // Ported leaf contract (see nezha_motor.cpp's NezhaMotor::
    // softRebaseline()): softResetCount_ is base-owned but the LEAF
    // increments it at its own call site.
    ++softResetCount_;
  }

 private:
  float mockPosition_ = 0.0f;
  float mockVelocity_ = 0.0f;
  float stagedDuty_ = 0.0f;
  float lastWrittenDuty_ = 0.0f;
};

// Ship-default config (both armor fields left unset — configureArmor()
// substitutes kDefaultReversalDwell=100ms / kDefaultOutputDeadband=0.03).
Devices::MotorConfig defaultArmorConfig() { return Devices::MotorConfig{}; }

// --- MotorArmor scenarios (via MockDeviceMotor) ------------------------

// 1. A commanded sign change writes 0 immediately, suppresses further
//    non-zero writes until reversalDwell_ has elapsed, then forwards the new
//    direction. Also proves a commanded stop (duty == 0) is immediate and
//    cancels an in-progress dwell.
void scenarioReversalDwellWritesZeroThenHoldsThroughDeadline() {
  beginScenario("reversal dwell writes 0 then holds through the deadline");
  MockDeviceMotor m;
  m.configureArmor(defaultArmorConfig());   // reversalDwell_=100ms, outputDeadband_=0.03

  m.requestDuty(0.5f, 1000);    // no prior direction — forwarded immediately
  m.requestDuty(-0.5f, 1010);   // sign flip — write 0, arm the 100ms dwell
  m.requestDuty(-0.5f, 1050);   // still inside the dwell — suppressed to 0
  m.requestDuty(-0.5f, 1109);   // still inside the dwell (1109 < 1110)
  m.requestDuty(-0.5f, 1110);   // dwell elapsed — new direction forwarded

  checkVecEq(m.writeRawDutyCalls, {0.5f, 0.0f, 0.0f, 0.0f, -0.5f},
             "write-call sequence across the reversal");
  checkTrue(!m.isDwelling(), "dwell cleared once the deadline elapses");

  // A commanded stop mid-dwell is immediate and cancels the dwell.
  MockDeviceMotor stopMotor;
  stopMotor.configureArmor(defaultArmorConfig());
  stopMotor.requestDuty(0.5f, 2000);    // establish a direction
  stopMotor.requestDuty(-0.5f, 2010);   // sign flip — arms dwell, deadline=2110
  stopMotor.requestDuty(0.0f, 2020);    // explicit stop mid-dwell — immediate, cancels
  checkFloatEq(stopMotor.writeRawDutyCalls.back(), 0.0f, "stop wrote 0 immediately");
  stopMotor.requestDuty(-0.5f, 2030);   // well before 2110 — forwarded immediately, dwell was cancelled
  checkFloatEq(stopMotor.writeRawDutyCalls.back(), -0.5f,
               "post-stop command forwarded immediately, dwell was cancelled");
}

// 2. A sub-outputDeadband_ duty request writes 0, not a tiny signed value —
//    including rapid sub-threshold sign dithering around zero, which never
//    arms a reversal dwell (proven by an immediate, unsuppressed write of a
//    later legitimate command).
void scenarioSubDeadbandDutyImmediateAndUnclamped() {
  beginScenario("sub-deadband duty is immediate/unclamped");
  MockDeviceMotor m;
  m.configureArmor(defaultArmorConfig());   // outputDeadband_=0.03

  m.requestDuty(0.01f, 2000);    // below deadband — writes 0, not 0.01
  m.requestDuty(-0.02f, 2001);   // below deadband, opposite sign — still 0
  m.requestDuty(0.02f, 2002);    // dithering back — still 0

  checkVecEq(m.writeRawDutyCalls, {0.0f, 0.0f, 0.0f},
             "sub-deadband requests all wrote 0, immediately and unclamped");

  // If the dither above had incorrectly armed a reversal dwell, this next
  // legitimate command would be suppressed to 0 instead of forwarded
  // immediately.
  m.requestDuty(0.5f, 2010);
  checkFloatEq(m.writeRawDutyCalls.back(), 0.5f,
               "post-dither command forwarded immediately (no phantom dwell)");
}

// 3. resetPosition() while moving (lastRequestedDuty_ != 0, so restTicks_
//    never accumulates to kRestTicksRequired) dispatches softRebaseline(),
//    never hardReset(); at verified standstill it dispatches hardReset().
//    Together: "standstill-guarded reset gates on rest ticks."
void scenarioStandstillGuardedResetGatesOnRestTicks() {
  beginScenario("standstill-guarded reset gates on rest ticks");

  // (a) Moving: two ticks of motion — restTicks_ stays 0 throughout.
  {
    MockDeviceMotor m;
    m.configureArmor(defaultArmorConfig());
    m.setMockVelocity(80.0f);   // well above kRestVelocity — also moving
    m.stageDuty(0.5f);          // above deadband — keeps lastRequestedDuty_ != 0

    uint32_t now = 4000;
    m.tick(now);
    now += 20;
    m.tick(now);

    m.resetPosition();          // stages resetPending_ = true
    now += 20;
    m.tick(now);                 // processResetIfPending() dispatches here

    checkUintEq(static_cast<uint32_t>(m.softRebaselineCalls), 1,
                "softRebaseline() called exactly once while moving");
    checkUintEq(static_cast<uint32_t>(m.hardResetCalls), 0,
                "hardReset() never called while moving");
    checkUintEq(m.softResetCount(), 1, "base softResetCount() reflects the call");
    checkUintEq(m.hardResetCount(), 0, "base hardResetCount() stays 0");
  }

  // (b) Verified standstill: well past kRestTicksRequired (proposed: 5)
  // ticks at rest.
  {
    MockDeviceMotor m;
    m.configureArmor(defaultArmorConfig());
    m.setMockVelocity(0.0f);   // below kRestVelocity throughout
    // stagedDuty_ defaults to 0 — never commanded to move.

    uint32_t now = 5000;
    for (int i = 0; i < 8; ++i) {
      m.tick(now);
      now += 20;
    }

    m.resetPosition();
    m.tick(now);   // processResetIfPending() dispatches here

    checkUintEq(static_cast<uint32_t>(m.hardResetCalls), 1,
                "hardReset() called exactly once at verified standstill");
    checkUintEq(static_cast<uint32_t>(m.softRebaselineCalls), 0,
                "softRebaseline() never called at verified standstill");
    checkUintEq(m.hardResetCount(), 1, "base hardResetCount() reflects the call");
    checkUintEq(m.softResetCount(), 0, "base softResetCount() stays 0");
  }
}

// 4. wedged() is the raw, unconditional stuck-encoder latch; wedgeSuspect()
//    is the same test additionally gated on |appliedDuty()| > outputDeadband_
//    — an idle parked motor with a frozen position never reports suspect.
void scenarioWedgeLatchAndSuspectDeriveAsBefore() {
  beginScenario("wedge latch + wedge-suspect derive as before");

  // (a) Idle parked motor: frozen position, zero applied duty throughout.
  {
    MockDeviceMotor idle;
    idle.configureArmor(defaultArmorConfig());
    idle.setMockPosition(100.0f);   // never changes
    idle.setMockVelocity(0.0f);
    // stagedDuty_ stays 0 — appliedDuty() never exceeds the deadband.
    uint32_t now = 6000;
    for (int i = 0; i < 15; ++i) {   // well past kWedgeThreshold (10)
      idle.tick(now);
      now += 20;
    }
    checkTrue(idle.wedged(), "idle parked motor: wedged() latches (unconditional)");
    checkTrue(!idle.wedgeSuspect(),
              "idle parked motor: wedgeSuspect() stays false (never moving)");
  }

  // (b) Same frozen position, but commanded above the deadband — moving
  //     (per appliedDuty()) every tick, so the motion-qualified counter runs
  //     alongside the unconditional one.
  {
    MockDeviceMotor moving;
    moving.configureArmor(defaultArmorConfig());
    moving.setMockPosition(100.0f);   // still never changes — genuinely stuck
    moving.setMockVelocity(0.0f);
    moving.stageDuty(0.5f);           // above deadband every tick
    uint32_t now = 7000;
    for (int i = 0; i < 15; ++i) {
      moving.tick(now);
      now += 20;
    }
    checkTrue(moving.wedged(), "moving-but-stuck motor: wedged() latches");
    checkTrue(moving.wedgeSuspect(),
              "moving-but-stuck motor: wedgeSuspect() also latches");
  }
}

// --- Devices::NezhaMotor scenarios (real leaf, scripted Devices::I2CBus) ---

// Packs positionMm into the little-endian int32 tenths-of-degree raw
// encoder reading NezhaMotor::collectEncoder() decodes (mirrors
// motor_policy_harness.cpp's scriptNezhaEncoderReading() convention).
// wheelTravelCalib=1.0, fwdSign=+1 in every scenario below, so raw ==
// positionMm*10 exactly (no rounding drift).
//
// Scripts TWO writes, not one: requestEncoder() (phase 1) always issues one
// 0x46 register-select write, but tick()'s own dispatch (phase 2, same
// cycle) may ALSO land a duty write via writeMotorRun() (0x60) — the
// scripted I2CBus's writes/reads are independent FIFOs matched on address
// only (not on payload), so both the request write and a possible duty
// write draw from the SAME scriptedWrites_ queue, in call order. Scripting
// one slack entry per cycle is always sufficient (tick()'s dispatch issues
// at most one writeRawDuty() call, which itself writes to the bus at most
// once) and harmless when the duty write doesn't land that cycle (write-
// on-change/throttle skip) — the unused slack entry is simply drained by a
// later cycle's write instead, and this fake's scriptWrite() never checks
// the written payload, only address+order, so which "logical" write
// consumes which slot does not matter.
void scriptEncoderRequestCollect(Devices::I2CBus& bus, uint16_t wireAddr,
                                  float positionMm) {
  bus.scriptWrite(wireAddr, /*status=*/0);   // requestEncoder()'s 0x46 write
  bus.scriptWrite(wireAddr, /*status=*/0);   // slack: a possible same-cycle duty write (0x60)

  int32_t raw = static_cast<int32_t>(std::lround(positionMm * 10.0f));
  uint8_t data[4] = {
      static_cast<uint8_t>(raw & 0xFF),
      static_cast<uint8_t>((raw >> 8) & 0xFF),
      static_cast<uint8_t>((raw >> 16) & 0xFF),
      static_cast<uint8_t>((raw >> 24) & 0xFF),
  };
  bus.scriptRead(wireAddr, data, 4, /*status=*/0);   // collectEncoder()'s 4-byte read
}

Devices::MotorConfig baseNezhaConfig() {
  Devices::MotorConfig cfg;
  cfg.port = 1;
  cfg.fwdSign = 1;
  cfg.wheelTravelCalib = 1.0f;
  cfg.velFiltAlpha = 1.0f;   // no smoothing — velocity() reflects each tick's raw difference-quotient exactly
  return cfg;
}

// 5. request->collect encoder pairing produces expected position()/
//    velocity(): requestSample() (phase 1, a scripted write) followed by
//    tick() (phase 2, collectEncoder() — a scripted read) yields the
//    expected position, and velocity() reflects the difference quotient
//    across two paired cycles.
void scenarioRequestCollectPairingYieldsExpectedPositionVelocity() {
  beginScenario("request->collect encoder pairing yields expected position()/velocity()");
  Devices::I2CBus::setClock(1000000);
  Devices::I2CBus bus;
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::NezhaMotor motor(bus, baseNezhaConfig());

  // Prime cycle: request -> collect at position 0.
  scriptEncoderRequestCollect(bus, wireAddr, 0.0f);
  motor.requestSample();
  motor.tick(0);   // nowUs=0

  checkFloatEq(motor.position(), 0.0f, "primed position is 0");

  // Second cycle, 20ms later: position steps to 10mm (500 mm/s, well within
  // plausibility).
  scriptEncoderRequestCollect(bus, wireAddr, 10.0f);
  motor.requestSample();
  motor.tick(20000);   // nowUs=20000 (20ms later)

  checkFloatEq(motor.position(), 10.0f, "position reflects the collected sample");
  checkFloatEq(motor.velocity(), 500.0f, "velocity == delta-position / delta-time");
  checkTrue(motor.connected(), "request+collect pairing reports connected");
  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run across the pairing");
}

// 6. PID-on chases a velocity target: with a simple first-order plant
//    stand-in (measured velocity chases the last-applied duty, one-tick
//    actuation lag — mirrors velocity_pid_harness.cpp's own plant model),
//    driven entirely through NezhaMotor's real tick()/armor/PID path, the
//    tracking error shrinks substantially from the first cycle to the last.
void scenarioPidOnChasesVelocityTarget() {
  beginScenario("PID-on chases a velocity target");
  Devices::I2CBus::setClock(1000000);
  Devices::I2CBus bus;
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::MotorConfig cfg = baseNezhaConfig();
  cfg.velGains = Devices::Gains{/*kp=*/0.01f, /*ki=*/0.05f, /*kff=*/0.002f,
                                 /*iMax=*/1.0f, /*kaw=*/2.0f};
  cfg.velDeadband = 5.0f;   // [mm/s] well below the target — not in deadband

  Devices::NezhaMotor motor(bus, cfg);
  checkTrue(motor.pidEnabled(), "PID is enabled by default");

  const float target = 300.0f;   // [mm/s]
  const uint32_t dtMs = 20;      // [ms] cycle cadence
  const float dtS = 0.02f;       // [s]

  motor.setVelocity(target);

  float position = 0.0f;
  float measuredVel = 0.0f;
  uint64_t nowUs = 0;

  // Prime cycle — establishes lastPosition_/lastTickUs_, no velocity yet.
  scriptEncoderRequestCollect(bus, wireAddr, position);
  motor.requestSample();
  motor.tick(nowUs);

  float firstError = std::fabs(target - motor.velocity());

  const int kTicks = 400;
  for (int i = 0; i < kTicks; ++i) {
    // Plant response to whatever was actually forwarded to the bus last
    // cycle (appliedDuty() — reflects armor's slew/throttle/dwell gating,
    // exactly like the real wheel would).
    float duty = motor.appliedDuty();
    measuredVel += (duty * 500.0f - measuredVel) * 0.1f;
    position += measuredVel * dtS;

    nowUs += static_cast<uint64_t>(dtMs) * 1000;
    scriptEncoderRequestCollect(bus, wireAddr, position);
    motor.requestSample();
    motor.tick(nowUs);
  }

  float lastError = std::fabs(target - motor.velocity());

  checkTrue(lastError < firstError,
            "tracking error shrinks from the first cycle to the last");
  checkTrue(lastError <= firstError * 0.5f,
            "converged error is well below the initial error (no blow-up)");
  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run across the chase");
}

// 7. PID-off feeds raw duty through the armor unchanged: with PID disabled,
//    setVelocity()'s staged target is ignored and setDuty()'s staged raw
//    duty reaches appliedDuty() via the SAME armoredWrite() gate PID-on
//    uses — including a reversal, proving the armor gates identically in
//    both modes ("armor applies in both modes").
void scenarioPidOffRoutesRawDutyThroughArmorUnchanged() {
  beginScenario("PID-off feeds raw duty through the armor unchanged");
  Devices::I2CBus::setClock(1000000);
  Devices::I2CBus bus;
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::MotorConfig cfg = baseNezhaConfig();
  cfg.slewRate = 100.0f;   // no slew clamping — isolates armor gating (reversal dwell) from the independent slew-cap concern
  Devices::NezhaMotor motor(bus, cfg);

  motor.setPidEnabled(false);
  motor.setVelocity(999.0f);   // staged but must be ignored while PID is off
  motor.setDuty(0.5f);

  uint64_t nowUs = 0;
  const float stationaryPosition = 0.0f;   // stationary plant — isolates the write-path behavior

  // Several well-spaced (>40ms apart) cycles so slew/throttle both allow
  // the raw duty to fully land.
  for (int i = 0; i < 5; ++i) {
    scriptEncoderRequestCollect(bus, wireAddr, stationaryPosition);
    motor.requestSample();
    nowUs += 50000;   // 50ms — clears the 40ms write-rate throttle every cycle
    motor.tick(nowUs);
  }

  checkFloatEq(motor.appliedDuty(), 0.5f,
               "raw staged duty reaches the write path (PID target ignored while PID is off)");

  // Now command a reversal while still PID-off: armor must gate it exactly
  // as it would in PID-on mode (write 0, hold through the dwell, then
  // resume in the new direction).
  motor.setDuty(-0.5f);
  scriptEncoderRequestCollect(bus, wireAddr, stationaryPosition);
  motor.requestSample();
  nowUs += 50000;
  motor.tick(nowUs);   // sign flip — armor writes 0, arms the 100ms dwell
  checkFloatEq(motor.appliedDuty(), 0.0f,
               "reversal while PID-off is dwelled by the SAME armor gate (write 0 first)");

  // Still mid-dwell (deadline is dwell-arm-time + 100ms).
  scriptEncoderRequestCollect(bus, wireAddr, stationaryPosition);
  motor.requestSample();
  nowUs += 30000;   // 30ms later — dwell (100ms) not yet elapsed
  motor.tick(nowUs);
  checkFloatEq(motor.appliedDuty(), 0.0f, "still held at 0 mid-dwell");

  // Dwell elapses — resumes the new direction.
  scriptEncoderRequestCollect(bus, wireAddr, stationaryPosition);
  motor.requestSample();
  nowUs += 100000;   // well past the 100ms deadline
  motor.tick(nowUs);
  checkFloatEq(motor.appliedDuty(), -0.5f, "dwell elapsed — resumes the new-direction raw duty");

  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run across the PID-off sequence");
}

// 8. Fresh-sample gate (HARDWARE-CONFIRMED DB-004 fix): the Nezha brick's
//    encoder register refreshes far slower (~80ms) than DeviceBus's fiber
//    cycle (~16ms, DB-007/DB-008). Scripts the SAME raw encoder value for
//    several consecutive request/collect cycles (simulating the brick not
//    having refreshed yet), then a jump to a fresh value, repeated across
//    several refresh windows -- while ALSO commanding a duty so
//    appliedDuty() is nonzero throughout ("normal driving"). Proves
//    velocity() computes the CORRECT speed (step / real elapsed time since
//    the LAST FRESH sample, not this tick's own dt) on every fresh sample,
//    is never rejected as a glitch, is never left starved at/near 0, and
//    that the wedge detector does not false-latch (neither wedged() nor
//    wedgeSuspect()) across the run -- reproducing and proving the fix for
//    the exact bring-up-image symptom (vel=0.000 always, glitch count
//    climbing, wedged=1) while the wheel was physically moving.
void scenarioFreshSampleGateSurvivesSlowBrickRefreshUnderFastFiberCycle() {
  beginScenario("fresh-sample gate survives slow brick refresh under fast fiber cycle");
  Devices::I2CBus::setClock(1000000);
  Devices::I2CBus bus;
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::MotorConfig cfg = baseNezhaConfig();   // velFiltAlpha=1.0 -- velocity() reflects each fresh sample's raw difference-quotient exactly
  Devices::NezhaMotor motor(bus, cfg);

  // Drive a raw duty throughout (PID off, to keep the plant simple/
  // deterministic) so appliedDuty() is nonzero -- exercises the
  // wedgeSuspect() ("commanded but not moving") path alongside the raw
  // wedged() latch, matching the ticket's "normal driving" concern.
  motor.setPidEnabled(false);
  motor.setDuty(0.4f);

  const uint64_t kFiberCycleUs = 16000;    // [us] ~16ms fiber cycle (DB-007/DB-008)
  const int kStaleCyclesPerRefresh = 4;    // 4 stale + 1 fresh = 5 cycles per ~80ms brick refresh
  const float kStepPerRefresh = 40.0f;     // [mm] realistic per-refresh position step (500mm/s-class motion)

  // Prime cycle: boot-anchor at position 0. Scripts an extra write slack
  // entry (scriptEncoderRequestCollect already provides one) to absorb the
  // very first duty write this tick's mode dispatch may also issue.
  uint64_t nowUs = 0;
  float position = 0.0f;
  scriptEncoderRequestCollect(bus, wireAddr, position);
  motor.requestSample();
  motor.tick(nowUs);
  checkFloatEq(motor.position(), 0.0f, "primed position is 0");
  checkUintEq(motor.encGlitchCount(), 0, "boot anchor is not counted as a glitch");

  for (int refresh = 0; refresh < 6; ++refresh) {
    // Stale cycles: the brick has NOT refreshed -- same raw value.
    for (int i = 0; i < kStaleCyclesPerRefresh; ++i) {
      nowUs += kFiberCycleUs;
      scriptEncoderRequestCollect(bus, wireAddr, position);   // unchanged raw
      motor.requestSample();
      motor.tick(nowUs);
    }

    // Fresh cycle: the brick has refreshed -- position jumps by a realistic
    // step. freshElapsed is ALWAYS exactly (kStaleCyclesPerRefresh+1) fiber
    // cycles here (every window is the same length), so the expected
    // velocity is constant across every refresh window.
    position += kStepPerRefresh;
    nowUs += kFiberCycleUs;
    scriptEncoderRequestCollect(bus, wireAddr, position);
    motor.requestSample();
    motor.tick(nowUs);

    const float freshElapsed = static_cast<float>(kFiberCycleUs) *
                                static_cast<float>(kStaleCyclesPerRefresh + 1) / 1e6f;
    const float expectedVel = kStepPerRefresh / freshElapsed;   // 40mm / 0.08s = 500mm/s

    checkFloatEq(motor.position(), position, "position reflects the fresh sample, not a stale intermediate one");
    checkFloatEq(motor.velocity(), expectedVel,
                 "velocity == step / real elapsed time since the LAST FRESH sample (not per-tick)", 1.0f);
    checkTrue(motor.velocity() > 100.0f,
              "velocity is not starved to ~0 by the intervening stale cycles");
  }

  checkUintEq(motor.encGlitchCount(), 0,
              "no false glitches across repeated stale-then-fresh refresh windows");
  checkTrue(!motor.wedged(),
            "raw wedge latch does not false-trigger across normal stale-then-fresh cycling");
  checkTrue(!motor.wedgeSuspect(),
            "motion-qualified wedge-suspect does not false-trigger while genuinely driving");
  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run across the run");
}

// 9. Boot anchor: the first-ever fresh sample may report a large lifetime-
//    accumulated raw count (hardware-observed: ~-33526mm) -- this is
//    NORMAL (an already-running brick's accumulated register), not a
//    glitch. It must be anchored directly (no diff-from-0 "glitch"), and
//    stale/fresh cycling immediately after boot must not re-glitch or
//    loop on the anchor, nor false-latch the wedge detector.
void scenarioBootAnchorAcceptsLargeInitialPositionWithoutGlitchOrWedge() {
  beginScenario("boot anchor accepts a large initial position without glitch or wedge");
  Devices::I2CBus::setClock(1000000);
  Devices::I2CBus bus;
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::NezhaMotor motor(bus, baseNezhaConfig());

  const float kBootPosition = -33526.0f;   // [mm] hardware-observed lifetime-accumulated boot value

  scriptEncoderRequestCollect(bus, wireAddr, kBootPosition);
  motor.requestSample();
  motor.tick(0);

  checkFloatEq(motor.position(), kBootPosition,
               "boot anchor lands directly on the first sample, no diff-from-0 glitch");
  checkFloatEq(motor.velocity(), 0.0f, "no spurious velocity computed on the boot anchor itself");
  checkUintEq(motor.encGlitchCount(), 0, "boot anchor is never counted as a glitch");
  checkTrue(!motor.wedged(), "boot anchor alone does not latch the wedge");

  // A handful of stale cycles right after boot (brick hasn't refreshed
  // yet) must hold the anchor, not loop or re-glitch on it.
  uint64_t nowUs = 0;
  float position = kBootPosition;
  for (int i = 0; i < 3; ++i) {
    nowUs += 16000;
    scriptEncoderRequestCollect(bus, wireAddr, position);   // still stale
    motor.requestSample();
    motor.tick(nowUs);
  }
  checkFloatEq(motor.position(), kBootPosition, "position holds through stale post-boot cycles");
  checkUintEq(motor.encGlitchCount(), 0, "stale post-boot cycles are not glitches (simply not fresh)");
  checkTrue(!motor.wedged(), "stale post-boot cycles alone do not latch the wedge");

  // First genuinely fresh post-boot sample: real, realistic motion off the
  // large boot anchor computes correctly and is not misclassified.
  nowUs += 16000;
  position += 40.0f;
  scriptEncoderRequestCollect(bus, wireAddr, position);
  motor.requestSample();
  motor.tick(nowUs);
  checkFloatEq(motor.position(), position, "first post-boot fresh sample lands correctly");
  checkTrue(motor.velocity() > 100.0f, "first post-boot fresh sample yields a real (non-zero) velocity");
  checkUintEq(motor.encGlitchCount(), 0, "genuine post-boot motion off a large anchor is not a glitch");
  checkTrue(!motor.wedged(), "no wedge latch through the boot+stale+fresh sequence");
  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run across the boot sequence");
}

}  // namespace

int main() {
  scenarioReversalDwellWritesZeroThenHoldsThroughDeadline();
  scenarioSubDeadbandDutyImmediateAndUnclamped();
  scenarioStandstillGuardedResetGatesOnRestTicks();
  scenarioWedgeLatchAndSuspectDeriveAsBefore();
  scenarioRequestCollectPairingYieldsExpectedPositionVelocity();
  scenarioPidOnChasesVelocityTarget();
  scenarioPidOffRoutesRawDutyThroughArmorUnchanged();
  scenarioFreshSampleGateSurvivesSlowBrickRefreshUnderFastFiberCycle();
  scenarioBootAnchorAcceptsLargeInitialPositionWithoutGlitchOrWedge();

  if (g_failureCount == 0) {
    std::printf("OK: all devices motor scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the devices motor scenarios\n",
              g_failureCount);
  return 1;
}
