// motor_policy_harness.cpp — off-hardware acceptance harness for ticket
// 078-004 (SUC-005): exercises Hal::Motor's shared armor policy (zero-dwell
// reversal, output deadband, standstill-guarded resets, motion-qualified
// wedge reporting — sprint 078) through a dependency-free MockMotor leaf.
//
// Per architecture-update.md Decision 9, this deliberately does NOT build
// the deferred new-tree simulator or a scripted I2CBus HOST_BUILD fake
// (neither exists in operable form today — see that decision's rationale).
// Instead: MockMotor implements only the four protected pure virtuals
// Hal::Motor now requires (writeRawDuty/hardReset/softRebaseline/
// configureDevice), recording every call instead of touching hardware — no
// I2C, no CODAL. This #includes only capability/motor.h and messages/*.h,
// both already dependency-free (no MicroBit.h, no I2CBus), so it compiles
// with the plain system C++ compiler — no CMake, no ARM toolchain.
//
// messages/common.h documents its own target as "CODAL C++11"; this harness
// is compiled to the same standard (see test_motor_policy.py's compile
// command) so it exercises exactly the language subset the firmware itself
// uses.
//
// Plain C++ program, hand-rolled assertions (six scenarios do not warrant a
// test-framework dependency) — prints a PASS/FAIL line per scenario and
// exits nonzero if any assertion failed. Run by the pytest wrapper in
// test_motor_policy.py, which compiles and runs this binary via subprocess
// and asserts exit code 0.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "hal/capability/motor.h"
#include "messages/motor.h"

namespace {

// --- Hand-rolled assertion plumbing -----------------------------------
// (kept intentionally tiny — six-to-eight scenarios do not justify pulling
// in a test framework for a dependency-free host harness.)

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

void checkFloatEq(float actual, float expected, const std::string& what) {
  if (std::fabs(actual - expected) > 1e-6f) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s — expected %g, got %g",
                  what.c_str(), static_cast<double>(expected),
                  static_cast<double>(actual));
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

// --- MockMotor ------------------------------------------------------------
//
// Implements only the four device-specific protected virtuals Hal::Motor
// requires (writeRawDuty/hardReset/softRebaseline/configureDevice), plus the
// minimal existing primitive setters/getters needed to drive scripted
// scenarios (see the ticket's acceptance criteria — no I2C, no CODAL, no
// dependency beyond capability/motor.h and messages/*.h). Every recorded
// call goes into a plain member vector/counter the scenarios assert on
// directly.
class MockMotor : public Hal::Motor {
 public:
  // --- Test-driving surface (beyond the Motor faceplate) ---

  // Injects the simulated encoder position() a scenario wants position()/
  // velocity() to report — nothing in Hal::Motor itself simulates a plant,
  // so the harness stands in for it.
  void setMockPosition(float position) { mockPosition_ = position; }
  void setMockVelocity(float velocity) { mockVelocity_ = velocity; }

  // Drives the write gate directly with an arbitrary (duty, now) pair, as
  // if it were the output of any upstream control law (a raw DUTY
  // passthrough, a VELOCITY-mode PID, anything else) — see the
  // PID-independence scenario, which asserts armoredWrite()'s decision
  // depends only on this pair, never on which call path produced it.
  void requestDuty(float duty, uint32_t now) { armoredWrite(duty, now); }

  // Drives the same 5-step call order NezhaMotor::tick() documents
  // (architecture-update.md / nezha_motor.cpp): standstill-guarded reset
  // dispatch, wedge detector (reads last tick's appliedDuty()), a DUTY-mode
  // dispatch of whatever setDutyCycle() last staged, then rest tracking.
  void tick(uint32_t now) override {
    processResetIfPending(now);
    updateWedgeDetector();
    armoredWrite(stagedDuty_, now);
    updateRestTracking();
  }

  // --- Motor primitive setters ---
  void setDutyCycle(float dutyCycle) override { stagedDuty_ = dutyCycle; }
  void setVoltage(float /*voltage*/) override {}
  void setVelocity(float /*velocity*/) override {}
  void setPosition(float /*position*/) override {}
  void setNeutral(msg::Neutral /*mode*/) override {}
  void setFeedforward(float /*feedforward*/) override {}

  // --- Motor primitive getters ---
  float position() const override { return mockPosition_; }
  float velocity() const override { return mockVelocity_; }
  float appliedDuty() const override { return lastWrittenDuty_; }
  bool connected() const override { return true; }

  msg::MotorCapabilities capabilities() const override {
    msg::MotorCapabilities caps;
    caps.duty_cycle = true;
    caps.has_encoder = true;
    return caps;
  }

  // --- Call recording (scenarios assert on these directly) ---
  std::vector<float> writeRawDutyCalls;
  int hardResetCalls = 0;
  int softRebaselineCalls = 0;
  int configureDeviceCalls = 0;

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
    // increments it at its own call site — hardResetCount_ is the one base
    // increments itself (in processResetIfPending()), asymmetrically.
    ++softResetCount_;
  }
  void configureDevice(const msg::MotorConfig& /*config*/) override {
    ++configureDeviceCalls;
  }

 private:
  float mockPosition_ = 0.0f;
  float mockVelocity_ = 0.0f;
  float stagedDuty_ = 0.0f;
  float lastWrittenDuty_ = 0.0f;
};

// Ship-default config (both armor fields left unset — Hal::Motor::configure()
// substitutes kDefaultReversalDwell=100ms / kDefaultOutputDeadband=0.03).
msg::MotorConfig defaultArmorConfig() { return msg::MotorConfig{}; }

// --- Scenarios --------------------------------------------------------

// 1. A commanded sign change writes 0 immediately, suppresses further
//    non-zero writes until reversalDwell_ has elapsed, then forwards the new
//    direction.
void scenarioHotSignFlip() {
  beginScenario("hot sign flip: immediate 0, dwell suppression, then resume");
  MockMotor m;
  m.configure(defaultArmorConfig());   // reversalDwell_=100ms, outputDeadband_=0.03

  m.requestDuty(0.5f, 1000);    // no prior direction — forwarded immediately
  m.requestDuty(-0.5f, 1010);   // sign flip — write 0, arm the 100ms dwell
  m.requestDuty(-0.5f, 1050);   // still inside the dwell — suppressed to 0
  m.requestDuty(-0.5f, 1109);   // still inside the dwell (1109 < 1110)
  m.requestDuty(-0.5f, 1110);   // dwell elapsed — new direction forwarded

  checkVecEq(m.writeRawDutyCalls, {0.5f, 0.0f, 0.0f, 0.0f, -0.5f},
             "write-call sequence across the reversal");
}

// 2. A sub-outputDeadband_ duty request writes 0, not a tiny signed value —
//    including rapid sub-threshold sign dithering around zero, which must
//    never arm a reversal dwell (proven by an immediate, unsuppressed write
//    of a later legitimate command).
void scenarioDeadbandSuppression() {
  beginScenario("deadband suppresses sub-threshold duty and dither flips");
  MockMotor m;
  m.configure(defaultArmorConfig());   // outputDeadband_=0.03

  m.requestDuty(0.01f, 2000);    // below deadband — writes 0, not 0.01
  m.requestDuty(-0.02f, 2001);   // below deadband, opposite sign — still 0
  m.requestDuty(0.02f, 2002);    // dithering back — still 0

  checkVecEq(m.writeRawDutyCalls, {0.0f, 0.0f, 0.0f},
             "sub-deadband requests all wrote 0");

  // If the dither above had incorrectly armed a reversal dwell, this next
  // legitimate command would be suppressed to 0 instead of forwarded
  // immediately (lastRequestedDuty_ was never set nonzero by the dither, so
  // there is no direction to be "reversing" from).
  m.requestDuty(0.5f, 2010);
  checkFloatEq(m.writeRawDutyCalls.back(), 0.5f,
               "post-dither command forwarded immediately (no phantom dwell)");
}

// 3. A commanded stop (duty == 0) is written immediately even mid-dwell, and
//    cancels the dwell in progress (the very next nonzero command is not
//    still gated behind the old deadline).
void scenarioStopCancelsDwell() {
  beginScenario("stop is always immediate and cancels an in-progress dwell");
  MockMotor m;
  m.configure(defaultArmorConfig());   // reversalDwell_=100ms

  m.requestDuty(0.5f, 1000);    // establish a direction
  m.requestDuty(-0.5f, 1010);   // sign flip — arms dwell, dwellDeadline=1110
  m.requestDuty(0.0f, 1020);    // explicit stop mid-dwell — immediate, cancels

  checkFloatEq(m.writeRawDutyCalls.back(), 0.0f, "stop wrote 0 immediately");

  // Well before the original 1110 deadline: if the stop had not cancelled
  // the dwell, this would still be suppressed to 0. Because lastRequestedDuty_
  // is now 0 (no direction to reverse from), it is forwarded immediately.
  m.requestDuty(-0.5f, 1030);
  checkFloatEq(m.writeRawDutyCalls.back(), -0.5f,
               "post-stop command forwarded immediately, dwell was cancelled");
}

// 4. reversalDwell_ == 0 (explicit legacy/A-B configuration) skips the dwell
//    transition entirely — a sign flip falls straight through to an
//    immediate write, reproducing sprint-077's shipped behavior.
void scenarioLegacyZeroDwell() {
  beginScenario("reversalDwell_ == 0 reproduces legacy immediate-flip behavior");
  MockMotor m;
  msg::MotorConfig cfg = defaultArmorConfig();
  cfg.setReversalDwell(0.0f);   // explicit zero, not "unset" (Design Rationale 2)
  m.configure(cfg);

  m.requestDuty(0.5f, 3000);
  m.requestDuty(-0.5f, 3010);   // sign flip, but dwell disabled — immediate

  checkVecEq(m.writeRawDutyCalls, {0.5f, -0.5f},
             "no intermediate 0 write with dwell disabled");
}

// 5. resetPosition() while moving (lastRequestedDuty_ != 0, so restTicks_
//    never accumulates to kRestTicksRequired) dispatches softRebaseline(),
//    never hardReset().
void scenarioMidMotionResetIsSoft() {
  beginScenario("resetPosition() while moving dispatches softRebaseline() only");
  MockMotor m;
  m.configure(defaultArmorConfig());
  m.setMockVelocity(80.0f);   // well above kRestVelocity — also moving

  m.setDutyCycle(0.5f);       // above deadband — keeps lastRequestedDuty_ != 0
  uint32_t now = 4000;
  m.tick(now);
  now += 20;
  m.tick(now);   // two ticks of motion — restTicks_ stays 0 throughout

  m.resetPosition();          // stages resetPending_ = true
  now += 20;
  m.tick(now);                 // processResetIfPending() dispatches here

  checkUintEq(static_cast<uint32_t>(m.softRebaselineCalls), 1,
              "softRebaseline() called exactly once");
  checkUintEq(static_cast<uint32_t>(m.hardResetCalls), 0,
              "hardReset() never called while moving");
  checkUintEq(m.softResetCount(), 1, "base softResetCount() reflects the call");
  checkUintEq(m.hardResetCount(), 0, "base hardResetCount() stays 0");
}

// 6. resetPosition() while genuinely at rest (restTicks_ >=
//    kRestTicksRequired) dispatches hardReset().
void scenarioStandstillResetIsHard() {
  beginScenario("resetPosition() at verified standstill dispatches hardReset()");
  MockMotor m;
  m.configure(defaultArmorConfig());
  m.setMockVelocity(0.0f);   // below kRestVelocity throughout
  // stagedDuty_ defaults to 0 — never commanded to move.

  uint32_t now = 5000;
  for (int i = 0; i < 8; ++i) {   // well past kRestTicksRequired (proposed: 5)
    m.tick(now);
    now += 20;
  }

  m.resetPosition();
  m.tick(now);   // processResetIfPending() dispatches here

  checkUintEq(static_cast<uint32_t>(m.hardResetCalls), 1,
              "hardReset() called exactly once");
  checkUintEq(static_cast<uint32_t>(m.softRebaselineCalls), 0,
              "softRebaseline() never called at verified standstill");
  checkUintEq(m.hardResetCount(), 1, "base hardResetCount() reflects the call");
  checkUintEq(m.softResetCount(), 0, "base softResetCount() stays 0");
}

// 7. wedged() is the raw, unconditional stuck-encoder latch; wedgeSuspect()
//    is the same test additionally gated on |appliedDuty()| > outputDeadband_
//    — an idle parked motor with a frozen position never reports suspect.
void scenarioWedgeUnconditionalVsSuspect() {
  beginScenario("wedged() unconditional vs wedgeSuspect() motion-qualified");

  // (a) Idle parked motor: frozen position, zero applied duty throughout.
  {
    MockMotor idle;
    idle.configure(defaultArmorConfig());
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
    MockMotor moving;
    moving.configure(defaultArmorConfig());
    moving.setMockPosition(100.0f);   // still never changes — genuinely stuck
    moving.setMockVelocity(0.0f);
    moving.setDutyCycle(0.5f);        // above deadband every tick
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

// 8. PID-independence: the write gate's decision depends only on the
//    (duty, now) pair handed to armoredWrite(), never on which upstream call
//    path produced that duty. Drives the identical reversal/deadband script
//    two ways — once through tick()'s DUTY-mode dispatch (as setDutyCycle()
//    would stage it) and once through a direct requestDuty() call (standing
//    in for e.g. a VELOCITY-mode PID's per-tick output) — and asserts the
//    resulting writeRawDuty() sequences are identical.
void scenarioPidIndependence() {
  beginScenario("policy is independent of the requested-duty's origin");

  MockMotor viaTickDispatch;
  viaTickDispatch.configure(defaultArmorConfig());
  uint32_t now = 8000;
  viaTickDispatch.setDutyCycle(0.5f);
  viaTickDispatch.tick(now);
  now += 10;
  viaTickDispatch.setDutyCycle(-0.5f);
  viaTickDispatch.tick(now);
  now += 40;
  viaTickDispatch.tick(now);   // still mid-dwell — same staged -0.5
  now += 60;
  viaTickDispatch.tick(now);   // dwell elapsed (100ms total) — resumes -0.5

  MockMotor viaDirectRequest;
  viaDirectRequest.configure(defaultArmorConfig());
  now = 8000;
  viaDirectRequest.requestDuty(0.5f, now);
  now += 10;
  viaDirectRequest.requestDuty(-0.5f, now);
  now += 40;
  viaDirectRequest.requestDuty(-0.5f, now);
  now += 60;
  viaDirectRequest.requestDuty(-0.5f, now);

  checkVecEq(viaTickDispatch.writeRawDutyCalls, viaDirectRequest.writeRawDutyCalls,
             "identical write sequence regardless of call path");
}

}  // namespace

int main() {
  scenarioHotSignFlip();
  scenarioDeadbandSuppression();
  scenarioStopCancelsDwell();
  scenarioLegacyZeroDwell();
  scenarioMidMotionResetIsSoft();
  scenarioStandstillResetIsHard();
  scenarioWedgeUnconditionalVsSuspect();
  scenarioPidIndependence();

  if (g_failureCount == 0) {
    std::printf("OK: all motor policy scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the motor policy scenarios\n",
              g_failureCount);
  return 1;
}
