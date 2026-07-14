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
// 099-003 addition: Decision 9's HOST_BUILD-fake abstinence covered the
// ORIGINAL (078) armor-policy scenarios only — a scripted I2CBus fake and
// the real Hal::NezhaMotor/Hal::SimMotor leaves now exist and are already
// exercised this way elsewhere (nezha_flipflop_harness.cpp's own
// scenarioFwdSignNegatesEncoderPositionSign(), which this ticket's Nezha
// scenario below mirrors: a bare leaf constructed directly on its own
// scripted bus, bypassing Subsystems::NezhaHardware/SimHardware entirely).
// Hal::Motor::trackAcceleration() (source/hal/capability/motor.h) is base
// policy that only ever runs from inside a REAL leaf's own tick() — a
// MockMotor never calls it — so proving it "responds plausibly" per the
// ticket's acceptance criteria requires the real leaves, not another
// MockMotor scenario. See scenarioNezhaAccelerationTracksVelocityRamp()/
// scenarioSimAccelerationTracksVelocityRamp() below, and
// test_motor_policy.py's compile command, which now links in
// com/i2c_bus_host.cpp, hal/nezha/nezha_motor.cpp, and hal/sim/sim_motor.cpp
// under -DHOST_BUILD alongside this file.
//
// messages/common.h documents its own target as "CODAL C++11"; this harness
// is compiled to the same standard (see test_motor_policy.py's compile
// command) so it exercises exactly the language subset the firmware itself
// uses.
//
// Plain C++ program, hand-rolled assertions (a dozen scenarios do not
// warrant a test-framework dependency) — prints a PASS/FAIL line per
// scenario and exits nonzero if any assertion failed. Run by the pytest
// wrapper in test_motor_policy.py, which compiles and runs this binary via
// subprocess and asserts exit code 0.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "hal/capability/motor.h"
#include "hal/sim/sim_motor.h"
#include "hal/velocity_pid.h"
#include "messages/common.h"
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

  // 086-002: drives ONE real Hal::MotorVelocityPid::compute() call, then
  // forwards its output through armoredWrite() — mirrors NezhaMotor::
  // tick()'s/SimMotor::tick()'s own Mode::VELOCITY dispatch (feedforward
  // omitted; neither invariant scenario below needs it) closely enough to
  // exercise the REAL control law the 086-002 fix lives in, together with
  // the REAL (unmodified) armor gate, rather than a synthetic duty
  // standing in for either. Returns the raw compute() output (before
  // armoredWrite()'s own deadband/dwell gating) so a scenario can inspect
  // both the control law's own behavior and what actually reached
  // writeRawDutyCalls.
  float driveVelocityTick(float target, float measured, float dt,
                           const msg::Gains& gains, float minDuty,
                           uint32_t now) {
    float duty = pid_.compute(target, measured, dt, gains, minDuty);
    armoredWrite(duty, now);
    return duty;
  }

  // Test-only peek at the base's own dwell-in-progress flag — lets a
  // scenario assert "no dwell left armed" without re-deriving it from the
  // write sequence.
  bool isDwelling() const { return dwelling_; }

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
  Hal::MotorVelocityPid pid_;   // 086-002: backs driveVelocityTick()
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

msg::Gains makeGains(float kp, float ki, float kff, float i_max, float kaw) {
  msg::Gains gains;
  gains.kp = kp;
  gains.ki = ki;
  gains.kff = kff;
  gains.i_max = i_max;
  gains.kaw = kaw;
  return gains;
}

// 9. Invariant A (086-002, architecture-update.md Design Rationale 1): a
//    genuine unrequested reversal — e.g. a stale/glitched command re-
//    issuing the opposite sign with NO intervening commanded decel-to-zero
//    — is still caught and dwelled by armoredWrite()'s reversalDwell_
//    EXACTLY as before the 086-002 fix. Driven through the REAL
//    Hal::MotorVelocityPid (not a synthetic duty, unlike
//    scenarioHotSignFlip/scenarioPidIndependence above) so this proves the
//    fix — which lives entirely in compute()'s deadband-entry handling —
//    does not weaken this path. |target|=150 stays far above minDuty=20 on
//    BOTH sides of the flip, so the fix's deadband-entry reset never
//    engages here (wasInDeadband_ stays false throughout): the write
//    sequence below must match scenarioHotSignFlip's own proven shape
//    (immediate 0, suppressed through the dwell, then the new direction) —
//    not a coincidence, but the direct consequence of the fix's own scope
//    being confined to the deadband boundary.
void scenarioInvariantAGenuineReversalStillDwells() {
  beginScenario("Invariant A: a genuine unrequested reversal (no decel) still dwells identically");

  MockMotor m;
  m.configure(defaultArmorConfig());   // reversalDwell_=100ms, outputDeadband_=0.03

  msg::Gains gains = makeGains(/*kp=*/0.01f, /*ki=*/0.0f, /*kff=*/0.0f,
                                /*i_max=*/1.0f, /*kaw=*/0.0f);
  const float minDuty = 20.0f;   // representative bench-tuned stiction floor
  const float dt = 0.024f;       // [s]
  uint32_t now = 20000;

  // Established direction: converging toward +150, well outside the
  // deadband — err=50 -> duty=kp*50=0.5, forwarded (no prior direction).
  m.driveVelocityTick(150.0f, 100.0f, dt, gains, minDuty, now);
  now += 10;

  // A FRESH command re-issues the opposite sign immediately — no ramp, no
  // intervening decel-to-zero tick. measured hasn't had time to react
  // (still 100, same as the instant before) — err=-250 -> duty clamps to
  // -1.0 — a large-magnitude, genuine direction reversal.
  m.driveVelocityTick(-150.0f, 100.0f, dt, gains, minDuty, now);   // sign flip -> dwell armed, deadline = now+100
  now += 40;
  m.driveVelocityTick(-150.0f, 100.0f, dt, gains, minDuty, now);   // still mid-dwell
  now += 59;
  m.driveVelocityTick(-150.0f, 100.0f, dt, gains, minDuty, now);   // still mid-dwell (1ms shy of the deadline)
  now += 1;
  m.driveVelocityTick(-150.0f, 100.0f, dt, gains, minDuty, now);   // dwell elapsed -- resumes -1.0

  checkVecEq(m.writeRawDutyCalls, {0.5f, 0.0f, 0.0f, 0.0f, -1.0f},
             "genuine reversal via VELOCITY-mode PID dwells identically to a raw duty flip");
}

// 10. Invariant B (086-002): a commanded decel-to-zero does NOT produce a
//     sustained, growing reverse-sign correction. Reproduces, in miniature,
//     the 086 issue's own captured shape (architecture-update.md Grounding
//     fact 2): a sustained turn whose feedforward overshoots the target
//     (building a modest positive integrator trim), followed by the target
//     dropping straight to 0 while the wheel is still coasting down and
//     mildly overshoots PAST zero before settling — exactly the
//     "EVT done RT ... wheel still spinning ... coasts through zero into a
//     reverse-sign residual" shape the issue measured. Pre-086-002 (the
//     literal `spAbs < minDuty` condition with the sim's own unconfigured
//     minDuty=0.0), the freeze branch is dead code (0.0 < 0.0 is always
//     false) and the integrator carries its turn-sustaining bias straight
//     through the stop, still trimming the SAME direction well after the
//     wheel has reversed sign (confirmed against this ticket's own
//     pre-fix instrumentation — see the ticket's completion notes for the
//     captured numbers). Post-086-002, entering the deadband (`<=`, so an
//     exact target==0.0 counts even when minDuty==0.0) resets the stale
//     bias at the FIRST tick target reaches 0, so the correction shrinks
//     rather than grows.
void scenarioInvariantBDecelToZeroNoSustainedResidual() {
  beginScenario("Invariant B: commanded decel-to-zero settles without a sustained residual");

  MockMotor m;
  m.configure(defaultArmorConfig());   // reversalDwell_=100ms, outputDeadband_=0.03

  msg::Gains gains = makeGains(/*kp=*/0.002f, /*ki=*/0.002f, /*kff=*/0.0f,
                                /*i_max=*/1.0f, /*kaw=*/0.0f);
  const float minDuty = 0.0f;   // matches the sim's own unconfigured default — the
                                 // exact config the 086 issue's own regression
                                 // tests (tests/sim/unit/test_motion_overshoot_
                                 // regression.py) run against.
  const float dt = 0.024f;      // [s] matches the sim's ~24ms tick convention
  uint32_t now = 30000;

  // Phase 1 — sustain a turn: target holds at -100 while the plant
  // overshoots to -120 (the same feedforward-overshoot shape the real RT
  // 9000 trace shows), building a modest positive integrator trim over 10
  // ticks, well outside the deadband (|target|=100 >> minDuty=0).
  for (int i = 0; i < 10; ++i) {
    m.driveVelocityTick(-100.0f, -120.0f, dt, gains, minDuty, now);
    now += 24;
  }
  checkTrue(m.writeRawDutyCalls.size() == 10, "10 writes recorded across the sustain phase");
  checkTrue(m.writeRawDutyCalls.back() > 0.0f,
            "sustain-phase duty is a consistent positive trim (builds the bias this ticket must not let leak through)");

  // Phase 2 — commanded decel-to-zero: target drops straight to 0 (the
  // Planner's SMOOTH ramp landing on its final value) while the wheel is
  // still coasting down from the turn and mildly overshoots past zero
  // before settling (-120 -> ... -> a small positive overshoot -> 0).
  const float measuredTrace[] = {-120.0f, -60.0f, -20.0f, -5.0f,
                                   2.0f,     5.0f,   3.0f,   1.0f, 0.0f};
  std::vector<float> postStopDuties;
  for (float measured : measuredTrace) {
    postStopDuties.push_back(m.driveVelocityTick(0.0f, measured, dt, gains, minDuty, now));
    now += 24;
  }

  // Invariant B, part 1: the correction that "finally lands" must not be
  // LARGER than the one computed the instant the stop began — the 086
  // bug's own signature (a stale, frozen/slowly-decaying integrator
  // letting the ever-growing coast error swing an oversized correction).
  // Every subsequent post-stop compute() output stays within the very
  // first post-stop tick's own magnitude.
  float firstPostStop = std::fabs(postStopDuties.front());
  checkTrue(firstPostStop > 0.0f, "first post-stop correction is a real, nonzero braking value");
  for (float d : postStopDuties) {
    checkTrue(std::fabs(d) <= firstPostStop + 1e-6f,
              "no post-stop correction exceeds the first post-stop correction's own magnitude");
  }

  // Invariant B, part 2: settles to a near-zero correction once the wheel
  // has visibly overshot past zero (the last three scripted measured
  // samples: +5, +3, +1 -> 0) — no sustained reverse-sign residual.
  for (size_t i = postStopDuties.size() - 3; i < postStopDuties.size(); ++i) {
    checkTrue(std::fabs(postStopDuties[i]) < 0.02f,
              "settles to a near-zero correction once past the zero-crossing overshoot");
  }

  // Invariant B, part 3: the actual WRITE sequence (post armoredWrite()'s
  // own, UNCHANGED deadband/dwell gate) never applies a negative (reverse)
  // duty to the wheel during this decel-to-zero tail, and never leaves a
  // dwell armed — the wheel eases off toward zero, it is never driven
  // further into the overshoot.
  for (size_t i = m.writeRawDutyCalls.size() - postStopDuties.size(); i < m.writeRawDutyCalls.size(); ++i) {
    checkTrue(m.writeRawDutyCalls[i] >= 0.0f,
              "no post-stop WRITE ever reverses sign (the wheel is never driven further into overshoot)");
  }
  checkTrue(!m.isDwelling(), "no reversal dwell left armed after settling");
}

// --- 099-003: acceleration-EMA scenarios against the REAL leaves ----------
//
// Hal::Motor::trackAcceleration() is only ever called from inside a real
// leaf's own tick() (NezhaMotor::tick()/SimMotor::tick() — see
// architecture-update.md Decision 3), so proving it "responds plausibly
// (correct sign, settles toward zero) to a velocity ramp up/down/hold
// sequence" per the ticket's acceptance criteria needs those real leaves,
// not another MockMotor scenario (which never reaches trackAcceleration()
// at all).

// 11. Hal::Motor::trackAcceleration(), exercised against the REAL
//     Hal::SimMotor leaf: the standalone constructor (no PhysicsWorld/plant
//     needed — mirrors this file's Nezha scenario's own "bare leaf, no
//     surrounding subsystem" shape). Drives the SAME ramp UP / HOLD / ramp
//     DOWN shape, but through SimMotor's own DUTY-mode dispatch (staging an
//     increasing/holding/decreasing duty target each tick) rather than
//     scripted encoder bytes: SimMotor's standalone integrator applies the
//     just-staged duty's velocity with a one-tick latency (sim_motor.cpp's
//     tick() step-5 comment), so a smoothly ramped duty target produces a
//     smoothly ramped SAMPLED velocity, exactly like the Nezha scenario's
//     scripted position ramp does. Every duty value stays positive and
//     above the default 0.03 output deadband, so no reversal dwell ever
//     engages.
void scenarioSimAccelerationTracksVelocityRamp() {
  beginScenario("099-003: SimMotor::trackAcceleration() responds to a velocity ramp (up/down/hold)");

  msg::MotorConfig cfg = msg::MotorConfig{}.setVelFiltAlpha(1.0f);
  Hal::SimMotor motor(cfg);   // standalone -- no plant

  uint32_t now = 0;
  motor.setDutyCycle(0.0f);
  motor.tick(now);   // prime tick -- no velocity/accel yet

  // Ramp UP: duty climbs 0.10 -> 0.80.
  const float rampUpDuty[] = {0.10f, 0.20f, 0.30f, 0.40f, 0.50f, 0.60f, 0.70f, 0.80f};
  for (float d : rampUpDuty) {
    now += 20;
    motor.setDutyCycle(d);
    motor.tick(now);
  }
  const float afterRampUp = motor.acceleration();
  checkTrue(afterRampUp > 0.0f,
            "SimMotor: acceleration() is positive after a sustained velocity ramp UP");

  // HOLD at the ramp's final constant duty -- constant velocity, EMA decays.
  for (int i = 0; i < 8; ++i) {
    now += 20;
    motor.setDutyCycle(0.80f);
    motor.tick(now);
  }
  const float afterHold = motor.acceleration();
  checkTrue(std::fabs(afterHold) < std::fabs(afterRampUp),
            "SimMotor: acceleration() magnitude shrinks toward zero while velocity holds steady");

  // Ramp DOWN: duty falls 0.70 -> 0.10.
  const float rampDownDuty[] = {0.70f, 0.60f, 0.50f, 0.40f, 0.30f, 0.20f, 0.10f};
  for (float d : rampDownDuty) {
    now += 20;
    motor.setDutyCycle(d);
    motor.tick(now);
  }
  const float afterRampDown = motor.acceleration();
  checkTrue(afterRampDown < 0.0f,
            "SimMotor: acceleration() is negative after a sustained velocity ramp DOWN");
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
  scenarioInvariantAGenuineReversalStillDwells();
  scenarioInvariantBDecelToZeroNoSustainedResidual();
  scenarioSimAccelerationTracksVelocityRamp();

  if (g_failureCount == 0) {
    std::printf("OK: all motor policy scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the motor policy scenarios\n",
              g_failureCount);
  return 1;
}
