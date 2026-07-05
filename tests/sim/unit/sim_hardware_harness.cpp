// sim_hardware_harness.cpp — off-hardware acceptance harness for ticket
// 081-003: proves (a) Subsystems::SimHardware's dt=0 re-entry guard
// (architecture-update.md (081) Decision 4) is real, not just declared; (b)
// the zero-error determinism gate (true encoder == reported encoder == OTOS
// accumulator, bit-for-bit, with every error knob at its zero default); and
// (c) Hal::SimMotor's VELOCITY mode genuinely calls Hal::MotorVelocityPid —
// ticket 081-001's exact shared class, not a re-derived approximation — by
// mirroring its own call sequence against an independent MotorVelocityPid
// instance fed the identical inputs.
//
// Same convention as the existing tests/sim/unit/*_harness.cpp files (ad hoc
// compile, no CMake yet — 078's Decision 9 precedent, reused here per this
// ticket's own acceptance criteria): hand-rolled assertions, PASS/FAIL per
// scenario, nonzero exit on any failure. Compiled by test_sim_hardware.py
// together with the real source/hal/sim/*.cpp, source/hal/velocity_pid.cpp,
// and source/subsystems/sim_hardware.cpp, with -DHOST_BUILD.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "hal/sim/physics_world.h"
#include "hal/sim/sim_motor.h"
#include "hal/velocity_pid.h"
#include "messages/common.h"
#include "messages/motor.h"
#include "subsystems/hardware.h"
#include "subsystems/sim_hardware.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors velocity_pid_harness.cpp /
// hardware_seam_harness.cpp) ---

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

// Asserts |actual - expected| <= tol. tol == 0.0f is an exact (bit-for-bit,
// modulo IEEE754 +/-0) equality check — used throughout the zero-error
// determinism gate and the dt=0 re-entry guard, both of which are meant to
// hold EXACTLY, not approximately (see the scenario comments below for why
// that is achievable without floating-point luck).
void checkNear(float actual, float expected, float tol, const std::string& what) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s — expected %.9g (tol %.3g), got %.9g",
                  what.c_str(), static_cast<double>(expected),
                  static_cast<double>(tol), static_cast<double>(actual));
    fail(buf);
  }
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

// Default configs for a 4-port Subsystems::SimHardware: fwd_sign=1,
// travel_calib=1.0, no dwell/deadband (so armoredWrite() forwards the
// commanded duty unchanged — see scenario comments for why this matters).
void fillDefaultConfigs(msg::MotorConfig configs[Subsystems::Hardware::kPortCount],
                         const msg::Gains& gains, float minDuty) {
  for (uint32_t i = 0; i < Subsystems::Hardware::kPortCount; ++i) {
    configs[i] = msg::MotorConfig{};
    configs[i].setPort(i + 1)
        .setFwdSign(1)
        .setTravelCalib(1.0f)
        .setVelFiltAlpha(1.0f)
        .setOutputDeadband(0.0f)
        .setReversalDwell(0.0f);
    configs[i].vel_gains = gains;
    configs[i].min_duty = minDuty;
  }
}

// --- Scenarios --------------------------------------------------------

// (a) The dt=0 re-entry guard (architecture-update.md (081) Decision 4):
// devLoopTick()'s ordinary two-slice `hardware.tick(now)` call feeds
// Subsystems::SimHardware the SAME `now` twice, every pass. A double-tick
// with an unchanged `now` must be a COMPLETE no-op — proven here two ways:
// (1) direct before/after comparison across the re-entrant call, and (2) a
// stronger "control" comparison against a second instance ticked only ONCE,
// which the double-ticked instance must match exactly — a double-
// integration bug that happened not to move any observable value in test
// (1) would still be caught by test (2), since the control never saw the
// second call at all.
void scenarioDtZeroReentryGuard() {
  beginScenario("Subsystems::SimHardware::tick(now) at an unchanged now is a complete no-op (Decision 4)");

  msg::Gains gains = makeGains(/*kp=*/0.01f, /*ki=*/0.05f, /*kff=*/0.002f,
                                /*i_max=*/1.0f, /*kaw=*/2.0f);
  msg::MotorConfig configs[Subsystems::Hardware::kPortCount];
  fillDefaultConfigs(configs, gains, /*minDuty=*/0.0f);

  // Instance A: ticked once at now=1000, then ticked AGAIN at the SAME
  // now=1000 — exactly devLoopTick()'s two-slice pattern.
  Subsystems::SimHardware hwA(configs);
  hwA.motor(1).apply(msg::MotorCommand{}.setVelocity(300.0f));
  hwA.tick(1000);
  float dutyAfterFirst = hwA.motor(1).appliedDuty();
  float posAfterFirst  = hwA.motor(1).position();
  float velAfterFirst  = hwA.motor(1).velocity();

  hwA.tick(1000);   // re-entrant, unchanged now -- must be a complete no-op

  checkNear(hwA.motor(1).appliedDuty(), dutyAfterFirst, 0.0f,
            "appliedDuty unchanged across the re-entrant same-now call");
  checkNear(hwA.motor(1).position(), posAfterFirst, 0.0f,
            "position unchanged across the re-entrant same-now call");
  checkNear(hwA.motor(1).velocity(), velAfterFirst, 0.0f,
            "velocity unchanged across the re-entrant same-now call");

  // Instance B (the control): ticked ONCE only, at now=1000.
  Subsystems::SimHardware hwB(configs);
  hwB.motor(1).apply(msg::MotorCommand{}.setVelocity(300.0f));
  hwB.tick(1000);

  checkNear(hwA.motor(1).appliedDuty(), hwB.motor(1).appliedDuty(), 0.0f,
            "double-ticked instance's duty matches the single-ticked control");
  checkNear(hwA.motor(1).position(), hwB.motor(1).position(), 0.0f,
            "double-ticked instance's position matches the single-ticked control");
  checkNear(hwA.motor(1).velocity(), hwB.motor(1).velocity(), 0.0f,
            "double-ticked instance's velocity matches the single-ticked control");

  // Both instances then advance identically at a genuinely NEW now — they
  // must continue to match exactly, proving the guard left no stray
  // internal divergence (e.g. a wrongly-advanced lastAdvancedNow_, or a
  // PID integral that silently drifted apart during the re-entrant call).
  hwA.tick(1024);
  hwB.tick(1024);
  checkNear(hwA.motor(1).appliedDuty(), hwB.motor(1).appliedDuty(), 0.0f,
            "instances still match after a subsequent genuine tick (duty)");
  checkNear(hwA.motor(1).position(), hwB.motor(1).position(), 0.0f,
            "instances still match after a subsequent genuine tick (position)");

  // Direct proof at the plant layer too: the SHARED PhysicsWorld's own
  // accumulators and staged actuator must be untouched by a re-entrant call.
  Subsystems::SimHardware hwC(configs);
  hwC.motor(1).apply(msg::MotorCommand{}.setVelocity(300.0f));
  hwC.tick(2000);
  float trueEncBefore     = hwC.plant().trueEncL();
  float reportedEncBefore = hwC.plant().reportedEncL();
  int8_t pwmBefore        = hwC.plant().pwmL();

  hwC.tick(2000);   // re-entrant

  checkNear(hwC.plant().trueEncL(), trueEncBefore, 0.0f,
            "plant trueEncL unchanged across the re-entrant call — no double PhysicsWorld::update()");
  checkNear(hwC.plant().reportedEncL(), reportedEncBefore, 0.0f,
            "plant reportedEncL unchanged across the re-entrant call");
  checkTrue(hwC.plant().pwmL() == pwmBefore,
            "plant pwmL unchanged across the re-entrant call — no re-issued SimMotor::tick()");
}

// (b) The zero-error determinism gate: with every error knob at its zero
// default, true encoder == reported encoder == OTOS accumulator,
// bit-for-bit, over a scripted sequence of ticks.
//
// Uses deliberately "nice" numbers throughout (50% duty -> exactly 200 mm/s
// off PhysicsWorld::kNominalMaxSpeed == 400; 125 ms steps -> dt_s == 0.125,
// an exact power-of-two binary fraction) so every accumulation below is an
// EXACT float operation (small dyadic-rational magnitudes, always well
// within a 24-bit mantissa's exact-integer range) — the bit-for-bit
// equalities below hold BY CONSTRUCTION, not by floating-point luck or an
// approximate tolerance.
void scenarioZeroErrorDeterminism() {
  beginScenario("zero-error determinism gate: true encoder == reported encoder == OTOS accumulator, bit-for-bit");

  msg::MotorConfig configs[Subsystems::Hardware::kPortCount];
  for (uint32_t i = 0; i < Subsystems::Hardware::kPortCount; ++i) {
    configs[i] = msg::MotorConfig{};
    configs[i].setPort(i + 1).setFwdSign(1).setTravelCalib(1.0f);
  }

  Subsystems::SimHardware hw(configs);
  // Equal DUTY on both plant-bound wheels (ports 1/2, the default binding)
  // -> straight-line drive, heading locked at exactly 0 (physics_world.cpp
  // sub-step B: dTh == ((dR-dL)/trackwidth)*slip == 0 exactly whenever
  // dR == dL bit-for-bit, which holds here since both wheels are commanded
  // the identical duty against identical configs).
  hw.motor(1).apply(msg::MotorCommand{}.setDutyCycle(0.5f));
  hw.motor(2).apply(msg::MotorCommand{}.setDutyCycle(0.5f));

  uint32_t now = 0;
  for (int i = 0; i < 6; ++i) {
    hw.tick(now);
    now += 125;
  }

  checkTrue(hw.plant().trueEncL() == hw.plant().reportedEncL(),
            "true encoder == reported encoder (left), bit-for-bit");
  checkTrue(hw.plant().trueEncR() == hw.plant().reportedEncR(),
            "true encoder == reported encoder (right), bit-for-bit");
  checkTrue(hw.plant().trueEncL() == hw.plant().trueEncR(),
            "left/right true encoders equal (identical commanded duty), bit-for-bit");
  checkTrue(hw.plant().truePoseY() == 0.0f,
            "no lateral drift on a straight-line drive, exactly 0");
  checkTrue(hw.plant().truePoseH() == 0.0f,
            "no heading drift on a straight-line drive, exactly 0");
  checkTrue(hw.plant().truePoseX() == hw.plant().trueEncL(),
            "world-frame X == wheel encoder travel on a heading-0 straight drive, bit-for-bit");
  checkTrue(hw.odometer().odomX() == hw.plant().truePoseX(),
            "OTOS accumulator == plant true pose X, bit-for-bit — the zero-error determinism gate");
  checkTrue(hw.odometer().odomY() == 0.0f, "OTOS Y accumulator stays exactly 0 on a straight drive");
  checkTrue(hw.odometer().odomH() == 0.0f, "OTOS heading accumulator stays exactly 0 on a straight drive");
  checkTrue(hw.plant().trueEncL() > 0.0f, "sanity: the scripted drive actually moved");
}

// (c) Hal::SimMotor's VELOCITY mode genuinely calls Hal::MotorVelocityPid —
// ticket 081-001's exact shared class, not a re-derived approximation — by
// replaying the IDENTICAL (target, measured, dt, gains, minDuty) sequence
// SimMotor feeds its own embedded pid_ into an independent
// Hal::MotorVelocityPid instance, and asserting the two never diverge.
// "measured" is read back from motor.velocity() after each tick() call (the
// same filteredVelocity_ SimMotor's OWN pid_.compute() call just used this
// tick — see sim_motor.cpp's tick()); "dt" mirrors SimMotor's own
// haveElapsed rule (0.0f on the very first tick, the uniform step interval
// thereafter). Both integrators start at zero and are pure functions of
// their inputs, so they should never drift apart even by a rounding hair —
// this is a sanity/wiring proof (per the ticket's own "sanity that sim and
// hardware control loops match" framing), not the bit-for-bit determinism
// gate (that is scenario (b) above).
void scenarioSimMotorVelocityMatchesSharedPid() {
  beginScenario("Hal::SimMotor VELOCITY mode matches an independent Hal::MotorVelocityPid fed the identical sequence");

  msg::Gains gains = makeGains(/*kp=*/0.01f, /*ki=*/0.05f, /*kff=*/0.002f,
                                /*i_max=*/1.0f, /*kaw=*/2.0f);
  msg::MotorConfig config;
  config.setPort(1).setFwdSign(1).setTravelCalib(1.0f).setVelFiltAlpha(1.0f)
      .setOutputDeadband(0.0f).setReversalDwell(0.0f);
  config.vel_gains = gains;
  config.min_duty = 0.0f;

  Hal::PhysicsWorld plant;
  Hal::SimMotor motor(plant, Hal::SimMotor::Side::LEFT, config);
  motor.apply(msg::MotorCommand{}.setVelocity(300.0f));

  Hal::MotorVelocityPid mirrorPid;
  const int kTicks = 40;
  const uint32_t kStepMs = 20;

  uint32_t now = 0;
  for (int i = 0; i < kTicks; ++i) {
    motor.tick(now);

    float measured = motor.velocity();               // this tick's fresh filteredVelocity_
    float dt = (i == 0) ? 0.0f : (static_cast<float>(kStepMs) / 1000.0f);   // mirrors SimMotor's own haveElapsed rule
    float mirrorDuty = mirrorPid.compute(300.0f, measured, dt, gains, 0.0f);

    checkNear(motor.appliedDuty(), mirrorDuty, 1e-5f,
              "SimMotor's appliedDuty matches an independently-computed MotorVelocityPid output");
    checkTrue(std::isfinite(motor.appliedDuty()), "SimMotor's appliedDuty stays finite");

    // Advance the plant AFTER comparing this tick — matches
    // Subsystems::SimHardware's own ordering (every motor ticks, THEN the
    // plant advances once), giving the NEXT tick() call a fresh position
    // sample to compute velocity from.
    plant.update(i == 0 ? 0u : kStepMs);
    now += kStepMs;
  }
}

}  // namespace

int main() {
  scenarioDtZeroReentryGuard();
  scenarioZeroErrorDeterminism();
  scenarioSimMotorVelocityMatchesSharedPid();

  if (g_failureCount == 0) {
    std::printf("OK: all Subsystems::SimHardware / Hal::PhysicsWorld / Hal::SimMotor scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the SimHardware scenarios\n", g_failureCount);
  return 1;
}
