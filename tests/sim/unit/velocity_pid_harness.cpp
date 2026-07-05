// velocity_pid_harness.cpp — off-hardware acceptance harness for ticket
// 081-001 (SUC-001): exercises Hal::MotorVelocityPid::compute() in
// isolation — the control law extracted byte-for-byte out of what used to
// be NezhaMotor::runVelocityPid() (source/hal/nezha/nezha_motor.cpp).
//
// Per motor_policy_harness.cpp's precedent (078-004), this #includes only
// the dependency-free header under test (hal/velocity_pid.h) plus
// messages/common.h (already dependency-free — no MicroBit.h, no I2CBus),
// so it compiles with the plain system C++ compiler — no CMake, no ARM
// toolchain.
//
// messages/common.h documents its own target as "CODAL C++11"; this
// harness is compiled to the same standard (see test_velocity_pid.py's
// compile command) so it exercises exactly the language subset the
// firmware itself uses.
//
// Plain C++ program, hand-rolled assertions (three scenarios do not
// warrant a test-framework dependency) — prints a PASS/FAIL line per
// scenario and exits nonzero if any assertion failed. Run by the pytest
// wrapper in test_velocity_pid.py, which compiles and runs this binary via
// subprocess and asserts exit code 0.

#include <cmath>
#include <cstdio>
#include <string>

#include "hal/velocity_pid.h"
#include "messages/common.h"

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

void checkLe(float actual, float bound, const std::string& what) {
  if (!(actual <= bound)) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s — expected <= %g, got %g",
                  what.c_str(), static_cast<double>(bound),
                  static_cast<double>(actual));
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

// --- Scenarios --------------------------------------------------------

// 1. A velocity step converges toward the target without oscillation
//    blow-up: after enough ticks at a fixed dt, the output settles into a
//    bounded, non-diverging range and the tracking error shrinks compared
//    to the very first tick's error.
void scenarioVelocityStepConverges() {
  beginScenario("velocity step converges without oscillation blow-up");

  Hal::MotorVelocityPid pid;
  msg::Gains gains = makeGains(/*kp=*/0.01f, /*ki=*/0.05f, /*kff=*/0.002f,
                                /*i_max=*/1.0f, /*kaw=*/2.0f);
  const float target = 300.0f;   // [mm/s]
  const float minDuty = 20.0f;   // [mm/s] well below the step target — not in deadband
  const float dt = 0.02f;        // [s]

  // Simple first-order plant stand-in: measured velocity chases the
  // commanded duty with a fixed gain, purely to give compute() a
  // closed-loop error signal to react to across ticks (this harness tests
  // the control law's own convergence behavior, not a calibrated plant).
  float measured = 0.0f;
  float firstError = 0.0f;
  float lastError = 0.0f;
  const int kTicks = 200;
  for (int i = 0; i < kTicks; ++i) {
    float duty = pid.compute(target, measured, dt, gains, minDuty);
    // Output must always stay within the documented clamp domain.
    checkLe(std::fabs(duty), 1.0f, "compute() output stays within [-1,1]");
    measured += (duty * 500.0f - measured) * 0.1f;   // stand-in plant response
    float err = target - measured;
    if (i == 0) firstError = std::fabs(err);
    lastError = std::fabs(err);
  }

  checkTrue(lastError < firstError,
            "tracking error shrinks from the first tick to the last");
  checkLe(lastError, firstError * 0.5f,
          "converged error is well below the initial error (no blow-up)");
}

// 2. Anti-windup clamps the integral under a saturating target: when the
//    commanded target is unreachable (measured never catches up because
//    the plant is held fixed), the integrator's contribution to the output
//    must not grow without bound — the output should sit at (or very near)
//    the +/-1 clamp rather than diverging past it, and the raw integral
//    state itself must stay bounded near +/- i_max.
void scenarioAntiWindupClampsIntegral() {
  beginScenario("anti-windup clamps the integral under a saturating target");

  Hal::MotorVelocityPid pid;
  msg::Gains gains = makeGains(/*kp=*/0.001f, /*ki=*/0.5f, /*kff=*/0.0f,
                                /*i_max=*/0.8f, /*kaw=*/5.0f);
  const float target = 1000.0f;   // [mm/s] deliberately unreachable
  const float measured = 0.0f;    // plant held fixed — error never closes
  const float minDuty = 10.0f;
  const float dt = 0.02f;

  float lastDuty = 0.0f;
  for (int i = 0; i < 500; ++i) {
    lastDuty = pid.compute(target, measured, dt, gains, minDuty);
    checkLe(std::fabs(lastDuty), 1.0f,
            "output stays clamped to [-1,1] under sustained saturation");
  }

  // The output must have driven to the positive rail (a large, persistent
  // positive error with no anti-windup would run away well past any
  // sensible bound; back-calculation keeps kp*err + integral near i_max).
  checkTrue(lastDuty > 0.9f,
            "saturating positive error drives output to the positive rail");
}

// 3. dt<=0 substitutes the nominal loop period rather than dividing by
//    zero or NaN-ing: a zero (and a negative) dt must produce the exact
//    same output as an explicit kNominalDt (~24ms) call against identical
//    integrator state, and the result must be finite.
void scenarioNonPositiveDtFallsBackToNominal() {
  beginScenario("dt<=0 substitutes kNominalDt rather than dividing by zero");

  msg::Gains gains = makeGains(/*kp=*/0.01f, /*ki=*/0.1f, /*kff=*/0.001f,
                                /*i_max=*/1.0f, /*kaw=*/2.0f);
  const float target = 150.0f;
  const float measured = 50.0f;
  const float minDuty = 10.0f;
  const float kNominalDt = 0.024f;   // [s] — mirrors velocity_pid.h's private constant

  // Zero dt.
  {
    Hal::MotorVelocityPid pidZero;
    Hal::MotorVelocityPid pidNominal;
    float outZero = pidZero.compute(target, measured, 0.0f, gains, minDuty);
    float outNominal = pidNominal.compute(target, measured, kNominalDt, gains, minDuty);
    checkTrue(std::isfinite(outZero), "dt=0 output is finite (no NaN/inf)");
    checkTrue(std::fabs(outZero - outNominal) < 1e-6f,
              "dt=0 output matches an explicit kNominalDt call");
  }

  // Negative dt.
  {
    Hal::MotorVelocityPid pidNegative;
    Hal::MotorVelocityPid pidNominal;
    float outNegative = pidNegative.compute(target, measured, -0.01f, gains, minDuty);
    float outNominal = pidNominal.compute(target, measured, kNominalDt, gains, minDuty);
    checkTrue(std::isfinite(outNegative), "dt<0 output is finite (no NaN/inf)");
    checkTrue(std::fabs(outNegative - outNominal) < 1e-6f,
              "dt<0 output matches an explicit kNominalDt call");
  }
}

}  // namespace

int main() {
  scenarioVelocityStepConverges();
  scenarioAntiWindupClampsIntegral();
  scenarioNonPositiveDtFallsBackToNominal();

  if (g_failureCount == 0) {
    std::printf("OK: all velocity PID scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the velocity PID scenarios\n",
              g_failureCount);
  return 1;
}
