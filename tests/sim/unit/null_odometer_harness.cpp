// null_odometer_harness.cpp -- off-hardware acceptance harness for ticket
// 090-003 (SUC-003): proves (1) Hal::NullOdometer (source/hal/capability/
// null_odometer.h) behaves inertly per its own file header's contract, and
// (2) Subsystems::Hardware::odometer()'s BASE-CLASS default (source/
// subsystems/hardware.h) now returns a non-null, shared NullOdometer
// instance rather than nullptr.
//
// Point (2) needs an owner that does NOT override odometer() -- neither
// existing concrete owner qualifies (Subsystems::NezhaHardware since ticket
// 086-006, Subsystems::SimHardware since ticket 081-003 both override it to
// their own real device) -- so this file declares a minimal local
// StubHardware/StubMotor pair, implementing only the pure virtuals
// Subsystems::Hardware/Hal::Motor require, and deliberately leaves
// odometer() untouched to exercise the inherited base default. Mirrors
// runtime_blackboard_harness.cpp's dependency-free shape (only
// <cstdint>/messages/*.h/subsystems/hardware.h -- no
// MicroBit.h, no I2CBus, no CMake, no ARM toolchain): headers-only, no
// additional .cpp compiled in. Hand-rolled assertions, prints PASS/FAIL,
// exits nonzero on any failure. Run by test_null_odometer.py, which compiles
// and runs this binary via subprocess.

#include <cstdint>
#include <cstdio>
#include <string>

#include "messages/common.h"
#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "subsystems/hardware.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors runtime_blackboard_harness.cpp
// / hardware_seam_harness.cpp) ---

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

void checkFalse(bool condition, const std::string& what) {
  if (condition) fail(what + " -- expected false, got true");
}

void checkFloatEq(float actual, float expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual));
    fail(buf);
  }
}

// --- StubMotor/StubHardware: minimal Subsystems::Hardware owner that
// implements only the pure virtuals, and deliberately does NOT override
// odometer() -- the one thing this file exists to exercise. ---

class StubMotor : public Hal::Motor {
 public:
  void setDutyCycle(float) override {}
  void setVoltage(float) override {}
  void setVelocity(float) override {}
  void setPosition(float) override {}
  void setNeutral(msg::Neutral) override {}
  void setFeedforward(float) override {}
  float position() const override { return 0.0f; }
  float velocity() const override { return 0.0f; }
  float appliedDuty() const override { return 0.0f; }
  bool connected() const override { return false; }
  void tick(uint32_t) override {}
  msg::MotorCapabilities capabilities() const override { return msg::MotorCapabilities{}; }

 protected:
  void writeRawDuty(float) override {}
  void hardReset() override {}
  void softRebaseline() override {}
  void configureDevice(const msg::MotorConfig&) override {}
};

class StubHardware : public Subsystems::Hardware {
 public:
  Hal::Motor& motor(uint32_t) override { return motor_; }
  void tick(uint32_t) override {}
  void apply(const Hal::CommandProcessorToHardwareCommand&) override {}
  void apply(const Hal::DrivetrainToHardwareCommand&) override {}
  msg::MotorConfig motorConfig(uint32_t) const override { return msg::MotorConfig{}; }
  msg::MotorState motorState(uint32_t) const override { return msg::MotorState{}; }
  // odometer() deliberately NOT overridden here -- this is the whole point:
  // exercise Subsystems::Hardware's own inherited base default.

 private:
  StubMotor motor_;
};

// --- Scenarios --------------------------------------------------------

// 1. Subsystems::Hardware::odometer()'s base-class default, reached through
// an owner that supplies no override of its own, is non-null and returns
// the SAME shared instance on every call (no per-call allocation).
void scenarioBaseDefaultOdometerIsNonNullAndShared() {
  beginScenario("Subsystems::Hardware base-class odometer() default: non-null, shared instance");
  StubHardware hardware;
  Subsystems::Hardware& base = hardware;   // held only through the abstract base, like main_loop.cpp

  Hal::Odometer* first = base.odometer();
  checkTrue(first != nullptr, "base-class odometer() default is non-null (090-003 Hal::NullOdometer)");

  Hal::Odometer* second = base.odometer();
  checkTrue(first == second, "odometer() returns the SAME shared instance across calls -- no per-call allocation");
}

// 2. Hal::NullOdometer's own inert contract, exercised through the
// Hal::Odometer base pointer the way every real caller (main_loop.cpp,
// configurator.cpp) holds it.
void scenarioNullOdometerIsInert() {
  beginScenario("Hal::NullOdometer: every primitive is an inert no-op/discard");
  StubHardware hardware;
  Hal::Odometer* odometer = hardware.odometer();

  checkFalse(odometer->connected(), "NullOdometer::connected() is always false");
  checkFalse(odometer->fusableThisPass(), "NullOdometer::fusableThisPass() is always false");

  msg::PoseEstimate pose = odometer->pose();
  checkFloatEq(pose.pose.x, 0.0f, "NullOdometer::pose() x is identity/zero");
  checkFloatEq(pose.pose.y, 0.0f, "NullOdometer::pose() y is identity/zero");
  checkFloatEq(pose.pose.h, 0.0f, "NullOdometer::pose() h is identity/zero");
  checkFalse(pose.stamp.valid, "NullOdometer::pose()'s stamp is not valid -- not a fresh sample");

  // Every primitive is callable and must not crash; none has an observable
  // side effect worth asserting on beyond "did not throw/crash" and
  // fusableThisPass() staying false afterward (proving apply()'s shared
  // reset-flag bookkeeping in the BASE class, if it ran at all, has zero
  // effect on this override).
  odometer->tick(1000);
  odometer->init();
  odometer->resetTracking();
  odometer->setPose(msg::Pose2D());
  odometer->setLinearScalar(1.5f);
  odometer->setAngularScalar(0.9f);
  checkFalse(odometer->fusableThisPass(),
             "NullOdometer stays never-fusable even after every primitive is exercised");

  // apply()/applySetPose() are the base class's own concrete dispatch
  // (odometer.h) -- route through them too, the same way main_loop.cpp's
  // drain sites do, confirming the whole message plane is inert end to end.
  msg::OdometerCommand cmd;
  cmd.setZero(true);
  odometer->apply(cmd);
  checkFalse(odometer->fusableThisPass(),
             "NullOdometer::apply(OdometerCommand) leaves fusableThisPass() false (override ignores the "
             "base's resetAppliedThisPass_ bookkeeping entirely)");

  msg::SetPose setPose;
  odometer->applySetPose(setPose);
  checkFalse(odometer->fusableThisPass(), "NullOdometer::applySetPose() leaves fusableThisPass() false too");
}

}  // namespace

int main() {
  scenarioBaseDefaultOdometerIsNonNullAndShared();
  scenarioNullOdometerIsInert();

  if (g_failureCount == 0) {
    std::printf("OK: Hal::NullOdometer is inert and Subsystems::Hardware's base odometer() default is non-null\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the NullOdometer scenarios\n", g_failureCount);
  return 1;
}
