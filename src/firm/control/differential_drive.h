// differential_drive.h -- Control:: composes the DiffDrive kernel PACKAGE
// (diffdrive/differential_drive.{h,cpp}) into this firmware. The package
// is the SINGLE implementation of the wheel kernel -- the same two files
// that lift into a MakeCode/PXT package or a MicroPython C module -- and
// what used to be a full copy of it here is now only the glue its README
// promises: one-line forwarding adapters from the firmware's Hal:: seams
// onto the package's own ports, plus the alias that keeps every existing
// Control::DifferentialDrive call site compiling unchanged.
//
// No logic lives in this header. If an adapter method ever grows past one
// line, it is doing work that belongs in the kernel or in the leaf.
//
// Composition (Core::RobotGraph): the graph owns one port object per Hal
// collaborator and hands the ports to the kernel --
//
//   Hal::Motor (MotorArmor) --> Control::MotorPort ---.
//   Hal::Clock             --> Control::ClockPort  ---+--> DiffDrive::
//   Hal::Sleeper           --> Control::SleeperPort---+    DifferentialDrive
//   Hal::FiberLauncher     --> Control::LauncherPort-'
//
// Lifecycle is unchanged from the in-tree kernel it replaces: construct +
// setConfig() at compose (data only), begin() after Preamble::done(),
// start() last -- on the host, start() is never called and the harness
// drives step() directly (Hal::FiberLauncher's host impl fails the test
// if launch() is ever reached; the microbit impl wraps codal
// create_fiber -- platform/microbit/microbit_fiber.h).
#pragma once

#include <cstdint>

#include "diffdrive/differential_drive.h"
#include "hal/clock.h"
#include "hal/fiber.h"
#include "hal/motor.h"

namespace Control {

// The kernel, under the name the firmware has always used. Config /
// Output / Status and every method come from the package; see
// diffdrive/differential_drive.h for the full contract.
using DifferentialDrive = DiffDrive::DifferentialDrive;

// ---- Hal -> package-port forwarding adapters ------------------------

class MotorPort final : public DiffDrive::Motor {
 public:
  explicit MotorPort(Hal::Motor& motor) : motor_(motor) {}

  void begin() override { motor_.begin(); }
  void requestSample() override { motor_.requestSample(); }
  void setDuty(float duty) override { motor_.setDuty(duty); }
  void emergencyStop() override { motor_.emergencyStop(); }
  void tick(uint64_t nowUs) override { motor_.tick(nowUs); }  // [us]

  float position() const override { return motor_.position(); }
  float velocity() const override { return motor_.velocity(); }
  float appliedDuty() const override { return motor_.appliedDuty(); }
  bool connected() const override { return motor_.connected(); }
  uint64_t sampleTime() const override { return motor_.sampleTime(); }
  void rebaseline() override { motor_.rebaseline(); }
  bool wedged() const override { return motor_.wedged(); }
  bool wedgeSuspect() const override { return motor_.wedgeSuspect(); }

 private:
  Hal::Motor& motor_;
};

class ClockPort final : public DiffDrive::Clock {
 public:
  explicit ClockPort(const Hal::Clock& clock) : clock_(clock) {}
  uint64_t nowMicros() const override { return clock_.nowMicros(); }

 private:
  const Hal::Clock& clock_;
};

class SleeperPort final : public DiffDrive::Sleeper {
 public:
  explicit SleeperPort(Hal::Sleeper& sleeper) : sleeper_(sleeper) {}
  void sleepMillis(uint32_t duration) override {  // [ms]
    sleeper_.sleepMillis(duration);
  }
  void yield() override { sleeper_.yield(); }

 private:
  Hal::Sleeper& sleeper_;
};

class LauncherPort final : public DiffDrive::FiberLauncher {
 public:
  explicit LauncherPort(Hal::FiberLauncher& launcher) : launcher_(launcher) {}
  void launch(void (*entry)(void*), void* context) override {
    launcher_.launch(entry, context);
  }

 private:
  Hal::FiberLauncher& launcher_;
};

}  // namespace Control
