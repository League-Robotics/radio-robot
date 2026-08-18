// boot_wiring.h -- Core::RobotGraph / Core::composeRobot(): the ONE shared
// composition root src/firm/main.cpp (ARM) and TestSim::SimHarness (sim,
// src/firm/platform/host/sim_harness.h) both build the whole Core::
// dependency graph through. Both roots call the SAME function, and the
// only thing that differs between an ARM build and a sim build is which
// Hal::I2CBus implementation (Platform::MicroBitI2CBus vs.
// TestSim::SimPlant) the caller constructs and passes in.
//
// EXPLORATORY-KERNEL REWRITE (2026-08-15,
// clasi/issues/differentialdrive-one-class-one-fiber-exploratory-worktree.md):
// Motion::Planner/Motion::Navigator/Motion::Odometry are DELETED along
// with the whole src/firm/motion/ tree -- RobotGraph no longer
// constructs any of the three. Control::DifferentialDrive is now the
// full-blown wheel KERNEL (its own fiber; see control/differential_drive.h)
// rather than a per-cycle tick()/update() object RobotLoop drove
// directly -- RobotGraph constructs it and pushes its boot config via
// Configurator (same as before), but does NOT call drive_.begin() or
// drive_.start() itself: begin() must run AFTER Preamble::done() (the
// bus power-settle discipline Preamble's own header documents), and
// start() launches the kernel's fiber, which only an ARM caller with a
// real Hal::FiberLauncher ever does. Both are the COMPOSITION ROOT'S own
// job, one level up (main.cpp / SimHarness::boot()) -- see this header's
// "Lifecycle, one level up" note below.
//
// Overrides: the composition-root unification's whole point is that NO
// value should differ between the two roots except by an explicit,
// visible exception. BootOverrides is that one visible seam. Two
// overrides remain genuinely justified (three others -- trackWidth,
// controlPeriod/actuationDelay, navigatorYawSign -- were REMOVED by this
// rewrite: trackWidth because the kernel takes no trackWidth parameter
// at all -- see differential_drive.h's own constructor -- and
// controlPeriod/actuationDelay/navigatorYawSign because they fed
// Motion::PlannerLimits/Motion::NavigatorLimits, both deleted):
//   - otosConfig: TestSim::SimPlant's OtosPlant is already a perfect,
//     zero-mounting-error sensor -- a real chip's measured lever-arm/
//     scale correction has no counterpart to correct in it.
//   - wheelCorrection: same shape of argument -- TestSim::WheelPlant is
//     linear, so a real gearbox's measured linearization has nothing to
//     linearize in it.
// Every override a caller sets must be commented AT THE CALL SITE
// explaining why -- see TestSim::SimHarness's own composeRobot() call.
//
// Lifecycle, one level up (NOT this file's job, documented here so the
// two call sites -- main.cpp and SimHarness -- stay in sync):
//   1. composeRobot() -- construct the whole graph; Configurator::
//      loadBaked()+install() already pushes the kernel's boot Config via
//      setConfig() (data only, no bus I/O, safe at construction time).
//   2. graph.robotLoop().boot() -- drives Core::Preamble to done()
//      (OTOS/color/line detection; the kernel's own motors are NOT part
//      of this probe sequence any more -- see preamble.h).
//   3. graph.drive().begin() -- primes both encoders + arms the boot
//      zero-write. Deliberately AFTER step 2, mirroring the old
//      preamble-then-motor-begin() order.
//   4. graph.drive().start(fiberRunner) -- ARM only; launches the
//      kernel's own fiber. A host test harness never calls this --
//      it drives graph.drive().step() directly instead.
#pragma once

#include "control/differential_drive.h"
#include "core/boot_calibration.h"
#include "core/comms.h"
#include "core/configurator.h"
#include "core/preamble.h"
#include "core/robot_loop.h"
#include "core/telemetry.h"
#include "config/boot_config.h"
#include "config/persisted_tuning.h"
#include "hal/clock.h"
#include "hal/fiber.h"
#include "hardware/planetx/color_sensor.h"
#include "hal/device_config.h"
#include "hal/i2c_bus.h"
#include "hal/transport.h"
#include "hardware/planetx/line_sensor.h"
#include "hardware/generic/motor_armor.h"
#include "hardware/nezha/nezha_motor.h"
#include "hardware/generic/real_otos.h"

namespace Core {

// BootOverrides -- see this file's own header above.
struct BootOverrides {
  // otosConfig -- see this file's own header for the full rationale. Must
  // be applied at CONSTRUCTION time (bakeBootValues()) -- RealOtos's own
  // setters are no-ops until begin() has run.
  const Hal::OtosConfig* otosConfig = nullptr;

  // wheelCorrection -- see this file's own header. Applied inside
  // Configurator::loadBaked(), before install() ever fans config_ out to
  // the kernel's setConfig().
  const Config::WheelCorrection* wheelCorrection = nullptr;
};

// RobotGraph -- owns the WHOLE Core:: object graph: both drive motors
// (bare NezhaMotor wrapped in MotorArmor), the OTOS/color/line leaves,
// Comms/Telemetry, the Control::DifferentialDrive kernel, Preamble,
// Configurator, and RobotLoop, wired in dependency order. Constructed
// once by composeRobot() (below) and then held for the life of the
// program/test.
class RobotGraph {
 public:
  // bus/clock/sleeper/serialTransport/radioTransport/tuningStore: the
  // leaves each root constructs itself (main.cpp: real hardware;
  // SimHarness: TestSim::SimPlant + fakes) -- this is the ONLY seam
  // between an ARM build and a sim build.
  RobotGraph(Hal::I2CBus& bus, const Hal::Clock& clock, Hal::Sleeper& sleeper,
             Hal::FiberLauncher& launcher,
             Hal::Transport& serialTransport, Hal::Transport& radioTransport,
             Config::TuningStore* tuningStore, const char* banner, const char* idLine,
             const BootOverrides& overrides = {});

  // Never copy or move: several members below hold references to OTHER
  // members of this SAME object.
  RobotGraph(const RobotGraph&) = delete;
  RobotGraph& operator=(const RobotGraph&) = delete;
  RobotGraph(RobotGraph&&) = delete;
  RobotGraph& operator=(RobotGraph&&) = delete;

  // --- Accessors ---
  Hardware::NezhaMotor& motorLeft() { return motorL_; }
  Hardware::NezhaMotor& motorRight() { return motorR_; }
  Hardware::MotorArmor& armorLeft() { return armorL_; }
  Hardware::MotorArmor& armorRight() { return armorR_; }
  Hal::Otos& otos() { return otos_; }
  Hardware::ColorSensorLeaf& color() { return color_; }
  Hardware::LineSensorLeaf& line() { return line_; }
  Comms& comms() { return comms_; }
  Telemetry& telemetry() { return tlm_; }
  Control::DifferentialDrive& drive() { return drive_; }
  Configurator& configurator() { return configurator_; }
  RobotLoop& robotLoop() { return robotLoop_; }
  const RobotLoop& robotLoop() const { return robotLoop_; }
  // Preamble -- exposed so a test composition root (TestSim::SimHarness)
  // can drive boot-probing itself (step-by-step, advancing virtual time
  // between attempts) instead of a single opaque robotLoop().boot() call.
  Preamble& preamble() { return preamble_; }

  // loadPersistedTuning -- main.cpp's own post-boot step: loads the
  // TuningStore's saved snapshot, if any, and either reapplies it (schema
  // version matches) or wipes a stale one. A no-op when tuningStore is
  // null (sim/test roots).
  void loadPersistedTuning();

 private:
  // BootValues -- the small residue of already-baked config this
  // constructor still needs across several member initializers, computed
  // ONCE by bakeBootValues() before any member below it is built.
  struct BootValues {
    Hal::MotorConfig motorCfgL;
    Hal::MotorConfig motorCfgR;
    Hal::OtosConfig otosConfig;
    Hal::ColorConfig colorConfig;
    Hal::LineConfig lineConfig;
    // drivetrainConfig -- kept ONLY for left_port/right_port (which of the
    // 4 generated msg::MotorConfig slots is "left"/"right"); every other
    // field this message carries (trackwidth, vel_*, ekf_*, rotation_*,
    // odom_*, ...) is unused here and was unused here even before this
    // rewrite -- boot_calibration.cpp's own toDeviceMotorConfig() never
    // read them.
    msg::DrivetrainConfig drivetrainConfig;
  };
  static BootValues bakeBootValues(const BootOverrides& overrides);

  BootValues bootValues_;

  Hardware::NezhaMotor motorL_;
  Hardware::NezhaMotor motorR_;
  // PARITY: bare NezhaMotor wrapped in the MotorArmor decorator, the ARMOR
  // handed to the rest of the graph.
  Hardware::MotorArmor armorL_;
  Hardware::MotorArmor armorR_;

  // realOtos_ is ALWAYS constructed (harmless: it does no bus I/O until
  // tick()).
  Hardware::RealOtos realOtos_;

  Hardware::ColorSensorLeaf color_;
  Hardware::LineSensorLeaf line_;

  Comms comms_;
  Telemetry tlm_;
  // Package-port adapters (control/differential_drive.h): the kernel is
  // the DiffDrive PACKAGE and speaks its own ports, not Hal::* -- these
  // four/five forwarders are the entire coupling between the firmware's
  // Hal seams and the package. Constructed before drive_, which holds
  // references into them.
  Control::MotorPort motorPortL_;
  Control::MotorPort motorPortR_;
  Control::ClockPort clockPort_;
  Control::SleeperPort sleeperPort_;
  Control::LauncherPort launcherPort_;
  // The wheel KERNEL -- owns both motors (via the ports over
  // armorL_/armorR_), the encoder split-phase schedule, the (velocity,
  // twist) control law, and its own fiber (started one level up -- see
  // this file's own "Lifecycle" note). Constructed here, BEFORE
  // otos_/preamble_/configurator_/robotLoop_, matching every other
  // member's own "constructed before its dependents" ordering.
  Control::DifferentialDrive drive_;

  // otos_ binds unconditionally to realOtos_.
  Hal::Otos& otos_;

  Preamble preamble_;
  Configurator configurator_;
  RobotLoop robotLoop_;

  Config::TuningStore* tuningStore_;
};

// composeRobot -- constructs and returns the whole RobotGraph. Thin
// wrapper so both call sites read the same way; RobotGraph's own
// constructor does the real work.
RobotGraph composeRobot(Hal::I2CBus& bus, const Hal::Clock& clock, Hal::Sleeper& sleeper,
                        Hal::FiberLauncher& launcher,
                        Hal::Transport& serialTransport, Hal::Transport& radioTransport,
                        Config::TuningStore* tuningStore, const char* banner, const char* idLine,
                        const BootOverrides& overrides = {});

}  // namespace Core
