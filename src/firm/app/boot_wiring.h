// boot_wiring.h -- App::RobotGraph / App::composeRobot(): the ONE shared
// composition root src/firm/main.cpp (ARM) and TestSim::SimHarness (sim,
// src/sim/sim_harness.h) both build the whole App::/Motion:: dependency
// graph through. 130-002 (unify-sim-and-robot-composition-roots.md):
// before this file existed, main.cpp and SimHarness each hand-wired their
// own copy of this graph, and the two copies had already drifted once --
// SimHarness's own simPlannerLimits() booted the wheel-trim gains at their
// fail-closed all-zero default while main.cpp booted them live, silently
// disabling the trim in every sim session. composeRobot() makes that
// drift structurally impossible: both roots call the SAME function, and
// the only thing that differs between an ARM build and a sim build is
// which Devices::I2CBus implementation (Devices::MicroBitI2CBus vs.
// TestSim::SimPlant) the caller constructs and passes in -- everything
// downstream of the bus is the identical call.
//
// Deliberately a SEPARATE translation unit from boot_calibration.{h,cpp}
// (which holds the smaller Config::boot_config-reading conversion/install
// helpers) -- see boot_calibration.h's own header for why the split
// matters at link time. composeRobot() here CALLS those helpers; it does
// not duplicate their logic.
//
// Overrides: the composition-root unification's whole point is that NO
// value should differ between the two roots except by an explicit,
// visible exception -- never a silent hand-wired divergence. BootOverrides
// is that one visible seam. Three overrides are genuinely justified (130-
// 002's own investigation found every OTHER historical sim/hardware
// difference to be accidental drift, not a real requirement) -- see
// BootOverrides' own doc comment below for the full rationale on each:
//   - trackWidth: a fixture property of the calling test's own scenario
//     (SimHarness's constructor has always taken a trackWidth parameter
//     for exactly this reason), not a hardware calibration fact.
//   - controlPeriod/actuationDelay: the sim genuinely delivers its own
//     cycle time (SimHarness::step() advances virtual time by EXACTLY
//     RobotLoop::kCycle, with none of a real board's vendor-bus-clearance
//     overrun -- see PlannerBootConfig::controlPeriod's own doc comment
//     for why the ROBOT JSON bakes a measured 47ms instead of the nominal
//     40).
//   - otosConfig: TestSim::SimPlant's OtosPlant is already a perfect,
//     zero-mounting-error sensor -- a real chip's measured lever-arm/scale
//     correction has no counterpart to correct in it (found empirically,
//     see the field's own doc comment for the measured symptom).
// Every override a caller sets must be commented AT THE CALL SITE
// explaining why -- see TestSim::SimHarness's own composeRobot() call.
#pragma once

#include "app/boot_calibration.h"
#include "app/comms.h"
#include "app/configurator.h"
#include "app/drive.h"
#include "app/fake_otos.h"
#include "app/preamble.h"
#include "app/robot_loop.h"
#include "app/telemetry.h"
#include "config/boot_config.h"
#include "config/persisted_tuning.h"
#include "devices/clock.h"
#include "devices/color_sensor.h"
#include "devices/device_config.h"
#include "devices/i2c_bus.h"
#include "devices/line_sensor.h"
#include "devices/motor_armor.h"
#include "devices/nezha_motor.h"
#include "devices/otos.h"
#include "motion/odometry.h"
#include "motion/planner/planner.h"

namespace App {

// BootOverrides -- see this file's own header above. nullptr (the default)
// means "use the baked robot-JSON value"; a caller that sets a field is
// making a deliberate, visible exception, not a silent divergence.
struct BootOverrides {
  const float* trackWidth = nullptr;      // [mm]
  const float* controlPeriod = nullptr;   // [ms]
  const float* actuationDelay = nullptr;  // [ms]

  // otosConfig -- a THIRD genuinely-justified override, found empirically
  // (130-002's own composition-root parity work): the baked robot-JSON
  // OtosConfig (offsetX/Y/Yaw, linearScale/angularScale) corrects a REAL
  // chip's measured mounting lever-arm and scale error. TestSim::SimPlant's
  // OtosPlant is already a perfect, zero-error synthetic sensor by
  // construction (its own fault-injection knobs, e.g. sim_set_otos_raw_
  // scale_err(), default to "0 = perfect") -- applying a real chip's
  // measured correction on TOP of an already-perfect synthetic reading
  // introduces a SYSTEMATIC bias with no counterpart to correct (measured:
  // a 2s/150mm/s straight run read 55.8mm of otos-vs-truth drift once the
  // real tovez.json linearScale/offsetX baked in unconditionally). Must be
  // applied at CONSTRUCTION time, not via a post-construction Devices::
  // Otos::setLinearScalar()/setOffset() call -- RealOtos's own setters are
  // no-ops until begin() has run and set initialized_ = true (otos.cpp),
  // and begin() itself applies config_'s own baked scale before any such
  // call could land, so the only safe seam is here, at resolve() time.
  const Devices::OtosConfig* otosConfig = nullptr;
};

// RobotGraph -- owns the WHOLE App::/Motion:: object graph: both drive
// motors (bare NezhaMotor wrapped in MotorArmor), the OTOS/color/line
// leaves, Comms/Telemetry, Drive, Odometry, Planner, Preamble,
// Configurator, and RobotLoop, wired in the same dependency order
// main.cpp/SimHarness always constructed them in. Constructed once by
// composeRobot() (below) and then held for the life of the program/test --
// every accessor returns a reference into this SAME storage, matching the
// `static` function-locals main.cpp used to declare directly.
//
// The ONE thing this class does NOT own: the leaf devices/services each
// root constructs differently (the I2CBus, the Clock/Sleeper, the two
// Transports, the optional TuningStore) -- those are passed in by
// reference/pointer, exactly like RobotLoop's own constructor already
// requires only already-constructed interface references.
class RobotGraph {
 public:
  // bus/clock/sleeper/serialTransport/radioTransport/tuningStore: the
  // leaves each root constructs itself (main.cpp: real hardware;
  // SimHarness: TestSim::SimPlant + fakes) -- this is the ONLY seam
  // between an ARM build and a sim build. banner/idLine: root-identity
  // strings (hardware serial number vs. a fixed sim label), not
  // calibration -- genuinely root-specific, not a drift risk.
  // tuningStore may be null (sim/test roots): persistence disabled,
  // everything else unchanged (mirrors Configurator's own contract).
  RobotGraph(Devices::I2CBus& bus, const Devices::Clock& clock, Devices::Sleeper& sleeper,
             Transport& serialTransport, Transport& radioTransport,
             Config::TuningStore* tuningStore, const char* banner, const char* idLine,
             const BootOverrides& overrides = {});

  // Never copy or move: several members below (Preamble/Configurator/
  // RobotLoop, and otos_ itself) hold references to OTHER members of this
  // SAME object -- a copy or move would leave those references pointing at
  // the ORIGINAL instance, not the new one. composeRobot() returns this
  // type by mandatory copy elision (C++17/20, guaranteed for a direct
  // `return RobotGraph(...)`/`= composeRobot(...)` prvalue), which never
  // invokes either of these -- deleting them just makes the invariant a
  // compile error instead of a silent dangling-reference bug if anyone
  // ever tries.
  RobotGraph(const RobotGraph&) = delete;
  RobotGraph& operator=(const RobotGraph&) = delete;
  RobotGraph(RobotGraph&&) = delete;
  RobotGraph& operator=(RobotGraph&&) = delete;

  // --- Accessors -- every caller need (main.cpp's boot sequence, and
  // every SimHarness test-injection method) reaches into the graph
  // through these, never by re-deriving a value the graph already holds.
  Devices::NezhaMotor& motorLeft() { return motorL_; }
  Devices::NezhaMotor& motorRight() { return motorR_; }
  Devices::MotorArmor& armorLeft() { return armorL_; }
  Devices::MotorArmor& armorRight() { return armorR_; }
  Devices::Otos& otos() { return otos_; }
  Devices::ColorSensorLeaf& color() { return color_; }
  Devices::LineSensorLeaf& line() { return line_; }
  Comms& comms() { return comms_; }
  Telemetry& telemetry() { return tlm_; }
  Drive& drive() { return drive_; }
  Motion::Odometry& odometry() { return odom_; }
  Motion::Planner& planner() { return planner_; }
  Configurator& configurator() { return configurator_; }
  RobotLoop& robotLoop() { return robotLoop_; }
  const RobotLoop& robotLoop() const { return robotLoop_; }
  // Preamble -- exposed so a test composition root (TestSim::SimHarness)
  // can drive boot-probing itself (step-by-step, advancing virtual time
  // between attempts) instead of a single opaque robotLoop().boot() call --
  // see SimHarness::driveBootToDone()'s own comment for why.
  Preamble& preamble() { return preamble_; }

  float trackWidth() const { return trackWidth_; }  // [mm] the resolved (post-override) value

  // loadPersistedTuning -- main.cpp's own post-boot step (unchanged
  // behavior, just relocated): loads the TuningStore's saved snapshot, if
  // any, and either reapplies it (schema version matches) or wipes a
  // stale one. A no-op when tuningStore is null (sim/test roots) -- exactly
  // main.cpp's own pre-130-002 flow, now shared instead of duplicated.
  void loadPersistedTuning();

 private:
  // Resolved, already-baked config this constructor needs across several
  // member initializers -- computed ONCE by resolve() before any member
  // below it is built, so every member initializer reads a plain value,
  // never re-derives one. Declared FIRST so it is guaranteed constructed
  // before every member after it (C++ initializes in declaration order).
  struct Resolved {
    Devices::MotorConfig motorCfgL;
    Devices::MotorConfig motorCfgR;
    Devices::OtosConfig otosConfig;
    Devices::ColorConfig colorConfig;
    Devices::LineConfig lineConfig;
    msg::DrivetrainConfig drivetrainConfig;
    Config::DriveBootConfig driveConfig;
    Config::WheelControllerBootConfig wheelControllerConfig;
    float trackWidth = 0.0f;  // [mm]
    Motion::PlannerLimits plannerLimits;
  };
  static Resolved resolve(const BootOverrides& overrides);

  Resolved resolved_;
  float trackWidth_;

  Devices::NezhaMotor motorL_;
  Devices::NezhaMotor motorR_;
  // PARITY: bare NezhaMotor wrapped in the MotorArmor decorator, the ARMOR
  // handed to the rest of the graph -- the ONE difference between an ARM
  // build and a sim build is what answers on the I2C bus underneath these,
  // never this wiring shape itself.
  Devices::MotorArmor armorL_;
  Devices::MotorArmor armorR_;

  // realOtos_ is ALWAYS constructed (harmless: it does no bus I/O until
  // tick()), exactly mirroring main.cpp's own pre-130-002 shape -- declared
  // here (right after the motor leaves) because it needs only `bus`.
  Devices::RealOtos realOtos_;

  Devices::ColorSensorLeaf color_;
  Devices::LineSensorLeaf line_;

  Comms comms_;
  Telemetry tlm_;
  Drive drive_;
  Motion::Odometry odom_;

  // otos_ binds to whichever OTOS implementation this build selects.
  // fakeOtos_ (FAKE_OTOS builds only -- never true for HOST_BUILD/sim,
  // src/sim/CMakeLists.txt never defines it) needs odom_/armorL_/armorR_,
  // so both must be declared AFTER odom_ above -- matching main.cpp's own
  // pre-130-002 order, where the FAKE_OTOS ifdef block came after odom's
  // construction for the same reason.
#ifdef FAKE_OTOS
  App::FakeOtos fakeOtos_;
#endif
  Devices::Otos& otos_;

  Motion::Planner planner_;
  Preamble preamble_;
  Configurator configurator_;
  RobotLoop robotLoop_;

  Config::TuningStore* tuningStore_;
};

// composeRobot -- constructs and returns the whole RobotGraph. Thin
// wrapper so both call sites read the same way; RobotGraph's own
// constructor does the real work. See RobotGraph's own doc comment above
// for the parameter contract.
RobotGraph composeRobot(Devices::I2CBus& bus, const Devices::Clock& clock, Devices::Sleeper& sleeper,
                        Transport& serialTransport, Transport& radioTransport,
                        Config::TuningStore* tuningStore, const char* banner, const char* idLine,
                        const BootOverrides& overrides = {});

}  // namespace App
