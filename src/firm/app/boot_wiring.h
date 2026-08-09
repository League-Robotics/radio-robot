// boot_wiring.h -- App::RobotGraph / App::composeRobot(): the ONE shared
// composition root src/firm/main.cpp (ARM) and TestSim::SimHarness (sim,
// src/firm/platform/host/sim_harness.h) both build the whole App::/Motion:: dependency
// graph through. 130-002 (unify-sim-and-robot-composition-roots.md):
// before this file existed, main.cpp and SimHarness each hand-wired their
// own copy of this graph, and the two copies had already drifted once --
// SimHarness's own simPlannerLimits() booted the wheel-trim gains at their
// fail-closed all-zero default while main.cpp booted them live, silently
// disabling the trim in every sim session. composeRobot() makes that
// drift structurally impossible: both roots call the SAME function, and
// the only thing that differs between an ARM build and a sim build is
// which Platform::I2CBus implementation (Platform::MicroBitI2CBus vs.
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
//     overrun or fiber-sleep rounding -- see PlannerBootConfig::
//     controlPeriod's own doc comment). 131-005: the ROBOT JSON's
//     control_period is now simply "= kCycle" (50), not a separately
//     re-measured value that drifts stale every time kCycle itself
//     changes (the OLD 40ms-nominal generation baked a measured "47ms"
//     this way, and the fixed-gap pacing scheme that made that number
//     true made the SAME kind of gap re-open at the 50ms nominal too --
//     see robot_loop.h's kCycle doc comment for the full history and the
//     absolute-deadline pacing fix that closes it by construction rather
//     than by re-measuring).
//   - otosConfig: TestSim::SimPlant's OtosPlant is already a perfect,
//     zero-mounting-error sensor -- a real chip's measured lever-arm/scale
//     correction has no counterpart to correct in it (found empirically,
//     see the field's own doc comment for the measured symptom).
//   - wheelCorrection (133-005, the FOURTH): same shape of argument as
//     otosConfig, and added for the same reason after the same class of
//     measured symptom -- TestSim::WheelPlant is linear, so a real
//     gearbox's measured linearization has nothing to linearize in it.
//     See the field's own doc comment for the regression that proved it.
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
#include "platform/clock.h"
#include "hardware/planetx/color_sensor.h"
#include "hal/device_config.h"
#include "platform/i2c_bus.h"
#include "hardware/planetx/line_sensor.h"
#include "hardware/generic/motor_armor.h"
#include "hardware/nezha/nezha_motor.h"
#include "hardware/generic/real_otos.h"
#include "motion/navigator/navigator.h"
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
  // call could land, so the only safe seam is here, at bakeBootValues() time.
  const Hal::OtosConfig* otosConfig = nullptr;

  // wheelCorrection -- the FOURTH genuinely-justified override (133-005),
  // and the one with a price tag attached.
  //
  // App::Drive's Stage A commanded->actual correction (measured =
  // gain*commanded + intercept, per wheel per direction of approach,
  // docs/design/wheel-speed-command-mapping.md) LINEARIZES one physical
  // drivetrain's gearbox. TestSim::WheelPlant is a plain first-order
  // LINEAR plant with none of the nonlinearity it corrects for, so
  // installing a hardware fit against it cancels nothing -- it just
  // divides every commanded speed by that gain, and bends every leg,
  // because the left and right gains deliberately differ. Identity (gain
  // 1, intercept 0) is the only correct value for this plant.
  //
  // That rule is not new; only its ENFORCEMENT is. The host push path has
  // always honoured it -- sim_boot_config.py's drive_boot_config_for()
  // deliberately omits the wheel correction from what it pushes, and says
  // so at length. What went wrong is that the SIM never needed the push:
  // 130-002 unified the sim and hardware composition roots, so the sim's
  // Drive takes its Stage A calibration from the BAKE (Config::
  // defaultDriveGroup(), generated from data/robots/tovez.json) like real
  // hardware does. The invariant survived on a coincidence -- the bake
  // happened to hold identity -- and the coincidence was documented as if
  // it were a guarantee ("deliberately left at composeRobot()'s own
  // installed value", sim_harness.h).
  //
  // 132-019 fitted tovez's real gearbox on the bench and baked the result
  // (gain 1.0 -> 0.9075 left / 0.8 right). The sim silently began running
  // one warm physical drivetrain's linearization, 13% asymmetric, against
  // a plant with no asymmetry at all. Measured cost, TOUR_1/ideal in
  // src/tests/testgui/test_tour_closure_gate.py: worst per-turn error
  // 21.8deg -> 53.8deg, every turn under-rotating. Nothing failed that
  // was already passing, so it went unnoticed for a sprint.
  //
  // Hence a typed override rather than another comment: a caller whose
  // plant is linear says so HERE, and Configurator::loadBaked() applies it
  // to config_ before install(DRIVE) ever reads it. The hardware path is
  // untouched -- main.cpp passes no override, so a real robot still gets
  // its fitted gains from the file, which is the entire point of the
  // configuration-discipline work. src/tests/sim/unit/
  // sim_harness_configure_harness.cpp asserts the sim end of it, so the
  // next composition-root refactor breaks a test instead of a tour.
  const Config::WheelCorrection* wheelCorrection = nullptr;

  // navigatorYawSign -- the FIFTH genuinely-justified override (135-004),
  // the exact same shape of argument as otosConfig/wheelCorrection above:
  // a real robot's measured hardware quirk baked into the active robot
  // JSON has no counterpart to correct in a plant that cannot represent
  // the quirk at all.
  //
  // Motion::NavigatorLimits::yawSign (src/motion/navigator/arc_solver.h)
  // relates commanded Move::omega to true-world CCW -- a REAL drivetrain
  // fact (measured, e.g. src/tests/bench/goto_otos.py's own YAW_SIGN),
  // baked per-robot (data/robots/tovez.json bakes -1.0 for tovez). Every
  // sim plant this composition root can construct (TestSim::SimPlant/
  // WheelPlant here, TestNav::IdealPlant in the standalone navigator
  // ctest) reads commanded wheel velocities directly as its own ground
  // truth, with no independent, camera-equivalent reference to disagree
  // with -- structurally, not by omission, the same way OtosPlant cannot
  // represent a real chip's mounting lever-arm error. Baking a nonzero-
  // sign correction against such a plant does not compensate for a
  // quirk the plant doesn't have; it makes Navigator's own commanded
  // omega WRONG relative to that plant's one self-consistent convention,
  // and the closed loop diverges instead of converging (measured
  // directly while deriving this fix -- see navigator.h's own
  // NavigatorLimits::yawSign doc comment for the full derivation).
  //
  // Identity (+1.0) is therefore the only correct value here, for the
  // same structural reason identity is the only correct wheelCorrection
  // for TestSim::WheelPlant. The hardware path is untouched -- main.cpp
  // passes no override, so a real robot still gets its measured sign
  // from the file.
  const float* navigatorYawSign = nullptr;
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
  RobotGraph(Platform::I2CBus& bus, const Platform::Clock& clock, Platform::Sleeper& sleeper,
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
  Hardware::NezhaMotor& motorLeft() { return motorL_; }
  Hardware::NezhaMotor& motorRight() { return motorR_; }
  Hardware::MotorArmor& armorLeft() { return armorL_; }
  Hardware::MotorArmor& armorRight() { return armorR_; }
  Hal::Otos& otos() { return otos_; }
  Hardware::ColorSensorLeaf& color() { return color_; }
  Hardware::LineSensorLeaf& line() { return line_; }
  Comms& comms() { return comms_; }
  Telemetry& telemetry() { return tlm_; }
  Drive& drive() { return drive_; }
  Motion::Odometry& odometry() { return odom_; }
  Motion::Planner& planner() { return planner_; }
  Motion::Navigator& navigator() { return navigator_; }
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
  // BootValues -- the small residue of already-baked config this
  // constructor still needs across several member initializers, computed
  // ONCE by bakeBootValues() before any member below it is built, so every
  // member initializer reads a plain value, never re-derives one. Declared
  // FIRST so it is guaranteed constructed before every member after it
  // (C++ initializes in declaration order).
  //
  // 132-006 (the-configuration-object.md): this struct used to be named
  // `Resolved` and additionally carried driveConfig/wheelControllerConfig
  // -- both DELETED here, because App::Configurator now owns those two
  // groups' values (Config::Robot, configurator.h) and installs them
  // itself, post-construction, via configurator_.loadBaked() +
  // configurator_.install() (see boot_wiring.cpp's constructor body). What
  // remains below is exactly the ~14-value "no post-construction setter"
  // residue the issue's own audit found (trackWidth, the OTOS lever
  // arm/scales, ColorConfig/LineConfig, PlannerLimits' non-re-appliable
  // fields, the per-port motor wiring) -- values that MUST be complete
  // before Drive/Odometry/the OTOS leaf/Motion::Planner/the motor leaves
  // are constructed, because none of them has a setter for these fields at
  // all. Config::Robot cannot supply these yet: it does not model per-port
  // motor wiring (msg::Motors is drive-pair-only, no port/slewRate/polled),
  // and Configurator itself cannot be constructed before Drive/Planner/the
  // motor leaves exist (it holds references to them) -- so this
  // construction-time bake stays a SEPARATE step from Configurator::
  // loadBaked(), which populates the read-back object AFTER construction,
  // from the same JSON via a different, newer generated function family
  // (config/boot_config.h's Config::default*Group(), 132-005). Both read
  // the identical robot-JSON keys; see 132-005's own completion note.
  struct BootValues {
    Hal::MotorConfig motorCfgL;
    Hal::MotorConfig motorCfgR;
    Hal::OtosConfig otosConfig;
    Hal::ColorConfig colorConfig;
    Hal::LineConfig lineConfig;
    msg::DrivetrainConfig drivetrainConfig;
    float trackWidth = 0.0f;  // [mm]
    Motion::PlannerLimits plannerLimits;
  };
  static BootValues bakeBootValues(const BootOverrides& overrides);

  BootValues bootValues_;
  float trackWidth_;

  Hardware::NezhaMotor motorL_;
  Hardware::NezhaMotor motorR_;
  // PARITY: bare NezhaMotor wrapped in the MotorArmor decorator, the ARMOR
  // handed to the rest of the graph -- the ONE difference between an ARM
  // build and a sim build is what answers on the I2C bus underneath these,
  // never this wiring shape itself.
  Hardware::MotorArmor armorL_;
  Hardware::MotorArmor armorR_;

  // realOtos_ is ALWAYS constructed (harmless: it does no bus I/O until
  // tick()), exactly mirroring main.cpp's own pre-130-002 shape -- declared
  // here (right after the motor leaves) because it needs only `bus`.
  Hardware::RealOtos realOtos_;

  Hardware::ColorSensorLeaf color_;
  Hardware::LineSensorLeaf line_;

  Comms comms_;
  Telemetry tlm_;
  Drive drive_;
  Motion::Odometry odom_;

  // otos_ binds to whichever OTOS implementation this build selects.
  // fakeOtos_ (FAKE_OTOS builds only -- never true for HOST_BUILD/sim,
  // src/firm/platform/host/CMakeLists.txt never defines it) needs odom_/armorL_/armorR_,
  // so both must be declared AFTER odom_ above -- matching main.cpp's own
  // pre-130-002 order, where the FAKE_OTOS ifdef block came after odom's
  // construction for the same reason.
#ifdef FAKE_OTOS
  App::FakeOtos fakeOtos_;
#endif
  Hal::Otos& otos_;

  Motion::Planner planner_;

  // navigatorLimits_/navigator_ (135-004): declared AFTER planner_ (navigator_
  // holds a Planner& to it) and BEFORE configurator_/robotLoop_ (both need
  // references into these). navigatorLimits_ starts default-constructed
  // (NavigatorLimits' own struct defaults, arc_solver.h) -- the REAL,
  // robot-JSON-baked values land via App::configureNavigator(), called from
  // configurator_.install() at the end of this constructor's body, exactly
  // like every other live group's boot-time fan-out (configurator.cpp's own
  // install()). Motion::Navigator holds navigatorLimits_ by const
  // reference (navigator.h's own design) so that later write is visible to
  // the very next tick() with no separate re-apply step.
  Motion::NavigatorLimits navigatorLimits_;
  Motion::Navigator navigator_;

  Preamble preamble_;
  Configurator configurator_;
  RobotLoop robotLoop_;

  Config::TuningStore* tuningStore_;
};

// composeRobot -- constructs and returns the whole RobotGraph. Thin
// wrapper so both call sites read the same way; RobotGraph's own
// constructor does the real work. See RobotGraph's own doc comment above
// for the parameter contract.
RobotGraph composeRobot(Platform::I2CBus& bus, const Platform::Clock& clock, Platform::Sleeper& sleeper,
                        Transport& serialTransport, Transport& radioTransport,
                        Config::TuningStore* tuningStore, const char* banner, const char* idLine,
                        const BootOverrides& overrides = {});

}  // namespace App
