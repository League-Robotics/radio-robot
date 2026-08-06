// boot_wiring.cpp -- see boot_wiring.h for the composition-root contract.
#include "app/boot_wiring.h"

#include "app/debug.h"

namespace App {

RobotGraph::BootValues RobotGraph::bakeBootValues(const BootOverrides& overrides) {
  BootValues r;

  msg::MotorConfig motorConfigs[Config::kMotorConfigCount];
  Config::defaultMotorConfigs(motorConfigs);
  r.drivetrainConfig = Config::defaultDrivetrainConfig();
  r.motorCfgL = toDeviceMotorConfig(motorConfigs[r.drivetrainConfig.left_port - 1]);
  r.motorCfgR = toDeviceMotorConfig(motorConfigs[r.drivetrainConfig.right_port - 1]);

  if (overrides.otosConfig) {
    r.otosConfig = *overrides.otosConfig;
  } else {
    const Config::OtosBootConfig otosBoot = Config::defaultOtosBootConfig();
    r.otosConfig.offsetX = otosBoot.offsetX;
    r.otosConfig.offsetY = otosBoot.offsetY;
    r.otosConfig.offsetYaw = otosBoot.offsetYaw;
    r.otosConfig.linearScale = otosBoot.linearScale;
    r.otosConfig.angularScale = otosBoot.angularScale;
  }

  // colorConfig/lineConfig stay at Devices::ColorConfig{}/LineConfig{}'s own
  // defaults -- neither root has ever baked a robot-JSON override for these
  // (main.cpp's own pre-130-002 code constructed the same all-default
  // structs inline).
  r.colorConfig = Devices::ColorConfig{};
  r.lineConfig = Devices::LineConfig{};

  // 132-007 (subsystem configure() entry points + derived-value methods):
  // trackWidth is now computed via Config::Robot::effectiveTrackWidth()
  // (config/robot.h) -- the ONE definition of the scrub-corrected track
  // width -- rather than the free function effectiveTrackWidth() above.
  // Config::defaultGeometryGroup() and Config::defaultDrivetrainConfig()
  // bake IDENTICAL trackwidth/rotational_slip values from the same robot
  // JSON (132-005's own parity note), so this is the same number via the
  // now-canonical path. effectiveTrackWidth(msg::DrivetrainConfig) itself
  // is UNCHANGED and stays in use by composition_root_parity_harness.cpp,
  // which deliberately computes its own "hardware-equivalent" value
  // independently of whatever composeRobot() does internally -- retargeting
  // its call site there is not this ticket's job.
  Config::Robot geometrySource;
  geometrySource.geometry = Config::defaultGeometryGroup();
  r.trackWidth = overrides.trackWidth ? *overrides.trackWidth : geometrySource.effectiveTrackWidth();

  // PlannerLimits below is the plant-validated tuning baked from the
  // active robot JSON's own `planner` section (see bootPlannerLimits()'s
  // own doc comment for the full provenance and the limit-cycle erratum
  // this tuning replaced) -- deliberately NOT a hand-rolled sim-plausible
  // literal set. controlPeriod/actuationDelay are overridable: the sim
  // genuinely delivers its own cycle time (see BootOverrides' own doc
  // comment, boot_wiring.h).
  r.plannerLimits = bootPlannerLimits(r.drivetrainConfig, r.trackWidth);
  if (overrides.controlPeriod) r.plannerLimits.plant.controlPeriod = *overrides.controlPeriod;
  if (overrides.actuationDelay) r.plannerLimits.plant.actuationDelay = *overrides.actuationDelay;

  return r;
}

RobotGraph::RobotGraph(Devices::I2CBus& bus, const Devices::Clock& clock, Devices::Sleeper& sleeper,
                       Transport& serialTransport, Transport& radioTransport,
                       Config::TuningStore* tuningStore, const char* banner, const char* idLine,
                       const BootOverrides& overrides)
    : bootValues_(bakeBootValues(overrides)),
      trackWidth_(bootValues_.trackWidth),
      motorL_(bus, bootValues_.motorCfgL),
      motorR_(bus, bootValues_.motorCfgR),
      armorL_(motorL_),
      armorR_(motorR_),
      realOtos_(bus, bootValues_.otosConfig),
      color_(bus, bootValues_.colorConfig),
      line_(bus, bootValues_.lineConfig),
      comms_(serialTransport, radioTransport, banner, idLine),
      tlm_(comms_),
      drive_(armorL_, armorR_, bootValues_.trackWidth),
      odom_(bootValues_.trackWidth, armorL_.position(), armorR_.position()),
#ifdef FAKE_OTOS
      // FAKE_OTOS (120-002, sprint's own "confined, acceptable" exception --
      // unify-sim-and-robot-composition-roots.md item 5): a bench build
      // variant that synthesizes the OTOS reading from encoder kinematics
      // instead of reading the real chip, selected by ONE compile-time
      // macro at this composition root. Never defined for HOST_BUILD/sim
      // (src/sim/CMakeLists.txt never adds -DFAKE_OTOS), so this branch
      // only ever compiles into an opt-in ARM bench image.
      fakeOtos_(odom_, armorL_, armorR_, bootValues_.trackWidth),
      otos_(fakeOtos_),
#else
      otos_(realOtos_),
#endif
      planner_(bootValues_.plannerLimits),
      // navigatorLimits_ default-constructs (arc_solver.h's own struct
      // defaults); the real, baked values land via configurator_.install()
      // below (App::configureNavigator()), same timing as PLANNER_SHAPER/
      // DRIVE/OTOS -- see navigatorLimits_'s own doc comment (boot_wiring.h).
      navigator_(navigatorLimits_, planner_),
      preamble_(armorL_, armorR_, otos_, color_, line_, clock),
      configurator_(drive_, armorL_, armorR_, otos_, planner_, navigatorLimits_, tuningStore),
      robotLoop_(bus, armorL_, armorR_, otos_, color_, line_, comms_, tlm_, drive_, configurator_,
                 odom_, planner_, navigator_, preamble_, clock, sleeper),
      tuningStore_(tuningStore) {
  // Wires the bench/Sim-only DBG debug channel to this graph's own Comms --
  // a no-op call unless ROBOT_DEBUG is defined (app/debug.h's own compile
  // gate) or, for the host/sim build, always live (HOST_BUILD, per
  // src/sim/CMakeLists.txt's own APP_SOURCES comment).
  App::setDebugSink(&comms_);

  // 132-006 (the-configuration-object.md): loadBaked() + install() replace
  // the old resolve()-fed installShaperLimits()/installDriveCalibration()/
  // installWheelController() sequence -- Configurator now owns the
  // Config::Robot those three read from (config/boot_config.h's newer
  // Config::default*Group() functions, 132-005), and fans it out itself.
  // Both calls run AFTER every member above is constructed, exactly as the
  // pre-132-006 install*Calibration() calls did.
  // 133-005: overrides.wheelCorrection (nullptr on hardware -- main.cpp
  // passes none, so a real robot still boots the JSON's own fitted gains)
  // is applied INSIDE loadBaked(), before install() fans config_ out to
  // Drive::configure(). See BootOverrides::wheelCorrection's own doc
  // comment (boot_wiring.h) for why the sim needs it and what it cost to
  // find out.
  configurator_.loadBaked(overrides.wheelCorrection);
  configurator_.install();

  // 135-004: overrides.navigatorYawSign (nullptr on hardware -- main.cpp
  // passes none, so a real robot still gets its measured sign from the
  // file) is applied AFTER install() -- unlike wheelCorrection, there is
  // no Configurator::loadBaked() parameter for a single NavigatorLimits
  // field to ride in on; install() has already written navigatorLimits_
  // from the baked config_.navigator by this point (App::
  // configureNavigator()), so overwriting the one field here lands after
  // it, not before, with the identical net effect. See BootOverrides::
  // navigatorYawSign's own doc comment (boot_wiring.h) for why the sim
  // needs it.
  if (overrides.navigatorYawSign != nullptr) {
    navigatorLimits_.yawSign = *overrides.navigatorYawSign;
  }

  // 132-007: rotation calibration now installs through RobotLoop's own
  // configure(const Config::Robot&) (robot_loop.h) instead of the old
  // installRotationCalibration() free function (deleted -- this was its
  // only call site) -- resolving the "solve cleanly" note 132-006 left
  // here (see git history for the prior version of this comment).
  // RobotLoop::configure() takes the whole Config::Robot exactly like
  // Drive/the boot_calibration.h adapters do, reading configurator_'s own
  // freshly-loaded object (loadBaked() above) rather than
  // bootValues_.drivetrainConfig (the old, separate msg::DrivetrainConfig
  // family) -- a direct call, not routed through Configurator::install():
  // RobotLoop already depends on Configurator (routeCommand()'s CONFIG
  // arm), so the reverse reference would be circular. RobotLoop::
  // configure() needs no Configurator& itself to avoid that -- it just
  // takes the object as a plain parameter, the same shape every other
  // subsystem's configure() takes.
  robotLoop_.configure(configurator_.config());
}

void RobotGraph::loadPersistedTuning() {
  if (tuningStore_ == nullptr) return;
  uint32_t storedVersion = 0;
  Config::Blob storedBlob{};
  bool storeHadData = tuningStore_->load(&storedVersion, &storedBlob);
  if (storeHadData && !Config::shouldWipe(storedVersion, Config::kConfigSchemaVersion)) {
    configurator_.reapplyPersistedTuning(Config::deserializeSnapshot(storedBlob));
  } else if (storeHadData) {
    tuningStore_->wipe();
  }
}

RobotGraph composeRobot(Devices::I2CBus& bus, const Devices::Clock& clock, Devices::Sleeper& sleeper,
                        Transport& serialTransport, Transport& radioTransport,
                        Config::TuningStore* tuningStore, const char* banner, const char* idLine,
                        const BootOverrides& overrides) {
  return RobotGraph(bus, clock, sleeper, serialTransport, radioTransport, tuningStore, banner,
                    idLine, overrides);
}

}  // namespace App
