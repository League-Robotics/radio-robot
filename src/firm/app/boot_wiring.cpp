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

  r.trackWidth =
      overrides.trackWidth ? *overrides.trackWidth : effectiveTrackWidth(r.drivetrainConfig);

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
      preamble_(armorL_, armorR_, otos_, color_, line_, clock),
      configurator_(drive_, armorL_, armorR_, otos_, planner_, tuningStore),
      robotLoop_(bus, armorL_, armorR_, otos_, color_, line_, comms_, tlm_, drive_, configurator_,
                 odom_, planner_, preamble_, clock, sleeper),
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
  configurator_.loadBaked();
  configurator_.install();

  // installRotationCalibration() stays a direct call, not routed through
  // Configurator::install(): RobotLoop already depends on Configurator
  // (routeCommand()'s CONFIG arm), so the reverse reference would be
  // circular, and rotation calibration's real home is RobotLoop's own
  // configure(const Config::Robot&) (ticket 007, "Subsystem configure()
  // consumers ... RobotLoop for geometry/rotation" -- sprint.md Step 3).
  // Data source retargeted from bootValues_.drivetrainConfig to
  // configurator_.config().geometry would need a signature change to
  // installRotationCalibration() (boot_calibration.h, out of this
  // ticket's file scope) -- left for ticket 007, which owns that
  // migration anyway.
  installRotationCalibration(robotLoop_, bootValues_.drivetrainConfig);
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
