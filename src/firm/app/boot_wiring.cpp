// boot_wiring.cpp -- see boot_wiring.h for the composition-root contract.
#include "app/boot_wiring.h"

#include "app/debug.h"

namespace App {

RobotGraph::Resolved RobotGraph::resolve(const BootOverrides& overrides) {
  Resolved r;

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

  r.driveConfig = Config::defaultDriveConfig();
  r.wheelControllerConfig = Config::defaultWheelControllerConfig();

  // PlannerLimits below is the plant-validated tuning baked from the
  // active robot JSON's own `planner` section (see bootPlannerLimits()'s
  // own doc comment for the full provenance and the limit-cycle erratum
  // this tuning replaced) -- deliberately NOT a hand-rolled sim-plausible
  // literal set. controlPeriod/actuationDelay are overridable: the sim
  // genuinely delivers its own cycle time (see BootOverrides' own doc
  // comment, boot_wiring.h).
  r.plannerLimits = bootPlannerLimits(r.drivetrainConfig, r.trackWidth);
  if (overrides.controlPeriod) r.plannerLimits.controlPeriod = *overrides.controlPeriod;
  if (overrides.actuationDelay) r.plannerLimits.actuationDelay = *overrides.actuationDelay;

  return r;
}

RobotGraph::RobotGraph(Devices::I2CBus& bus, const Devices::Clock& clock, Devices::Sleeper& sleeper,
                       Transport& serialTransport, Transport& radioTransport,
                       Config::TuningStore* tuningStore, const char* banner, const char* idLine,
                       const BootOverrides& overrides)
    : resolved_(resolve(overrides)),
      trackWidth_(resolved_.trackWidth),
      motorL_(bus, resolved_.motorCfgL),
      motorR_(bus, resolved_.motorCfgR),
      armorL_(motorL_),
      armorR_(motorR_),
      realOtos_(bus, resolved_.otosConfig),
      color_(bus, resolved_.colorConfig),
      line_(bus, resolved_.lineConfig),
      comms_(serialTransport, radioTransport, banner, idLine),
      tlm_(comms_),
      drive_(armorL_, armorR_, resolved_.trackWidth),
      odom_(resolved_.trackWidth, armorL_.position(), armorR_.position()),
#ifdef FAKE_OTOS
      // FAKE_OTOS (120-002, sprint's own "confined, acceptable" exception --
      // unify-sim-and-robot-composition-roots.md item 5): a bench build
      // variant that synthesizes the OTOS reading from encoder kinematics
      // instead of reading the real chip, selected by ONE compile-time
      // macro at this composition root. Never defined for HOST_BUILD/sim
      // (src/sim/CMakeLists.txt never adds -DFAKE_OTOS), so this branch
      // only ever compiles into an opt-in ARM bench image.
      fakeOtos_(odom_, armorL_, armorR_, resolved_.trackWidth),
      otos_(fakeOtos_),
#else
      otos_(realOtos_),
#endif
      planner_(resolved_.plannerLimits),
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

  // Shaping/drive/rotation calibration -- installed AFTER every member
  // above is constructed, exactly as main.cpp's own pre-130-002 sequence
  // did (planner.applyShaperLimits() / drive.setDutyPerSpeed() /
  // robotLoop.setRotationCalibration() all called only once their targets
  // already exist).
  installShaperLimits(planner_, resolved_.plannerLimits);
  installDriveCalibration(drive_, resolved_.driveConfig);
  installWheelController(drive_, resolved_.wheelControllerConfig);
  installRotationCalibration(robotLoop_, resolved_.drivetrainConfig);
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
