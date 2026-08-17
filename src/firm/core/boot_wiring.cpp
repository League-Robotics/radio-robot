// boot_wiring.cpp -- see boot_wiring.h for the composition-root contract.
#include "core/boot_wiring.h"

#include "core/debug.h"

namespace Core {

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

  // colorConfig/lineConfig stay at Hal::ColorConfig{}/LineConfig{}'s own
  // defaults -- neither root has ever baked a robot-JSON override for
  // these.
  r.colorConfig = Hal::ColorConfig{};
  r.lineConfig = Hal::LineConfig{};

  return r;
}

RobotGraph::RobotGraph(Hal::I2CBus& bus, const Hal::Clock& clock, Hal::Sleeper& sleeper,
                       Hal::Transport& serialTransport, Hal::Transport& radioTransport,
                       Config::TuningStore* tuningStore, const char* banner, const char* idLine,
                       const BootOverrides& overrides)
    : bootValues_(bakeBootValues(overrides)),
      motorL_(bus, bootValues_.motorCfgL),
      motorR_(bus, bootValues_.motorCfgR),
      armorL_(motorL_),
      armorR_(motorR_),
      realOtos_(bus, bootValues_.otosConfig),
      color_(bus, bootValues_.colorConfig),
      line_(bus, bootValues_.lineConfig),
      comms_(serialTransport, radioTransport, banner, idLine),
      tlm_(comms_),
      drive_(armorL_, armorR_, clock, sleeper),
      otos_(realOtos_),
      preamble_(otos_, color_, line_, clock),
      configurator_(drive_, armorL_, armorR_, otos_, tuningStore),
      robotLoop_(bus, otos_, color_, line_, comms_, tlm_, drive_, configurator_,
                 preamble_, clock, sleeper),
      tuningStore_(tuningStore) {
  // Wires the bench/Sim-only DBG debug channel to this graph's own Comms.
  Core::setDebugSink(&comms_);

  // loadBaked() + install() push the kernel's whole boot Config via
  // Control::DifferentialDrive::setConfig() -- data only, no bus I/O, so
  // this is safe at CONSTRUCTION time even though drive_.begin() itself
  // must wait until after Preamble::done() (see this class's own header
  // "Lifecycle, one level up" note -- begin()/start() are NOT called
  // here).
  configurator_.loadBaked(overrides.wheelCorrection);
  configurator_.install();

  // Rotation calibration installs through RobotLoop's own
  // configure(const Config::Robot&) -- read-back only now (MOVE's
  // angle-stop correction that used to consume it is deregistered), kept
  // for GET_CONFIG parity and 132-007's existing test coverage.
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

RobotGraph composeRobot(Hal::I2CBus& bus, const Hal::Clock& clock, Hal::Sleeper& sleeper,
                        Hal::Transport& serialTransport, Hal::Transport& radioTransport,
                        Config::TuningStore* tuningStore, const char* banner, const char* idLine,
                        const BootOverrides& overrides) {
  return RobotGraph(bus, clock, sleeper, serialTransport, radioTransport, tuningStore, banner,
                    idLine, overrides);
}

}  // namespace Core
