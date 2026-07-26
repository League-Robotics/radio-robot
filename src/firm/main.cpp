// main.cpp -- the ARM entry point. Owns the real MicroBit hardware singleton,
// constructs and wires every leaf/app module, then hands off to
// App::RobotLoop (app/robot_loop.{h,cpp}) for the boot loop + main cycle.
// No cycle logic lives here. Design/rationale: DESIGN.md.
#include "MicroBit.h"

#include <cstdio>
#include <cstring>

#include "app/comms.h"
#include "app/drive.h"
#include "app/fake_otos.h"
#include "app/preamble.h"
#include "app/robot_loop.h"
#include "app/telemetry.h"
#include "com/banner.h"
#include "com/radio.h"
#include "com/serial_port.h"
#include "config/boot_config.h"
#include "config/persisted_tuning.h"
#include "devices/color_sensor.h"
#include "devices/device_config.h"
#include "devices/microbit_clock.h"
#include "devices/microbit_i2c_bus.h"
#include "devices/line_sensor.h"
#include "devices/motor_armor.h"
#include "devices/nezha_motor.h"
#include "devices/otos.h"
#include "motion/move_queue.h"
#include "motion/odometry.h"
#include "motion/state_estimator.h"
#include "types/version_generated.h"

static MicroBit uBit;

namespace {

// Converts the boot config's wire-plane msg::MotorConfig into the
// Devices-local MotorConfig NezhaMotor's constructor needs. Lives here
// because main.cpp is the one place both types are reachable -- the
// devices/ isolation invariant (DESIGN.md) forbids devices/ from including
// messages/ or config/.
Devices::MotorConfig toDeviceMotorConfig(const msg::MotorConfig& src) {
  Devices::MotorConfig cfg;
  cfg.wheelTravelCalib = src.travel_calib;
  cfg.fwdSign = src.fwd_sign;
  cfg.velGains.kp = src.vel_gains.kp;
  cfg.velGains.ki = src.vel_gains.ki;
  cfg.velGains.kff = src.vel_gains.kff;
  cfg.velGains.iMax = src.vel_gains.i_max;
  cfg.velGains.kaw = src.vel_gains.kaw;
  cfg.velFiltAlpha = src.vel_filt_alpha;
  cfg.velDeadband = src.min_duty;
  cfg.slewRate = src.slew_rate;
  cfg.port = src.port;
  // Devices::MotorConfig's reversalDwell/outputDeadband are plain required
  // floats (sprint 114 ticket 003) -- gen_boot_config.py + ticket 002's
  // required-key gate guarantee src.reversal_dwell/src.output_deadband are
  // always set (.has == true) here, so read .val directly rather than
  // changing the wire msg::MotorConfig itself (still Opt<float> -- the wire
  // schema is out of scope, see the ticket's own Approach step 6).
  cfg.reversalDwell = src.reversal_dwell.val;
  cfg.outputDeadband = src.output_deadband.val;
  cfg.polled = src.polled;
  return cfg;
}


Motion::FusionWeights toFusionWeights(const Config::EstimatorBootConfig& src) {
  Motion::FusionWeights weights;
  weights.headingOtos = src.headingOtos;
  weights.omegaOtos = src.omegaOtos;
  weights.staleness = src.staleness;
  return weights;
}

Motion::ShaperLimits toShaperLimits(const Config::ShaperBootConfig& src) {
  Motion::ShaperLimits limits;
  limits.aMax = src.aMax;
  limits.aDecel = src.aDecel;
  limits.alphaMax = src.alphaMax;
  limits.alphaDecel = src.alphaDecel;
  limits.jMax = src.jMax;
  limits.yawJerkMax = src.yawJerkMax;
  return limits;
}

}  // namespace

int main() {
  uBit.init();

  static SerialPort serial(uBit.serial);
  serial.begin();
  static Radio radio(uBit.radio, uBit.messageBus);
  radio.begin();

  static char banner[64];
  formatBanner(banner, sizeof(banner));
  serial.sendReliable(banner);
  radio.send(reinterpret_cast<const uint8_t*>(banner), static_cast<uint16_t>(std::strlen(banner)));


  static Devices::MicroBitI2CBus bus(uBit.i2c);

  msg::MotorConfig motorConfigs[Config::kMotorConfigCount];
  Config::defaultMotorConfigs(motorConfigs);
  msg::DrivetrainConfig drivetrainConfig = Config::defaultDrivetrainConfig();
  Config::OtosBootConfig otosBootConfig = Config::defaultOtosBootConfig();

  Devices::MotorConfig motorCfgL =
      toDeviceMotorConfig(motorConfigs[drivetrainConfig.left_port - 1]);
  Devices::MotorConfig motorCfgR =
      toDeviceMotorConfig(motorConfigs[drivetrainConfig.right_port - 1]);
  static Devices::NezhaMotor motorLBare(bus, motorCfgL);
  static Devices::NezhaMotor motorRBare(bus, motorCfgR);
  static Devices::MotorArmor motorL(motorLBare);
  static Devices::MotorArmor motorR(motorRBare);

  (void)motorL.reconfigure(motorCfgL);
  (void)motorR.reconfigure(motorCfgR);

  Devices::OtosConfig otosConfig;
  otosConfig.offsetX = otosBootConfig.offsetX;
  otosConfig.offsetY = otosBootConfig.offsetY;
  otosConfig.offsetYaw = otosBootConfig.offsetYaw;
  otosConfig.linearScale = otosBootConfig.linearScale;
  otosConfig.angularScale = otosBootConfig.angularScale;

  [[maybe_unused]] static Devices::RealOtos realOtos(bus, otosConfig);

  static Devices::ColorConfig colorConfig;
  static Devices::ColorSensorLeaf color(bus, colorConfig);
  static Devices::LineConfig lineConfig;
  static Devices::LineSensorLeaf line(bus, lineConfig);

  static Devices::MicroBitClock clock;
  static Devices::MicroBitSleeper sleeper;

  // ID:<drivetrain>:<profile>:<version> -- sprint 124 architecture
  // Decision 4: configured-robot identity (drivetrain type +
  // calibration-profile name/version), distinct from `banner`'s hardware
  // identity. Both Config::kDrivetrainType and Config::kRobotProfileName
  // are generated string constants (boot_config.h) baked from the robot
  // JSON's own identity.drivetrain_type/filename stem -- NOT derived from
  // any wire-level DrivetrainConfig field (defaultDrivetrainConfig()
  // never bakes half_track; it stays at its wire default 0.0f for every
  // profile, so it cannot answer this question -- see kDrivetrainType's
  // own doc comment for why an earlier draft's half_track-based check was
  // wrong). The version reuses VER:'s own generated build-version
  // constant (zero new version-tracking infrastructure, same Decision 4).
  static char idLine[96];
  snprintf(idLine, sizeof(idLine), "ID:%s:%s:%s", Config::kDrivetrainType,
           Config::kRobotProfileName, FIRMWARE_VERSION_STR);

  static App::SerialTransport serialLink(serial);
  static App::RadioTransport radioLink(radio);
  static App::Comms comms(serialLink, radioLink, banner, idLine);
  static App::Telemetry tlm(comms, serialLink, radioLink);
  static App::Drive drive(motorL, motorR, drivetrainConfig.trackwidth);
  static Motion::Odometry odom(drivetrainConfig.trackwidth, motorL.position(), motorR.position());


#ifdef FAKE_OTOS
  static App::FakeOtos fakeOtos(odom, motorL, motorR, drivetrainConfig.trackwidth);
  Devices::Otos& otos = fakeOtos;
#else
  Devices::Otos& otos = realOtos;
#endif

  Config::EstimatorBootConfig estimatorBootConfig = Config::defaultEstimatorConfig();
  static Motion::StateEstimator stateEstimator(toFusionWeights(estimatorBootConfig));

  Motion::ShaperLimits shaperLimits = toShaperLimits(Config::defaultShaperConfig());

  static Motion::MoveQueue moveQueue(drive, odom, drivetrainConfig.trackwidth, shaperLimits);
  static App::Preamble preamble(motorL, motorR, otos, color, line, clock);


  static Config::MicroBitTuningStore tuningStore(uBit.storage);


  static App::RobotLoop robotLoop(bus, motorL, motorR, otos, color, line,
                                   comms, tlm, drive, odom, moveQueue, preamble,
                                   stateEstimator, clock, sleeper, &tuningStore);

  uint32_t storedVersion = 0;
  Config::Blob storedBlob{};
  bool storeHadData = tuningStore.load(&storedVersion, &storedBlob);
  if (storeHadData && !Config::shouldWipe(storedVersion, Config::kConfigSchemaVersion)) {
    robotLoop.reapplyPersistedTuning(Config::deserializeSnapshot(storedBlob));
  } else if (storeHadData) {
    tuningStore.wipe();
  }

  robotLoop.markConfigured();
  robotLoop.run();
}
