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
#include "motion/planner/planner.h"
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
//
// 125-003: vel_gains/vel_filt_alpha/min_duty are NO LONGER copied here --
// Devices::MotorConfig dropped those fields (the velocity PID they fed
// relocated to Motion::WheelVelocityPid, App::Drive's own interim instances
// -- see toMotionGains() below and drive.h's own header). vel_filt_alpha
// has no live consumer at all this sprint (the EMA estimator it fed was
// deleted outright, pending ticket 004's App::WheelObserver) -- the wire
// field itself is untouched (no protocol change), simply unread by
// firmware for now.
Devices::MotorConfig toDeviceMotorConfig(const msg::MotorConfig& src) {
  Devices::MotorConfig cfg;
  cfg.wheelTravelCalib = src.travel_calib;
  cfg.fwdSign = src.fwd_sign;
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

// toMotionGains -- 125-003: the vel_gains half of msg::MotorConfig now feeds
// App::Drive's own interim Motion::WheelVelocityPid gains (drive.h's own
// header) instead of Devices::MotorConfig -- same wire fields, different
// application-side destination (sprint.md Decision 7's CONFIG-routing
// split, wired here at boot ahead of ticket 008's live-CONFIG-patch half,
// which RobotLoop::applyMotorConfigPatch() already does -- robot_loop.cpp).
Motion::Gains toMotionGains(const msg::MotorConfig& src) {
  Motion::Gains gains;
  gains.kp = src.vel_gains.kp;
  gains.ki = src.vel_gains.ki;
  gains.kff = src.vel_gains.kff;
  gains.iMax = src.vel_gains.i_max;
  gains.kaw = src.vel_gains.kaw;
  return gains;
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
  // 124-009: Telemetry no longer holds direct Transport& references (those
  // existed only for TelemetrySecondary's own independently-armored line,
  // now deleted) -- comms already owns both transports internally.
  static App::Telemetry tlm(comms);
  static App::Drive drive(motorL, motorR, drivetrainConfig.trackwidth);
  // 125-003: seeds Drive's own interim Motion::WheelVelocityPid gains from
  // the SAME boot-config vel_gains this leaf used to apply directly to
  // Devices::MotorConfig -- see toMotionGains()'s own doc comment above.
  drive.applyGainsLeft(toMotionGains(motorConfigs[drivetrainConfig.left_port - 1]));
  drive.applyGainsRight(toMotionGains(motorConfigs[drivetrainConfig.right_port - 1]));
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

  // Planner integration (2026-07-26): the on-robot Motion::Planner replaces
  // Motion::MoveQueue as the loop's motion decider. Limits assembled from
  // the SAME boot-config sources the old stack used: shaper keys ->
  // profile ceilings, vel_gains -> the planner's own duty-stage PID,
  // vel_filt_alpha -> the planner's velocity-filter weight. controlPeriod
  // is the loop's own kCycle; actuationDelay is the one-cycle staging
  // latency (duty staged at the NEXT cycle top -- the exact shape the
  // planner's duty scenario tier validates).
  Motion::PlannerLimits plannerLimits;
  plannerLimits.vMax = 600.0f;  // [mm/s] plausibility ceiling (kMaxPlausibleSpeed)
  plannerLimits.aMax = shaperLimits.aMax;
  plannerLimits.aDecel = shaperLimits.aDecel;
  plannerLimits.omegaMax = 8.0f;  // [rad/s]
  plannerLimits.alphaMax = shaperLimits.alphaMax;
  plannerLimits.alphaDecel = shaperLimits.alphaDecel;
  plannerLimits.jerkMax = shaperLimits.jMax;
  plannerLimits.yawJerkMax = shaperLimits.yawJerkMax;
  plannerLimits.trackWidth = drivetrainConfig.trackwidth;
  plannerLimits.controlPeriod = static_cast<float>(App::RobotLoop::kCycle);
  plannerLimits.actuationDelay = static_cast<float>(App::RobotLoop::kCycle);
  plannerLimits.velocityFilterWeight =
      drivetrainConfig.vel_filt_alpha > 0.05f ? drivetrainConfig.vel_filt_alpha
                                              : 1.0f;
  plannerLimits.requireSettle = true;
  // Rest floors sized to the measured hardware encoder-velocity noise
  // (plant ID 2026-07-26: ~+-7 mm/s at rest) -- the rest-damping stage
  // outputs exactly zero duty below the floor, so it must clear the
  // noise band or the wheels twitch at rest forever.
  plannerLimits.settleRestVelocity = 10.0f;  // [mm/s]
  plannerLimits.settleRestOmega = 0.16f;     // [rad/s] 2*floor/trackWidth
  plannerLimits.settleWindow = 2000.0f;  // [ms]
  plannerLimits.headingHoldGain = 1.5f;  // [1/s] sim-validated
  {
    const Motion::Gains bootGains =
        toMotionGains(motorConfigs[drivetrainConfig.left_port - 1]);
    plannerLimits.velKff = bootGains.kff;
    plannerLimits.velKp = bootGains.kp;
    plannerLimits.velKi = bootGains.ki;
    plannerLimits.velIMax = bootGains.iMax;
    // Accel feedforward from the measured plant time constant (~230 ms,
    // docs/design/encoder-refresh-characterization.md's sibling plant-ID
    // measurement) -- a derived value, not a new tunable, until a config
    // key exists for tau.
    plannerLimits.velKaff = 0.23f * bootGains.kff;
    plannerLimits.velIAccelGate = 50.0f;  // [mm/s^2]
  }
  static Motion::Planner planner(plannerLimits);
  // Boot-config shaper block present (both axes' enable gate satisfied,
  // the same zero-means-disabled sentinel the old ShaperLimits carried):
  // land it through the SAME applyShaperLimits() entry the wire push uses,
  // marking shaping CONFIGURED so kFlagFaultShapingDisabled (119-001)
  // stays quiet. A zeroed/absent block leaves the flag armed -- the loud
  // off-state, exactly as before the planner cutover.
  if (shaperLimits.aMax > 0.0f && shaperLimits.aDecel > 0.0f &&
      shaperLimits.alphaMax > 0.0f && shaperLimits.alphaDecel > 0.0f) {
    planner.applyShaperLimits(shaperLimits.aMax, shaperLimits.aDecel,
                              shaperLimits.alphaMax, shaperLimits.alphaDecel,
                              shaperLimits.jMax, shaperLimits.yawJerkMax);
  }
  static App::Preamble preamble(motorL, motorR, otos, color, line, clock);


  static Config::MicroBitTuningStore tuningStore(uBit.storage);


  static App::RobotLoop robotLoop(bus, motorL, motorR, otos, color, line,
                                   comms, tlm, drive, odom, planner, preamble,
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
