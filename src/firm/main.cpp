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
// Devices::MotorConfig dropped those fields (the velocity PID they fed has
// since been deleted outright -- see drive.h's own header). vel_filt_alpha
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

// toMotionGains -- DELETED (command-ingestion-...-two-stops.md §4). It fed
// App::Drive's interim Motion::WheelVelocityPid pair, which is gone: Drive
// is open-loop duty from calibrated speed and holds no controller at all
// (drive.h's own file header). The boot JSON's vel_gains reach the one
// controller that still exists -- Motion::Planner's duty stage -- through
// App::Configurator's pid.* wire keys, not through this seeding path.


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
  // Wheel calibration comes from the ROBOT JSON, never from C++
  // (command-ingestion-ring-buffered-comms-subsystem-routing-two-stops.md
  // §6): App::Drive carries no calibration defaults, and without this
  // install it refuses to drive. gen_boot_config.py fails the build outright
  // if the active robot's JSON is missing any of the three keys, so
  // reaching here means they are real, measured, per-robot numbers.
  {
    const Config::DriveBootConfig driveConfig = Config::defaultDriveConfig();
    drive.setDutyPerSpeed(driveConfig.dutyPerSpeedLeft, driveConfig.dutyPerSpeedRight);
    drive.setWheelCorrection(
        driveConfig.gainLeftAccel, driveConfig.interceptLeftAccel,
        driveConfig.gainLeftDecel, driveConfig.interceptLeftDecel,
        driveConfig.gainRightAccel, driveConfig.interceptRightAccel,
        driveConfig.gainRightDecel, driveConfig.interceptRightDecel);
    drive.setCrawlPulse(driveConfig.crawlPulse);
  }
  static Motion::Odometry odom(drivetrainConfig.trackwidth, motorL.position(), motorR.position());


#ifdef FAKE_OTOS
  static App::FakeOtos fakeOtos(odom, motorL, motorR, drivetrainConfig.trackwidth);
  Devices::Otos& otos = fakeOtos;
#else
  Devices::Otos& otos = realOtos;
#endif

  Config::EstimatorBootConfig estimatorBootConfig = Config::defaultEstimatorConfig();
  static Motion::StateEstimator stateEstimator(toFusionWeights(estimatorBootConfig));


  // Planner integration (2026-07-26): the on-robot Motion::Planner replaces
  // Motion::MoveQueue as the loop's motion decider. Limits assembled from
  // the SAME boot-config sources the old stack used: shaper keys ->
  // profile ceilings, vel_gains -> the planner's own duty-stage PID,
  // vel_filt_alpha -> the planner's velocity-filter weight. controlPeriod
  // is the loop's own kCycle; actuationDelay is the one-cycle staging
  // latency (duty staged at the NEXT cycle top -- the exact shape the
  // planner's duty scenario tier validates).
  // Planner tuning: the PLANT-VALIDATED set from the measured-constants
  // reference tour (motion checkout, square_tour_sim.py tourLimits() --
  // plant ID 2026-07-26: gain ~1370 mm/s per duty, tau ~230 ms), NOT the
  // boot JSON's vel_gains/shaper block: those numbers were bench-tuned
  // for the DELETED NezhaMotor MotorVelocityPid (the JSON's own
  // _vel_gains_domain note) and are wrong for this loop -- deployed as-is
  // (kp 0.0016, aMax 800, jMax 5000) they limit-cycled the real wheels at
  // ~2-3 Hz across the whole first on-robot tour (2026-07-27 plot). A
  // planner-domain config surface can supersede these constants later;
  // until then the JSON's old-loop gains must not reach this controller.
  Motion::PlannerLimits plannerLimits;
  plannerLimits.vMax = 400.0f;   // [mm/s]
  plannerLimits.aMax = 300.0f;   // [mm/s^2]
  plannerLimits.aDecel = 250.0f; // [mm/s^2]
  plannerLimits.omegaMax = 3.0f;     // [rad/s]
  plannerLimits.alphaMax = 6.0f;     // [rad/s^2]
  plannerLimits.alphaDecel = 5.0f;   // [rad/s^2]
  plannerLimits.jerkMax = 1500.0f;   // [mm/s^3] aMax reached in ~0.2 s
  plannerLimits.yawJerkMax = 30.0f;  // [rad/s^3] alphaMax reached in ~0.2 s
  plannerLimits.trackWidth = drivetrainConfig.trackwidth;
  // MEASURED loop period, not the kCycle nominal: the schedule's real
  // delivered cycle is 46-48 ms on the bench (tlm cycle-delta capture,
  // 2026-07-27, after the 1 ms scheduler tick + overrun-yield fixes) --
  // the vendor bus clearances outside the paced windows add ~7 ms the
  // nominal does not include. The planner's discrete math (accel steps,
  // braking sums, ramp tick counts) must use the period the loop actually
  // delivers; telemetry cycle_period re-measures this on every frame if
  // the schedule ever changes.
  plannerLimits.controlPeriod = 47.0f;   // [ms]
  plannerLimits.actuationDelay = 47.0f;  // [ms]
  plannerLimits.velocityFilterWeight =
      drivetrainConfig.vel_filt_alpha > 0.05f ? drivetrainConfig.vel_filt_alpha
                                              : 1.0f;
  // Settle-confirm OFF (stakeholder 2026-07-27): the creep/breakaway-kick
  // landing machinery pulsed the wheels at ~0.18 duty against gearbox
  // stiction after every move -- functional but unacceptable sawtooth.
  // Landing residual (small at the measured 47 ms period) is instead
  // absorbed by the NEXT chained move via the cumulative baseline ledger,
  // at full speed where the plant is linear and no stiction compensation
  // is needed. The last move of a chain keeps its own small residual.
  plannerLimits.requireSettle = false;
  // Rest floors sized to the measured hardware encoder-velocity noise
  // (plant ID 2026-07-26: ~+-7 mm/s at rest) -- the rest-damping stage
  // outputs exactly zero duty below the floor, so it must clear the
  // noise band or the wheels twitch at rest forever.
  plannerLimits.settleRestVelocity = 10.0f;  // [mm/s]
  plannerLimits.settleRestOmega = 0.16f;     // [rad/s] 2*floor/trackWidth
  plannerLimits.settleWindow = 2500.0f;  // [ms]
  plannerLimits.headingHoldGain = 2.0f;  // [1/s] sim-validated
  {
    // Measured-plant duty-stage gains (square_tour_sim.py tourLimits()):
    // kff = 1/gain, kaff = tau/gain -- physics-derived, per-robot only
    // through the plant measurement, not the JSON's old-loop vel_gains.
    constexpr float kPlantGain = 1370.0f;  // [mm/s per duty]
    constexpr float kPlantTau = 0.23f;     // [s]
    plannerLimits.velKff = 1.0f / kPlantGain;
    plannerLimits.velKp = 0.0009f;
    plannerLimits.velKi = 0.004f;
    plannerLimits.velIMax = 0.25f;
    plannerLimits.velKaff = kPlantTau / kPlantGain;
    plannerLimits.velIAccelGate = 50.0f;  // [mm/s^2]
    // Stiction floor: must clear the gearbox BREAKAWAY duty, not merely
    // MotorArmor's write-suppression threshold (output_deadband 0.03 --
    // a whine gate, not a friction model). 0.05 is estimated from the
    // measured integral wind-up time to first motion during the stalled
    // settle creep (2026-07-27 single-turn trace); replace with a proper
    // plant-ID breakaway measurement when one exists.
    plannerLimits.dutyFloor = 0.18f;
    // Arrival tolerances sized to the stiction-limited creep resolution
    // (one dutyFloor pulse per period ~= 2-4 deg of heading): tighter is
    // unreachable and just burns the settle window at every landing.
    plannerLimits.settleEpsilonLinear = 4.0f;      // [mm]
    plannerLimits.settleEpsilonAngular = 0.035f;   // [rad] ~2 deg --
    // the demonstrated one-sided-creep resolution (single-turn probe
    // 2026-07-27 landed +1.76 deg; the right wheel does not reverse
    // under the breakaway kick, so fine correction rides the left wheel)
  }
  static Motion::Planner planner(plannerLimits);
  // Mark shaping CONFIGURED through the same applyShaperLimits() entry the
  // wire push uses, with the validated ceilings above (NOT the boot JSON's
  // old-shaper block -- see the tuning comment) so
  // kFlagFaultShapingDisabled (119-001) stays quiet.
  planner.applyShaperLimits(plannerLimits.aMax, plannerLimits.aDecel,
                            plannerLimits.alphaMax, plannerLimits.alphaDecel,
                            plannerLimits.jerkMax, plannerLimits.yawJerkMax);
  static App::Preamble preamble(motorL, motorR, otos, color, line, clock);


  static Config::MicroBitTuningStore tuningStore(uBit.storage);


  // App::Configurator owns the CONFIG lifecycle and the persisted-tuning
  // store (command-ingestion-...-two-stops.md §6); RobotLoop routes to it.
  static App::Configurator configurator(drive, motorL, motorR, otos, planner,
                                        stateEstimator, &tuningStore);

  static App::RobotLoop robotLoop(bus, motorL, motorR, otos, color, line,
                                   comms, tlm, drive, configurator, odom,
                                   planner, preamble, stateEstimator, clock,
                                   sleeper);

  uint32_t storedVersion = 0;
  Config::Blob storedBlob{};
  bool storeHadData = tuningStore.load(&storedVersion, &storedBlob);
  if (storeHadData && !Config::shouldWipe(storedVersion, Config::kConfigSchemaVersion)) {
    configurator.reapplyPersistedTuning(Config::deserializeSnapshot(storedBlob));
  } else if (storeHadData) {
    tuningStore.wipe();
  }

  robotLoop.markConfigured();


  robotLoop.run();
}
