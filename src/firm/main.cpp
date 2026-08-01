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
#include "motion/planner/planner.h"
#include "motion/odometry.h"
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
// App::Drive's interim per-wheel closed-loop velocity-PID pair, which is
// gone (128-015 deleted that class outright, zero instantiations -- see
// src/motion/DESIGN.md's "wheel control generations" note): Drive is
// open-loop duty from calibrated speed and holds no controller at all
// (drive.h's own file header). The boot JSON's vel_gains reach the one
// controller that still exists -- Motion::Planner's duty stage -- through
// App::Configurator's pid.* wire keys, not through this seeding path.


// toFusionWeights() -- DELETED (128-016,
// robot-state-pose-needs-exactly-one-writer.md): its one consumer,
// Motion::StateEstimator's constructor, is gone (a per-cycle computation
// with no consumer -- its own former header said so). Config::
// EstimatorBootConfig/Config::defaultEstimatorConfig() themselves are
// UNTOUCHED here (out of this ticket's scope -- see boot_config.h's own
// doc comment): they still bake fail-closed fusion-weight defaults from
// each robot JSON for a future estimator rebuild
// (clasi/issues/estimator-v2-otos-fusion-sim-first.md) to read; this
// file simply no longer has anywhere to feed the result.

// The boot tag: the DAY of the version's date field, then the build number.
// "0.20260726.1" -> "261" (day 26, build 1).
//
// A single trailing digit cannot distinguish two builds made on different
// days -- the board showed "1" for both 0.20260726.1 and 0.20260729.1, which
// is exactly the confusion that costs bench time. Day+build stays short
// enough to read off the matrix at a glance while being unique across any
// plausible session (stakeholder directive 2026-07-29).
//
// Parsed rather than indexed at fixed offsets, so it survives a format change
// in gen_version.py; emits "?" if the string lacks the two dots the
// major.date.build shape requires.
void versionTag(char* out, size_t cap) {
  if (cap == 0) return;
  const char* firstDot = nullptr;
  const char* lastDot = nullptr;
  for (const char* p = FIRMWARE_VERSION_STR; *p != '\0'; ++p) {
    if (*p == '.') {
      if (firstDot == nullptr) firstDot = p;
      lastDot = p;
    }
  }
  size_t n = 0;
  // Two distinct dots, and a date field of at least two characters to take a
  // day from.
  if (firstDot != nullptr && lastDot != firstDot && (lastDot - firstDot) >= 3) {
    if (n + 1 < cap) out[n++] = *(lastDot - 2);
    if (n + 1 < cap) out[n++] = *(lastDot - 1);
    for (const char* p = lastDot + 1; *p != '\0' && n + 1 < cap; ++p) out[n++] = *p;
  }
  if (n == 0 && cap > 1) out[n++] = '?';
  out[n] = '\0';
}

// Boot identity on the LED matrix: heart, the last digit of the firmware
// version, then the heart again -- and the heart STAYS LIT.
//
// This is the only way to tell WHICH build is actually on the board without
// opening a serial session, and "did that flash land?" is a question that
// has cost real debugging time. The trailing heart is the resting state: a
// lit display through boot means "powered, flashed, and running"; a dark
// one means it never got here.
//
// It is not left on forever. main() disables the display the instant boot
// completes and before the first control cycle: the LED matrix driver
// refreshes continuously off its own timer, and the loop runs to a measured
// ~47 ms budget whose motion tuning assumes those cycles are not being
// spent elsewhere. Boot is the one window where the cost is free.
void showBootIdentity() {
  // Pixel values are BRIGHTNESS 0..255, not booleans -- a 1 here renders
  // at 1/255 duty, i.e. invisibly dim next to the font's full-bright
  // digit (stakeholder observed "only the 1, no hearts", 2026-07-27).
  constexpr uint8_t kOn = 255;
  static const uint8_t kHeart[] = {
      0,   kOn, 0,   kOn, 0,
      kOn, kOn, kOn, kOn, kOn,
      kOn, kOn, kOn, kOn, kOn,
      0,   kOn, kOn, kOn, 0,
      0,   0,   kOn, 0,   0,
  };
  constexpr int kHeartHold = 500;  // [ms]
  constexpr int kDigitHold = 700;  // [ms] per digit -- it is the payload
  constexpr int kDigitGap = 150;   // [ms] blank between digits, so a repeated
                                   // digit ("22") reads as two, not one long

  uBit.display.enable();
  MicroBitImage heart(5, 5, kHeart);
  uBit.display.print(heart);
  uBit.sleep(kHeartHold);
  uBit.display.clear();

  char tag[8];
  versionTag(tag, sizeof(tag));
  for (const char* p = tag; *p != '\0'; ++p) {
    uBit.display.printChar(*p);
    uBit.sleep(kDigitHold);
    uBit.display.clear();
    uBit.sleep(kDigitGap);
  }

  uBit.display.print(heart);  // resting state -- left lit through boot
}

}  // namespace

int main() {
  uBit.init();

  // Before anything touches the buses: say which build this is.
  showBootIdentity();

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
  // Effective track width = physical separation corrected for SCRUB.
  //
  // Ideal differential kinematics say omega = (vR - vL) / b, but a skid-steer
  // robot drags its wheels sideways through a turn and rotates LESS than that
  // for a given wheel differential. rotational_slip is the measured ratio of
  // actual to ideal rotation, so every kinematic use of the track wants
  // b / slip, not b.
  //
  // `trackwidth` stays the caliper-measured wheel separation and must NOT be
  // bent to absorb scrub -- that would destroy the one value in the robot
  // JSON that is independently verifiable, and hide the scrub instead of
  // measuring it.
  //
  // slip == 0 is the "uncalibrated" sentinel (config.proto's `{0} u
  // [0.5, 1.0]` domain), meaning apply no correction.
  const float kScrub = drivetrainConfig.rotational_slip;
  const float kEffectiveTrack =
      (kScrub > 0.0f) ? (drivetrainConfig.trackwidth / kScrub)
                      : drivetrainConfig.trackwidth;

  static App::Drive drive(motorL, motorR, kEffectiveTrack);
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
  static Motion::Odometry odom(kEffectiveTrack, motorL.position(), motorR.position());


#ifdef FAKE_OTOS
  static App::FakeOtos fakeOtos(odom, motorL, motorR, kEffectiveTrack);
  Devices::Otos& otos = fakeOtos;
#else
  Devices::Otos& otos = realOtos;
#endif

  // Planner integration (2026-07-26): the on-robot Motion::Planner is the
  // loop's motion decider, writing Types::RobotState::Wheel::cmdVelocity
  // directly (robot_state.h's own field doc). Limits assembled from
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
  plannerLimits.trackWidth = kEffectiveTrack;
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

    // VELOCITY-DOMAIN TRIM (wheel_trim.h) -- the closed loop that actually
    // reaches the wheels. The loop's one actuation contract is a wheel
    // VELOCITY (RobotState::Wheel::cmdVelocity), which App::Drive converts
    // through its measured per-wheel per-direction map. The duty-stage
    // gains above configure Planner::stageDuty() -- PARKED as of 128-015:
    // Planner::tick() no longer calls it at all (it used to run every
    // cycle and its output was DISCARDED here regardless, since nothing on
    // this robot ever read commandedDutyLeft/Right()) -- see
    // src/motion/DESIGN.md's "wheel control generations" note.
    //
    // COMMISSIONING VALUES, to be raised on the stand rather than trusted:
    //   trimKp is DIMENSIONLESS (mm/s of trim per mm/s of error). The sim
    //     tour ran 0.25 against a +-4.8% wheel mismatch; this robot's
    //     residual after the 2026-07-27 calibration is ~2%, and measured
    //     wheel velocity is a raw per-tick difference quotient, so start
    //     at 0.15. An over-gained loop on THIS hardware limit-cycled at
    //     2-3 Hz (see the tuning note above) -- the failure to watch for.
    //   trimKaff is the plant time constant [s]. FULL tau measured
    //     UNSTABLE in sim (closure 696 mm vs 16 mm at half) -- half it is.
    plannerLimits.trimKp = 0.15f;            // [1]
    plannerLimits.trimKi = 0.4f;             // [1/s]
    plannerLimits.trimIMax = 40.0f;          // [mm/s]
    plannerLimits.trimKaff = kPlantTau / 2;  // [s]
    plannerLimits.trimMax = 80.0f;           // [mm/s]
    // Plan the brake START at 40% of the decel ceiling, keeping the rest
    // in reserve so the plant can TRACK the ramp down and arrives at each
    // boundary already slow instead of coasting past (tau ~230 ms).
    // Swept 0.0-0.6 in sim: 0.4 gave the best closure (16.0 vs 23.3 mm).
    plannerLimits.decelPlanFraction = 0.4f;  // [1]
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
                                        &tuningStore);

  static App::RobotLoop robotLoop(bus, motorL, motorR, otos, color, line,
                                   comms, tlm, drive, configurator, odom,
                                   planner, preamble, clock,
                                   sleeper);

  // Turn calibration from the robot JSON. Degrees on the wire/JSON side (what
  // a human reads and what the camera measurement produced), radians inside,
  // matching Motion::Move::threshold. See RobotLoop::setRotationCalibration().
  {
    constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
    robotLoop.setRotationCalibration(
        drivetrainConfig.rotation_gain_pos,
        drivetrainConfig.rotation_offset * kDegToRad,
        drivetrainConfig.rotation_gain_neg,
        drivetrainConfig.rotation_offset_neg * kDegToRad);
  }

  uint32_t storedVersion = 0;
  Config::Blob storedBlob{};
  bool storeHadData = tuningStore.load(&storedVersion, &storedBlob);
  if (storeHadData && !Config::shouldWipe(storedVersion, Config::kConfigSchemaVersion)) {
    configurator.reapplyPersistedTuning(Config::deserializeSnapshot(storedBlob));
  } else if (storeHadData) {
    tuningStore.wipe();
  }

  robotLoop.markConfigured();

  // RobotLoop::run() is boot() followed by cycle() forever; it is spelled
  // out here instead so the display can be turned off in between. Both
  // methods are already public and this changes neither -- the loop's own
  // behavior is byte-identical to run().
  robotLoop.boot();

  // Boot is done and the first control cycle is next: give the LED matrix's
  // refresh timer back to the loop. Everything from here runs to a measured
  // ~47 ms budget (see PlannerLimits::controlPeriod), and the motion tuning
  // assumes those cycles are the loop's.
  uBit.display.clear();
  uBit.display.disable();

  for (;;) {
    robotLoop.cycle();
  }
}
