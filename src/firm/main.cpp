// main.cpp -- the ARM entry point. Owns the real MicroBit hardware singleton,
// constructs and wires every leaf/app module, then hands off to
// App::RobotLoop (app/robot_loop.{h,cpp}) for the boot loop + main cycle.
// No cycle logic lives here. Design/rationale: DESIGN.md.
#include "MicroBit.h"

#include <cstdio>
#include <cstring>

#include "app/comms.h"
#include "app/debug.h"
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
// vel_gains/vel_filt_alpha/min_duty are NOT copied here -- Devices::
// MotorConfig has no such fields (the velocity PID they fed has been
// deleted outright -- see drive.h's own header). vel_filt_alpha has no
// live consumer at all -- the wire field itself is untouched (no
// protocol change), simply unread by firmware for now.
Devices::MotorConfig toDeviceMotorConfig(const msg::MotorConfig& src) {
  Devices::MotorConfig cfg;
  cfg.wheelTravelCalib = src.travel_calib;
  cfg.fwdSign = src.fwd_sign;
  cfg.slewRate = src.slew_rate;
  cfg.port = src.port;
  // Devices::MotorConfig's reversalDwell/outputDeadband are plain required
  // floats -- gen_boot_config.py's required-key gate guarantees
  // src.reversal_dwell/src.output_deadband are always set (.has == true)
  // here, so read .val directly rather than changing the wire
  // msg::MotorConfig itself (still Opt<float> -- the wire schema is out
  // of scope here).
  cfg.reversalDwell = src.reversal_dwell.val;
  cfg.outputDeadband = src.output_deadband.val;
  cfg.polled = src.polled;
  return cfg;
}

// Converts the boot config's fail-closed Config::PlannerBootConfig (baked
// from the active robot JSON's `planner` block) into the
// Motion::PlannerLimits Motion::Planner's constructor needs. Lives here
// for the same reason toDeviceMotorConfig() above does -- main.cpp is the
// one place both types are reachable (config/ may depend only on
// messages/, never on src/motion; see PlannerBootConfig's own doc
// comment, config/boot_config.h).
//
// Deliberately leaves trackWidth/velocityFilterWeight UNSET on the
// returned struct -- both are sourced from DrivetrainConfig at the call
// site below.
Motion::PlannerLimits toPlannerLimits(const Config::PlannerBootConfig& src) {
  Motion::PlannerLimits out;
  out.vMax = src.vMax;
  out.aMax = src.aMax;
  out.aDecel = src.aDecel;
  out.omegaMax = src.omegaMax;
  out.alphaMax = src.alphaMax;
  out.alphaDecel = src.alphaDecel;
  out.jerkMax = src.jerkMax;
  out.yawJerkMax = src.yawJerkMax;

  out.controlPeriod = src.controlPeriod;
  out.actuationDelay = src.actuationDelay;

  out.requireSettle = src.requireSettle;
  out.settleRestVelocity = src.settleRestVelocity;
  out.settleRestOmega = src.settleRestOmega;
  out.settleWindow = src.settleWindow;
  out.settleEpsilonLinear = src.settleEpsilonLinear;
  out.settleEpsilonAngular = src.settleEpsilonAngular;
  out.headingHoldGain = src.headingHoldGain;

  out.velKff = src.velKff;
  out.velKp = src.velKp;
  out.velKi = src.velKi;
  out.velIMax = src.velIMax;
  out.velKaff = src.velKaff;
  out.velIAccelGate = src.velIAccelGate;
  out.dutyFloor = src.dutyFloor;

  out.trimKp = src.trimKp;
  out.trimKi = src.trimKi;
  out.trimIMax = src.trimIMax;
  out.trimKaff = src.trimKaff;
  out.trimMax = src.trimMax;
  out.decelPlanFraction = src.decelPlanFraction;
  return out;
}

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

  // ID:<drivetrain>:<profile>:<version> -- configured-robot identity
  // (drivetrain type + calibration-profile name/version), distinct from
  // `banner`'s hardware identity. Both Config::kDrivetrainType and
  // Config::kRobotProfileName are generated string constants
  // (boot_config.h) baked from the robot JSON's own
  // identity.drivetrain_type/filename stem -- NOT derived from any
  // wire-level DrivetrainConfig field (defaultDrivetrainConfig() never
  // bakes half_track; it stays at its wire default 0.0f for every
  // profile, so it cannot answer this question -- see kDrivetrainType's
  // own doc comment). The version reuses VER:'s own generated
  // build-version constant.
  static char idLine[96];
  snprintf(idLine, sizeof(idLine), "ID:%s:%s:%s", Config::kDrivetrainType,
           Config::kRobotProfileName, FIRMWARE_VERSION_STR);

  static App::SerialTransport serialLink(serial);
  static App::RadioTransport radioLink(radio);
  static App::Comms comms(serialLink, radioLink, banner, idLine);
  // Wires the bench/Sim-only DBG debug channel to this robot's own Comms
  // -- a no-op call unless ROBOT_DEBUG is defined (app/debug.h's own
  // compile gate), so a shipped ARM release build never even calls this.
  App::setDebugSink(&comms);
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
  // Wheel calibration comes from the ROBOT JSON, never from C++:
  // App::Drive carries no calibration defaults, and without this
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

  // Motion::Planner is the on-robot loop's motion decider, writing
  // Types::RobotState::Wheel::cmdVelocity directly (robot_state.h's own
  // field doc).
  //
  // PlannerLimits below is the plant-validated tuning from the
  // measured-constants reference tour (motion checkout,
  // square_tour_sim.py tourLimits()), deliberately NOT the boot JSON's
  // old-loop control.vel_gains/shaper block: those numbers were bench-tuned
  // for the DELETED NezhaMotor MotorVelocityPid and the also-deleted
  // Motion::VelocityShaper. ERRATUM: deployed as-is against THIS
  // controller (kp 0.0016, aMax 800, jMax 5000), they limit-cycled the
  // real wheels at ~2-3 Hz across the whole first on-robot tour -- do not
  // resurrect those values for this controller. The validated tuning
  // lives in the active robot JSON's own `planner` section
  // (data/robots/robot_config.schema.json), baked fail-closed via
  // Config::defaultPlannerLimits(). Full measurement provenance (plant ID
  // dates, sweep results, the limit-cycle warning) lives in that JSON's
  // own planner._domain_note/_timing_note/_settle_note/_duty_stage_note/
  // _trim_note.
  //
  // trackWidth/velocityFilterWeight are the two PlannerLimits fields NOT
  // sourced from the `planner` block -- they come from DrivetrainConfig
  // (trackWidth is the scrub-corrected kEffectiveTrack computed above;
  // velocityFilterWeight mirrors the same EMA weight the old stack used,
  // vel_filt_alpha, with the same >0.05 sanity floor).
  Motion::PlannerLimits plannerLimits = toPlannerLimits(Config::defaultPlannerLimits());
  plannerLimits.trackWidth = kEffectiveTrack;
  plannerLimits.velocityFilterWeight =
      drivetrainConfig.vel_filt_alpha > 0.05f ? drivetrainConfig.vel_filt_alpha
                                              : 1.0f;
  static Motion::Planner planner(plannerLimits);
  // Mark shaping CONFIGURED through the same applyShaperLimits() entry the
  // wire push uses, with the validated ceilings above (NOT the boot JSON's
  // old-shaper block -- see the doc comment above) so
  // kFlagFaultShapingDisabled stays quiet.
  planner.applyShaperLimits(plannerLimits.aMax, plannerLimits.aDecel,
                            plannerLimits.alphaMax, plannerLimits.alphaDecel,
                            plannerLimits.jerkMax, plannerLimits.yawJerkMax);
  static App::Preamble preamble(motorL, motorR, otos, color, line, clock);


  static Config::MicroBitTuningStore tuningStore(uBit.storage);


  // App::Configurator owns the CONFIG lifecycle and the persisted-tuning
  // store; RobotLoop routes to it.
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
