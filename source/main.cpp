// ---------------------------------------------------------------------------
// main.cpp -- the ARM entry point. Owns the real MicroBit hardware singleton
// and constructs every leaf/app module, then hands off to App::RobotLoop
// (source/app/robot_loop.{h,cpp}) for the boot loop + main cycle body.
//
// Sprint 103 ticket 008 originally built the single loop inline here (see
// git history for that shape: a telemetry-emitting boot loop, then the
// runAndWait/markTime/sleepUntil main cycle, wired per the archived plan's
// canonical "The main loop (the whole program, one page)" sketch). Sprint
// 105 ticket 001 extracted that boot loop and cycle body verbatim into
// App::RobotLoop, parameterized on Devices::Clock&/Devices::Sleeper&
// instead of raw vendor timer/sleep calls, so it compiles under
// -DHOST_BUILD with no MicroBit.h dependency -- a mechanical move, zero
// intended behavior change on ARM (robot_loop.h/.cpp carry the full
// rationale and the preserved inline documentation). This file's own
// remaining logic is limited to real hardware construction and wiring --
// no cycle logic remains inline here.
// ---------------------------------------------------------------------------
#include <cstdio>

#include "MicroBit.h"

#include "app/comms.h"
#include "app/deadman.h"
#include "app/drive.h"
#include "app/odometry.h"
#include "app/preamble.h"
#include "app/robot_loop.h"
#include "app/telemetry.h"
#include "com/radio.h"
#include "com/serial_port.h"
#include "config/boot_config.h"
#include "devices/clock.h"
#include "devices/color_sensor.h"
#include "devices/device_config.h"
#include "devices/i2c_bus.h"
#include "devices/microbit_i2c_bus.h"
#include "devices/line_sensor.h"
#include "devices/nezha_motor.h"
#include "devices/otos.h"

static MicroBit uBit;

namespace {

// DEVICE:NEZHA2:robot:<name>:<serial> -- byte-identical to the sprint-102
// stub's own formatDeviceAnnouncement() (main.cpp git history) so a host
// client's existing banner parser keeps working unchanged.
void formatBanner(char* buf, int size) {
  const char* name = microbit_friendly_name();
  uint32_t serial = microbit_serial_number();
  snprintf(buf, size, "DEVICE:NEZHA2:robot:%s:%lu", name,
            static_cast<unsigned long>(serial));
}

// Converts the boot config's wire-plane msg::MotorConfig into the
// Devices-local MotorConfig NezhaMotor's constructor needs. Lives here (not
// in source/devices/ or source/config/) because it is the one place both
// types are reachable: the isolation invariant forbids source/devices/ from
// including messages/ or config/ (device_config.h's own file header), and
// config/boot_config.h has no reason to know Devices:: exists.
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
  cfg.reversalDwell.has = src.reversal_dwell.has;
  cfg.reversalDwell.val = src.reversal_dwell.val;
  cfg.outputDeadband.has = src.output_deadband.has;
  cfg.outputDeadband.val = src.output_deadband.val;
  cfg.polled = src.polled;
  return cfg;
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

  // ---- Construction order matches device_bus.h's own documented
  // rationale (bus before leaves, leaves before app/ modules that read
  // them) even though DeviceBus itself is gone -- this ticket's own
  // acceptance criterion. ----
  static Devices::MicroBitI2CBus bus(uBit.i2c);

  msg::MotorConfig motorConfigs[Config::kMotorConfigCount];
  Config::defaultMotorConfigs(motorConfigs);
  msg::DrivetrainConfig drivetrainConfig = Config::defaultDrivetrainConfig();
  Config::OtosBootConfig otosBootConfig = Config::defaultOtosBootConfig();

  // left_port/right_port are 1-based port labels (boot_config.h's own
  // convention, tovez.json: left_port=1, right_port=2) -> 0-based index
  // into the 4-entry motorConfigs array.
  static Devices::NezhaMotor motorL(
      bus, toDeviceMotorConfig(motorConfigs[drivetrainConfig.left_port - 1]));
  static Devices::NezhaMotor motorR(
      bus, toDeviceMotorConfig(motorConfigs[drivetrainConfig.right_port - 1]));

  Devices::OtosConfig otosConfig;
  otosConfig.offsetX = otosBootConfig.offsetX;
  otosConfig.offsetY = otosBootConfig.offsetY;
  otosConfig.offsetYaw = otosBootConfig.offsetYaw;
  otosConfig.linearScale = otosBootConfig.linearScale;
  otosConfig.angularScale = otosBootConfig.angularScale;
  static Devices::Otos otos(bus, otosConfig);

  static Devices::ColorConfig colorConfig;
  static Devices::ColorSensorLeaf color(bus, colorConfig);
  static Devices::LineConfig lineConfig;
  static Devices::LineSensorLeaf line(bus, lineConfig);

  static Devices::Clock clock;
  static Devices::Sleeper sleeper;

  static App::SerialTransport serialLink(serial);
  static App::RadioTransport radioLink(radio);
  static App::Comms comms(serialLink, radioLink, banner);
  static App::Telemetry tlm(comms, serialLink, radioLink);
  static App::Deadman deadman(clock);
  static App::Drive drive(motorL, motorR, drivetrainConfig.trackwidth);
  static App::Odometry odom(motorL, motorR, drivetrainConfig.trackwidth);
  static App::Preamble preamble(motorL, motorR, otos, color, line, clock);

  // The extracted boot loop + main cycle body (sprint 105 ticket 001,
  // source/app/robot_loop.{h,cpp}) -- takes every leaf/app module above by
  // reference plus the Clock/Sleeper time seam. run() never returns.
  static App::RobotLoop robotLoop(bus, motorL, motorR, otos, comms, tlm,
                                   drive, odom, deadman, preamble, clock,
                                   sleeper);
  robotLoop.run();
}
