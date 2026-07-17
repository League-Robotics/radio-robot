// main.cpp -- the ARM entry point. Owns the real MicroBit hardware singleton,
// constructs and wires every leaf/app module, then hands off to
// App::RobotLoop (app/robot_loop.{h,cpp}) for the boot loop + main cycle.
// No cycle logic lives here. Design/rationale: DESIGN.md.
#include <cstdio>

#include "MicroBit.h"

#include "app/comms.h"
#include "app/deadman.h"
#include "app/drive.h"
#include "app/odometry.h"
#include "app/pilot.h"
#include "app/preamble.h"
#include "app/robot_loop.h"
#include "app/telemetry.h"
#include "com/radio.h"
#include "com/serial_port.h"
#include "config/boot_config.h"
#include "devices/color_sensor.h"
#include "devices/device_config.h"
#include "devices/i2c_bus.h"
#include "devices/microbit_clock.h"
#include "devices/microbit_i2c_bus.h"
#include "devices/line_sensor.h"
#include "devices/nezha_motor.h"
#include "devices/otos.h"
#include "motion/executor.h"

static MicroBit uBit;

namespace {

// DEVICE:NEZHA2:robot:<name>:<serial> -- byte-frozen wire format; host
// banner parsers depend on it.
void formatBanner(char* buf, int size) {
  const char* name = microbit_friendly_name();
  uint32_t serial = microbit_serial_number();
  snprintf(buf, size, "DEVICE:NEZHA2:robot:%s:%lu", name,
            static_cast<unsigned long>(serial));
}

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

  // Construction order: bus before leaves, leaves before app/ modules that
  // read them (DESIGN.md §4).
  static Devices::MicroBitI2CBus bus(uBit.i2c);

  msg::MotorConfig motorConfigs[Config::kMotorConfigCount];
  Config::defaultMotorConfigs(motorConfigs);
  msg::DrivetrainConfig drivetrainConfig = Config::defaultDrivetrainConfig();
  Config::OtosBootConfig otosBootConfig = Config::defaultOtosBootConfig();

  // left_port/right_port are 1-based port labels (boot_config.h's
  // convention) -> 0-based index into the motorConfigs array.
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

  static Devices::MicroBitClock clock;
  static Devices::MicroBitSleeper sleeper;

  static App::SerialTransport serialLink(serial);
  static App::RadioTransport radioLink(radio);
  static App::Comms comms(serialLink, radioLink, banner);
  static App::Telemetry tlm(comms, serialLink, radioLink);
  static App::Deadman deadman(clock);
  static App::Drive drive(motorL, motorR, drivetrainConfig.trackwidth);
  static App::Odometry odom(motorL, motorR, drivetrainConfig.trackwidth);
  static App::Preamble preamble(motorL, motorR, otos, color, line, clock);

  // Motion::Executor + App::Pilot (109-003) -- configured from the same
  // boot PlannerConfig defaults the pre-rebuild segment executor used
  // (Config::defaultPlannerConfig(), config/boot_config.h).
  static Motion::Executor executor;
  executor.configure(Config::defaultPlannerConfig());
  static App::Pilot pilot(executor, drive);

  // Boot loop + main cycle -- takes every leaf/app module above by
  // reference plus the Clock/Sleeper time seam. run() never returns.
  static App::RobotLoop robotLoop(bus, motorL, motorR, otos, comms, tlm,
                                   drive, odom, deadman, preamble, pilot,
                                   clock, sleeper);
  robotLoop.run();
}
