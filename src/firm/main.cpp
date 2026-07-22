// main.cpp -- the ARM entry point. Owns the real MicroBit hardware singleton,
// constructs and wires every leaf/app module, then hands off to
// App::RobotLoop (app/robot_loop.{h,cpp}) for the boot loop + main cycle.
// No cycle logic lives here. Design/rationale: DESIGN.md.
#include <cstdio>

#include "MicroBit.h"

#include "app/comms.h"
#include "app/drive.h"
#include "app/move_queue.h"
#include "app/odometry.h"
#include "app/preamble.h"
#include "app/robot_loop.h"
#include "app/telemetry.h"
#include "com/radio.h"
#include "com/serial_port.h"
#include "config/boot_config.h"
#include "config/persisted_tuning.h"
#include "devices/color_sensor.h"
#include "devices/device_config.h"
#include "devices/i2c_bus.h"
#include "devices/microbit_clock.h"
#include "devices/microbit_i2c_bus.h"
#include "devices/line_sensor.h"
#include "devices/motor_armor.h"
#include "devices/nezha_motor.h"
#include "devices/otos.h"

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
  //
  // Composition (stakeholder 2026-07-18, motor.h): construct the bare
  // NezhaMotor, wrap it in the MotorArmor decorator (wedge detection +
  // standstill-guarded resets), and hand the ARMOR to the app graph — the
  // ARM build always drives armored motors. The sim composes the bare
  // leaves directly (src/sim/sim_harness.h) — no armor in that loop.
  Devices::MotorConfig motorCfgL =
      toDeviceMotorConfig(motorConfigs[drivetrainConfig.left_port - 1]);
  Devices::MotorConfig motorCfgR =
      toDeviceMotorConfig(motorConfigs[drivetrainConfig.right_port - 1]);
  static Devices::NezhaMotor motorLBare(bus, motorCfgL);
  static Devices::NezhaMotor motorRBare(bus, motorCfgR);
  static Devices::MotorArmor motorL(motorLBare);
  static Devices::MotorArmor motorR(motorRBare);
  // REVISION 1 (114-001): configure() -> reconfigure() rename, discarding
  // the now-[[nodiscard]] bool. Always succeeds here (motorL/motorR are
  // freshly constructed, mode_ == Mode::None) -- pure rename, no
  // real-hardware behavior change (Decision 2's "always-immediate"
  // precedent).
  (void)motorL.reconfigure(motorCfgL);
  (void)motorR.reconfigure(motorCfgR);

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
  static App::Drive drive(motorL, motorR, drivetrainConfig.trackwidth);
  static App::Odometry odom(motorL, motorR, drivetrainConfig.trackwidth);
  // 116 (protocol-set-point issue): App::MoveQueue replaces App::Deadman --
  // constructed after drive/odom (it holds references to both, see
  // move_queue.h's own boundary comment: "no new dependency direction").
  static App::MoveQueue moveQueue(drive, odom, clock);
  static App::Preamble preamble(motorL, motorR, otos, color, line, clock);

  // 114-004 (SUC-003): the real ARM-only MicroBitStorage-backed persistence
  // adapter. Declared BEFORE robotLoop below -- RobotLoop only ever holds a
  // pointer to it, never owns it, so it must outlive robotLoop (both are
  // `static`, i.e. the whole program's lifetime, so this is really just
  // declaration-order bookkeeping, not a real lifetime risk).
  static Config::MicroBitTuningStore tuningStore(uBit.storage);

  // Boot loop + main cycle -- takes every leaf/app module above by
  // reference plus the Clock/Sleeper time seam, and (114-004) the
  // persisted-tuning store. run() never returns.
  static App::RobotLoop robotLoop(bus, motorL, motorR, otos, color, line,
                                   comms, tlm, drive, odom, moveQueue, preamble,
                                   clock, sleeper, &tuningStore);
  // Configuration-completeness gate (114-001): the boot-configure sequence
  // above (every Config::default*() call) is atomic and always complete by
  // this point on real firmware -- this call is unconditional and always
  // immediate, no observable startup delay (Decision 2, sprint.md).

  // 114-004 (SUC-003): persisted live-tuning read/wipe/reapply -- AFTER the
  // Tier-1 boot bake above (every Config::default*() call has already
  // completed) and BEFORE markConfigured() below, matching this ticket's
  // own Approach step 4 sequencing. A
  // version match reapplies whatever was live-tuned in a previous session,
  // through the SAME applier handleConfig() itself uses (no
  // partially-applied or misinterpreted stale patch); a version mismatch
  // wipes the ENTIRE store (SUC-003 -- not a partial/best-effort reapply of
  // a patch whose field meanings may have changed since the version that
  // wrote it). A store that was never written (first-ever boot) is left
  // alone -- nothing to wipe, nothing to reapply, proceeds on the boot-bake
  // values alone either way.
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
