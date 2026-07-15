// ---------------------------------------------------------------------------
// main.cpp -- the single loop (sprint 103, ticket 008). Replaces the
// sprint-102 banner-only stub with the real firmware: a telemetry-emitting
// boot loop, then the runAndWait/markTime/sleepUntil main cycle, wired per
// the archived plan's canonical shape (clasi/sprints/done/102-single-loop-
// firmware-spikes-archive-and-delete-to-stub-p0-p2/issues/single-loop-
// firmware-de-fiber-delete-the-elite-plumbing-telemetry-only-return-
// path.md, "The main loop (the whole program, one page)") using this
// sprint's ACTUAL class/method names (tickets 002-007), not the archived
// plan's illustrative naming. See architecture-update.md (103) Step 3 for
// each module's own boundary and Step 8 for why composition-only logic
// lives here.
//
// This ticket's own new logic is limited to two things (implementation
// plan): the runAndWait/markTime/sleepUntil timing primitives themselves,
// and the command-dispatch switch (deciding WHAT a decoded command does --
// no other module owns that decision). Everything else below is
// construction and a fixed call sequence.
// ---------------------------------------------------------------------------
#include <cstdio>

#include "MicroBit.h"

#include "app/comms.h"
#include "app/deadman.h"
#include "app/drive.h"
#include "app/odometry.h"
#include "app/preamble.h"
#include "app/telemetry.h"
#include "com/radio.h"
#include "com/serial_port.h"
#include "config/boot_config.h"
#include "devices/clock.h"
#include "devices/color_sensor.h"
#include "devices/device_config.h"
#include "devices/i2c_bus.h"
#include "devices/line_sensor.h"
#include "devices/nezha_motor.h"
#include "devices/otos.h"
#include "messages/envelope.h"

static MicroBit uBit;

namespace {

// --- Timing primitives (stakeholder-mandated shape, this ticket's own --
// small, mechanical, used nowhere else). runAndWait(gap, body) ==
// markTime(); body(); sleepUntil(mark, gap): the block visibly scopes
// exactly the work that borrows the wait; the body itself never touches
// the bus and never sleeps. `grep 'runAndWait\|sleepUntil'` on this file
// is therefore the firmware's complete timing schedule. Built directly on
// system_timer_current_time() (vendor SDK call, [ms]) + uBit.sleep() --
// the archived plan's own "Properties to reason from" paragraph says the
// main-loop settle waits themselves "yield via uBit.sleep", the same
// primitive the boot loop below uses.

uint32_t markTime() { return system_timer_current_time(); }  // [ms]

void sleepUntil(uint32_t mark, uint32_t gap) {  // [ms] [ms]
  uint32_t elapsed = system_timer_current_time() - mark;
  uint32_t remaining = (elapsed < gap) ? (gap - elapsed) : 0;
  // Always sleeps >=1ms -- never a zero-length "sleep" (that would be a
  // spin in disguise) and always a real yield back to the radio/serial
  // fibers, matching the archived plan's own "always sleeps >=1ms (radio
  // yield)" contract on the final pace call, applied uniformly here so no
  // runAndWait block can ever degrade into a busy-wait.
  uBit.sleep(remaining > 0 ? remaining : 1);
}

template <typename Body>
void runAndWait(uint32_t gap, Body body) {  // [ms]
  uint32_t mark = markTime();
  body();
  sleepUntil(mark, gap);
}

// --- Loop timing constants -- ported from the retired DeviceBus
// (device_bus.h, git history 88e04f1b^): kEncoderSettleMs = 4 (the vendor
// settle window between a motor's own request and collect, shared here by
// BOTH motors' settle windows) and the same 4ms clearance value NezhaMotor/
// Otos already use for every bus_.write()/bus_.read() postClear/preClear
// pair (nezha_motor.cpp's requestEncoder()/writeMotorRun(), otos.h's
// kBusClearance) for the new post-duty-write clearance window the archived
// plan's 3-block schedule adds. kCycle is the archived plan's own sketch
// comment ("sleepUntil(cycleStart, kCycle); // pace to ~16ms") verbatim --
// renamed per naming-and-style.md (no unit suffix in the identifier; unit
// in the trailing comment).
constexpr uint32_t kSettle = 4;   // [ms] encoder-settle window, both motors
constexpr uint32_t kClear = 4;    // [ms] post-duty-write clearance window
constexpr uint32_t kCycle = 16;   // [ms] cycle pace target (~60 Hz)
constexpr uint32_t kPreamblePace = 10;  // [ms] boot-loop probe pacing

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
  static Devices::I2CBus bus(uBit.i2c);

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

  static App::SerialTransport serialLink(serial);
  static App::RadioTransport radioLink(radio);
  static App::Comms comms(serialLink, radioLink, banner);
  static App::Telemetry tlm(comms, serialLink, radioLink);
  static App::Deadman deadman(clock);
  static App::Drive drive(motorL, motorR, drivetrainConfig.trackwidth);
  static App::Odometry odom(motorL, motorR, drivetrainConfig.trackwidth);
  static App::Preamble preamble(motorL, motorR, otos, color, line, clock);

  bool driving = false;  // true once a Twist is applied, cleared on Stop/deadman

  // Persists across cycles: each field is written by the part of the cycle
  // that owns it (encoder/vel/conn after motorL/motorR's own tick(); pose
  // after odom.integrate(); otos via applyOtosSample()) and read back
  // whole by the NEXT cycle's tlm.setFrame()/emit() call -- Telemetry
  // itself always carries "the last staged snapshot" (telemetry.h's own
  // doc comment), so a field updated late in one cycle is simply one
  // cycle "stale" when it reaches the wire, never lost.
  App::Telemetry::Frame frame;

  // ---- Boot: resolve every device before entering the control loop.
  // Telemetry flows from power-on (frames report per-device status), so the
  // host can tell booting from dead; commands are NOT consumed until the
  // main loop starts (no Comms::pump() call here). ----
  while (!preamble.done()) {
    preamble.step();  // one bounded probe action per pass

    App::Telemetry::Frame bootFrame;
    bootFrame.connLeft = preamble.leftConnected();
    bootFrame.connRight = preamble.rightConnected();
    bootFrame.otosConnected = preamble.otosConnected();
    tlm.setFrame(bootFrame);
    tlm.emit(markTime());  // boot frames: device detection status, faults

    uBit.sleep(kPreamblePace);  // paces probes AND yields (radio RX)
  }
  tlm.setEvent(App::kEventBootReady, true);  // Preamble::done() first-true transition

  // ---- Main loop: devices resolved, no readiness checks below this line.
  // TIMING: device calls are pure bus transactions and NEVER sleep. Every
  // required gap is a runAndWait block: it marks time on entry (immediately
  // after the bus event that starts the clock), runs its body, then sleeps
  // until at least the gap has elapsed since the mark. The block visibly
  // scopes exactly the work that borrows the dead time; the body never
  // touches the bus and never sleeps. I2CBus keeps per-device readyAt
  // stamps as a sleep-not-spin safety net (+ telemetry fault bit), so a
  // mis-ordered loop degrades loudly, never silently. ----
  for (;;) {
    uint32_t cycleStart = markTime();  // [ms] pace anchor

    App::Cmd cmd;
    motorL.requestSample();  // 0x46 write (brick holds ONE pending read)
    runAndWait(kSettle, [&] {           // >=4ms: L encoder settling, meanwhile --
      comms.pump(cmd);                  //   drain RX, decode <=1 frame into cmd
    });
    motorL.tick(clock.nowMicros());     // collect -> velocity PID -> armored duty write

    runAndWait(kClear, [&] {  // >=4ms: brick clears L's duty write, meanwhile --
      // Stage this cycle's encoder/velocity/connection fields onto the
      // persistent `frame` (pose/otos were last updated at the END of the
      // PREVIOUS cycle, below -- still the frame's own "last staged
      // snapshot" contract) and emit.
      frame.mode = driving ? msg::DriveMode::VELOCITY : msg::DriveMode::IDLE;
      frame.hasEnc = true;
      frame.encLeft = motorL.position();
      frame.encRight = motorR.position();
      frame.hasVel = true;
      frame.velLeft = motorL.velocity();
      frame.velRight = motorR.velocity();
      frame.hasPose = true;
      frame.active = driving;
      frame.connLeft = motorL.connected();
      frame.connRight = motorR.connected();

      tlm.setFault(App::kFaultI2CSafetyNet, bus.clearanceSafetyNetCount() > 0);
      tlm.setFault(App::kFaultWedgeLatch, motorL.wedged() || motorR.wedged());

      tlm.setFrame(frame);
      tlm.emit(cycleStart);
    });

    motorR.requestSample();
    runAndWait(kSettle, [&] {  // >=4ms: R encoder settling, meanwhile --
      // Apply <=1 decoded command; every path that applies one acks via
      // the telemetry ack ring. `cmd` is a fresh, cycle-local variable
      // (declared above, populated by at most one comms.pump() call this
      // cycle), so reading it here bounds dispatch to at most once per
      // cycle by construction -- no separate "take" flag is needed.
      msg::CommandEnvelope::CmdKind kind = (cmd.status == App::CmdStatus::kDecoded)
          ? cmd.env.cmd_kind
          : msg::CommandEnvelope::CmdKind::NONE;
      switch (kind) {
        case msg::CommandEnvelope::CmdKind::TWIST:
          drive.setTwist(cmd.env.cmd.twist.v_x, cmd.env.cmd.twist.omega);
          deadman.arm(cmd.env.cmd.twist.duration);
          driving = true;
          tlm.ack(cmd.env.corr_id, msg::AckStatus::ACK_STATUS_OK, 0);
          break;
        case msg::CommandEnvelope::CmdKind::CONFIG:
          // ConfigDelta runtime application deferred this sprint
          // (architecture-update.md (103) Step 7 Open Question 3) --
          // decode succeeds, but nothing is applied; ack ERR_UNIMPLEMENTED
          // so the host does not mistake silence for success.
          tlm.ack(cmd.env.corr_id, msg::AckStatus::ACK_STATUS_ERR,
                  static_cast<uint32_t>(msg::ErrCode::ERR_UNIMPLEMENTED));
          break;
        case msg::CommandEnvelope::CmdKind::STOP:
          drive.stop();
          deadman.disarm();
          driving = false;
          tlm.ack(cmd.env.corr_id, msg::AckStatus::ACK_STATUS_OK, 0);
          break;
        case msg::CommandEnvelope::CmdKind::NONE:
        default:
          break;
      }

      bool expired = deadman.expired();
      tlm.setEvent(App::kEventDeadmanExpired, expired);
      if (expired) {
        drive.stop();  // host silent -> wheels stop. No exceptions, no
        driving = false;  // other path to stop being gated by the deadman.
      }

      drive.tick();  // twist -> wheel targets (R consumes them below)
    });
    motorR.tick(clock.nowMicros());

    // Perception (OTOS only this sprint -- architecture-update.md (103)
    // Step 7 Open Question 1) + odometry, outside any motor request/collect
    // window (this file's own bus-discipline responsibility per
    // odometry.h's file header). Both stage into `frame` for the NEXT
    // cycle's tlm.setFrame()/emit() call, per applyOtosSample()'s own
    // "reaches Telemetry before that cycle's frame is built" contract.
    App::applyOtosSample(otos, clock.nowMicros(), frame);
    odom.integrate();  // odometry from both fresh wheel samples
    frame.pose = {odom.x(), odom.y(), odom.theta()};

    sleepUntil(cycleStart, kCycle);  // pace to ~16ms; covers post-R-write
  }                                  //   clearance; always sleeps >=1ms
}
