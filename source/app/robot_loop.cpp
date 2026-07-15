// robot_loop.cpp -- App::RobotLoop implementation. See robot_loop.h's file
// header for the module's boundary and the "mechanical extraction, zero
// behavior change" contract this ticket (105-001) is held to.
//
// Every comment below that describes WHY a step exists (not just what it
// does) is carried forward from main.cpp's pre-extraction inline
// documentation (105-001's own acceptance criterion: "do not lose it in the
// move"), not re-derived.
#include "app/robot_loop.h"

#include "messages/envelope.h"

namespace App {

namespace {

// --- Loop timing constants -- ported from the retired DeviceBus
// (device_bus.h, git history 88e04f1b^): kEncoderSettleMs = 4 (the vendor
// settle window between a motor's own request and collect, shared here by
// BOTH motors' settle windows) and the same 4ms clearance value NezhaMotor/
// Otos already use for every bus_.write()/bus_.read() postClear/preClear
// pair (nezha_motor.cpp's requestEncoder()/writeMotorRun(), otos.h's
// kBusClearance) for the post-duty-write clearance window the 3-block
// schedule below adds. kCycle is the archived plan's own sketch comment
// ("sleepUntil(cycleStart, kCycle); // pace to ~16ms") verbatim -- unit in
// the trailing comment, per naming-and-style.md.
constexpr uint32_t kSettle = 4;         // [ms] encoder-settle window, both motors
constexpr uint32_t kClear = 4;          // [ms] post-duty-write clearance window
constexpr uint32_t kCycle = 16;         // [ms] cycle pace target (~60 Hz)
constexpr uint32_t kPreamblePace = 10;  // [ms] boot-loop probe pacing

}  // namespace

RobotLoop::RobotLoop(Devices::I2CBus& bus, Devices::NezhaMotor& motorL,
                      Devices::NezhaMotor& motorR, Devices::Otos& otos,
                      Comms& comms, Telemetry& tlm, Drive& drive,
                      Odometry& odom, Deadman& deadman, Preamble& preamble,
                      const Devices::Clock& clock, Devices::Sleeper& sleeper)
    : bus_(bus),
      motorL_(motorL),
      motorR_(motorR),
      otos_(otos),
      comms_(comms),
      tlm_(tlm),
      drive_(drive),
      odom_(odom),
      deadman_(deadman),
      preamble_(preamble),
      clock_(clock),
      sleeper_(sleeper) {}

// --- Timing primitives (stakeholder-mandated shape, unchanged by the move
// -- see robot_loop.h's header). markTime() now reads clock_.nowMicros()
// ([us]) and converts to [ms], the unit every other timing constant/field
// in this file already uses. sleepUntil() now sleeps via
// sleeper_.sleepMillis() instead of uBit.sleep() -- always sleeps >=1ms,
// never a zero-length "sleep" (that would be a spin in disguise), always a
// real yield back to the radio/serial fibers on the real Sleeper impl, so
// no runAndWait block can ever degrade into a busy-wait. ---

uint32_t RobotLoop::markTime() const {
  return static_cast<uint32_t>(clock_.nowMicros() / 1000);  // [us] -> [ms]
}

void RobotLoop::sleepUntil(uint32_t mark, uint32_t gap) {  // [ms] [ms]
  uint32_t elapsed = markTime() - mark;
  uint32_t remaining = (elapsed < gap) ? (gap - elapsed) : 0;
  sleeper_.sleepMillis(remaining > 0 ? remaining : 1);
}

template <typename Body>
void RobotLoop::runAndWait(uint32_t gap, Body body) {  // [ms]
  uint32_t mark = markTime();
  body();
  sleepUntil(mark, gap);
}

[[noreturn]] void RobotLoop::run() {
  boot();
  for (;;) {
    cycle();
  }
}

// ---- Boot: resolve every device before entering the control loop.
// Telemetry flows from power-on (frames report per-device status), so the
// host can tell booting from dead; commands are NOT consumed until the
// main loop starts (no Comms::pump() call here). ----
void RobotLoop::boot() {
  while (!preamble_.done()) {
    preamble_.step();  // one bounded probe action per pass

    Telemetry::Frame bootFrame;
    bootFrame.connLeft = preamble_.leftConnected();
    bootFrame.connRight = preamble_.rightConnected();
    bootFrame.otosConnected = preamble_.otosConnected();
    tlm_.setFrame(bootFrame);
    tlm_.emit(markTime());  // boot frames: device detection status, faults

    sleeper_.sleepMillis(kPreamblePace);  // paces probes AND yields (radio RX)
  }
  tlm_.setEvent(kEventBootReady, true);  // Preamble::done() first-true transition
}

// ---- Main cycle: devices resolved, no readiness checks below this line.
// TIMING: device calls are pure bus transactions and NEVER sleep. Every
// required gap is a runAndWait block: it marks time on entry (immediately
// after the bus event that starts the clock), runs its body, then sleeps
// until at least the gap has elapsed since the mark. The block visibly
// scopes exactly the work that borrows the dead time; the body never
// touches the bus and never sleeps. I2CBus keeps per-device readyAt stamps
// as a sleep-not-spin safety net (+ telemetry fault bit), so a mis-ordered
// loop degrades loudly, never silently. ----
void RobotLoop::cycle() {
  uint32_t cycleStart = markTime();  // [ms] pace anchor

  Cmd cmd;
  motorL_.requestSample();  // 0x46 write (brick holds ONE pending read)
  runAndWait(kSettle, [&] {           // >=4ms: L encoder settling, meanwhile --
    comms_.pump(cmd);                 //   drain RX, decode <=1 frame into cmd
  });
  motorL_.tick(clock_.nowMicros());   // collect -> velocity PID -> armored duty write

  runAndWait(kClear, [&] {  // >=4ms: brick clears L's duty write, meanwhile --
    // Stage this cycle's encoder/velocity/connection fields onto the
    // persistent `frame_` (pose/otos were last updated at the END of the
    // PREVIOUS cycle, below -- still the frame's own "last staged
    // snapshot" contract) and emit.
    frame_.mode = driving_ ? msg::DriveMode::VELOCITY : msg::DriveMode::IDLE;
    frame_.hasEnc = true;
    frame_.encLeft = motorL_.position();
    frame_.encRight = motorR_.position();
    frame_.hasVel = true;
    frame_.velLeft = motorL_.velocity();
    frame_.velRight = motorR_.velocity();
    frame_.hasPose = true;
    frame_.active = driving_;
    frame_.connLeft = motorL_.connected();
    frame_.connRight = motorR_.connected();

    tlm_.setFault(kFaultI2CSafetyNet, bus_.clearanceSafetyNetCount() > 0);
    tlm_.setFault(kFaultWedgeLatch, motorL_.wedged() || motorR_.wedged());
    tlm_.setFault(kFaultCommsMalformed, comms_.malformedCount() > 0);

    tlm_.setFrame(frame_);
    tlm_.emit(cycleStart);
  });

  motorR_.requestSample();
  runAndWait(kSettle, [&] {  // >=4ms: R encoder settling, meanwhile --
    // Apply <=1 decoded command; every path that applies one acks via the
    // telemetry ack ring. `cmd` is a fresh, cycle-local variable (declared
    // above, populated by at most one comms_.pump() call this cycle), so
    // reading it here bounds dispatch to at most once per cycle by
    // construction -- no separate "take" flag is needed.
    msg::CommandEnvelope::CmdKind kind = (cmd.status == CmdStatus::kDecoded)
        ? cmd.env.cmd_kind
        : msg::CommandEnvelope::CmdKind::NONE;
    switch (kind) {
      case msg::CommandEnvelope::CmdKind::TWIST:
        drive_.setTwist(cmd.env.cmd.twist.v_x, cmd.env.cmd.twist.omega);
        deadman_.arm(cmd.env.cmd.twist.duration);
        driving_ = true;
        tlm_.ack(cmd.env.corr_id, msg::AckStatus::ACK_STATUS_OK, 0);
        break;
      case msg::CommandEnvelope::CmdKind::CONFIG:
        // ConfigDelta runtime application deferred (architecture-update.md
        // (103) Step 7 Open Question 3) -- decode succeeds, but nothing is
        // applied; ack ERR_UNIMPLEMENTED so the host does not mistake
        // silence for success.
        tlm_.ack(cmd.env.corr_id, msg::AckStatus::ACK_STATUS_ERR,
                  static_cast<uint32_t>(msg::ErrCode::ERR_UNIMPLEMENTED));
        break;
      case msg::CommandEnvelope::CmdKind::STOP:
        drive_.stop();
        deadman_.disarm();
        driving_ = false;
        tlm_.ack(cmd.env.corr_id, msg::AckStatus::ACK_STATUS_OK, 0);
        break;
      case msg::CommandEnvelope::CmdKind::NONE:
      default:
        break;
    }

    bool expired = deadman_.expired();
    tlm_.setEvent(kEventDeadmanExpired, expired);
    if (expired) {
      drive_.stop();     // host silent -> wheels stop. No exceptions, no
      driving_ = false;  // other path to stop being gated by the deadman.
    }

    drive_.tick();  // twist -> wheel targets (R consumes them below)
  });
  motorR_.tick(clock_.nowMicros());

  // Perception (OTOS only -- architecture-update.md (103) Step 7 Open
  // Question 1) + odometry, outside any motor request/collect window (this
  // class's own bus-discipline responsibility per odometry.h's file
  // header). Both stage into `frame_` for the NEXT cycle's
  // tlm_.setFrame()/emit() call, per applyOtosSample()'s own "reaches
  // Telemetry before that cycle's frame is built" contract.
  applyOtosSample(otos_, clock_.nowMicros(), frame_);
  odom_.integrate();  // odometry from both fresh wheel samples
  frame_.pose = {odom_.x(), odom_.y(), odom_.theta()};

  sleepUntil(cycleStart, kCycle);  // pace to ~16ms; covers post-R-write
}                                  //   clearance; always sleeps >=1ms

}  // namespace App
