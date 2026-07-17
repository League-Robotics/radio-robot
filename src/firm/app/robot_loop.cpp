// robot_loop.cpp -- App::RobotLoop implementation. See robot_loop.h's file
// header for the module's boundary and entry points; DESIGN.md for the
// timing-schedule rationale.
#include "app/robot_loop.h"

#include "messages/envelope.h"

namespace App {

namespace {

// Loop timing constants. kSettle is the vendor settle window between a
// motor's own request and collect, shared by both motors' settle windows;
// kClear is the same clearance value NezhaMotor/Otos use for every
// bus_.write()/bus_.read() postClear/preClear pair, applied here as the
// post-duty-write clearance window.
//
// kCycle is the STATED TOTAL for the whole schedule (all four pacing
// blocks, not just the trailing one) -- ~25 Hz/~40ms, matching
// Devices::Telemetry's own kPrimaryPeriod=40ms (telemetry.h) so the
// primary-frame throttle and the loop's own pace agree by construction.
constexpr uint32_t kSettle = 4;  // [ms] encoder-settle window, both motors
constexpr uint32_t kClear = 4;   // [ms] post-duty-write clearance window
constexpr uint32_t kCycle = 40;  // [ms] whole-schedule pace target (~25 Hz)

// kWindows is what the three settle/clearance blocks above already consume
// before the final (perception+odometry+pace) block runs; kPace is that
// final block's own gap, DERIVED so it absorbs kWindows into kCycle's total
// rather than stacking a fresh kCycle on top of it -- anchoring the final
// block to kCycle directly (instead of kPace) would double-count kWindows
// under a zero-real-time-cost virtual clock, where every block's own
// elapsed-since-mark is provably 0 (each of the four blocks would then
// request its full nominal gap on top of the others instead of the whole
// schedule summing to kCycle). Passing kPace to the final block's own
// runAndWait keeps the schedule's four blocks summing to exactly kCycle
// under that worst case, the same invariant the other three blocks already
// have individually.
constexpr uint32_t kWindows = 2 * kSettle + kClear;  // [ms] time the 3 settle/clear
                                                      // blocks consume before the pace block
static_assert(kWindows <= kCycle,
              "kSettle+kClear+kSettle must fit inside the kCycle budget");
constexpr uint32_t kPace = kCycle - kWindows;  // [ms] final block's own gap, absorbing kWindows

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

// --- Timing primitives -- see robot_loop.h's header. markTime() reads
// clock_.nowMicros() ([us]) and converts to [ms], the unit every other
// timing constant/field in this file uses. sleepUntil() always sleeps
// >=1ms, never a zero-length "sleep" (that would be a spin in disguise),
// so it is always a real yield back to the radio/serial fibers on the real
// Sleeper impl -- no runAndWait block can ever degrade into a busy-wait. ---

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


void RobotLoop::updateTlm() {

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
}

void RobotLoop::handleTwist(const msg::CommandEnvelope& env) {
  drive_.setTwist(env.cmd.twist.v_x, env.cmd.twist.omega);
  deadman_.arm(env.cmd.twist.duration);
  driving_ = true;
  tlm_.ack(env.corr_id, msg::AckStatus::ACK_STATUS_OK, 0);
}

// ConfigDelta runtime application: a MotorConfigPatch is live-applied
// below; every other patch kind (DRIVETRAIN/PLANNER/WATCHDOG/NONE) stays
// ERR_UNIMPLEMENTED, deliberately out of scope -- see DESIGN.md §3.
void RobotLoop::handleConfig(const msg::CommandEnvelope& env) {
  if (env.cmd.config.patch_kind != msg::ConfigDelta::PatchKind::MOTOR) {
    tlm_.ack(env.corr_id, msg::AckStatus::ACK_STATUS_ERR,
              static_cast<uint32_t>(msg::ErrCode::ERR_UNIMPLEMENTED));
    return;
  }

  const msg::MotorConfigPatch& patch = env.cmd.config.patch.motor;

  // Merge each motor's OWN current gains against whatever wire fields are
  // PRESENT (config.proto's Opt<T>-presence convention) -- NOT a blanket
  // mirror of one motor's gains onto the other, since the two leaves'
  // calibration can legitimately differ.
  Devices::Gains gainsL = motorL_.gains();
  Devices::Gains gainsR = motorR_.gains();
  if (patch.kp.has) { gainsL.kp = patch.kp.val; gainsR.kp = patch.kp.val; }
  if (patch.ki.has) { gainsL.ki = patch.ki.val; gainsR.ki = patch.ki.val; }
  if (patch.kff.has) { gainsL.kff = patch.kff.val; gainsR.kff = patch.kff.val; }
  if (patch.i_max.has) { gainsL.iMax = patch.i_max.val; gainsR.iMax = patch.i_max.val; }
  if (patch.kaw.has) { gainsL.kaw = patch.kaw.val; gainsR.kaw = patch.kaw.val; }

  // travel_calib is side-selected (config.proto's own MotorConfigPatch.side
  // comment) -- applies to exactly one leaf.
  Devices::Opt<float> travelCalibL;
  Devices::Opt<float> travelCalibR;
  if (patch.travel_calib.has) {
    if (patch.side == msg::BoundMotorSide::LEFT) {
      travelCalibL.has = true;
      travelCalibL.val = patch.travel_calib.val;
    } else {
      travelCalibR.has = true;
      travelCalibR.val = patch.travel_calib.val;
    }
  }

  motorL_.applyGains(gainsL, travelCalibL);
  motorR_.applyGains(gainsR, travelCalibR);

  tlm_.ack(env.corr_id, msg::AckStatus::ACK_STATUS_OK, 0);
}

void RobotLoop::handleStop(const msg::CommandEnvelope& env) {
  drive_.stop();
  deadman_.disarm();
  driving_ = false;
  tlm_.ack(env.corr_id, msg::AckStatus::ACK_STATUS_OK, 0);
}

// Dispatches the <=1 decoded command in cmd to its own handler by
// cmd_kind. `cmd` is a fresh, cycle-local variable (populated by at most
// one comms_.pump() call this cycle), so reading it here bounds dispatch
// to at most once per cycle by construction -- no separate "take" flag
// needed.
void RobotLoop::processMessage(const Cmd& cmd) {
  msg::CommandEnvelope::CmdKind kind = (cmd.status == CmdStatus::kDecoded)
      ? cmd.env.cmd_kind
      : msg::CommandEnvelope::CmdKind::NONE;
  switch (kind) {
    case msg::CommandEnvelope::CmdKind::TWIST:
      handleTwist(cmd.env);
      break;
    case msg::CommandEnvelope::CmdKind::CONFIG:
      handleConfig(cmd.env);
      break;
    case msg::CommandEnvelope::CmdKind::STOP:
      handleStop(cmd.env);
      break;
    case msg::CommandEnvelope::CmdKind::NONE:
    default:
      break;
  }
}

[[noreturn]] void RobotLoop::run() {
  boot();
  for (;;) {
    cycle();
  }
}

// ---- Boot: resolve every device before entering the control loop.
// Telemetry flows from power-on (frames report per-device status), so the
// host can tell booting from dead; commands are not consumed until the
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
// TIMING: device calls are pure bus transactions and never sleep. Every
// required gap is a runAndWait block: it marks time on entry (immediately
// after the bus event that starts the clock), runs its body, then sleeps
// until at least the gap has elapsed since the mark. The block visibly
// scopes exactly the work that borrows the dead time. I2CBus keeps
// per-device readyAt stamps as a sleep-not-spin safety net (+ telemetry
// fault bit), so a mis-ordered loop degrades loudly, never silently. The
// three settle/clearance blocks' own bodies never touch the bus and never
// sleep; the schedule's 4th block (the trailing perception+odometry+pace
// block, kPace) is the one exception -- see its own comment below. ----
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

    updateTlm();
    tlm_.emit(cycleStart);
  });

  motorR_.requestSample();

  runAndWait(kSettle, [&] {  // >=4ms: R encoder settling, meanwhile --
    // Apply <=1 decoded command; every path that applies one acks via the
    // telemetry ack ring. `cmd` is a fresh, cycle-local variable (declared
    // above, populated by at most one comms_.pump() call this cycle), so
    // reading it here bounds dispatch to at most once per cycle by
    // construction -- no separate "take" flag is needed.
    processMessage(cmd);

    bool expired = deadman_.expired();
    tlm_.setEvent(kEventDeadmanExpired, expired);
    if (expired) {
      drive_.stop();     // host silent -> wheels stop. No exceptions, no
      driving_ = false;  // other path to stop being gated by the deadman.
    }

    drive_.tick();  // twist -> wheel targets (R consumes them below)
  });
  
  motorR_.tick(clock_.nowMicros());

  // Final (perception + odometry + pace) block -- the schedule's 4th
  // runAndWait, matching the same "own mark, own gap" shape as the three
  // settle/clearance blocks above (see kPace's own comment for why the gap
  // must be derived, not a bare kCycle anchored to the cycle start). Body:
  // OTOS + odometry, outside any motor request/collect window (this
  // class's own bus-discipline responsibility) -- both stage into `frame_`
  // for the NEXT cycle's tlm_.setFrame()/emit() call. Unlike the other
  // three blocks, this one DOES touch the bus (the OTOS read) -- it is the
  // schedule's pace block, not a settle/clearance window, so that
  // constraint doesn't apply to it.
  runAndWait(kPace, [&] {
    applyOtosSample(otos_, clock_.nowMicros(), frame_);
    odom_.integrate();  // odometry from both fresh wheel samples
    frame_.pose = {odom_.x(), odom_.y(), odom_.theta()};
  });
}

}  // namespace App
