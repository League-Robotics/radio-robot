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
// schedule below adds.
//
// kCycle (106-001, retargeted from the archived plan's original,
// never-achievable "sleepUntil(cycleStart, kCycle); // pace to ~16ms"
// sketch comment): ticket 105-004's virtual-cycle-timing diagnostic proved
// that 16ms was arithmetically impossible -- the three kSettle/kClear
// windows below alone already consume 12 of that 16ms budget, leaving 4ms
// for two PID ticks, two encoder requests, a full telemetry frame
// build+emit, an OTOS sample, and odometry integration. kCycle is now the
// STATED TOTAL for the whole schedule (all four pacing blocks, not just the
// trailing one) -- ~25 Hz/~40ms, matching Devices::Telemetry's own
// kPrimaryPeriod=40ms (telemetry.h) so the primary-frame throttle and the
// loop's own pace agree by construction. See architecture-update.md
// (106) Decision 1.
constexpr uint32_t kSettle = 4;  // [ms] encoder-settle window, both motors
constexpr uint32_t kClear = 4;   // [ms] post-duty-write clearance window
constexpr uint32_t kCycle = 40;  // [ms] whole-schedule pace target (~25 Hz)

// kWindows is what the three settle/clearance blocks above ALREADY consume
// before the final (perception+odometry+pace) block ever runs; kPace is
// that final block's own gap, DERIVED so it absorbs kWindows into kCycle's
// total rather than stacking a fresh kCycle on top of it (105-004's
// diagnosed defect: the OLD code called `sleepUntil(cycleStart, kCycle)`
// for the final block, which -- proved by the sim's zero-real-time-cost
// virtual clock, where every block's own elapsed-since-mark is provably 0
// -- requested kCycle IN ADDITION to the 12ms already spent, not instead of
// it: 4+4+4+16=28ms virtual against a 16ms target). Passing kPace (not
// kCycle) to the final block's own runAndWait fixes this by construction:
// the schedule's four blocks now sum to EXACTLY kCycle under the sim's
// worst-case frozen clock, the same invariant the other three blocks
// already had individually.
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
        // ConfigDelta runtime application (106-002/SUC-025, resolving
        // architecture-update.md (103) Step 7 Open Question 3 for the ONE
        // patch type this sprint scopes -- architecture-update.md (106)
        // Decision 2): a MotorConfigPatch is live-applied below; every
        // other patch kind (DRIVETRAIN/PLANNER/WATCHDOG/NONE) stays
        // ERR_UNIMPLEMENTED, deliberately out of scope (DrivetrainConfigPatch
        // has no on-robot fusion consumer this sprint; PlannerConfigPatch's
        // heading_kp/heading_kd target Motion::SegmentExecutor, deleted
        // post-102).
        if (cmd.env.cmd.config.patch_kind == msg::ConfigDelta::PatchKind::MOTOR) {
          const msg::MotorConfigPatch& patch = cmd.env.cmd.config.patch.motor;

          // Merge each motor's OWN current gains against whatever wire
          // fields are PRESENT (config.proto's Opt<T>-presence convention)
          // -- NOT a blanket mirror of one motor's gains onto the other,
          // since the two leaves' calibration can legitimately differ.
          Devices::Gains gainsL = motorL_.gains();
          Devices::Gains gainsR = motorR_.gains();
          if (patch.kp.has) { gainsL.kp = patch.kp.val; gainsR.kp = patch.kp.val; }
          if (patch.ki.has) { gainsL.ki = patch.ki.val; gainsR.ki = patch.ki.val; }
          if (patch.kff.has) { gainsL.kff = patch.kff.val; gainsR.kff = patch.kff.val; }
          if (patch.i_max.has) { gainsL.iMax = patch.i_max.val; gainsR.iMax = patch.i_max.val; }
          if (patch.kaw.has) { gainsL.kaw = patch.kaw.val; gainsR.kaw = patch.kaw.val; }

          // travel_calib is side-selected (config.proto's own
          // MotorConfigPatch.side comment) -- applies to exactly one leaf.
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

          tlm_.ack(cmd.env.corr_id, msg::AckStatus::ACK_STATUS_OK, 0);
        } else {
          tlm_.ack(cmd.env.corr_id, msg::AckStatus::ACK_STATUS_ERR,
                    static_cast<uint32_t>(msg::ErrCode::ERR_UNIMPLEMENTED));
        }
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

  // Final (perception + odometry + pace) block -- the schedule's 4th
  // runAndWait, matching the same "own mark, own gap" shape as the three
  // settle/clearance blocks above (106-001: this used to be a bare trailing
  // `sleepUntil(cycleStart, kCycle)` anchored to the CYCLE'S start rather
  // than its own; see kPace's own comment above for why that double-counted
  // kWindows against the sim's zero-real-time-cost virtual clock). Body:
  // OTOS (architecture-update.md (103) Step 7 Open Question 1) + odometry,
  // outside any motor request/collect window (this class's own
  // bus-discipline responsibility per odometry.h's file header) -- both
  // stage into `frame_` for the NEXT cycle's tlm_.setFrame()/emit() call,
  // per applyOtosSample()'s own "reaches Telemetry before that cycle's
  // frame is built" contract. Unlike the other three blocks, this one DOES
  // touch the bus (the OTOS read) -- it is the schedule's pace block, not a
  // settle/clearance window, so that constraint doesn't apply to it.
  runAndWait(kPace, [&] {
    applyOtosSample(otos_, clock_.nowMicros(), frame_);
    odom_.integrate();  // odometry from both fresh wheel samples
    frame_.pose = {odom_.x(), odom_.y(), odom_.theta()};
  });
}

}  // namespace App
