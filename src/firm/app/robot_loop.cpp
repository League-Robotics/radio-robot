#include "app/robot_loop.h"
#include "app/debug.h"

#include <cmath>
#include <cstdio>

#include "messages/envelope.h"
#include "motion/body_kinematics.h"

namespace App {

namespace {

// The brick's mandatory encoder select->read settle, one window per motor.
// This wait is NOT optional and NOT avoidable by deleting the window:
// Platform::I2CBus::waitForClearance() enforces the same clearance from
// requestSample()'s postClear, so zeroing kSettle measures identical
// wall-clock (verified on tovez 2026-08-07) while losing the comms pump
// that runs inside the window. Spending the mandatory wait usefully is the
// entire point of the window.
constexpr uint32_t kSettle = 4;  // [ms] encoder-settle window, both motors

// There is deliberately no post-duty-write window. One used to sit between
// the two motors (kClear = 4 ms) on the theory that the brick needed
// clearance after a duty write. It does -- but the next 0x10 transaction
// after that write is the NEXT cycle's encoder select, tens of ms away and
// far past the deadline, so the window guarded nothing and simply padded
// every cycle. Removed 2026-08-07 after measuring it: busy dropped 21.9 ->
// 17.3 ms with no change to encoder integrity.
constexpr uint32_t kWindows = 2 * kSettle;  // [ms]
static_assert(kWindows <= RobotLoop::kCycle,
              "both kSettle windows must fit inside the kCycle budget");

constexpr uint32_t kPreamblePace = 10;  // [ms] boot-loop probe pacing

constexpr float kPositionWireBound = 32000.0f;       // [mm]
constexpr float kPositionRebaselineMargin = 30000.0f;  // [mm]

// [mm/s] a wheel reading at or below this counts as "still" for CALIBRATE's
// parked precondition. Encoder velocity at true rest reads ~0; this only
// needs to reject genuine motion, not split hairs about noise.
constexpr float kCalibrateStillSpeed = 5.0f;

uint32_t packLine(const Devices::LineReading& reading) {
  return (reading.raw[0] & 0xFFu) | ((reading.raw[1] & 0xFFu) << 8) |
         ((reading.raw[2] & 0xFFu) << 16) | ((reading.raw[3] & 0xFFu) << 24);
}

// Squeeze one 16-bit channel into the wire's 8 bits by scaling against the
// ADC's ACTUAL full scale, not against 65535.
//
// This used to be a bare `>> 8`, which silently assumed the ADC ran to
// 65535. It does not: at the shipped ATIME (252) the APDS integrates for
// 11.1ms and saturates at 4100 counts, so `>> 8` could only ever emit 0..16
// and rounded every channel under 256 counts to zero. Measured on tovez
// 2026-08-08: r/g/b pinned at 0 and c at 2 across 375 consecutive frames,
// zero spread on all four -- the sensor was reading correctly and the wire
// was throwing the answer away.
uint8_t scaleColorChannel(uint32_t value, uint32_t fullScale) {  // [counts]
  if (fullScale == 0) return 0;
  const uint32_t scaled = (value * 255u) / fullScale;
  return static_cast<uint8_t>(scaled > 255u ? 255u : scaled);
}

uint32_t packColor(const Devices::ColorReading& reading, uint32_t fullScale) {  // [counts]
  return static_cast<uint32_t>(scaleColorChannel(reading.r, fullScale)) |
         (static_cast<uint32_t>(scaleColorChannel(reading.g, fullScale)) << 8) |
         (static_cast<uint32_t>(scaleColorChannel(reading.b, fullScale)) << 16) |
         (static_cast<uint32_t>(scaleColorChannel(reading.c, fullScale)) << 24);
}

}  // namespace

RobotLoop::RobotLoop(Platform::I2CBus& bus, Devices::Motor& motorL,
                      Devices::Motor& motorR, Devices::Otos& otos,
                      Devices::ColorSensorLeaf& color, Devices::LineSensorLeaf& line,
                      Comms& comms, Telemetry& tlm, Drive& drive,
                      Configurator& configurator, Motion::Odometry& odom,
                      Motion::Planner& planner, Motion::Navigator& navigator,
                      Preamble& preamble, const Platform::Clock& clock,
                      Platform::Sleeper& sleeper)
    : bus_(bus),
      motorL_(motorL),
      motorR_(motorR),
      otos_(otos),
      color_(color),
      line_(line),
      comms_(comms),
      tlm_(tlm),
      drive_(drive),
      configurator_(configurator),
      odom_(odom),
      planner_(planner),
      navigator_(navigator),
      preamble_(preamble),
      clock_(clock),
      sleeper_(sleeper) {}

uint32_t RobotLoop::markTime() const {
  return static_cast<uint32_t>(clock_.nowMicros() / 1000);  // [us] -> [ms]
}

void RobotLoop::sleepUntil(uint32_t mark, uint32_t gap) {  // [ms] [ms]
  uint32_t elapsed = markTime() - mark;
  uint32_t remaining = (elapsed < gap) ? (gap - elapsed) : 0;
  if (remaining > 0) {
    sleeper_.sleepMillis(remaining);
  } else {
    sleeper_.yield();
  }
}

template <typename Body>
void RobotLoop::runAndWait(uint32_t gap, Body body) {  // [ms]
  uint32_t mark = markTime();
  body();
  sleepUntil(mark, gap);
}

template <typename Body>
void RobotLoop::runAndWaitUntil(uint32_t deadlineMark, uint32_t deadlineGap, Body body) {  // [ms] [ms]
  body();
  sleepUntil(deadlineMark, deadlineGap);
}

float RobotLoop::clampToPositionWireBound(float pos, bool* clamped) {
  *clamped = std::fabs(pos) > kPositionWireBound;
  return *clamped ? std::copysign(kPositionWireBound, pos) : pos;
}

void RobotLoop::routeCommand(const Cmd& cmd) {
  if (cmd.status != CmdStatus::kDecoded) return;

  switch (cmd.env.cmd_kind) {
    case msg::CommandEnvelope::CmdKind::MOVE:
      handleMove(cmd.env);
      break;
    case msg::CommandEnvelope::CmdKind::WHEELS:
      handleWheels(cmd.env);
      break;
    case msg::CommandEnvelope::CmdKind::GO_TO:
      handleGoto(cmd.env);
      break;
    case msg::CommandEnvelope::CmdKind::STOP:
      handleStop(cmd.env);
      break;
    case msg::CommandEnvelope::CmdKind::ESTOP:
      handleEstop(cmd.env);
      break;
    case msg::CommandEnvelope::CmdKind::CONFIG:
      tlm_.ack(cmd.env.corr_id,
               static_cast<uint32_t>(configurator_.applyGroup(
                   cmd.env.cmd.config.target, cmd.env.cmd.config.body_,
                   cmd.env.cmd.config.body_count)));
      break;
    case msg::CommandEnvelope::CmdKind::GET_CONFIG:
      handleGetConfig(cmd.env);
      break;
    case msg::CommandEnvelope::CmdKind::CALIBRATE:
      handleCalibrate(cmd.env);
      break;
    case msg::CommandEnvelope::CmdKind::SET_FIELD:
      tlm_.ack(cmd.env.corr_id,
               static_cast<uint32_t>(configurator_.applyField(
                   cmd.env.cmd.set_field.target,
                   static_cast<uint16_t>(cmd.env.cmd.set_field.field),
                   cmd.env.cmd.set_field.value)));
      break;
    case msg::CommandEnvelope::CmdKind::NONE:
    default:
      break;
  }
}

void RobotLoop::handleCalibrate(const msg::CommandEnvelope& env) {
  // Parked precondition, enforced HERE. The gyro averages ~612ms of samples
  // into its bias estimate; motion during that window is exactly the boot
  // fault this command exists to repair (see envelope.proto's Calibrate).
  // Both measured wheel velocities must be still AND nothing may be
  // commanding velocity this cycle -- the commanded check catches a move
  // whose measured speed happens to pass through zero (reversals), the
  // measured check catches coasting with no owner.
  if (!state_.otos.present) {
    tlm_.ack(env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_NOT_CONFIGURED));
    return;
  }
  const bool still =
      std::fabs(state_.wheelLeft.velocity) <= kCalibrateStillSpeed &&
      std::fabs(state_.wheelRight.velocity) <= kCalibrateStillSpeed &&
      state_.wheelLeft.cmdVelocity == 0.0f &&
      state_.wheelRight.cmdVelocity == 0.0f;
  if (!still) {
    tlm_.ack(env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_BUSY));
    return;
  }
  otos_.calibrateImu(static_cast<uint8_t>(env.cmd.calibrate.samples));
  tlm_.ack(env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_NONE));
}

void RobotLoop::handleGetConfig(const msg::CommandEnvelope& env) {
  const msg::ConfigGroupTarget target = env.cmd.get_config.target;

  msg::ConfigSnapshot snapshot;
  const msg::ErrCode result = configurator_.encodeSnapshot(target, snapshot);

  msg::ReplyEnvelope reply;
  reply.corr_id = env.corr_id;
  if (result == msg::ErrCode::ERR_NONE) {
    reply.body_kind = msg::ReplyEnvelope::BodyKind::CFG;
    reply.body.cfg = snapshot;
  } else {
    reply.body_kind = msg::ReplyEnvelope::BodyKind::ERR;
    reply.body.err.code = result;
    reply.body.err.field = static_cast<uint32_t>(target);
  }
  comms_.sendReply(reply);
}

void RobotLoop::handleMove(const msg::CommandEnvelope& env) {
  // A new motion command is the host acknowledging the stall: from here the
  // flag would describe a motion that is over. Cleared at the TOP, before any
  // acceptance gate, deliberately -- a rejected command still proves the host
  // is talking to us and has moved on, and it gets its own error ack to act on.
  clearStallLatch();
  if (!configured_) {
    tlm_.ack(env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_NOT_CONFIGURED));
    return;
  }

  const msg::Move& move = env.cmd.move;
  if (move.velocity_kind == msg::Move::VelocityKind::NONE ||
      move.stop_kind == msg::Move::StopKind::NONE || move.timeout <= 0.0f) {
    tlm_.ack(env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_BADARG));
    return;
  }

  Motion::Move m;
  m.id = move.id;
  m.timeout = move.timeout;
  switch (move.stop_kind) {
    case msg::Move::StopKind::TIME:
      m.kind = Motion::Move::Kind::Time;
      m.threshold = move.stop.time;
      break;
    case msg::Move::StopKind::DISTANCE:
      m.kind = Motion::Move::Kind::Distance;
      m.threshold = move.stop.distance;
      break;
    case msg::Move::StopKind::ANGLE:
      m.kind = Motion::Move::Kind::Angle;
      m.threshold = move.stop.angle;
      break;
    default:
      break;  // unreachable: stop_kind validated above
  }

  if (move.velocity_kind == msg::Move::VelocityKind::WHEELS) {
    m.velocityKind = Motion::Move::VelocityKind::Wheels;
    m.vLeft = move.velocity.wheels.v_left;
    m.vRight = move.velocity.wheels.v_right;
  } else {
    m.velocityKind = Motion::Move::VelocityKind::Twist;
    m.v_x = move.velocity.twist.v_x;
    m.v_y = move.velocity.twist.v_y;
    m.omega = move.velocity.twist.omega;
    const bool badShape =
        m.threshold < 0.0f ||
        (m.kind == Motion::Move::Kind::Distance && m.v_x == 0.0f) ||
        (m.kind == Motion::Move::Kind::Angle && m.omega == 0.0f);
    if (badShape) {
      tlm_.ack(env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_BADARG));
      return;
    }
  }

  m.requestedThreshold = m.threshold;

  // The per-direction rotation gain/offset is an OPEN-LOOP correction: it
  // pre-distorts the commanded angle so a scrub-limited wheel estimate lands
  // on the requested one. When the OTOS is measuring heading the loop is
  // closed on optical truth, and pre-distorting the target would just make it
  // stop at the wrong place -- accurately.
  //
  // 135-004 "Landmine 3": Motion::Navigator's own pivots (SUC-004) never
  // reach this correction at all -- they are issued internally via
  // Planner::move(), never through handleMove()/this wire-command path.
  // This is correct, not an oversight: a Navigator pivot only ever
  // happens with OTOS connected (handleGoto()'s own gate, above), exactly
  // the condition under which this correction is already a no-op here.
  // See Navigator::issuePivotMove() (navigator.cpp) for the pivot-side
  // half of this same reasoning -- do not add this correction to that
  // path.
  if (m.kind == Motion::Move::Kind::Angle && !state_.otos.present) {
    const bool positive =
        (m.velocityKind == Motion::Move::VelocityKind::Twist)
            ? (m.omega >= 0.0f)
            : (m.vRight >= m.vLeft);
    const float gain = positive ? rotGainPos_ : rotGainNeg_;
    const float offset = positive ? rotOffsetPos_ : rotOffsetNeg_;
    if (gain > 0.0f) {
      const float corrected = (m.threshold - offset) / gain;
      m.threshold = (corrected > 0.0f) ? corrected : m.threshold;
    }
  }

  if (alreadyAccepted(move.id)) {
    tlm_.ack(env.corr_id, 0);
    return;
  }

  // 135-004 ownership rule: MOVE cancels an active goto -- no completion
  // ack, preempted, not completed -- before proceeding. cancel() is a
  // harmless no-op when no goto is active.
  navigator_.cancel();

  drive_.takeover();
  const bool accepted = planner_.move(m, move.replace);
  if (accepted) {
    recordAccepted(move.id);
  }
  tlm_.ack(env.corr_id,
           accepted ? 0 : static_cast<uint32_t>(msg::ErrCode::ERR_FULL));
}

bool RobotLoop::alreadyAccepted(uint32_t id) const {
  if (id == 0) {
    return false;  // "unset": every id-0 move is its own move
  }
  for (int i = 0; i < kAcceptedMoveIdCount; ++i) {
    if (acceptedMoveIds_[i] == id) {
      return true;
    }
  }
  return false;
}

void RobotLoop::recordAccepted(uint32_t id) {
  if (id == 0) {
    return;
  }
  acceptedMoveIds_[acceptedMoveIdNext_] = id;
  acceptedMoveIdNext_ = (acceptedMoveIdNext_ + 1) % kAcceptedMoveIdCount;
}

void RobotLoop::handleWheels(const msg::CommandEnvelope& env) {
  // A new motion command is the host acknowledging the stall: from here the
  // flag would describe a motion that is over. Cleared at the TOP, before any
  // acceptance gate, deliberately -- a rejected command still proves the host
  // is talking to us and has moved on, and it gets its own error ack to act on.
  clearStallLatch();
  if (!configured_) {
    tlm_.ack(env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_NOT_CONFIGURED));
    return;
  }

  const msg::Wheels& wheels = env.cmd.wheels;
  if (wheels.duration <= 0.0f) {
    tlm_.ack(env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_BADARG));
    return;
  }

  // 135-004 ownership rule: WHEELS cancels an active goto -- no completion
  // ack, preempted, not completed. cancel() is a harmless no-op when no
  // goto is active.
  navigator_.cancel();

  planner_.estop();  // Drive takes over motion (one owner at a time)
  drive_.command(wheels.v_left, wheels.v_right, wheels.duration, wheels.id,
                 state_.time.cycleStart);
  tlm_.ack(env.corr_id, 0);
}

void RobotLoop::handleStop(const msg::CommandEnvelope& env) {
  const bool accepted = planner_.plannedStop(env.cmd.stop.id);
  tlm_.ack(env.corr_id,
           accepted ? 0 : static_cast<uint32_t>(msg::ErrCode::ERR_FULL));
}

void RobotLoop::handleEstop(const msg::CommandEnvelope& env) {
  clearStallLatch();
  drive_.estop();
  planner_.estop();
  // 135-004: clears the Navigator's own target in the SAME cycle it
  // clears the Planner's queue -- no completion ack, no fault ack, just
  // gone, matching ESTOP's existing "halt now, everywhere" contract
  // (.claude/rules/playfield-testing.md's "Halting" section).
  navigator_.cancel();
  state_.wheelLeft.cmdVelocity = 0.0f;
  state_.wheelRight.cmdVelocity = 0.0f;
  tlm_.ack(env.corr_id, 0);
}

// handleGoto() -- 135-004. ErrCode choice (this ticket's own open
// question, resolved here): ERR_NOT_CONFIGURED (8) for BOTH the
// config-completeness gate (matching every other handler in this file)
// AND the OTOS-disconnected gate. Surveyed every other existing ErrCode
// first (envelope.proto:48-60) -- ERR_NOT_LIVE/ERR_BUSY are config-push-
// specific (Configurator::applyGroup()/install(MOTORS)), ERR_UNIMPLEMENTED
// means "no live consumer" (false here -- GO_TO has one), ERR_BADARG/
// ERR_RANGE describe a malformed COMMAND, not an unmet ENVIRONMENTAL
// precondition. ERR_NOT_CONFIGURED's own doc comment -- "composition root
// refused MOVE -- config-completeness gate not yet satisfied" -- is a
// stretch (OTOS connectivity is a runtime sensor fact, not boot-time
// config completeness) but the closest existing fit in SPIRIT ("a
// precondition for accepting this motion command is not satisfied"), and
// reusing an existing code beats adding a new one for this ticket's
// narrow need.
//
// ROBOT-frame resolution mirrors src/tests/bench/goto_otos.py's own
// relative-target resolution EXACTLY (that script's `main()`, the
// `relative` branch): world_x/y = otos.x/y + dx*cos(h) -+ dy*sin(h), using
// RAW, un-negated state_.otos.heading -- a pure geometric transform
// within the self-consistent OTOS/world frame (ticket 008 settled that
// this raw wire value tracks true-world CCW directly), UNRELATED to
// NavigatorLimits::yawSign (which only ever applies at the Navigator's
// own omega/Move-command boundary, never here).
void RobotLoop::handleGoto(const msg::CommandEnvelope& env) {
  // A new motion command is the host acknowledging the stall: from here the
  // flag would describe a motion that is over. Cleared at the TOP, before any
  // acceptance gate, deliberately -- a rejected command still proves the host
  // is talking to us and has moved on, and it gets its own error ack to act on.
  clearStallLatch();
  if (!configured_) {
    tlm_.ack(env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_NOT_CONFIGURED));
    return;
  }

  const msg::GoTo& goTo = env.cmd.go_to;
  if (goTo.timeout <= 0.0f) {
    tlm_.ack(env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_BADARG));
    return;
  }

  // SUC-005's own explicit gate: a goto with no OTOS fix to navigate on is
  // refused outright, never accepted-then-immediately-aborted.
  if (!state_.otos.connected) {
    tlm_.ack(env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_NOT_CONFIGURED));
    return;
  }

  Motion::GotoTarget target;
  target.id = goTo.id;
  target.tolerance = goTo.arrive;
  target.timeout = static_cast<uint32_t>(goTo.timeout);
  target.speed = goTo.speed;
  if (goTo.frame == 1) {  // ROBOT -- resolve to world ONCE, here, at acceptance
    const float c = std::cos(state_.otos.heading);
    const float s = std::sin(state_.otos.heading);
    target.x = state_.otos.x + goTo.x * c - goTo.y * s;
    target.y = state_.otos.y + goTo.x * s + goTo.y * c;
  } else {  // WORLD (frame == 0, and fail-open for any other value)
    target.x = goTo.x;
    target.y = goTo.y;
  }

  // 135-004 ownership rule: GO_TO cancels active Drive teleop, the same
  // call handleMove() already makes. Does NOT also planner_.estop(): the
  // Navigator's own first replace=true issue (navigator.cpp's tick())
  // flushes whatever the Planner was doing within one cycle, the same
  // way any other Move(replace=true) already preempts an in-flight one --
  // no separate clear needed here.
  drive_.takeover();
  navigator_.start(target);
  tlm_.ack(env.corr_id, 0);
}

void RobotLoop::rejectDuringBoot(const Cmd& cmd) {
  if (cmd.status != CmdStatus::kDecoded) return;
  tlm_.ack(cmd.env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_NOT_CONFIGURED));
}

[[noreturn]] void RobotLoop::run() {
  boot();
  for (;;) {
    cycle();
  }
}

void RobotLoop::boot() {
  while (!preamble_.done()) {
    comms_.pump(markTime());
    Cmd bootCmd;
    while (comms_.takeCommand(bootCmd)) rejectDuringBoot(bootCmd);

    preamble_.step();  // one bounded probe action per pass

    Types::RobotState bootState;
    bootState.time.cycleStart = markTime();
    bootState.wheelLeft.connected = preamble_.leftConnected();
    bootState.wheelRight.connected = preamble_.rightConnected();
    bootState.otos.connected = preamble_.otosConnected();
    tlm_.update(bootState, drive_);
    tlm_.emit(bootState.time.cycleStart);

    sleeper_.sleepMillis(kPreamblePace);  // paces probes AND yields (radio RX)
  }

  comms_.sendBanner();
  comms_.sendReady();
  state_.health.ready = true;
}

void RobotLoop::zeroUnownedMotion() {
  // navigator_.active() is LOAD-BEARING here, not belt-and-braces.
  //
  // Measured on the playfield 2026-08-06: forward motion under GO_TO was a
  // sawtooth, wheel velocity collapsing from cruise to ~40% and back ~15
  // times in 2.2s (src/tests/bench/output/goto_trace.png). This guard was
  // the cause.
  //
  // Motion::Navigator drives by replacing the in-flight Move every few
  // cycles (planner_.move(..., replace=true)). `Planner::move()` clears
  // `active_.occupied` immediately and only re-activates on the planner's
  // NEXT tick() -- and Navigator::tick() (which owns that call) runs at the
  // END of cycle(). So for the whole top half of every replace cycle
  // `planner_.active()` is false while a goto is very much still running.
  //
  // Without the navigator_ term this function then zeroed both cmdVelocity
  // fields, and drive_.tick() -- two statements later in cycle() -- actuated
  // that zero. One full 50 ms cycle of commanded stop on EVERY replace.
  //
  // The guard's actual contract is "zero the wheels when NOBODY owns
  // motion." Sprint 135 added a third owner and did not tell this function.
  if (planner_.active() || drive_.owns() || navigator_.active()) return;
  state_.wheelLeft.cmdVelocity = 0.0f;
  state_.wheelRight.cmdVelocity = 0.0f;
}

// haltOnStall() -- stakeholder directive 2026-08-08, after the robot repeatedly
// drove into the playfield rails and ground there. Nothing in the firmware
// noticed that a commanded motion was not happening: the wheels kept pushing
// against the obstacle until a human intervened. App::Drive now detects the
// condition (drive.cpp's updateStall) and this stops the robot on it.
//
// This is a HALT, not a report, and it is deliberately the SAME halt ESTOP
// performs -- zero the wheel targets, clear the planner queue, cancel any goto.
// Halting the wheels alone would not do: a queued Move or an in-flight GO_TO
// would resume pushing against the obstacle the instant the current segment
// ended, which is the grinding this exists to stop.
//
// Called BEFORE drive_.tick() in cycle() so the zeroed command reaches the
// motors on this very cycle -- Drive::tick() is what writes duty, so halting
// after it would spend one more cycle (~32ms) driving into the obstacle.
// It acts on the stall the PREVIOUS cycle's tick() latched, which is the
// correct pairing: that tick compared its command against the encoder reading
// published for it.
void RobotLoop::haltOnStall() {
  const bool left = drive_.stallLeft();
  const bool right = drive_.stallRight();
  if (!left && !right) return;

  // Latch into health BEFORE the halt clears Drive's own detector state. The
  // halt erases the condition within a cycle (cmdVelocity drops to zero, so
  // the demand falls below stallDemand), so without this the robot would stop
  // for no reason the host could ever see.
  state_.health.stallLeft = state_.health.stallLeft || left;
  state_.health.stallRight = state_.health.stallRight || right;

  drive_.estop();
  planner_.estop();
  navigator_.cancel();
  state_.wheelLeft.cmdVelocity = 0.0f;
  state_.wheelRight.cmdVelocity = 0.0f;
}

// clearStallLatch() -- the host acknowledging the stall by asking for motion
// again. Every command that starts a new motion clears it, so the flag on the
// wire always refers to THIS motion, never a previous one. Nothing else clears
// it: a stall that is never acknowledged stays visible indefinitely.
void RobotLoop::clearStallLatch() {
  state_.health.stallLeft = false;
  state_.health.stallRight = false;
}

void RobotLoop::publishWheel(Devices::Motor& motor,
                             Types::RobotState::Wheel& wheel,
                             uint8_t& positionEpoch, bool& clamped) {
  float pos = motor.position();
  if (std::fabs(pos) >= kPositionRebaselineMargin) {
    motor.rebaseline();
    pos = motor.position();
    positionEpoch = static_cast<uint8_t>(positionEpoch + 1);
  }
  pos = clampToPositionWireBound(pos, &clamped);

  wheel.position = pos;
  wheel.velocity = motor.velocity();
  wheel.sampleTime = static_cast<uint32_t>(motor.sampleTime() / 1000);  // [us] -> [ms]
  wheel.connected = motor.connected();
  wheel.positionEpoch = positionEpoch;
}

void RobotLoop::publishWheels() {
  bool clampedL = false;
  bool clampedR = false;
  publishWheel(motorL_, state_.wheelLeft, positionEpochLeft_, clampedL);
  publishWheel(motorR_, state_.wheelRight, positionEpochRight_, clampedR);
  state_.health.wedgeLatch = motorL_.wedged() || motorR_.wedged();
  state_.health.wheelFrozenLeft = motorL_.wedgeSuspect();
  state_.health.wheelFrozenRight = motorR_.wedgeSuspect();
  state_.health.positionClamped = clampedL || clampedR;
}

void RobotLoop::publishOtos() {
  const bool otosPresent = otos_.present() && otos_.poseFresh();
  state_.otos.present = otosPresent;
  state_.otos.connected = otos_.connected();
  if (!otosPresent) return;

  const Devices::PoseReading reading = otos_.pose();
  state_.otos.x = reading.x;
  state_.otos.y = reading.y;
  state_.otos.heading = reading.heading;
  state_.otos.v_x = reading.v_x;
  state_.otos.v_y = reading.v_y;
  state_.otos.omega = reading.omega;
  state_.otos.sampleTime = static_cast<uint32_t>(otos_.sampleTime() / 1000);  // [us] -> [ms]
}

bool RobotLoop::tickLineColor(uint64_t nowUs) {
  // Odd cycles read the line sensor, even cycles the colour sensor, so each
  // lands at kCycle/2 for one I2C transaction per cycle between them.
  const bool tickedLine = (cycleCount_ % 2) == 1;
  if (tickedLine) {
    line_.tick(nowUs);
  } else {
    color_.tick(nowUs);
  }
  return tickedLine;
}

void RobotLoop::publishLineColor(bool tickedLine) {
  const bool lineFresh = tickedLine && line_.readingFresh();
  const bool colorFresh = !tickedLine && color_.readingFresh();

  state_.perception.lineFresh = lineFresh;
  state_.perception.colorFresh = colorFresh;
  if (lineFresh) state_.perception.line = packLine(line_.reading());
  if (colorFresh) state_.perception.color = packColor(color_.reading(), color_.fullScale());

  // Validity LATCHES on the first reading and only drops when the leaf
  // itself reports the device gone -- it deliberately does NOT track
  // per-cycle freshness. Only one leaf is read per cycle, so tying validity
  // to freshness took the other sensor off the wire entirely on every
  // alternate cycle, and any consumer sampling at that cadence could see one
  // of them never (measured on tovez 2026-08-07 when the telemetry emit
  // floor aliased with this parity: `line_present 93, color_present 0`).
  // A value up to one alternation old is worth far more than no value, and
  // `*Fresh` above is there for anyone who needs to tell the difference.
  state_.perception.lineValid = (state_.perception.lineValid || lineFresh) && line_.connected();
  state_.perception.colorValid = (state_.perception.colorValid || colorFresh) && color_.connected();
}

void RobotLoop::applySeed() {
  const Comms::SeedRequest seed = comms_.takeSeed();
  if (!seed.pending) return;

  // Both pose sources, one fix: the chip (lever arm applied inside setPose)
  // and the encoder odometry. Their later divergence is the drift we measure.
  otos_.setPose(seed.x, seed.y, seed.heading);
  odom_.reset(seed.x, seed.y, seed.heading, motorL_.position(), motorR_.position());

  if (seed.reply != nullptr) {
    // Heading as integer milliradians: newlib-nano's printf has no float.
    char line[64];
    std::snprintf(line, sizeof(line), "SEED:%d:%d:%d",
                  static_cast<int>(seed.x), static_cast<int>(seed.y),
                  static_cast<int>(seed.heading * 1000.0f));
    seed.reply->sendReliable(line);
  }
}

void RobotLoop::publishPose() {
  float twistVx = 0.0f;
  float twistOmega = 0.0f;
  BodyKinematics::forward(motorL_.velocity(), motorR_.velocity(),
                          drive_.trackWidth(), twistVx, twistOmega);

  state_.pose.x = odom_.x();
  state_.pose.y = odom_.y();
  state_.pose.heading = odom_.theta();
  state_.pose.v_x = twistVx;
  state_.pose.v_y = 0.0f;
  state_.pose.omega = twistOmega;
}

void RobotLoop::ackDriveCompletion() {
  uint32_t moveId = 0;
  if (drive_.takeCompletion(&moveId)) tlm_.ack(moveId, 0);
}

void RobotLoop::publishHealth() {
  const bool moving = planner_.active() || drive_.owns();
  state_.command.mode = moving ? Types::Mode::Velocity : Types::Mode::Idle;
  state_.command.moveActive = moving;
  state_.health.i2cSafetyNetCount = bus_.clearanceSafetyNetCount();
  state_.health.commsMalformedCount = comms_.malformedCount();
  state_.health.commandsDroppedCount = comms_.commandsDroppedCount();
}

void RobotLoop::publishTiming(uint64_t cycleStartUs) {
  state_.time.cycleBusy = static_cast<uint32_t>(clock_.nowMicros() - cycleStartUs);  // [us]
  state_.time.cyclePeriod =
      everCycled_ ? static_cast<uint32_t>(cycleStartUs - previousCycleStartUs_) : 0u;  // [us]
  previousCycleStartUs_ = cycleStartUs;
  everCycled_ = true;
}

void RobotLoop::publishMoveResult(const Motion::TickResult& moveResult) {
  state_.health.moveTimeout = moveResult.completed && moveResult.timedOut;
  state_.health.shapingDisabled =
      planner_.active() && !planner_.shaperConfigured();

  tlm_.setLiveFlag(kFlagFaultMoveTimeout, state_.health.moveTimeout);
  tlm_.setLiveFlag(kFlagFaultShapingDisabled, state_.health.shapingDisabled);

  if (moveResult.completed) {
    tlm_.ack(moveResult.moveId, 0);  // timeout signaled by flag, not ack_err
  }
}

// publishGotoResult() -- 135-004, NavResult's own counterpart to
// publishMoveResult() above, called instead of it on a cycle a goto owns
// Drive (cycle()'s own if/navigator_.active() branch). Mirrors both of
// publishMoveResult()'s fault computations exactly, computed every cycle
// (not just on completion) so neither flag can linger stale from an
// earlier, unrelated ordinary Move for this goto's entire duration:
// shapingDisabled reads planner_.active() directly (still meaningful --
// Navigator's own internal segment Moves ARE ordinary Planner Moves);
// moveTimeout mirrors NavResult's own completed/fault pair the same
// "timeout signaled by flag, not ack_err" way an ordinary Move's own
// completed/timedOut pair already does -- NavResult carries no err code
// of its own (just Done vs Aborted), and reusing this EXISTING flag bit
// for "this motion ended via a safety backstop, not a clean arrival"
// avoids a new wire flag bit this ticket's own scope does not call for.
//
// Landmine 1 fix: this is the ONLY place a goto's completion reaches
// tlm_.ack() -- the internal TickResult Navigator consumes every cycle
// (inside its own tick(), navigator.cpp) never reaches this function or
// publishMoveResult() at all, so an internal segment's own completion
// (id == 0, every cycle a replace lands) can never fire a spurious ack(0).
void RobotLoop::publishGotoResult(const Motion::NavResult& navResult) {
  state_.health.moveTimeout = navResult.completed && navResult.fault;
  state_.health.shapingDisabled = planner_.active() && !planner_.shaperConfigured();

  tlm_.setLiveFlag(kFlagFaultMoveTimeout, state_.health.moveTimeout);
  tlm_.setLiveFlag(kFlagFaultShapingDisabled, state_.health.shapingDisabled);

  if (navResult.completed) {
    tlm_.ack(navResult.id, 0);
  }
}

void RobotLoop::cycle() {
  state_.time.cycleStart = markTime();  // [ms] pace anchor + wire `now`
  const uint64_t cycleStartUs = clock_.nowMicros();  // [us]
  ++cycleCount_;  // drives the line/color alternation in the pace block below

  zeroUnownedMotion();
  haltOnStall();

  drive_.tick(state_);

  motorL_.requestSample();  // brick latches ONE pending read per select
  runAndWait(kSettle, [&] {
    comms_.pump(state_.time.cycleStart);
  });
  motorL_.tick(clock_.nowMicros());  // collect L

  motorR_.requestSample();
  runAndWait(kSettle, [&] {
    comms_.pump(state_.time.cycleStart);
    Cmd cmd;
    while (comms_.takeCommand(cmd)) routeCommand(cmd);
  });
  motorR_.tick(clock_.nowMicros());  // collect R

  publishWheels();  // at the point of same-generation L/R coherence

  runAndWaitUntil(state_.time.cycleStart, kCycle, [&] {
    comms_.pump(state_.time.cycleStart);

    uint64_t nowUs = clock_.nowMicros();

    odom_.integrate(motorL_.position(), motorR_.position(), state_.wheelLeft.positionEpoch,
                    state_.wheelRight.positionEpoch);  // before OTOS: FakeOtos reads it
    otos_.tick(nowUs);
    publishOtos();
    publishLineColor(tickLineColor(nowUs));
    publishPose();
    publishHealth();
    publishTiming(cycleStartUs);

    tlm_.update(state_, drive_);

#ifdef ROBOT_DEBUG
    applyDbgAction(state_.time.cycleStart);
#endif

    applySeed();

    const Comms::TlmAction tlmAction = comms_.takeTlmAction();
    const bool forceFrame = tlm_.applyAction(tlmAction);

    tlm_.emit(state_.time.cycleStart, forceFrame);

    comms_.updateStatus(state_, tlm_);

    comms_.sendTlmReply(tlmAction);

    // 135-004: exactly ONE of Navigator or the Planner may call
    // planner_.tick()/update() this cycle -- navigator.h's own "one
    // owner" doc comment (Navigator::tick() calls both ITSELF,
    // internally). navigator_.active() is Navigator's own "does a goto
    // currently own Drive" signal, already correct by this point in the
    // schedule: handleGoto()/navigator_.start() (if a GO_TO arrived this
    // cycle) already ran inside routeCommand(), earlier in cycle(), and
    // navigator_.cancel() (if a MOVE/WHEELS/ESTOP preempted a goto
    // instead) ran there too.
    if (navigator_.active()) {
      const Motion::NavResult navResult = navigator_.tick(state_);
      publishGotoResult(navResult);
    } else {
      const Motion::TickResult moveResult = planner_.tick(state_);
      planner_.update(state_);
      publishMoveResult(moveResult);
    }
    drive_.update(state_, state_.time.cycleStart);
    ackDriveCompletion();
  });
}

#ifdef ROBOT_DEBUG
void RobotLoop::captureTuningBaseline() {
  if (dbgTuningBaselined_) return;
  dbgBoundsBaseline_ = drive_.adaptationBounds();
  dbgDutyPerSpeedBaseLeft_ = drive_.dutyPerSpeedLeft();
  dbgDutyPerSpeedBaseRight_ = drive_.dutyPerSpeedRight();
  dbgTuningBaselined_ = true;
}

void RobotLoop::applyDbgAction(uint32_t now) {
  if (dbgWedgeUntilL_ != 0 && dbgWedgeUntilL_ != UINT32_MAX &&
      static_cast<int32_t>(now - dbgWedgeUntilL_) >= 0) {
    motorL_.setForcedWedge(false);
    dbgWedgeUntilL_ = 0;
  }
  if (dbgWedgeUntilR_ != 0 && dbgWedgeUntilR_ != UINT32_MAX &&
      static_cast<int32_t>(now - dbgWedgeUntilR_) >= 0) {
    motorR_.setForcedWedge(false);
    dbgWedgeUntilR_ = 0;
  }

  for (Comms::DbgAction action = comms_.takeDbgAction();
       action.kind != Comms::DbgActionKind::kNone;
       action = comms_.takeDbgAction()) {
  switch (action.kind) {
    case Comms::DbgActionKind::kNone:
      break;
    case Comms::DbgActionKind::kMark:
      App::debugf("%s", action.text);
      break;
    case Comms::DbgActionKind::kPing:
      App::debugf("pong");
      break;
    case Comms::DbgActionKind::kWedge: {
      const uint32_t until =
          action.duration == 0 ? UINT32_MAX : now + action.duration;
      if (action.port & 1) { motorL_.setForcedWedge(true); dbgWedgeUntilL_ = until; }
      if (action.port & 2) { motorR_.setForcedWedge(true); dbgWedgeUntilR_ = until; }
      App::debugf("wedge armed port=%u dur=%lu", action.port,
                  static_cast<unsigned long>(action.duration));
      break;
    }
    case Comms::DbgActionKind::kVmin: {
      captureTuningBaseline();
      drive_.setSpeedFloor(action.value);
      const long milli = DBG_MILLI(action.value);
      App::debugf("vmin %ld.%03ld applied", milli / 1000, milli % 1000);
      break;
    }
    case Comms::DbgActionKind::kASteady: {
      captureTuningBaseline();
      drive_.setASteady(action.value);
      const long milli = DBG_MILLI(action.value);
      App::debugf("asteady %ld.%03ld applied", milli / 1000, milli % 1000);
      break;
    }
    case Comms::DbgActionKind::kPos: {
      captureTuningBaseline();
      drive_.setPositionErrorMax(action.value);
      const long milli = DBG_MILLI(action.value);
      App::debugf("pos %ld.%03ld applied", milli / 1000, milli % 1000);
      break;
    }
    case Comms::DbgActionKind::kGain: {
      captureTuningBaseline();
      drive_.setDutyPerSpeed(dbgDutyPerSpeedBaseLeft_ * action.value,
                             dbgDutyPerSpeedBaseRight_ * action.value2);
      const long milliLeft = DBG_MILLI(action.value);
      const long milliRight = DBG_MILLI(action.value2);
      App::debugf("gain L=%ld.%03ld R=%ld.%03ld applied",
                  milliLeft / 1000, milliLeft % 1000,
                  milliRight / 1000, milliRight % 1000);
      break;
    }
    case Comms::DbgActionKind::kClear:
      motorL_.setForcedWedge(false);
      motorR_.setForcedWedge(false);
      dbgWedgeUntilL_ = 0;
      dbgWedgeUntilR_ = 0;
      if (dbgTuningBaselined_) {
        drive_.setAdaptationBounds(dbgBoundsBaseline_);
        drive_.setDutyPerSpeed(dbgDutyPerSpeedBaseLeft_,
                               dbgDutyPerSpeedBaseRight_);
        dbgTuningBaselined_ = false;
        App::debugf("clear (tuning restored to boot)");
      } else {
        App::debugf("clear");
      }
      break;
    case Comms::DbgActionKind::kOtos: {
      // begin() latches initialized_ once at boot, so a re-probe here is the
      // only way to see the raw id or whether a later probe would succeed.
      App::debugf("otos before: present=%d connected=%d probeId=0x%02X",
                  otos_.present() ? 1 : 0, otos_.connected() ? 1 : 0,
                  static_cast<unsigned>(otos_.lastProbeId()));
      otos_.begin();
      App::debugf("otos after: present=%d connected=%d probeId=0x%02X",
                  otos_.present() ? 1 : 0, otos_.connected() ? 1 : 0,
                  static_cast<unsigned>(otos_.lastProbeId()));
      break;
    }
    case Comms::DbgActionKind::kUnrecognized:
      App::debugf("unrecognized dbg: %s", action.text);
      break;
  }
  }
}
#endif  // ROBOT_DEBUG

}  // namespace App
