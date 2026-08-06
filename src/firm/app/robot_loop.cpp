#include "app/robot_loop.h"
#include "app/debug.h"

#include <cmath>
#include <cstdio>

#include "messages/envelope.h"
#include "motion/body_kinematics.h"

namespace App {

namespace {

constexpr uint32_t kSettle = 4;  // [ms] encoder-settle window, both motors
constexpr uint32_t kClear = 4;   // [ms] post-duty-write clearance window

constexpr uint32_t kWindows = 2 * kSettle + kClear;  // [ms]
static_assert(kWindows <= RobotLoop::kCycle,
              "kSettle+kClear+kSettle must fit inside the kCycle budget");

constexpr uint32_t kPreamblePace = 10;  // [ms] boot-loop probe pacing

constexpr float kPositionWireBound = 32000.0f;       // [mm]
constexpr float kPositionRebaselineMargin = 30000.0f;  // [mm]

uint32_t packLine(const Devices::LineReading& reading) {
  return (reading.raw[0] & 0xFFu) | ((reading.raw[1] & 0xFFu) << 8) |
         ((reading.raw[2] & 0xFFu) << 16) | ((reading.raw[3] & 0xFFu) << 24);
}

uint32_t packColor(const Devices::ColorReading& reading) {
  return ((reading.r >> 8) & 0xFFu) | (((reading.g >> 8) & 0xFFu) << 8) |
         (((reading.b >> 8) & 0xFFu) << 16) | (((reading.c >> 8) & 0xFFu) << 24);
}

}  // namespace

RobotLoop::RobotLoop(Devices::I2CBus& bus, Devices::Motor& motorL,
                      Devices::Motor& motorR, Devices::Otos& otos,
                      Devices::ColorSensorLeaf& color, Devices::LineSensorLeaf& line,
                      Comms& comms, Telemetry& tlm, Drive& drive,
                      Configurator& configurator, Motion::Odometry& odom,
                      Motion::Planner& planner, Preamble& preamble,
                      const Devices::Clock& clock,
                      Devices::Sleeper& sleeper)
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
  if (!configured_) {
    tlm_.ack(env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_NOT_CONFIGURED));
    return;
  }

  const msg::Wheels& wheels = env.cmd.wheels;
  if (wheels.duration <= 0.0f) {
    tlm_.ack(env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_BADARG));
    return;
  }

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
  drive_.estop();
  planner_.estop();
  state_.wheelLeft.cmdVelocity = 0.0f;
  state_.wheelRight.cmdVelocity = 0.0f;
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
  if (planner_.active() || drive_.owns()) return;
  state_.wheelLeft.cmdVelocity = 0.0f;
  state_.wheelRight.cmdVelocity = 0.0f;
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

void RobotLoop::publishLineColor(bool tickedLine) {
  const bool lineFresh = tickedLine && line_.readingFresh();
  const bool colorFresh = !tickedLine && color_.readingFresh();

  state_.perception.lineFresh = lineFresh;
  state_.perception.colorFresh = colorFresh;
  if (lineFresh) state_.perception.line = packLine(line_.reading());
  if (colorFresh) state_.perception.color = packColor(color_.reading());
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

void RobotLoop::cycle() {
  state_.time.cycleStart = markTime();  // [ms] pace anchor + wire `now`
  const uint64_t cycleStartUs = clock_.nowMicros();  // [us]

  zeroUnownedMotion();

  drive_.tick(state_);

  motorL_.requestSample();  // brick latches ONE pending read per select
  runAndWait(kSettle, [&] {
    comms_.pump(state_.time.cycleStart);
  });
  motorL_.tick(clock_.nowMicros());  // collect L

  runAndWait(kClear, [&] {  // brick's post-duty-write clearance
    comms_.pump(state_.time.cycleStart);
  });

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
    const bool tickedLine = (cycleCount_ % 2) == 1;  // first cycle ticks line
    if (tickedLine) line_.tick(nowUs); else color_.tick(nowUs);
    publishLineColor(tickedLine);
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

    const Motion::TickResult moveResult = planner_.tick(state_);
    planner_.update(state_);
    drive_.update(state_, state_.time.cycleStart);
    ackDriveCompletion();
    publishMoveResult(moveResult);
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
