#include "app/robot_loop.h"
#include "app/debug.h"

#include <cmath>

#include "messages/envelope.h"
#include "motion/body_kinematics.h"

namespace App {

namespace {

constexpr uint32_t kSettle = 4;  // [ms] encoder-settle window, both motors
constexpr uint32_t kClear = 4;  // [ms] post-duty-write clearance window

constexpr uint32_t kWindows = 2 * kSettle + kClear;  // [ms]
static_assert(kWindows <= RobotLoop::kCycle,
              "kSettle+kClear+kSettle must fit inside the kCycle budget");

constexpr uint32_t kPreamblePace = 10;  // [ms] boot-loop probe pacing

constexpr float kPositionWireBound = 32000.0f;  // [mm]
constexpr float kPositionRebaselineMargin = 30000.0f;  // [mm]

uint32_t packLine(const Devices::LineReading& reading) {
  return (reading.raw[0] & 0xFFu) | ((reading.raw[1] & 0xFFu) << 8) |
         ((reading.raw[2] & 0xFFu) << 16) | ((reading.raw[3] & 0xFFu) << 24);
}

uint32_t packColor(const Devices::ColorReading& reading) {
  return ((reading.r >> 8) & 0xFFu) | (((reading.g >> 8) & 0xFFu) << 8) |
         (((reading.b >> 8) & 0xFFu) << 16) | (((reading.c >> 8) & 0xFFu) << 24);
}

}

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
      // 132-013 (patch-surface retirement): CONFIG now carries a
      // SetConfigGroup (robot_config.proto), not the deleted ConfigDelta --
      // a whole-group push, decoded straight into Configurator's owned
      // Config::Robot via applyGroup(), same "no per-command ReplyEnvelope,
      // outcome rides the ack ring" shape as SET_FIELD below. The
      // Configurator owns everything a CONFIG means (configurator.h);
      // RobotLoop's whole job here is the ack.
      tlm_.ack(cmd.env.corr_id,
               static_cast<uint32_t>(configurator_.applyGroup(
                   cmd.env.cmd.config.target, cmd.env.cmd.config.body_,
                   cmd.env.cmd.config.body_count)));
      break;
    case msg::CommandEnvelope::CmdKind::GET_CONFIG:
      handleGetConfig(cmd.env);
      break;
    case msg::CommandEnvelope::CmdKind::SET_FIELD:
      // 132-012 (SetConfigField / Configurator::applyField()): the
      // single-field dev-mode push, same "no per-command ReplyEnvelope,
      // outcome rides the ack ring" shape as CONFIG just above -- unlike
      // GET_CONFIG, which replies synchronously (a group's worth of
      // values has no room in the ack ring).
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

// handleGetConfig() -- 132-011 (GetConfig/ConfigSnapshot wire read-back):
// the one CONFIG-arm outcome that does NOT ride the ack ring (§7.1,
// docs/protocol-v5.md) -- a ring entry has no room for a group's worth of
// values, so this replies SYNCHRONOUSLY via Comms::sendReply(), the same
// path App::Telemetry::emitPrimary() uses for its own unsolicited push
// (the only other live call site, until now). Read-back is honest for
// every ConfigGroupTarget, including boot-only ones (GEOMETRY/PLANNER) --
// Configurator::encodeSnapshot() is deliberately NOT gated by
// isLiveConfigurable() the way applyGroup()/install() are; only WRITES are
// gated.
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
    // An unrecognized target is reported, never silently dropped -- the
    // same "loud rejection, not silence" discipline applyGroup()'s own
    // ERR_NOT_LIVE rejection follows for writes.
    reply.body_kind = msg::ReplyEnvelope::BodyKind::ERR;
    reply.body.err.code = result;
    reply.body.err.field = static_cast<uint32_t>(target);
  }
  comms_.sendReply(reply);
}

// Every MOVE goes to the planner, twist or wheels-velocity, whatever its
// stop condition -- never diverted to Drive, which has no odometry and so
// cannot honor a DISTANCE/ANGLE stop condition.
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
      break;
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

  // The caller's INTENT, captured before the calibration rewrite below
  // turns `threshold` into an actuation-sized command (134-001). Nothing
  // downstream can reconstruct it once that happens, and
  // Motion::Planner's cumulative-baseline ledger needs it: a chained turn
  // must repay the residual its predecessor left, which means targeting
  // "baseline + what this Move was ASKED to turn", not "baseline + what we
  // commanded the wheels to do". Set for every Kind -- only the Angle
  // branch of the ledger reads it today, but the field means the same
  // thing on every axis and a Kind-conditional assignment would be a trap
  // for the next reader.
  m.requestedThreshold = m.threshold;

  // Turn calibration. The measured response of an ANGLE-stopped move is
  // affine, actual = gain * commanded + offset, so to LAND on the requested
  // angle we command (requested - offset) / gain.
  //
  // Measured 2026-07-29 against overhead camera truth, 48 shuffled in-place
  // turns: 15..180 deg, both directions, omega 2.5..8.0 rad/s. The same law
  // holds at every rate -- this is geometry/stiction, not a deceleration
  // artifact -- so ONE pair of constants per direction covers the range.
  //
  // The offset is what makes this worth doing: it is roughly -6 deg
  // regardless of size, which is invisible at 180 and ruinous at 15 (a 15
  // deg command landed at 9). A pure scale factor cannot correct it, which
  // is why calibrating against 180-degree turns alone did not transfer down.
  //
  // Per direction because the two gearboxes are not identical. Direction is
  // taken from the commanded rotation itself: omega for a twist, the wheel
  // difference for a wheels-velocity move.
  if (m.kind == Motion::Move::Kind::Angle) {
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
    return false;
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

  planner_.estop();
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
  // Zero the blackboard's own commanded speeds too. Both subsystems'
  // update() calls would land the same zeros at the end of this cycle
  // anyway, but an emergency stop should not depend on the rest of the
  // schedule running to completion to take effect.
  //
  // 133-001: this is App::RobotLoop acting as the SAFETY ARBITER, not as a
  // third decider -- the same rule zeroUnownedMotion() runs under, stated
  // in full at publishWheels() below. Zero-only, superseding every
  // decider. It is an instance of the invariant, not an unexplained
  // special case.
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

    preamble_.step();

    Types::RobotState bootState;
    bootState.time.cycleStart = markTime();
    bootState.wheelLeft.connected = preamble_.leftConnected();
    bootState.wheelRight.connected = preamble_.rightConnected();
    bootState.otos.connected = preamble_.otosConnected();
    tlm_.update(bootState, drive_);
    tlm_.emit(bootState.time.cycleStart);

    sleeper_.sleepMillis(kPreamblePace);
  }

  comms_.sendBanner();
  comms_.sendReady();
  state_.health.ready = true;
}


// UNOWNED-MOTION GUARD (133-001, the 2026-08-03 runaway). Measured on
// vevov, 16/16 reproductions: after a WHEELS command expires with the host
// silent, the wheels keep turning at the last commanded speed with no
// decay -- 936mm of continued travel, still going when the capture ended,
// and estop() failed to stop it in 5 of 6 attempts. The Nezha brick
// physically latches its last commanded speed and does not reset on an
// nRF52 reset, so a lost zero write is permanent, not transient.
//
// Every individual link was defensible in isolation, which is exactly why
// this belongs here rather than inside one of them: Drive::update()
// publishes one zero pair on the expiry cycle and then returns early
// forever after (`if (!owned) return;`), and Planner::update() runs
// unconditionally but republishes only ITS OWN idea of the command.
// Nothing re-stated "no one is driving, so the speed is zero" on the
// cycles in between.
//
// This states it, every cycle, ahead of the single actuation path, so the
// wheels cannot inherit a stale target from a decider that has stopped
// publishing. Idleness is DERIVED from the two existing public ownership
// queries -- no new interface, no new handoff edge between the deciders,
// and no idle owner that would have to be told to take over (see this
// method's own doc comment, robot_loop.h, for why that distinction is the
// whole design). Both deciders write cmdVelocity later in the same cycle
// anyway, so when either owns the wheels this is a plain no-op.
void RobotLoop::zeroUnownedMotion() {
  if (planner_.active() || drive_.owns()) return;
  // ONLY 0.0f is ever written here -- the monotone contract (robot_loop.h).
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

// THE cmdVelocity OWNERSHIP INVARIANT (revised 133-001; this comment used
// to claim "exactly one writer per cycle," which was already false on
// master -- handleEstop() below has written cmdVelocity from the loop
// since 128-001, with a comment explaining why that is correct. The honest
// invariant was never "one writer"; it was "one decider plus a safety
// override," merely undocumented):
//
//   cmdVelocity has exactly one DECIDER per cycle -- Motion::Planner::
//   update() or App::Drive::update() -- and exactly one SAFETY ARBITER,
//   App::RobotLoop, whose writes are restricted to zero, which runs after
//   every decider and before actuation, and which supersedes all deciders.
//   No other writer exists.
//
// App::RobotLoop's arbiter writes are exactly two, both zero-only:
// zeroUnownedMotion() (the per-cycle derived-idle guard) and
// handleEstop() (the ESTOP wire verb). Adding a third is a change to this
// invariant, not a local edit.
//
// publishWheels() itself deliberately does NOT touch wheel.cmdVelocity --
// it is neither a decider nor the arbiter, it is the measurement publisher.
// Writing it from here would overwrite the owner's staged command with the
// value being actuated THIS cycle, one generation stale.
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
    tlm_.ack(moveResult.moveId, 0);
  }
}

void RobotLoop::cycle() {
  state_.time.cycleStart = markTime();  // [ms] pace anchor + wire `now`
  const uint64_t cycleStartUs = clock_.nowMicros();  // [us]

  // Safety arbitration, LAST thing before actuation: if neither decider
  // owns motion, the commanded speed is zero (133-001). Zero-only by
  // contract -- see this method's own doc comment (robot_loop.h) and its
  // definition above.
  zeroUnownedMotion();

  // Actuate the speeds the owning subsystem staged onto the blackboard last
  // cycle -- one actuation path regardless of which decider produced them
  // (one cycle of command-to-wheels latency, the same the planner's own
  // actuationDelay compensates for).
  drive_.tick(state_);

  motorL_.requestSample();
  runAndWait(kSettle, [&] {
    comms_.pump(state_.time.cycleStart);
  });
  motorL_.tick(clock_.nowMicros());

  runAndWait(kClear, [&] {
    comms_.pump(state_.time.cycleStart);
  });

  motorR_.requestSample();
  runAndWait(kSettle, [&] {
    comms_.pump(state_.time.cycleStart);
    Cmd cmd;
    while (comms_.takeCommand(cmd)) routeCommand(cmd);
  });
  motorR_.tick(clock_.nowMicros());

  publishWheels();

  runAndWaitUntil(state_.time.cycleStart, kCycle, [&] {
    comms_.pump(state_.time.cycleStart);

    uint64_t nowUs = clock_.nowMicros();

    odom_.integrate(motorL_.position(), motorR_.position(), state_.wheelLeft.positionEpoch,
                    state_.wheelRight.positionEpoch);
    otos_.tick(nowUs);
    publishOtos();
    const bool tickedLine = (cycleCount_ % 2) == 1;
    if (tickedLine) line_.tick(nowUs); else color_.tick(nowUs);
    publishLineColor(tickedLine);
    publishPose();
    publishHealth();
    publishTiming(cycleStartUs);

    tlm_.update(state_, drive_);

#ifdef ROBOT_DEBUG
    applyDbgAction(state_.time.cycleStart);
#endif

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
    // --- 133-003 live tuning arms ---------------------------------------
    //
    // Every echo below reports the value that LANDED, formatted from the
    // float actually handed to the setter -- never a re-print of the wire
    // text. newlib-nano's printf has no float support (debug.h's own
    // header), so each splits DBG_MILLI()'s rounded integer milli-unit
    // into whole and thousandths parts: "vmin 60.000 applied". Rounding
    // rather than truncating matters at this scale -- static_cast<int>(
    // 1.02f * 1000.0f) is 1019, so a truncating echo would report a gain
    // of 1.019 for a push of 1.02 and send an operator hunting a
    // firmware bug that is really a printf.
    //
    // The literal token "applied" is a CONTRACT with
    // src/tests/bench/velocity_profile_gate.py's _assert_tuning(), which
    // waits for it before it will report a run. Do not reword it.
    case Comms::DbgActionKind::kVmin: {
      captureTuningBaseline();
      drive_.setSpeedFloor(action.value);
      const long milli = DBG_MILLI(action.value);
      App::debugf("vmin %ld.%03ld applied", milli / 1000, milli % 1000);
      break;
    }
    case Comms::DbgActionKind::kASteady: {
      // Stage C's bias adaptation gates on |cmdAccel| < aSteady, so at the
      // baked 30 mm/s^2 it is frozen through every ramp the profile gate
      // drives (even the gentle 200 mm/s trapezoid ramps at ~267 mm/s^2).
      // Raising this is how 004 finds out whether the residual L/R
      // distance imbalance is banked in the ramps.
      captureTuningBaseline();
      drive_.setASteady(action.value);
      const long milli = DBG_MILLI(action.value);
      App::debugf("asteady %ld.%03ld applied", milli / 1000, milli % 1000);
      break;
    }
    case Comms::DbgActionKind::kPos: {
      // Stage B's position-error clamp [mm] -- 133-002's setter. The
      // INPUT-side bound; its OUTPUT-side sibling (ControlGains::iMax) has
      // a CONFIG key already. NOTE for a tuning session: pos_err_max IS in
      // the robot JSON and IS live-configurable over the WHEEL_CONTROL
      // wire group, so this verb is the RAM-only shortcut, not the only
      // path -- a value worth keeping goes in data/robots/<robot>.json.
      captureTuningBaseline();
      drive_.setPositionErrorMax(action.value);
      const long milli = DBG_MILLI(action.value);
      App::debugf("pos %ld.%03ld applied", milli / 1000, milli % 1000);
      break;
    }
    case Comms::DbgActionKind::kGain: {
      // Per-wheel multiplier on the BOOT-INSTALLED dutyPerSpeed pair --
      // the L/R plateau-speed imbalance trim. Multiplying the captured
      // baseline (not the current value) makes a re-push idempotent and
      // `gain 1 1` a true restore; see captureTuningBaseline()'s own doc
      // comment (robot_loop.h) for why the baseline is the installed pair
      // rather than Drive::kDutyPerSpeed.
      //
      // MEASURED, and the reason that distinction is not academic: the Sim
      // composition root boots at dutyPerSpeed 0.002, while
      // Drive::kDutyPerSpeed is 0.001182. Scaling the CONSTANT would make
      // `DBG:gain 1 1` -- the documented identity push -- a silent 41%
      // recalibration there, and the same trap waits on any robot whose
      // JSON does not happen to carry 0.001182. See
      // src/tests/sim/system/test_dbg_tuning_verbs.py's own idempotence
      // and clear-restores scenarios.
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
      // "clear every injected override" (this verb's own documented
      // contract, comms.cpp) now genuinely means EVERY one: the tuning
      // arms above are overrides too, and leaving them latched after a
      // clear would let a sweep's last value silently ride into the next
      // measurement. Restores only when a tuning verb actually landed --
      // otherwise there is no baseline and nothing to restore, and
      // writing zeros here would UNCALIBRATE the robot.
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
    case Comms::DbgActionKind::kUnrecognized:
      App::debugf("unrecognized dbg: %s", action.text);
      break;
  }
  }
}
#endif

}
