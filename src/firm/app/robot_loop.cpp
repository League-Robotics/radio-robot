// robot_loop.cpp -- App::RobotLoop implementation. See robot_loop.h and
// DESIGN.md for the schedule and routing rationale.
#include "app/robot_loop.h"
#include "app/debug.h"

#include <cmath>

#include "messages/envelope.h"
#include "motion/body_kinematics.h"

namespace App {

namespace {

constexpr uint32_t kSettle = 4;  // [ms] encoder-settle window, both motors
constexpr uint32_t kClear = 4;   // [ms] post-duty-write clearance window

// The three settle/clear windows have a fixed budget of their own; the
// trailing pacing block (cycle()'s own runAndWaitUntil() call, below) does
// NOT get a fourth fixed budget on top of them any more (131-005) -- it
// targets the cycle's own absolute end, state_.time.cycleStart + kCycle,
// so whatever real time the three windows above (plus their own bodies'
// work) actually consumed this cycle is exactly what the final block's
// wait shrinks by. This static_assert stays: the three windows must still
// leave a nonnegative amount of the budget for that final wait to absorb.
constexpr uint32_t kWindows = 2 * kSettle + kClear;  // [ms]
static_assert(kWindows <= RobotLoop::kCycle,
              "kSettle+kClear+kSettle must fit inside the kCycle budget");

constexpr uint32_t kPreamblePace = 10;  // [ms] boot-loop probe pacing

// kPositionWireBound must match telemetry.proto EncoderReading.position abs_max.
constexpr float kPositionWireBound = 32000.0f;       // [mm]
constexpr float kPositionRebaselineMargin = 30000.0f;  // [mm]

// 4 grayscale channels into one uint32, ch1 low byte (telemetry.proto `line`).
uint32_t packLine(const Devices::LineReading& reading) {
  return (reading.raw[0] & 0xFFu) | ((reading.raw[1] & 0xFFu) << 8) |
         ((reading.raw[2] & 0xFFu) << 16) | ((reading.raw[3] & 0xFFu) << 24);
}

// RGBC, 16->8 bit per channel, R low byte (telemetry.proto `color`).
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
  // On overrun, yield (never sleep: fiber sleeps quantize to whole ticks).
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

// 131-005: see this method's own doc comment (robot_loop.h). Unlike
// runAndWait() above, the mark this waits against is supplied by the
// CALLER (the cycle's own start, not this call's own entry) -- the whole
// point being that any jitter/rounding the earlier blocks picked up this
// cycle already shows up in markTime() by the time this runs, so
// sleepUntil() below computes a correspondingly SMALLER remaining wait
// rather than a fixed one.
template <typename Body>
void RobotLoop::runAndWaitUntil(uint32_t deadlineMark, uint32_t deadlineGap, Body body) {  // [ms] [ms]
  body();
  sleepUntil(deadlineMark, deadlineGap);
}

float RobotLoop::clampToPositionWireBound(float pos, bool* clamped) {
  *clamped = std::fabs(pos) > kPositionWireBound;
  return *clamped ? std::copysign(kPositionWireBound, pos) : pos;
}

// --- Routing ------------------------------------------------------------

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
      // The Configurator owns everything a CONFIG means (configurator.h);
      // RobotLoop's whole job here is the ack.
      tlm_.ack(cmd.env.corr_id, configurator_.apply(cmd.env));
      break;
    case msg::CommandEnvelope::CmdKind::GET_CONFIG:
      handleGetConfig(cmd.env);
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

  // Convert the wire Move to the planner's Motion::Move; the planner
  // activates it on its next tick().
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
      // Never invert or zero the request: a correction large enough to do
      // that means the constants are wrong, and turning backwards is worse
      // than turning inaccurately.
      m.threshold = (corrected > 0.0f) ? corrected : m.threshold;
    }
  }

  // A retried enqueue whose original ack was lost carries this same id under
  // a fresh corr_id. Ack it as success -- the move genuinely is enqueued,
  // running, or done, and an error would make the host abandon a move that
  // actually executed. Returning here also skips the drive_.takeover() below,
  // which would otherwise disturb a planner move already in flight, and
  // precedes any `replace` handling, so a duplicate cannot restart a move
  // mid-flight.
  if (alreadyAccepted(move.id)) {
    tlm_.ack(env.corr_id, 0);
    return;
  }

  // 131-001: takeover(), not estop() -- the planner takes over motion (one
  // owner at a time), but Drive's learned state (Stage B's integrators,
  // Stage C's bias, the deficit latch) survives the handover, so a
  // chained tour's adaptation is not cold-started on every leg (drive.h's
  // own header, Decisions 1/2).
  drive_.takeover();
  const bool accepted = planner_.move(m, move.replace);
  // Only a real accept is recorded: an ERR_FULL move never ran, and the host
  // is entitled to send it again once the queue drains.
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

// WHEELS: the dumb teleop primitive. Straight to Drive, superseding the
// planner -- no queue, no stop condition, just a wheel pair held for a
// bounded window.
void RobotLoop::handleWheels(const msg::CommandEnvelope& env) {
  if (!configured_) {
    tlm_.ack(env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_NOT_CONFIGURED));
    return;
  }

  const msg::Wheels& wheels = env.cmd.wheels;
  if (wheels.duration <= 0.0f) {
    // A wheel command with no positive duration is unbounded, which is the
    // one thing this verb exists to make impossible.
    tlm_.ack(env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_BADARG));
    return;
  }

  planner_.estop();  // Drive takes over motion (one owner at a time)
  drive_.command(wheels.v_left, wheels.v_right, wheels.duration, wheels.id,
                 state_.time.cycleStart);
  tlm_.ack(env.corr_id, 0);
}

// STOP: the PLANNED stop -- a queue entry, executed in sequence. The ack
// here is the ENQUEUE ack (corr_id); the completion ack (Stop::id) rides
// the planner's own TickResult when the robot actually comes to rest.
void RobotLoop::handleStop(const msg::CommandEnvelope& env) {
  const bool accepted = planner_.plannedStop(env.cmd.stop.id);
  tlm_.ack(env.corr_id,
           accepted ? 0 : static_cast<uint32_t>(msg::ErrCode::ERR_FULL));
}

// ESTOP: halt now, both subsystems, no completion acks for what was
// discarded.
void RobotLoop::handleEstop(const msg::CommandEnvelope& env) {
  drive_.estop();
  planner_.estop();
  // Zero the blackboard's own commanded speeds too. Both subsystems'
  // update() calls would land the same zeros at the end of this cycle
  // anyway, but an emergency stop should not depend on the rest of the
  // schedule running to completion to take effect.
  state_.wheelLeft.cmdVelocity = 0.0f;
  state_.wheelRight.cmdVelocity = 0.0f;
  tlm_.ack(env.corr_id, 0);
}

// Boot-time commands are NACKed (ERR_NOT_CONFIGURED), never silently dropped.
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

// Boot: probe devices until the preamble is done, emitting status frames
// and NACKing any command that arrives early.
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
    // NOT forced. Forcing here pushed one frame per probe pass -- a ~5s
    // flood on every reset, ahead of the DEVICE banner and READY.
    //
    // Under the three-mode emit predicate (mode_ defaults kAuto,
    // everMoved_ is still false, no motion is possible before the
    // preamble even finishes) this emits nothing on its own -- a silent
    // host sees zero binary bytes during boot. A host that DOES transmit
    // during boot still gets served: a bare TLM request forces one frame
    // (reason 1), and any early command's NACK rides the ack ring (reason
    // 3, rejectDuringBoot() above) -- both honored in every mode.
    tlm_.emit(bootState.time.cycleStart);

    sleeper_.sleepMillis(kPreamblePace);  // paces probes AND yields (radio RX)
  }

  comms_.sendBanner();
  // READY last, and only here: past this point the loop dispatches commands
  // instead of NACKing them via rejectDuringBoot(). A host that waits for
  // this line never loses its first Move. Emitted after the banner so a
  // fresh listener gets identity first, then permission.
  comms_.sendReady();
  // state_.health.ready: the one genuinely loop-owned fact
  // Comms::updateStatus()'s STATUS projection answers -- "we are past
  // boot()" -- published onto the blackboard here, exactly once, rather
  // than hard-coded `true` at the projection call site (robot_state.h's
  // own doc comment on Health::ready).
  state_.health.ready = true;
}

// --- cycle() steps ---

void RobotLoop::publishWheel(Devices::Motor& motor,
                             Types::RobotState::Wheel& wheel,
                             uint8_t& positionEpoch, bool& clamped) {
  // Software rebaseline keeps position inside the wire bound; the clamp
  // is the loud defensive fallback.
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

// Deliberately does NOT touch wheel.cmdVelocity: that field has exactly one
// writer per cycle -- whichever of Motion::Planner::update()/App::Drive::
// update() currently owns motion, both of which run later in the pace
// block. Publishing it from here as well would overwrite the owner's
// staged command with the value being actuated THIS cycle, one generation
// stale.
void RobotLoop::publishWheels() {
  bool clampedL = false;
  bool clampedR = false;
  publishWheel(motorL_, state_.wheelLeft, positionEpochLeft_, clampedL);
  publishWheel(motorR_, state_.wheelRight, positionEpochRight_, clampedR);
  state_.health.wedgeLatch = motorL_.wedged() || motorR_.wedged();
  // Per-wheel, GATED stall visibility -- wedgeSuspect(), not
  // wedged()/wedgeLatch above. wedgeSuspect() requires |appliedDuty()|
  // above the motion-threshold deadband as well as the stuck-position
  // read, so it stays false on an idle parked robot (unlike wedged(),
  // which latches on ANY stuck reading, including at rest) -- see
  // telemetry.h's kFlagFaultWheelFrozenLeft/Right doc comment for the
  // full rationale. Do not source these from wedged()/wedgeLatch.
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

// `tickedLine` says which leaf the loop just ticked; the untouched
// leaf's fresh flag stays false.
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

// Ack a wheel command that Drive expired this cycle (bookkeeping). Called
// straight after Drive::update() latches it, so it rides the NEXT frame --
// the same visibility contract publishMoveResult() gives the planner's own
// completions.
void RobotLoop::ackDriveCompletion() {
  uint32_t moveId = 0;
  if (drive_.takeCompletion(&moveId)) tlm_.ack(moveId, 0);
}

void RobotLoop::publishHealth() {
  // "Motion in progress" is true when EITHER decider owns motion -- a live
  // WHEELS command is motion, and a host watching flags bit 2 must see it.
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

// Main cycle. Vendor windows: the brick needs kSettle between select and
// collect and kClear after a duty write; interposed brick traffic during a
// settle invalidates the pending sample, so the windows' bodies never
// touch the bus. The trailing block does touch the bus (OTOS + one of
// line/color), outside any select->collect window.
//
// comms_.pump() runs in ALL FOUR blocks -- it is pure CPU/buffer work, no
// bus traffic, so it is legal inside a settle window, and putting it there
// is what frees the transports several times per cycle instead of once.
// The four blocks stay exactly four: the schedule's own shape is asserted
// by src/tests/sim/system/sim_api_harness.cpp (exactly four sleepMillis()
// calls per cycle), so pump() is folded into the EXISTING bodies rather
// than given a block of its own.
//
// 131-005: the trailing block's own wait is an ABSOLUTE end-of-cycle
// deadline (runAndWaitUntil(state_.time.cycleStart, kCycle, ...), not a
// gap relative to its own entry mark -- see this call's own comment below
// and robot_loop.h's kCycle doc comment for the measured defect this
// fixes (a rock-stable 54ms delivered period against a 50ms nominal,
// traced to the three earlier blocks' own rounding/overrun being re-added
// on top of the fourth block's fixed budget instead of absorbed by it).
void RobotLoop::cycle() {
  state_.time.cycleStart = markTime();  // [ms] pace anchor + wire `now`
  const uint64_t cycleStartUs = clock_.nowMicros();  // [us]

  // Actuate the speeds the owning subsystem staged onto the blackboard last
  // cycle -- one actuation path regardless of which decider produced them
  // (one cycle of command-to-wheels latency, the same the planner's own
  // actuationDelay compensates for).
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
    // Drain the WHOLE ring, once per cycle. Every command ingested since
    // the last drain is routed here, in arrival order.
    Cmd cmd;
    while (comms_.takeCommand(cmd)) routeCommand(cmd);
  });
  motorR_.tick(clock_.nowMicros());  // collect R

  publishWheels();  // at the point of same-generation L/R coherence

  // 131-005: targets state_.time.cycleStart + kCycle directly (an absolute
  // deadline), not a fresh gap from this block's own entry markTime() --
  // whatever the three settle/clear blocks above (plus their own bodies'
  // real work) already consumed this cycle is exactly what this wait
  // shrinks by, so a jittery/overrunning cycle is absorbed HERE instead of
  // compounding into next cycle's own start mark (which is always taken
  // fresh, via markTime(), at the top of THIS function -- so a cycle's own
  // final-block rounding never carries forward into the next cycle).
  runAndWaitUntil(state_.time.cycleStart, kCycle, [&] {
    comms_.pump(state_.time.cycleStart);

    uint64_t nowUs = clock_.nowMicros();

    // 131-004: positionEpoch already reflects THIS cycle's publishWheels()
    // (which ran before this trailing block), so a same-cycle rebaseline's
    // epoch bump is visible here -- odom_ re-anchors instead of
    // differencing across the rebaseline jump (Motion::Odometry::
    // integrate()'s own doc comment, odometry.h).
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

    // TLM: command surface. The mode-change switch and the "is this a
    // force-a-frame request" answer live in Telemetry::applyAction() --
    // see that method's own doc comment for the dependency-direction
    // choice. Comms parses a `TLM`/`TLM:...` line's argument and stages
    // the result; Telemetry never parses wire text, so RobotLoop is the
    // one that hands Comms's staged action across.
#ifdef ROBOT_DEBUG
    // DBG fault-injection surface (system test) -- drained once per cycle,
    // same stage/consume split as the TLM action below.
    applyDbgAction(state_.time.cycleStart);
#endif

    const Comms::TlmAction tlmAction = comms_.takeTlmAction();
    const bool forceFrame = tlm_.applyAction(tlmAction);

    // A bare `TLM`/`TLM:NOW` line is a request for one frame NOW -- force
    // past the mode-gated unsolicited check, since "nothing is happening"
    // is exactly the state someone asking is trying to observe. Still
    // subject to the cadence gate, so a request storm cannot outrun the
    // wire.
    tlm_.emit(state_.time.cycleStart, forceFrame);

    // Refresh what STATUS answers from (Comms::updateStatus() -- see its
    // own doc comment for the full field-by-field contract). Sourced from
    // the SAME state_ the telemetry projection just used, so
    // the queryable line and the wire frame can never disagree -- STATUS is
    // a second VIEW of the state, not a second copy of it. Cheap enough to
    // run unconditionally; it must run even when the idle gate suppressed
    // the frame above, since answering STATUS on a parked robot is
    // precisely the case it exists for. LOAD-BEARING ORDER: called AFTER
    // tlm_.applyAction() above, so a same-cycle mode change is already
    // reflected in tlm_.mode() -- sendTlmReply() below relies on that.
    comms_.updateStatus(state_, tlm_);

    // TLM: mode-change / garbage reply -- the STATUS line for a
    // recognized ON/AUTO/OFF (now reporting the mode applied above), or the
    // HELP line for an unrecognized `TLM:<data>` argument. No-op for
    // kNone/kFrame -- see Comms::sendTlmReply()'s own doc comment.
    // LOAD-BEARING ORDER: called AFTER comms_.updateStatus() above, so its
    // STATUS reply reads Comms's own status_ snapshot already carrying the
    // NEW mode.
    comms_.sendTlmReply(tlmAction);

    // Both deciders tick AFTER emit: their completion acks ride the NEXT
    // frame. Drive::update() runs LAST so that, while Drive owns motion,
    // its targets are the ones left on the blackboard -- the planner's
    // update() is unconditional and would otherwise overwrite them with a
    // drained-to-zero command.
    const Motion::TickResult moveResult = planner_.tick(state_);
    planner_.update(state_);
    drive_.update(state_, state_.time.cycleStart);
    ackDriveCompletion();
    publishMoveResult(moveResult);
  });
}


#ifdef ROBOT_DEBUG
void RobotLoop::applyDbgAction(uint32_t now) {
  // Expire duration-bounded injections FIRST, so a wedge armed for N ms
  // clears on time even if no further DBG traffic ever arrives.
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
      // Echo through the robot's own output stream so the marker lands in
      // the dataset with correct ordering relative to telemetry.
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
    case Comms::DbgActionKind::kClear:
      motorL_.setForcedWedge(false);
      motorR_.setForcedWedge(false);
      dbgWedgeUntilL_ = 0;
      dbgWedgeUntilR_ = 0;
      App::debugf("clear");
      break;
    case Comms::DbgActionKind::kUnrecognized:
      App::debugf("unrecognized dbg: %s", action.text);
      break;
  }
  }
}
#endif  // ROBOT_DEBUG

}  // namespace App
