// robot_loop.cpp -- App::RobotLoop implementation. See robot_loop.h and
// DESIGN.md for the schedule rationale.
#include "app/robot_loop.h"

#include "motion/move_queue.h"  // Motion::ShaperLimits only

#include <cmath>

#include "motion/body_kinematics.h"
#include "messages/envelope.h"

namespace App {

namespace {

constexpr uint32_t kSettle = 4;  // [ms] encoder-settle window, both motors
constexpr uint32_t kClear = 4;   // [ms] post-duty-write clearance window

// The four pacing blocks must sum to kCycle; kPace absorbs the three
// settle/clear windows rather than stacking a fresh kCycle on top.
constexpr uint32_t kWindows = 2 * kSettle + kClear;  // [ms]
static_assert(kWindows <= RobotLoop::kCycle,
              "kSettle+kClear+kSettle must fit inside the kCycle budget");
constexpr uint32_t kPace = RobotLoop::kCycle - kWindows;  // [ms]

constexpr uint32_t kPreamblePace = 10;  // [ms] boot-loop probe pacing



// kPositionWireBound must match telemetry.proto EncoderReading.position abs_max.
constexpr float kPositionWireBound = 32000.0f;       // [mm]
constexpr float kPositionRebaselineMargin = 30000.0f;  // [mm]

// Present-field merges for persisted tuning. Gains mirror onto both sides;
// travel_calib is side-selected and merged by handleConfig() itself.
void mergeMotorGainsPatch(msg::MotorConfigPatch& slot, const msg::MotorConfigPatch& incoming) {
  if (incoming.kp.has) slot.kp = incoming.kp;
  if (incoming.ki.has) slot.ki = incoming.ki;
  if (incoming.kff.has) slot.kff = incoming.kff;
  if (incoming.i_max.has) slot.i_max = incoming.i_max;
  if (incoming.kaw.has) slot.kaw = incoming.kaw;
}

// `init` is a one-shot trigger, never persisted.
void mergeOtosPatch(msg::OtosConfigPatch& slot, const msg::OtosConfigPatch& incoming) {
  if (incoming.linear_scale.has) slot.linear_scale = incoming.linear_scale;
  if (incoming.angular_scale.has) slot.angular_scale = incoming.angular_scale;
  if (incoming.offset_x.has) slot.offset_x = incoming.offset_x;
  if (incoming.offset_y.has) slot.offset_y = incoming.offset_y;
  if (incoming.offset_yaw.has) slot.offset_yaw = incoming.offset_yaw;
}

// Estimator patches are never persisted; reboot reverts to baked defaults.
void mergeEstimatorPatch(Motion::FusionWeights& weights, const msg::EstimatorConfigPatch& patch) {
  if (patch.weight_heading_otos.has) weights.headingOtos = patch.weight_heading_otos.val;
  if (patch.weight_omega_otos.has) weights.omegaOtos = patch.weight_omega_otos.val;
  if (patch.staleness_ms.has) weights.staleness = static_cast<uint32_t>(patch.staleness_ms.val);
}

void mergeShaperPatch(Motion::ShaperLimits& limits, const msg::EstimatorConfigPatch& patch) {
  if (patch.a_max.has) limits.aMax = patch.a_max.val;
  if (patch.a_decel.has) limits.aDecel = patch.a_decel.val;
  if (patch.alpha_max.has) limits.alphaMax = patch.alpha_max.val;
  if (patch.alpha_decel.has) limits.alphaDecel = patch.alpha_decel.val;
  if (patch.j_max.has) limits.jMax = patch.j_max.val;
  if (patch.yaw_jerk_max.has) limits.yawJerkMax = patch.yaw_jerk_max.val;
}

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
                      Motion::Odometry& odom, Motion::Planner& planner, Preamble& preamble,
                      Motion::StateEstimator& stateEstimator, const Devices::Clock& clock,
                      Devices::Sleeper& sleeper, Config::TuningStore* tuningStore)
    : bus_(bus),
      motorL_(motorL),
      motorR_(motorR),
      otos_(otos),
      color_(color),
      line_(line),
      comms_(comms),
      tlm_(tlm),
      drive_(drive),
      odom_(odom),
      planner_(planner),
      preamble_(preamble),
      stateEstimator_(stateEstimator),
      clock_(clock),
      sleeper_(sleeper),
      tuningStore_(tuningStore) {}

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

float RobotLoop::clampToPositionWireBound(float pos, bool* clamped) {
  *clamped = std::fabs(pos) > kPositionWireBound;
  return *clamped ? std::copysign(kPositionWireBound, pos) : pos;
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
  // A wheel-speeds command never touches the planner: the speeds go
  // straight into the robot state and the cycle drives the wheels from
  // there. TIME stop (or the timeout backstop) bounds it; completion is
  // acked at expiry with the Move's own id.
  if (move.velocity_kind == msg::Move::VelocityKind::WHEELS) {
    stopAll();  // the wheel command supersedes whatever was running
    const float bound =
        move.stop_kind == msg::Move::StopKind::TIME ? move.stop.time
                                                     : move.timeout;
    drive_.command(move.velocity.wheels.v_left, move.velocity.wheels.v_right,
                   bound, move.id, state_.time.cycleStart);
    tlm_.ack(env.corr_id, 0);
    return;
  }

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
  drive_.stop();  // a planned move takes over the targets
  const bool accepted = planner_.move(m, move.replace);
  tlm_.ack(env.corr_id,
           accepted ? 0 : static_cast<uint32_t>(msg::ErrCode::ERR_FULL));
}

void RobotLoop::handleConfig(const msg::CommandEnvelope& env) {
  if (env.cmd.config.patch_kind == msg::ConfigDelta::PatchKind::OTOS) {
    const msg::OtosConfigPatch& patch = env.cmd.config.patch.otos;

    applyOtosPatch(patch);
    mergeOtosPatch(persistedTuning_.otos, patch);
    persistTuningIfChanged();

    tlm_.ack(env.corr_id, 0);
    return;
  }

  if (env.cmd.config.patch_kind == msg::ConfigDelta::PatchKind::ESTIMATOR) {
    const msg::EstimatorConfigPatch& patch = env.cmd.config.patch.estimator;

    Motion::FusionWeights weights = stateEstimator_.weights();
    mergeEstimatorPatch(weights, patch);
    stateEstimator_.setWeights(weights);

    // Shaper wire keys retarget the planner's live limits.
    Motion::ShaperLimits shaperLimits{};
    shaperLimits.aMax = planner_.limits().aMax;
    shaperLimits.aDecel = planner_.limits().aDecel;
    shaperLimits.alphaMax = planner_.limits().alphaMax;
    shaperLimits.alphaDecel = planner_.limits().alphaDecel;
    shaperLimits.jMax = planner_.limits().jerkMax;
    shaperLimits.yawJerkMax = planner_.limits().yawJerkMax;
    mergeShaperPatch(shaperLimits, patch);
    planner_.applyShaperLimits(shaperLimits.aMax, shaperLimits.aDecel,
                               shaperLimits.alphaMax, shaperLimits.alphaDecel,
                               shaperLimits.jMax, shaperLimits.yawJerkMax);

    tlm_.ack(env.corr_id, 0);
    return;
  }

  if (env.cmd.config.patch_kind != msg::ConfigDelta::PatchKind::MOTOR) {
    tlm_.ack(env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_UNIMPLEMENTED));
    return;
  }

  const msg::MotorConfigPatch& patch = env.cmd.config.patch.motor;

  // Gains mirror onto both sides; travel_calib only to the addressed side.
  mergeMotorGainsPatch(persistedTuning_.motorL, patch);
  mergeMotorGainsPatch(persistedTuning_.motorR, patch);
  if (patch.travel_calib.has) {
    msg::MotorConfigPatch& target = (patch.side == msg::BoundMotorSide::LEFT)
                                         ? persistedTuning_.motorL
                                         : persistedTuning_.motorR;
    target.travel_calib = patch.travel_calib;
  }
  persistedTuning_.motorL.side = msg::BoundMotorSide::LEFT;
  persistedTuning_.motorR.side = msg::BoundMotorSide::RIGHT;

  applyMotorConfigPatch(persistedTuning_.motorL);
  applyMotorConfigPatch(persistedTuning_.motorR);
  persistTuningIfChanged();

  tlm_.ack(env.corr_id, 0);
}

void RobotLoop::applyMotorConfigPatch(const msg::MotorConfigPatch& patch) {
  // kff is the open-loop duty-per-speed scale (the same wire key the old
  // velocity PID's feedforward used, so sim/bench configs keep working);
  // the wire patch sets both wheels (per-wheel split is boot calibration).
  if (patch.kff.has) drive_.setDutyPerSpeed(patch.kff.val, patch.kff.val);

  // pid.* wire keys still retarget the (dormant) planner's gains.
  float kff = planner_.limits().velKff;
  float kp = planner_.limits().velKp;
  float ki = planner_.limits().velKi;
  float iMax = planner_.limits().velIMax;
  if (patch.kp.has) kp = patch.kp.val;
  if (patch.ki.has) ki = patch.ki.val;
  if (patch.kff.has) kff = patch.kff.val;
  if (patch.i_max.has) iMax = patch.i_max.val;
  planner_.applyVelGains(kff, kp, ki, iMax);

  if (patch.travel_calib.has) {
    if (patch.side == msg::BoundMotorSide::LEFT) {
      motorL_.applyTravelCalib(patch.travel_calib.val);
    } else {
      motorR_.applyTravelCalib(patch.travel_calib.val);
    }
  }
}

void RobotLoop::applyOtosPatch(const msg::OtosConfigPatch& patch) {
  if (patch.linear_scale.has) otos_.setLinearScalar(patch.linear_scale.val);
  if (patch.angular_scale.has) otos_.setAngularScalar(patch.angular_scale.val);

  // setOffset writes x/y/heading together: read-merge-write so absent
  // fields keep the chip's current values.
  if (patch.offset_x.has || patch.offset_y.has || patch.offset_yaw.has) {
    float x = 0.0f, y = 0.0f, heading = 0.0f;
    otos_.getOffset(x, y, heading);
    if (patch.offset_x.has) x = patch.offset_x.val;
    if (patch.offset_y.has) y = patch.offset_y.val;
    if (patch.offset_yaw.has) heading = patch.offset_yaw.val;
    otos_.setOffset(x, y, heading);
  }

  if (patch.init) otos_.init();
}

// Change-detection debounce: only write flash when the serialized snapshot
// actually differs from the last one written.
void RobotLoop::persistTuningIfChanged() {
  if (tuningStore_ == nullptr) return;

  Config::Blob blob = Config::serializeSnapshot(persistedTuning_);
  if (blob == lastPersistedBlob_) return;

  tuningStore_->save(Config::kConfigSchemaVersion, blob);
  lastPersistedBlob_ = blob;
}

void RobotLoop::reapplyPersistedTuning(const Config::TuningSnapshot& snapshot) {
  applyMotorConfigPatch(snapshot.motorL);
  applyMotorConfigPatch(snapshot.motorR);
  applyOtosPatch(snapshot.otos);

  persistedTuning_ = snapshot;
  lastPersistedBlob_ = Config::serializeSnapshot(persistedTuning_);
}

// Stop everything that can command the wheels (shared by STOP and the
// takeover paths -- no near-duplicates).
void RobotLoop::stopAll() {
  drive_.stop();
  planner_.stop();
}

void RobotLoop::handleStop(const msg::CommandEnvelope& env) {
  stopAll();
  tlm_.ack(env.corr_id, 0);
}

// Boot-time commands are NACKed (ERR_NOT_CONFIGURED), never silently dropped.
void RobotLoop::rejectDuringBoot(const Cmd& cmd) {
  if (cmd.status != CmdStatus::kDecoded) return;
  tlm_.ack(cmd.env.corr_id, static_cast<uint32_t>(msg::ErrCode::ERR_NOT_CONFIGURED));
}

void RobotLoop::processMessage(const Cmd& cmd) {
  msg::CommandEnvelope::CmdKind kind = (cmd.status == CmdStatus::kDecoded)
      ? cmd.env.cmd_kind
      : msg::CommandEnvelope::CmdKind::NONE;
      
  switch (kind) {
    case msg::CommandEnvelope::CmdKind::MOVE:
      handleMove(cmd.env);
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

// Boot: probe devices until the preamble is done, emitting status frames
// and NACKing any command that arrives early.
void RobotLoop::boot() {
  while (!preamble_.done()) {
    Cmd bootCmd;
    comms_.pump(bootCmd, markTime());
    rejectDuringBoot(bootCmd);

    preamble_.step();  // one bounded probe action per pass

    Types::RobotState bootState;
    bootState.time.cycleStart = markTime();
    bootState.wheelLeft.connected = preamble_.leftConnected();
    bootState.wheelRight.connected = preamble_.rightConnected();
    bootState.otos.connected = preamble_.otosConnected();
    tlm_.update(bootState);
    tlm_.emit(bootState.time.cycleStart);

    sleeper_.sleepMillis(kPreamblePace);  // paces probes AND yields (radio RX)
  }
  tlm_.setLiveFlag(kFlagEventBootReady, true);

  comms_.sendBanner();
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

void RobotLoop::publishWheels() {
  bool clampedL = false;
  bool clampedR = false;
  publishWheel(motorL_, state_.wheelLeft, positionEpochLeft_, clampedL);
  publishWheel(motorR_, state_.wheelRight, positionEpochRight_, clampedR);
  state_.wheelLeft.cmdVelocity = drive_.targetLeft();
  state_.wheelRight.cmdVelocity = drive_.targetRight();
  state_.health.wedgeLatch = motorL_.wedged() || motorR_.wedged();
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

// Ack a wheel command that Drive expired this tick (bookkeeping).
void RobotLoop::ackDriveCompletion() {
  uint32_t moveId = 0;
  if (drive_.takeCompletion(&moveId)) tlm_.ack(moveId, 0);
}

void RobotLoop::publishHealth() {
  state_.command.mode = planner_.active() ? Types::Mode::Velocity : Types::Mode::Idle;
  state_.command.moveActive = planner_.active();
  state_.health.i2cSafetyNetCount = bus_.clearanceSafetyNetCount();
  state_.health.commsMalformedCount = comms_.malformedCount();
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
// touch the bus. The trailing kPace block does touch the bus (OTOS + one
// of line/color), outside any select->collect window.
void RobotLoop::cycle() {
  state_.time.cycleStart = markTime();  // [ms] pace anchor + wire `now`
  const uint64_t cycleStartUs = clock_.nowMicros();  // [us]

  Cmd cmd;

  // The planner (when it owns motion) published its wheel targets into
  // the state last cycle; hand them to Drive. A live wheel command owns
  // the targets instead -- Drive ignores this while its own command runs.
  drive_.setPlannerTargets(state_.wheelLeft.cmdVelocity,
                           state_.wheelRight.cmdVelocity,
                           planner_.active());
  drive_.tick(state_.time.cycleStart);
  ackDriveCompletion();

  motorL_.requestSample();  // brick latches ONE pending read per select
  runAndWait(kSettle, [&] { 
    comms_.pump(cmd, state_.time.cycleStart); 
  });
  motorL_.tick(clock_.nowMicros());  // collect L

  runAndWait(kClear, [&] {});  // brick's post-duty-write clearance

  motorR_.requestSample();
  runAndWait(kSettle, [&] { 
    processMessage(cmd); 
  });
  motorR_.tick(clock_.nowMicros());  // collect R

  publishWheels();  // at the point of same-generation L/R coherence

  runAndWait(kPace, [&] {
    uint64_t nowUs = clock_.nowMicros();

    odom_.integrate(motorL_.position(), motorR_.position());  // before OTOS: FakeOtos reads it
    otos_.tick(nowUs);
    publishOtos();
    const bool tickedLine = (cycleCount_ % 2) == 1;  // first cycle ticks line
    if (tickedLine) line_.tick(nowUs); else color_.tick(nowUs);
    publishLineColor(tickedLine);
    publishPose();
    publishHealth();
    stateEstimator_.update(state_, static_cast<uint32_t>(nowUs / 1000));  // [us] -> [ms]
    publishTiming(cycleStartUs);

    tlm_.update(state_);
    tlm_.emit(state_.time.cycleStart);

    // Planner tick AFTER emit: its completion ack rides the NEXT frame.
    const Motion::TickResult moveResult = planner_.tick(state_);
    planner_.update(state_);
    publishMoveResult(moveResult);
  });
}

}  // namespace App
