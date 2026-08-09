#include "core/telemetry.h"

#include "core/differential_drive.h"

namespace Core {

static_assert(sizeof(msg::Telemetry{}.acks_) == static_cast<size_t>(kAckRingDepth) * sizeof(uint32_t),
              "Core::kAckRingDepth (telemetry.h) must match telemetry.proto's Telemetry.acks (max_count)");

constexpr uint32_t kAckErrBits = 4;
constexpr uint32_t kAckErrMask = (1u << kAckErrBits) - 1;

constexpr uint32_t kMaxAge = 255;  // [ms]

Telemetry::Telemetry(Comms& comms) : comms_(comms) {}

uint32_t Telemetry::ageOf(uint32_t now, uint32_t sampleTime) {
  if (now < sampleTime) return 0;
  const uint32_t age = now - sampleTime;
  return age > kMaxAge ? kMaxAge : age;
}

void Telemetry::setFlag(uint32_t bit, bool active) {
  if (active) {
    flags_ |= bit;
  } else {
    flags_ &= ~bit;
  }
}

void Telemetry::setLiveFlag(uint32_t bit, bool active) {
  setFlag(bit, active);
}

void Telemetry::update(const Types::RobotState& state, const DifferentialDrive& drive) {
  const uint32_t now = state.time.cycleStart + (state.time.cycleBusy / 1000u);  // [ms] ([us]->[ms])

  frame_.mode = static_cast<msg::DriveMode>(state.command.mode);

  frame_.encLeft.position = msg::EncoderReading::packPosition(state.wheelLeft.position);
  frame_.encLeft.velocity = msg::EncoderReading::packVelocity(state.wheelLeft.velocity);
  frame_.encLeft.age = ageOf(now, state.wheelLeft.sampleTime);
  frame_.encLeft.position_epoch = state.wheelLeft.positionEpoch;

  frame_.encRight.position = msg::EncoderReading::packPosition(state.wheelRight.position);
  frame_.encRight.velocity = msg::EncoderReading::packVelocity(state.wheelRight.velocity);
  frame_.encRight.age = ageOf(now, state.wheelRight.sampleTime);
  frame_.encRight.position_epoch = state.wheelRight.positionEpoch;

  frame_.twist.v_x = msg::BodyTwist3::packVX(state.pose.v_x);
  frame_.twist.omega = msg::BodyTwist3::packOmega(state.pose.omega);

  frame_.pose = {msg::Pose2D::packX(state.pose.x), msg::Pose2D::packY(state.pose.y),
                 msg::Pose2D::packH(state.pose.heading)};

  if (state.otos.present) {
    frame_.otos.x = msg::OtosReading::packX(state.otos.x);
    frame_.otos.y = msg::OtosReading::packY(state.otos.y);
    frame_.otos.heading = msg::OtosReading::packHeading(state.otos.heading);
    frame_.otos.v_x = msg::OtosReading::packVX(state.otos.v_x);
    frame_.otos.v_y = msg::OtosReading::packVY(state.otos.v_y);
    frame_.otos.omega = msg::OtosReading::packOmega(state.otos.omega);
    frame_.otos.age = ageOf(now, state.otos.sampleTime);
  }

  if (state.perception.lineFresh) frame_.line = state.perception.line;
  if (state.perception.colorFresh) frame_.color = state.perception.color;
  // frame_ keeps the last value for the leaf that was NOT sampled this
  // cycle; the Present/Fresh flag pair below is what tells them apart.

  frame_.cycleBusy = state.time.cycleBusy;
  frame_.cyclePeriod = state.time.cyclePeriod;

  frame_.dutyPerSpeedLeft = drive.dutyPerSpeedLeft();
  frame_.dutyPerSpeedRight = drive.dutyPerSpeedRight();
  frame_.biasLeft = drive.biasLeft();
  frame_.biasRight = drive.biasRight();
  frame_.pidLeft = drive.pidLeft();
  frame_.pidRight = drive.pidRight();

  setFlag(kFlagActive, state.command.moveActive);
  setFlag(kFlagConnLeft, state.wheelLeft.connected);
  setFlag(kFlagConnRight, state.wheelRight.connected);
  setFlag(kFlagFaultI2CSafetyNet, state.health.i2cSafetyNetCount > 0);
  setFlag(kFlagFaultWedgeLatch, state.health.wedgeLatch);
  setFlag(kFlagFaultCommsMalformed, state.health.commsMalformedCount > 0);
  setFlag(kFlagFaultCommandsDropped, state.health.commandsDroppedCount > 0);
  setFlag(kFlagOtosPresent, state.otos.present);
  setFlag(kFlagOtosConnected, state.otos.connected);
  setFlag(kFlagLinePresent, state.perception.lineValid);
  setFlag(kFlagColorPresent, state.perception.colorValid);
  setFlag(kFlagLineFresh, state.perception.lineFresh);
  setFlag(kFlagColorFresh, state.perception.colorFresh);
  setFlag(kFlagFaultPositionClamped, state.health.positionClamped);
  setFlag(kFlagFaultWheelFrozenLeft, state.health.wheelFrozenLeft);
  setFlag(kFlagFaultWheelFrozenRight, state.health.wheelFrozenRight);
  setFlag(kFlagFaultWheelDeficitLeft, drive.deficitLeft());
  setFlag(kFlagFaultWheelDeficitRight, drive.deficitRight());
  setFlag(kFlagFaultStallLeft, state.health.stallLeft);
  setFlag(kFlagFaultStallRight, state.health.stallRight);

  if (flags_ & kFlagActive) {
    everMoved_ = true;
    lastActivity_ = state.time.cycleStart;
  } else {
    const bool windowOpen =
        everMoved_ && (state.time.cycleStart - lastActivity_) < kCoastHoldoff;
    const bool wheelsMoving = frame_.encLeft.velocity != 0 || frame_.encRight.velocity != 0;
    if (windowOpen && wheelsMoving) lastActivity_ = state.time.cycleStart;
  }
}

bool Telemetry::applyAction(Comms::TlmAction action) {
  switch (action) {
    case Comms::TlmAction::kSetOff:
      setMode(TlmMode::kOff);
      break;
    case Comms::TlmAction::kSetAuto:
      setMode(TlmMode::kAuto);
      break;
    case Comms::TlmAction::kSetOn:
      setMode(TlmMode::kOn);
      break;
    case Comms::TlmAction::kNone:
    case Comms::TlmAction::kFrame:
    case Comms::TlmAction::kUnrecognized:
    default:
      break;
  }
  return action == Comms::TlmAction::kFrame;
}

void Telemetry::ack(uint32_t corrId, uint32_t errCode) {
  pushAckRing(corrId, errCode);
}

void Telemetry::pushAckRing(uint32_t corrId, uint32_t errCode) {
  uint8_t tail;
  if (ackRingCount_ < kAckRingDepth) {
    tail = static_cast<uint8_t>((ackRingHead_ + ackRingCount_) % kAckRingDepth);
    ++ackRingCount_;
  } else {
    tail = ackRingHead_;
    ackRingHead_ = static_cast<uint8_t>((ackRingHead_ + 1) % kAckRingDepth);
  }
  ackRing_[tail] = (corrId << kAckErrBits) | (errCode & kAckErrMask);
  ackSends_[tail] = 0;
}

bool Telemetry::primaryDue(uint32_t now) const {
  if (!everEmittedPrimary_) return true;
  return (now - lastPrimaryEmit_) >= kPrimaryPeriod;
}

bool Telemetry::pendingAckDeliveries() const {
  for (uint8_t i = 0; i < ackRingCount_; ++i) {
    const uint8_t idx = static_cast<uint8_t>((ackRingHead_ + i) % kAckRingDepth);
    if (ackSends_[idx] < kAckRepeats) return true;
  }
  return false;
}

void Telemetry::emit(uint32_t now, bool force) {
  const bool activity =
      (flags_ & kFlagActive) || (everMoved_ && (now - lastActivity_) < kCoastHoldoff);
  bool unsolicited = false;
  switch (mode_) {
    case TlmMode::kOff:
      unsolicited = false;
      break;
    case TlmMode::kAuto:
      unsolicited = activity;
      break;
    case TlmMode::kOn:
      unsolicited = true;
      break;
  }
  if (primaryDue(now) && (force || unsolicited || pendingAckDeliveries())) {
    emitPrimary(now);
  }
}

void Telemetry::emitPrimary(uint32_t now) {
  msg::Telemetry tlm;

  tlm.now = now;
  tlm.seq = seq_;
  seq_ = (seq_ + 1) % 128u;
  tlm.mode = frame_.mode;

  tlm.flags = flags_;

  tlm.acks_count = ackRingCount_;
  for (uint8_t i = 0; i < ackRingCount_; ++i) {
    const uint8_t idx = static_cast<uint8_t>((ackRingHead_ + i) % kAckRingDepth);
    tlm.acks_[i] = ackRing_[idx];
    if (ackSends_[idx] < kAckRepeats) ++ackSends_[idx];
  }

  tlm.enc_left = frame_.encLeft;
  tlm.enc_right = frame_.encRight;
  tlm.otos = frame_.otos;
  tlm.pose = frame_.pose;
  tlm.twist = frame_.twist;
  tlm.line = frame_.line;
  tlm.color = frame_.color;

  tlm.cycle_busy = frame_.cycleBusy;
  tlm.cycle_period = frame_.cyclePeriod;

  tlm.duty_per_speed_left = frame_.dutyPerSpeedLeft;
  tlm.duty_per_speed_right = frame_.dutyPerSpeedRight;
  tlm.bias_left = frame_.biasLeft;
  tlm.bias_right = frame_.biasRight;
  tlm.pid_left = frame_.pidLeft;
  tlm.pid_right = frame_.pidRight;

  msg::ReplyEnvelope env;
  env.corr_id = 0;
  env.body_kind = msg::ReplyEnvelope::BodyKind::TLM;
  env.body.tlm = tlm;

  comms_.sendReply(env);

  everEmittedPrimary_ = true;
  lastPrimaryEmit_ = now;
  ++primaryEmitCount_;
}

}
