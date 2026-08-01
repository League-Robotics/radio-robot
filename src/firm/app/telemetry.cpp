// telemetry.cpp -- App::Telemetry implementation. See telemetry.h's file
// header for the module's boundary, its send path, and the flags-bit
// layout.
#include "app/telemetry.h"

namespace App {

// kAckRingDepth (telemetry.h) must match the generated msg::Telemetry::
// acks_[] array width (telemetry.proto's Telemetry.acks max_count) exactly
// -- a mismatch here would silently truncate (kAckRingDepth too big) or
// under-fill (too small) the wire array in emitPrimary() below. sizeof on
// a temporary's member is a valid, never-evaluated sizeof operand (the
// object is never constructed at runtime) -- the same "prove a size fact
// at compile time with no runtime cost" idiom this file's own generated
// siblings (layout_checks.h) already use for standard-layout checks.
//
// acks_[] is uint32_t[4] (packed corr_id<<4|err).
static_assert(sizeof(msg::Telemetry{}.acks_) == static_cast<size_t>(kAckRingDepth) * sizeof(uint32_t),
              "App::kAckRingDepth (telemetry.h) must match telemetry.proto's Telemetry.acks (max_count)");

// Packed ack-ring word format: corr_id in the upper 28 bits, err (an
// ErrCode, 0-8) in the low 4 bits. err's real range fits comfortably in 4
// bits (ERR_NOT_CONFIGURED=8 is the enum's own ceiling); corr_id needs 16
// bits in practice (CommandEnvelope.corr_id/Move.id) and has 28 available.
constexpr uint32_t kAckErrBits = 4;
constexpr uint32_t kAckErrMask = (1u << kAckErrBits) - 1;

// kMaxAge -- EncoderReading.age/OtosReading.age's own wire (max) = 255
// (telemetry.proto): a pathological value obvious rather than silently
// wrapping.
constexpr uint32_t kMaxAge = 255;  // [ms]

Telemetry::Telemetry(Comms& comms) : comms_(comms) {}

uint32_t Telemetry::ageOf(uint32_t now, uint32_t sampleTime) {
  // Unsigned-subtract guard: sampleTime should never be ahead of `now` (both
  // are in the same cycle-domain [ms] clock, and a sample is always
  // collected before the cycle that reports it finishes), but a defensive
  // guard costs nothing and avoids a huge wrapped age on any clock-domain
  // surprise (e.g. state.time.cycleStart genuinely 0 before the first-ever
  // publish).
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

void Telemetry::update(const Types::RobotState& state) {
  // now (for age ONLY -- NOT the wire `now` field, which stays
  // state.time.cycleStart via the caller's own emit(cycleStart) call,
  // unchanged from before this ticket): every sampleTime this method reads
  // below (wheelLeft/wheelRight/otos) was captured DURING this cycle's own
  // body, chronologically AFTER cycleStart (the top-of-cycle mark) -- an
  // age computed against cycleStart itself would be negative (sampleTime
  // in the "future" relative to it) and get silently floored to 0 by
  // ageOf()'s own now<sampleTime guard, defeating the entire point of a
  // real per-sample age. state.time.cycleBusy is measured from the SAME
  // cycleStartUs instant to the frame-staging point immediately before
  // this call (RobotLoop::cycle()'s own doc comment on that field) -- the
  // latest instant available on state, guaranteed at or after every
  // sampleTime this cycle published. cycleStart + cycleBusy therefore
  // approximates the genuine "now, at frame assembly" the age computation
  // needs, without adding a new RobotState field or changing what the wire
  // `now` field itself reports.
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

  // otos.* -- only refreshed when fresh THIS cycle (state.otos.present);
  // otherwise frame_.otos keeps its last-staged snapshot (frame_ persists
  // across update() calls, not a diff since the last call).
  if (state.otos.present) {
    frame_.otos.x = msg::OtosReading::packX(state.otos.x);
    frame_.otos.y = msg::OtosReading::packY(state.otos.y);
    frame_.otos.heading = msg::OtosReading::packHeading(state.otos.heading);
    frame_.otos.v_x = msg::OtosReading::packVX(state.otos.v_x);
    frame_.otos.v_y = msg::OtosReading::packVY(state.otos.v_y);
    frame_.otos.omega = msg::OtosReading::packOmega(state.otos.omega);
    frame_.otos.age = ageOf(now, state.otos.sampleTime);
  }

  // line/color -- exactly one of {line, color} ticks a given cycle
  // (RobotLoop's own kPace-block body alternates them); only the
  // fresh one's word is refreshed here, the other's stays at its
  // last-staged snapshot -- matching the wire spec's "fresh THIS frame"
  // semantics.
  if (state.perception.lineFresh) frame_.line = state.perception.line;
  if (state.perception.colorFresh) frame_.color = state.perception.color;

  frame_.cycleBusy = state.time.cycleBusy;
  frame_.cyclePeriod = state.time.cyclePeriod;

  // Flags -- the single assembly point: every bit that CAN be known at
  // this point in the cycle, derived straight from `state`.
  // kFlagFaultMoveTimeout/kFlagFaultShapingDisabled (bits 15/16) are
  // deliberately NOT touched here -- see setLiveFlag()'s own doc comment
  // (telemetry.h) for why their defining condition does not exist yet at
  // update() time, and why leaving them untouched here (rather than
  // re-deriving them from a state.health field that itself is only fresh
  // post-tick) is exactly what makes a flag "ride the next frame":
  // whatever setLiveFlag() last set survives across this update() call
  // unchanged, precisely because this method never mutates those two bits
  // itself.
  setFlag(kFlagActive, state.command.moveActive);
  setFlag(kFlagConnLeft, state.wheelLeft.connected);
  setFlag(kFlagConnRight, state.wheelRight.connected);
  setFlag(kFlagFaultI2CSafetyNet, state.health.i2cSafetyNetCount > 0);
  setFlag(kFlagFaultWedgeLatch, state.health.wedgeLatch);
  setFlag(kFlagFaultCommsMalformed, state.health.commsMalformedCount > 0);
  setFlag(kFlagFaultCommandsDropped, state.health.commandsDroppedCount > 0);
  setFlag(kFlagOtosPresent, state.otos.present);
  setFlag(kFlagOtosConnected, state.otos.connected);
  setFlag(kFlagLinePresent, state.perception.lineFresh);
  setFlag(kFlagColorPresent, state.perception.colorFresh);
  setFlag(kFlagFaultPositionClamped, state.health.positionClamped);
  setFlag(kFlagFaultWheelFrozenLeft, state.health.wheelFrozenLeft);
  setFlag(kFlagFaultWheelFrozenRight, state.health.wheelFrozenRight);

  // lastActivity_/everMoved_ -- computed against state.time.cycleStart,
  // NOT the `now` local above: that local is the age-computation instant
  // (cycleStart + cycleBusy); emit()'s own `now` parameter -- the clock domain
  // lastActivity_ is compared against -- is always the bare
  // state.time.cycleStart RobotLoop::cycle() passes it. Order matters:
  // windowOpen is evaluated against lastActivity_'s value BEFORE this
  // cycle's own refresh, so wheels alone can never open a window that was
  // already closed -- only kFlagActive can do that.
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

// applyAction -- see telemetry.h's own doc comment for the full contract
// (dependency-direction choice, why emit()'s `force` parameter survives).
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
      break;  // no mode change
  }
  return action == Comms::TlmAction::kFrame;
}

void Telemetry::ack(uint32_t corrId, uint32_t errCode) {
  pushAckRing(corrId, errCode);
}

void Telemetry::pushAckRing(uint32_t corrId, uint32_t errCode) {
  // Classic bounded circular buffer: the tail (next-write) slot is always
  // (ackRingHead_ + ackRingCount_) % kAckRingDepth. While there's spare
  // capacity, that slot is unused -- write it and grow the count. Once
  // full (count == depth), that SAME formula reduces to ackRingHead_
  // itself (the oldest entry's own slot) -- write the new value there,
  // evicting the oldest, then advance ackRingHead_ by one so the
  // just-written slot becomes the newest and the next slot over becomes
  // the new oldest.
  uint8_t tail;
  if (ackRingCount_ < kAckRingDepth) {
    tail = static_cast<uint8_t>((ackRingHead_ + ackRingCount_) % kAckRingDepth);
    ++ackRingCount_;
  } else {
    tail = ackRingHead_;
    ackRingHead_ = static_cast<uint8_t>((ackRingHead_ + 1) % kAckRingDepth);
  }
  // Packed word: corr_id<<4 | err.
  ackRing_[tail] = (corrId << kAckErrBits) | (errCode & kAckErrMask);
  // Undelivered: this entry now forces frames until it has been carried
  // kAckRepeats times (pendingAckDeliveries()). Reset explicitly rather than
  // relying on the slot's prior value -- a reused slot would otherwise
  // inherit the evicted entry's delivered count and never be sent at all.
  ackSends_[tail] = 0;
}

bool Telemetry::primaryDue(uint32_t now) const {
  if (!everEmittedPrimary_) return true;  // always on from boot -- no arming
  return (now - lastPrimaryEmit_) >= kPrimaryPeriod;
}

// pendingAckDeliveries -- true while any ack-ring entry has not yet been
// carried kAckRepeats times. Honored in EVERY mode (see emit() below),
// never gated: a host that commands the robot the instant READY lands
// must still get its ack, in kOff exactly like every other mode --
// protocol v5 has no separate ack message, so the telemetry frame is the
// ack's only vehicle, and suppressing it would strand the outcome of any
// command issued to a parked robot ("acked but nothing happened").
bool Telemetry::pendingAckDeliveries() const {
  for (uint8_t i = 0; i < ackRingCount_; ++i) {
    const uint8_t idx = static_cast<uint8_t>((ackRingHead_ + i) % kAckRingDepth);
    if (ackSends_[idx] < kAckRepeats) return true;
  }
  return false;
}

// emit -- the whole three-mode policy, and nothing more.
// `activity` mirrors update()'s own lastActivity_/everMoved_ derivation
// (kFlagActive live OR an already-open window kept alive by nonzero staged
// wheel velocity within kCoastHoldoff of the last refresh). `unsolicited`
// is what mode_ actually controls: never in kOff, the activity window in
// kAuto (the default), every cadence tick in kOn. The three reasons to
// emit -- force (a request), unsolicited (mode-dependent), or
// pendingAckDeliveries() (an ack, honored regardless of mode) -- are the
// ONLY three; there is no fourth (no flag-change push, no arming state, no
// boot-completion signal here).
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
  // seq wraps at telemetry.proto's own declared (max)=127 -- kept
  // genuinely small on the real value too, not just the wire bound, so
  // the two never drift: a caller reading the declared bound sees the
  // true worst case, not an optimistic one.
  tlm.seq = seq_;
  seq_ = (seq_ + 1) % 128u;
  tlm.mode = frame_.mode;

  // flags -- the caller-derived bits (update()'s own single assembly
  // point) plus setLiveFlag()'s two late bits, whatever this
  // call's flags_ currently holds.
  tlm.flags = flags_;

  // Ack ring -- serialize the ring's CURRENT contents, oldest-to-newest
  // (ackRingHead_'s own push/evict order), every emitPrimary() call,
  // exactly like every other Frame field: the last staged snapshot, not a
  // diff since the last send (app/DESIGN.md Sec 3). A frame this call
  // sends carries whatever the ring holds RIGHT NOW, regardless of
  // whether a new ack() landed since the last emit -- no separate "is
  // this new" bit applies (telemetry.proto's own Telemetry.acks doc
  // comment). Each entry is a packed uint32_t (corr_id<<4|err).
  tlm.acks_count = ackRingCount_;
  for (uint8_t i = 0; i < ackRingCount_; ++i) {
    const uint8_t idx = static_cast<uint8_t>((ackRingHead_ + i) % kAckRingDepth);
    tlm.acks_[i] = ackRing_[idx];
    // Count this delivery. Saturate rather than wrap: an entry that has been
    // carried kAckRepeats times must STAY delivered, or a long-lived ring
    // entry would silently start forcing frames again on overflow and the
    // link would never go quiet.
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

  msg::ReplyEnvelope env;
  env.corr_id = 0;  // unsolicited push -- envelope.proto's own convention
  env.body_kind = msg::ReplyEnvelope::BodyKind::TLM;
  env.body.tlm = tlm;

  comms_.sendReply(env);

  everEmittedPrimary_ = true;
  lastPrimaryEmit_ = now;
  ++primaryEmitCount_;
}

}  // namespace App
