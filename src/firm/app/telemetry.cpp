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
// 124-008: acks_[] is now uint32_t[4] (packed corr_id<<4|err), not
// msg::AckEntry[4] -- msg::AckEntry is deleted (issue §B4).
static_assert(sizeof(msg::Telemetry{}.acks_) == static_cast<size_t>(kAckRingDepth) * sizeof(uint32_t),
              "App::kAckRingDepth (telemetry.h) must match telemetry.proto's Telemetry.acks (max_count)");

// Packed ack-ring word format (124-008, issue §B4): corr_id in the upper
// 28 bits, err (an ErrCode, 0-8) in the low 4 bits. err's real range fits
// comfortably in 4 bits (ERR_NOT_CONFIGURED=8 is the enum's own ceiling);
// corr_id needs 16 bits in practice (CommandEnvelope.corr_id/Move.id) and
// has 28 available.
constexpr uint32_t kAckErrBits = 4;
constexpr uint32_t kAckErrMask = (1u << kAckErrBits) - 1;

// kMaxAge -- EncoderReading.age/OtosReading.age's own wire (max) = 255
// (telemetry.proto), issue §B2's own bound: "a pathological value obvious
// rather than silently wrapping."
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
  // otherwise frame_.otos keeps its last-staged snapshot, matching the
  // pre-124-009 assembleFrame()'s own "only update when present" behavior
  // (frame_ persists across update() calls exactly like it persisted
  // across assembleFrame() calls).
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
  // (115-005 alternation, RobotLoop's own kPace-block body); only the
  // fresh one's word is refreshed here, the other's stays at its
  // last-staged snapshot -- matching the wire spec's "fresh THIS frame"
  // semantics.
  if (state.perception.lineFresh) frame_.line = state.perception.line;
  if (state.perception.colorFresh) frame_.color = state.perception.color;

  frame_.cycleBusy = state.time.cycleBusy;
  frame_.cyclePeriod = state.time.cyclePeriod;

  // Flags -- the single assembly point (124-009, issue §B1): every bit
  // that CAN be known at this point in the cycle, derived straight from
  // `state`, replacing the ten scattered setFlag() calls the old
  // assembleFrame() made. kFlagFaultMoveTimeout/kFlagFaultShapingDisabled
  // (bits 15/16) are deliberately NOT touched here -- see
  // setLiveFlag()'s own doc comment (telemetry.h) for why their
  // defining condition does not exist yet at update() time, and why
  // leaving them untouched here (rather than re-deriving them from a
  // state.health field that itself is only fresh post-tick) is exactly
  // what reproduces the pre-124-009 "flag rides the next frame" behavior:
  // whatever setLiveFlag() last set survives across this update()
  // call unchanged, precisely because this method never mutates those two
  // bits itself.
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
  // Packed word (124-008, issue §B4): corr_id<<4 | err.
  ackRing_[tail] = (corrId << kAckErrBits) | (errCode & kAckErrMask);
  // Undelivered: this entry now forces frames until it has been carried
  // kAckRepeats times (hasSomethingToSay). Reset explicitly rather than
  // relying on the slot's prior value -- a reused slot would otherwise
  // inherit the evicted entry's delivered count and never be sent at all.
  ackSends_[tail] = 0;
}

bool Telemetry::primaryDue(uint32_t now) const {
  if (!everEmittedPrimary_) return true;  // always on from boot -- no arming
  return (now - lastPrimaryEmit_) >= kPrimaryPeriod;
}

// hasSomethingToSay -- the idle-silence gate (stakeholder directive
// 2026-07-29: "I don't want telemetry when the robot's not moving").
//
// Two reasons to speak, and the second is what keeps the wire honest:
//   - motion in progress (kFlagActive): the host needs the stream.
//   - an ack not yet carried kAckRepeats times: acks ride telemetry frames
//     and there is no other delivery path, so going silent with an
//     undelivered ack would strand the outcome of any command issued to a
//     parked robot -- reintroducing the "acked but nothing happened" class
//     that cost this project a bench session.
bool Telemetry::hasSomethingToSay() const {
  // 1. A Move is running.
  if (flags_ & kFlagActive) return true;

  // 2. The WHEELS are turning, whether or not a Move owns them. "Not moving"
  //    has to mean the wheels, not the queue: after a STOP the Move ends
  //    immediately but the chassis coasts, and gating on kFlagActive alone
  //    cut telemetry mid-deceleration -- a harness watching velocity settle
  //    saw it frozen at 366 mm/s because the frames stopped, not the wheels.
  if (frame_.encLeft.velocity != 0 || frame_.encRight.velocity != 0) return true;

  // 3. An ack not yet carried kAckRepeats times. Acks ride telemetry frames
  //    and have no other delivery path, so silence with an undelivered ack
  //    would strand the outcome of any command sent to a parked robot.
  for (uint8_t i = 0; i < ackRingCount_; ++i) {
    const uint8_t idx = static_cast<uint8_t>((ackRingHead_ + i) % kAckRingDepth);
    if (ackSends_[idx] < kAckRepeats) return true;
  }

  // 4. Anything in the flags word CHANGED since the last frame -- boot-ready
  //    coming up, a motor dropping off the bus, a fault latching. Report on
  //    change: a state transition nobody can observe is a state transition
  //    that will be debugged the hard way, and this is the difference
  //    between "quiet because nothing is happening" and "quiet because we
  //    stopped listening".
  if (flags_ != lastEmittedFlags_) return true;

  return false;
}

void Telemetry::emit(uint32_t now, bool force) {
  if (primaryDue(now) && (force || hasSomethingToSay())) {
    emitPrimary(now);
  }
}

void Telemetry::emitPrimary(uint32_t now) {
  msg::Telemetry tlm;

  tlm.now = now;
  // seq wraps at telemetry.proto's own declared (max)=127 (124-008, issue
  // §B5's size-accounting) -- kept genuinely small on the real value too,
  // not just the wire bound, so the two never drift: a caller reading the
  // declared bound sees the true worst case, not an optimistic one.
  tlm.seq = seq_;
  seq_ = (seq_ + 1) % 128u;
  tlm.mode = frame_.mode;

  // flags -- the caller-derived bits (update()'s own single assembly
  // point) plus setLiveFlag()'s two late bits, whatever this
  // call's flags_ currently holds.
  tlm.flags = flags_;

  // Ack ring (120, ADDITIVE) -- serialize the ring's CURRENT contents,
  // oldest-to-newest (ackRingHead_'s own push/evict order), every
  // emitPrimary() call, exactly like every other Frame field: the last
  // staged snapshot, not a diff since the last send (app/DESIGN.md Sec 3).
  // A frame this call sends carries whatever the ring holds RIGHT NOW,
  // regardless of whether a new ack() landed since the last emit -- no
  // separate "is this new" bit applies (telemetry.proto's own
  // Telemetry.acks doc comment). 124-008: each entry is a packed uint32_t
  // (corr_id<<4|err), not msg::AckEntry (deleted).
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
  // Record what this frame reported, so the next hasSomethingToSay() can
  // tell "nothing has changed" from "something changed and nobody saw it".
  lastEmittedFlags_ = flags_;
  ++primaryEmitCount_;
}

}  // namespace App
