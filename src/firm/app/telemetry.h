// telemetry.h -- App::Telemetry: the always-on outbound frame. Builds and
// emits the primary msg::Telemetry frame (ack ring + fault/event bits) at a
// fixed cadence, and the slower msg::TelemetrySecondary diagnostic frame on
// other cycles, NEVER both in the same emit() call.
//
// architecture-update.md (103) Step 3 "Telemetry" boundary: inside --
// primary/secondary frame assembly, the depth-3 ack ring, fault/event bit
// encoding, cadence pacing; outside -- deciding WHEN a fault occurred
// (callers -- I2CBus's safety net, Deadman's trip -- set the bit; Telemetry
// only carries it). Serves SUC-005.
//
// Two send paths, per protos/telemetry.proto's own Decision 3 resolution
// (ticket 001's completion notes): the PRIMARY frame rides a
// msg::ReplyEnvelope{corr_id=0, body_kind=TLM} through Comms::sendReply()
// (ticket 004) -- Telemetry holds a Comms& for this. TelemetrySecondary is
// NOT a ReplyEnvelope oneof arm (envelope.proto's body oneof is fixed at
// ok/err/tlm) -- it rides as its own, independently-armored "*B" line, so
// Telemetry also holds the two Transport& references directly (the
// architecture-update.md (103) Step 4 "Telemetry --> Com" dependency-graph
// edge, distinct from -- and in addition to -- "Telemetry --> Comms" for
// the primary path) and performs its own armor+broadcast for that one
// frame type, reusing Comms's public kArmoredBufSize constant and
// WireRuntime::base64Encode() (the same primitives Comms::sendReply()
// itself is built on) rather than duplicating a private encode path.
//
// This ticket (103-005) builds Telemetry as a standalone, testable class:
// it never holds a pointer to a leaf, I2CBus, or Deadman instance (that
// wiring is ticket 008's loop construction). Callers stage the next
// frame's data via setFrame()/setSecondaryFrame() and report fault/event
// conditions via setFault()/setEvent() using the bit constants below --
// Telemetry only carries whatever the caller last told it, per the
// boundary comment above.
#pragma once

#include <cstdint>

#include "app/comms.h"
#include "messages/envelope.h"
#include "messages/telemetry.h"

namespace App {

// --- fault_bits / event_bits layout -----------------------------------
// Decided by ticket 001 (protos/telemetry.proto's own doc comment) and
// reproduced here verbatim, per this ticket's own documentation-update
// requirement, as the single place a future reader looks to decode a bit.
//
// fault_bits:
//   bit 0 (kFaultI2CSafetyNet) -- I2CBus `readyAt` clearance safety-net
//                                  trip (source/devices/i2c_bus.h,
//                                  Devices::I2CBus::clearanceSafetyNetCount()
//                                  -- ticket 002). WIRED this ticket: the
//                                  real call site's boolean result is what
//                                  a caller passes to setFault().
//                                  CHARACTERIZED by ticket 103-010's bench
//                                  session: a boot-time ONE-SHOT latch, not
//                                  a continuous/live indicator -- it fires
//                                  once, coincident with the frame
//                                  `event_bits` first shows
//                                  kEventBootReady (Preamble::done()'s
//                                  first-true transition; plausibly
//                                  preamble's own hardReset()-driven
//                                  back-to-back device-detection writes),
//                                  and then never re-fires (matches
//                                  clearanceSafetyNetCount()'s own
//                                  monotonic, never-cleared counter
//                                  semantics). Observed pegged at exactly 1
//                                  across sustained driving in every
//                                  re-run capture in that session, with no
//                                  behavioral signal (no stall, no dropped
//                                  ack, no missed cycle) correlated with
//                                  driving activity. A healthy robot can
//                                  show fault_bits bit 0 set permanently
//                                  after boot with no ongoing problem -- a
//                                  future bench reader should NOT chase a
//                                  steady fault=1 as live evidence of a
//                                  defect; only a bit that flips DURING
//                                  driving (not just once at boot) is
//                                  actionable.
//   bit 1 (kFaultWedgeLatch)   -- NezhaMotor/I2CBus wedge-latch detected
//                                  (Devices::MotorArmor::wedged(), ticket
//                                  002/003). Declared, not yet wired live
//                                  by any ticket -- no dead-bit ambiguity:
//                                  the constant exists so a future ticket
//                                  (008) calls setFault(kFaultWedgeLatch, ...)
//                                  without inventing a new bit number.
//   bit 2 (kFaultI2CNak)       -- I2C bus NAK/timeout error. Declared, not
//                                  yet wired live (no per-transaction NAK
//                                  aggregate exists at this ticket's scope).
//   bit 3 (kFaultCommsMalformed) -- malformed/undecodable inbound frame
//                                  (App::Comms::malformedCount() > 0 --
//                                  source/app/comms.h/.cpp; malformed
//                                  armor prefix, malformed base64,
//                                  malformed protobuf decode, or an
//                                  unrecognized text-plane line all
//                                  increment it). WIRED by ticket 104-004:
//                                  main.cpp's loop reads
//                                  Comms::malformedCount() every cycle,
//                                  same idiom as kFaultI2CSafetyNet above.
//   bits 4-31 -- reserved for future faults.
//
// event_bits:
//   bit 0 (kEventDeadmanExpired) -- Deadman staleness timer expired
//                                    (source/app/deadman.h,
//                                    App::Deadman::expired() -- ticket
//                                    004). WIRED this ticket.
//   bit 1 (kEventBootReady)      -- boot-ready transition
//                                    (Preamble::done() first true, ticket
//                                    007). Declared, not yet wired --
//                                    Preamble does not exist yet.
//   bit 2 (kEventConfigApplied)  -- a ConfigDelta was applied. Declared,
//                                    not yet wired -- runtime ConfigDelta
//                                    application is a ticket-008-time
//                                    decision (architecture-update.md (103)
//                                    Step 7 Open Question 3).
//   bits 3-31 -- reserved for future events.
constexpr uint32_t kFaultI2CSafetyNet = 1u << 0;
constexpr uint32_t kFaultWedgeLatch = 1u << 1;
constexpr uint32_t kFaultI2CNak = 1u << 2;
constexpr uint32_t kFaultCommsMalformed = 1u << 3;

constexpr uint32_t kEventDeadmanExpired = 1u << 0;
constexpr uint32_t kEventBootReady = 1u << 1;
constexpr uint32_t kEventConfigApplied = 1u << 2;

// Primary cadence target: spike-001's 25 Hz/40 ms measurement
// (architecture-update.md (103) Step 7 Open Question 5) -- this ticket
// does not need to HIT this exactly, only pace against it and measure its
// own real number (this ticket's own acceptance criterion).
constexpr uint32_t kPrimaryPeriod = 40;  // [ms] ~25 Hz

// Secondary cadence: this ticket's own P4 implementation decision
// (architecture-update.md (103) Step 7 Open Question 4, left open by
// Decision 3) -- 5x the primary period (~5 Hz) keeps the diagnostic frame
// far enough from the primary's own deadline that the two essentially
// never contend for the same emit() call, while still refreshing at a
// useful bench-diagnostic rate.
constexpr uint32_t kSecondaryPeriod = 200;  // [ms] ~5 Hz

class Telemetry {
 public:
  // Primary-frame snapshot -- mirrors msg::Telemetry's own has_*/value
  // pairs (envelope-independent: no acks/now/seq/fault_bits/event_bits
  // here -- those are owned by the ack ring, emit()'s own `now` argument,
  // an internal sequence counter, and setFault()/setEvent() respectively).
  struct Frame {
    msg::DriveMode mode = msg::DriveMode::IDLE;
    bool hasEnc = false;
    float encLeft = 0.0f;   // [mm]
    float encRight = 0.0f;  // [mm]
    bool hasVel = false;
    float velLeft = 0.0f;   // [mm/s] signed
    float velRight = 0.0f;  // [mm/s] signed
    bool hasPose = false;
    msg::Pose2D pose{};
    bool hasOtos = false;
    msg::Pose2D otos{};
    bool otosConnected = false;
    bool hasTwist = false;
    msg::BodyTwist3 twist{};
    bool active = false;
    bool connLeft = false;
    bool connRight = false;
  };

  // Secondary-frame snapshot -- mirrors msg::TelemetrySecondary's own
  // has_*/value pairs (no `now` -- emit()'s own argument fills it).
  struct SecondaryFrame {
    bool hasCmdVel = false;
    float cmdVelLeft = 0.0f;   // [mm/s] signed
    float cmdVelRight = 0.0f;  // [mm/s] signed
    float accLeft = 0.0f;      // [mm/s^2] EMA-filtered
    float accRight = 0.0f;     // [mm/s^2] EMA-filtered
    uint32_t glitchLeft = 0;
    uint32_t glitchRight = 0;
    uint32_t tsLeft = 0;   // [ms]
    uint32_t tsRight = 0;  // [ms]
  };

  // comms -- primary-frame send path (Comms::sendReply(), ticket 004).
  // serialLink/radioLink -- direct Transport access for TelemetrySecondary's
  // own independently-armored line (see this file's own header comment).
  Telemetry(Comms& comms, Transport& serialLink, Transport& radioLink);

  // Stage the next frame's snapshot data. Persists across emit() calls
  // that don't send that frame type -- emit() always encodes the LAST
  // staged snapshot, not "only what changed since the last send".
  void setFrame(const Frame& frame);
  void setSecondaryFrame(const SecondaryFrame& frame);

  // Generic bit set/clear -- `bit` is one of the k*/kEvent* constants
  // above (or a future one this ticket declares but doesn't wire). Level-
  // set, not edge-latched: the caller mirrors whatever it currently
  // observes (e.g. `setFault(kFaultI2CSafetyNet,
  // i2cBus.clearanceSafetyNetCount() > 0)`), so a bit clears the cycle its
  // condition clears -- Telemetry invents no sticky-latch semantics on top
  // of what the real call site already reports.
  void setFault(uint32_t bit, bool active);
  void setEvent(uint32_t bit, bool active);
  uint32_t faultBits() const { return faultBits_; }
  uint32_t eventBits() const { return eventBits_; }

  // Ack ring: pushes one entry; the ring holds exactly the last 3 (oldest
  // evicted first). Every PRIMARY emit() call carries the ring's current,
  // full contents (not just entries pushed since the last send) -- a
  // single dropped/unread frame can never lose an ack, because the very
  // next primary frame repeats it.
  void ack(uint32_t corrId, msg::AckStatus status, uint32_t errCode);

  // Cadence-gated: call once per loop cycle with the current time [ms]
  // (also the wire `now` field's value for whichever frame this call
  // sends). Sends AT MOST ONE frame type per call. Bounded work: one frame
  // build, one encode, one armor, up to two Transport sends -- never
  // sleeps, never touches the I2C bus. ALWAYS ON from boot: the first
  // call always sends the primary frame (no arming step, and a tie on
  // that very first call always resolves to primary -- see the tie-break
  // note below).
  //
  // Tie-break (106-002 fix, `secondary-telemetry-starved-by-106-001-
  // cadence-retarget.md`): 106-001 retargeted the real loop period to
  // ~52 ms, ABOVE kPrimaryPeriod (40 ms), so primaryDue() is true on
  // EVERY call -- under the old "primary always wins a same-call tie"
  // rule (this file's own pre-106-002 comment, and the 103-009 comment it
  // quoted), secondary NEVER got a turn at all (measured 0 Hz on the
  // stand). The fix: when BOTH frames are genuinely due in the same call,
  // ALTERNATE instead of always favoring primary -- `tieFavorsSecondary_`
  // flips after every tie. "Genuinely due" for secondary's own
  // pre-first-ever-emission window means real elapsed time
  // (`now >= kSecondaryPeriod`), NOT secondaryDue()'s own
  // "!everEmittedSecondary_ -> true" boot bypass -- otherwise a caller
  // whose second-ever call already lands on/after kPrimaryPeriod (e.g.
  // exactly kPrimaryPeriod after the first, long before any real
  // starvation) would spuriously tie-divert onto secondary (emit.cpp's
  // own comment has the full derivation). Because secondaryDue() only
  // stays true once every kSecondaryPeriod (200 ms) until an actual
  // secondary send resets it, this alternation costs at most ONE primary
  // frame delayed by one loop cycle roughly once per kSecondaryPeriod
  // (the very next call, no longer tied, sends the deferred primary
  // immediately) -- primary's own steady-state cadence is otherwise
  // untouched, and secondary is guaranteed a slot within roughly one
  // kSecondaryPeriod instead of starving forever. A non-tied call (only
  // one of the two genuinely due) is unaffected: that frame sends
  // immediately, exactly as before.
  void emit(uint32_t now);

  // Measurement/test seam -- lets a HOST_BUILD test report the realized
  // cadence (this ticket's own acceptance criterion) without parsing a
  // FakeTransport's send log.
  uint32_t primaryEmitCount() const { return primaryEmitCount_; }
  uint32_t secondaryEmitCount() const { return secondaryEmitCount_; }
  uint32_t lastPrimaryEmit() const { return lastPrimaryEmit_; }      // [ms]
  uint32_t lastSecondaryEmit() const { return lastSecondaryEmit_; }  // [ms]

 private:
  bool primaryDue(uint32_t now) const;
  bool secondaryDue(uint32_t now) const;
  void emitPrimary(uint32_t now);
  void emitSecondary(uint32_t now);

  Comms& comms_;
  Transport& serialLink_;
  Transport& radioLink_;

  Frame frame_;
  SecondaryFrame secondaryFrame_;

  msg::AckEntry ring_[3]{};
  uint8_t ringCount_ = 0;  // number of valid entries in ring_[0..ringCount_)

  uint32_t faultBits_ = 0;
  uint32_t eventBits_ = 0;

  uint32_t seq_ = 0;  // increments once per SENT primary frame

  bool everEmittedPrimary_ = false;
  uint32_t lastPrimaryEmit_ = 0;  // [ms]
  uint32_t primaryEmitCount_ = 0;

  bool everEmittedSecondary_ = false;
  uint32_t lastSecondaryEmit_ = 0;  // [ms]
  uint32_t secondaryEmitCount_ = 0;

  // 106-002 tie-break state (see emit()'s own comment): false means the
  // NEXT simultaneous-due tie favors primary, true means it favors
  // secondary. Starts false so the very first-ever call (both "due" by
  // construction) still sends primary, preserving the documented
  // no-arming-step boot contract.
  bool tieFavorsSecondary_ = false;
};

}  // namespace App
