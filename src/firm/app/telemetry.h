// telemetry.h -- App::Telemetry: the always-on outbound frame. Builds and
// emits the primary msg::Telemetry frame (ack ring + fault/event bits) at a
// fixed cadence, and the slower msg::TelemetrySecondary diagnostic frame on
// other cycles, never both in the same emit() call.
//
// Boundary: inside -- primary/secondary frame assembly, the depth-3 ack
// ring, fault/event bit encoding, cadence pacing; outside -- deciding WHEN
// a fault occurred (callers -- I2CBus's safety net, Deadman's trip -- set
// the bit; Telemetry only carries it).
//
// Two send paths: the PRIMARY frame rides a
// msg::ReplyEnvelope{corr_id=0, body_kind=TLM} through Comms::sendReply()
// -- Telemetry holds a Comms& for this. TelemetrySecondary is NOT a
// ReplyEnvelope oneof arm (envelope.proto's body oneof is fixed at
// ok/err/tlm) -- it rides as its own, independently-armored "*B" line, so
// Telemetry also holds the two Transport& references directly and performs
// its own armor+broadcast for that one frame type, reusing Comms's public
// kArmoredBufSize constant and WireRuntime::base64Encode() (the same
// primitives Comms::sendReply() itself is built on) rather than
// duplicating a private encode path.
//
// Telemetry is a standalone, testable class: it never holds a pointer to a
// leaf, I2CBus, or Deadman instance (that wiring is RobotLoop's job).
// Callers stage the next frame's data via setFrame()/setSecondaryFrame()
// and report fault/event conditions via setFault()/setEvent() using the
// bit constants below -- Telemetry only carries whatever the caller last
// told it. Design/rationale: DESIGN.md.
#pragma once

#include <cstdint>

#include "app/comms.h"
#include "messages/envelope.h"
#include "messages/telemetry.h"

namespace App {

// --- fault_bits / event_bits layout -----------------------------------
// The single place a reader decodes a bit against. Callers pass the
// current boolean state to setFault()/setEvent(); Telemetry only carries
// it (see this file's boundary comment above).
//
// fault_bits:
//   bit 0 (kFaultI2CSafetyNet) -- I2CBus `readyAt` clearance safety-net
//                                  trip (Devices::I2CBus::
//                                  clearanceSafetyNetCount() > 0). Bench-
//                                  characterized as a boot-time ONE-SHOT
//                                  latch, not a continuous/live indicator:
//                                  it fires once, coincident with
//                                  event_bits first showing
//                                  kEventBootReady, and never re-fires
//                                  (matches clearanceSafetyNetCount()'s own
//                                  monotonic, never-cleared counter
//                                  semantics). A healthy robot can show
//                                  fault_bits bit 0 set permanently after
//                                  boot with no ongoing problem -- do NOT
//                                  read a steady fault=1 as live evidence
//                                  of a defect; only a bit that flips
//                                  DURING driving (not just once at boot)
//                                  is actionable.
//   bit 1 (kFaultWedgeLatch)   -- NezhaMotor/I2CBus wedge-latch detected
//                                  (Devices::MotorArmor::wedged()).
//   bit 2 (kFaultI2CNak)       -- I2C bus NAK/timeout error. Declared, not
//                                  yet wired live (no per-transaction NAK
//                                  aggregate exists yet).
//   bit 3 (kFaultCommsMalformed) -- malformed/undecodable inbound frame
//                                  (App::Comms::malformedCount() > 0 --
//                                  malformed armor prefix, malformed
//                                  base64, malformed protobuf decode, or an
//                                  unrecognized text-plane line all
//                                  increment it).
//   bits 4-31 -- reserved for future faults.
//
// event_bits:
//   bit 0 (kEventDeadmanExpired) -- Deadman staleness timer expired
//                                    (App::Deadman::expired()).
//   bit 1 (kEventBootReady)      -- boot-ready transition
//                                    (Preamble::done() first true).
//   bit 2 (kEventConfigApplied)  -- a ConfigDelta was applied. Declared,
//                                    not yet wired.
//   bits 3-31 -- reserved for future events.
constexpr uint32_t kFaultI2CSafetyNet = 1u << 0;
constexpr uint32_t kFaultWedgeLatch = 1u << 1;
constexpr uint32_t kFaultI2CNak = 1u << 2;
constexpr uint32_t kFaultCommsMalformed = 1u << 3;

constexpr uint32_t kEventDeadmanExpired = 1u << 0;
constexpr uint32_t kEventBootReady = 1u << 1;
constexpr uint32_t kEventConfigApplied = 1u << 2;
constexpr uint32_t kEventHeadingFallback = 1u << 3;  // App::HeadingSource transition (109-005)

// Primary cadence target: ~25 Hz/40 ms. Callers pace against this and
// measure their own real number; emit() does not need to hit it exactly.
constexpr uint32_t kPrimaryPeriod = 40;  // [ms] ~25 Hz

// Secondary cadence: 5x the primary period (~5 Hz) keeps the diagnostic
// frame far enough from the primary's own deadline that the two
// essentially never contend for the same emit() call, while still
// refreshing at a useful bench-diagnostic rate.
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

    // Motion::Executor visibility (109-003) -- mirrors telemetry.proto's
    // queue_depth/active_id/exec_state fields field-for-field. Populated
    // by RobotLoop::updateTlm() from App::Pilot's own accessors.
    uint8_t queueDepth = 0;
    uint32_t activeId = 0;
    msg::ExecutorState execState = msg::ExecutorState::EXEC_IDLE;

    // App::HeadingSource visibility (109-005, SUC-004) -- mirrors
    // telemetry.proto's heading_source field. Populated by RobotLoop::
    // updateTlm() from App::Pilot::headingSourceIsOtos().
    msg::HeadingSourceStatus headingSource = msg::HeadingSourceStatus::HEADING_SOURCE_STATUS_OTOS;
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
  // Tie-break: at a real loop period at or above kPrimaryPeriod (40ms),
  // primaryDue() can be true on EVERY call -- an unconditional "primary
  // always wins a tie" rule then starves secondary to 0 Hz forever. The
  // fix: when BOTH frames are genuinely due in the same call, ALTERNATE
  // instead of always favoring primary -- `tieFavorsSecondary_` flips
  // after every tie. "Genuinely due" for secondary's own
  // pre-first-ever-emission window means real elapsed time
  // (`now >= kSecondaryPeriod`), NOT secondaryDue()'s own
  // "!everEmittedSecondary_ -> true" boot bypass -- otherwise a caller
  // whose second-ever call already lands on/after kPrimaryPeriod (long
  // before any real starvation) would spuriously tie-divert onto
  // secondary. Because secondaryDue() only stays true once every
  // kSecondaryPeriod (200 ms) until an actual secondary send resets it,
  // this alternation costs at most ONE primary frame delayed by one loop
  // cycle roughly once per kSecondaryPeriod (the very next call, no
  // longer tied, sends the deferred primary immediately) -- primary's own
  // steady-state cadence is otherwise untouched, and secondary is
  // guaranteed a slot within roughly one kSecondaryPeriod instead of
  // starving forever. A non-tied call (only one of the two genuinely due)
  // is unaffected: that frame sends immediately.
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

  // Tie-break state (see emit()'s own comment): false means the NEXT
  // simultaneous-due tie favors primary, true means it favors secondary.
  // Starts false so the very first-ever call (both "due" by construction)
  // still sends primary, preserving the documented no-arming-step boot
  // contract.
  bool tieFavorsSecondary_ = false;
};

}  // namespace App
