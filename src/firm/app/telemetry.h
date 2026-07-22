// telemetry.h -- App::Telemetry: the always-on outbound frame. Builds and
// emits the primary msg::Telemetry frame (single ack slot + a unified
// flags bit-string) at a fixed cadence, and the slower msg::TelemetrySecondary
// diagnostic frame on other cycles, never both in the same emit() call.
//
// Boundary: inside -- primary/secondary frame assembly, the single ack
// slot, flags-bit encoding, cadence pacing; outside -- deciding WHEN a
// fault/event/presence condition occurred (callers -- I2CBus's safety net,
// Deadman's trip, RobotLoop's own updateTlm()/line-color polling -- set the
// bit; Telemetry only carries it and folds in its own ack_fresh bit at
// encode time).
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
// and report status/fault/event conditions via setFlag() using the bit
// constants below -- Telemetry only carries whatever the caller last told
// it, plus its own internally-tracked ack_fresh bit. Design/rationale:
// DESIGN.md.
#pragma once

#include <cstdint>

#include "app/comms.h"
#include "messages/telemetry.h"

namespace App {

// --- flags bit layout (115-005, gut S1 -- telemetry-frame-tightening-
// amendment-to-gut-s1.md) ------------------------------------------------
// The single place a reader decodes a bit against. Callers pass the
// current boolean state to setFlag(); Telemetry only carries it (see this
// file's boundary comment above), except bit 5 (ack_fresh), which Telemetry
// tracks itself from ack() call timing and ORs in at encode time -- a
// caller never calls setFlag(kFlagAckFresh, ...) directly.
//
//   bit 0  (kFlagOtosPresent)    -- OtosReading fresh THIS frame (chip
//                                    detected AND this cycle's burst read
//                                    actually refreshed the cached pose --
//                                    see odometry.h's applyOtosSample()
//                                    doc comment). Frame.otos is valid iff
//                                    this bit is set.
//   bit 1  (kFlagOtosConnected)  -- live OTOS bus health.
//   bit 2  (kFlagActive)         -- motion in progress.
//   bit 3  (kFlagConnLeft)       -- left motor bus connectivity.
//   bit 4  (kFlagConnRight)      -- right motor bus connectivity.
//   bit 5  (kFlagAckFresh)       -- ack_corr/ack_err are a NEW ack this
//                                    frame (Telemetry-internal -- see
//                                    above).
//   bit 6  (kFlagFaultI2CSafetyNet) -- I2CBus `readyAt` clearance
//                                    safety-net trip
//                                    (Devices::I2CBus::
//                                    clearanceSafetyNetCount() > 0).
//                                    Bench-characterized as a boot-time
//                                    ONE-SHOT latch, not a continuous/live
//                                    indicator: it fires once, coincident
//                                    with bit 11 (kFlagEventBootReady)
//                                    first setting, and never re-fires. A
//                                    healthy robot can show this bit set
//                                    permanently after boot with no
//                                    ongoing problem -- do NOT read a
//                                    steady 1 as live evidence of a
//                                    defect; only a bit that flips DURING
//                                    driving (not just once at boot) is
//                                    actionable.
//   bit 7  (kFlagFaultWedgeLatch)   -- NezhaMotor/I2CBus wedge-latch
//                                    detected (Devices::MotorArmor::
//                                    wedged()).
//   bit 8  (kFlagFaultI2CNak)       -- I2C NAK/timeout. Declared, not yet
//                                    wired live (no per-transaction NAK
//                                    aggregate exists yet).
//   bit 9  (kFlagFaultCommsMalformed) -- malformed/undecodable inbound
//                                    frame (App::Comms::malformedCount() >
//                                    0).
//   bit 10 (kFlagEventDeadmanExpired) -- Deadman staleness timer expired
//                                    (App::Deadman::expired()), the
//                                    transition cycle only.
//   bit 11 (kFlagEventBootReady)    -- boot-ready transition
//                                    (Preamble::done() first true).
//   bit 12 (kFlagEventConfigApplied) -- a ConfigDelta was applied.
//                                    Declared, not yet wired.
//   bit 13 (kFlagLinePresent)       -- line word fresh THIS frame.
//   bit 14 (kFlagColorPresent)      -- color word fresh THIS frame.
//   bit 15 (kFlagFaultMoveTimeout)  -- MOVE timeout backstop fired.
//                                    Declared now, wired by sprint 116's
//                                    protocol-set-point issue -- S1 has no
//                                    MOVE command to time out.
//   bits 16-31 -- reserved for future use.
constexpr uint32_t kFlagOtosPresent = 1u << 0;
constexpr uint32_t kFlagOtosConnected = 1u << 1;
constexpr uint32_t kFlagActive = 1u << 2;
constexpr uint32_t kFlagConnLeft = 1u << 3;
constexpr uint32_t kFlagConnRight = 1u << 4;
constexpr uint32_t kFlagAckFresh = 1u << 5;  // Telemetry-internal -- see above
constexpr uint32_t kFlagFaultI2CSafetyNet = 1u << 6;
constexpr uint32_t kFlagFaultWedgeLatch = 1u << 7;
constexpr uint32_t kFlagFaultI2CNak = 1u << 8;
constexpr uint32_t kFlagFaultCommsMalformed = 1u << 9;
constexpr uint32_t kFlagEventDeadmanExpired = 1u << 10;
constexpr uint32_t kFlagEventBootReady = 1u << 11;
constexpr uint32_t kFlagEventConfigApplied = 1u << 12;
constexpr uint32_t kFlagLinePresent = 1u << 13;
constexpr uint32_t kFlagColorPresent = 1u << 14;
constexpr uint32_t kFlagFaultMoveTimeout = 1u << 15;

// Primary cadence target: primary period == cycle period (115-005, closes
// kcycle-kprimaryperiod-mismatch.md -- the frame is emitted every loop
// iteration, ~50 Hz/20 ms). Callers pace against this and measure their
// own real number; emit() does not need to hit it exactly.
constexpr uint32_t kPrimaryPeriod = 20;  // [ms] ~50 Hz, matches robot_loop.cpp's kCycle

// Secondary cadence: 10x the primary period (~5 Hz) keeps the diagnostic
// frame far enough from the primary's own deadline that the two
// essentially never contend for the same emit() call, while still
// refreshing at a useful bench-diagnostic rate.
constexpr uint32_t kSecondaryPeriod = 200;  // [ms] ~5 Hz

class Telemetry {
 public:
  // Primary-frame snapshot -- staged by RobotLoop's updateTlm()/kPace block
  // and consumed whole by emitPrimary() (envelope-independent: no
  // acks/now/seq/flags here -- those are owned by the ack slot, emit()'s
  // own `now` argument, an internal sequence counter, and setFlag()
  // respectively).
  struct Frame {
    msg::DriveMode mode = msg::DriveMode::IDLE;

    msg::EncoderReading encLeft{};
    msg::EncoderReading encRight{};

    msg::OtosReading otos{};
    bool otosPresent = false;    // staging only (not wire) -- flags bit 0 source
    bool otosConnected = false;  // staging only (not wire) -- flags bit 1 source

    msg::Pose2D pose{};
    msg::BodyTwist3 twist{};

    uint32_t line = 0;
    bool linePresent = false;   // staging only (not wire) -- flags bit 13 source
    uint32_t color = 0;
    bool colorPresent = false;  // staging only (not wire) -- flags bit 14 source
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

  // Generic flags-bit set/clear -- `bit` is one of the kFlag* constants
  // above EXCEPT kFlagAckFresh (Telemetry-internal, driven by ack() calls
  // only -- see that constant's own comment). Level-set, not edge-latched:
  // the caller mirrors whatever it currently observes (e.g.
  // `setFlag(kFlagFaultI2CSafetyNet, i2cBus.clearanceSafetyNetCount() >
  // 0)`), so a bit clears the cycle its condition clears -- Telemetry
  // invents no sticky-latch semantics on top of what the real call site
  // already reports.
  void setFlag(uint32_t bit, bool active);
  uint32_t flags() const { return flags_; }

  // ack -- pushes the single ack slot (115-005: replaces the old depth-3
  // ack ring -- ack-depth-1 is a stakeholder-accepted tradeoff, rare at
  // bench rates, wait_for_ack timeout+retry covers it). errCode == 0 means
  // OK; nonzero is the msg::ErrCode value. Marks the ack "fresh" so the
  // VERY NEXT emitPrimary() call sets flags bit 5 (kFlagAckFresh) and then
  // clears the fresh marker -- a one-shot pulse, not a level condition.
  void ack(uint32_t corrId, uint32_t errCode);

  // Cadence-gated: call once per loop cycle with the current time [ms]
  // (also the wire `now` field's value for whichever frame this call
  // sends). Sends AT MOST ONE frame type per call. Bounded work: one frame
  // build, one encode, one armor, up to two Transport sends -- never
  // sleeps, never touches the I2C bus. ALWAYS ON from boot: the first
  // call always sends the primary frame (no arming step, and a tie on
  // that very first call always resolves to primary -- see the tie-break
  // note below).
  //
  // Tie-break: at a real loop period at or above kPrimaryPeriod (20ms),
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

  uint32_t flags_ = 0;  // every bit EXCEPT kFlagAckFresh -- see setFlag()

  uint32_t ackCorr_ = 0;
  uint32_t ackErr_ = 0;
  bool ackPending_ = false;  // true iff ack() was called since the last emitPrimary()

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
