// telemetry.h -- App::Telemetry: the always-on outbound frame. Projects
// Types::RobotState into the primary msg::Telemetry frame (single ack slot
// + a unified flags bit-string) and emits it at a fixed cadence.
//
// 124-009 (issue §B1, "RobotState is not the wire frame"): Telemetry no
// longer holds caller-staged `Frame`/`SecondaryFrame` snapshots filled by
// ten scattered setFlag() calls plus a ten-argument assembleFrame(). One
// method, update(const Types::RobotState&), reads the state ONCE and
// stages the WHOLE next frame -- every field the old assembleFrame() used
// to fill, and every flag the old scattered setFlag() calls used to set,
// derived here instead. `TelemetrySecondary` -- the frame type, the wire
// schema arm, and the tie-break/alternation cadence machinery that used to
// pick between it and the primary frame -- is DELETED outright (issue's
// own "Telemetry is a lean projection -- and TelemetrySecondary dies"):
// it emitted nothing but `now` in production (no firmware caller ever
// populated it), and `state ⊇ wire` means there is no wire-visibility
// invariant obligating anything to fold into the primary frame that the
// host does not genuinely consume (see this ticket's own commit message
// for the survivor-field review -- nothing on TelemetrySecondary had a
// live host consumer; `protocol.py`'s own `cmd_vel` field is documented
// as "permanent gap, TelemetrySecondary only," i.e. never actually wired
// off the wire).
//
// Boundary: inside -- the RobotState-to-wire projection (scaled field
// conversion, `age` timestamps, flag derivation), the bounded ack ring,
// cadence pacing; outside -- RobotState's own construction (RobotLoop's
// job), COBS/CRC framing (Comms::sendReply()'s job, via WireRuntime).
//
// Send path: the primary frame rides a
// msg::ReplyEnvelope{corr_id=0, body_kind=TLM} through Comms::sendReply()
// -- Telemetry holds a Comms& for this and nothing else; it no longer
// holds direct Transport& references (those existed only for
// TelemetrySecondary's own independently-armored line, now deleted).
//
// Telemetry is a standalone, testable class: it never holds a pointer to a
// leaf, I2CBus, or Deadman instance (that wiring is RobotLoop's job).
// Design/rationale: DESIGN.md.
#pragma once

#include <cstdint>

#include "app/comms.h"
#include "firm/types/robot_state.h"
#include "messages/telemetry.h"

namespace App {

// --- flags bit layout (115-005, gut S1 -- telemetry-frame-tightening-
// amendment-to-gut-s1.md) ------------------------------------------------
// The single place a reader decodes a bit against. Every bit below is
// derived INSIDE update() from a Types::RobotState field, except bits 15/16
// (kFlagFaultMoveTimeout/kFlagFaultShapingDisabled), which setLiveFlag()
// sets directly -- see that method's own doc comment for why those two
// cannot fold into update().
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
//   bit 5  -- RESERVED (124-008: formerly kFlagAckFresh -- deleted with
//                                    the single "freshest ack" scalar slot
//                                    it gated; ring membership already
//                                    means "really acked").
//   bit 6  (kFlagFaultI2CSafetyNet) -- I2CBus `readyAt` clearance
//                                    safety-net trip
//                                    (Devices::I2CBus::
//                                    clearanceSafetyNetCount() > 0).
//                                    Bench-characterized (120-003,
//                                    pyOCD/DBG trace against real
//                                    hardware, 2026-07-23) as a
//                                    CONTINUOUSLY LIVE, monotonically
//                                    growing counter, NOT a boot-time
//                                    one-shot latch (a prior claim here
//                                    was wrong -- falsified by direct
//                                    on-chip measurement: the raw count
//                                    keeps climbing for as long as the
//                                    robot runs, idle or driving, at a
//                                    rate matching Devices::Otos's own
//                                    ~20ms read cadence 1:1). Root cause:
//                                    Devices::Otos::readPositionVelocity()
//                                    (and its sibling register helpers)
//                                    issue a register-select write()
//                                    immediately followed by a read() on
//                                    the SAME device with no intervening
//                                    loop-scheduled gap, so
//                                    waitForClearance() trips on every
//                                    single Otos burst read,
//                                    unconditionally -- this has nothing
//                                    to do with the loop schedule or
//                                    118-001's kSettle/kClear restore
//                                    (confirmed on hardware: the motor's
//                                    own split-phase request/collect,
//                                    which DOES cross a real scheduled
//                                    gap, contributes ZERO trips in
//                                    either an idle or a driving window).
//                                    A healthy robot with the real OTOS
//                                    present therefore shows this bit set
//                                    on nearly every frame, idle AND
//                                    driving, by design -- do NOT read a
//                                    steady 1 as evidence of a motor/
//                                    loop-timing defect. The bit's CURRENT
//                                    derivation (never reset) also cannot
//                                    surface a genuine FUTURE motor-side
//                                    regression, since it saturates to 1
//                                    within about a second of boot and
//                                    stays there -- see
//                                    clasi/issues/i2c-safety-net-bit-conflates-otos-settle-wait-with-loop-schedule-health.md
//                                    for the follow-up fix options (a
//                                    stakeholder-level design choice, not
//                                    guessed here).
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
//                                    Set via setLiveFlag(), not
//                                    update() -- see that method's doc
//                                    comment.
//   bit 16 (kFlagFaultShapingDisabled) -- a MOVE is active AND BOTH
//                                    ShaperLimits axes (linear, angular)
//                                    are disabled (App::MoveQueue::
//                                    shapingDisabled(), mirroring
//                                    shapeAndStage()'s own early-return
//                                    gate, move_queue.cpp) -- the loud
//                                    off-state for the silent-off
//                                    shaping/anticipation config boundary
//                                    (119 ticket 001,
//                                    kill-the-silent-off-shaping-config-
//                                    boundary.md). Set via
//                                    setLiveFlag(), not update() --
//                                    see that method's doc comment.
//   bit 17 (kFlagFaultPositionClamped) -- 124-008, position-rebaseline
//                                    policy's defensive fallback (sprint
//                                    124 architecture Decision 6): a
//                                    wheel's position was clamped to
//                                    EncoderReading.position's own
//                                    (abs_max) at the encode step rather
//                                    than allowed to wrap. Not the
//                                    expected path -- RobotLoop's own
//                                    per-cycle rebaseline trigger (a
//                                    2000mm margin below the bound) should
//                                    prevent this in normal operation;
//                                    purely observable evidence the
//                                    defensive fallback engaged. Derived
//                                    from Types::RobotState::Health::
//                                    positionClamped inside update() (124-009).
//   bit 18 (kFlagFaultCommandsDropped) -- command-ingestion-ring-buffered-
//                                    comms-subsystem-routing-two-stops.md
//                                    §1: a well-formed inbound command was
//                                    dropped because App::Comms's command
//                                    ring was full. Distinct in KIND from
//                                    bit 9, not merely in count: bit 9 is a
//                                    wire/link problem (a line arrived
//                                    corrupt), this is firmware
//                                    backpressure (commands arrived faster
//                                    than one cycle's drain could route).
//                                    Derived from Types::RobotState::
//                                    Health::commandsDroppedCount inside
//                                    update(), exactly like bit 9.
//   bits 19-31 -- reserved for future use.
constexpr uint32_t kFlagOtosPresent = 1u << 0;
constexpr uint32_t kFlagOtosConnected = 1u << 1;
constexpr uint32_t kFlagActive = 1u << 2;
constexpr uint32_t kFlagConnLeft = 1u << 3;
constexpr uint32_t kFlagConnRight = 1u << 4;
// bit 5 -- RESERVED (124-008, formerly kFlagAckFresh) -- see above.
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
constexpr uint32_t kFlagFaultShapingDisabled = 1u << 16;
constexpr uint32_t kFlagFaultPositionClamped = 1u << 17;
constexpr uint32_t kFlagFaultCommandsDropped = 1u << 18;

// Primary cadence target: primary period == cycle period (115-005, closes
// kcycle-kprimaryperiod-mismatch.md -- the frame is emitted every loop
// iteration). 118 restores robot_loop.cpp's own kCycle to its genuine
// 40ms/~25Hz (kSettle/kClear had been zeroed to fake a 20ms cycle -- see
// clasi/issues/restore-the-interleaved-request-settle-tick-loop-schedule.md),
// so this constant follows it back to 40ms. Callers pace against this and
// measure their own real number; emit() does not need to hit it exactly.
constexpr uint32_t kPrimaryPeriod = 40;  // [ms] ~25 Hz, matches robot_loop.cpp's kCycle

// Ack ring depth (120, ack-ring ticket -- bench-single-ack-slot-
// observability-collapses-at-40ms.md). MUST match telemetry.proto's
// Telemetry.acks (max_count) exactly -- checked by a static_assert against
// the generated msg::Telemetry::acks_[] array width in telemetry.cpp, so a
// future edit to either side without the other fails the build rather than
// silently truncating the ring. Chosen (not merely "some depth"): the
// issue's own suggested "e.g. 4", sized against the queue's 5-deep
// ERR_FULL ceiling by this ticket's own rapid-fire test -- see
// sprint.md's Architecture Decision 1 for the full alternatives-considered
// rationale.
//
// 4 -> 12 (command-ingestion-ring-buffered-comms-subsystem-routing-two-
// stops.md §1): the sizing constraint changed. With App::Comms's command
// ring, RobotLoop drains up to App::kCmdRingDepth commands in a SINGLE
// cycle and acks each one, so a burst that fits the command ring must also
// fit here or it executes unobservably -- a host chaining on completion
// acks would hang on a command that really did run. Kept EQUAL to
// App::kCmdRingDepth (comms.h); change the two together.
constexpr uint8_t kAckRingDepth = 12;

class Telemetry {
 public:
  // Wire-shaped staging area, filled WHOLE by update() every call -- no
  // caller-visible setFrame()/setFlag() any more (124-009). Presence bools
  // (otosPresent/linePresent/...) that used to live here purely to carry
  // flag-derivation inputs from assembleFrame() into Telemetry are GONE --
  // update() derives flags straight from the Types::RobotState argument it
  // is handed, so this struct is now pure wire-shaped data.
  struct Frame {
    msg::DriveMode mode = msg::DriveMode::IDLE;

    msg::EncoderReading encLeft{};
    msg::EncoderReading encRight{};

    msg::OtosReading otos{};

    msg::Pose2D pose{};
    msg::BodyTwist3 twist{};

    uint32_t line = 0;
    uint32_t color = 0;

    uint32_t cycleBusy = 0;    // [us] cycleStart -> frame-staging instant, THIS cycle
    uint32_t cyclePeriod = 0;  // [us] this cycle's own cycleStart minus the previous cycle's
  };

  // comms -- primary-frame send path (Comms::sendReply()). No Transport&
  // references any more (124-009): those existed only for
  // TelemetrySecondary's own independently-armored line, now deleted --
  // Comms already owns both transports internally and Telemetry never
  // needs to reach them directly.
  explicit Telemetry(Comms& comms);

  // update -- THE one-line RobotState projection (124-009, issue §B1).
  // Reads `state` ONCE and stages the WHOLE next frame: every field the old
  // ten-argument assembleFrame() used to fill (scaled position/velocity/
  // pose/twist/otos conversion, packed line/color words, loop-timing
  // cycleBusy/cyclePeriod) AND every flag the old ten scattered setFlag()
  // calls used to set (all derived from `state` here, not set alongside
  // it). `age` fields (EncoderReading.age/OtosReading.age, issue §B2/
  // SUC-006) are computed as `now - state.<section>.sampleTime`, clamped to
  // 255 -- `now` here is `state.time.cycleStart + state.time.cycleBusy`
  // (converted [us]->[ms]), NOT the bare wire `now` field's own value
  // (`state.time.cycleStart` alone, unchanged, still what the caller's own
  // emit(now) call passes): every sampleTime is captured DURING this
  // cycle's body, strictly after the top-of-cycle cycleStart mark, so
  // ageOf() against cycleStart alone would see sampleTime "in the future"
  // and floor to 0 every time -- cycleBusy (measured to the frame-staging
  // instant immediately before this call, RobotLoop's own doc comment)
  // is the latest instant available on state, at or after every
  // sampleTime this cycle published. See ageOf()'s own doc comment.
  //
  // Call EXACTLY ONCE per cycle, before emit() -- RobotLoop::cycle()'s own
  // grep-enforceable contract (SUC-004). Does not itself send anything.
  // Does NOT touch kFlagFaultMoveTimeout/kFlagFaultShapingDisabled, or any
  // bit setLiveFlag() below owns -- update() only ever OR/AND-NOT's the
  // bits it derives from `state`, so a bit it does not own survives an
  // update() call untouched (see setLiveFlag()'s own doc comment for why
  // that is exactly the behavior two of those bits need).
  void update(const Types::RobotState& state);

  // setLiveFlag -- the ONE narrow, deliberately-NOT-named-setFlag escape
  // hatch from update()'s single-assembly-point contract (124-009), for
  // the bits whose defining condition genuinely cannot be known at
  // update() time:
  //   - kFlagFaultMoveTimeout/kFlagFaultShapingDisabled -- both depend on
  //     Motion::MoveQueue::tick()'s own per-cycle outcome, which is not
  //     known yet at update()/emit() time: tick() must stay positioned
  //     AFTER update()/emit() every cycle (protocol-v4 §7.2 -- a
  //     completion ack staged by tick() must not be visible before the
  //     NEXT cycle's own emit() call, and the same "rides the next frame"
  //     timing applies to these two fault bits, bench-verified by
  //     app_robot_loop_harness.cpp's SUC-054/119-001 scenarios: both bits
  //     must already read live via flags() by the time cycle() returns on
  //     the exact cycle tick() ends/toggles them, well before that value
  //     would otherwise reach update()'s next call). RobotLoop::cycle()
  //     calls this immediately after moveQueue_.tick(), the same position
  //     the pre-124-009 code called tlm_.setFlag() from directly.
  //   - kFlagEventBootReady -- boot()'s own one-shot transition (see that
  //     method), fired once, outside cycle() entirely (update() is never
  //     the only writer of every bit -- boot() has no RobotState worth
  //     building yet at that instant beyond the per-device connectivity
  //     bits it already folds into its own throwaway RobotState/update()
  //     call).
  // Mechanically identical to the private setFlag() update() uses
  // internally (a level-set OR/AND-NOT bit mutation) -- only the name and
  // caller differ, so `grep setFlag src/firm/app/robot_loop.cpp` (SUC-004's
  // own acceptance check) finds nothing.
  void setLiveFlag(uint32_t bit, bool active);

  uint32_t flags() const { return flags_; }

  // ack -- pushes to the bounded ack ring (120, ADDITIVE -- see
  // kAckRingDepth's own comment below and telemetry.proto's Telemetry.acks
  // doc comment for the rationale). errCode == 0 means OK; nonzero is the
  // msg::ErrCode value. Acks stay OUT of Types::RobotState entirely
  // (protocol bookkeeping, not robot state -- robot_state.h's own "What
  // stays OUT" note) -- RobotLoop's handleMove()/handleConfig()/
  // handleStop()/moveQueue completion call this directly, unaffected by
  // the update()/setLiveFlag() split above.
  void ack(uint32_t corrId, uint32_t errCode);

  // Cadence-gated: call once per loop cycle with the current time [ms]
  // (also the wire `now` field's value). Sends the primary frame
  // `update()` last staged when due; a no-op otherwise. Bounded work: one
  // frame build, one encode, one armor, one Transport-pair send (via
  // Comms::sendReply()) -- never sleeps, never touches the I2C bus.
  // ALWAYS ON from boot: the first call always sends (no arming step).
  // 124-009: TelemetrySecondary's tie-break/alternation cadence machinery
  // is GONE with the message type itself -- there is only one frame type
  // to pace any more, so this is a plain "due since last send" gate.
  void emit(uint32_t now);

  // Measurement/test seam -- lets a HOST_BUILD test report the realized
  // cadence without parsing a FakeTransport's send log.
  uint32_t primaryEmitCount() const { return primaryEmitCount_; }
  uint32_t lastPrimaryEmit() const { return lastPrimaryEmit_; }  // [ms]

 private:
  bool primaryDue(uint32_t now) const;
  void emitPrimary(uint32_t now);
  void pushAckRing(uint32_t corrId, uint32_t errCode);

  // setFlag -- private now (124-009): the only callers are update() (the
  // ten state-derived bits) and setLiveFlag() (the two post-tick
  // bits) -- RobotLoop never calls this directly any more, which is
  // exactly SUC-004's acceptance bar.
  void setFlag(uint32_t bit, bool active);

  // ageOf -- state.<section>.sampleTime is already in the same [ms]
  // cycle-domain `now` uses (RobotLoop converts each device's own [us]
  // sampleTime() at the wheel/otos publish point) -- this is a plain
  // clamped subtract, not a unit conversion. `now` here is update()'s own
  // derived "at frame assembly" instant (cycleStart + cycleBusy), not the
  // bare wire `now` field -- see update()'s own doc comment.
  static uint32_t ageOf(uint32_t now, uint32_t sampleTime);  // [ms] [ms] -> [ms], clamped to 255

  Comms& comms_;

  Frame frame_;

  uint32_t flags_ = 0;  // every derived bit -- see setFlag()

  // Bounded ack ring (120) -- a plain circular buffer of the last
  // kAckRingDepth pushes, oldest-evicted-first, persisting across emit()
  // calls exactly like every other Frame field (Telemetry always carries
  // the last staged snapshot, not a diff -- app/DESIGN.md Sec 3's own
  // invariant, extended to this new field). ackRingHead_ indexes the
  // OLDEST valid entry; ackRingCount_ (0..kAckRingDepth) is how many of
  // ackRing_[]'s kAckRingDepth slots currently hold a real, pushed entry.
  //
  // 124-008 (issue §B4): storage is a plain `uint32_t` packed word
  // (`corr_id<<4 | err`), not msg::AckEntry (DELETED) -- pushAckRing() does
  // the packing.
  uint32_t ackRing_[kAckRingDepth]{};
  uint8_t ackRingHead_ = 0;
  uint8_t ackRingCount_ = 0;

  uint32_t seq_ = 0;  // increments once per SENT primary frame

  bool everEmittedPrimary_ = false;
  uint32_t lastPrimaryEmit_ = 0;  // [ms]
  uint32_t primaryEmitCount_ = 0;
};

}  // namespace App
