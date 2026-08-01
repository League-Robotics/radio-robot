// telemetry.h -- App::Telemetry: the always-on outbound frame. Projects
// Types::RobotState into the primary msg::Telemetry frame (single ack slot
// + a unified flags bit-string) and emits it at a fixed cadence.
//
// One method, update(const Types::RobotState&), reads the state ONCE and
// stages the WHOLE next frame -- every wire field and every flag is
// derived here, nothing is set piecemeal by scattered callers. There is
// no secondary frame type or tie-break/alternation cadence machinery: one
// frame type, paced by one cadence gate. `state ⊇ wire` means there is no
// wire-visibility invariant obligating anything to fold into the primary
// frame that the host does not genuinely consume.
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

// --- flags bit layout ----------------------------------------------------
// The single place a reader decodes a bit against. Every bit below is
// derived INSIDE update() from a Types::RobotState field, except bits 15/16
// (kFlagFaultMoveTimeout/kFlagFaultShapingDisabled), which setLiveFlag()
// sets directly -- see that method's own doc comment for why those two
// cannot fold into update().
//
// Every live bit below is classified into EXACTLY ONE of three declared
// classes, under its own labeled section header. ERRATUM this taxonomy
// exists to prevent: a Freshness bit (kFlagLinePresent/kFlagColorPresent)
// was once treated as stability evidence by a report-on-change arm, which
// deadlocked on any robot with a connected line/color sensor (the flags
// word toggles every cycle by design on such a robot, so "wait for the
// flags word to hold still" never happens). A bit's CLASS is a property of
// what its VALUE MEANS across frames, not of its symbol name --
// kFlagEventConfigApplied (bit 12) is named "Event" but is classified
// STATE below; do not infer class from name alone.
//
// === State bits -- level, meaningful across frames (safe to compare
//     cycle-to-cycle; these are the only bits a change-detection/stability
//     check may ever look at) =============================================
//   bit 1  (kFlagOtosConnected)  -- live OTOS bus health.
//   bit 2  (kFlagActive)         -- motion in progress.
//   bit 3  (kFlagConnLeft)       -- left motor bus connectivity.
//   bit 4  (kFlagConnRight)      -- right motor bus connectivity.
//   bit 6  (kFlagFaultI2CSafetyNet) -- I2CBus `readyAt` clearance
//                                    safety-net trip
//                                    (Devices::I2CBus::
//                                    clearanceSafetyNetCount() > 0).
//                                    ERRATUM, measured on hardware: this is
//                                    a CONTINUOUSLY LIVE, monotonically
//                                    growing counter, NOT a boot-time
//                                    one-shot latch. Root cause:
//                                    Devices::Otos::readPositionVelocity()
//                                    (and its sibling register helpers)
//                                    issue a register-select write()
//                                    immediately followed by a read() on
//                                    the SAME device with no intervening
//                                    loop-scheduled gap, so
//                                    waitForClearance() trips on every
//                                    single Otos burst read,
//                                    unconditionally (the motor's own
//                                    split-phase request/collect, which
//                                    DOES cross a real scheduled gap,
//                                    contributes ZERO trips). A healthy
//                                    robot with the real OTOS present
//                                    therefore shows this bit set on
//                                    nearly every frame, idle AND driving,
//                                    by design -- do NOT read a steady 1
//                                    as evidence of a motor/loop-timing
//                                    defect. Because the derivation never
//                                    resets, it also cannot surface a
//                                    genuine FUTURE motor-side regression
//                                    once it has saturated to 1 (within
//                                    about a second of boot).
//   bit 7  (kFlagFaultWedgeLatch)   -- NezhaMotor/I2CBus wedge-latch
//                                    detected (Devices::MotorArmor::
//                                    wedged()).
//   bit 8  (kFlagFaultI2CNak)       -- I2C NAK/timeout. Declared, not yet
//                                    wired live (no per-transaction NAK
//                                    aggregate exists yet).
//   bit 9  (kFlagFaultCommsMalformed) -- malformed/undecodable inbound
//                                    frame (App::Comms::malformedCount() >
//                                    0).
//   bit 12 (kFlagEventConfigApplied) -- a ConfigDelta was applied.
//                                    Declared, not yet wired. Named "Event"
//                                    but classified STATE here -- see this
//                                    section's own header note.
//   bit 16 (kFlagFaultShapingDisabled) -- a MOVE is active AND the
//                                    planner's own shaper is not
//                                    configured (RobotLoop::
//                                    publishMoveResult(): planner_.active()
//                                    && !planner_.shaperConfigured()) --
//                                    the loud off-state for the silent-off
//                                    shaping/anticipation config boundary.
//                                    Set via setLiveFlag(), not update()
//                                    -- see that method's doc comment.
//   bit 17 (kFlagFaultPositionClamped) -- the position-rebaseline
//                                    policy's defensive fallback: a
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
//                                    positionClamped inside update().
//   bit 18 (kFlagFaultCommandsDropped) -- a well-formed inbound command
//                                    was dropped because App::Comms's
//                                    command ring was full. Distinct in
//                                    KIND from bit 9, not merely in count:
//                                    bit 9 is a wire/link problem (a line
//                                    arrived corrupt), this is firmware
//                                    backpressure (commands arrived faster
//                                    than one cycle's drain could route).
//                                    Derived from Types::RobotState::
//                                    Health::commandsDroppedCount inside
//                                    update(), exactly like bit 9.
//   bit 19 (kFlagFaultWheelFrozenLeft) -- the LEFT wheel was commanded a
//                                    nonzero duty for N consecutive cycles
//                                    with NO encoder change --
//                                    Devices::MotorArmor::wedgeSuspect(),
//                                    the GATED (motion-qualified)
//                                    derivation, NOT the raw
//                                    wedged()/kFlagFaultWedgeLatch (bit 7)
//                                    this bit is deliberately independent
//                                    of. wedged() fires on ANY stuck
//                                    encoder reading, including a healthy
//                                    robot sitting parked at rest (it
//                                    reads ~always-set while idle) --
//                                    publishing THAT as a fault would cry
//                                    wolf on every idle frame. wedgeSuspect()
//                                    additionally requires |appliedDuty()|
//                                    above the motion-threshold deadband,
//                                    i.e. "asked to move, not moving" --
//                                    the actual "frozen wheel" signal a
//                                    driving robot cares about. Derived
//                                    from Types::RobotState::Health::
//                                    wheelFrozenLeft inside update(),
//                                    exactly like bit 7. Also the
//                                    regression guard for ESTOP
//                                    unlosability: a duty write silently
//                                    lost again shows up here within the
//                                    wedge threshold instead of requiring
//                                    another incident to notice, and is
//                                    the required guard against the
//                                    adaptive gain learner ever training
//                                    on a stalled wheel's bogus zero
//                                    speed.
//   bit 20 (kFlagFaultWheelFrozenRight) -- same as bit 19, RIGHT wheel
//                                    (Devices::MotorArmor::wedgeSuspect()
//                                    on motorR_, Types::RobotState::
//                                    Health::wheelFrozenRight).
//
// === Freshness bits -- valid-THIS-FRAME qualifiers for a payload field;
//     toggle BY DESIGN every cycle they are not fresh; carry NO
//     cross-frame information. These bits may NEVER participate in any
//     change-detection or stability logic -- comparing one of these across
//     frames, or gating a "hold still" wait on one, is EXACTLY the defect
//     that made the deleted report-on-change arm deadlock on a robot with
//     a connected line/color sensor (see this section's own header note
//     above). If a bit's own name ends in "Present", assume Freshness
//     unless it is listed under State above (kFlagOtosPresent, bit 0, is
//     the one bit in this file where the name and the class agree) =========
//   bit 0  (kFlagOtosPresent)    -- OtosReading fresh THIS frame (chip
//                                    detected AND this cycle's burst read
//                                    actually refreshed the cached pose --
//                                    see odometry.h's applyOtosSample()
//                                    doc comment). Frame.otos is valid iff
//                                    this bit is set.
//   bit 13 (kFlagLinePresent)       -- line word fresh THIS frame.
//   bit 14 (kFlagColorPresent)      -- color word fresh THIS frame.
//
// === Event bits -- true for the TRANSITION cycle only (the cycle the
//     condition first becomes true), never a level -- reading one after
//     its own transition cycle has passed tells you nothing =============
//   bit 10 (kFlagEventDeadmanExpired) -- Deadman staleness timer expired
//                                    (App::Deadman::expired()), the
//                                    transition cycle only.
//   bit 15 (kFlagFaultMoveTimeout)  -- MOVE timeout backstop fired, rides
//                                    the completing frame. Set via
//                                    setLiveFlag(), not update() -- see
//                                    that method's doc comment.
//
// === Reserved (never set by any code path) ================================
//   bit 5  -- RESERVED. Ring membership already means "really acked"; do
//                                    not reuse this for a scalar
//                                    ack-freshness flag.
//   bit 11 -- RESERVED. A one-shot, latched-forever boot-ready bit is
//                                    unobservable by construction (boot
//                                    ends before any frame could carry the
//                                    edge) -- the `READY` cleartext line
//                                    is the right vehicle for that signal.
//   bits 21-31 -- reserved for future use.
constexpr uint32_t kFlagOtosPresent = 1u << 0;
constexpr uint32_t kFlagOtosConnected = 1u << 1;
constexpr uint32_t kFlagActive = 1u << 2;
constexpr uint32_t kFlagConnLeft = 1u << 3;
constexpr uint32_t kFlagConnRight = 1u << 4;
// bit 5 -- RESERVED -- see above.
constexpr uint32_t kFlagFaultI2CSafetyNet = 1u << 6;
constexpr uint32_t kFlagFaultWedgeLatch = 1u << 7;
constexpr uint32_t kFlagFaultI2CNak = 1u << 8;
constexpr uint32_t kFlagFaultCommsMalformed = 1u << 9;
constexpr uint32_t kFlagEventDeadmanExpired = 1u << 10;
// bit 11 -- RESERVED -- see above.
constexpr uint32_t kFlagEventConfigApplied = 1u << 12;
constexpr uint32_t kFlagLinePresent = 1u << 13;
constexpr uint32_t kFlagColorPresent = 1u << 14;
constexpr uint32_t kFlagFaultMoveTimeout = 1u << 15;
constexpr uint32_t kFlagFaultShapingDisabled = 1u << 16;
constexpr uint32_t kFlagFaultPositionClamped = 1u << 17;
constexpr uint32_t kFlagFaultCommandsDropped = 1u << 18;
constexpr uint32_t kFlagFaultWheelFrozenLeft = 1u << 19;
constexpr uint32_t kFlagFaultWheelFrozenRight = 1u << 20;

// Primary cadence target: primary period == cycle period -- the frame is
// emitted every loop iteration. Matches robot_loop.cpp's genuine
// 40ms/~25Hz kCycle. Callers pace against this and measure their own real
// number; emit() does not need to hit it exactly.
constexpr uint32_t kPrimaryPeriod = 40;  // [ms] ~25 Hz, matches robot_loop.cpp's kCycle

// Ack ring depth. MUST match telemetry.proto's Telemetry.acks (max_count)
// exactly -- checked by a static_assert against the generated
// msg::Telemetry::acks_[] array width in telemetry.cpp, so a future edit
// to either side without the other fails the build rather than silently
// truncating the ring.
//
// With App::Comms's command ring, RobotLoop drains up to
// App::kCmdRingDepth commands in a SINGLE cycle and acks each one, so a
// burst that fits the command ring must also fit here or it executes
// unobservably -- a host chaining on completion acks would hang on a
// command that really did run. Kept EQUAL to App::kCmdRingDepth
// (comms.h); change the two together.
constexpr uint8_t kAckRingDepth = 12;

// How many emitted frames a freshly-pushed ack forces before it is treated as
// delivered. See Telemetry::ackSends_ for why this is >1.
constexpr uint8_t kAckRepeats = 3;

// TlmMode -- the three-state, host-controllable emit policy. A single
// member (Telemetry::mode_) with a single writer (the TLM: command
// handler, via setMode() below); resets to kAuto at construction every
// boot -- it is never persisted. Namespace scope (not a nested class
// type) so comms.cpp (the STATUS `tlm=` field) and its wire parsing can
// name it without reaching into Telemetry's own scope.
enum class TlmMode : uint8_t { kOff, kAuto, kOn };

// How long an activity window stays open after the last thing that opened
// or refreshed it (kFlagActive, or -- only while already open -- nonzero
// staged wheel velocity). This is the ONE tunable in the emit policy,
// sized to cover the bench-observed ~1.2s post-STOP settle with margin.
constexpr uint32_t kCoastHoldoff = 2000;  // [ms]

class Telemetry {
 public:
  // Wire-shaped staging area, filled WHOLE by update() every call -- no
  // caller-visible setFrame()/setFlag(). update() derives flags straight
  // from the Types::RobotState argument it is handed, so this struct is
  // pure wire-shaped data with no flag-derivation inputs of its own.
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

  // comms -- primary-frame send path (Comms::sendReply()). Comms already
  // owns both transports internally, so Telemetry holds no Transport&
  // references of its own and never reaches them directly.
  explicit Telemetry(Comms& comms);

  // update -- THE one-line RobotState projection. Reads `state` ONCE and
  // stages the WHOLE next frame: every wire field (scaled
  // position/velocity/pose/twist/otos conversion, packed line/color words,
  // loop-timing cycleBusy/cyclePeriod) and every flag, all derived from
  // `state` here. `age` fields (EncoderReading.age/OtosReading.age) are
  // computed as `now - state.<section>.sampleTime`, clamped to 255 --
  // `now` here is `state.time.cycleStart + state.time.cycleBusy`
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
  // grep-enforceable contract. Does not itself send anything.
  // Does NOT touch kFlagFaultMoveTimeout/kFlagFaultShapingDisabled, or any
  // bit setLiveFlag() below owns -- update() only ever OR/AND-NOT's the
  // bits it derives from `state`, so a bit it does not own survives an
  // update() call untouched (see setLiveFlag()'s own doc comment for why
  // that is exactly the behavior two of those bits need).
  void update(const Types::RobotState& state);

  // setLiveFlag -- the ONE narrow, deliberately-NOT-named-setFlag escape
  // hatch from update()'s single-assembly-point contract, for the bits
  // whose defining condition genuinely cannot be known at update() time:
  //   - kFlagFaultMoveTimeout/kFlagFaultShapingDisabled -- both depend on
  //     Motion::Planner::tick()'s own per-cycle outcome (its
  //     Motion::TickResult return value plus planner_.shaperConfigured()),
  //     which is not known yet at update()/emit() time: tick() must stay
  //     positioned AFTER update()/emit() every cycle (protocol-v4 §7.2 -- a
  //     completion ack staged by tick() must not be visible before the
  //     NEXT cycle's own emit() call, and the same "rides the next frame"
  //     timing applies to these two fault bits, bench-verified by
  //     app_robot_loop_harness.cpp: both bits must already read live via
  //     flags() by the time cycle() returns on the exact cycle tick()
  //     ends/toggles them, well before that value would otherwise reach
  //     update()'s next call). RobotLoop::cycle() calls this (via
  //     publishMoveResult()) immediately after planner_.tick().
  // Mechanically identical to the private setFlag() update() uses
  // internally (a level-set OR/AND-NOT bit mutation) -- only the name and
  // caller differ, so `grep setFlag src/firm/app/robot_loop.cpp` finds
  // nothing.
  void setLiveFlag(uint32_t bit, bool active);

  uint32_t flags() const { return flags_; }

  // setMode/mode -- the TLM three-mode policy surface. The ONE writer is
  // the `TLM:` command handler (RobotLoop::cycle() -> tlm_.setMode(...))
  // -- Telemetry itself never parses wire text. No persistence: mode_
  // resets to kAuto at construction, so a power cycle always forgets a
  // prior session's TLM:ON/TLM:OFF.
  void setMode(TlmMode mode) { mode_ = mode; }
  TlmMode mode() const { return mode_; }

  // applyAction -- absorbs the TLM command-surface's mode-change switch
  // AND the "should THIS cycle's emit() be forced" answer (a bare
  // TLM/TLM:NOW line -- action == kFrame -- forces one frame NOW, past the
  // mode-gated unsolicited check, since "nothing is happening" is exactly
  // the state someone asking is trying to observe). kSetOff/kSetAuto/
  // kSetOn call setMode(); every other action (kNone/kFrame/kUnrecognized)
  // leaves mode_ untouched.
  //
  // Dependency-direction choice: Telemetry takes Comms::TlmAction BY VALUE
  // here rather than (a) the enum moving to a shared/telemetry-owned
  // header, or (b) Telemetry exposing three mode setters + requestFrame()
  // for Comms/RobotLoop to call individually. comms.h is already an
  // unconditional #include of this header (see the file header's own
  // "Send path" note -- Telemetry holds a Comms&), so accepting
  // Comms::TlmAction as a parameter type adds no NEW edge to the
  // dependency graph, only a second use of the edge that already exists;
  // option (b) is rejected because it would re-scatter the switch's arms
  // across two call sites (RobotLoop choosing which setter to call would
  // put another module's policy job in the loop) for no offsetting
  // benefit.
  //
  // Returns the force-frame answer rather than latching a pending request
  // internally and dropping emit()'s own `force` parameter: emit()'s
  // existing force semantics -- still gated behind primaryDue(), covered
  // by this file's own extensive, pre-existing unit coverage exercising
  // emit(now, force) directly -- stay exactly as they are. Only WHERE the
  // "should this cycle be forced" decision gets COMPUTED moves, from
  // RobotLoop's own inline ternary to here; emit()'s external contract is
  // unchanged.
  //
  // Call once per cycle, immediately after comms_.takeTlmAction() -- the
  // returned bool is the very next emit() call's own `force` argument.
  bool applyAction(Comms::TlmAction action);

  // ack -- pushes to the bounded ack ring (see kAckRingDepth's own comment
  // above and telemetry.proto's Telemetry.acks doc comment for the
  // rationale). errCode == 0 means OK; nonzero is the
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
  // Comms::sendReply()) -- never sleeps, never touches the I2C bus. There
  // is only one frame type to pace, so `primaryDue()` is a plain "due
  // since last send" gate.
  //
  // emit -- the whole three-mode policy, and nothing more: a frame goes
  // out only for one of three reasons, `force` (a bare
  // TLM/TLM:NOW request, or RobotLoop::boot()'s own per-probe forced call),
  // `unsolicited` (mode-dependent: never in kOff, the activity window in
  // kAuto, every cadence tick in kOn), or pendingAckDeliveries() (an
  // undelivered ack, honored in EVERY mode -- protocol v5 has no separate
  // ack message, so the telemetry frame is the ack's only vehicle). No
  // fourth reason: no flag-change push, no arming state, no boot-completion
  // signal into this class. A freshly constructed Telemetry behaves
  // identically to one that has run for an hour.
  void emit(uint32_t now, bool force = false);

  // Measurement/test seam -- lets a HOST_BUILD test report the realized
  // cadence without parsing a FakeTransport's send log.
  uint32_t primaryEmitCount() const { return primaryEmitCount_; }
  uint32_t lastPrimaryEmit() const { return lastPrimaryEmit_; }  // [ms]

 private:
  bool primaryDue(uint32_t now) const;
  // pendingAckDeliveries -- true while any ack-ring entry has not yet been
  // carried kAckRepeats times. Honored in EVERY mode (emit()'s own call
  // site), never gated on mode_ or activity:
  // kOff suppressing ack frames would strand the outcome of any command
  // issued to a parked robot, the "acked but nothing happened" failure
  // class this class exists to prevent.
  bool pendingAckDeliveries() const;
  void emitPrimary(uint32_t now);
  void pushAckRing(uint32_t corrId, uint32_t errCode);

  // setFlag -- private: the only callers are update() (the state-derived
  // bits) and setLiveFlag() (the two post-tick bits) -- RobotLoop never
  // calls this directly.
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

  // Bounded ack ring -- a plain circular buffer of the last kAckRingDepth
  // pushes, oldest-evicted-first, persisting across emit() calls exactly
  // like every other Frame field (Telemetry always carries the last
  // staged snapshot, not a diff -- app/DESIGN.md Sec 3's own invariant,
  // extended to this field). ackRingHead_ indexes the OLDEST valid entry;
  // ackRingCount_ (0..kAckRingDepth) is how many of ackRing_[]'s
  // kAckRingDepth slots currently hold a real, pushed entry.
  //
  // Storage is a plain `uint32_t` packed word (`corr_id<<4 | err`) --
  // pushAckRing() does the packing.
  uint32_t ackRing_[kAckRingDepth]{};
  uint8_t ackRingHead_ = 0;
  uint8_t ackRingCount_ = 0;

  // Per-entry delivery count, parallel to ackRing_ and sharing its indexing.
  // An entry pushed by ack() starts at 0 and increments on every emitted
  // frame that carries it; once it reaches kAckRepeats it stops forcing
  // frames. That is what lets telemetry go SILENT at rest without losing
  // acks: a command issued to a parked robot still gets kAckRepeats frames
  // carrying its outcome, then the link goes quiet again.
  //
  // kAckRepeats > 1 because the wire is lossy: measured residual
  // physical-layer corruption rate that CRC catches but cannot repair
  // means a single carrying frame can vanish. Three gives redundancy
  // without meaningfully extending the quiet-after-command window
  // (3 x kPrimaryPeriod = 120 ms).
  uint8_t ackSends_[kAckRingDepth]{};

  // TlmMode policy state -- this IS the emit policy; there is no other
  // lifecycle beyond it. mode_ resets to kAuto by construction every
  // boot; ONE writer (setMode(), the TLM: handler).
  TlmMode mode_ = TlmMode::kAuto;

  // everMoved_ -- latches true the first time kFlagActive is seen this
  // power cycle, never cleared. This is what makes power-on silence
  // unconditional: before the first commanded motion, wheel velocity (a
  // bogus first-sample read, or a hand-spun wheel on the stand) can never
  // open the activity window. Belt-and-suspenders with
  // Devices::Motor::velocity()'s own two-sample floor, not a substitute
  // for it.
  bool everMoved_ = false;

  // lastActivity_ -- refreshed to `now` (update()'s own `state.time.
  // cycleStart`, the SAME clock domain emit()'s `now` parameter lives in)
  // whenever kFlagActive is set, and -- ONLY while the window is already
  // open -- whenever either wheel's staged velocity (frame_.encLeft/
  // encRight.velocity, the value about to go out on the wire) is nonzero.
  // That window-open precondition on the velocity refresh is deliberate:
  // coasting wheels keep an ALREADY-OPEN window alive (so a STOP's
  // deceleration tail keeps streaming), but wheels alone can never OPEN a
  // CLOSED window (so a hand-spun wheel with everMoved_ still false, or a
  // wheel reading arriving kCoastHoldoff after the last real activity,
  // wakes nothing).
  uint32_t lastActivity_ = 0;  // [ms]

  uint32_t seq_ = 0;  // increments once per SENT primary frame

  bool everEmittedPrimary_ = false;
  uint32_t lastPrimaryEmit_ = 0;  // [ms]
  uint32_t primaryEmitCount_ = 0;
};

}  // namespace App
