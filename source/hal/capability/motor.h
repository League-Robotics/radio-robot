// motor.h — the Motor faceplate: the four-verb contract (configure/apply/
// tick/state, plus capabilities) applied to one wheel-motor channel.
//
// Ported from the locked interface sketch in clasi/sprints/077-greenfield-
// faceplate-hal-drivetrain-and-dev-bench-system/issues/greenfield-rebuild-
// faceplate-hal-in-a-fresh-source-old-tree-parked.md, Step 3. Concrete
// leaves (NezhaMotor today; a future SimMotor/MockMotor could follow) supply
// the primitive setters/getters and the faceplate verbs configureDevice()/
// tick()/capabilities(); apply()/state() are the message plane, implemented
// ONCE here on top of those primitives so no leaf re-derives the oneof-
// unpack / capability-gating logic.
//
// Deviation from the issue's sketch: apply() returns bool here (true =
// accepted), not void. The issue's code sketch shows `void apply(...)`, but
// ticket 003's acceptance criteria requires that a command whose mode is
// absent from capabilities() (e.g. VOLT on Nezha) be rejected "before
// touching hardware" in a way ticket 5's DEV command layer can turn into
// "ERR unsupported" — a void return gives the caller no way to distinguish
// "accepted" from "silently ignored" without re-deriving it from state().
// Returning bool is the minimal change that preserves the rest of the
// locked shape (same parameters, same non-virtual/shared-once
// implementation, same capability-gating description) while making
// rejection observable to the command layer that needs it.
//
// Sprint 078 addition — the reversal-latch armor (motor-generic write-path
// policy) lives HERE, shared once across every leaf, the same way
// apply()/state() already are, per the sprint's binding placement mandate
// (see clasi/sprints/078-.../architecture-update.md, "The base/leaf split —
// exact contract"). A leaf supplies four new device-specific protected
// pure virtuals (writeRawDuty()/hardReset()/softRebaseline()/
// configureDevice()); the base builds the zero-dwell-reversal + output-
// deadband write gate, the standstill-guarded reset dispatch, and the
// motion-qualified wedge-suspect derivation on top of them. NezhaMotor is
// the first (and only, this sprint) leaf refit to this contract.
#pragma once

#include <math.h>
#include <stdint.h>

#include "messages/motor.h"

namespace Hal {

class Motor {
 public:
  virtual ~Motor() = default;
  virtual void begin() {}

  // Primitive setters — the real implementations, one per command mode.
  // setVelocity() sets the target the embedded PID chases in tick();
  // setDutyCycle() stages the duty the slew limiter walks toward; etc.
  // Wheel motors have one degree of freedom, so a directionless magnitude is
  // a speed; velocity here is the signed scalar along that axis.
  virtual void setDutyCycle(float dutyCycle) = 0;      // [-1, 1]
  virtual void setVoltage(float voltage) = 0;          // [V] Nezha: unsupported (capability)
  virtual void setVelocity(float velocity) = 0;        // [mm/s] signed
  virtual void setPosition(float position) = 0;        // [deg]
  virtual void setNeutral(msg::Neutral mode) = 0;
  virtual void setFeedforward(float feedforward) = 0;  // [V]

  // resetPosition() is concrete (sprint 078 armor policy): it only stages
  // resetPending_ = true; processResetIfPending() (called at the top of the
  // leaf's next tick()) decides hard-reset vs. soft-rebaseline based on
  // verified standstill (restTicks_) — see Design Rationale 5.
  void resetPosition();   // zero encoder (staged, not immediate)

  // Primitive getters — the real reads, served from what tick() last sampled.
  virtual float position() const = 0;      // [mm]
  virtual float velocity() const = 0;      // [mm/s] signed
  virtual float appliedDuty() const = 0;   // [-1, 1]
  virtual bool connected() const = 0;

  // wedged()/wedgeSuspect()/hardResetCount()/softResetCount() are concrete
  // (sprint 078 armor policy): they read base-tracked state maintained by
  // updateWedgeDetector()/processResetIfPending().
  bool wedged() const;            // raw, unconditional stuck-encoder latch (unchanged semantics — no target-gating/arming-grace)
  bool wedgeSuspect() const;      // wedged() qualified by |appliedDuty()| > outputDeadband_ for the same window
  uint32_t hardResetCount() const;   // cumulative hard (encoder-zeroing) resets
  uint32_t softResetCount() const;   // cumulative soft (non-zeroing) rebaselines

  // Faceplate verbs.
  // configure() is concrete (sprint 078 armor policy): caches
  // reversalDwell_/outputDeadband_ from the two new optional MotorConfig
  // fields (ship defaults when unset — Design Rationale 2), then delegates
  // to configureDevice() for the leaf's own device-specific config caching.
  void configure(const msg::MotorConfig& config);
  virtual void tick(uint32_t now) = 0;   // [ms] sample encoder; run the active mode
  virtual msg::MotorCapabilities capabilities() const = 0;

  // Message plane — implemented ONCE in this base class on top of the
  // primitives: apply() validates vs capabilities(), unpacks the oneof, and
  // calls the matching setter; state() assembles a MotorState from the
  // getters. See the file-level comment for the bool-return deviation from
  // the issue's void sketch.
  bool apply(const msg::MotorCommand& command);
  msg::MotorState state() const;

 protected:
  // --- Leaf-supplied device-specific primitives (sprint 078 armor split).
  // The base decides *whether/what* to write or reset; the leaf decides
  // *how* — see architecture-update.md, "The base/leaf split — exact
  // contract". ---
  virtual void writeRawDuty(float duty) = 0;   // [-1, 1] device write shaping (throttle/slew/write-on-change) + bus write
  virtual void hardReset() = 0;                // atomic, at-rest hardware re-prime burst
  virtual void softRebaseline() = 0;           // software-only rebaseline; issues no bus transaction
  virtual void configureDevice(const msg::MotorConfig& config) = 0;   // device-specific config caching (everything but the two armor fields)

  // --- Shared (non-virtual, inline) armor policy — called by the leaf's
  // tick() at the documented call-order points (see architecture-update.md
  // and NezhaMotor::tick() for the exact 5-step sequence). ---
  void armoredWrite(float duty, uint32_t now);   // [-1, 1] [ms] zero-dwell-reversal + output-deadband write gate
  void processResetIfPending(uint32_t now);      // [ms] standstill-guarded hard/soft reset dispatch
  void updateRestTracking();                     // feeds next tick's processResetIfPending()
  void updateWedgeDetector();                    // raw stuck-encoder latch + motion-qualified wedge-suspect derivation

  // --- Base-owned protected state (one instance per motor — each leaf
  // object embeds one Motor base subobject). ---
  float reversalDwell_ = 0.0f;            // [ms] cached from MotorConfig
  float outputDeadband_ = 0.0f;           // [-1,1] fraction, cached from MotorConfig
  bool dwelling_ = false;
  uint32_t dwellDeadline_ = 0;            // [ms]
  float lastRequestedDuty_ = 0.0f;        // [-1,1] last duty actually forwarded to writeRawDuty()

  bool resetPending_ = false;
  uint8_t restTicks_ = 0;                 // consecutive at-rest ticks observed
  uint32_t hardResetCount_ = 0;
  uint32_t softResetCount_ = 0;

  float wedgePrevPosition_ = 0.0f;        // [mm]
  bool wedgePrevValid_ = false;
  uint8_t stuckCount_ = 0;                // raw, unconditional (unchanged semantics)
  uint8_t movingStuckCount_ = 0;          // same test, gated by |appliedDuty()| > outputDeadband_
  bool wedgeLatched_ = false;
  bool wedgeSuspect_ = false;

 private:
  // Ship defaults substituted by configure() when MotorConfig's two new
  // optional armor fields are unset (architecture-update.md Design
  // Rationale 2 — optional, not the slew_rate-style zero-sentinel, because
  // an explicit 0 must remain a valid, distinct, meaningful configuration
  // for both fields).
  static constexpr float kDefaultReversalDwell = 100.0f;    // [ms]
  static constexpr float kDefaultOutputDeadband = 0.03f;    // [-1,1] fraction

  // Standstill-guard constants for updateRestTracking()/
  // processResetIfPending() — engineering starting guesses (Open Question 1
  // in architecture-update.md), NOT stakeholder-set values. Ticket 005's
  // friction-rig bench pass may find these too eager or too lazy; retuning
  // them there is in-scope, promoting them to MotorConfig fields is not.
  static constexpr float kRestVelocity = 5.0f;        // [mm/s] proposed starting guess
  static constexpr uint8_t kRestTicksRequired = 5;    // proposed starting guess

  // Consecutive-identical-reading threshold for the wedge latch — ported
  // unchanged from NezhaMotor's former file-local kWedgeThreshold constant
  // (moved here along with updateWedgeDetector(); same value, same
  // semantics, per 064-004 hardening — do not reintroduce target-gating or
  // arming-grace).
  static constexpr uint8_t kWedgeThreshold = 10;
};

// motorCommandAllowed() -- sprint 079 extraction (architecture-update.md
// Design Rationale 3): the capability-gate rule ("which MotorCapabilities
// bit gates which ControlKind") lives here ONCE so Motor::apply() and
// commands/dev_commands.cpp's pre-validation (staging into DevLoopState's
// outbox instead of calling apply() to discover rejection after the fact)
// can never drift apart. NEUTRAL/NONE (and any future default) are never
// gated -- every motor must be able to go neutral regardless of drive mode.
inline bool motorCommandAllowed(const msg::MotorCapabilities& caps,
                                msg::MotorCommand::ControlKind kind) {
  switch (kind) {
    case msg::MotorCommand::ControlKind::DUTY_CYCLE: return caps.duty_cycle;
    case msg::MotorCommand::ControlKind::VOLTAGE:    return caps.voltage;
    case msg::MotorCommand::ControlKind::VELOCITY:   return caps.velocity;
    case msg::MotorCommand::ControlKind::POSITION:   return caps.position;
    case msg::MotorCommand::ControlKind::NEUTRAL:
    case msg::MotorCommand::ControlKind::NONE:
    default: return true;   // never gated
  }
}

// --- resetPosition()/wedged()/wedgeSuspect()/hardResetCount()/
// softResetCount()/configure(): small concrete accessors and the config
// cache, defined inline here (same headers-only style as apply()/state()
// below — see Design Rationale 7: no capability/motor.cpp exists). ---

inline void Motor::resetPosition() { resetPending_ = true; }

inline bool Motor::wedged() const { return wedgeLatched_; }
inline bool Motor::wedgeSuspect() const { return wedgeSuspect_; }
inline uint32_t Motor::hardResetCount() const { return hardResetCount_; }
inline uint32_t Motor::softResetCount() const { return softResetCount_; }

inline void Motor::configure(const msg::MotorConfig& config) {
  reversalDwell_ = config.reversal_dwell.has ? config.reversal_dwell.val
                                              : kDefaultReversalDwell;
  outputDeadband_ = config.output_deadband.has ? config.output_deadband.val
                                                : kDefaultOutputDeadband;
  configureDevice(config);
}

// --- apply()/state(): the shared message plane, defined once here (inline
// so this header stays the only translation unit backing every Motor leaf;
// no capability/motor.cpp exists — see the sprint's Build acceptance
// criterion, which lists capability/ as headers-only). ---

inline bool Motor::apply(const msg::MotorCommand& command) {
  const msg::MotorCapabilities caps = capabilities();
  const msg::MotorCommand::ControlKind kind = command.get_control_kind();

  if (!motorCommandAllowed(caps, kind)) return false;

  switch (kind) {
    case msg::MotorCommand::ControlKind::DUTY_CYCLE:
      setDutyCycle(command.control.duty_cycle);
      break;
    case msg::MotorCommand::ControlKind::VOLTAGE:
      setVoltage(command.control.voltage);
      break;
    case msg::MotorCommand::ControlKind::VELOCITY:
      setVelocity(command.control.velocity);
      break;
    case msg::MotorCommand::ControlKind::POSITION:
      setPosition(command.control.position);
      break;
    case msg::MotorCommand::ControlKind::NEUTRAL:
      // Neutral is always accepted — every motor must be able to go
      // neutral regardless of which drive modes it otherwise supports.
      setNeutral(command.control.neutral);
      break;
    case msg::MotorCommand::ControlKind::NONE:
    default:
      // No control arm set — e.g. a command whose only payload is
      // reset_position (DEV M <n> RESET). Nothing to dispatch here; the
      // reset/feedforward side-channels below still apply.
      break;
  }

  // Feedforward and reset_position ride beside whichever arm (or lack of
  // one) was just processed — never gated by capabilities(), since they
  // are not control modes themselves.
  if (command.get_feedforward().has) {
    setFeedforward(command.get_feedforward().val);
  }
  if (command.get_reset_position().has && command.get_reset_position().val) {
    resetPosition();
  }

  return true;
}

inline msg::MotorState Motor::state() const {
  msg::MotorState s;
  const msg::MotorCapabilities caps = capabilities();

  s.connected = connected();
  s.applied.has = true;
  s.applied.val = appliedDuty();

  // Position/velocity/wedged(+ sprint 078's wedge_suspect/hard_reset_count/
  // soft_reset_count) only mean something for a motor with an encoder; a
  // future capability-less leaf would leave these unset rather than report
  // a fabricated zero.
  if (caps.has_encoder) {
    s.position.has = true;
    s.position.val = position();
    s.velocity.has = true;
    s.velocity.val = velocity();
    s.wedged.has = true;
    s.wedged.val = wedged();
    s.wedge_suspect.has = true;
    s.wedge_suspect.val = wedgeSuspect();
    s.hard_reset_count.has = true;
    s.hard_reset_count.val = hardResetCount();
    s.soft_reset_count.has = true;
    s.soft_reset_count.val = softResetCount();
  }

  return s;
}

// --- Armor policy (sprint 078) — zero-dwell reversal, output deadband,
// standstill-guarded resets, motion-qualified wedge reporting. See
// clasi/sprints/078-.../architecture-update.md for the full state diagram
// and rationale; this is that document's normative sequence implemented
// verbatim. ---

// armoredWrite() — the write-path gate every DUTY/VELOCITY/NEUTRAL mode
// tick funnels through. Stop (duty == 0) and sub-deadband duty are always
// immediate and unclamped, even mid-dwell (they cancel any dwell in
// progress). A commanded sign change (relative to lastRequestedDuty_) walks
// through: write 0 now, arm the dwell timer, suppress every non-zero write
// until now >= dwellDeadline_, then forward the new-direction duty as-is
// (the leaf's own slew cap ramps it from zero). reversalDwell_ == 0 is the
// explicit legacy/A-B configuration (Design Rationale 2) and skips the
// dwell transition entirely — a detected reversal falls straight through to
// an immediate write, reproducing sprint-077's shipped behavior.
inline void Motor::armoredWrite(float duty, uint32_t now) {
  if (duty == 0.0f || fabsf(duty) < outputDeadband_) {
    // Stop always wins: immediate, unclamped, cancels any dwell in flight.
    dwelling_ = false;
    lastRequestedDuty_ = 0.0f;
    writeRawDuty(0.0f);
    return;
  }

  if (dwelling_) {
    if (now < dwellDeadline_) {
      // Still holding at commanded-zero through the dwell window.
      lastRequestedDuty_ = 0.0f;
      writeRawDuty(0.0f);
      return;
    }
    // Dwell elapsed — proceed in the new direction below.
    dwelling_ = false;
  } else if (reversalDwell_ > 0.0f && lastRequestedDuty_ != 0.0f &&
             ((duty > 0.0f) != (lastRequestedDuty_ > 0.0f))) {
    // Commanded sign change relative to the last duty we actually forwarded
    // — write 0 now and arm the dwell; the new direction is withheld until
    // the dwell deadline (see the class-level state diagram in
    // architecture-update.md).
    dwelling_ = true;
    dwellDeadline_ = now + static_cast<uint32_t>(reversalDwell_);
    lastRequestedDuty_ = 0.0f;
    writeRawDuty(0.0f);
    return;
  }

  // Same-sign duty (or no prior direction to reverse from, or the dwell
  // just elapsed): forward as-is.
  lastRequestedDuty_ = duty;
  writeRawDuty(duty);
}

// processResetIfPending() — called first in the leaf's tick(), before this
// tick's encoder sample (restTicks_ reflects prior ticks' rest state, which
// is fine — "verified standstill" does not need this tick's not-yet-taken
// sample). Dispatches hardReset() only when restTicks_ has reached
// kRestTicksRequired consecutive at-rest ticks; otherwise performs an
// immediate softRebaseline() — never deferred (Design Rationale 5). The
// leaf's hardReset() body is unchanged, so this function is the one that
// increments hardResetCount_; softRebaseline() increments softResetCount_
// itself (ported from source_old's Motor::rebaselineSoft(), which does the
// same at its own call site).
inline void Motor::processResetIfPending(uint32_t now) {
  (void)now;   // explicit per the base/leaf contract's signature (no implicit tick state); unused today
  if (!resetPending_) return;
  resetPending_ = false;

  if (restTicks_ >= kRestTicksRequired) {
    hardReset();
    ++hardResetCount_;
  } else {
    softRebaseline();
  }
}

// updateRestTracking() — called last in the leaf's tick(), after this
// tick's mode dispatch (so lastRequestedDuty_ reflects whatever
// armoredWrite() just decided). Gates on the *commanded* value
// (lastRequestedDuty_), matching source_old's MotorController::
// computeAtRest() precedent — not appliedDuty(), which updateWedgeDetector()
// uses instead (Design Rationale 4).
inline void Motor::updateRestTracking() {
  bool atRest = (fabsf(velocity()) < kRestVelocity) && (lastRequestedDuty_ == 0.0f);
  if (atRest) {
    if (restTicks_ < 255) ++restTicks_;
  } else {
    restTicks_ = 0;
  }
}

// updateWedgeDetector() — the raw, unconditional stuck-encoder latch
// (stuckCount_/wedgeLatched_) is ported exactly as NezhaMotor's former
// per-leaf version: it counts consecutive identical position() reads with
// no gating by commanded target or arming grace (064-004 hardening — do NOT
// reintroduce those blind spots). wedgeSuspect_ is a second, independent
// derivation of the same identical-reads test, additionally gated on
// |appliedDuty()| > outputDeadband_ (the motor was actually being asked to
// move) — Design Rationale 4's second signal. Both counters reset whenever
// their own gating condition breaks, so resuming motion after a genuine
// stop never carries over stale "stuck while moving" state from before the
// motor went idle.
inline void Motor::updateWedgeDetector() {
  float pos = position();
  bool unchanged = wedgePrevValid_ && (pos == wedgePrevPosition_);
  bool moving = fabsf(appliedDuty()) > outputDeadband_;

  if (unchanged) {
    if (stuckCount_ < 255) ++stuckCount_;
  } else {
    stuckCount_ = 0;
    wedgeLatched_ = false;
  }

  if (unchanged && moving) {
    if (movingStuckCount_ < 255) ++movingStuckCount_;
  } else {
    movingStuckCount_ = 0;
    wedgeSuspect_ = false;
  }

  wedgePrevPosition_ = pos;
  wedgePrevValid_ = true;

  if (stuckCount_ >= kWedgeThreshold) wedgeLatched_ = true;
  if (movingStuckCount_ >= kWedgeThreshold) wedgeSuspect_ = true;
}

}  // namespace Hal
