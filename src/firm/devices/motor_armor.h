// motor_armor.h — Devices::MotorArmor: the shared write-gate/reset/wedge
// armor policy shared by every Devices motor leaf: reversal dwell, output
// deadband, standstill-guarded resets, wedge detector.
//
// Deliberately narrower than a full motor abstraction: no message plane
// (apply()/state()/capabilities()/msg::MotorCommand — msg:: is unreachable
// under the isolation invariant; the loop is the Devices-native replacement
// for that surface, not this leaf base) and no acceleration tracking or
// active()/encGlitchCount()/sampleTime() virtuals (leaf-specific
// bookkeeping, not shared armor policy — Devices::NezhaMotor keeps its own).
//
// Leaf contract: a concrete leaf (Devices::NezhaMotor today) derives from
// MotorArmor, supplies the three device-specific protected primitives below
// (writeRawDuty()/hardReset()/softRebaseline()) plus its own position()/
// velocity()/appliedDuty() getters, calls configureArmor() once from its own
// config-caching path, and calls the four protected armor steps
// (processResetIfPending()/updateWedgeDetector()/armoredWrite()/
// updateRestTracking()) from its own tick() in that order (nezha_motor.cpp
// documents the full 5-step contract).
//
// Design/rationale: DESIGN.md.
#pragma once

#include <cmath>
#include <cstdint>

#include "devices/device_config.h"

namespace Devices {

class MotorArmor {
 public:
  virtual ~MotorArmor() = default;

  // Caches the two armor tuning fields from a MotorConfig, substituting
  // ship defaults when unset (Opt<T>::has == false) — ported from
  // Hal::Motor::configure()'s armor half (device_config.h's Opt<T> is the
  // Devices-local counterpart of the optional-field mechanism that
  // motivated it — see that file's own Opt<T> comment).
  void configureArmor(const MotorConfig& config);

  // resetPosition() — stages resetPending_ = true; processResetIfPending()
  // (called at the top of the leaf's next tick()) decides hard-reset vs.
  // soft-rebaseline based on verified standstill (restTicks_).
  void resetPosition();   // zero encoder (staged, not immediate)

  bool wedged() const;            // raw, unconditional stuck-encoder latch (unchanged semantics — no target-gating/arming-grace)
  bool wedgeSuspect() const;      // wedged() qualified by |appliedDuty()| > outputDeadband_ for the same window
  uint32_t hardResetCount() const;   // cumulative hard (encoder-zeroing) resets
  uint32_t softResetCount() const;   // cumulative soft (non-zeroing) rebaselines

  // --- Leaf-supplied getters (public pure virtuals — callers outside the
  // leaf, e.g. a test harness or the loop, read these directly; the armor
  // internals below also read them). ---
  virtual float position() const = 0;      // [mm]
  virtual float velocity() const = 0;      // [mm/s] signed
  virtual float appliedDuty() const = 0;   // [-1, 1]

 protected:
  // --- Leaf-supplied device-specific primitives. The base decides
  // *whether/what* to write or reset; the leaf decides *how*. ---
  virtual void writeRawDuty(float duty) = 0;   // [-1, 1] device write shaping (throttle/slew/write-on-change) + bus write
  virtual void hardReset() = 0;                // atomic, at-rest hardware re-prime burst
  virtual void softRebaseline() = 0;           // software-only rebaseline; issues no bus transaction

  // --- Shared (non-virtual, inline) armor policy — called by the leaf's
  // tick() at the documented call-order points. ---
  void armoredWrite(float duty, uint32_t now);   // [-1, 1] [ms] zero-dwell-reversal + output-deadband write gate
  void processResetIfPending(uint32_t now);      // [ms] standstill-guarded hard/soft reset dispatch
  void updateRestTracking();                     // feeds next tick's processResetIfPending()
  void updateWedgeDetector();                    // raw stuck-encoder latch + motion-qualified wedge-suspect derivation

  // --- Base-owned protected state (one instance per motor — each leaf
  // object embeds one MotorArmor base subobject). ---
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
  // Ship defaults substituted by configureArmor() when MotorConfig's two
  // armor fields are unset. Optional, not a zero-sentinel, because an
  // explicit 0 must remain a valid, distinct, meaningful configuration for
  // both fields.
  static constexpr float kDefaultReversalDwell = 100.0f;    // [ms]
  static constexpr float kDefaultOutputDeadband = 0.03f;    // [-1,1] fraction

  // Standstill-guard constants for updateRestTracking()/
  // processResetIfPending() — engineering starting guesses, a bench-tuning
  // question, not a rename target.
  static constexpr float kRestVelocity = 5.0f;        // [mm/s] proposed starting guess
  static constexpr uint8_t kRestTicksRequired = 5;    // proposed starting guess

  // Consecutive-identical-reading threshold for the wedge latch — do not
  // reintroduce target-gating or arming-grace here (see updateWedgeDetector()
  // below).
  static constexpr uint8_t kWedgeThreshold = 10;
};

// --- resetPosition()/wedged()/wedgeSuspect()/hardResetCount()/
// softResetCount()/configureArmor(): small concrete accessors and the
// config cache, defined inline here (headers-only). ---

inline void MotorArmor::resetPosition() { resetPending_ = true; }

inline bool MotorArmor::wedged() const { return wedgeLatched_; }
inline bool MotorArmor::wedgeSuspect() const { return wedgeSuspect_; }
inline uint32_t MotorArmor::hardResetCount() const { return hardResetCount_; }
inline uint32_t MotorArmor::softResetCount() const { return softResetCount_; }

inline void MotorArmor::configureArmor(const MotorConfig& config) {
  reversalDwell_ = config.reversalDwell.has ? config.reversalDwell.val
                                             : kDefaultReversalDwell;
  outputDeadband_ = config.outputDeadband.has ? config.outputDeadband.val
                                               : kDefaultOutputDeadband;
}

// --- Armor policy — zero-dwell reversal, output deadband, standstill-
// guarded resets, motion-qualified wedge reporting. ---

// armoredWrite() — the write-path gate every DUTY/VELOCITY/NEUTRAL mode
// tick funnels through. Stop (duty == 0) and sub-deadband duty are always
// immediate and unclamped, even mid-dwell (they cancel any dwell in
// progress). A commanded sign change (relative to lastRequestedDuty_) walks
// through: write 0 now, arm the dwell timer, suppress every non-zero write
// until now >= dwellDeadline_, then forward the new-direction duty as-is
// (the leaf's own slew cap ramps it from zero). reversalDwell_ == 0 is the
// explicit legacy/A-B configuration and skips the dwell transition entirely
// — a detected reversal falls straight through to an immediate write.
inline void MotorArmor::armoredWrite(float duty, uint32_t now) {
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
    // the dwell deadline.
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
// immediate softRebaseline() — never deferred. The leaf's hardReset() body
// is unchanged, so this function is the one that increments
// hardResetCount_; softRebaseline() increments softResetCount_ itself.
inline void MotorArmor::processResetIfPending(uint32_t now) {
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
// (lastRequestedDuty_), not appliedDuty() (which updateWedgeDetector() uses
// instead).
inline void MotorArmor::updateRestTracking() {
  bool atRest = (fabsf(velocity()) < kRestVelocity) && (lastRequestedDuty_ == 0.0f);
  if (atRest) {
    if (restTicks_ < 255) ++restTicks_;
  } else {
    restTicks_ = 0;
  }
}

// updateWedgeDetector() — the raw, unconditional stuck-encoder latch
// (stuckCount_/wedgeLatched_) counts consecutive identical position() reads
// with no gating by commanded target or arming grace — do NOT reintroduce
// those blind spots. wedgeSuspect_ is a second,
// independent derivation of the same identical-reads test, additionally
// gated on |appliedDuty()| > outputDeadband_ (the motor was actually being
// asked to move). Both counters reset whenever their own gating condition
// breaks, so resuming motion after a genuine stop never carries over stale
// "stuck while moving" state from before the motor went idle.
inline void MotorArmor::updateWedgeDetector() {
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

}  // namespace Devices
