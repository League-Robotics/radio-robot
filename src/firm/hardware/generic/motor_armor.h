// motor_armor.h — Hardware::MotorArmor: a composing Hal::Motor decorator carrying
// the OBSERVATION/RECOVERY policies — the wedge detector and the
// standstill-guarded reset dispatch.
//
// A pure DECORATOR over Hal::Motor (motor.h): construct a motor, hand
// it to MotorArmor, hand the armor to whatever wants a Hal::Motor. Don't want
// the armor? Hand the motor over directly. The sim does exactly that
// (src/firm/platform/host/sim_harness.h — bare motors); the ARM build wraps
// (src/firm/main.cpp).
//
// The write gate (reversal dwell + output deadband) is NOT here — it is
// Nezha-brick wedge PROTECTION, write shaping in the same family as the
// slew cap and write throttle, so it lives in Hardware::NezhaMotor's own
// write path (nezha_motor.cpp's writeShapedDuty(); configured by the same
// Hal::MotorConfig reversalDwell/outputDeadband fields, 0/0 = off).
//
// Policies kept here:
//   - Wedge detector: the raw, unconditional stuck-encoder latch
//     (consecutive identical position() reads — no target-gating or
//     arming-grace, see updateWedgeDetector()) plus the motion-qualified
//     wedgeSuspect() derivation (same test, gated on |appliedDuty()| above
//     the motion threshold). Feeds RobotLoop's kFaultWedgeLatch bit.
//   - Standstill-guarded resets: resetPosition() stages; the next tick()
//     dispatches inner.resetPosition() (hard, bus-touching) only after
//     kRestTicksRequired consecutive verified-at-rest ticks, and
//     inner.rebaseline() (software-only) otherwise — a hard reset's atomic
//     read burst mid-motion is itself a wedge trigger (see
//     docs/knowledge/2026-07-04-encoder-wedge.md).
//
// tick() ordering: reset dispatch runs BEFORE inner.tick() (matching the
// old step-1 placement — restTicks_ reflects prior ticks' rest state,
// which is the point of "verified standstill"); the wedge detector and
// rest tracking run AFTER inner.tick(), observing this tick's fresh
// position()/appliedDuty().
//
// History/rationale for every policy: DESIGN.md and
// docs/knowledge/2026-07-04-encoder-wedge.md.
#pragma once

#include <cmath>
#include <cstdint>

#include "hal/device_config.h"
#include "hal/motor.h"

namespace Hardware {

class MotorArmor : public Hal::Motor {
 public:
  explicit MotorArmor(Hal::Motor& inner) : inner_(inner) {}

  // reconfigure — forwards the whole config to the wrapped inner_ motor
  // FIRST (see motor.h), then, only if the inner motor actually accepted
  // it, refreshes this armor's own derived motionThreshold_ cache from the
  // SAME config's required outputDeadband field — the SAME value the inner
  // NezhaMotor's write shaping uses for its deadband, read here for the
  // independent "was the motor actually being asked to move" gate. Only
  // updating motionThreshold_ when applied is true means an armor whose
  // inner motor refused the new config never silently drifts its own
  // wedge-detection threshold away from what the motor actually uses.
  bool reconfigure(const Hal::MotorConfig& config) override {
    bool applied = inner_.reconfigure(config);
    if (applied) {
      motionThreshold_ = config.outputDeadband;
    }
    return applied;
  }

  // --- Hal::Motor: command/lifecycle forwarding ---
  void begin() override { inner_.begin(); }
  void requestSample() override { inner_.requestSample(); }
  void setDuty(float duty) override { inner_.setDuty(duty); }
  void setNeutral(Hal::Neutral mode) override { inner_.setNeutral(mode); }
  // Straight through, never policed: the armor's job is to constrain
  // COMMANDED motion, and an emergency zero is the one command it must
  // never delay, dwell-gate or veto.
  void emergencyStop() override { inner_.emergencyStop(); }
  // applyTravelCalib() is DELETED (counts-native leaf, 2026-08-15 --
  // motor.h's own header) -- Hal::Motor no longer declares it, so this
  // override has nothing left to override. Removed here as a required
  // compile fix for the exploratory-kernel rewrite (the leaf/base-class
  // side of this change had already landed; this decorator's own mirror
  // of the deleted virtual had not) -- see this rewrite's own report for
  // why a file the task brief said not to touch still needed this one
  // line gone.

  void tick(uint64_t nowUs) override {
    processResetIfPending();
    inner_.tick(nowUs);
    updateWedgeDetector();
    updateRestTracking();
  }

  // --- Hal::Motor: getters ---
  float position() const override { return inner_.position(); }
  float velocity() const override { return inner_.velocity(); }
  float appliedDuty() const override { return inner_.appliedDuty(); }
  bool connected() const override { return inner_.connected(); }
  uint64_t sampleTime() const override { return inner_.sampleTime(); }  // [us] pure passthrough -- no armor-side state of its own

  // --- Hal::Motor: resets — the armor's whole reason to intercept ---
  // Stages; the next tick() dispatches hard-at-rest / rebaseline-otherwise.
  void resetPosition() override { resetPending_ = true; }
  void rebaseline() override { inner_.rebaseline(); }

  // --- Hal::Motor: observability ---
  bool wedged() const override { return wedgeLatched_ || forcedWedge_; }
  // DBG-injected wedge (system test): OR-ed into wedged() above so every
  // consumer -- health publication, flag bit 7, Drive's wedge handling --
  // sees an induced wedge exactly as it would a real one. The real
  // detector's own latch state is untouched; clearing the force restores
  // the true state.
  void setForcedWedge(bool on) override { forcedWedge_ = on; }
  bool wedgeSuspect() const override { return wedgeSuspect_; }

  // --- Armor-specific accessors (beyond the Hal::Motor faceplate) ---
  uint32_t hardResetCount() const { return hardResetCount_; }
  uint32_t softResetCount() const { return softResetCount_; }

 private:
  // Standstill-guarded reset dispatch — see the file header for placement.
  void processResetIfPending() {
    if (!resetPending_) return;
    resetPending_ = false;
    if (restTicks_ >= kRestTicksRequired) {
      inner_.resetPosition();   // hard — bare NezhaMotor acts immediately
      ++hardResetCount_;
    } else {
      inner_.rebaseline();      // soft — never a bus transaction mid-motion
      ++softResetCount_;
    }
  }

  // Rest tracking — feeds the NEXT tick's processResetIfPending(). At rest
  // == not measurably moving AND nothing written to the H-bridge
  // (appliedDuty() reflects the last successfully written percent — the
  // only motion signal a decorator can observe from outside the inner
  // motor).
  void updateRestTracking() {
    bool atRest = (fabsf(inner_.velocity()) < kRestVelocity) &&
                  (inner_.appliedDuty() == 0.0f);
    if (atRest) {
      if (restTicks_ < 255) ++restTicks_;
    } else {
      restTicks_ = 0;
    }
  }

  // Wedge detector — the raw, unconditional stuck-encoder latch counts
  // consecutive identical position() reads with no gating by commanded
  // target or arming grace — do NOT reintroduce those blind spots (they
  // hid the boundary-latch flavor; see the knowledge doc). wedgeSuspect_
  // is the same test additionally gated on |appliedDuty()| >
  // motionThreshold_ (the motor was actually being asked to move). Both
  // counters reset whenever their own gating condition breaks.
  void updateWedgeDetector() {
    float pos = inner_.position();
    bool unchanged = wedgePrevValid_ && (pos == wedgePrevPosition_);
    bool moving = fabsf(inner_.appliedDuty()) > motionThreshold_;

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

  Hal::Motor& inner_;

  float motionThreshold_ = kDefaultMotionThreshold;   // [-1,1] duty fraction

  bool resetPending_ = false;
  uint8_t restTicks_ = 0;                 // consecutive at-rest ticks observed
  uint32_t hardResetCount_ = 0;
  uint32_t softResetCount_ = 0;

  float wedgePrevPosition_ = 0.0f;        // [counts]
  bool wedgePrevValid_ = false;
  uint8_t stuckCount_ = 0;                // raw, unconditional
  uint8_t movingStuckCount_ = 0;          // gated by |appliedDuty()| > motionThreshold_
  bool wedgeLatched_ = false;
  bool forcedWedge_ = false;  // DBG injection override (setForcedWedge)
  bool wedgeSuspect_ = false;

  // Ship default for the wedge-suspect motion gate — matches NezhaMotor's
  // own default output deadband (the two describe the same physical "is
  // the motor being driven" boundary).
  static constexpr float kDefaultMotionThreshold = 0.03f;   // [-1,1] fraction

  // Standstill-guard constants — engineering starting guesses, a
  // bench-tuning question.
  // COUNTS REBAKE (2026-08-15, counts-native leaf): was 5.0 mm/s ~= 64
  // counts/s at tovez's 0.7837 mm/deg; rounded to 60. An un-rebaked value
  // would have made the at-rest gate 10x tighter and near-unreachable,
  // starving the staged hard-reset path.
  static constexpr float kRestVelocity = 60.0f;        // [counts/s]
  static constexpr uint8_t kRestTicksRequired = 5;

  // Consecutive-identical-reading threshold for the wedge latch.
  static constexpr uint8_t kWedgeThreshold = 10;
};

}  // namespace Hardware
