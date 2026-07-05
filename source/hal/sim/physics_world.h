// physics_world.h — Hal::PhysicsWorld: the single source of ground truth for
// the simulated chassis (sprint 081-003, ported from
// source_old/hal/sim/PhysicsWorld.{h,cpp}).
//
// PhysicsWorld owns ONLY ground truth (true pose, true per-wheel travel)
// plus the legacy "reported" (errored) encoder accumulator — no line/color/
// port sensor truth lives here any more. The design write-up's resolved
// decision 2 drops those three aux channels (`_lineRaw`/`_colorRGBC`/`_port`
// and every setter/getter that touched them): the new tree's DEV command
// family has no line/color/port wire surface yet, so there is nothing for
// them to feed, and porting dead state forward is not "ported unchanged" —
// it is scope creep this ticket declines. Every other member — the
// midpoint-arc pose integrator, the true/reported encoder split, the
// per-wheel error knobs (scale/slip/noise), the stiction gate, the optional
// motor-lag filter, and the body-truth scrub — ports byte-for-byte.
//
// Two ways in:
//   1. Evolve mode:  setActuators(pwmL, pwmR) + update(dt) — advances the
//      chassis from PWM commands (Hal::SimMotor's write path stages here).
//   2. Truth-injection mode: setTruePose / setTrueWheelTravel /
//      setTrueVelocity — sets ground truth directly for isolation tests.
//
// GOLDEN-TLM CONSTRAINT (carried over verbatim from source_old): update()
// sub-step A (true-encoder accumulation) and sub-step A' (reported-encoder
// accumulation) MUST match source_old's MockMotor::integrate bit-for-bit for
// zero-slip / zero-noise / offset-factor-1.0 inputs — this is exactly what
// ticket 003's zero-error determinism gate re-proves against the new tree.
// Do NOT simplify update()'s sub-step expressions — see physics_world.cpp.
//
// Reported vs. true encoder split (ported from source_old's "OQ-1 Option A"):
// PhysicsWorld tracks BOTH a true (unslipped) encoder accumulator AND a
// reported (legacy encoder-step slip + noise) accumulator. Hal::SimMotor::
// position() reads the REPORTED value; the true accumulator remains the
// unslipped ground truth for test/ctypes truth-reads.
//
// Namespace/placement: Hal::PhysicsWorld, source/hal/sim/ — private,
// hal/sim/-local infrastructure with no Hal:: capability interface of its
// own, the same way I2CBus is source/com/-local infrastructure without
// implementing a Hal:: interface (architecture-update.md (081) Step 7 Open
// Question 3). Its only consumer is Subsystems::SimHardware
// (source/subsystems/sim_hardware.{h,cpp}), which owns one instance plus
// four Hal::SimMotor leaves and one Hal::SimOdometer leaf constructed
// against it.
//
// Kept the internal [-100,100] actuator scale (int8_t pwmL/pwmR, PWM-unit
// stiction thresholds) unchanged, per the ticket's own directive, so the
// stiction-knob semantics and this class's arithmetic port unchanged.
//
// Zero-heap, single-threaded, value-member ownership. Compiles host-side
// (HOST_BUILD) with no CODAL dependency — excluded from the ARM firmware
// build by CMakeLists.txt's blanket ".*/hal/sim/.*" EXCLUDE REGEX.
#pragma once
#include <stdint.h>

#ifdef HOST_BUILD
#include <random>
#endif

namespace Hal {

class PhysicsWorld {
 public:
  // Default dynamics parameters — match source_old's MockMotor/MockHAL defaults.
  static constexpr float kNominalMaxSpeed = 400.0f;  // [mm/s]
  static constexpr float kDefaultTrackwidth = 150.0f;  // [mm]

  PhysicsWorld() = default;

  // --- Evolve mode ---------------------------------------------------------

  // Store the per-wheel PWM commands [-100, 100]; consumed by the next update().
  void setActuators(int8_t pwmL, int8_t pwmR) {
    pwmL_ = pwmL;
    pwmR_ = pwmR;
  }

  // Store ONE wheel's PWM command (Hal::SimMotor::writeRawDuty forwards
  // here; the authoritative plant tick uses setActuators(pwmL, pwmR)).
  // side: 0 = left, 1 = right.
  void setActuator(int side, int8_t pwm) {
    if (side == 0) pwmL_ = pwm;
    else           pwmR_ = pwm;
  }

  // Advance the chassis one step of dt milliseconds. Two structurally
  // separate sub-steps: (A/A') encoder accumulation (golden-TLM bit-exact
  // path), (B) chassis pose integration (slip applied here, not on the TLM
  // path). dt == 0 is a no-op (Subsystems::SimHardware's dt=0 re-entry guard
  // is the caller-side contract this class relies on — see
  // architecture-update.md (081) Decision 4 — but update() itself also
  // guards dt==0 defensively, matching source_old exactly).
  void update(uint32_t dt);  // [ms]

  // --- Truth-injection mode (isolation tests) ------------------------------

  void setTruePose(float x, float y, float h) {
    truePoseX_ = x;
    truePoseY_ = y;
    truePoseH_ = h;
  }

  void setTrueWheelTravel(float encL, float encR) {
    trueEncL_ = encL;
    trueEncR_ = encR;
  }

  void setTrueVelocity(float velL, float velR) {
    trueVelL_ = velL;
    trueVelR_ = velR;
  }

  // Zero all state.
  void reset();

  // --- Dynamics configuration ----------------------------------------------

  // Fractional slip. Applied at the chassis body-rotation step (sub-step
  // B) — see effectiveSlip() in physics_world.cpp.
  void setSlip(float straight, float turnExtra) {
    slipStraight_  = straight;
    slipTurnExtra_ = turnExtra;
    rotationalSlip_ = straight;
  }

  // Body-truth scrub: independent, wire-settable rotational/linear
  // efficiency applied in sub-step B, MULTIPLICATIVELY combined with (not
  // replacing) the effectiveSlip(rotationalSlip_) term above. Default 1.0 =
  // no-op.
  void setBodyRotationalScrub(float f) { bodyRotationalScrub_ = f; }
  void setBodyLinearScrub(float f)     { bodyLinearScrub_ = f; }

  // Per-wheel offset factor. side: 0 = left, 1 = right, 2 = both.
  void setOffsetFactor(int side, float f) {
    if (side == 0)      { offsetFactorL_ = f; }
    else if (side == 1) { offsetFactorR_ = f; }
    else                { offsetFactorL_ = f; offsetFactorR_ = f; }
  }

  // Reported-encoder (legacy) noise config. side: 0 = left, 1 = right,
  // 2 = both. Gaussian encoder noise (mm per tick) added to the REPORTED
  // encoder accumulator only; the true accumulator is unaffected.
  void setEncoderNoise(int side, float sigma) {  // [mm] noise sigma per tick
    if (side == 0 || side > 1) encNoiseSigmaL_ = sigma;
    if (side == 1 || side > 1) encNoiseSigmaR_ = sigma;
  }

  // Encoder error injection: per-wheel scale error and slip applied to the
  // REPORTED encoder accumulator only. The true accumulator (trueEncL_ /
  // trueEncR_) and chassis pose remain unaffected. side: 0 = left,
  // 1 = right, 2 = both. err: fractional scale error (0 = perfect, 0.05 =
  // 5% over-report). fraction: fraction of motion not registered (0 =
  // perfect, 0.05 = 5% under-report).
  void setEncoderScaleError(int side, float err) {
    if (side == 0 || side > 1) encScaleErrL_ = err;
    if (side == 1 || side > 1) encScaleErrR_ = err;
  }
  void setEncoderSlip(int side, float fraction) {
    if (side == 0 || side > 1) encSlipL_ = fraction;
    if (side == 1 || side > 1) encSlipR_ = fraction;
  }

  void setTrackwidth(float trackwidth)  { trackwidth_ = trackwidth; }        // [mm]
  void setNominalMaxSpeed(float speed)  { nominalMaxSpeed_ = speed; }        // [mm/s]

  // Motor stiction/breakaway gate: a stateless per-tick PWM dead-zone.
  // |pwm| < stictionPwmSide => this tick's target velocity is forced to 0,
  // regardless of the wheel's velocity on any previous tick. Default 0 =>
  // never fires (no-op). side: 0 = left, 1 = right, 2 = both.
  void setStictionPwm(int side, float pwm) {  // [PWM units, 0-100]
    if (side == 0 || side > 1) stictionPwmL_ = pwm;
    if (side == 1 || side > 1) stictionPwmR_ = pwm;
  }

  // Optional first-order motor response lag: per-wheel time constant.
  // tau <= 0 (default) skips the exp() call so the output velocity equals
  // the (possibly stiction-gated) target velocity bit-for-bit; tau > 0
  // converges the reported velocity toward the target exponentially. side:
  // 0 = left, 1 = right, 2 = both.
  void setMotorLag(int side, float tauMs) {  // [ms]
    if (side == 0 || side > 1) motorLagL_ = tauMs;
    if (side == 1 || side > 1) motorLagR_ = tauMs;
  }

  // Per-tick turn rate (set by Subsystems::SimHardware before update());
  // used only by the reported-encoder slip term (sub-step A').
  void setTurnRate(float r) { turnRate_ = r; }

  // --- Read accessors (const ground-truth) ---------------------------------

  float truePoseX() const { return truePoseX_; }
  float truePoseY() const { return truePoseY_; }
  float truePoseH() const { return truePoseH_; }

  float trueEncL() const { return trueEncL_; }
  float trueEncR() const { return trueEncR_; }

  // Reported (legacy encoder-step slip + noise) encoder accumulators.
  // Hal::SimMotor::position() reads these. Equal to the true accumulators
  // when slip == 0 and noise == 0 (the zero-error determinism gate).
  float reportedEncL() const { return reportedEncL_; }
  float reportedEncR() const { return reportedEncR_; }

  // Directly set the reported encoder accumulator (used by
  // Hal::SimMotor::hardReset()/softRebaseline()).
  void setReportedEncoder(int side, float position) {  // [mm]
    if (side == 0) reportedEncL_ = position;
    else           reportedEncR_ = position;
  }

  // Zero one side's reported encoder accumulator. The true accumulator is
  // the ground truth and is NOT reset here.
  void resetReportedEncoder(int side) {
    if (side == 0) reportedEncL_ = 0.0f;
    else           reportedEncR_ = 0.0f;
  }

  float trueVelL() const { return trueVelL_; }
  float trueVelR() const { return trueVelR_; }

  // Dynamics-parameter accessors (for tests / observation models).
  float trackwidth()      const { return trackwidth_; }
  float nominalMaxSpeed() const { return nominalMaxSpeed_; }  // [mm/s]
  float rotationalSlip()  const { return rotationalSlip_; }
  int8_t pwmL()           const { return pwmL_; }
  int8_t pwmR()           const { return pwmR_; }

  float bodyRotationalScrub() const { return bodyRotationalScrub_; }
  float bodyLinearScrub()     const { return bodyLinearScrub_; }

  float offsetFactorL() const { return offsetFactorL_; }
  float offsetFactorR() const { return offsetFactorR_; }

  float encoderScaleErrL() const { return encScaleErrL_; }
  float encoderScaleErrR() const { return encScaleErrR_; }
  float encoderSlipL()     const { return encSlipL_; }
  float encoderSlipR()     const { return encSlipR_; }
  float encoderNoiseL()    const { return encNoiseSigmaL_; }
  float encoderNoiseR()    const { return encNoiseSigmaR_; }

  float stictionPwmL() const { return stictionPwmL_; }
  float stictionPwmR() const { return stictionPwmR_; }
  float motorLagL()    const { return motorLagL_; }
  float motorLagR()    const { return motorLagR_; }

 private:
  // --- Commanded actuator state ---
  int8_t pwmL_ = 0;
  int8_t pwmR_ = 0;

  // --- True chassis pose ---
  float truePoseX_ = 0.0f;
  float truePoseY_ = 0.0f;
  float truePoseH_ = 0.0f;

  // --- True per-wheel travel / velocity (unslipped ground truth) ---
  float trueEncL_ = 0.0f;
  float trueEncR_ = 0.0f;
  float trueVelL_ = 0.0f;
  float trueVelR_ = 0.0f;

  // --- Reported per-wheel travel (legacy encoder-step slip + noise);
  // read by Hal::SimMotor::position(). ---
  float reportedEncL_ = 0.0f;
  float reportedEncR_ = 0.0f;

  // --- Dynamics parameters ---
  float trackwidth_      = kDefaultTrackwidth;
  float nominalMaxSpeed_ = kNominalMaxSpeed;  // [mm/s]
  float rotationalSlip_  = 0.0f;              // 0/unset -> effectiveSlip() -> 1.0
  float slipStraight_    = 0.0f;
  float slipTurnExtra_   = 0.0f;

  // --- Body-truth scrub — independent, multiplicative with the
  // effectiveSlip(rotationalSlip_) term above; default 1.0 = no-op. ---
  float bodyRotationalScrub_ = 1.0f;
  float bodyLinearScrub_     = 1.0f;

  // --- Per-wheel offset factors (default symmetric) ---
  float offsetFactorL_ = 1.0f;
  float offsetFactorR_ = 1.0f;

  // --- Reported-encoder error model ---
  // Per-tick turn rate in [0, 1] (set by Subsystems::SimHardware before
  // update()); drives the encoder-step slip term (slip = slipStraight_ +
  // slipTurnExtra_ * turnRate_).
  float turnRate_       = 0.0f;
  float encNoiseSigmaL_ = 0.0f;
  float encNoiseSigmaR_ = 0.0f;

  // Encoder error injection: per-wheel scale error and slip applied to the
  // REPORTED encoder accumulator only. Default zero = no effect.
  float encScaleErrL_ = 0.0f;  // fractional over/under-report (0 = perfect)
  float encScaleErrR_ = 0.0f;
  float encSlipL_     = 0.0f;  // fraction of motion not registered (0 = perfect)
  float encSlipR_     = 0.0f;

  // --- Stiction/breakaway gate + optional lag ---
  // stictionPwmL_/R_: per-wheel PWM dead-zone threshold, default 0 => gate
  // never fires (no-op). motorLagL_/R_: per-wheel first-order response time
  // constant [ms], default 0 => no-op (no exp() call). lagVelL_/R_: the
  // filter's persistent state (last output velocity); zeroed in reset().
  float stictionPwmL_ = 0.0f;
  float stictionPwmR_ = 0.0f;
  float motorLagL_    = 0.0f;  // [ms]
  float motorLagR_    = 0.0f;  // [ms]
  float lagVelL_      = 0.0f;  // [mm/s] persistent lag-filter state
  float lagVelR_      = 0.0f;  // [mm/s] persistent lag-filter state

#ifdef HOST_BUILD
  // Two independent generators so the LEFT/RIGHT encoder-noise draw streams
  // match source_old's retired MockMotor (each MockMotor owned its own
  // std::mt19937{42u}).
  std::mt19937 rngL_{42u};
  std::mt19937 rngR_{42u};
#endif
};

}  // namespace Hal
