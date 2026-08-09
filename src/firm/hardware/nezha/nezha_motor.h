// nezha_motor.h — Hardware::NezhaMotor: the BARE concrete leaf for one
// channel of the PlanetX Nezha V2 motor controller, implementing the
// Hal::Motor interface (motor.h). Owns the register map, split-phase
// 0x46 encoder sequencing, and ALL of the brick's own write shaping — slew
// limiting, write throttle, write-on-change, reversal dwell, and output
// deadband (see writeShapedDuty()/writeRawDuty() below). The output
// deadband BOOSTS a genuine nonzero sub-deadband duty to the deadband
// floor instead of zeroing it (an exact zero still stays an immediate
// hard stop) -- see writeShapedDuty()'s own doc comment. Wedge
// OBSERVATION/RECOVERY policy lives in the Hardware::MotorArmor decorator
// (motor_armor.h), which a caller may wrap this leaf in — or not (the sim
// composes the bare leaf directly).
//
// velocity()/position() below report a NAIVE per-tick difference quotient
// / the raw collected sample -- no freshness gate, glitch rejection, or
// smoothing. This is safe because the encoder register was measured LIVE
// at <=16 ms refresh on the current interleaved schedule
// (docs/design/encoder-refresh-characterization.md), so every tick
// collects a genuinely fresh sample and the naive difference quotient is
// a real velocity every cycle. A wheel observer could still replace this
// with a single principled predict-correct estimator (freshness +
// glitch/innovation rejection + the estimate) if a future need arises.
//
// This leaf owns the split-phase 0x46 protocol, hardReset()'s
// median-of-3 + readback-verify + retry, connected()/failure-hold, bus/
// write hygiene (fwdSign, clamp ±100%, integer-% quantization,
// write-on-change, NAK retry, write-rate throttle), and the slew cap.
//
// LOAD-BEARING (129-001, issue 07): write-on-change (writeRawDuty(), above)
// is NOT a pure "skip if unchanged" -- it is stopNotTaken-exempt. The Nezha
// brick physically latches its last commanded speed and does not reset on
// an nRF52 reset, only on power loss. lastWrittenPct_ records what this
// leaf last ATTEMPTED to write, not what actually landed on the brick, so a
// single lost zero write used to be permanent: the host believed the stop
// was sent, the wheel kept its prior nonzero speed forever, and every
// subsequent stop (including ESTOP) was suppressed as a no-op because
// lastWrittenPct_ already said 0. writeRawDuty() now re-issues a commanded
// zero whenever the wheel is still measurably moving above
// kStopConfirmVelocity, regardless of what lastWrittenPct_ claims. Do not
// simplify this back to a bare `pct == lastWrittenPct_` guard.
//
// Also unchanged: reversal dwell + output deadband (writeShapedDuty()),
// wheelTravelCalib, and the software-offset rebaseline mechanism
// (rebaseline()/softRebaseline() -- the stakeholder ruling that encoders
// are NEVER reset by device command stands; RobotLoop's own
// position-rebaseline policy, robot_loop.cpp, is separate).
//
// Deliberate scope-downs from a full motor abstraction:
//   - No message-plane surface (apply()/state()/capabilities()/
//     msg::MotorCommand) — msg:: is unreachable under the isolation
//     invariant; the loop constructs and drives this leaf directly.
//   - No POSITION mode (the onboard 0x5D absolute-angle move) — this leaf
//     only covers raw-duty mode (see DESIGN.md §3, historical).
//   - Time seam: tick() takes a single `uint64_t nowUs` [us] parameter
//     rather than reading a clock internally — Platform::Clock (clock.h) is
//     the fiber-level time seam, scoped to "the fiber's OWN cycle-level
//     time reads ... not the bus's clearance windows" (a DIFFERENT seam
//     from I2CBus's own internal clearance-timer bookkeeping — see
//     i2c_bus.h). This leaf takes "now" as a plain parameter supplied by
//     its caller (ultimately the loop's own Clock instance) — fully
//     deterministic for a host harness with zero clock coupling.
//
// Design/rationale: DESIGN.md.
#pragma once

#include <cstdint>

#include "hal/device_config.h"
#include "hal/device_types.h"
#include "platform/i2c_bus.h"
#include "hal/motor.h"

namespace Hardware {

// 7-bit I2C address shared by all four Nezha V2 motor channels (the
// motorId byte in each frame selects the channel, not the address).
constexpr uint8_t kNezhaDeviceAddr = 0x10;

class NezhaMotor : public Hal::Motor {
 public:
  NezhaMotor(Platform::I2CBus& bus, const Hal::MotorConfig& config);

  // Primes the encoder: the Nezha 0x46 register sits frozen at 0 until the
  // chip receives its first atomic read transaction (calls hardReset()).
  // The fiber preamble calls this once per port before the cycle starts.
  void begin() override;

  // Split-phase phase 1, public entry point. Wraps requestEncoder() so the
  // loop's own cycle can request this port's encoder sample without
  // reaching into NezhaMotor's private register-verb surface (the Hal::Motor
  // interface frames it generically as "prepare this cycle's sample").
  void requestSample() override;

  // --- Primitive setters — stage the command; tick() executes it. ---
  void setDuty(float duty) override;           // [-1, 1] raw duty target
  void setNeutral(Hal::Neutral mode) override;      // coast / brake — Nezha maps both to the same 0x60 speed-0 write (no distinct brake register)

  // --- Resets (bare-motor semantics — see motor.h): resetPosition() acts
  // IMMEDIATELY (== hardReset()'s median-of-3 re-prime burst; the caller —
  // or a wrapping MotorArmor — owns any at-rest discipline);
  // rebaseline() is the software-only re-anchor. ---
  void resetPosition() override;
  void rebaseline() override;

  // applyTravelCalib -- the ONE field this leaf still live-applies (see
  // motor.h's own header). No reflash, no I2C side effect -- tick()'s own
  // position() conversion reads config_.wheelTravelCalib fresh every call.
  void applyTravelCalib(float travelCalib) override;

  // reconfigure — whole-config replacement, guarded (see motor.h).
  // Refuses (returns false, leaves config_ unchanged) unless
  // mode_ == Mode::None (never yet commanded) or the motor is
  // independently at rest (|velocity()| < kReconfigureRestVelocity AND
  // appliedDuty() == 0.0f). On success, reassigns config_ wholesale and
  // re-derives the slew-rate/write-shaping substitution fields exactly as
  // the constructor does, then returns true. See motor.h's own doc comment
  // for why this is a separate, narrower surface from applyTravelCalib().
  [[nodiscard]] bool reconfigure(const Hal::MotorConfig& config) override;

  // Full live config readback -- for a caller that must merge a partial
  // update onto everything this motor is actually running (sim_ctypes.cpp's
  // sim_configure_motor(): its Tier-2 fwdSign push round-trips through
  // reconfigure()'s whole-config replacement, so building the replacement
  // from anything less than this live config clobbers every other field --
  // wheelTravelCalib/slewRate/outputDeadband/... -- back to zero). Kept
  // non-virtual/NezhaMotor-local: the one caller holds a concrete
  // NezhaMotor&, and the Hal::Motor interface has no such readback surface.
  const Hal::MotorConfig& config() const { return config_; }

  // --- Primitive getters (Hal::Motor overrides) ---
  float position() const override;      // [mm]
  float velocity() const override;      // [mm/s] signed -- naive per-tick difference quotient, see this file's own header
  float appliedDuty() const override;   // [-1, 1]

  bool connected() const override { return connected_; }

  // Motor::sampleTime() override -- returns lastFreshUs_ (131-002, issue
  // A-commanded-zero-leaks-through-stage-b.md), which advances ONLY when
  // this leaf's own tick() step 1 collect actually SUCCEEDED (connected_
  // true), NOT merely on every tick() call -- see lastFreshUs_'s own
  // field comment below. On the clean schedule (fresh sample every
  // cycle; docs/design/encoder-refresh-characterization.md) this is
  // indistinguishable from "every tick() call," which is what this
  // accessor used to return outright (lastTickUs_) -- but on a
  // disconnected/glitching bus it now correctly HOLDS the last genuine
  // reading's timestamp instead of reporting a failed collect as fresh.
  uint64_t sampleTime() const override { return lastFreshUs_; }  // [us]

  // tick() — the leaf's 2-step contract (see nezha_motor.cpp; MotorArmor's
  // own tick() wraps this with its own decorator-level steps):
  //   1. sample + cache this motor's own encoder (device-specific), and
  //      compute a naive per-tick velocity from it (see this file's own
  //      header); advance lastFreshUs_ (131-002) only when this collect
  //      genuinely succeeded.
  //   2. mode dispatch — Mode::Active writes the staged raw duty via
  //      writeShapedDuty(); Mode::Neutral writes 0 via writeShapedDuty();
  //      Mode::None dispatches nothing.
  void tick(uint64_t nowUs) override;   // [us]

 private:
  // --- Device write path + resets (leaf internals — no longer virtuals) ---
  void writeShapedDuty(float duty, uint32_t now);   // [-1,1] [ms] output-deadband boost (sub-deadband nonzero -> deadband floor; exact zero stays zero), then reversal dwell, then writeRawDuty() -- see nezha_motor.cpp's own doc comment
  void writeRawDuty(float duty);    // clamp + write-on-change (stopNotTaken-exempt, 129-001) + throttle + slew + fwdSign + bus write
  void hardReset();                 // median-of-3 + readback-verify + retry
  void softRebaseline();            // software-only rebaseline

  enum class Mode : uint8_t { None, Active, Neutral };

  // ---- Wiring ----
  Platform::I2CBus& bus_;
  Hal::MotorConfig config_;

  // ---- Staged command (set by the primitive setters; executed by tick()) ----
  Mode mode_ = Mode::None;
  float dutyTarget_ = 0.0f;                   // [-1, 1]
  Hal::Neutral neutralTarget_ = Hal::Neutral::Coast;

  // ---- tick() encoder-sample cache ----
  float lastPosition_ = 0.0f;          // [mm]
  float velocity_ = 0.0f;              // [mm/s] naive per-tick difference quotient (see this file's own header)
  uint64_t lastTickUs_ = 0;            // [us] this leaf's own time seam — see file header
  // Genuine freshness (131-002, issue A-commanded-zero-leaks-through-
  // stage-b.md): advances ONLY when THIS tick's collectEncoder() actually
  // succeeded (connected_ true) -- unlike lastTickUs_ above, which stamps
  // every tick() call regardless of collect outcome. sampleTime() (above)
  // returns THIS field, not lastTickUs_, so a caller's freshness/age
  // computation (App::Drive's Stage B/C gates, drive.cpp; Telemetry's own
  // age fields) tells the truth about the last SUCCESSFUL read on a
  // disconnected/glitching bus, instead of reading "fresh" off a tick
  // that never actually collected anything.
  uint64_t lastFreshUs_ = 0;           // [us]
  bool hasLastTick_ = false;
  bool connected_ = false;

  // ---- Write path ----
  int8_t lastWrittenPct_ = -128;        // [%] sentinel (outside +/-100) forces the first write
  // Sigma-delta accumulator for the integer-percent quantizer
  // (writeRawDuty(), 133-002). Holds the residual between the duty
  // actually wanted and the integer percent written, so the TIME-AVERAGED
  // output resolves below one count -- one count is 8.46 mm/s at the
  // measured plant gain, which is coarser than the imbalance the wheel
  // controller is trying to correct. ZEROED ON COMMANDED STOP, always: a
  // residual carried across a stop would round to +-1 and creep a stopped
  // wheel, which is the runaway class this file's own LOAD-BEARING note
  // exists to prevent.
  float dutyCarry_ = 0.0f;  // [percent] fractional residual, [-1, 1]
  uint64_t lastWriteTimeUs_ = 0;        // [us]

  // ---- Write shaping: reversal dwell + output deadband — Nezha-brick
  // wedge protection (an instantaneous H-bridge sign flip under way
  // latches the 0x46 readback; near-zero dither would request such flips
  // every tick — see docs/knowledge/2026-07-04-encoder-wedge.md).
  // Config-driven: cached straight from Hal::MotorConfig's required
  // reversalDwell/outputDeadband fields in reconfigure() -- no code-side
  // ship default substitution; gen_boot_config.py always emits real
  // values, see data/robots/*.json's control.reversal_dwell_ms/
  // output_deadband. An explicit 0/0 makes writeShapedDuty() a pure
  // pass-through. outputDeadband_ BOOSTS a genuine nonzero sub-deadband
  // duty up to itself (sign-preserving) rather than zeroing it -- an
  // explicit 0 here still means "never boost," i.e. still a pure
  // pass-through. ----
  float reversalDwell_ = 0.0f;          // [ms] cached from Hal::MotorConfig
  float outputDeadband_ = 0.0f;         // [-1,1] fraction, cached from Hal::MotorConfig
  bool dwelling_ = false;
  uint32_t dwellDeadline_ = 0;          // [ms]
  float lastRequestedDuty_ = 0.0f;      // [-1,1] last duty actually forwarded to writeRawDuty()

  // ---- Encoder software offset / failure-hold state ----
  int32_t encOffset_ = 0;               // [tenths of degrees]
  int32_t lastGoodRawEnc_ = 0;          // held on I2C failure
  bool pendingEncRequestOk_ = true;     // requestEncoder()/collectEncoder() pairing

  // ---- Register-map wire constants ----
  static constexpr uint8_t kDirCw = 1;      // positive speed from chip perspective
  static constexpr uint8_t kDirCcw = 2;     // negative speed from chip perspective
  static constexpr float kDefaultSlewRate = 25.0f;   // default max |delta PWM| per write

  // reconfigure()'s own at-rest guard threshold. Mirrors MotorArmor's own
  // kRestVelocity at-rest threshold (motor_armor.h)
  // conceptually, but is NOT shared across the class boundary: this is a
  // leaf-local constant for a leaf-local guard.
  static constexpr float kReconfigureRestVelocity = 5.0f;  // [mm/s] mirrors MotorArmor's own kRestVelocity at-rest threshold

  // writeRawDuty()'s stopNotTaken threshold (129-001, issue 07 -- see that
  // file's own doc comment below). lastWrittenPct_ records the WRITE
  // ATTEMPT, not the physically landed brick state -- the Nezha brick
  // latches its last commanded speed and does not reset on an nRF52 reset,
  // only on power loss, so one lost zero write is otherwise permanent and
  // every later ESTOP is suppressed as a no-op. A commanded zero whose
  // wheel is still measurably turning above this threshold is NOT
  // write-on-change-suppressed, regardless of what lastWrittenPct_ already
  // claims was sent. NOT shared with kReconfigureRestVelocity above or
  // MotorArmor's own kRestVelocity -- same "leaf-local constant for a
  // leaf-local guard" reasoning.
  static constexpr float kStopConfirmVelocity = 8.0f;  // [mm/s]

  // ---- Private helpers: write path ----
  // Returns the CODAL status from bus_.write() (0/kOk == success):
  // writeRawDuty() commits lastWrittenPct_/lastWriteTimeUs_ ONLY when this
  // status is kOk, so a NAK'd write is retried next tick instead of being
  // latched as "already written."
  int writeMotorRun(uint8_t direction, uint8_t speed);   // writes the 0x60 motor-run command

  // ---- Private helpers: encoder read paths ----
  int32_t readEncoderAtomicRaw();   // one-off sample: preClear/postClear-settled 0x46 write -> read
  void requestEncoder();            // split-phase phase 1; wrapped by the public requestSample() above
  int32_t collectEncoder();         // split-phase phase 2; wired into tick()'s step 2
};

}  // namespace Hardware
