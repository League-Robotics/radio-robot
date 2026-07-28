// nezha_motor.h — Devices::NezhaMotor: the BARE concrete leaf for one
// channel of the PlanetX Nezha V2 motor controller, implementing the
// Devices::Motor interface (motor.h). Owns the register map, split-phase
// 0x46 encoder sequencing, and ALL of the brick's own write shaping — slew
// limiting, write throttle, write-on-change, reversal dwell, and output
// deadband (see writeShapedDuty()/writeRawDuty() below). Sprint 114 ticket
// 005: the output deadband BOOSTS a genuine nonzero sub-deadband duty to
// the deadband floor instead of zeroing it (an exact zero still stays an
// immediate hard stop) -- see writeShapedDuty()'s own doc comment. Wedge
// OBSERVATION/RECOVERY policy lives in the Devices::MotorArmor decorator
// (motor_armor.h), which a caller may wrap this leaf in — or not (the sim
// composes the bare leaf directly).
//
// 125-003 (sprint 125, Decision 2 -- "protection vs. control", see
// sprint.md): SHRUNK to its base-contract residue -- protocol + bus hygiene
// + dwell/deadband + clamp, ~200 lines (from 885). Everything that was a
// VELOCITY DECISION or MEASUREMENT-CONDITIONING mechanism is gone:
//   - The staged-velocity setter, the embedded per-channel velocity
//     control law, and the kff mapping -- DELETED. The closed-loop
//     velocity control law relocated to the motion library
//     (src/motion/wheel_velocity_pid.{h,cpp}) -- App::Drive holds the
//     interim instances this sprint (see drive.h's own header). [The
//     original rate argument -- "~80ms encoder freshness bounds the loop"
//     -- was measured FALSE 2026-07-26 (docs/design/
//     encoder-refresh-characterization.md); the relocation stands on
//     one-estimate-one-controller and host tunability instead.]
//   - The freshness gate, source-side glitch rejection, and the
//     live-switchable EMA/least-squares velocity-estimator pair -- DELETED
//     OUTRIGHT, not relocated. velocity()/position() below report a NAIVE
//     per-tick difference quotient / the raw collected sample -- honest,
//     and BETTER than its author feared: the register was measured LIVE
//     at <=16 ms (docs/design/encoder-refresh-characterization.md -- the
//     "~80ms refresh" was a pre-118 schedule artifact), so on the clean
//     interleaved schedule every tick collects a genuinely fresh sample
//     and the naive difference quotient is a real velocity every cycle.
//     A wheel observer can still replace this with one principled
//     predict-correct estimator (freshness + glitch/innovation rejection +
//     the estimate, folded into a single model) -- this is the sprint's
//     own accepted, disclosed interim (see sprint.md Migration Concerns).
//   - Duty-boxcar smoothing (the bench-only output-averaging window and
//     its setter) -- DELETED, no replacement planned (never shipped
//     live-tuned).
//
// KEPT unchanged: the split-phase 0x46 protocol, hardReset()'s
// median-of-3 + readback-verify + retry, connected()/failure-hold, bus/
// write hygiene (fwdSign, clamp ±100%, integer-% quantization,
// write-on-change, NAK retry, write-rate throttle), the slew cap
// (UNMODIFIED -- ticket 010 owns its disposition, not this ticket),
// reversal dwell + output deadband (writeShapedDuty()), wheelTravelCalib,
// and the software-offset rebaseline mechanism (rebaseline()/
// softRebaseline() -- the stakeholder ruling that encoders are NEVER reset
// by device command stands unmodified; RobotLoop's position-rebaseline
// policy, robot_loop.cpp, is untouched by this ticket).
//
// Deliberate scope-downs from a full motor abstraction:
//   - No message-plane surface (apply()/state()/capabilities()/
//     msg::MotorCommand) — msg:: is unreachable under the isolation
//     invariant; the loop constructs and drives this leaf directly.
//   - No POSITION mode (the onboard 0x5D absolute-angle move) — this leaf
//     only covers raw-duty mode (see DESIGN.md §3, historical).
//   - Time seam: tick() takes a single `uint64_t nowUs` [us] parameter
//     rather than reading a clock internally — Devices::Clock (clock.h) is
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

#include "devices/device_config.h"
#include "devices/device_types.h"
#include "devices/i2c_bus.h"
#include "devices/motor.h"

namespace Devices {

// 7-bit I2C address shared by all four Nezha V2 motor channels (the
// motorId byte in each frame selects the channel, not the address).
constexpr uint8_t kNezhaDeviceAddr = 0x10;

class NezhaMotor : public Motor {
 public:
  NezhaMotor(I2CBus& bus, const MotorConfig& config);

  // Primes the encoder: the Nezha 0x46 register sits frozen at 0 until the
  // chip receives its first atomic read transaction (calls hardReset()).
  // The fiber preamble calls this once per port before the cycle starts.
  void begin() override;

  // Split-phase phase 1, public entry point. Wraps requestEncoder() so the
  // loop's own cycle can request this port's encoder sample without
  // reaching into NezhaMotor's private register-verb surface (the Motor
  // interface frames it generically as "prepare this cycle's sample").
  void requestSample() override;

  // --- Primitive setters — stage the command; tick() executes it. ---
  void setDuty(float duty) override;           // [-1, 1] raw duty target
  void setNeutral(Neutral mode) override;      // coast / brake — Nezha maps both to the same 0x60 speed-0 write (no distinct brake register)

  // --- Resets (bare-motor semantics — see motor.h): resetPosition() acts
  // IMMEDIATELY (== hardReset()'s median-of-3 re-prime burst; the caller —
  // or a wrapping MotorArmor — owns any at-rest discipline);
  // rebaseline() is the software-only re-anchor. ---
  void resetPosition() override;
  void rebaseline() override;

  // applyTravelCalib -- 125-003 (motor.h's own header): narrowed from the
  // pre-125-003 applyGains(Gains, Opt<float>) to the ONE field this leaf
  // still live-applies. No reflash, no I2C side effect -- tick()'s own
  // position() conversion reads config_.wheelTravelCalib fresh every call.
  void applyTravelCalib(float travelCalib) override;

  // reconfigure — REVISION 1 (114-001, motor.h): whole-config replacement,
  // guarded. Refuses (returns false, leaves config_ unchanged) unless
  // mode_ == Mode::None (never yet commanded) or the motor is
  // independently at rest (|velocity()| < kReconfigureRestVelocity AND
  // appliedDuty() == 0.0f). On success, reassigns config_ wholesale and
  // re-derives the slew-rate/write-shaping substitution fields exactly as
  // the constructor does, then returns true. See motor.h's own doc comment
  // for why this is a separate, narrower surface from applyTravelCalib().
  [[nodiscard]] bool reconfigure(const MotorConfig& config) override;

  // Full live config readback -- for a caller that must merge a partial
  // update onto everything this motor is actually running (sim_ctypes.cpp's
  // sim_configure_motor(): its Tier-2 fwdSign push round-trips through
  // reconfigure()'s whole-config replacement, so building the replacement
  // from anything less than this live config clobbers every other field --
  // wheelTravelCalib/slewRate/outputDeadband/... -- back to zero). Kept
  // non-virtual/NezhaMotor-local: the one caller holds a concrete
  // NezhaMotor&, and the Motor interface has no such readback surface.
  const MotorConfig& config() const { return config_; }

  // --- Primitive getters (Motor overrides) ---
  float position() const override;      // [mm]
  float velocity() const override;      // [mm/s] signed -- naive per-tick difference quotient, see this file's own header
  float appliedDuty() const override;   // [-1, 1]

  bool connected() const override { return connected_; }

  // Motor::sampleTime() override -- 125-003: with the freshness gate
  // deleted (this file's own header), every tick() call is now treated as
  // "fresh" -- this simply returns lastTickUs_, the nowUs of the most
  // recent tick() call -- which the encoder characterization says is the
  // truth on the clean schedule (fresh sample every cycle; docs/design/
  // encoder-refresh-characterization.md), not a degraded placeholder.
  uint64_t sampleTime() const override { return lastTickUs_; }  // [us]

  // tick() — the leaf's 2-step contract (see nezha_motor.cpp; the old base-
  // armor steps 1/3/5 now live in the MotorArmor DECORATOR's own tick()):
  //   1. sample + cache this motor's own encoder (device-specific), and
  //      compute a naive per-tick velocity from it (see this file's own
  //      header for why this is a disclosed interim, not the pre-125-003
  //      freshness-gated behavior).
  //   2. mode dispatch — Mode::Active writes the staged raw duty via
  //      writeShapedDuty(); Mode::Neutral writes 0 via writeShapedDuty();
  //      Mode::None dispatches nothing.
  void tick(uint64_t nowUs) override;   // [us]

 private:
  // --- Device write path + resets (leaf internals — no longer virtuals;
  // the old MotorArmor base-class seam is gone) ---
  void writeShapedDuty(float duty, uint32_t now);   // [-1,1] [ms] output-deadband boost (sub-deadband nonzero -> deadband floor; exact zero stays zero), then reversal dwell, then writeRawDuty() -- see nezha_motor.cpp's own doc comment (114-005)
  void writeRawDuty(float duty);    // clamp + write-on-change + throttle + slew + fwdSign + bus write
  void hardReset();                 // median-of-3 + readback-verify + retry
  void softRebaseline();            // software-only rebaseline

  enum class Mode : uint8_t { None, Active, Neutral };

  // ---- Wiring ----
  I2CBus& bus_;
  MotorConfig config_;

  // ---- Staged command (set by the primitive setters; executed by tick()) ----
  Mode mode_ = Mode::None;
  float dutyTarget_ = 0.0f;                   // [-1, 1]
  Neutral neutralTarget_ = Neutral::Coast;

  // ---- tick() encoder-sample cache ----
  float lastPosition_ = 0.0f;          // [mm]
  float velocity_ = 0.0f;              // [mm/s] naive per-tick difference quotient (125-003 -- see this file's own header)
  uint64_t lastTickUs_ = 0;            // [us] this leaf's own time seam — see file header
  bool hasLastTick_ = false;
  bool connected_ = false;

  // ---- Write path ----
  int8_t lastWrittenPct_ = -128;        // [%] sentinel (outside +/-100) forces the first write
  uint64_t lastWriteTimeUs_ = 0;        // [us]

  // ---- Write shaping (folded from the old MotorArmor base, 2026-07-18):
  // reversal dwell + output deadband — Nezha-brick wedge protection (an
  // instantaneous H-bridge sign flip under way latches the 0x46 readback;
  // near-zero dither would request such flips every tick — see
  // docs/knowledge/2026-07-04-encoder-wedge.md). Config-driven: cached
  // straight from MotorConfig's required reversalDwell/outputDeadband
  // fields in reconfigure() (sprint 114 ticket 003 — no more code-side ship
  // default substitution; gen_boot_config.py always emits real values, see
  // data/robots/*.json's control.reversal_dwell_ms/output_deadband). An
  // explicit 0/0 makes writeShapedDuty() a pure pass-through. Sprint 114
  // ticket 005: outputDeadband_ BOOSTS a genuine nonzero sub-deadband duty
  // up to itself (sign-preserving) rather than zeroing it -- an explicit 0
  // here still means "never boost," i.e. still a pure pass-through. ----
  float reversalDwell_ = 0.0f;          // [ms] cached from MotorConfig
  float outputDeadband_ = 0.0f;         // [-1,1] fraction, cached from MotorConfig
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

  // reconfigure()'s own at-rest guard threshold — REVISION 1 (114-001).
  // Mirrors MotorArmor's own kRestVelocity at-rest threshold (motor_armor.h)
  // conceptually, but is NOT shared across the class boundary: this is a
  // leaf-local constant for a leaf-local guard.
  static constexpr float kReconfigureRestVelocity = 5.0f;  // [mm/s] mirrors MotorArmor's own kRestVelocity at-rest threshold

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

}  // namespace Devices
