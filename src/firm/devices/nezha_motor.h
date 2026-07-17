// nezha_motor.h — Devices::NezhaMotor: the concrete internal leaf for one
// channel of the PlanetX Nezha V2 motor controller. Owns the register map,
// split-phase 0x46 encoder sequencing, slew limiting, and wedge-latch
// signal (see nezha_motor.cpp for the per-function notes); embeds one
// velocity PID (Devices::MotorVelocityPid) per motor.
//
// Deliberate scope-downs from a full motor abstraction:
//   - No message-plane surface (apply()/state()/capabilities()/
//     msg::MotorCommand) — msg:: is unreachable under the isolation
//     invariant; the loop constructs and drives this leaf directly.
//   - No POSITION mode (the onboard 0x5D absolute-angle move) — this leaf
//     only covers velocity-PID and raw-duty modes (see DESIGN.md §3).
//   - No additive velocity feedforward beyond Gains::kff — VELOCITY-mode
//     duty is pid_.compute()'s output directly.
//   - PID on/off: setPidEnabled()/setDuty() let Mode::Active cover both the
//     PID-chase and raw-duty-passthrough cases, selected at tick() time by
//     pidEnabled_ rather than by which setter was last called (see tick()'s
//     own comment). Armor (armoredWrite()) applies identically either way.
//   - Time seam: tick() takes a single `uint64_t nowUs` [us] parameter
//     rather than reading a clock internally — Devices::Clock (clock.h) is
//     the fiber-level time seam, scoped to "the fiber's OWN cycle-level
//     time reads ... not the bus's clearance windows" (a DIFFERENT seam
//     from I2CBus's own internal clearance-timer bookkeeping — see
//     i2c_bus.h). This leaf takes "now" as a plain parameter supplied by
//     its caller (ultimately the loop's own Clock instance) — fully
//     deterministic for a host harness with zero clock coupling. Armor
//     timing (dwell/rest-tracking) runs in ms, matching MotorConfig's
//     documented [ms] reversalDwell unit; the write-rate throttle inside
//     writeRawDuty() runs in us, reading the SAME nowUs this tick already
//     cached in lastTickUs_ before dispatch — see writeRawDuty()'s own
//     comment for why that is exactly equivalent to a fresh clock read at
//     that point.
//
// Design/rationale: DESIGN.md.
#pragma once

#include <cstdint>

#include "devices/device_config.h"
#include "devices/device_types.h"
#include "devices/i2c_bus.h"
#include "devices/motor_armor.h"
#include "devices/velocity_pid.h"

namespace Devices {

// 7-bit I2C address shared by all four Nezha V2 motor channels (the
// motorId byte in each frame selects the channel, not the address).
constexpr uint8_t kNezhaDeviceAddr = 0x10;

class NezhaMotor : public MotorArmor {
 public:
  NezhaMotor(I2CBus& bus, const MotorConfig& config);

  // Primes the encoder: the Nezha 0x46 register sits frozen at 0 until the
  // chip receives its first atomic read transaction (calls hardReset()).
  // The fiber preamble calls this once per port before the cycle starts.
  void begin();

  // Split-phase phase 1, public entry point. Wraps requestEncoder() so the
  // loop's own cycle can request this port's encoder sample without
  // reaching into NezhaMotor's private register-verb surface. NOT a
  // MotorArmor virtual: request/collect splitting is a Nezha-specific
  // consequence of four ports sharing one device address (0x10), not a
  // universal concept a future leaf would need.
  void requestSample();

  // --- Primitive setters — stage the command; tick() executes it. ---
  void setVelocity(float velocity);   // [mm/s] signed — PID target (consumed only while PID is enabled)
  void setDuty(float duty);           // [-1, 1] raw duty target (consumed only while PID is disabled)
  void setNeutral(Neutral mode);      // coast / brake — Nezha maps both to the same 0x60 speed-0 write (no distinct brake register)
  void setPidEnabled(bool on);        // default true — armor applies in both modes

  // Live gain-apply: mutates this motor's velocity-PID gains (and,
  // optionally, its wheel-travel calibration) in place -- no reflash, no
  // I2C side effect (MotorVelocityPid::compute() reads config_.velGains
  // fresh every tick). Parameters are exclusively Devices-local types
  // (Gains, Opt<float>) -- never the wire msg::MotorConfigPatch --
  // preserving the isolation invariant: RobotLoop (app/, which already
  // includes messages/...) is the one legitimate translation boundary
  // between the wire patch and this call. `travelCalib` defaults to absent
  // (has=false) -- pass it only when the caller means to also update
  // config_.wheelTravelCalib.
  void applyGains(const Gains& gains, Opt<float> travelCalib = {});

  // Current live gains -- lets a caller (RobotLoop's CONFIG dispatch) merge
  // a partially-populated wire patch against whatever this motor is
  // actually running today, field by field, rather than clobbering an
  // absent field back to some default.
  const Gains& gains() const { return config_.velGains; }

  // Velocity-estimator selection (bench A/B). mode 0 = EMA
  // (velFiltAlpha — the shipped/default behavior); mode 1 = least-squares
  // line-fit slope over the last `window` FRESH position samples
  // (Savitzky-Golay order 1). The line fit rejects encoder-quantization noise
  // with less lag than an equivalent heavier EMA. `window` is clamped to
  // [3, kMaxVelWindow]; it is ignored in mode 0. Live-settable so the bench
  // can compare EMA vs. line-fit(N) on the stand without reflashing.
  void setVelEstimator(uint8_t mode, uint8_t window);
  uint8_t velEstMode() const { return velEstMode_; }
  uint8_t velWindow() const { return velWindow_; }

  // Output duty smoothing (bench). Applies a boxcar moving
  // average of the last `window` PID duty outputs before the armored write,
  // to smooth the visible/electrical duty jitter (the write path quantizes
  // duty to integer percent, so a jittering PID output toggles the written
  // percent by +/-1-2 LSB every ~40ms). window 1 = off (default, no
  // averaging — unchanged behavior). Clamped to [1, kMaxDutyAvg]. Adds a
  // small amount of control-output lag (~window/2 cycles); live-settable so
  // the bench can find the point where smoothing stops being worth the lag.
  void setDutyAvg(uint8_t window);
  uint8_t dutyAvgWindow() const { return dutyAvgWindow_; }

  // --- Primitive getters (MotorArmor overrides) ---
  float position() const override;      // [mm]
  float velocity() const override;      // [mm/s] signed, filtered
  float appliedDuty() const override;   // [-1, 1]

  bool connected() const { return connected_; }
  uint32_t encGlitchCount() const { return encGlitchCount_; }   // cumulative rejected samples (never resets)
  bool pidEnabled() const { return pidEnabled_; }

  // tick() — the leaf's 5-step call-order contract (see nezha_motor.cpp):
  //   1. processResetIfPending(nowMs)  — base armor policy, before this
  //      tick's encoder sample.
  //   2. sample + cache this motor's own encoder (device-specific).
  //      Velocity/glitch computation is gated on a FRESHNESS check (the
  //      collected raw count differs from the last FRESH raw count) --
  //      the Nezha brick's register refreshes far slower (~80ms) than the
  //      fiber's own cycle (~16ms), so most cycles re-collect the same
  //      value; see nezha_motor.cpp's tick() comment.
  //   3. updateWedgeDetector()         — base armor policy; reads
  //      position()/appliedDuty(), both now reflecting this tick's fresh
  //      sample and last tick's write.
  //   4. mode dispatch — Mode::Active routes through armoredWrite() (PID or
  //      raw duty, per pidEnabled_); Mode::Neutral writes 0 via
  //      armoredWrite(); Mode::None dispatches nothing.
  //   5. updateRestTracking()          — base armor policy; reads
  //      velocity() and lastRequestedDuty_, the latter possibly just
  //      updated by step 4.
  void tick(uint64_t nowUs);   // [us]

 protected:
  // --- Device-specific armor primitives (MotorArmor) ---
  void writeRawDuty(float duty) override;    // write path; the reversal-exemption branch is unreachable here — armoredWrite() never forwards a raw sign flip
  void hardReset() override;                 // median-of-3 + readback-verify + retry
  void softRebaseline() override;            // software-only rebaseline

 private:
  // Mode::Active covers both the raw-duty and velocity-PID cases — tick()
  // picks PID vs. raw duty at dispatch time via pidEnabled_ rather than by
  // which setter was last called, so setVelocity()/setDuty() can both be
  // staged and the live pidEnabled_ flag decides which one actually drives
  // the write each tick (the bench/DEV surface's whole point).
  enum class Mode : uint8_t { None, Active, Neutral };

  // ---- Wiring ----
  I2CBus& bus_;
  MotorConfig config_;

  // ---- Staged command (set by the primitive setters; executed by tick()) ----
  Mode mode_ = Mode::None;
  float velocityTarget_ = 0.0f;               // [mm/s]
  float dutyTarget_ = 0.0f;                   // [-1, 1]
  Neutral neutralTarget_ = Neutral::Coast;
  bool pidEnabled_ = true;                    // default

  // ---- tick() encoder-sample cache ----
  float lastPosition_ = 0.0f;          // [mm]
  float filteredVelocity_ = 0.0f;      // [mm/s] EMA-filtered (velFiltAlpha); fed to the embedded PID and velocity()
  uint64_t lastTickUs_ = 0;            // [us] this leaf's own time seam — see file header; per-TICK dt, feeds ONLY the embedded PID's dt (step 4)
  bool hasLastTick_ = false;
  bool connected_ = false;

  // ---- Fresh-sample tracking (tick() step 2's freshness gate) ----
  // The Nezha brick's 0x46 register refreshes far slower (~80ms) than
  // the loop's own cycle (~16ms): most cycles re-collect
  // the SAME raw count. Velocity/glitch computation runs ONLY when
  // collectEncoder() returns a raw count different from the last FRESH raw
  // count, using the elapsed time SINCE THAT sample (lastFreshUs_) instead
  // of this tick's own (much shorter) dt — see nezha_motor.cpp's tick() for
  // the full rationale and the hardware-confirmed bug this fixes
  // (filteredVelocity_ permanently starved / rejected as a false glitch).
  int32_t lastFreshRawEnc_ = 0;   // [tenths of degrees, offset-corrected] raw count at the last FRESH sample
  uint64_t lastFreshUs_ = 0;      // [us] this leaf's own time seam, timestamp of the last FRESH sample
  bool hasFreshSample_ = false;   // false until the first fresh sample is anchored; also cleared by hardReset()/softRebaseline()

  // ---- Source-side encoder outlier rejection (tick() step 2's
  // position-step plausibility gate) ----
  uint32_t encGlitchCount_ = 0;   // cumulative rejected samples (never resets)
  uint8_t encGlitchStreak_ = 0;   // consecutive rejections; re-anchor at kGlitchStreakAccept

  // ---- Velocity estimator (sprint 101 bench A/B) ----
  // A short ring of the most recent ACCEPTED fresh (time, position) samples.
  // mode 1 fits a least-squares line through the last `velWindow_` of them and
  // takes the slope as the velocity; mode 0 ignores the ring and uses the
  // legacy 2-point + EMA path. Cleared on any encoder discontinuity
  // (hardReset()/softRebaseline()) so a fit never spans a re-anchor.
  static constexpr uint8_t kVelEstEma = 0;
  static constexpr uint8_t kVelEstLineFit = 1;
  static constexpr uint8_t kMaxVelWindow = 8;
  uint8_t velEstMode_ = kVelEstEma;   // default: shipped EMA behavior
  uint8_t velWindow_ = 6;             // line-fit sample count, clamped [3, kMaxVelWindow]
  uint64_t velWinT_[kMaxVelWindow] = {};   // [us] fresh-sample times (ring)
  float velWinP_[kMaxVelWindow] = {};      // [mm] fresh-sample positions (ring)
  uint8_t velWinCount_ = 0;                // valid entries in the ring (<= kMaxVelWindow)
  uint8_t velWinHead_ = 0;                 // next write slot

  // ---- Output duty smoothing (sprint 101 bench) ----
  // Boxcar moving average of the last dutyAvgWindow_ PID duty outputs, applied
  // just before armoredWrite(). window 1 = off (default). Ring cleared on an
  // encoder discontinuity alongside the velocity window (clearVelWindow()).
  static constexpr uint8_t kMaxDutyAvg = 8;
  uint8_t dutyAvgWindow_ = 1;              // 1 = off (default, unchanged behavior)
  float dutyRing_[kMaxDutyAvg] = {};       // [-1,1] recent PID duty outputs (ring)
  uint8_t dutyRingCount_ = 0;
  uint8_t dutyRingHead_ = 0;

  // ---- Write path ----
  int8_t lastWrittenPct_ = -128;        // [%] sentinel (outside +/-100) forces the first write
  uint64_t lastWriteTimeUs_ = 0;        // [us]

  // ---- Embedded velocity PID ----
  MotorVelocityPid pid_;

  // ---- Encoder software offset / failure-hold state ----
  int32_t encOffset_ = 0;               // [tenths of degrees]
  int32_t lastGoodRawEnc_ = 0;          // held on I2C failure
  bool pendingEncRequestOk_ = true;     // requestEncoder()/collectEncoder() pairing

  // ---- Register-map wire constants ----
  static constexpr uint8_t kDirCw = 1;      // positive speed from chip perspective
  static constexpr uint8_t kDirCcw = 2;     // negative speed from chip perspective
  static constexpr float kDefaultSlewRate = 25.0f;   // default max |delta PWM| per write

  // ---- Private helpers: write path ----
  // Returns the CODAL status from bus_.write() (0/kOk == success):
  // writeRawDuty() commits lastWrittenPct_/lastWriteTimeUs_ ONLY when this
  // status is kOk, so a NAK'd write is retried next tick instead of being
  // latched as "already written."
  int writeMotorRun(uint8_t direction, uint8_t speed);   // writes the 0x60 motor-run command

  // ---- Private helpers: velocity estimator ----
  void pushVelSample(uint64_t t, float position);   // [us] [mm] append an accepted fresh sample to the ring
  void clearVelWindow();                             // reset the vel + duty rings on an encoder discontinuity
  float lineFitVelocity() const;                     // [mm/s] least-squares slope over the last velWindow_ samples
  float averageDuty(float duty);                     // [-1,1] boxcar moving average of the last dutyAvgWindow_ duties

  // ---- Private helpers: encoder read paths ----
  int32_t readEncoderAtomicRaw();   // one-off sample: preClear/postClear-settled 0x46 write -> read
  void requestEncoder();            // split-phase phase 1; wrapped by the public requestSample() above
  int32_t collectEncoder();         // split-phase phase 2; wired into tick()'s step 2
};

}  // namespace Devices
