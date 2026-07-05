// nezha_motor.h — NezhaMotor: the concrete Hal::Motor leaf for one channel
// of the PlanetX Nezha V2 motor controller.
//
// Ports the register map, split-phase 0x46 encoder sequencing, slew
// limiting, and wedge-latch signal from source_old/hal/real/Motor.cpp
// (sequencing preserved exactly — see nezha_motor.cpp for the byte-for-byte
// notes), and embeds the velocity PID directly (no separate controller
// object — see architecture-update.md Design Rationale / Control-
// architecture decision 1), following the control law in
// source_old/control/VelocityController.cpp.
//
// Instantiated per-port: config.port selects the vendor motorId byte
// (1..4) sent in every frame — NOT a left/right pair (Design Rationale 3).
// All encoder plumbing and raw register verbs are private; the only public
// surface is the Hal::Motor faceplate.
//
// Sprint 078 refit — the reversal-latch armor (zero-dwell reversal, output
// deadband, standstill-guarded resets, motion-qualified wedge reporting) is
// now shared Hal::Motor base-class policy (source/hal/capability/motor.h),
// not owned here. This class supplies only the four device-specific
// protected primitives the base calls (writeRawDuty()/hardReset()/
// softRebaseline()/configureDevice()) plus its own write shaping (40 ms
// throttle, ±slew_rate) and encoder sampling — see architecture-update.md,
// "The base/leaf split — exact contract".
#pragma once

#include <stdint.h>

#include "hal/capability/motor.h"
#include "com/i2c_bus.h"
#include "messages/motor.h"

namespace Hal {

// 7-bit I2C address shared by all four Nezha V2 motor channels (the
// motorId byte in each frame selects the channel, not the address).
// Promoted from NezhaMotor's former private kAddr constant (sprint 079-004)
// to a namespace Hal constant so NezhaHardware's brick flip-flop sequencer can
// name it too (its bus_.clear(kNezhaDeviceAddr) gate — architecture-
// update.md's "clear()'s address convention" section — must use the SAME
// bare 7-bit value every NezhaMotor register write/read shifts left by one
// to form the 8-bit wire address).
constexpr uint8_t kNezhaDeviceAddr = 0x10;

class NezhaMotor : public Motor {
 public:
  NezhaMotor(I2CBus& bus, const msg::MotorConfig& config);

  // Primes the encoder: the Nezha 0x46 register sits frozen at 0 until the
  // chip receives its first atomic read transaction. Ports source_old's
  // Motor::begin() (which calls resetEncoder()) exactly; NezhaHardware::begin()
  // calls this once per port before the main loop starts.
  void begin() override;

  // Split-phase phase 1, public entry point — sprint 079-004. Wraps the
  // already-ported requestEncoder() so NezhaHardware's brick flip-flop sequencer
  // (subsystems/nezha_hardware.cpp) can request this port's encoder sample without reaching
  // into NezhaMotor's private register-verb surface. NOT a Hal::Motor
  // virtual: request/collect splitting is a Nezha-specific consequence of
  // four ports sharing one device address (0x10), not a universal HAL
  // concept a future SimMotor leaf would need.
  void requestSample();

  // --- Primitive setters (Hal::Motor) ---
  void setDutyCycle(float dutyCycle) override;         // [-1, 1]
  void setVoltage(float voltage) override;              // [V] unsupported — capabilities().voltage == false
  void setVelocity(float velocity) override;            // [mm/s] signed
  void setPosition(float position) override;            // [deg]
  void setNeutral(msg::Neutral mode) override;
  void setFeedforward(float feedforward) override;      // [V]

  // --- Primitive getters (Hal::Motor) ---
  float position() const override;       // [mm]
  float velocity() const override;       // [mm/s] signed, filtered
  float appliedDuty() const override;    // [-1, 1]
  bool connected() const override;

  // --- Faceplate verbs (Hal::Motor) ---
  void tick(uint32_t now) override;      // [ms]
  msg::MotorCapabilities capabilities() const override;

 protected:
  // --- Device-specific armor primitives (Hal::Motor, sprint 078) ---
  void writeRawDuty(float duty) override;    // ported Motor::setSpeed(), minus the reversal-exemption branch (structurally unreachable — see .cpp)
  void hardReset() override;                 // ported Motor::resetEncoder() (median-of-3 + readback-verify + retry), unchanged
  void softRebaseline() override;            // new — ported from source_old's Motor::rebaselineSoft() (064-003)
  void configureDevice(const msg::MotorConfig& config) override;   // slew_rate defaulting etc., minus the two base-owned armor fields

 private:
  enum class Mode : uint8_t { NONE, DUTY, VELOCITY, POSITION, NEUTRAL };

  // ---- Wiring ----
  I2CBus& bus_;
  msg::MotorConfig config_;

  // ---- Staged command (set by the primitive setters; executed by tick()) ----
  Mode mode_ = Mode::NONE;
  float dutyTarget_ = 0.0f;                          // [-1, 1]
  float velocityTarget_ = 0.0f;                       // [mm/s]
  float positionTarget_ = 0.0f;                       // [deg]
  bool positionCommandPending_ = false;               // write-on-change gate for 0x5D
  msg::Neutral neutralTarget_ = msg::Neutral::COAST;
  float feedforward_ = 0.0f;                          // additive term, folded into the raw PID output before the final clamp

  // ---- tick() encoder-sample cache (ported from Motor::tick()'s
  // _lastPosition/_lastVelocityMmps) ----
  float lastPosition_ = 0.0f;          // [mm]
  float filteredVelocity_ = 0.0f;      // [mm/s] EMA-filtered (vel_filt_alpha); fed to the embedded PID and velocity()
  uint32_t lastTick_ = 0;            // [ms]
  bool hasLastTick_ = false;
  bool connected_ = false;

  // ---- Write path (ported from Motor::setSpeed()) ----
  int8_t lastWrittenPct_ = -128;        // [%] sentinel (outside ±100) forces the first write
  uint64_t lastWriteTimeUs_ = 0;        // [us]

  // ---- Embedded velocity PID state (see nezha_motor.cpp for the control
  // law derivation from VelocityController::update()) ----
  float integral_ = 0.0f;

  // ---- Encoder software offset / failure-hold state (ported from Motor) ----
  int32_t encOffset_ = 0;               // [tenths of degrees]
  int32_t lastGoodRawEnc_ = 0;          // held on I2C failure (CR-03 pattern)
  bool pendingEncRequestOk_ = true;     // requestEncoder()/collectEncoder() pairing

  // ---- Register-map wire constants ----
  // kAddr promoted to the namespace-Hal kNezhaDeviceAddr constant above
  // (sprint 079-004) — shared with NezhaHardware's flip-flop sequencer.
  static constexpr uint8_t kDirCw = 1;      // positive speed from chip perspective
  static constexpr uint8_t kDirCcw = 2;     // negative speed from chip perspective
  static constexpr float kDefaultSlewRate = 25.0f;   // architecture-update.md Design Rationale 2: ports kMaxDeltaPwmPerWrite's default

  // ---- Private helpers: write path ----
  void writeMotorRun(uint8_t direction, uint8_t speed);  // ported Motor::writeMotorCmd() (0x60)
  void writePositionMove(float positionDeg);            // ported Motor::moveToAngle() (0x5D)

  // ---- Private helpers: encoder read paths (all ported from Motor.cpp) ----
  // readEncoderSettle() (the fused, always-blocking write+4ms-spin+read) is
  // DELETED (sprint 079-004) — its sole caller (tick()'s step 2) now calls
  // collectEncoder() instead; see nezha_motor.cpp's tick() for the mapping.
  int32_t readEncoderAtomicRaw();       // one-off sample: 4ms pre-idle -> 0x46 write -> 4ms settle -> read
  void requestEncoder();                // split-phase phase 1 (ported byte-for-byte); wrapped by the public requestSample() above
  int32_t collectEncoder();             // split-phase phase 2 (ported byte-for-byte); now wired into tick()'s step 2 (sprint 079-004)

  // ---- Private helpers: control ----
  float runVelocityPid(float target, float measured, float dt);   // [mm/s] [mm/s] [s] -> duty [-1,1]

  // ---- Vendor register wrappers ported for completeness (matching
  // source_old's coverage) but NOT reachable from the public faceplate
  // this sprint — see ticket 003 acceptance criteria. ----
  void timedMove(uint8_t dir, int16_t value, uint8_t mode);   // 0x70
  void resetHome();                                            // 0x1D
  void setGlobalSpeed(uint8_t speed);                          // 0x77 (board-global)
  bool readVersion(uint8_t& maj, uint8_t& min, uint8_t& patch); // 0x88 (board-global)
};

}  // namespace Hal
