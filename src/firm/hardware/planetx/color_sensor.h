// color_sensor.h — Hardware::ColorSensorLeaf: the internal leaf for RGBC color
// sensing. Supports two chip variants: an alt/"PlanetX" chip at I2C address
// 0x43 (primary) and an APDS9960 at 0x39 (fallback). The loop constructs and
// drives this leaf directly — there is no separate handle class.
//
// Mirrors nezha_motor.h/otos.h's leaf shape: a begin-style detection entry
// point, present()/connected(), and a non-blocking tick(nowUs) that
// publishes into reading().
//
// --- Re-wake-each-retry detection (PRESERVED) ---
// Color detection must re-assert its wake registers each retry: begin()
// re-asserts the ALT chip's wake writes (0x81=0xCA, 0x80=0x17) INSIDE every
// retry, settles ~50ms, then checks the 16-bit value at 0xA4/0xA5 is
// non-zero — a wake-once version fails to detect a chip that was still
// powering up on the first attempt (docs/knowledge/encoders-read-zero-i2c-
// bus-hang.md). beginStep(nowUs) is a non-blocking single-step state
// machine driven by Hal::Clock, paced instead of blocked on
// fiber_sleep(50) — the caller (the fiber detection preamble) calls it once
// per cycle until detectDone() is true (see the "No leaf sleeps or blocks"
// invariant, DESIGN.md §3): no devices/ leaf may fiber_sleep() itself.
//
// --- Steady-state reads (tick(), below) ---
// Uses a non-blocking poll (a single cheap register peek, decode only if
// the chip's own data-ready condition is met THIS call, otherwise leave the
// cached reading alone and retry next call) rather than a blocking read
// (which would fiber_sleep(100) the ALT chip's integration window, or poll
// the APDS STATUS register up to 250ms/50 tries). tick() also carries a
// readDue(nowUs) rate-limit gate sourced from ColorConfig::lagColor — the
// leaf's own "sensor polling budget."
//
// --- ColorSensorLeaf / LineSensorLeaf naming ---
// Both leaves carry a `Leaf` suffix (rather than the bare `ColorSensor`/
// `LineSensor`) to leave those bare names free for a future public handle
// type in the same `Devices` namespace, should one ever be reintroduced —
// the loop currently constructs and drives both leaves directly.
#pragma once

#include <cstdint>

#include "hal/device_config.h"
#include "hal/device_types.h"
#include "hal/color_sensor.h"
#include "hal/i2c_bus.h"

namespace Hardware {

constexpr uint8_t kColorDeviceAddrApds = 0x39;
constexpr uint8_t kColorDeviceAddrAlt = 0x43;

class ColorSensorLeaf : public Hal::ColorSensor {
 public:
  ColorSensorLeaf(Hal::I2CBus& bus, const Hal::ColorConfig& config);

  // Non-blocking single detection step. Call once per fiber cycle (DB-007's
  // detection preamble) until detectDone() is true; a no-op once it is.
  //
  // Phase 1 (AltProbe): up to kMaxAltAttempts attempts, kAltRetryPeriod
  // apart (paced by nowUs, never a real sleep). Each due attempt re-writes
  // the ALT chip's wake registers then checks 0xA4/0xA5 (see file header) —
  // non-zero means found: present()/connected() become true, isAlt_ true,
  // phase Done.
  // Phase 2 (ApdsProbe): entered once AltProbe is exhausted; exactly ONE
  // attempt (the APDS fallback has no retry loop) — write ENABLE
  // off (0x80=0x00) and read it back; 0x00 means the APDS9960 answered:
  // initApds() runs its register program, present()/connected() become
  // true, phase Done. Either way (found or not), phase becomes Done after
  // this one attempt — a caller must stop calling beginStep() once
  // detectDone() is true.
  void beginStep(uint64_t nowUs) override;  // [us]
  bool detectDone() const override;

  // present()/connected(): same distinction as otos.h.
  // present() is set once by beginStep() reaching a successful terminal
  // state and never re-evaluated; connected() is the live, per-tick()
  // bus-health result (see tick()'s own comment).
  bool present() const override;
  bool connected() const override;

  // True if a real bus read is due: no real read has ever happened, or at
  // least (ColorConfig::lagColor * 1000) [us] have elapsed since the last
  // one. Pure function of this leaf's own bookkeeping — no bus traffic.
  bool readDue(uint64_t nowUs) const;  // [us]

  // The leaf's one steady-state bus-touching entry point. No-op (no bus
  // traffic) if beginStep() never found a chip, or if the call arrives
  // before readDue() is true (rate-limited, same "always retried, never
  // permanently latched" contract as Otos::tick()). Publishes a fresh
  // Hal::ColorReading (readingFresh() true) only when the chip's own data-ready
  // condition was met THIS call; a poll miss (chip not ready yet) or a bus
  // failure both leave readingFresh() false and reading() unchanged,
  // connected() reflecting the latter case only.
  void tick(uint64_t nowUs) override;  // [us]

  Hal::ColorReading reading() const override;
  bool readingFresh() const override;

  // [counts] the ADC's full-scale value at the CURRENT integration time --
  // 1025 per 2.78ms integration step, i.e. 1025 * (256 - ATIME), saturating
  // at the register's own 16-bit ceiling.
  //
  // Callers that squeeze a reading into fewer bits MUST scale against this,
  // not against 65535. At the shipped ATIME (252 -> 11.1ms) full scale is
  // only 4100, so a naive `>> 8` leaves just 16 of 256 output codes usable
  // and rounds every channel below 256 counts to zero. Measured on tovez
  // 2026-08-08 before this was exposed: r/g/b pinned at 0 and c at 2 across
  // 375 consecutive frames, zero spread -- a sensor that reads fine but
  // could not express anything.
  uint32_t fullScale() const override;

 private:
  enum class DetectPhase : uint8_t { AltProbe, ApdsProbe, Done };

  Hal::I2CBus& bus_;
  Hal::ColorConfig config_;

  DetectPhase phase_ = DetectPhase::AltProbe;
  int altAttempts_ = 0;
  bool hasAttempted_ = false;
  uint64_t lastAttemptUs_ = 0;  // [us]

  bool initialized_ = false;  // present()'s backing field
  bool connected_ = false;
  bool isAlt_ = false;

  Hal::ColorReading cachedReading_{};
  bool readingFresh_ = false;

  uint64_t lastReadUs_ = 0;  // [us] time of the most recent real tick() read
  bool hasRead_ = false;

  static constexpr uint8_t kMaxAltAttempts = 20;
  static constexpr uint64_t kAltRetryPeriod = 50000;      // [us]
  static constexpr uint32_t kDefaultLagColor = 100;       // [ms]
  static constexpr uint8_t kDefaultIntegration = 252;     // ATIME register default
  static constexpr uint8_t kDefaultGain = 0x03;           // CONTROL register default

  void initApds();

  // writeReg8()/readReg8()/readReg16()/readReg16Alt() ignore the I2C
  // transaction status. The *Status() variants below are used by tick()'s
  // steady-state path (which, like NezhaMotor/Hal::Otos, DOES track bus health
  // for connected()) AND by beginStep()'s ApdsProbe phase — a
  // status-ignoring probe read there once latched present()==true on a NAK
  // (a status-ignoring readback decodes a NAK as en==0x00, exactly the
  // "detected" condition), so that phase must always use the *Status()
  // variant.
  void writeReg8(uint8_t addr, uint8_t reg, uint8_t val);
  uint8_t readReg8(uint8_t addr, uint8_t reg);
  uint16_t readReg16(uint8_t addr, uint8_t regLo);
  uint16_t readReg16Alt(uint8_t regLo);  // alt-chip single-byte protocol, hardcoded to kColorDeviceAddrAlt

  bool readReg8Status(uint8_t addr, uint8_t reg, uint8_t& out);
  bool readReg16Status(uint8_t addr, uint8_t regLo, uint16_t& out);
  bool readReg16AltStatus(uint8_t regLo, uint16_t& out);
};

}  // namespace Hardware
