// color_sensor.h — Devices::ColorSensor: the internal leaf for RGBC color
// sensing. Supports two chip variants: an alt/"PlanetX" chip at I2C address
// 0x43 (primary) and an APDS9960 at 0x39 (fallback).
//
// Ticket DB-006 (device-bus-tickets.md). Ported from
// source_old/hal/real/ColorSensor.{h,cpp} into the greenfield
// `source/devices/` subsystem (namespace `Devices`), per clasi/issues/
// device-bus-fiber-owned-self-contained-device-subsystem.md's "Shape" —
// "Line and color sensing don't exist in the new tree yet." Mirrors
// nezha_motor.h/otos.h's leaf shape (DB-004/DB-005): a begin-style detection
// entry point, present()/connected() (sprint-099 distinction, carried
// forward per otos.h's own precedent), and a non-blocking tick(nowUs) that
// publishes into reading().
//
// --- Re-wake-each-retry detection (PRESERVED) ---
// docs/knowledge/encoders-read-zero-i2c-bus-hang.md's "Color detection must
// re-assert its wake registers each retry" lesson: the pre-port begin()
// re-asserts the ALT chip's wake writes (0x81=0xCA, 0x80=0x17) INSIDE every
// retry, settles ~50ms, then checks the 16-bit value at 0xA4/0xA5 is
// non-zero — a wake-once version fails to detect a chip that was still
// powering up on the first attempt. That exact write-then-check sequence is
// preserved verbatim below (beginStep()'s AltProbe phase). What changes is
// the RETRY PACING: the pre-port file's `for (...) { ...; fiber_sleep(50); }`
// blocking loop (up to 20 * 50ms = 1s worst case) becomes beginStep(nowUs), a
// non-blocking single-step state machine driven by Devices::Clock — the
// caller (DB-007's fiber detection preamble) calls it once per cycle until
// detectDone() is true, exactly matching the issue's own description
// ("Because this runs in the fiber, retries no longer freeze the control
// loop") and device-bus-tickets.md's DB-006 "NON-BLOCKING reads only" note:
// no source/devices/ leaf may fiber_sleep() itself (there is no Sleeper
// wired to this leaf, only a plain nowUs parameter — the same time-seam
// convention DB-004/DB-005 already established).
//
// --- Steady-state reads (tick(), below) ---
// Ports pollRGBC() — NOT the blocking readRGBC() (which fiber_sleep(100)s
// the ALT chip's integration window or polls the APDS STATUS register up to
// 250ms/50 tries). pollRGBC() was already the pre-port driver's own
// non-blocking counterpart ("Use this in time-critical loops instead of
// readRGBC()."): a single cheap register peek, decode only if the chip's own
// data-ready condition is met THIS call, otherwise leave the cached reading
// alone and retry next call. tick() also carries a readDue(nowUs) rate-limit
// gate sourced from ColorConfig::lagColor — this is not a NEW invention: the
// pre-port architecture's own `Sensors::periodic()` used cfg.lagColor as
// exactly this "sensor polling budget" gate (see
// source_old/subsystems/sensors/SensorsConfig.h's own "polling budget, ms"
// comment on the field this config's lagColor descends from) before this
// port folded that gate into the leaf itself.
#pragma once

#include <cstdint>

#include "devices/device_config.h"
#include "devices/device_types.h"
#include "devices/i2c_bus.h"

namespace Devices {

constexpr uint8_t kColorDeviceAddrApds = 0x39;
constexpr uint8_t kColorDeviceAddrAlt = 0x43;

class ColorSensor {
 public:
  ColorSensor(I2CBus& bus, const ColorConfig& config);

  // Non-blocking single detection step. Call once per fiber cycle (DB-007's
  // detection preamble) until detectDone() is true; a no-op once it is.
  //
  // Phase 1 (AltProbe): up to kMaxAltAttempts attempts, kAltRetryPeriod
  // apart (paced by nowUs, never a real sleep). Each due attempt re-writes
  // the ALT chip's wake registers then checks 0xA4/0xA5 (see file header) —
  // non-zero means found: present()/connected() become true, isAlt_ true,
  // phase Done.
  // Phase 2 (ApdsProbe): entered once AltProbe is exhausted; exactly ONE
  // attempt (the pre-port fallback has no retry loop either) — write ENABLE
  // off (0x80=0x00) and read it back; 0x00 means the APDS9960 answered:
  // initApds() runs its register program, present()/connected() become
  // true, phase Done. Either way (found or not), phase becomes Done after
  // this one attempt — a caller must stop calling beginStep() once
  // detectDone() is true.
  void beginStep(uint64_t nowUs);  // [us]
  bool detectDone() const;

  // present()/connected(): sprint-099 distinction, ported from otos.h.
  // present() is set once by beginStep() reaching a successful terminal
  // state and never re-evaluated; connected() is the live, per-tick()
  // bus-health result (see tick()'s own comment).
  bool present() const;
  bool connected() const;

  // True if a real bus read is due: no real read has ever happened, or at
  // least (ColorConfig::lagColor * 1000) [us] have elapsed since the last
  // one. Pure function of this leaf's own bookkeeping — no bus traffic.
  bool readDue(uint64_t nowUs) const;  // [us]

  // The leaf's one steady-state bus-touching entry point. No-op (no bus
  // traffic) if beginStep() never found a chip, or if the call arrives
  // before readDue() is true (rate-limited, same "always retried, never
  // permanently latched" contract as Otos::tick()). Publishes a fresh
  // ColorReading (readingFresh() true) only when the chip's own data-ready
  // condition was met THIS call; a poll miss (chip not ready yet) or a bus
  // failure both leave readingFresh() false and reading() unchanged,
  // connected() reflecting the latter case only.
  void tick(uint64_t nowUs);  // [us]

  ColorReading reading() const;
  bool readingFresh() const;

 private:
  enum class DetectPhase : uint8_t { AltProbe, ApdsProbe, Done };

  I2CBus& bus_;
  ColorConfig config_;

  DetectPhase phase_ = DetectPhase::AltProbe;
  int altAttempts_ = 0;
  bool hasAttempted_ = false;
  uint64_t lastAttemptUs_ = 0;  // [us]

  bool initialized_ = false;  // present()'s backing field
  bool connected_ = false;
  bool isAlt_ = false;

  ColorReading cachedReading_{};
  bool readingFresh_ = false;

  uint64_t lastReadUs_ = 0;  // [us] time of the most recent real tick() read
  bool hasRead_ = false;

  static constexpr uint8_t kMaxAltAttempts = 20;         // matches the ported retry count
  static constexpr uint64_t kAltRetryPeriod = 50000;      // [us] matches the ported fiber_sleep(50)
  static constexpr uint32_t kDefaultLagColor = 100;       // [ms] matches source_old DefaultConfig p.lagColor
  static constexpr uint8_t kDefaultIntegration = 252;     // matches initApds()'s ported ATIME literal
  static constexpr uint8_t kDefaultGain = 0x03;           // matches initApds()'s ported CONTROL literal

  void initApds();

  // writeReg8()/readReg8()/readReg16()/readReg16Alt() ignore the I2C
  // transaction status — matches the pre-port beginStep()/initApds() code,
  // which never checked it either. The *Status() variants below are used
  // ONLY by tick()'s steady-state path, which (like NezhaMotor/Otos) DOES
  // track bus health for connected().
  void writeReg8(uint8_t addr, uint8_t reg, uint8_t val);
  uint8_t readReg8(uint8_t addr, uint8_t reg);
  uint16_t readReg16(uint8_t addr, uint8_t regLo);
  uint16_t readReg16Alt(uint8_t regLo);  // alt-chip single-byte protocol, hardcoded to kColorDeviceAddrAlt

  bool readReg8Status(uint8_t addr, uint8_t reg, uint8_t& out);
  bool readReg16Status(uint8_t addr, uint8_t regLo, uint16_t& out);
  bool readReg16AltStatus(uint8_t regLo, uint16_t& out);
};

}  // namespace Devices
