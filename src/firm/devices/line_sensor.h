// line_sensor.h — Devices::LineSensorLeaf: the internal leaf for the PlanetX
// 4-channel line sensor, I2C address 0x1A. The loop constructs and drives
// this leaf directly — there is no separate handle class.
//
// Mirrors color_sensor.h's leaf shape: a non-blocking beginStep(nowUs)
// detection state machine, present()/connected(), and a non-blocking
// tick(nowUs) that publishes a combined raw+normalized LineReading.
//
// Protocol: write a 1-byte channel index (0-3), then read 1 byte of
// grayscale data (0 = white, 255 = black approximately) -- FOUR such
// write/read pairs per full sample (readRaw(), below) — already
// non-blocking (no fiber_sleep anywhere in the read path); only
// detection's retry loop needed restructuring into beginStep() (see its
// own comment).
//
// Calibration (captureCalibMin()/captureCalibMax()/setSmoothingAlpha(),
// below) operates on a local mutable copy of the LineConfig this leaf was
// constructed with (config_.calMin/calMax/filtAlpha), so this leaf's one
// config_ stays the single source of truth for both the boot-time
// calibration AND any later runtime recalibration call.
//
// See color_sensor.h's "ColorSensorLeaf / LineSensorLeaf naming" note for
// why this class carries a `Leaf` suffix.
#pragma once

#include <cstdint>

#include "devices/device_config.h"
#include "devices/device_types.h"
#include "devices/i2c_bus.h"

namespace Devices {

constexpr uint8_t kLineDeviceAddr = 0x1A;

class LineSensorLeaf {
 public:
  LineSensorLeaf(I2CBus& bus, const LineConfig& config);

  // Non-blocking single detection step. Call once per fiber cycle until
  // detectDone() is true; a no-op once it is.
  //
  // Up to kMaxAttempts attempts, kRetryPeriod apart (paced by nowUs, never a
  // real sleep): a successful 4-channel raw read means present; retried
  // with a settle pause so a sensor still powering up after a cold boot is
  // caught once it answers. present()/connected() become true on the first
  // successful attempt; both stay false if every attempt is exhausted.
  void beginStep(uint64_t nowUs);  // [us]
  bool detectDone() const;

  // present()/connected(): same distinction as otos.h/color_sensor.h.
  // present() is set once by beginStep() and never re-evaluated;
  // connected() is the live, per-tick() bus-health result.
  bool present() const;
  bool connected() const;

  // True if a real bus read is due: no real read has ever happened, or at
  // least (LineConfig::lagLine * 1000) [us] have elapsed since the last one
  // -- the leaf's own "sensor polling budget" (see color_sensor.h's
  // identical readDue() note for lagColor).
  bool readDue(uint64_t nowUs) const;  // [us]

  // The leaf's one steady-state bus-touching entry point: a single
  // non-blocking 4-channel raw read (readRaw()), normalized against
  // config_.calMin/calMax with optional EMA smoothing (config_.filtAlpha).
  // No-op (no bus traffic) if beginStep() never found a chip, or before
  // readDue() is true.
  void tick(uint64_t nowUs);  // [us]

  LineReading reading() const;
  bool readingFresh() const;

 private:
  enum class DetectPhase : uint8_t { Probing, Done };

  I2CBus& bus_;
  LineConfig config_;

  DetectPhase phase_ = DetectPhase::Probing;
  int attempts_ = 0;
  bool hasAttempted_ = false;
  uint64_t lastAttemptUs_ = 0;  // [us]

  bool initialized_ = false;  // present()'s backing field
  bool connected_ = false;

  LineReading cachedReading_{};
  bool readingFresh_ = false;

  uint64_t lastReadUs_ = 0;  // [us] time of the most recent real tick() read
  bool hasRead_ = false;

  float emaState_[4] = {0.0f, 0.0f, 0.0f, 0.0f};  // normalized, float 0..1000

  static constexpr uint8_t kMaxAttempts = 20;    // matches the ported retry count
  static constexpr uint64_t kRetryPeriod = 50000;  // [us] matches the ported fiber_sleep(50)
  static constexpr uint32_t kDefaultLagLine = 50;  // [ms] matches source_old DefaultConfig p.lagLine
  static constexpr uint32_t kDefaultCalMax = 255;  // matches the pre-port _calMax[] default

  // Low-level 4-channel raw read (ungated by readDue()); out may be nullptr
  // (probe use, matches the pre-port readRaw()'s own nullptr-tolerant
  // signature). Returns false on the first I2C write/read failure.
  bool readRaw(uint16_t out[4]);
};

}  // namespace Devices
