// line_sensor.h — Devices::LineSensor: the internal leaf for the PlanetX
// 4-channel line sensor, I2C address 0x1A.
//
// Ticket DB-006 (device-bus-tickets.md). Ported from
// source_old/hal/real/LineSensor.{h,cpp} into the greenfield
// `source/devices/` subsystem (namespace `Devices`), per clasi/issues/
// device-bus-fiber-owned-self-contained-device-subsystem.md's "Shape" —
// "Line and color sensing don't exist in the new tree yet." Mirrors
// color_sensor.h's leaf shape (this ticket's sibling): a non-blocking
// beginStep(nowUs) detection state machine, present()/connected(), and a
// non-blocking tick(nowUs) that publishes a combined raw+normalized
// LineReading.
//
// Protocol: write a 1-byte channel index (0-3), then read 1 byte of
// grayscale data (0 = white, 255 = black approximately) -- FOUR such
// write/read pairs per full sample. This primitive (readRaw(), below) was
// ALREADY non-blocking in the pre-port driver (no fiber_sleep anywhere in
// it) -- device-bus-tickets.md's DB-006 "If the source_old driver has
// blocking waits, restructure to non-blocking" note therefore applies here
// ONLY to begin()'s detection retry loop (see beginStep()'s own comment),
// not to the steady-state read path, which is carried over unchanged.
//
// Calibration (captureCalibMin()/captureCalibMax()/setSmoothingAlpha(),
// below) is preserved from the pre-port public surface, operating on a
// local mutable copy of the LineConfig this leaf was constructed with
// (config_.calMin/calMax/filtAlpha) rather than the pre-port file's own
// dedicated _calMin/_calMax/_alpha members -- same fields, sourced from
// DB-001's LineConfig (device_config.h) instead of a bespoke member set, so
// this leaf's one config_ stays the single source of truth for both the
// boot-time calibration AND any later runtime recalibration call.
#pragma once

#include <cstdint>

#include "devices/device_config.h"
#include "devices/device_types.h"
#include "devices/i2c_bus.h"

namespace Devices {

constexpr uint8_t kLineDeviceAddr = 0x1A;

class LineSensor {
 public:
  LineSensor(I2CBus& bus, const LineConfig& config);

  // Non-blocking single detection step. Call once per fiber cycle (DB-007's
  // detection preamble) until detectDone() is true; a no-op once it is.
  //
  // Up to kMaxAttempts attempts, kRetryPeriod apart (paced by nowUs, never a
  // real sleep) -- the non-blocking restructuring of the pre-port begin()'s
  // `for (...) { if (readRaw()) return true; fiber_sleep(50); }` loop (a
  // successful 4-channel raw read means present; retried with a settle
  // pause so a sensor still powering up after a cold boot is caught once it
  // answers). present()/connected() become true on the first successful
  // attempt; both stay false if every attempt is exhausted.
  void beginStep(uint64_t nowUs);  // [us]
  bool detectDone() const;

  // present()/connected(): same sprint-099 distinction as otos.h/
  // color_sensor.h. present() is set once by beginStep() and never
  // re-evaluated; connected() is the live, per-tick() bus-health result.
  bool present() const;
  bool connected() const;

  // True if a real bus read is due: no real read has ever happened, or at
  // least (LineConfig::lagLine * 1000) [us] have elapsed since the last one
  // -- LineConfig::lagLine is the same "sensor polling budget" gate the
  // pre-port architecture's Sensors::periodic() applied at the consumer
  // layer (see color_sensor.h's identical readDue() note for lagColor).
  bool readDue(uint64_t nowUs) const;  // [us]

  // The leaf's one steady-state bus-touching entry point: a single
  // non-blocking 4-channel raw read (readRaw()), normalized against
  // config_.calMin/calMax with optional EMA smoothing (config_.filtAlpha) --
  // the exact math of the pre-port file's readNormalized(). No-op (no bus
  // traffic) if beginStep() never found a chip, or before readDue() is true.
  void tick(uint64_t nowUs);  // [us]

  LineReading reading() const;
  bool readingFresh() const;

  // Snapshot the CURRENT raw readings into the calibration bounds -- call
  // while physically over a white (captureCalibMin()) or black
  // (captureCalibMax()) surface. Each issues its own fresh, non-blocking
  // readRaw() (ungated by readDue() -- an explicit calibration action is
  // not subject to the steady-read rate limit). Returns false (config_
  // unchanged) if the read fails or the chip was never detected.
  bool captureCalibMin();
  bool captureCalibMax();

  // EMA smoothing coefficient applied in tick()'s normalize step.
  // alpha == 0.0 means no smoothing (default); clamped to [0, 0.99] --
  // ported unchanged from setSmoothingAlpha()'s pre-port clamp.
  void setSmoothingAlpha(float alpha);

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
