// preamble.h -- Core::Preamble: the boot-time device-detection driver. A
// flat sequencer over each bare leaf's OWN already-existing detection
// entry point (Otos::begin(), ColorSensorLeaf::beginStep(nowUs),
// LineSensorLeaf::beginStep(nowUs)) to a done() terminal signal.
//
// EXPLORATORY-KERNEL REWRITE (2026-08-15,
// clasi/issues/differentialdrive-one-class-one-fiber-exploratory-worktree.md):
// the two drive motors LEFT this probe sequence. Control::
// DifferentialDrive::begin() now primes both encoders itself (called by
// the composition root AFTER Preamble::done(), see boot_wiring.h's own
// "Lifecycle, one level up" note) -- Preamble has no motor slot, no
// leftConnected()/rightConnected() accessor, and no Hal::Motor
// dependency at all any more.
//
// Boundary: inside -- calling order (begin()/beginStep(nowUs) at most once
// per step()) and a done() terminal signal; outside -- each leaf's own
// detection retry logic (leaves own it).
//
// --- The boot loop this drives ---
//   while (!preamble.done()) {
//     preamble.step();   // one bounded probe action per pass
//     tlm.emit();        // boot frames: device detection status, faults
//     uBit.sleep(kPreamblePace);   // the BOOT LOOP owns this sleep, never Preamble
//   }
//
// --- step()'s contract: ONE bounded probe action per call, never sleeps ---
// Each call to step() advances AT MOST ONE not-yet-resolved device's own
// detection entry point by exactly one call. Pacing between retries
// (OTOS, color, line) is expressed as "not due yet, do nothing this call,
// try the next unresolved device instead" -- step() itself never sleeps;
// the BOOT LOOP above owns the real gap between calls.
//
// --- Time seam ---
// Preamble takes a `const Hal::Clock&` and reads clock_.nowMicros()
// internally at the top of every step() call, rather than taking a nowUs
// parameter -- matching the boot loop's own bare `preamble.step();` call
// site.
//
// --- Decision: KEEP the boot power-settle wait ---
// An explicit power-settle wait (kPowerSettle, below) is kept rather than
// relying on each leaf's own retry pacing alone: it is cheap, it is a
// bench-tuned value already proven on real hardware, and dropping it
// would let the FIRST probe race the rails on every boot.
//
// --- Defensive bound ---
// kMaxPreamble (below) is a wall-clock safety net, not the primary
// termination mechanism: every slot already self-bounds (OTOS: Preamble's
// own kOtosBeginAttempts counter; color/line: each leaf's own
// kMaxAltAttempts/kMaxAttempts internal bound) PROVIDED step() is called
// often enough with real elapsed time between calls (the boot loop's
// job).
#pragma once

#include <cstdint>

#include "hal/clock.h"
#include "hal/color_sensor.h"
#include "hal/line_sensor.h"
#include "hardware/generic/real_otos.h"

namespace Core {

class Preamble {
 public:
  Preamble(Hal::Otos& otos, Hal::ColorSensor& color,
           Hal::LineSensor& line, const Hal::Clock& clock);

  // Advances AT MOST ONE not-yet-resolved device's own detection entry
  // point by exactly one call. Never sleeps, never blocks -- a no-op call
  // (nothing due yet, or done() already true) returns immediately having
  // touched no leaf and no bus. See this file's own header comment for the
  // full contract.
  void step();

  // True once every device has reached a terminal state: present-and-ready,
  // OR confirmed-absent after exhausting its own (or Preamble's, for OTOS)
  // retry budget.
  bool done() const;

  // --- Per-device status accessors -- boot telemetry. ---
  bool otosPresent() const { return otos_.present(); }
  bool otosConnected() const { return otos_.connected(); }
  bool colorPresent() const { return color_.present(); }
  bool linePresent() const { return line_.present(); }

 private:
  // Round-robin device slots -- step() visits at most one unresolved slot
  // per call, cursor_ remembering where the NEXT call should resume so
  // every slot gets a fair turn.
  enum class Slot : uint8_t { Otos, Color, Line, kCount };
  static constexpr uint8_t kSlotCount = static_cast<uint8_t>(Slot::kCount);

  // Boot power-settle wait, ported from device_bus.h's kPowerSettleMs (50).
  static constexpr uint64_t kPowerSettle = 50000;  // [us]

  // OTOS product-ID probe retry -- Otos::begin() is a single probe with no
  // retry of its own (otos.h), so Preamble owns this pacing.
  static constexpr int kOtosBeginAttempts = 20;
  static constexpr uint64_t kOtosBeginRetryPeriod = 100000;  // [us]

  // Defensive wall-clock bound -- see this file's header "Defensive bound"
  // comment.
  static constexpr uint64_t kMaxPreamble = 5000000;  // [us]

  bool dueSlot(Slot slot, uint64_t nowUs) const;
  void probeSlot(Slot slot, uint64_t nowUs);
  void forceResolveAll();

  Hal::Otos& otos_;
  Hal::ColorSensor& color_;
  Hal::LineSensor& line_;
  const Hal::Clock& clock_;

  bool resolved_[kSlotCount] = {};
  uint8_t cursor_ = 0;

  bool started_ = false;
  uint64_t startUs_ = 0;  // [us] time of the first step() call

  uint8_t otosAttempts_ = 0;
  uint64_t otosLastAttemptUs_ = 0;  // [us]
};

}  // namespace Core
