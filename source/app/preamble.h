// preamble.h -- App::Preamble: the boot-time device-detection driver.
// Replaces the retired Devices::DeviceBus::runPreamble() (deleted ticket
// 103-003; see git history 88e04f1b^:source/devices/device_bus.cpp) with a
// flatter equivalent that sequences each bare leaf's OWN already-existing
// detection entry point (NezhaMotor::begin(), Otos::begin(),
// ColorSensorLeaf::beginStep(nowUs), LineSensorLeaf::beginStep(nowUs) -- all
// unchanged, KEPT) to a done() terminal signal.
//
// architecture-update.md (103) Step 3 "Preamble" boundary: inside --
// calling order (begin()/beginStep(nowUs) at most once per step()) and a
// done() terminal signal; outside -- each leaf's own detection retry logic
// (unchanged, leaves own it). Serves SUC-007.
//
// --- The boot loop this drives (issue "The main loop", archived plan) ---
//   while (!preamble.done()) {
//     preamble.step();   // one bounded probe action per pass
//     tlm.emit();        // boot frames: device detection status, faults
//     uBit.sleep(kPreamblePace);   // the BOOT LOOP owns this sleep, never Preamble
//   }
// Telemetry flows from power-on (frames report per-device status, via this
// class's own accessors below); commands are not consumed until the main
// loop starts. Wiring Preamble's accessors into Telemetry::setFrame()/
// setEvent(kEventBootReady, ...) is ticket 008's job (main.cpp construction)
// -- this ticket only builds the driver and its read-only status surface.
//
// --- step()'s contract: ONE bounded probe action per call, never sleeps ---
// Each call to step() advances AT MOST ONE not-yet-resolved device's own
// detection entry point by exactly one call -- "one bounded probe action
// per pass," the archived plan's own wording. Pacing between retries (OTOS,
// color, line) is expressed as "not due yet, do nothing this call, try the
// next unresolved device instead" -- step() itself never sleeps; the BOOT
// LOOP above owns the real gap between calls. No leaf's own retry loop is
// reimplemented here, only sequenced/called -- see probeSlot()'s own
// per-device comments for the one exception (OTOS) and why it is not a
// counterexample.
//
// --- Time seam ---
// Preamble takes a `const Devices::Clock&` (the same fiber-level time seam
// NezhaMotor/Otos/Deadman already read "now" through -- clock.h's own file
// header) and reads clock_.nowMicros() internally at the top of every
// step() call, rather than taking a nowUs parameter -- matching the boot
// loop's own bare `preamble.step();` call site (usecases.md SUC-008) and
// Deadman's identical precedent (ticket 004's actual constructor takes a
// Clock&, diverging from the issue's illustrative bare `Deadman deadman;`
// sketch for the same reason).
//
// --- Decision: KEEP the boot power-settle wait ---
// This ticket's own acceptance criterion requires deciding, not leaving
// implicit, whether to keep an explicit power-settle wait (mirroring the
// retired DeviceBus::kPowerSettleMs) or rely on each leaf's own retry
// pacing alone. Decision: KEEP it, unchanged in value (kPowerSettleUs,
// below) -- it is cheap (step() simply does nothing until it elapses, no
// leaf touched, no bus traffic), it is a bench-tuned value already proven
// on real hardware (device_bus.h's own comment: "a starting, bench-tunable
// value"), and dropping it would let the FIRST probe (motor1's begin(),
// first in sequence) race the rails on every boot instead of only when a
// leaf's own retry pacing happens to help -- motor begin() has no retry
// pacing of its own to lean on (NezhaMotor::begin() is a single hardReset()
// call, not a paced retry loop), so "rely on each leaf's own pacing
// instead" is not actually available for the very first device probed.
//
// --- Ported constants (from device_bus.h, git history 88e04f1b^) ---
//   kPowerSettleUs        = 50000   [us]  (was kPowerSettleMs = 50)
//   kOtosBeginAttempts    = 20            (unchanged)
//   kOtosBeginRetryPeriod = 100000  [us]  (was kOtosBeginRetryPacingMs = 100)
// color_sensor.h's kMaxAltAttempts/kAltRetryPeriod and line_sensor.h's
// kMaxAttempts/kRetryPeriod are NOT re-ported here -- they already live on
// the leaves themselves (unchanged), and Preamble calls beginStep() without
// re-implementing their own internal pacing (see step()'s contract above).
//
// --- Defensive bound (this ticket's own, NOT kMaxPreambleTicks verbatim) ---
// kMaxPreambleUs (below) is a wall-clock safety net, not the primary
// termination mechanism: every slot already self-bounds (motor: one call;
// OTOS: Preamble's own kOtosBeginAttempts counter; color/line: each leaf's
// own kMaxAltAttempts/kMaxAttempts internal bound) PROVIDED step() is
// called often enough with real elapsed time between calls (the boot
// loop's job). kMaxPreambleUs exists only to guard against a future leaf
// regression (e.g. a detectDone() that never returns true) turning this
// into a real infinite loop -- the same defensive-bound spirit as the
// retired DeviceBus::kMaxPreambleTicks, expressed as an elapsed-wall-time
// bound instead of a step()-call count, because step()'s own call cadence
// is now the BOOT LOOP's choice (ticket 008), not a fixed pacing internal
// to this class the way the old blocking runPreamble() loop had. Sized
// generously above the natural worst case (OTOS-bound:
// 20 * 100ms = 2000ms; color/line: ~21 * 50ms = 1050ms and 20 * 50ms =
// 1000ms respectively; plus the 50ms power-settle wait) -- see
// kMaxPreambleUs's own comment.
#pragma once

#include <cstdint>

#include "devices/clock.h"
#include "devices/color_sensor.h"
#include "devices/line_sensor.h"
#include "devices/nezha_motor.h"
#include "devices/otos.h"

namespace App {

class Preamble {
 public:
  // left/right -- the two drive-wheel NezhaMotor leaves, same L/R
  // convention as Drive (which slot is "left" vs "right" is main.cpp's own
  // construction-time wiring, ticket 008 -- Preamble does not care which
  // physical wheel each is, only that begin() is called on each).
  Preamble(Devices::NezhaMotor& left, Devices::NezhaMotor& right,
           Devices::Otos& otos, Devices::ColorSensorLeaf& color,
           Devices::LineSensorLeaf& line, const Devices::Clock& clock);

  // Advances AT MOST ONE not-yet-resolved device's own detection entry
  // point by exactly one call. Never sleeps, never blocks -- a no-op call
  // (nothing due yet, or done() already true) returns immediately having
  // touched no leaf and no bus. See this file's own header comment for the
  // full contract.
  void step();

  // True once every device has reached a terminal state: present-and-ready,
  // OR confirmed-absent after exhausting its own (or Preamble's, for OTOS)
  // retry budget. An absent sensor cannot hang this forever -- see
  // kMaxPreambleUs's own comment.
  bool done() const;

  // --- Per-device status accessors -- boot telemetry (ticket 008 wires
  // these into App::Telemetry::setFrame()'s connLeft/connRight/otosConnected
  // fields and setEvent(kEventBootReady, done()) on done()'s first-true
  // transition; telemetry.h's own fault/event bit-layout comment names
  // kEventBootReady as "Preamble::done() first true, ticket 007"). Each is a
  // cheap forwarding accessor to the leaf's own existing status method --
  // Preamble holds no separate copy of this state. ---
  bool leftConnected() const { return left_.connected(); }
  bool rightConnected() const { return right_.connected(); }
  bool otosPresent() const { return otos_.present(); }
  bool otosConnected() const { return otos_.connected(); }
  bool colorPresent() const { return color_.present(); }
  bool linePresent() const { return line_.present(); }

 private:
  // Round-robin device slots -- step() visits at most one unresolved slot
  // per call, cursor_ remembering where the NEXT call should resume so
  // every slot gets a fair turn (a slot not yet due -- OTOS's own pacing --
  // is skipped in favor of the next unresolved slot, not spun on).
  enum class Slot : uint8_t { Left, Right, Otos, Color, Line, kCount };
  static constexpr uint8_t kSlotCount = static_cast<uint8_t>(Slot::kCount);

  // [us] boot power-settle wait, ported from device_bus.h's kPowerSettleMs
  // (50) -- see this file's header "Decision: KEEP the boot power-settle
  // wait" comment.
  static constexpr uint64_t kPowerSettleUs = 50000;

  // OTOS product-ID probe retry, ported from device_bus.h's
  // kOtosBeginAttempts/kOtosBeginRetryPacingMs -- Otos::begin() is a single
  // probe with no retry of its own (otos.h), so Preamble owns this pacing
  // (NOT a "leaf's own retry loop" being reimplemented -- there is none to
  // duplicate; this is Preamble supplying one where the leaf has none, the
  // same role the retired DeviceBus::runPreamble() played).
  static constexpr int kOtosBeginAttempts = 20;
  static constexpr uint64_t kOtosBeginRetryPeriod = 100000;  // [us]

  // [us] defensive wall-clock bound -- see this file's header "Defensive
  // bound" comment for the derivation (~2s natural worst case + margin).
  static constexpr uint64_t kMaxPreambleUs = 5000000;

  bool dueSlot(Slot slot, uint64_t nowUs) const;
  void probeSlot(Slot slot, uint64_t nowUs);
  void forceResolveAll();

  Devices::NezhaMotor& left_;
  Devices::NezhaMotor& right_;
  Devices::Otos& otos_;
  Devices::ColorSensorLeaf& color_;
  Devices::LineSensorLeaf& line_;
  const Devices::Clock& clock_;

  bool resolved_[kSlotCount] = {};
  uint8_t cursor_ = 0;

  bool started_ = false;
  uint64_t startUs_ = 0;  // [us] time of the first step() call

  uint8_t otosAttempts_ = 0;
  uint64_t otosLastAttemptUs_ = 0;  // [us]
};

}  // namespace App
