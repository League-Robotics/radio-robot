// preamble.cpp -- App::Preamble implementation. See preamble.h's file
// header for the module's full contract and the ported-constants table.
#include "app/preamble.h"

namespace App {

Preamble::Preamble(Devices::NezhaMotor& left, Devices::NezhaMotor& right,
                    Devices::Otos& otos, Devices::ColorSensorLeaf& color,
                    Devices::LineSensorLeaf& line, const Devices::Clock& clock)
    : left_(left),
      right_(right),
      otos_(otos),
      color_(color),
      line_(line),
      clock_(clock) {}

bool Preamble::done() const {
  for (uint8_t i = 0; i < kSlotCount; ++i) {
    if (!resolved_[i]) return false;
  }
  return true;
}

void Preamble::step() {
  const uint64_t nowUs = clock_.nowMicros();

  if (!started_) {
    started_ = true;
    startUs_ = nowUs;
  }

  if (done()) return;  // already latched -- matches beginStep()'s own
                        // "no-op once done" contract on the leaves

  // Boot power-settle wait -- no leaf is touched at all until this
  // elapses. Also satisfies "no I2C traffic before Preamble has begun
  // probing" for the very first probe (there is no probe to issue yet).
  if (nowUs - startUs_ < kPowerSettleUs) return;

  // Defensive wall-clock bound (this file's header "Defensive bound"
  // comment) -- never reached by a correctly-functioning leaf; forces
  // every remaining slot terminal so done() cannot hang forever even if a
  // leaf's own detectDone() has a latent bug.
  if (nowUs - startUs_ >= kMaxPreambleUs) {
    forceResolveAll();
    return;
  }

  // Round robin: at most ONE leaf's own detection entry point is called
  // per step() -- "one bounded probe action per pass," the archived
  // plan's own wording (this file's header comment, "The boot loop this
  // drives").
  for (uint8_t i = 0; i < kSlotCount; ++i) {
    Slot slot = static_cast<Slot>((cursor_ + i) % kSlotCount);
    uint8_t slotIndex = static_cast<uint8_t>(slot);
    if (resolved_[slotIndex]) continue;
    if (!dueSlot(slot, nowUs)) continue;

    probeSlot(slot, nowUs);
    cursor_ = static_cast<uint8_t>((slotIndex + 1) % kSlotCount);
    return;  // exactly one probe action this call
  }
  // Nothing due this pass (every remaining slot is a paced retry not yet
  // due) -- a true no-op; the BOOT LOOP's own sleep between step() calls
  // is what advances time toward the next due attempt.
}

bool Preamble::dueSlot(Slot slot, uint64_t nowUs) const {
  switch (slot) {
    case Slot::Left:
    case Slot::Right:
      // One-shot terminal probe (NezhaMotor::begin() already runs its own
      // internal median-of-3 + retry inside hardReset()) -- always due
      // until resolved.
      return true;
    case Slot::Otos:
      return otosAttempts_ == 0 ||
             (nowUs - otosLastAttemptUs_) >= kOtosBeginRetryPeriod;
    case Slot::Color:
    case Slot::Line:
      // The leaf's OWN beginStep(nowUs) already owns its internal retry
      // pacing (color_sensor.h's kAltRetryPeriod / line_sensor.h's
      // kRetryPeriod) -- always "due" from Preamble's point of view; a
      // call that arrives before the leaf's own internal deadline is a
      // harmless, bus-traffic-free no-op inside beginStep() itself.
      // Preamble does not duplicate that pacing check -- this file's
      // header "no leaf's own retry loop is reimplemented" contract.
      return true;
    default:
      return false;
  }
}

void Preamble::probeSlot(Slot slot, uint64_t nowUs) {
  switch (slot) {
    case Slot::Left:
      left_.begin();
      resolved_[static_cast<uint8_t>(Slot::Left)] = true;
      break;
    case Slot::Right:
      right_.begin();
      resolved_[static_cast<uint8_t>(Slot::Right)] = true;
      break;
    case Slot::Otos:
      otos_.begin();
      ++otosAttempts_;
      otosLastAttemptUs_ = nowUs;
      if (otos_.connected() || otosAttempts_ >= kOtosBeginAttempts) {
        resolved_[static_cast<uint8_t>(Slot::Otos)] = true;
      }
      break;
    case Slot::Color:
      color_.beginStep(nowUs);
      if (color_.detectDone()) {
        resolved_[static_cast<uint8_t>(Slot::Color)] = true;
      }
      break;
    case Slot::Line:
      line_.beginStep(nowUs);
      if (line_.detectDone()) {
        resolved_[static_cast<uint8_t>(Slot::Line)] = true;
      }
      break;
    default:
      break;
  }
}

void Preamble::forceResolveAll() {
  for (uint8_t i = 0; i < kSlotCount; ++i) resolved_[i] = true;
}

}  // namespace App
