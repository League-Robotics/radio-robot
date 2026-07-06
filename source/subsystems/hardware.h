// hardware.h — Subsystems::Hardware: the abstract owner base for "which
// aggregator/scheduler/distributor owns the addressable motor-port surface"
// — see clasi/sprints/081-.../architecture-update.md Decision 1 for the full
// naming rationale. This is NOT the design write-up's `Hal::MotorHal` (a
// name that referred to `Hal::NezhaHal`/`DrivetrainToHalCommand` — symbols
// renamed away the same day the design was reviewed, see that document's
// "Reconciliation with the design write-up" section); do not reintroduce
// that pre-rename vocabulary here.
//
// Subsystems::NezhaHardware (the real I2C brick flip-flop sequencer) and the
// forthcoming Subsystems::SimHardware (ticket 003: simulated plant + four
// motors + one odometer) are both aggregator/scheduler/distributor classes —
// the Subsystems tier, not per-device Hal faceplates (nezha_hardware.h's own
// header comment makes this same distinction) — so the abstraction over
// "which one owns the ports" belongs here, beside its two implementations,
// the same way hal/capability/hal_command.h's edge types sit beside
// Hal::Motor as a tier every higher layer already depends on. No
// Hal -> Subsystems include in either direction results: this header
// depends only on Hal::Motor / Hal::CommandProcessorToHardwareCommand /
// Hal::DrivetrainToHardwareCommand (data-only), exactly how
// Subsystems::NezhaHardware already depended on Hal.
//
// Contract every concrete Hardware::tick(now) must satisfy (Decision 4):
// it must be safe to call tick() twice in the same pass with an UNCHANGED
// now. Subsystems::NezhaHardware already satisfies this incidentally (the
// I2C bus's microsecond-resolution clearance timer naturally blocks a
// same-now second collect); a future Subsystems::SimHardware has no
// equivalent bus latency to lean on and must guard this deliberately (its
// own same-now re-entry guard, ticket 003).
//
// odometer() (082-003): a SECOND, independent seam alongside motor()/tick() —
// the active owner's Hal::Odometer leaf, if it has one at all. Defaulted to
// nullptr (a virtual no-op default, NOT pure) rather than every owner having
// to implement it, for the same reason begin() is a no-op default (Open
// Question 1, above): Subsystems::NezhaHardware has no real-hardware OTOS
// driver this sprint (stakeholder-approved, 2026-07-05 — sim-only fused pose
// for 082; a real-hardware Hal::Odometer leaf is deferred to its own later
// sprint) and must compile/link unchanged inheriting this default.
// Subsystems::SimHardware (ticket 081-003's Hal::SimOdometer member) is the
// only override this sprint.
//
// Headers-only — no hardware.cpp: a pure interface, matching
// hal/capability/*.h's own headers-only convention (see e.g.
// capability/motor.h's file header). Every method here is either pure
// virtual or a virtual no-op default.
#pragma once

#include <stdint.h>

#include "hal/capability/hal_command.h"
#include "hal/capability/motor.h"
#include "hal/capability/odometer.h"

namespace Subsystems {

class Hardware {
 public:
  static constexpr uint32_t kPortCount = 4;

  virtual ~Hardware() = default;

  // Convenience no-op default — architecture-update.md's Step 7 Open
  // Question 1: no caller needs polymorphic begin() this sprint (each
  // constructs its concrete owner directly and calls begin() before ever
  // assigning it through this base pointer); declared virtual for interface
  // completeness only.
  virtual void begin() {}

  // Port-indexed accessor, port in [1, kPortCount]. Always returns the
  // Hal::Motor faceplate — callers (DEV commands, Drivetrain, devLoopTick)
  // never see a concrete leaf's raw register verbs. Out-of-range handling is
  // each concrete owner's own business (see Subsystems::NezhaHardware::motor()'s
  // doc comment for its clamp-to-port-4 convention).
  virtual Hal::Motor& motor(uint32_t port) = 0;

  // Runs one scheduling pass. now: [ms]. See the file header's twice-per-pass,
  // unchanged-now re-entry contract (Decision 4) every concrete owner must
  // satisfy.
  virtual void tick(uint32_t now) = 0;   // [ms]

  // Distribution — see hal/capability/hal_command.h for both edge types'
  // shapes and that file's own doc comment on why they live there rather
  // than beside either producer or this consumer.
  virtual void apply(const Hal::CommandProcessorToHardwareCommand& cmd) = 0;
  virtual void apply(const Hal::DrivetrainToHardwareCommand& cmd) = 0;

  // The active owner's Hal::Odometer leaf, or nullptr if it has none — see
  // the file header's "odometer()" section for the defaulted-nullptr
  // rationale. devLoopTick() (source/dev_loop.cpp) is this seam's one
  // caller: it queries this every pass and, when non-null, feeds the
  // sample into Subsystems::PoseEstimator.
  virtual Hal::Odometer* odometer() { return nullptr; }
};

}  // namespace Subsystems
