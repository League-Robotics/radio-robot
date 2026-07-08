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
// the active owner's Hal::Odometer leaf, if it has one at all. Originally
// defaulted to nullptr (a virtual no-op default, NOT pure) rather than every
// owner having to implement it, for the same reason begin() is a no-op
// default (Open Question 1, above): at the time, Subsystems::NezhaHardware
// had no real-hardware OTOS driver (stakeholder-approved, 2026-07-05 —
// sim-only fused pose for 082) and had to compile/link unchanged inheriting
// this default.
//
// (090-003) The default is now Hal::NullOdometer (hal/capability/
// null_odometer.h) instead of nullptr — an inert Null Object, never a real
// per-call allocation (one shared static instance, below) — so
// `hardware_.odometer()` NEVER returns null and every caller's former
// `if (odometer != nullptr)` guard (main_loop.cpp's three branches,
// main.cpp's bb.otosPresent snapshot, configurator.cpp's config guard) drops
// to its unconditional form. Both concrete owners keep overriding odometer()
// unchanged: Subsystems::NezhaHardware (its real Hal::OtosOdometer member,
// since ticket 086-006) and Subsystems::SimHardware (ticket 081-003's
// Hal::SimOdometer member) — this base default is reachable only by a
// hypothetical THIRD owner that supplies no odometer of its own (exercised
// directly by tests/sim/unit/null_odometer_harness.cpp's bare stub owner,
// since no currently-constructed owner takes this path).
//
// Headers-only — no hardware.cpp: a pure interface, matching
// hal/capability/*.h's own headers-only convention (see e.g.
// capability/motor.h's file header). Every method here is either pure
// virtual or a virtual no-op default.
#pragma once

#include <stdint.h>

#include "hal/capability/hal_command.h"
#include "hal/capability/motor.h"
#include "hal/capability/null_odometer.h"
#include "hal/capability/odometer.h"
#include "messages/motor.h"
#include "runtime/queue.h"

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
  //
  // motorIn/motorResetIn (087-004, architecture-update-r1.md Decision 2): the
  // per-port command-plane inputs, consumed uniformly (no addressed-dispatch
  // branch — every port is treated identically):
  //   - motorIn[i] (Rt::Mailbox<msg::MotorCommand>, source/runtime/queue.h):
  //     when non-empty, popped (latest-wins) and applied to port i+1 via the
  //     SAME Hal::Motor::apply() the existing apply() overloads below use.
  //   - motorResetIn[i] (a plain flag, not a queue — "reset twice = reset
  //     once" is idempotent by nature, so no queue is needed): when true,
  //     applies port i+1's existing Hal::Motor::resetPosition() (itself
  //     staged, not immediate — see that method's own doc comment) and
  //     clears the flag.
  // Both arrays are the CALLER's own storage (typically Rt::Blackboard's
  // motorIn[]/motorResetIn[] members) — this method mutates them in place
  // (draining motorIn[i], clearing a consumed motorResetIn[i]).
  virtual void tick(uint32_t now, Rt::Mailbox<msg::MotorCommand> motorIn[kPortCount],
                     bool motorResetIn[kPortCount]) = 0;   // [ms]

  // Distribution — see hal/capability/hal_command.h for both edge types'
  // shapes and that file's own doc comment on why they live there rather
  // than beside either producer or this consumer.
  virtual void apply(const Hal::CommandProcessorToHardwareCommand& cmd) = 0;
  virtual void apply(const Hal::DrivetrainToHardwareCommand& cmd) = 0;

  // config()/state() (087-004) — a uniform, port-indexed faceplate at the
  // Hardware level itself, not only reachable by narrowing through motor(p)
  // (today's only path, and today there is no config getter at all: Hal::
  // Motor's configure() has no matching getter). Same [1, kPortCount]
  // port-indexed convention and out-of-range clamp behavior as motor()/
  // motorAt() (see each concrete owner's own doc comment). Kills the
  // per-motor config shadow this sprint's design removes elsewhere — a
  // caller (the Configurator, ticket 005) can read back the currently
  // configured/observed value per port without narrowing to a concrete
  // Hal::Motor reference.
  virtual msg::MotorConfig config(uint32_t port) const = 0;
  virtual msg::MotorState state(uint32_t port) const = 0;

  // The active owner's Hal::Odometer leaf, or a shared Hal::NullOdometer
  // instance if it has none — NEVER nullptr (090-003) — see the file
  // header's "odometer()" section for the NullOdometer default's rationale.
  // Rt::MainLoop::tick() (source/runtime/main_loop.cpp) is this seam's one
  // production caller: it queries this every pass and feeds the sample into
  // Subsystems::PoseEstimator, unconditionally now that there is always a
  // (possibly inert) Hal::Odometer to feed it from.
  virtual Hal::Odometer* odometer() {
    static Hal::NullOdometer nullOdometer;   // one shared instance, no per-call allocation
    return &nullOdometer;
  }
};

}  // namespace Subsystems
