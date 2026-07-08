---
status: in-progress
sprint: 090
tickets:
- 090-003
---

# NullOdometer: replace `odometer != nullptr` checks with a Null Object

Extracted from `sprint-089-omnibus.md` entry 4 (stakeholder design discussion 2026-07-07).

Related: [[odometer-owns-reset-and-fusability]] (shares the fusability contract).

## Problem

`hardware_.odometer()` (`source/runtime/main_loop.cpp:158`) returns a raw
pointer that may be null, so "no device" handling is smeared across the loop:

- `:173` — reset-drain guard `if (odometer != nullptr)`, with an else-branch
  (`:189-194`) that discards `otosCommandIn`/`otosSetPoseIn` purely to keep the
  mailboxes from looking perpetually full.
- `:270-276` — commit block: the null branch sets `bb.otosValid = false`.

The repeated null checks are a missing abstraction.

## Direction

Make `hardware_.odometer()` **always** return a valid object — a `NullOdometer`
when no physical device is configured — that responds inertly:

- `tick()` no-ops
- `pose()` returns identity
- `apply()` discards (so the loop can always drain-and-apply without a branch)
- **reports itself not-fusable** (see below)

Then all three branches collapse: the loop always drains/applies/ticks the
odometer, and `bb.otosValid` derives from the odometer's own report instead of a
`== nullptr` test.

## Composes with the fusability contract (issue: odometer-owns-reset-and-fusability)

`NullOdometer`'s "don't fuse me" is exactly the fusability query that issue adds
to the odometer contract — `NullOdometer::fusableThisPass()` simply always
returns `false`, and `bb.otosValid` falls out of that. This issue should land
**on top of** that contract, not re-invent a validity signal.

## Sequencing

Keep separate from [[odometer-owns-reset-and-fusability]] but dependent on it:
that issue is a behavior-preserving relocation guarding the live-debugged
stale-OTOS fix; this one is a new type + a changed return contract on
`hardware_.odometer()`. Landing them separately keeps the risky EKF-fix-
preserving work out of the mechanical pattern swap.

## Scope

- `source/hal/.../null_odometer.{h,cpp}` (new) or a `NullOdometer` alongside the
  existing `Hal::Odometer` interface
- `source/hal/.../` hardware: `odometer()` returns the NullOdometer instead of
  `nullptr` when no device is present
- `source/runtime/main_loop.cpp` (remove the three null branches)
