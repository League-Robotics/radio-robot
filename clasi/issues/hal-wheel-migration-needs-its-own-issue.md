---
status: pending
---

# `Hal::Wheel` migration needs its own trackable issue

## Description

Found and flagged during sprint 136 ticket 008 (documentation sweep +
reorganization-proposal verification and closure, 2026-08-12).

The platform/hardware/hal/core reorganization proposal
(`clasi/sprints/136-green-baseline-and-firmware-layering-interfaces-to-hal-com-dissolved-control-law-out-of-core/issues/proposal-platform-hardware-hal-core-reorganization.md`)
names its Sequencing step 8 as two pieces: Robot/RobotState formalization
(now owned by `clasi/issues/robot-base-class-and-robots-subsystem.md`) and
a new `Hal::Wheel` class. The Robot half has its own issue; the `Hal::Wheel`
half does not — it exists only as a subsection of two co-located design
docs, `src/firm/hal/DESIGN.md` §4 and `src/firm/control/DESIGN.md` §4. That
leaves it undiscoverable the way step 7 (WifiTransport,
`clasi/issues/wifi-alternative-command-path.md`) and step 8a are not: a
future sprint-planner scanning `clasi/issues/` for open reorganization work
would miss it entirely.

## What the migration is

Today the wheel-speed control law — `fastPid()` plus the Stage A/B/C
correction, bias adaptation, and stall/deficit-latch machinery — lives in
`Control::DifferentialDrive` (`control/differential_drive.{h,cpp}`,
relocated out of `core/` by sprint 136 ticket 006). The reorganization
proposal's original vision was to move this down a layer, onto a
per-wheel `Hal::Wheel` abstraction (linear, mm/s or m/s) wrapping a
`Hal::Motor&`, matching the pattern surveyed across other frameworks
(Pybricks `Motor`→`DriveBase`, WPILib `MotorController`→`*Drive`+
`*Kinematics`, ROS2 `ActuatorInterface`→`diff_drive_controller`).

The actual blocker, confirmed twice now (once during the original
proposal's research, again during sprint 136's own execution): **`Hal::
Motor` is duty-commanded only** — its one command verb is `setDuty()`,
open loop, `[-1, 1]`. There is no closed-loop angular-velocity entry point
on `Hal::Motor` to build a `Wheel` on top of. Writing `Hal::Wheel` today
would create a second, competing home for wheel control while the real
control law keeps running in `control/` — the exact layering ambiguity
this reorganization exists to remove, not reduce.

## Why it's deferred, not just unscheduled

This is a **behavioral** change, not a relocation: it moves where the
control law's ownership boundary sits and requires first deciding what a
closed-loop `Hal::Motor` (or a new interface beneath `Hal::Wheel`) looks
like. That is real design work — a new interface shape, a decision about
per-wheel vs. whole-drivetrain scoping (per-wheel already favored, matching
ROS2's single-joint-before-chassis-controller precedent) — not something a
docs-only or mechanical-rename ticket should absorb.

## Where the design reasoning already lives

- [`src/firm/hal/DESIGN.md`](../../src/firm/hal/DESIGN.md) §4 — "`Hal::
  Wheel` is not here yet, on purpose," with the two findings above spelled
  out in full.
- [`src/firm/control/DESIGN.md`](../../src/firm/control/DESIGN.md) §4 —
  "The deferred `Hal::Wheel` migration lands here eventually, not yet,"
  the mirror-image note from `control/`'s own side.

A future sprint-planner has a running start from these two sections alone;
this issue exists so that starting point has somewhere to be picked up
from other than a reorganization proposal that is otherwise being closed
out.

## Recommendation

Scope this into a proper ticket in a future sprint once a `Hal::Motor`
closed-loop entry point (or its replacement) is designed — not before.
Until then, this issue is the pointer; the two `DESIGN.md` §4 sections
remain the source of truth for the reasoning.

## Related

- `clasi/sprints/136-green-baseline-and-firmware-layering-interfaces-to-hal-com-dissolved-control-law-out-of-core/issues/proposal-platform-hardware-hal-core-reorganization.md` — the parent proposal, re-scoped to steps 7-8 by sprint 136 ticket 008.
- `clasi/issues/robot-base-class-and-robots-subsystem.md` — step 8's sibling half (Robot/RobotState).
- `clasi/issues/wifi-alternative-command-path.md` — step 7 (WifiTransport), the other still-open reorganization step.
