---
status: in-progress
sprint: 090
tickets:
- 090-001
---

# Drivetrain owns motor-observation port resolution (remove `p.left - 1` from the main loop)

Extracted from `sprint-089-omnibus.md` entry 1 (stakeholder design discussion 2026-07-07).

## Problem

`source/runtime/main_loop.cpp:152`:

```cpp
Subsystems::DrivetrainPorts p = drivetrain_.ports();   // bound pair, from config
drivetrain_.tick(now, bb.motor[p.left - 1], bb.motor[p.right - 1], bb.driveIn);
```

This is an **ownership split**, not just a readability nit. Drivetrain owns the
port binding (`drivetrain_.ports()`, resolved from robot config), but the main
loop performs the port→cell resolution — including the bare `- 1` base
conversion (Nezha ports are 1-based; `bb.motor[]` is a 0-based C array). The
knowledge lives in Drivetrain; the *use* of it leaks into the loop. Both sides
are bare integers, so nothing type-checks the off-by-one, and the unchecked
invariant ("ports are 1-based, dense, ≤ kMotorCount") can walk off the array on
a bad config with no assert.

## Direction

Move the resolution to the port-owner, over the **observation plane** — not the
hardware:

```cpp
drivetrain_.tick(now, bb.motor, bb.driveIn);   // loop does no indexing
```
```cpp
// inside Drivetrain::tick, where ports_ actually lives:
const auto& leftObs  = motor[ports_.left - 1];
const auto& rightObs = motor[ports_.right - 1];
// range-assert ports_.left/right against kMotorCount here
```

The `- 1` then exists exactly once, with the object that owns the port numbers;
the loop reads as pure sequencing; the range assert has a home.

**Also rename `bb.motor` → `bb.motors`** (stakeholder, 2026-07-07): it is an
array of per-port motor observations, so the plural is correct. Do the rename as
part of this change, since the blackboard field is being touched here anyway.

## Rejected alternative: pass `Hardware&` into `Drivetrain::tick`

Considered and rejected — recorded so it is not re-proposed:

- **Drivetrain needs motor *observations* (read), not actuation.** It already
  emits a command message routed `driveIn → motorIn` by the loop; it never
  calls a motor to drive it. So a handle to hardware buys nothing the
  observation array doesn't already provide.
- **`Hardware&` breaks the x[k] committed snapshot.** `hardware_.tick()` runs
  before `drivetrain_.tick()` in the same pass; `bb.motor[]` is the *committed*
  (last-pass) observation cell, whereas a live `hardware.motor(i)` read would
  return intra-pass state hardware just mutated — the exact read-ordering hazard
  the two-plane ordered-tick model (sprints 060/087) exists to remove.
- **It reintroduces direct-call coupling.** A `Hardware&` inside Drivetrain
  brings back `hardware.motor(i).apply(...)`-style point-to-point calls that the
  blackboard command plane replaced.

The config-level dependency ("a Tovez drivetrain is bound to specific ports") is
already expressed correctly by Drivetrain holding `ports_`; it does not need to
become a compile-time dependency on the `Hardware` object.

## Scope

- `source/runtime/main_loop.cpp` (~line 152)
- `source/subsystems/drivetrain.{h,cpp}` (tick signature + internal indexing +
  range assert)
