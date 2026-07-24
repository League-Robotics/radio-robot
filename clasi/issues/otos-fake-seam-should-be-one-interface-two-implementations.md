---
status: pending
filed: 2026-07-24
filed_by: team-lead (Eric design review)
related:
- estimator-v2-otos-fusion-sim-first.md
---

# FAKE_OTOS should be one Otos interface with two implementations, not a call-site `#ifdef`

## Problem

The FAKE_OTOS build seam (120-002) is implemented as a compile-time
`#ifdef FAKE_OTOS` **branch inside the hot loop** at
[`src/firm/app/robot_loop.cpp:676`](../../src/firm/app/robot_loop.cpp#L676),
plus a test-only `feedSyntheticSample()` method bolted onto the real
hardware leaf [`Devices::Otos`](../../src/firm/devices/otos.h). This is
the wrong shape and defeats the reason `Otos` is an object at all.

Three concrete defects:

1. **`#ifdef` at the call site defeats polymorphism.** The loop should
   not know whether the sensor behind `otos_` is a real I2C chip or a
   synthetic one. Today it branches on the build and runs two
   structurally different pieces of code:
   - fake arm: `otos_.feedSyntheticSample(...)` **then hand-copies six
     fields** into `frame_.otos`;
   - real arm: `applyOtosSample(otos_, nowUs, frame_)`.

2. **The two arms are not equivalent.** The fake arm re-implements,
   inline, the frame-population that `applyOtosSample()` already does.
   Real and fake builds therefore execute *different* perception code —
   they will drift, and a bug reproduced in one build may not exist in
   the code path the other build runs. (Same divergent-real-vs-sim-path
   trap that has already cost debugging time elsewhere.)

3. **Test-only API on the production driver.** `feedSyntheticSample()`
   exists solely to inject fake data, yet it lives on the real SparkFun
   I2C leaf. The production driver should not know the fake exists.

## Correct design

One `Otos` interface, two implementations, choice made once at
construction (dependency injection):

- **`Otos`** — the interface: `tick(nowUs)`, `pose()`, `present()`,
  `connected()`, `poseFresh()`, `setPose()`.
- **`RealOtos`** — today's I2C leaf (current `Devices::Otos` body,
  unchanged behavior).
- **`FakeOtos`** — synthesizes its pose in its own `tick()` from an
  injected truth source (a reference to `Odometry` + the last body
  twist). All synthetic-generation logic lives inside this class.

Then:

- The loop holds an `Otos&` and is **build-agnostic and identical in
  both builds**:
  ```cpp
  otos_.tick(nowUs);
  applyOtosSample(otos_, nowUs, frame_);
  ```
  No `#ifdef` in `robot_loop.cpp`. `feedSyntheticSample()` is deleted
  from the real leaf.
- The **only** build-time choice (if the macro is kept at all) is in
  `main.cpp`, selecting which concrete type to construct and inject.

An abstract base with two subclasses is fine here — the vtable cost on
the M4F is negligible. (An earlier hedge about the leaf's
"no-virtual / isolation invariant" posture was overthinking it; virtual
dispatch is acceptable at this seam. A compile-time type alias is an
alternative if virtuals are still undesired, but the base-class form is
the default.)

## Scope / notes

- Landed sprint-120 code (120-002); this is a design-quality refactor,
  not a functional bug. Behavior of the real build must not change.
- `applyOtosSample()` ([`src/firm/app/odometry.cpp:51`](../../src/firm/app/odometry.cpp#L51))
  stays a free function and becomes the single perception path for both
  builds.
- Touch points: `src/firm/devices/otos.{h,cpp}`,
  `src/firm/app/robot_loop.cpp`, `src/firm/app/main.cpp`, plus the
  `app/DESIGN.md` / `devices/otos.h` header docs that currently describe
  the `#ifdef` seam.
- The prior `on-chip-fake-otos-test-device.md` issue that drove 120-002
  is gone (consumed at sprint-120 close); its "servo-mounted OTOS"
  premise was wrong (OTOS is rigidly mounted on the robot's I2C bus).
  Any pickup here should not resurrect that premise.
