---
status: pending
filed: 2026-07-24
filed_by: team-lead (stakeholder review of the base, sprint 122 follow-up)
related:
- firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md
- otos-fake-seam-should-be-one-interface-two-implementations.md
sprint: '124'
---

# FakeOtos belongs in devices/, not app/ — resolve the isolation-invariant tension

## Observation (stakeholder, 2026-07-24)

`App::FakeOtos` (`src/firm/app/fake_otos.{h,cpp}`) is conceptually a
`Devices::Otos` implementation — the sibling of `Devices::RealOtos` — and
belongs in `devices/`, next to it, not in `app/`.

## Why it's currently in app/ (the blocker)

`FakeOtos` reports the dead-reckoned pose + wheel-fused twist, so it depends
on `Motion::Odometry` and `BodyKinematics` (both in `src/motion`) plus
`Devices::Motor`. The `devices/` **isolation invariant** (devices/DESIGN.md
§3) forbids `devices/*.{h,cpp}` from depending on `motion/` (or `app/`,
`messages/`). That `Motion::Odometry` dependency is exactly why the OTOS
fake-seam refactor parked `FakeOtos` in `app/` (the composition layer that
can see both `devices/` and `motion/`). Moving it into `devices/` as-is
would break the invariant and the build.

## Resolution options (for sprint 124's design)

1. **Relax the invariant for a bench/test device** — allow `FakeOtos` (and
   only it) in `devices/` to depend on `motion/`, documented as the one
   test/bench-variant exception.
2. **Restructure FakeOtos's inputs** so it depends only on `devices/` types
   — e.g. take a minimal pose-source seam declared in `devices/`, or hold
   the two `Devices::Motor&` only and get pose from a `devices/`-level
   provider. (Risks reintroducing a per-cycle feed or duplicating odometry.)
3. **A new home for the OTOS implementations** — e.g. a small
   `devices/`-adjacent tree that is allowed to see both layers, holding
   `RealOtos` + `FakeOtos`.

This lands naturally in sprint 124 (the base-hardening rewrite that
reorganizes the base/motion boundary and the device stack). Stakeholder
directed it NOT be done out-of-process — it needs the design decision
above, not a blind move.

## Companion cleanup (already in the 124 plan)

`src/firm/app/drive.{h,cpp}` is now a vestigial 3-method wheel-velocity sink
(`setWheels` latches two floats, `tick()` forwards to `setVelocity`,
`trackWidth_` is dead). The stakeholder wants velocities set directly in the
loop. That is already the sprint-124 explicit-dataflow rewrite
(`docs/design/base-explicit-loop-sketch.md`: "Drive ceases to exist as
wiring"); recorded here so the two base-cleanup observations stay together.
