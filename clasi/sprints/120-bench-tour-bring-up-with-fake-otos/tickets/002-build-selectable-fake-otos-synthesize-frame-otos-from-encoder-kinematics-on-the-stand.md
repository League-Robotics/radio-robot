---
id: '002'
title: 'Build-selectable fake OTOS: synthesize frame.otos from encoder kinematics
  on the stand'
status: in-progress
use-cases:
- SUC-070
depends-on:
- '001'
github-issue: ''
issue: on-chip-fake-otos-test-device.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Build-selectable fake OTOS: synthesize frame.otos from encoder kinematics on the stand

## Description

The real OTOS chip is genuinely present, connected, and read every cycle
on the bench robot (a premise correction — earlier triage wrongly
assumed a disconnected/servo-mounted sensor). The actual gap: on the
stand, wheels spin free and the robot never translates, so the real
OTOS reports a near-static pose while the encoders count — `frame.otos`
is real but useless for verifying that a bench tour tracks its commanded
path. This ticket adds a build-selectable, on-chip fake OTOS that
synthesizes `frame.otos` from the SAME encoder-kinematics forward
integration `App::Odometry` already computes every cycle, reported back
through the identical `Devices::Otos` type and the identical
`pose()`/`poseFresh()`/`present()`/`connected()` contract every
downstream consumer already reads — so a bench tour exercises the exact
OTOS-present code path the table (real-chip) build uses, with the real
build provably unchanged.

Depends on ticket 001 (ack FIFO): reliably driving and verifying a
multi-leg tour on the real link needs trustworthy enqueue-ack
observability for each leg.

**Design decisions already made in sprint.md's Architecture (read before
implementing — do not re-derive from scratch):**
- Decision 2: the fake is a NEW METHOD on the SAME `Devices::Otos` type
  (not a second, polymorphic implementation) — `Otos` stays a concrete,
  non-virtual leaf, matching every other device leaf's established "one
  shape, no inheritance" convention (`devices/DESIGN.md` §4).
- Decision 3: the build-variant branch lives in `RobotLoop::cycle()`
  (`app/robot_loop.cpp`)'s existing per-cycle Otos call site, NOT in
  `main.cpp`'s construction — `main.cpp`'s `Devices::Otos otos(bus,
  otosConfig)` line stays IDENTICAL between the real and bench builds.
  `main.cpp` only gains the build option's plumbing (a CMake define).
- The new method mirrors `setPose()`'s existing "staged, drained by the
  next `tick()`-equivalent call" shape, but (unlike `setPose()`) reports
  FRESH and carries the FULL pose+twist — `setPose()` itself is NOT
  reused; it is scoped for a rare reset operation, wrong contract for
  "every cycle, fresh, with velocity."
- `Devices::Otos` must NOT gain any dependency on `App::Odometry` or any
  other `app/` type (devices isolation invariant) — `RobotLoop` is the
  one place both are in scope and does the feeding.
- Build selection is compile-time only (a `FAKE_OTOS`-style CMake
  option), never a runtime/wire toggle.

See sprint.md's Architecture Step 3/5/6 (Decisions 2, 3) and the Design
Overlay's `src/firm/app/DESIGN.md` overlay ("120 (bench tour bring-up...)"
paragraph) for the full rationale and drafted doc text.

## Acceptance Criteria

- [ ] `Devices::Otos` gains a new synthetic-sample method (e.g.
      `feedSyntheticSample(x, y, heading, v_x, v_y, omega, nowUs)`) with
      the SAME freshness/present/connected semantics real reads already
      populate — `pose()`/`poseFresh()` read fresh every cycle it's
      called, `present()`/`connected()` read true.
- [ ] The real build's `Devices::Otos::tick()`/`begin()` and every other
      existing method are byte-for-byte unchanged; a diff review (or an
      existing unit/sim regression covering the real path) confirms it.
- [ ] `App::RobotLoop::cycle()` gains exactly one macro-gated branch at
      the existing Otos call site: real build calls `tick(nowUs)` as
      today; `FAKE_OTOS` build calls the new method, fed from that SAME
      cycle's `Odometry` pose/twist output.
- [ ] `main.cpp`'s `Devices::Otos` construction line is unchanged text
      between the two builds.
- [ ] A new CMake build option selects the `FAKE_OTOS` variant;
      documented (build instructions, e.g. in a README or the relevant
      `DESIGN.md`) as a compile-time-only choice.
- [ ] A bench tour (multi-leg MOVE sequence) driven on the FAKE_OTOS
      build, on the stand, over the real serial link, shows `frame.otos`
      tracking the commanded path within a stated band (captured via
      `tlm_log.py` or equivalent, compared against encoder-derived
      pose), and the tour completes (closes) — recorded in this ticket.
- [ ] `App::StateEstimator`'s fusion weights are confirmed unchanged
      (still 0.0) — this ticket makes `frame.otos` meaningful, it does
      NOT wire fusion into motion (sprint.md Scope, Out of Scope).
- [ ] `src/firm/devices/DESIGN.md` gets a direct edit describing the new
      method and the `FAKE_OTOS` build seam (per this sprint's Design
      Overlay — not overlaid, ticket-direct-edit).
- [ ] `src/firm/app/DESIGN.md`'s "120 (bench tour bring-up...)" paragraph
      (already drafted in this sprint's design overlay) is
      verified/refined against the shipped code and applied to the
      canonical doc at sprint close.

## Implementation Plan

### Approach

1. Add `Devices::Otos::feedSyntheticSample(...)` (naming per the
   project's no-units-in-identifiers convention — quantity names, units
   in `// [unit]` comment tags) — stages the given pose+twist and marks
   it fresh, mirroring `setPose()`'s staged-drain shape but with fresh
   `poseFresh()`/populated velocities and no bus interaction whatsoever
   (no real burst read attempted in `FAKE_OTOS` builds — literal reading
   of the source issue: "instead of reading the I2C chip").
2. Decide (and document in this ticket) whether `present()`/`connected()`
   are hardcoded true at construction in `FAKE_OTOS` builds (no real
   chip dependency at all — the more literal reading of the issue, and
   the more useful posture for ticket 003's own "OTOS tick present vs.
   skipped" diagnostic dimension) or still gated behind a real
   `begin()` probe. Sprint's own Design Rationale leans toward the
   former (zero real-chip dependency in the bench build); confirm during
   implementation and note the final call here.
3. Add a `FAKE_OTOS` compile-time macro (CMake option, threaded through
   the firmware target — likely `src/utils/` CMake helpers or the
   top-level firmware `CMakeLists.txt`).
4. In `app/robot_loop.cpp`'s `cycle()`, add the ONE macro-gated branch at
   the existing Otos call site: `#ifdef FAKE_OTOS` calls
   `otos_.feedSyntheticSample(...)` sourced from `odom_`'s just-integrated
   pose/twist (available in the same trailing pace block, after
   `odom_.integrate()`); `#else` calls `otos_.tick(nowUs)` exactly as
   today.
5. Confirm `main.cpp`'s `Devices::Otos otos(bus, otosConfig)` construction
   line needs no change (the `bus` reference is harmless to hold even if
   unused in `FAKE_OTOS` builds).
6. Build both variants (`just build-clean` for the default/real build;
   the new CMake option for the bench build), flash the BENCH build via
   `mbdeploy deploy <robot-UID> --hex MICROBIT.hex` (UID
   `9906360200052820a8fdb5e413abb276000000006e052820`; APPROTECT
   auto-mass-erase expected/normal; reflash once more if comms look
   malformed post-mass-erase).
7. Drive a bench tour on the stand over the real serial link; capture
   `frame.otos` alongside encoder-derived pose (`tlm_log.py` or
   equivalent); confirm tracking within a stated band and tour closure.
8. Rebuild/reflash the REAL (table) variant and confirm — via diff
   review and/or existing regression — that its behavior is unchanged.

### Files to Create/Modify

- `src/firm/devices/otos.h` / `.cpp` — new synthetic-sample method.
- `src/firm/app/robot_loop.cpp` — one macro-gated call-site branch.
- `src/firm/main.cpp` — build-option plumbing only (construction line
  unchanged).
- CMake build config (`src/utils/` or the firmware target's
  `CMakeLists.txt`) — new `FAKE_OTOS` option.
- `src/firm/devices/DESIGN.md` (canonical) — direct edit.
- `src/firm/app/DESIGN.md` (canonical) — apply this sprint's overlay
  edit at close.

### Testing Plan

- Unit/sim: the synthetic-sample method's pure computation (staging,
  freshness marking) can be covered off-hardware.
- Hardware (required): bench tour on the FAKE_OTOS build, on the stand,
  over `/dev/cu.usbmodem2121102`; capture and compare `frame.otos` vs.
  encoder-derived pose; confirm tour closure. Rebuild/reflash the real
  build and confirm unchanged behavior. Record results in this ticket.

### Documentation Updates

- `src/firm/devices/DESIGN.md` — direct edit (new method, build seam).
- `src/firm/app/DESIGN.md` — apply this sprint's overlay diff.
