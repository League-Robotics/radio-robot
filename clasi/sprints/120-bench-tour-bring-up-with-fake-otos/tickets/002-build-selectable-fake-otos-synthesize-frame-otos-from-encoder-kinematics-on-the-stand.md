---
id: '002'
title: 'Build-selectable fake OTOS: synthesize frame.otos from encoder kinematics
  on the stand'
status: done
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

- [x] `Devices::Otos` gains a new synthetic-sample method (e.g.
      `feedSyntheticSample(x, y, heading, v_x, v_y, omega, nowUs)`) with
      the SAME freshness/present/connected semantics real reads already
      populate — `pose()`/`poseFresh()` read fresh every cycle it's
      called, `present()`/`connected()` read true.
- [x] The real build's `Devices::Otos::tick()`/`begin()` and every other
      existing method are byte-for-byte unchanged; a diff review (or an
      existing unit/sim regression covering the real path) confirms it.
- [x] `App::RobotLoop::cycle()` gains exactly one macro-gated branch at
      the existing Otos call site: real build calls `tick(nowUs)` as
      today; `FAKE_OTOS` build calls the new method, fed from that SAME
      cycle's `Odometry` pose/twist output.
- [x] `main.cpp`'s `Devices::Otos` construction line is unchanged text
      between the two builds.
- [x] A new CMake build option selects the `FAKE_OTOS` variant;
      documented (build instructions, e.g. in a README or the relevant
      `DESIGN.md`) as a compile-time-only choice.
- [x] A bench tour (multi-leg MOVE sequence) driven on the FAKE_OTOS
      build, on the stand, over the real serial link, shows `frame.otos`
      tracking the commanded path within a stated band (captured via
      `tlm_log.py` or equivalent, compared against encoder-derived
      pose), and the tour completes (closes) — recorded in this ticket.
- [x] `App::StateEstimator`'s fusion weights are confirmed unchanged
      (still 0.0) — this ticket makes `frame.otos` meaningful, it does
      NOT wire fusion into motion (sprint.md Scope, Out of Scope).
- [x] `src/firm/devices/DESIGN.md` gets a direct edit describing the new
      method and the `FAKE_OTOS` build seam (per this sprint's Design
      Overlay — not overlaid, ticket-direct-edit).
- [x] `src/firm/app/DESIGN.md`'s "120 (bench tour bring-up...)" paragraph
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

## Implementation Record

**Doc-edit location gotcha (worth recording):** an early stale `/Volumes`
filesystem read of the CANONICAL `src/firm/app/DESIGN.md` (before any
edits this ticket) spuriously showed ticket 001's own ack-ring prose
already present there — it is not; `git show HEAD:src/firm/app/DESIGN.md`
confirms ticket 001 never touched canonical, only the sprint's OVERLAY
copy (`clasi/sprints/120-.../design/DESIGN.md`, commit `2f1a2e9f`), per
this sprint's own "Overlaid" doc-editing convention (sprint.md's Design
Overlay section). This ticket's own §1/§2 doc edits for the Otos
call-site reorder + `feedSyntheticSample()` therefore landed on that SAME
overlay copy (matching ticket 1's own precedent), NOT canonical — an
initial mistaken edit to canonical was caught and reverted (`git checkout
--`) before committing. `src/firm/devices/DESIGN.md` (a DIFFERENT file,
NOT overlaid this sprint per sprint.md's own list) was edited directly on
canonical throughout, correctly, per its own "ticket-direct-edit" owner
note.

**Approach step 2 resolved:** `present()`/`connected()` are neither
hardcoded true at construction nor left gated behind `begin()`'s real
probe — `feedSyntheticSample()` itself sets `initialized_`/`connected_`
true as its own side effect, the first time (and every time) it is
called, mirroring how a real `tick()` read already sets `connected_`
after a successful burst. `present()`/`connected()`'s own bodies are
untouched (zero `#ifdef` inside either — byte-identical source in both
builds); `Preamble`'s boot-time `begin()` probe still runs unchanged in a
`FAKE_OTOS` build (harmless real I2C traffic this call site does not
depend on) but is no longer load-bearing for `present()`/`connected()`
once `RobotLoop::cycle()` starts calling `feedSyntheticSample()` every
cycle — the more literal "zero real-chip dependency" reading, achieved
without any `#ifdef` inside the two accessors themselves.

**Call-site reorder:** to feed Otos "that SAME cycle's" `Odometry`
pose/twist (not the previous cycle's), `odom_.integrate()`/`frame_.pose`
staging is hoisted to run immediately BEFORE the (single) macro-gated
Otos branch in `robot_loop.cpp`'s trailing pace block — previously it ran
after. Verified side-effect-free for the real build: `Odometry::
integrate()` reads neither `otos_` nor any `frame_.otos*` field (and vice
versa), so the two operations commute; `stateEstimator_.update()` still
runs after both, unaffected.

**Build command (FAKE_OTOS variant):**
```
uv run python3 build.py --fw-only --fake-otos --clean
```
(`--fake-otos` always passes an explicit `-DFAKE_OTOS=ON`/`OFF` to cmake,
so a stale `CMakeCache.txt` from a prior invocation never leaves the flag
silently stuck; `--fw-only` skips the unaffected host-sim build.) Plain
`uv run python3 build.py` / `just build-clean` (no `--fake-otos`) build
the real, table variant — confirmed to compile identically before and
after this ticket (`just build-clean` also rebuilds `libfirmware_host`,
the HOST_BUILD sim library, which never defines `FAKE_OTOS`).

## Hardware Verification Results (2026-07-23, robot "tovez",
`/dev/cu.usbmodem2121102`, UID
`9906360200052820a8fdb5e413abb276000000006e052820`)

Deployed via `mbdeploy deploy <uid> --hex MICROBIT.hex` (run from the repo
root — `MICROBIT.hex` lands at the repo root per `codal.json`'s
`output_folder` default, NOT `build/`). APPROTECT auto-mass-erase fired on
the first deploy attempt (expected/normal); the very next flash attempt
succeeded cleanly. `twist_drive.py` 6/6 after flashing.

**Otos tracks commanded motion (the core acceptance):**
- Forward drive (300mm/s, 2s): `pose=(555,-119,-22.9deg)`
  `otos=(555,-119,-22.9deg)` — otos.x climbed with the encoder pose,
  exactly, not near-static.
- 90° turn (omega=1.5rad/s): `pose=(548,-123,+67.2deg)`
  `otos=(548,-123,+67.2deg)` — otos.heading changed ~90° with the
  commanded turn, exactly matching pose.
- `otos_present`/`otos_connected` both `True` throughout, on a build that
  never depends on the real chip.
- Confirmed the REAL (table) build's own physical symptom is UNCHANGED
  (proving the real path genuinely untouched, not just via code diff):
  same forward-drive command on the real build gave
  `pose=(569,-58,-12.3deg)` (encoders counting) vs.
  `otos=(47,-3,0.0deg)` (near-static) — the exact "useless on a stand"
  behavior this ticket's own source issue describes.

**Bench tour with retry (`src/tests/bench/fake_otos_tour_bench.py`,
TOUR_1, 13 legs) — two full runs, both closed:**
- Run A: 13/13 legs `completed`, `stopped_at=None` (CLOSED). 4 enqueue
  retries fired across the tour (each recovered on attempt 2/4), 0 final
  failures. `frame.otos` vs `frame.pose`: 435/435 polled frames
  `otos_present=True`, max position deviation 0.00mm (band <5mm), max
  heading deviation 0.00deg (band <2deg) — OVERALL PASS.
- Run B (rerun for reproducibility): 13/13 legs `completed`, CLOSED. 4
  retries, 0 final failures. 436/436 frames `otos_present=True`, 0.00mm /
  0.00deg deviation — OVERALL PASS.
- A third run (before the retry wrapper's single-consumer-queue bug was
  fixed — see `fake_otos_tour_bench.py`'s own module docstring) genuinely
  FAULTed at leg 6/13 on a 15s `Move.timeout`: root-caused via the 4-phase
  debugging protocol to the retry wrapper's own `wait_for_ack()` call
  destructively draining the shared TLM queue and silently discarding a
  DIFFERENT, concurrently-active leg's own completion frame while
  confirming a lookahead leg's enqueue ack — the same single-consumer race
  `turn_prediction_capture.py`'s own docstring already diagnosed for the
  sim. Fixed by making the wrapper the one buffering, non-lossy consumer
  of the queue; confirmed via a targeted replay test reproducing the exact
  interleaving, then reproduced clean on hardware twice (Runs A/B above).
- Tour-wide position/heading closure (~750-1370mm/~120-155deg across
  three runs) is real, uncalibrated dead-reckoning drift on this session's
  untuned bench robot over a 13-leg tour — reported by the bench script,
  deliberately NOT gated (motion-accuracy tuning is explicitly out of this
  sprint's scope, sprint.md Out of Scope; `otos` reproduces this drift
  IDENTICALLY to `pose` every frame, confirming it is genuine robot
  behavior, not a synthetic-feed artifact).
- Robot stopped and port released after every run
  (`.claude/rules/hardware-bench-testing.md`).

Real (table) firmware rebuilt and reflashed after the FAKE_OTOS session
(`uv run python3 build.py --fw-only --clean` + `mbdeploy deploy`);
`twist_drive.py` 6/6 confirms the robot is left in its default, real-OTOS
state.

**Suite:** `uv run python -m pytest` — 1393 passed, 2 skipped, 9 xfailed,
2 xpassed, 0 failed (includes a new `devices_otos_harness.cpp` scenario
for `feedSyntheticSample()`; every pre-existing scenario passes
unmodified). Both the default/real ARM build (`python3 build.py --fw-only`)
and the `FAKE_OTOS` variant compile cleanly; `just build-clean` (default
variant + HOST_BUILD sim lib) unaffected.
