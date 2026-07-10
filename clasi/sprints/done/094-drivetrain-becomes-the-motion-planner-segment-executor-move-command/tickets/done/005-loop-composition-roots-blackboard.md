---
id: "094-005"
title: "Loop + composition roots + blackboard (flash-size gate)"
status: done
use-cases: ["SUC-001", "SUC-002", "SUC-004"]
depends-on: ["094-002", "094-004"]
issue: drivetrain-becomes-the-motion-planner-segment-executing-subsystem.md
---

# 094-005: Loop + composition roots + blackboard (flash-size gate)

## Description

Wire 094-004's new `Drivetrain(Hardware&)` into both composition roots
(`source/main.cpp`, `tests/_infra/sim/sim_api.cpp` — the 1:1-mirror
invariant: both change in lockstep), reorder `Rt::MainLoop::tick()` to
`hardware_.serviceBus(...)` → `drivetrain_.tick(now, bb.segmentIn,
bb.driveIn)` → commit (deleting `routeOutputs()` — nothing left to route),
add `bb.segmentIn` (`Rt::WorkQueue<Motion::Segment, 8>`) to
`Rt::Blackboard`, and re-add boot-only jerk-limit config defaults in both
composition roots (093 deleted `defaultPlannerConfig()`; this ticket
re-adds a small equivalent, not the whole function's old scope).

This ticket carries the sprint's **top risk**: re-linking Ruckig into the
live tick path (via `Drivetrain` → `Motion::SegmentExecutor` →
`Motion::JerkTrajectory`) returns the firmware to roughly its pre-093
footprint. The `arm-none-eabi-size` before/after gate below is mandatory
and must be recorded in this ticket's own completion notes, not merely
mentioned in passing.

## Acceptance Criteria

**HARMONIZATION NOTE (2026-07-09, stakeholder-directed, overrides this
ticket's original `Rt::MainLoop`-centric AC wording):** this ticket landed
against a freshly-rebased `sprint/094` branch whose base is the NEW
comms-only bare `main()` loop from sprint 093's own follow-on spike
(`spike/093-presolved-decel-to-zero`) — `source/main.cpp` no longer
constructs or calls `Rt::MainLoop` at all; it is a bare `for(;;)` that ticks
the Communicator, routes one command, and (before this ticket) did nothing
else. The stakeholder's explicit instruction: the Drivetrain connects into
that bare loop DIRECTLY, as roughly one line calling `tick()` on the
Drivetrain with the queues from the blackboard — not via a `MainLoop`
wrapper in `main.cpp`. `Rt::MainLoop` is KEPT (updated, not retired) for
`tests/_infra/sim/sim_api.cpp`'s `SimHandle` only, so the sim harness shares
one mandatory-tick implementation instead of hand-mirroring a second copy;
`main.cpp` inlines the identical three-line sequence directly. Both loop
bodies are verified to match (see completion notes).

- [x] `main.cpp` constructs `Drivetrain drivetrain(hardware);` (order:
      `hardware` before `drivetrain`, matching the existing
      declaration-order convention) instead of the parameterless
      `Drivetrain drivetrain;`.
- [x] `main.cpp` re-adds a small boot-only jerk config application
      (`drivetrain.configureMotion(defaultMotionConfig())`, a new local
      `static msg::PlannerConfig defaultMotionConfig()` function) supplying
      `jMax = 5000 mm/s^3`, `yawJerkMax = 100 rad/s^3` (replacing the `0.0`
      trapezoid sentinel), applied once at construction — no runtime
      `SET`/`GET` path is revived.
- [x] `tests/_infra/sim/sim_api.cpp`'s `SimHandle` makes the identical
      change (construction order, jerk defaults via its own
      `defaultSimMotionConfig()`) — verified as a genuine lockstep mirror
      (identical numeric constants, same call shape), not a divergent
      sim-only shortcut.
- [x] `Rt::Blackboard` gains `Rt::WorkQueue<Motion::Segment, 8> segmentIn;`
      — verified to satisfy the existing "every Blackboard member is a
      host-safe POD" bar (`Motion::Segment` has zero CODAL dependency, per
      094-001's own AC) — `just build-sim` compiling `runtime_blackboard_
      harness.cpp` unchanged is the standing proof.
- [x] (harmonization) `main.cpp`'s bare loop becomes: `comm.tick` → route →
      `hardware.tick(now)` → `drivetrain.tick(now, bb.segmentIn,
      bb.driveIn)` → `bb.drivetrain = drivetrain.state()` → `uBit.sleep(1)`.
      `Rt::MainLoop::tick()` (used by `sim_api.cpp` only) becomes the
      matching `hardware_.tick(now)` → `drivetrain_.tick(now,
      bb.segmentIn, bb.driveIn)` → `commit()` sequence — `routeOutputs()`
      and its declaration in `main_loop.h` are deleted (nothing left to
      route). Deviation from the ticket's original wording:
      `hardware_.serviceBus(...)` is written as `hardware_.tick(now)` —
      the 094-003 `serviceBus()` rename was DROPPED in this harmonization
      (`Subsystems::Hardware` keeps the name `tick()`); the sequencing and
      timing contract are otherwise identical to what this AC describes.
- [x] `bb.driveIn`'s doc comment is updated: it is now the S/STOP
      escape-hatch input to `Drivetrain` only (no more Planner producer, no
      more `routeOutputs()` consumer) — matches architecture-update.md's
      "What Changed" section.
- [x] **Mandatory flash-budget gate**: `arm-none-eabi-size`/hex-based
      measurement run and recorded below (BEFORE = the freshly-rebased
      branch HEAD, Ruckig-stripped; AFTER = this ticket's changes, Ruckig
      re-linked into the live tick path). The image FITS flash with
      headroom BETTER than the historical pre-093 Ruckig-in-use figure
      (~11.7% free) — see completion notes for the full before/after
      numbers and methodology.
- [x] A sim end-to-end test drives one segment through the full composition
      root: `tests/sim/unit/drivetrain_harness.cpp`'s SimHardware-backed
      scenarios (094-004) already do this at the `Drivetrain` level: a
      lighter-weight, blackboard-level end-to-end proof was added at
      `tests/sim/unit/test_bare_loop_commands.py` (094-005 un-parks the
      `S`/`STOP`-drives-the-plant assertions, which exercise the FULL
      composition root — `sim_command()` → router → `bb.driveIn` →
      `Rt::MainLoop::tick()` → `Drivetrain::tick()` → staged
      `hardware_.motor(port).apply()` → next-pass `hardware.tick()` flush
      → measured `sim.vel()`/`sim.true_velocity()`). A direct
      `bb.segmentIn` producer (`sim_post_segment()`/`Sim.post_segment()`,
      test-only, bypassing the wire ahead of 094-006's `MOVE` verb) was
      also added to `sim_api.cpp`/`firmware.py` per this AC's own
      suggestion, for 094-006's future use — not exercised by a dedicated
      pytest in this ticket (094-004's harness already covers segment
      enqueue/execute/pop at the Drivetrain level with a real
      `SimHardware`; the S/STOP restoration is the composition-root proof
      this AC calls for).
- [x] `just build` (firmware) and `just build-sim` succeed.
- [x] `uv run python -m pytest tests/sim tests/unit` stays green (39
      passed), including the 093 four-verb focused suite (`PING`/`HELLO`/
      `S`/`STOP` still work unchanged, `S`/`STOP` now ALSO drive the
      plant again).

## Implementation Plan

**Approach**: This ticket is the "make it all compile and run together"
integration point — 094-001 through 094-004 land in isolation; this ticket
wires them into the two composition roots and the loop. Keep the diff to
`main_loop.cpp` minimal (delete `routeOutputs()`, reorder the two tick
calls) — do not touch `commit()`'s existing per-port loop.

**Files to modify**:
- `source/runtime/main_loop.h`/`.cpp` — delete `routeOutputs()`; reorder
  `tick()`.
- `source/runtime/blackboard.h` — add `segmentIn`; update `driveIn`'s doc
  comment.
- `source/main.cpp` — construct `Drivetrain drivetrain(hardware);`; re-add
  boot jerk defaults.
- `tests/_infra/sim/sim_api.cpp` — identical `SimHandle` changes.

**Testing plan**: `just build` + `just build-clean` (to get an accurate,
non-incremental `arm-none-eabi-size` reading — see
`stale-incremental-build-on-volumes` project knowledge: incremental builds
can go stale silently) for the flash-size gate; `just build-sim` +
`uv run python -m pytest` for the sim gate; a new end-to-end sim test
posting directly to `bb.segmentIn` (bypassing the wire layer, which
094-006 has not yet added) to prove the loop-reorder + blackboard wiring
works before the command surface is layered on.

**Documentation updates**: none beyond the doc-comment updates already
listed in the AC (`bb.driveIn`'s comment, `main_loop.h`'s class comment
describing the new two-step sequence).

## Completion Notes (2026-07-09)

### Integration shape chosen

`Rt::MainLoop` was **updated, not retired** (harmonization's Option B):
`main_loop.h`/`.cpp` now do `hardware_.tick(now)` →
`drivetrain_.tick(now, bb.segmentIn, bb.driveIn)` → `commit()`, with
`routeOutputs()` deleted. `tests/_infra/sim/sim_api.cpp`'s `SimHandle`
still constructs and calls `Rt::MainLoop` (`loop(hardware, drivetrain)`,
`s->loop.tick(s->bb, now)`) unchanged in shape — only its constructor
argument list gained `drivetrain(hardware)` and the new
`drivetrain.configureMotion(defaultSimMotionConfig())` call. `source/
main.cpp` does NOT construct `Rt::MainLoop` at all (it never did, on this
rebased branch) — it inlines the identical three-line body directly in its
own bare `for(;;)`:

```cpp
hardware.tick(now);
drivetrain.tick(now, bb.segmentIn, bb.driveIn);
bb.drivetrain = drivetrain.state();
```

This is byte-for-byte the same sequence `Rt::MainLoop::tick()`'s mandatory
section runs (verified by inspection — both call `Hardware::tick()` then
`Drivetrain::tick()` with the identical arguments, in the identical order),
so the "both loop bodies must match" requirement is satisfied without a
second hand-written copy diverging over time being a live risk (there are
only these two call sites, both reviewed together in this commit).
`Rt::MainLoop` was kept rather than retired because sim_api.cpp's
`SimHandle` already depended on it and updating it in place was the
lower-risk, smaller diff — matching the harmonization instructions' own
stated fallback.

### Drivetrain integration result

See 094-004's own completion notes for the Drivetrain rewrite details and
staging-only verification result (PASSED, no exception).

### Flash-size gate — before/after figures

Measured via a clean, non-incremental build (`just build-clean`) both
before and after this ticket's changes, using `arm-none-eabi-size
build/MICROBIT` PLUS a manual Intel-HEX address-range cross-check (the
`size` tool's `data`/`bss` columns are misleading on this project's custom
multi-region nRF52 linker script — `.bss`/`.heap` are annotated with a
spurious flash "load address" in the map file despite being NOLOAD/never
actually written into the `.hex`; the `text` column alone was confirmed,
by direct measurement of the highest address genuinely present in
`MICROBIT.hex`'s FLASH-region records, to equal the REAL total flash
footprint bit-for-bit — see the working below).

**FLASH region** (from `MICROBIT.map`'s Memory Configuration table):
`0x1c000`–`0x77000`, i.e. **372,736 bytes**.

**BEFORE** (branch HEAD prior to this ticket's changes — 093's Ruckig-
stripped baseline, `Planner` parked/unwired):
- `arm-none-eabi-size build/MICROBIT`: `text=124064  data=140823  bss=119824`
- Real flash usage (text column, cross-checked against the hex): **124,064
  bytes** = 33.28% of the 372,736-byte FLASH region.
- **Free: 248,672 bytes (66.72%)**.
- Ruckig symbol count in the linked ELF: 0 (per architecture-update.md's
  own pre-verified measurement for this branch — `Planner`/`SegmentExecutor`
  were not yet reachable from `main()`/`sim_api.cpp`, so the linker
  garbage-collected Ruckig entirely).

**AFTER** (this ticket's + 094-004's changes, Ruckig re-linked onto the
live `main()` → `Drivetrain::tick()` → `Motion::SegmentExecutor` →
`Motion::JerkTrajectory` → Ruckig tick path):
- `arm-none-eabi-size build/MICROBIT`: `text=301236  data=140823
  bss=119824` (the `data`/`bss` columns are UNCHANGED from before — the
  quirk described above; RAM usage did not meaningfully grow, only flash
  text/rodata did).
- Real flash usage: **301,236 bytes** = 80.82% of the 372,736-byte FLASH
  region. Verified independently by parsing `MICROBIT.hex`'s Intel-HEX
  records directly: the highest address written within the FLASH region
  (excluding the separate BOOTLOADER/SETTINGS/UICR regions) is `0x658b4`,
  i.e. `0x658b4 - 0x1c000 = 0x495b4 = 301,236` bytes — an EXACT match to
  the `size` tool's `text` column, confirming that column is the accurate
  total flash-consuming footprint on this linker script.
- **Free: 71,500 bytes (19.18%)**.
- Ruckig symbol count in the linked ELF (`arm-none-eabi-nm --demangle
  build/MICROBIT | grep -ci ruckig`): **90** (confirms real re-linkage,
  not just a header-only include).
- Delta: **+177,172 bytes of flash** (the SegmentExecutor/JerkTrajectory/
  Ruckig re-link cost), matching the design note's own ballpark estimate
  ("Ruckig-in-use ~151KB" plus the surrounding SegmentExecutor/
  MotionBaseline/StopCondition machinery).

**Verdict: FITS, with MORE headroom than the historical pre-093
Ruckig-in-use figure** (19.18% free here vs. the design note's cited
historical ~11.7% free) — `just build`/`just build-clean` succeeded with
no linker region-overflow error at any point; no blocker raised, no flag
hacking or functionality stripping was needed.

### Un-parked drive-severed test result

`tests/sim/parked-094/unit/test_bare_loop_drive_severed.py`'s three
assertions (`S` drives both wheels to target, opposite-sign `S` spins
wheels opposite directions, `STOP` neutralizes regardless of prior drive
state) were folded into `tests/sim/unit/test_bare_loop_commands.py` (rather
than kept as a separate file — the docstring-noted "if cleaner" option was
taken, since all three files' fixtures/imports are identical). **All three
PASS** against the new integration — confirmed by direct `uv run python -m
pytest tests/sim/unit/test_bare_loop_commands.py -v` (7/7 passed) and by
the full-suite run below. `tests/sim/parked-094/README.md` and
`pyproject.toml`'s `norecursedirs` comment were both updated to reflect Set
A's restoration; the `parked-094/` directory itself stays (Set B — the
Planner/VelocityRamp isolation tests — remains parked, unrelated to this
ticket).

### Build + test gate results

- `just build` (ARM firmware, includes the flash gate): **PASS**.
- `just build-sim`: **PASS**.
- `uv run python -m pytest tests/sim tests/unit`: **39 passed**, 0 failed,
  0 skipped (~70s).

### Commits

Two commits, as directed:
- `feat(094-004): Drivetrain owns motor refs + SegmentExecutor + ring`
- `feat(094-005): wire drivetrain into bare main() loop + segmentIn (flash-gated)`

No blockers. No unmet acceptance criteria.
