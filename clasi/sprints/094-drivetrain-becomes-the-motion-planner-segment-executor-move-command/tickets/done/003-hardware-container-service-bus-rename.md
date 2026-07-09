---
id: "094-003"
title: "Hardware container: tick() → serviceBus()"
status: done
use-cases: ["SUC-004"]
depends-on: []
issue: drivetrain-becomes-the-motion-planner-segment-executing-subsystem.md
---

# 094-003: Hardware container: tick() → serviceBus()

## RESOLUTION — DESCOPED during master-harmonization (2026-07-09)

The `serviceBus()` rename was **dropped, not implemented.** When `sprint/094`
was rebased onto master (after sprint 093 closed and merged), master had kept
`Subsystems::Hardware::tick(now)` and reshaped `main()` into a comms-only bare
loop. Re-applying the rename produced 12 conflicts across the hardware files
for a purely cosmetic change; the stakeholder's steer was "minimal / harmonize
with master," so the rename commit was skipped in the rebase.

**The ticket's actual PURPOSE is still met** — Hardware is now a pumped
container that `Drivetrain` writes through (via `hardware.motor(port).apply()`),
not a controlling subsystem — but that role is realized by **094-005's**
bare-loop integration (`hardware.tick(now)` pumps the flip-flop once per pass;
`Drivetrain` owns the wheels), NOT by a method rename. Hardware keeps its
`tick(now)` name. No AC below was implemented; they are moot. Closed as
descoped, its intent absorbed into 094-004/005.

## Description

Rename `Subsystems::Hardware::tick()` (and both concrete overrides,
`NezhaHardware::tick()`/`SimHardware::tick()`) to `serviceBus()` — **a pure
rename, no behavior change**. This signals the container's new role: it is
a bus scheduler `Drivetrain` writes through, not a ticked controller in its
own right (that responsibility moves to `Drivetrain` in 094-004).

This ticket is independent of 094-001/002 (no dependency on the executor or
Planner's removal) and can run in parallel with them. It is the load-bearing
"timing must not change" ticket — the whole sprint's flip-flop-safety
argument rests on this being verifiably a no-op rename.

## Acceptance Criteria

- [ ] `Subsystems::Hardware::tick(...)` → `Subsystems::Hardware::
      serviceBus(...)` (pure virtual, same signature/parameters).
- [ ] `NezhaHardware::tick(...)` → `NezhaHardware::serviceBus(...)` — the
      `REQUEST_DUE`/`COLLECT_DUE` flip-flop state machine body
      (`nezha_hardware.cpp:34-68`), the `motorIn[]`/`motorResetIn[]`
      draining order (consumed before the flip-flop's own scheduling
      decision), and the `bus_.clear(kNezhaDeviceAddr)` settle gate are all
      byte-for-byte unchanged — only the method name changes.
- [ ] `SimHardware::tick(...)` → `SimHardware::serviceBus(...)` — the dt=0
      re-entry guard (`sim_hardware.cpp:52-54`) and its
      `lastAdvancedNow_`/`hasAdvanced_` bookkeeping are unchanged.
- [ ] Every call site (`Rt::MainLoop`, `tests/_infra/sim/sim_api.cpp`, any
      host-unit-test harness that calls `hardware.tick(...)` directly) is
      updated to call `serviceBus(...)` instead — grep confirms zero
      remaining call sites named `.tick(` against a `Hardware&`/
      `NezhaHardware&`/`SimHardware&` receiver anywhere in `source/` or
      `tests/`.
- [ ] Existing flip-flop/encoder host-unit tests (e.g. the 079-004 flip-flop
      harness) pass unchanged after the rename, with **zero** test-body
      edits beyond the method-name substitution itself — this is the
      ticket's own proof that timing did not change.
- [ ] `just build` and `just build-sim` both succeed.
- [ ] `uv run python -m pytest` stays fully green.

## Implementation Plan

**Approach**: A global, mechanical rename across three files' declarations/
definitions plus every call site. Do not touch any line inside the renamed
method bodies beyond the signature line itself (the method name in the
`::` qualifier and the header declaration) — if a diff to this ticket shows
any change to flip-flop sequencing, dead-time, or the dt=0 guard's
comparison logic, that is out of scope and must be reverted.

**Files to modify**:
- `source/subsystems/hardware.h` — base virtual declaration + doc comment
  (update prose that says "tick()" to say "serviceBus()" where it
  describes this method; leave the rest of the file header's design
  rationale untouched).
- `source/subsystems/nezha_hardware.h`/`.cpp` — declaration + definition.
- `source/subsystems/sim_hardware.h`/`.cpp` — declaration + definition.
- `source/runtime/main_loop.cpp` — its call site (this ticket only renames
  the call; 094-005 changes the surrounding sequencing).
- `tests/_infra/sim/sim_api.cpp` — any direct call site, if present outside
  what `MainLoop::tick()` already covers.
- Any `tests/sim/unit/*_harness.cpp` file that calls `.tick(...)` on a
  `Hardware`/`NezhaHardware`/`SimHardware` instance directly.

**Testing plan**: run the existing flip-flop/encoder/dt=0-guard host unit
tests unchanged (method-name substitution only) and confirm they still pass
— this IS the test for "no timing change." Run `just build-sim` +
`uv run python -m pytest` for the full sim gate.

**Documentation updates**: update any doc comment in the three touched
headers that names `tick()` where it now means `serviceBus()` (e.g.
`hardware.h`'s "Contract every concrete Hardware::tick(now) must satisfy"
sentence). No wire-protocol or ticket-facing doc changes — this is an
internal method rename with no wire-visible effect.
