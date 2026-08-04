---
id: '003'
title: 'Live tuning surface: DBG verbs, velocity-profile-gate flags, and the stop-tail
  bench cases'
status: done
use-cases:
- SUC-001
- SUC-002
depends-on:
- '002'
github-issue: ''
issue: B-wheel-controller-position-loop-and-tuning.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Live tuning surface: DBG verbs, velocity-profile-gate flags, and the stop-tail bench cases

## Description

Ticket 004 is a bench session on `tovez`, and the stakeholder is waiting for it.
This ticket builds the instruments that session needs — nothing more. Every item
here exists because 004's acceptance cannot be measured without it.

Reference: `clasi/issues/B-wheel-controller-position-loop-and-tuning.md` §7-F
and §7-G, and `clasi/issues/attachments/wheel-controller-2026-08-03/gate-changes.patch`
(that patch contains **only** `velocity_profile_gate.py`).

### A. Four RAM-only DBG verbs — `ROBOT_DEBUG` builds only

`src/firm/app/comms.h`, `comms.cpp`, `robot_loop.cpp`. All four follow the
existing `kMark`/`kWedge` pattern — an enum arm, a parser arm in
`classifyDbgArg()`, an apply arm in `applyDbgAction()`. `comms.h` gains
`float value2` (for the two-argument `gain` verb) and the enum arms.

| verb | effect |
|---|---|
| `DBG:vmin <mm/s>` | `drive_.setSpeedFloor()` |
| `DBG:gain <L> <R>` | scales the baked `kDutyPerSpeed` per wheel |
| `DBG:asteady <mm/s^2>` | `drive_.setASteady()` |
| `DBG:pos <mm>` | `drive_.setPositionErrorMax()` — new in ticket 002 |

Each **must echo `"... applied"`**. That echo is not cosmetic: it is what the
gate waits for before it will report a run, and a verb that applies silently
produces a run the gate reports against gains it never actually set.

These are a **development** tuning surface, and that is sanctioned —
`.claude/rules/configuration-discipline.md` binds production boot, not bench
tuning, precisely because read-back makes ad-hoc pushes safe. The permanent home
for a tuned value is still `data/robots/tovez.json` (ticket 004 promotes them).

### B. `velocity_profile_gate.py` flags

`src/tests/bench/velocity_profile_gate.py` is the standing distance-fidelity
gate and is already confirmed clean of the deleted `.config()` API. Apply
`gate-changes.patch`:

- `--vmin --pid-kp --pid-ki --pid-kff --pid-imax --pid-kaw --poserr --asteady
  --gain-left --gain-right`, asserted **after connect**. Opening the port resets
  the board, so config set by any other process is already gone — after connect
  is the only place it survives. (Ticket 006 later removes the reset; that is
  deliberately sequenced *after* the bench run so the measurement transport does
  not change on the eve of the measurement. Do not pre-empt it here.)
- `--wheel {both,left,right}` for single-motor rigs.
- `--attempts N`: discard truncated captures and reconnect to clear a wedged
  board. This one matters more than it looks — a short capture reads as a short
  **distance** (77 mm of 450 mm, once) and is indistinguishable from real
  under-travel. Without it the gate silently reports control errors that are
  actually capture errors.

### C. Stop-tail bench cases — extend, do not create

Ticket 004 must measure four stop tails: silent host, one `estop()`, one
`wheels(0,0)`, and streaming zeros.

> **Extend `src/tests/bench/estop_unlosable_bench.py`, which is already on
> master.** It was written for 129-001 and already proves the closest
> neighbouring property: one `estop()` per trial, ten consecutive trials on the
> same connection and same boot, encoders must stop within a bound and stay
> stopped. Its per-trial structure, its "timestamp the wire write not the ack"
> discipline, and its settle-window polling are all directly reusable.
>
> What it lacks, and what this ticket adds: the **silent-host** tail, the
> **one-`wheels(0,0)`** tail, and a measurement of **travel accumulated after
> commanded zero** (the issue's headline number — 936 mm as shipped).
>
> **Do not create `tail_forensics.py` from scratch.** The issue references it
> and its plotters, but they are **not on master** and they are **not
> recoverable** from `src/tests/bench/output/tuned_20260803/` — that directory
> holds only run data plus the two `.patch` files, no `.py` at all. Confirmed;
> do not spend time hunting for them. A fresh harness is the fallback only if
> extending the existing bench proves genuinely awkward, and this sprint is
> meant to be short.

Carry across one measurement bug the issue documents as fixed: `t` anchored on
the baseline frame (captured during the leading settle window) charges the
profile's own last second to the tail and inflates every tail figure by roughly
150 mm. Compute from the **actual commanded-zero transition**.

### D. Minimal repair of two bench scripts — and only these two

Five HITL scripts still call the deleted `.config(**...)` API
(`plant_id.py`, `velocity_step_response.py`, `wheel_controller_ab_bench.py`,
`move_protocol_bench.py`, `src/tests/dev/rig_dev.py`) — tracked in
`clasi/issues/B-sprint-132-capability-gaps-and-broken-bench-scripts.md`.

**That issue is NOT adopted by this sprint.** Repair exactly the two a
wheel-controller tuning session reaches for, because they will otherwise fail at
the bench in the middle of ticket 004:

- `src/tests/bench/velocity_step_response.py` (line ~267)
- `src/tests/bench/wheel_controller_ab_bench.py` (lines ~474, ~483, ~509)

Port them to the current config API (`set_field`/`config` group push per
`src/host/robot_radio/robot/protocol.py`). Leave `plant_id.py`,
`move_protocol_bench.py`, and `rig_dev.py` alone — out of scope, and touching
them grows a sprint that is deliberately short.

### Build trap

**A plain `just build` compiles the DBG channel OUT**, after which the gate
aborts on an unconfirmed `DBG:vmin`. Always
`uv run python build.py --clean --robot-debug`. This is the single most likely
way this ticket appears broken when it is not.

## Acceptance Criteria

- [x] Four `DBG` verbs (`vmin`, `gain`, `asteady`, `pos`) exist behind
      `ROBOT_DEBUG`, each following the existing `kMark`/`kWedge` enum/parser/
      apply pattern, and each echoes `"... applied"`.
- [x] `comms.h` carries `float value2` and the new enum arms.
- [x] `DBG:pos` reaches `Drive::setPositionErrorMax()` (ticket 002's setter).
- [x] `velocity_profile_gate.py` accepts the ten tuning flags plus `--wheel` and
      `--attempts`, and asserts config **after** connect.
- [x] `--attempts` discards truncated captures and reconnects, so a short capture
      cannot be reported as a short distance.
- [x] `estop_unlosable_bench.py` covers all four tails — silent, one `estop()`,
      one `wheels(0,0)`, streaming zeros — and reports travel accumulated after
      the **commanded-zero transition**, not from the baseline frame.
- [x] `velocity_step_response.py` and `wheel_controller_ab_bench.py` no longer
      call the deleted `.config()` API and run against current firmware.
      *(API port done and unit-tested against a protocol double; NOT executed
      against a robot — see the hardware note below.)*
- [x] `plant_id.py`, `move_protocol_bench.py` and `rig_dev.py` are **untouched**.
- [x] A `--robot-debug` build succeeds and the gate confirms a `DBG:vmin` round
      trip. *(Build: verified, both arms — `uv run python build.py --clean
      --robot-debug` AND a plain `--clean` release build. Round trip: verified
      end to end through the REAL compiled firmware in the Sim
      (`src/tests/sim/system/test_dbg_tuning_verbs.py`) — inject `DBG:vmin 60`
      → `classifyDbgArg()` → ring → `applyDbgAction()` → `Drive::setSpeedFloor()`
      → `debugf()` → `DBG:vmin 60.000 applied` back on the wire. NOT verified
      by the gate against `tovez` — see below.)*

## Hardware verification NOT performed — read before ticket 004

**`tovez` was not attached for the whole of this ticket.** `mbdeploy list`
showed only `getez` (RADIOBRIDGE) and `vevov` (NEZHA2), both other people's
work, so nothing was flashed and nothing ran on a robot
(`.claude/rules/hardware-bench-testing.md`: "If `mbdeploy list` does not show
`tovez`, it is unplugged — stop and say so").

Verified in Sim / by unit test only, and therefore **first to check on the
stand**:

1. The DBG echo round trip over a real serial link (verified through the
   compiled firmware in Sim, but never over the wire).
2. `velocity_profile_gate.py --vmin ...` end to end, including
   `_assert_tuning()`'s `on_debug` capture of the echo — the Sim test proves
   the firmware emits it; nothing has yet proved `SerialConnection`'s
   `on_debug` hook delivers it to the gate.
3. The four stop tails. `estop_unlosable_bench.py`'s tail arithmetic is
   unit-tested on synthetic frames; no tail has been measured on a robot.
4. Both repaired bench scripts (`velocity_step_response.py`,
   `wheel_controller_ab_bench.py`) against real firmware.

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/sim
  src/tests/unit` — the comms/DBG parser arms are unit-testable and the
  `ROBOT_DEBUG` path must not perturb the non-debug build.
- **New tests to write**:
  - Unit tests for `classifyDbgArg()`'s four new arms, including the
    two-argument `gain` form via `value2` and rejection of malformed input.
  - A test that the non-`ROBOT_DEBUG` build is unaffected (the verbs compile out
    cleanly, nothing else changes).
  - Host-side: the repaired `velocity_step_response.py` and
    `wheel_controller_ab_bench.py` import and construct their config pushes
    against the current protocol API without touching hardware.
- **Verification command**: `uv run python -m pytest src/tests/sim src/tests/unit`
- **Bench smoke (not full acceptance)**: a single `--robot-debug` flash to
  `tovez` by UID confirming one `DBG:vmin` applies and echoes. Full bench
  acceptance is ticket 004.
