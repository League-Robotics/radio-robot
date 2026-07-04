---
id: '006'
title: 'New tests/ structure: park old tests, three-domain skeleton, velocity_chart
  reinvigorated'
status: done
use-cases:
- SUC-007
depends-on:
- '005'
github-issue: ''
issue: greenfield-rebuild-faceplate-hal-in-a-fresh-source-old-tree-parked.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# New tests/ structure: park old tests, three-domain skeleton, velocity_chart reinvigorated

## Description

Build the new `tests/` tree: three independent test domains (`sim/`,
`bench/`, `playfield/`) that are never combined, plus the kept `unit/` and
`tools/` categories. `tests/` was already renamed to `tests_old/` in ticket
1 — this ticket populates the new `tests/` from scratch. Depends on ticket 5
because `velocity_chart.py`'s reinvigoration targets the `DEV` protocol
ticket 5 implements.

Note: `git mv tests tests_old` already happened in ticket 1 (see ticket
001's acceptance criteria). If for any reason it did not, do it here first,
as one commit, before building the new tree — but check ticket 1's actual
result before assuming this.

## Acceptance Criteria

- [x] New `tests/` skeleton matches the issue's locked layout:
  ```
  tests/
    CLAUDE.md          # fresh: structure, the three domains, pointer to .claude/rules/
    conftest.py
    sim/
      unit/
      system/
    bench/
      dev_exercise.py
      pid_hold_speed.py
      ratio_governor_curve.py
      velocity_chart.py
    playfield/
      plot_square.py
      world_goto_chart.py
    unit/
    tools/
  ```
- [x] `tests/sim/` merges the OLD `tests_old/sim/` + `tests_old/simulation/`
      pair conceptually — this ticket creates `tests/sim/{unit,system}` as
      skeleton + `conftest.py` only (no test content ported yet; a fresh sim
      harness under `tests/sim/` is explicitly later-ticket work, not this
      sprint's).
- [x] `tests/playfield/` is the rename of the old `field/` category (per the
      standing "playfield not floor" terminology,
      `.clasi/knowledge/playfield-not-floor.md`), carrying over
      `plot_square.py` and `world_goto_chart.py` **verbatim**, each with an
      added header comment: "parked — needs new-tree motion/odometry;
      reactivates once square runs, G/goto, OTOS, and camera sync return in
      a later sprint." Do not attempt to make these runnable against the new
      tree this ticket.
- [x] `tests/bench/velocity_chart.py` is **reinvigorated**, not carried over
      verbatim: rewire its data source from the old motion-command telemetry
      path to the `DEV` protocol — drive via `DEV DT VW` / `DEV M <n> VEL`,
      sample `DEV M <n> STATE` for velocity + applied duty. Preserve the
      existing dashboard's panel layout and interactive key bindings
      (documented in the file's own docstring) as much as is compatible with
      the new data source; note any panel that no longer applies (e.g., if
      an old panel depended on a wire field the DEV protocol doesn't emit)
      and either adapt or remove it, documenting which.
- [x] `tests/bench/dev_exercise.py` (new): scripts the bench Verification
      sequence from the issue (per-motor DUTY/VEL/VOLT/RESET, `DEV DT VW`
      with hand-drag expectation, watchdog silence) over
      `NezhaProtocol.send()` (`host/robot_radio/robot/protocol.py`), runnable
      against both direct serial and the relay's `!GO` data plane (see
      `.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md` for the
      relay handshake: open with DTR asserted, send `!GO`, then plain
      commands with no `>` prefix).
- [x] `tests/bench/pid_hold_speed.py` (new): motor 3 holds a `VEL` target
      while motor 4 steps through load duties (assist → freewheel → drag →
      reverse) on the coupled rig. Script computes/reports whether motor-3
      measured velocity stays inside a tolerance band and recovers within a
      bounded settle time after each load step, with applied duty visibly
      rising as load increases.
- [x] `tests/bench/ratio_governor_curve.py` (new): `DEV DT PORTS 3 4`, then
      commands an unequal-wheel-target curve; script reports whether the
      governor lowers BOTH targets so the measured wheel-speed ratio holds
      the commanded ratio within tolerance, and supports a governor-off run
      (`sync_gain=0`) for the drift control comparison.
- [x] `testgui/` (whole category) is dropped — not carried into the new
      tree in any form.
- [x] The empty `calibrate/` shell and the old `_infra` sim shims are left
      behind in `tests_old/` — not carried over, not fixed.
- [x] `tests/CLAUDE.md` is rewritten (not copied) to describe the new
      three-domain structure — what lives in `sim/`/`bench/`/`playfield/`,
      why they are never combined (different machines/rigs), and a pointer
      to `.claude/rules/` for coding conventions.
- [x] `pyproject.toml`'s `[tool.pytest.ini_options]` is repointed:
      `testpaths` includes the new tree's collectible tiers (per this
      sprint's scope, likely `tests/sim` — mirroring the old default of
      collecting the "always-run" tier; adjust if the new `sim/` skeleton
      has no collectible tests yet, in which case document what `uv run
      python -m pytest` collects with zero tests as the expected state until
      a later sprint populates `tests/sim/`); `norecursedirs` (or
      equivalent exclusion) adds `tests_old` and `source_old` explicitly.
- [x] `uv run python -m pytest` (or the project's documented pytest
      invocation) succeeds and collects zero tests from `tests_old/` or
      `source_old/`.

## Testing

- **Existing tests to run**: `uv run python -m pytest` against the new tree
  — expected to collect whatever `tests/sim/` contains (likely nothing
  populated yet; a clean, zero-test, zero-error collection is the pass
  condition for this ticket, not a nonempty pass count).
- **New tests to write**: The bench scripts listed above are Python tools,
  not pytest-collected tests (they are `tests/bench/`, opt-in, HITL-driven).
  No pytest-collected unit tests are required by this ticket unless the
  programmer finds it cheap to add coverage for shared logic (e.g., a
  `tests/unit/` test for any new host-side parsing helper introduced by
  `dev_exercise.py`) — optional, not blocking.
- **Verification command**: `uv run python -m pytest` (collects the new tree
  only, zero errors). Bench scripts are exercised for real in ticket 7 and
  cannot be verified without hardware — this ticket's gate is that they
  exist, are syntactically correct (`python -m py_compile` or an import
  smoke check), and are wired to the `DEV` protocol per the descriptions
  above.

## Implementation Notes (for ticket 7)

- `tests/sim/` collects one placeholder test (`test_placeholder.py`) rather
  than zero — chosen so `uv run python -m pytest` proves clean collection
  with a pass, not just an easy-to-overlook "0 tests collected". Delete it
  once a real sim harness lands.
- `velocity_chart.py` drives via `DEV DT WHEELS <left> <right>` (not `DEV DT
  VW`) — this tool steers per-wheel ratio directly, and `WHEELS` sets that
  without going through body-twist kinematics; it is the same ratio-governed
  code path as `VW`. Two old panels (colour sensor, line sensor) and one
  (OTOS odometry) were removed/replaced — DEV only implements the Motor
  faceplate this sprint, so there is no colour/line/OTOS data on the wire.
  Replacements: an applied-duty panel and a position-delta-since-connect
  panel (see the file's docstring for the full mapping). `--selftest` is the
  no-hardware-testable surface (ratio math + reply parsing).
- **Gap found, not fixed here (in scope for ticket 7 if needed)**:
  `DrivetrainConfig.sync_gain` defaults to `0.0` in `source/main.cpp` and
  there is no `DEV DT CFG` verb to set it live — the ratio governor is
  therefore OFF by default on today's firmware. `ratio_governor_curve.py`'s
  `--sync-gain` flag is a **label only** (documented in its own docstring);
  it sends no wire command. If ticket 7's bench pass finds the governor
  never engages, that's the firmware gap to fix (either a nonzero boot
  default or a live setter) — flagged here per that ticket's "fix a defect
  found here" instruction.
- `world_goto_chart.py` (parked, `tests/playfield/`) imports a sibling
  `bench_safety.BenchRun` helper that was not carried over (not in this
  ticket's locked file list) — it will fail to import until that dependency
  exists too; this does not block this ticket (the script is explicitly not
  required to run against the new tree yet).
