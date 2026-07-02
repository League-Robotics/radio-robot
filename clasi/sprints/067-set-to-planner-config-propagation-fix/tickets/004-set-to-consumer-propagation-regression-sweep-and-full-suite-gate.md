---
id: '004'
title: SET-to-consumer propagation regression sweep and full-suite gate
status: open
use-cases:
- SUC-003
depends-on:
- '001'
- '002'
- '003'
github-issue: ''
issue: set-config-not-propagated-to-planner.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# SET-to-consumer propagation regression sweep and full-suite gate

## Description

Tickets 001-003 fix the three distinct propagation bugs this sprint's audit
found (Planner's owned-value `_cfg`, Drive's `_drvCfg` shadow for
`tw`/`lag.otos`, and the missing EKF noise-update path for `ekfRHead`).
This ticket adds the recurrence guard the issue's own acceptance criteria
demand: a table-driven sweep test that `SET`s each motion-critical key this
sprint's audit identified as STALE and asserts the owning consumer observes
the new value, so a future regression that reintroduces a stale config copy
anywhere in this key set fails a specific, named test — not a test that
happens to pass regardless of underlying behavior (which is exactly what
`test_rt_slip.py` was doing before Ticket 001 fixed it).

The audit's STALE rows (from `architecture-update.md`'s full audit table)
that this sweep must cover: `rotSlip`, `tw`, `vWheelMax`, `rotGainPos`,
`rotGainNeg`, `turnGate`, `ctrlPeriod`, and `ekfRHead`.

Each measurement must be isolated — a fresh `Sim()` instance or a
successful (reply-checked) `ZERO enc`, never a bare `ZERO` — per the exact
false-positive pattern Ticket 001 found and fixed in `test_rt_slip.py`
(`parseZero()` in `source/commands/SystemCommands.cpp` rejects a bare
`ZERO` with `ERR badarg`; an unchecked reply lets encoder state accumulate
across sequential measurements within one test, faking a slip/behavior
change that isn't real).

This ticket also serves as the sprint's final acceptance gate: the full
default pytest suite must be green (baseline 2506 passed / 0 failed, per
`architecture-update.md`'s Step 4-5 item 4) after all three fix tickets and
this sweep test are in place.

See `architecture-update.md` Step 4-5 item 4 and `usecases.md` SUC-003 for
the full design and acceptance criteria this ticket implements.

## Acceptance Criteria

- [ ] New sim test file (e.g.
      `tests/simulation/unit/test_set_config_propagation_sweep.py` or
      similar, per project test-naming convention) with a table-driven
      case for each of: `rotSlip`, `tw`, `vWheelMax`, `rotGainPos`,
      `rotGainNeg`, `turnGate`, `ctrlPeriod`, `ekfRHead`.
- [ ] Each case: `SET`s the key to a value distinct from its boot default,
      exercises the one sim-observable behavior that depends on it (e.g.
      RT arc for `rotSlip`/`tw`/`rotGainPos`/`rotGainNeg`/`turnGate`; the
      EKF-predict trackwidth/lag compensation for `tw`/`lag.otos`-adjacent
      behavior already covered by Ticket 002; the OTOS heading-correction
      weighting for `ekfRHead`), and asserts the observed behavior differs
      from the boot-default behavior.
- [ ] Every measurement is isolated: fresh `Sim()` per value under test, OR
      a `ZERO enc` (never bare `ZERO`) with the reply checked
      (`assert "OK" in reply`) between measurements within the same test.
- [ ] `tests/simulation/unit/test_rt_slip.py`'s three existing tests are
      confirmed to still pass for the right reason (already fixed in
      Ticket 001; this ticket only re-verifies as part of the full-suite
      gate, no further changes expected here).
- [ ] Full default pytest suite green: `uv run python -m pytest` reports
      2506 passed (baseline) plus this sprint's new tests, 0 failed. This
      is the sprint's closing acceptance gate.

## Testing

- **Existing tests to run**: full default suite via
  `uv run python -m pytest` (this IS the ticket's primary deliverable —
  confirming the baseline plus new tests are green).
- **New tests to write**: the table-driven sweep test described above,
  covering all eight STALE keys from the audit.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Write one new table-driven (or explicitly enumerated, if the
project's sim test style prefers explicit functions over
`@pytest.mark.parametrize`) test module that exercises each STALE key from
the audit table in `architecture-update.md`, using the isolation pattern
Ticket 001 established for `test_rt_slip.py` (fresh `Sim()` or
reply-checked `ZERO enc`). This ticket depends on 001, 002, and 003 because
it tests behavior those tickets introduce — it cannot pass (and should not
be started) before all three land.

**Files to create**:
- A new test file under `tests/simulation/unit/` covering the sweep (name
  per existing test-file conventions in that directory — check sibling
  files like `test_rt_slip.py`, `test_059_config_routing.py` for the
  established pattern before naming).

**Files to modify**: none expected beyond the new test file — this ticket
should not require further source changes if Tickets 001-003 are complete
and correct. If the sweep surfaces a gap Tickets 001-003 missed, treat that
as a signal to revisit those tickets rather than papering over it in the
test.

**Testing plan**:
- Implement the sweep test per the acceptance criteria above.
- Run the full default suite (`uv run python -m pytest`) and confirm the
  final green count matches or exceeds the 2506-passed / 0-failed baseline
  recorded in `architecture-update.md`.
- This is the sprint's closing gate — do not mark this ticket (or the
  sprint) done until the full suite is confirmed green.

**Documentation updates**: none — `architecture-update.md` already
documents this change in full (Step 4-5 item 4, `usecases.md` SUC-003). No
wire-protocol change, no `RobotConfig` schema change.
