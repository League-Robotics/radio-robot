---
id: "003"
title: "TestGUI default-profile reconciliation"
status: open
use-cases:
- SUC-004
depends-on:
- "002"
github-issue: ""
issue: sim-turn-undershoot.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# TestGUI default-profile reconciliation

## Description

`sim_prefs.DEFAULT_PROFILE` (`host/robot_radio/testgui/sim_prefs.py:112-137`)
hardcodes `slip_turn_extra: 0.26` and `body_rot_scrub: 1.0` (neutral). Even
after Tickets 001+002 fix the underlying sim/firmware defects, a TestGUI
operator on a fresh install (no persisted `sim_error_profile.json`) still
gets a turn-inaccurate experience out of the box: the encoder-report-only
`slip_turn_extra=0.26` combined with a neutral `body_rot_scrub=1.0` default
under-rotates turns net ~14%, and reconciling `body_rot_scrub` against the
active robot's real calibration requires manually discovering and clicking
the "From Calibration" button (070-004) — `__main__.py::_on_sim_errors_from_cal()`
(lines 824-892) ALREADY implements the exact reconciliation this ticket
wants (`sim_err_body_rot_scrub.setValue(rot_slip)`,
`sim_err_slip_turn.setValue(0.0)`), just only on manual click, with its
lookup/fallback logic duplicated inline rather than factored out.

This ticket factors that button's lookup into a shared, Qt-free resolver
in `sim_prefs.py`, and wires it into `DEFAULT_PROFILE`'s fallback path so
the reconciled calibration is the factory default, not a manual,
undiscoverable opt-in. Depends on Ticket 002: this ticket propagates
Ticket 002's now-correct sim-level default (plant genuinely scrubs by
`rotationalSlip`) into the TestGUI's own factory default — it should
follow, not precede, Ticket 002 conceptually, even though there is no
compile-time dependency between the files.

See `architecture-update.md` Step 1 (confirms `_on_sim_errors_from_cal()`
already implements this reconciliation manually), Step 3 (`sim_prefs.py`/
`__main__.py` module boundaries), Step 5 "Ticket 003", Design Rationale
Decision 4 (shared resolver, not a duplicated lookup); `usecases.md`
SUC-004.

## Acceptance Criteria

- [ ] `host/robot_radio/testgui/sim_prefs.py` gains a new function (e.g.
      `resolve_calibration_defaults() -> tuple[float, float]`, returning
      `(body_rot_scrub, trackwidth_mm)`) that mirrors EXACTLY the lookup
      `_on_sim_errors_from_cal()` performs today: `get_robot_config()` →
      `cfg.calibration.rotational_slip` / `cfg.geometry.trackwidth`, with
      the same WARN-and-neutral-fallback semantics for a missing config or
      missing field.
- [ ] `sim_prefs.py` remains Qt-free (its own module docstring's existing
      constraint) — the new dependency on
      `robot_radio.config.robot_config.get_robot_config()` is a downward
      dependency on a lower-level, Qt-free config module, introducing no
      import cycle.
- [ ] `DEFAULT_PROFILE["slip_turn_extra"]` changes from `0.26` to `0.0`.
- [ ] `load_sim_error_profile()`'s fallback path (no persisted file, or a
      persisted file missing the `body_rot_scrub` key) calls the new
      resolver for `body_rot_scrub` instead of using the literal `1.0`
      default.
- [ ] `__main__.py::_on_sim_errors_from_cal()` is refactored to call
      `sim_prefs.resolve_calibration_defaults()` instead of its own
      inline `get_robot_config()`/fallback logic. Its OBSERVABLE behavior
      (values set, log messages, fallback semantics) is byte-identical
      before and after — confirmed by the ticket's own before/after test
      run of `test_070_004_sim_errors_from_cal.py`.
- [ ] `load_sim_error_profile()` with no persisted file returns
      `body_rot_scrub` matching the active robot's `rotational_slip` (or
      neutral `1.0` with a logged fallback if no active robot config is
      found) and `slip_turn_extra == 0.0`.
- [ ] `tests/testgui/test_sim_prefs.py`, `test_transport.py`, and
      `test_070_004_sim_errors_from_cal.py` are updated for the new
      `DEFAULT_PROFILE` values — deliberately, with the before (`0.26`/
      `1.0` hardcoded) and after (`0.0`/calibration-resolved) values
      documented in the ticket's implementation notes.
- [ ] An operator with an EXISTING persisted
      `data/testgui/sim_error_profile.json` is unaffected until they
      delete it or reset fields and re-Apply — this is a documented
      migration note (Open Questions item 4), not a code change; do not
      write logic that rewrites or migrates existing persisted files.
- [ ] Full suite (`uv run python -m pytest`) passes at 2655 + this
      ticket's net new/changed test count, zero unexplained failures.

## Testing

- **Existing tests to run**: `tests/testgui/test_sim_prefs.py`,
  `test_transport.py`, `test_070_004_sim_errors_from_cal.py` (all three,
  before AND after the change, to document the exact before/after
  values), full suite.
- **New tests to write**: a unit test for
  `resolve_calibration_defaults()` covering both the found-config and
  missing-config/fallback paths; a test confirming
  `load_sim_error_profile()`'s fallback now resolves `body_rot_scrub` from
  calibration rather than a hardcoded `1.0` when no persisted file exists.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: In `host/robot_radio/testgui/__main__.py`, read
`_on_sim_errors_from_cal()` (lines 824-892) in full to capture its EXACT
current lookup/fallback/log-message behavior. In
`host/robot_radio/testgui/sim_prefs.py`, add a new function replicating
that logic verbatim (same `get_robot_config()` call, same field paths,
same fallback values, same log messages), returning the tuple of resolved
values. Change `DEFAULT_PROFILE["slip_turn_extra"]` to `0.0`. Update
`load_sim_error_profile()`'s fallback-construction path to call the new
resolver for `body_rot_scrub` instead of hardcoding `1.0`. Then go back to
`__main__.py` and replace `_on_sim_errors_from_cal()`'s inline logic with
a call to the new `sim_prefs` function, deleting the now-duplicated code.
Run `test_070_004_sim_errors_from_cal.py` before and after this refactor
to confirm byte-identical observable behavior. Update the three named
test files' hardcoded `0.26`/`1.0` expectations to the new values,
documenting before/after in this ticket's Implementation Notes.

**Files to create/modify**:
- `host/robot_radio/testgui/sim_prefs.py` — new
  `resolve_calibration_defaults()` function; `DEFAULT_PROFILE`'s
  `slip_turn_extra` constant; `load_sim_error_profile()`'s fallback path.
- `host/robot_radio/testgui/__main__.py` — `_on_sim_errors_from_cal()`
  refactored to call the new shared resolver.
- `tests/testgui/test_sim_prefs.py` — updated for new `DEFAULT_PROFILE`
  values; new resolver unit tests.
- `tests/testgui/test_transport.py` — updated for new default values
  where exercised.
- `tests/testgui/test_070_004_sim_errors_from_cal.py` — updated/confirmed
  for byte-identical `_on_sim_errors_from_cal()` behavior post-refactor.

**Testing plan**: run the three named TestGUI test files before making
any change (capture baseline pass/fail and asserted values), make the
`sim_prefs.py`/`__main__.py` changes, re-run the same three files and
diff the asserted values against the documented before/after, then full
suite.

**Documentation updates**: none beyond inline docstrings for the new
`resolve_calibration_defaults()` function; no wire-protocol change (this
is TestGUI-local, host-side only).
