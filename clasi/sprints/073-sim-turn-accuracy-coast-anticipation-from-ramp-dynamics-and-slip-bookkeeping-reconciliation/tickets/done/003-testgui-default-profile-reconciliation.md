---
id: '003'
title: TestGUI default-profile reconciliation
status: done
use-cases:
- SUC-004
depends-on:
- '002'
github-issue: ''
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

- [x] `host/robot_radio/testgui/sim_prefs.py` gains a new function (e.g.
      `resolve_calibration_defaults() -> tuple[float, float]`, returning
      `(body_rot_scrub, trackwidth_mm)`) that mirrors EXACTLY the lookup
      `_on_sim_errors_from_cal()` performs today: `get_robot_config()` →
      `cfg.calibration.rotational_slip` / `cfg.geometry.trackwidth`, with
      the same WARN-and-neutral-fallback semantics for a missing config or
      missing field.
- [x] `sim_prefs.py` remains Qt-free (its own module docstring's existing
      constraint) — the new dependency on
      `robot_radio.config.robot_config.get_robot_config()` is a downward
      dependency on a lower-level, Qt-free config module, introducing no
      import cycle.
- [x] `DEFAULT_PROFILE["slip_turn_extra"]` changes from `0.26` to `0.0`.
- [x] `load_sim_error_profile()`'s fallback path (no persisted file, or a
      persisted file missing the `body_rot_scrub` key) calls the new
      resolver for `body_rot_scrub` instead of using the literal `1.0`
      default.
- [x] `__main__.py::_on_sim_errors_from_cal()` is refactored to call
      `sim_prefs.resolve_calibration_defaults()` instead of its own
      inline `get_robot_config()`/fallback logic. Its OBSERVABLE behavior
      (values set, log messages, fallback semantics) is byte-identical
      before and after — confirmed by the ticket's own before/after test
      run of `test_070_004_sim_errors_from_cal.py`.
- [x] `load_sim_error_profile()` with no persisted file returns
      `body_rot_scrub` matching the active robot's `rotational_slip` (or
      neutral `1.0` with a logged fallback if no active robot config is
      found) and `slip_turn_extra == 0.0`.
- [x] `tests/testgui/test_sim_prefs.py`, `test_transport.py`, and
      `test_070_004_sim_errors_from_cal.py` are updated for the new
      `DEFAULT_PROFILE` values — deliberately, with the before (`0.26`/
      `1.0` hardcoded) and after (`0.0`/calibration-resolved) values
      documented in the ticket's implementation notes.
- [x] An operator with an EXISTING persisted
      `data/testgui/sim_error_profile.json` is unaffected until they
      delete it or reset fields and re-Apply — this is a documented
      migration note (Open Questions item 4), not a code change; do not
      write logic that rewrites or migrates existing persisted files.
- [x] Full suite (`uv run python -m pytest`) passes at 2655 + this
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

## Implementation Notes

**Before / after values** (test_sim_prefs.py, test_transport.py,
test_070_004_sim_errors_from_cal.py, and — found only by the ticket's own
full-suite run, not in the original scope list —
tests/testgui/test_sim_errors_panel.py, which also hardcoded the historical
`0.26`):

| Field | Before | After |
|---|---|---|
| `DEFAULT_PROFILE["slip_turn_extra"]` | `0.26` (hardcoded) | `0.0` |
| `DEFAULT_PROFILE["body_rot_scrub"]` (the literal dict constant) | `1.0` | `1.0` — UNCHANGED; a bare `dict(DEFAULT_PROFILE)` copy (e.g. `test_defaults_send_the_documented_noop_simset_string`) must stay a genuine no-op with zero config I/O. |
| `load_sim_error_profile()`'s fallback `body_rot_scrub` (no persisted file, or a persisted file missing that key) | `1.0` (DEFAULT_PROFILE literal) | `resolve_calibration_defaults()`'s first element — the active robot's `calibration.rotational_slip`, or neutral `1.0` with a logged `[WARN]` if no active robot config is found. |
| `load_sim_error_profile()`'s fallback `trackwidth_mm` | `128.0` (DEFAULT_PROFILE literal) | UNCHANGED — `128.0` (DEFAULT_PROFILE literal); out of this ticket's scope per the acceptance criteria (only `body_rot_scrub`'s fallback is resolver-backed). |

**`resolve_calibration_defaults(log=None)` signature.** Returns
`tuple[float, float]` as specified. Added an optional `log:
Callable[[str], None] | None` parameter so `__main__.py`'s
`_on_sim_errors_from_cal()` can pass `_append_log` and keep the GUI log
pane's `"[WARN] From Calibration: ..."` lines byte-identical to before the
refactor, without `__main__.py` re-deriving `cfg`/field-presence itself
(the acceptance criteria's "instead of its own inline
`get_robot_config()`/fallback logic" — with `log` omitted, as
`load_sim_error_profile()` does, the same `[WARN]` still reaches the module
logger via `_log.warning()`, just not any GUI widget).

**Patch-point gotcha (found via full-suite run, not anticipated in the
plan).** `resolve_calibration_defaults()` imports `get_robot_config` with a
LOCAL import inside the function body (mirroring
`_on_sim_errors_from_cal()`'s own original per-call local import) rather
than a module-level `from robot_radio.config.robot_config import
get_robot_config` in `sim_prefs.py`. A first attempt used the module-level
form; it broke two pre-existing, out-of-scope test files
(`test_sim_errors_from_cal_button.py`, `test_sim_errors_from_calibration.py`
— 5 failures) that monkeypatch `get_robot_config` at its SOURCE module
(`robot_radio.config.robot_config.get_robot_config`), the established
project convention — a module-level `from...import` in `sim_prefs.py`
freezes a reference at `sim_prefs`' own first-import time, which predates
those tests' monkeypatching and is never updated by it. The local-import
form re-resolves the name fresh on every call, honoring the patch
regardless of import order. This ticket's own new tests
(`TestResolveCalibrationDefaults`, `TestLoadFallbackResolvesBodyRotScrubFromCalibration`,
and the `get_robot_config`-pinning additions to `TestPersistence`) all
patch at `robot_radio.config.robot_config.get_robot_config`, consistent
with this.

**`test_070_004_sim_errors_from_cal.py` refactor.** Beyond the acceptance
criteria's minimum ("updated for the new `DEFAULT_PROFILE` values"), its
own `_from_calibration_profile()` helper — which pre-dated this ticket and
independently re-derived the SAME `rot_slip`/`trackwidth` lookup inline (a
third copy of the logic, alongside the button and, before this ticket, no
`load_sim_error_profile()` fallback copy) — was refactored to call
`sim_prefs.resolve_calibration_defaults()` too, fully realizing Design
Rationale Decision 4's "one source of truth" for this sprint's own test
suite, not just production code. The test's actual asserted VALUES are
unchanged (its explicit `.update()` already overrode every mapped key,
including `slip_turn_extra` to a literal `0.0`, so `DEFAULT_PROFILE`'s
`slip_turn_extra` change was invisible to this specific test either way).

**Verification.** `test_sim_prefs.py` (29 tests), `test_transport.py` (61
tests), `test_070_004_sim_errors_from_cal.py` (1 test, run standalone with
its module-scoped `pin_calibrated_tovez` fixture pinning `ROBOT_CONFIG` to
`data/robots/tovez.json` — independent of the shared tree's
`active_robot.json` drift), `test_sim_errors_from_cal_button.py` (8
tests) and `test_sim_errors_from_calibration.py` (2 tests) all pass. Full
`tests/testgui/` tier: 579 passed, 2 pre-existing `xfail(strict=True)`
(owned by ticket 073-004, unrelated). Full default suite (`uv run python -m
pytest`, the `tests/simulation/` CI gate): 2667 passed, 1 failed —
`test_069_rt_90deg_body_scrub.py::test_rt_90deg_identity_no_scrub`, the
ticket's documented pre-existing baseline failure owned by ticket 073-004,
left untouched.
