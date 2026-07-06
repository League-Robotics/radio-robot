---
id: '002'
title: Map keyboard driving to DEV DT VW/STOP/PORTS and fix Operations STOP
status: done
use-cases:
- SUC-002
- SUC-005
depends-on:
- '001'
github-issue: ''
issue: host-testgui-sim-cockpit.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Map keyboard driving to DEV DT VW/STOP/PORTS and fix Operations STOP

## Description

`host/robot_radio/testgui/drive.py`'s `KeyboardDriver` sends a top-level
`VW <v> <omega_mrads>` and a bare `STOP` — neither verb is registered by the
current firmware (confirmed by reading every `makeCmd`/`makeSchemaCmd` in
`source/commands/*.cpp`: only `DEV DT VW`/`DEV DT STOP` exist for driving).
`host/robot_radio/testgui/operations.py`'s Operations panel STOP button has
the identical defect (`transport.command("STOP", ...)`).

This ticket rewrites the wire strings `KeyboardDriver` builds to
`DEV DT VW <v_x> 0 <omega>` (rad/s) and `DEV DT STOP`, adds a one-time
`DEV DT PORTS <left> <right>` bind per drive session, and fixes
`operations.py`'s STOP handler to send `DEV DT STOP`. See
`architecture-update.md` Design Rationale Decisions 2 (why `KeyboardDriver`,
not `SimTransport`, owns the `DEV DT PORTS` bind) and 3 (why the
`operations.py` STOP fix is in-scope here even though the issue's file list
does not name that file — same bug, safety-relevant, one line).

## Acceptance Criteria

- [x] `drive.py`'s `ROTATE_OMEGA_MRADS` module constant is renamed to drop
      the embedded unit (`naming-and-style.md` rule 1) and its value changes
      from milli-rad/s to rad/s (e.g. `ROTATE_OMEGA: float = 0.5  # [rad/s]`
      replacing `ROTATE_OMEGA_MRADS: int = 500`). All references to the old
      name in this module and its tests are updated in the same commit.
- [x] `_qt_arrow_keys()` builds `"DEV DT VW {FWD_SPEED} 0 0"` (Up),
      `"DEV DT VW -{FWD_SPEED} 0 0"` (Down), `"DEV DT VW 0 0 {ROTATE_OMEGA}"`
      (Left/CCW), `"DEV DT VW 0 0 -{ROTATE_OMEGA}"` (Right/CW) — matching
      `docs/protocol-v2.md`'s `DEV DT VW <v_x> <v_y> <omega>` (mm/s, mm/s,
      rad/s) signature.
- [x] `vw_line_for_key`/`vw_line_for_key_set` remain pure, Qt-free functions
      importable and testable without a `QApplication` (unchanged contract,
      only the returned strings change).
- [x] `KeyboardDriver.attach()` sends `DEV DT PORTS <left> <right>` exactly
      once per attach (before any `DEV DT VW`), using a module-level default
      port pair `(1, 2)` matching the firmware boot default and the sim's
      default plant binding (`tests/_infra/sim/firmware.py`'s `vel()`
      docstring: "port 1=LEFT, port 2=RIGHT"). This send happens for every
      `Transport` subclass, not just `SimTransport` (transport-agnostic, per
      Decision 2).
- [x] `_send_cmd()`'s `"STOP"` sentinel value is replaced with `"DEV DT STOP"`
      everywhere it is sent (the deadman resend sequence itself —
      `STOP_RESEND_COUNT`, the resend timer, the focus-loss handling — is
      unchanged).
- [x] `operations.py`'s `OpsController.on_stop()` sends `"DEV DT STOP"`
      instead of `"STOP"`. The rest of `on_stop()` (cancel worker threads
      first, then `STREAM 0`) is unchanged.
- [x] Module docstring's "Units" section in `drive.py` is updated to state
      `omega` is rad/s (matching the renamed constant), not milli-rad/s.

## Testing

- **Existing tests to run**: none yet in `tests/` (ported in ticket 004);
  manually verify by running the GUI against a connected `SimTransport`
  (ticket 001) and confirming arrow keys move the sim avatar and STOP halts
  it.
- **New tests to write**: pure unit tests for `vw_line_for_key`/
  `vw_line_for_key_set` asserting the exact `DEV DT VW ...` strings for each
  arrow key and combination; a test that `KeyboardDriver.attach()` sends the
  `DEV DT PORTS` bind exactly once (using a fake `Transport` recording sent
  lines) even across multiple key presses within one session; a test that
  the STOP deadman sequence sends `DEV DT STOP` `STOP_RESEND_COUNT` times on
  release and on focus-loss; a test that `operations.py`'s STOP button sends
  `DEV DT STOP` via a fake transport.
- **Verification command**: `QT_QPA_PLATFORM=offscreen uv run pytest tests/testgui -k "drive or operations"` (once ticket 004 creates the directory; until then, run the new test files directly with `uv run pytest <path>`).
