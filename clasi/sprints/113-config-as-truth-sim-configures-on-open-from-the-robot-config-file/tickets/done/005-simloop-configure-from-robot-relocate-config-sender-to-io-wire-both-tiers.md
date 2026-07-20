---
id: '005'
title: 'SimLoop.configure_from_robot(): relocate config-sender to io/, wire both tiers'
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
depends-on:
- '002'
- '003'
- '004'
github-issue: ''
issue: config-as-truth-sim-configure-on-open.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# SimLoop.configure_from_robot(): relocate config-sender to io/, wire both tiers

## Description

This is the ticket that actually makes "the sim configures on open" true for
a **headless** caller, not just the TestGUI. `SimLoop`
(`src/host/robot_radio/io/sim_loop.py`) is deliberately layered *below*
`testgui/` — its own module docstring says it "implements
`planner/executor.py`'s `TwistTransport` structural protocol DIRECTLY... A
`SimLoop` instance can be handed anywhere a `NezhaProtocol` currently is,
with no adapter" — but today the ONLY thing that can push config into a
running sim is `_SimConfigConn`, which lives inside `testgui/transport.py`
and has zero Qt/GUI dependency of its own (it's a thin `NezhaProtocol.
config()`-compatible sender wrapping `SimLoop.inject_command()`). Its current
home is an accident of "TestGUI needed it first," not a real layering
requirement.

**Relocate `_SimConfigConn`** (or a lightly-renamed equivalent, e.g.
`SimConfigConn`) from `testgui/transport.py` to `io/sim_loop.py` (or a new
sibling module `io/sim_config.py` if that keeps `sim_loop.py` more focused —
implementer's call). `testgui/transport.py` then imports it from its new home
instead of defining its own copy. This is Design Rationale Decision 3 in
`sprint.md`: (a) duplicating it in both places was rejected (exactly the
"two copies of the same knowledge" bug class this sprint exists to close);
(b) importing `testgui.transport._SimConfigConn` from `io/sim_loop.py` was
rejected as a layering inversion (`io/` is lower-level than `testgui/`).

Add `SimLoop.configure_from_robot(config)` — **one method, both tiers**:

- **Tier 1**: build a `NezhaProtocol(SimConfigConn(self))` internally (or
  reuse one constructed once and cached), call `.set_config(**calibration_kwargs(config))`
  (ticket 003's extracted kwargs function) — reusing the exact envelope-
  building/ack-correlation code hardware transports use, per the sprint's
  "one mechanism, not a Sim-specific fork" precedent (109-002 Architecture
  Revision 1). Do NOT reimplement Tier-1 field selection here — import and
  call `calibration_kwargs()`.
- **Tier 2**: call `planner_boot_config_for(config)`/`motor_boot_config_for(config,
  port)` (ticket 004) and pass the results as keyword-unpacked arguments to
  the new `sim_configure_planner`/`sim_configure_motor` ctypes bindings
  (ticket 002) — bind these two new C ABI exports in `_bind_ctypes()`
  alongside the existing 19-symbol transcription (update `sim_ctypes.cpp`'s
  own header-comment export count if it enumerates a specific number).

`configure_from_robot()` must have **zero import-time or call-time dependency
on `testgui/`** — this is what makes a headless caller (pytest fixture,
diagnostic script) able to use it without pulling in Qt at all (SUC-002's
explicit acceptance criterion).

## Acceptance Criteria

- [x] `_SimConfigConn` (or equivalent) is relocated to `io/` (either
      `sim_loop.py` or a new `io/sim_config.py`); `testgui/transport.py`
      imports it from the new location — no duplicate class definition
      exists anywhere in the tree after this ticket.
- [x] `SimLoop.configure_from_robot(self, config: RobotConfig) -> None` is
      added, requiring an active connection (`self._require_connected()`,
      matching every other `SimLoop` method's precondition style). Pushes
      Tier 1 via `NezhaProtocol(...).set_config(**calibration_kwargs(config))`
      and Tier 2 via the two new ctypes calls, in that order (Tier 1 first —
      it's the smaller, already-proven mechanism; Tier 2 second, since
      nothing depends on ordering between the two but this keeps the method
      readable top-to-bottom).
- [x] `_bind_ctypes()` gains `argtypes`/`restype` bindings for
      `sim_configure_planner`/`sim_configure_motor` (ticket 002's exports),
      following the file's existing exhaustive-transcription convention —
      no `ctypes.c_int` default-assumption bugs (the file's own header
      comment explains why this matters: silent float/pointer corruption on
      64-bit platforms).
- [x] `configure_from_robot()` has no top-level or function-local `import`
      of anything under `robot_radio.testgui` — verified by grep, not just
      by inspection (a stray import would silently reintroduce the
      dependency this ticket exists to remove).
- [x] A minimal manual/smoke check: constructing a bare `SimLoop` (no
      `SimTransport`, no Qt, e.g. `QT_QPA_PLATFORM` unset) and calling
      `configure_from_robot()` succeeds — proves SUC-002's "no dependency on
      TestGUI/Qt code path" criterion concretely, not just by import-grep.

## Testing

- **Existing tests to run**: `src/tests/testgui/test_calibration_push_on_connect.py`,
  `test_tour_closure_gate.py`, `test_otos_calibration_convergence.py`,
  `test_turn_error_characterization.py` — all four are `_SimConfigConn`
  consumers (verified during sprint planning via
  `grep -rn _SimConfigConn src/tests/`); each must import the relocated
  class successfully and pass unchanged after the move.
- **New tests to write**: a headless test (new file, e.g.
  `src/tests/sim/system/test_sim_configure_from_robot.py`) that constructs a
  bare `SimLoop` (`start_tick_thread=False` for deterministic stepping, per
  the module's own documented ticket-009 pattern), calls
  `configure_from_robot()` with a `tovez_nocal.json`-loaded `RobotConfig`,
  and confirms no exception and no `testgui` import occurred (this test file
  itself must not import `robot_radio.testgui` anywhere, which is itself
  part of the proof).
- **Verification command**:
  `uv run python -m pytest src/tests/testgui/test_calibration_push_on_connect.py src/tests/testgui/test_tour_closure_gate.py src/tests/testgui/test_otos_calibration_convergence.py src/tests/testgui/test_turn_error_characterization.py -v`,
  then the full suite.

## Files to touch

- `src/host/robot_radio/io/sim_loop.py` (relocated `SimConfigConn` class,
  `configure_from_robot()`, `_bind_ctypes()` additions)
- Possibly new: `src/host/robot_radio/io/sim_config.py` (if the sender is
  split out rather than kept inline in `sim_loop.py`)
- `src/host/robot_radio/testgui/transport.py` (import site update — delete
  the local `_SimConfigConn` class definition, import from its new home;
  `SimTransport`'s own use of it is otherwise unaffected by this ticket —
  see ticket 006 for `connect()`-time wiring)
- New: `src/tests/sim/system/test_sim_configure_from_robot.py`

## Depends On

- Ticket 002 (needs `sim_configure_planner`/`sim_configure_motor` ctypes
  exports to bind against).
- Ticket 003 (needs `calibration_kwargs()`).
- Ticket 004 (needs `planner_boot_config_for()`/`motor_boot_config_for()`).
