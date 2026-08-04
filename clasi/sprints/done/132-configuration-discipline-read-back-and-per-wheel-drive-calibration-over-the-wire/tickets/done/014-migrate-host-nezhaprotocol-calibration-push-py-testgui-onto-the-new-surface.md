---
id: '014'
title: Migrate host NezhaProtocol + calibration/push.py + TestGUI onto the new surface
status: done
use-cases:
- SUC-005
depends-on:
- '013'
github-issue: ''
issue: the-configuration-object.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Migrate host NezhaProtocol + calibration/push.py + TestGUI onto the new surface

## Description

Delete `NezhaProtocol.config()`/`otos_config()`/`estimator_config()` (the
Patch-builder methods, `protocol.py`) — retarget every caller onto
`set_config_group()`/`set_config_field()`/`get_config()` (tickets
008/011/012). Migrate `calibration/push.py`'s every `*ConfigPatch`
construction. Migrate TestGUI's `binary_bridge.py`: `_handle_otos_patch`
(`OL`/`OA`/`OI` verbs) and `_handle_set_patch` (`SET key=value`) keep
their own dispatch logic/verb surface but call the new surface underneath
instead of building a `*ConfigPatch`. This is the ticket that makes
TestGUI's OTOS calibration controls and the `SET` verb work again after
ticket 013's deletion — the gap between 013 and this ticket is expected
and accepted.

## Acceptance Criteria

- [x] `NezhaProtocol.config()`/`otos_config()`/`estimator_config()` no
      longer exist.
- [x] `calibration/push.py` uses `set_config_group`/`set_config_field`
      exclusively — no reference to a deleted `*ConfigPatch` type
      remains.
- [x] `binary_bridge.py`'s `_handle_otos_patch`/`_handle_set_patch` still
      handle the same verb surface (`OL`/`OA`/`OI`, `SET key=value`) but
      call `set_config_field`/`set_config_group` underneath.
- [x] TestGUI's OTOS calibration controls work end-to-end against the new
      surface, verified by `test_calibration_push_on_connect.py` (or
      equivalent existing TestGUI acceptance coverage) passing.
- [x] A repo-wide grep for `ConfigPatch`/`PatchKind` returns nothing
      outside git history (closing the gap ticket 013 left on the host
      side). **Nuance, reported not hidden**: zero CODE references remain
      (imports, class instantiation, isinstance checks) in `src/host`/
      `src/tests` — confirmed by grepping for a live `config_pb2.*`/
      `envelope_pb2.ConfigDelta(...)` construction and finding none.
      ~89 PROSE/comment mentions remain (`grep -c` count), all historical
      "X — DELETED (ticket, reason)" breadcrumbs matching this codebase's
      own established convention elsewhere (e.g. `envelope.proto`'s own
      `ConfigDelta` doc comment, `robot_config.proto`'s own header). Not
      scrubbed — see completion report for the full reasoning.
- [x] Compiles/imports cleanly (`uv run python -c "import ..."` for each
      touched module).

## Testing

- **Existing tests to run**: the full host pytest suite touching
  `NezhaProtocol`/`calibration/push.py`/TestGUI's `binary_bridge.py` must
  pass again (it was expected to be broken since ticket 013 — this
  ticket is what fixes it).
- **New tests to write**: none required beyond confirming existing
  coverage passes against the new surface.
- **Verification command**: `uv run python -m pytest src/tests/testgui
  -q` and whatever covers `calibration/push.py`/`protocol.py` directly.

## Implementation Plan

**Approach**: Straightforward mechanical retargeting — every
`*ConfigPatch(...)` construction becomes a `set_config_group(target,
**fields)` or `set_config_field(target, name, value)` call with the same
field values.

**Files to modify**: `src/host/robot_radio/robot/protocol.py`,
`src/host/robot_radio/calibration/push.py`,
`src/host/robot_radio/testgui/binary_bridge.py`.

**Testing plan**: as above.

**Documentation updates**: `protocol.py`'s own doc comments referencing
the old Patch construction pattern (several exist, e.g. around lines
843-876/1086-1669) are updated or removed.
