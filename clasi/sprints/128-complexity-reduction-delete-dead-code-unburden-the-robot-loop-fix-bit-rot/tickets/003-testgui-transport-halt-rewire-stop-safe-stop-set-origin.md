---
id: '003'
title: testgui Transport.halt() + rewire STOP/_safe_stop/_set_origin
status: open
use-cases: [SUC-005]
depends-on: []
github-issue: ''
issue: testgui-stop-paths-must-halt-through-the-transport-not-the-dead-bridge.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# testgui Transport.halt() + rewire STOP/_safe_stop/_set_origin

**Worktree note**: implement in the git worktree checked out for
`sprint/128-complexity-reduction-delete-dead-code-unburden-the-robot-loop-fix-bit-rot`.
All paths below are relative to that worktree.

**Sequencing note (safety-critical)**: this ticket must land BEFORE
ticket 004 (deleting `binary_bridge.py`'s dead half). The GUI's STOP
currently routes through that bridge; deleting the bridge first, before
`Transport.halt()` exists as the replacement path, would leave a window
with no working halt path on hardware at all.

## Description

`testgui/operations.py:557-614` (`on_stop`) sends the string `"STOP"`
through `transport.command()`, which on `SerialTransport`/`RelayTransport`
routes into `binary_bridge.translate_command()` — an unconditional dead
stub that returns `ERR unavailable ...` without touching the wire.
`on_stop()` discards the reply and logs `"[INFO] STOP sent"` while the
robot keeps driving. `__main__.py:2117-2122` (`_safe_stop`) and `:2599`
(`_set_origin`) have the same defect. Even if the stub worked, `"STOP"`
maps to the PLANNED stop, not the halt-now `estop()`.

## Acceptance Criteria

- [ ] `Transport` ABC (`testgui/transport.py`) gains an abstract
      `halt() -> None` method, documented as estop semantics (clears the
      active Move AND the planner queue) and as raising on failure —
      callers must surface a failed halt, never log success on faith.
- [ ] `_HardwareTransport.halt()` calls `self.protocol.estop()`
      (raising `ConnectionError` if not connected).
- [ ] `SimTransport.halt()` calls `self.protocol.stop()` (`SimLoop.stop()`
      already sends `ESTOP` on the sim ABI).
- [ ] `on_stop()` step 2, `_safe_stop()`, and `_set_origin()`'s
      pre-teleport halt are rewired onto `transport.halt()`, logging
      `[INFO] estop sent -- motion halted` on success and
      `[ERROR] HALT FAILED -- ROBOT MAY STILL BE MOVING: <exc>` on a
      raise — never logging success on faith.
- [ ] `on_stop()` step 3 (`STREAM 0`) takes the same direct-call
      treatment (`transport.protocol.tlmOff()` on hardware) or logs an
      honest "not available" — full removal of the STREAM verb's
      dead-bridge path is ticket 004's job; this ticket only needs
      `on_stop()` itself not to depend on the dead translation layer for
      its own halt step.
- [ ] A hardware-transport-shaped test (mocked `NezhaProtocol`) added to
      `test_gui_button_acceptance.py`'s harness asserts: the STOP button
      results in exactly one `estop()` call, and a raising `estop()`
      produces an `[ERROR]` log, not `[INFO]`. (The existing suite is
      Sim-only, which is exactly what let this ship undetected — `SimLoop.stop()`
      already halts correctly in Sim, hiding that hardware's STOP does
      nothing.)
- [ ] `grep -rn "command(\"STOP\"\|send(\"STOP\"" src/host/robot_radio/testgui/`
      returns nothing.
- [ ] `grep -rn "estop" src/host/robot_radio/testgui/` shows the ABC,
      both backends, and the rewired call sites.

## Testing

- **Existing tests to run**: `uv run python -m pytest`;
  `test_gui_button_acceptance.py` (must still pass on Sim after the
  rewire).
- **New tests to write**: the hardware-transport-shaped mock-`NezhaProtocol`
  test named in the acceptance criteria above, added to
  `test_gui_button_acceptance.py`'s own harness (not a separate file —
  per the project's "GUI work needs headless button acceptance" rule).
- **Verification command**: `uv run python -m pytest src/tests -k gui_button -q`,
  then on the bench (robot on stand): click STOP during a Managed `D`
  move — wheels halt within one cycle; disconnect the serial link and
  click STOP — the log shows `HALT FAILED ... MAY STILL BE MOVING`, not
  success.

## Implementation Notes

- **Approach**: add the ABC method and both backend implementations
  first, get the new hardware-shaped test passing against the mock, then
  rewire the three call sites. Do not touch `binary_bridge.py` itself in
  this ticket — that's ticket 004, sequenced after this one specifically
  so the halt path is never unavailable.
- **Files to modify**: `src/host/robot_radio/testgui/transport.py`,
  `src/host/robot_radio/testgui/operations.py`,
  `src/host/robot_radio/testgui/__main__.py`,
  `src/tests/testgui/test_gui_button_acceptance.py` (or wherever its
  harness lives).
- **Documentation updates**: none required this ticket; ticket 004 will
  update `robot_radio/DESIGN.md`'s `testgui/` row once the bridge itself
  is gutted.
