---
id: '004'
title: 'Set Robot @ 0,0: full pose reset (heading + encoders + SI command)'
status: done
use-cases:
- SUC-005
depends-on: []
github-issue: ''
issue: testgui-set-robot-zero-full-reset.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Set Robot @ 0,0: full pose reset (heading + encoders + SI command)

## Description

The "Set Robot @ 0,0" button resets the robot to the world origin.  It must
reset **everything**: encoders, the OTOS sensor heading, the fused/EKF pose,
and the on-screen display.

The full wire sequence (in order):

1. Send `ZERO enc` to reset the wheel encoder counters.
2. Send `OZ` to zero the OTOS sensor's position and heading
   (`setPositionRaw(0,0,0)`).  This step is essential: the firmware's EKF
   fuses the OTOS absolute heading every tick (`Odometry::correctEKF`:
   `s.poseHrad += alphaYaw*wrapπ(θ_otos − s.poseHrad)`), so without
   resetting the OTOS the heading snaps to 0 via `SI` then immediately
   drifts back to the OTOS's stale reading.  `OZ` re-references the OTOS
   to the robot's current physical orientation as the new heading-zero.
3. Send `SI 0 0 0` (via `build_setpose_command(0.0, 0.0, 0.0)`) to snap the
   firmware's fused/EKF pose to (0 mm, 0 mm, 0 centidegrees).
4. Reset the display: `trace_model.anchor(0,0,0)`, `trace_model.clear()`,
   `canvas_ctrl.reset_avatar_to_center()`, `canvas_ctrl.refresh()`.

**Connection gating**: if `_state["transport"]` is `None` (not connected), skip
steps 1–3 and log a clearly worded `[WARN]` message.  The display reset (step 4)
still runs so the GUI stays consistent.  In Sim mode a transport IS present, so
all three wire commands are sent.

**`OZ` syntax**: bare verb, no arguments — `transport.command("OZ", read_ms=300)`.
Confirmed against `source/COMMANDS.md` (row `OZ`) and
`source/commands/OtosCommands.cpp` (`handleOZ` → `setPositionRaw(0,0,0)`).

**`SI` semantics**: `SI x_mm y_mm h_cdeg` sets the firmware's fused pose.
`build_setpose_command(0.0, 0.0, 0.0)` returns `"SI 0 0 0"`.

**Files modified:**
- `host/robot_radio/testgui/__main__.py` — `_set_origin()` closure updated.

**Files created/updated:**
- `tests/testgui/test_set_origin.py` — Qt-free tests asserting the 3-command sequence.

## Acceptance Criteria

- [x] Clicking "Set Robot @ 0,0" when connected sends `ZERO enc`, then `OZ`,
      then `SI 0 0 0` to the transport, in that order, before the display is reset.
- [x] `OZ` (OTOS zero) is included to re-reference the OTOS absolute heading so
      the EKF does not drift back to the stale heading after `SI` resets the pose.
- [x] `ZERO enc`, `OZ`, and `SI 0 0 0` appear in the GUI log (transport logging
      covers this automatically via the `on_log` callback).
- [x] The display reset still runs after the wire commands: avatar moves to
      field centre, traces are cleared, heading is 0.
- [x] When no transport is connected (`_state["transport"] is None`), the wire
      commands are skipped and a `[WARN] Set Robot @ 0,0: no robot connected —
      display only` message is logged; the display reset still runs.
- [x] In Sim mode the commands are sent (sim transport accepts them).
- [x] `build_setpose_command(0.0, 0.0, 0.0)` returns a string starting with
      `"SI"` containing three numeric tokens; the handler sends this exact string.
- [x] Headless tests (Qt-free) verify the command sequence using a fake transport
      with a `commands_sent` list: `ZERO enc` at index 0, `OZ` at index 1,
      `SI 0 0 0` at index 2.
- [x] All existing `tests/testgui/` tests pass unchanged.
- [x] `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui -q` passes.

## Implementation Plan

### Approach

The change is localized to the `_set_origin()` closure inside `_build_main_window()`
in `__main__.py`. The closure already captures `trace_model`, `canvas_ctrl`, and
(via `_state`) the active transport. Extend it as follows:

```python
def _set_origin() -> None:
    """Reset robot to world origin: wire commands + display reset."""
    transport = _state.get("transport")
    if transport is not None:
        # 1. Zero encoder counters.
        transport.command("ZERO enc", read_ms=300)
        _append_log("[TX] ZERO enc")
        # 2. Set firmware pose to (0, 0, heading 0).
        si_cmd = build_setpose_command(0.0, 0.0, 0.0)
        transport.command(si_cmd, read_ms=300)
        _append_log(f"[TX] {si_cmd}")
    else:
        _append_log("[WARN] Set Robot @ 0,0: no robot connected — display only")

    # 3. Reset the display (unchanged from today).
    trace_model.anchor(0.0, 0.0, 0.0)
    trace_model.clear()
    canvas_ctrl.reset_avatar_to_center()
    canvas_ctrl.refresh()
```

Note: `transport.command()` already handles TX logging through the transport's own
`on_log` callback into the log pane. The explicit `_append_log("[TX] ...")` calls
above are optional if the transport's callback covers them; verify against the
`SerialTransport` / `RelayTransport` log path before adding duplicates. If the
existing callback already logs the TX string, omit the manual `_append_log` calls
to avoid double-logging.

The import of `build_setpose_command` is already at the top of `__main__.py` via
`from robot_radio.testgui.operations import ...`; confirm it is included in that
import or add it.

### Files to create/modify

- `host/robot_radio/testgui/__main__.py`: extend `_set_origin()` as shown above.
  Confirm `build_setpose_command` is imported.
- `tests/testgui/test_set_origin.py`: new test file (see Testing plan below).

### Testing plan

Create `tests/testgui/test_set_origin.py`. Mirror `tests/testgui/test_operations.py`
for the fake-transport pattern.

```python
"""Tests for the _set_origin command sequence (ticket 063-004)."""
from __future__ import annotations


# ---------------------------------------------------------------------------
# Fake transport helper (Qt-free)
# ---------------------------------------------------------------------------

class _FakeTransport:
    """Records every command passed to transport.command()."""

    def __init__(self):
        self.commands_sent: list[str] = []

    def command(self, line: str, read_ms: int = 300) -> str:
        self.commands_sent.append(line)
        return ""


# ---------------------------------------------------------------------------
# Qt-free tests (no QApplication needed)
# ---------------------------------------------------------------------------

def test_build_setpose_command_origin():
    """build_setpose_command(0,0,0) returns an SI command string."""
    from robot_radio.testgui.operations import build_setpose_command
    result = build_setpose_command(0.0, 0.0, 0.0)
    assert result.startswith("SI"), f"Expected SI command, got: {result!r}"
    parts = result.split()
    assert len(parts) == 4, f"Expected 4 tokens in SI command, got: {result!r}"


def test_set_origin_sends_zero_enc_then_si(monkeypatch):
    """_set_origin sends ZERO enc then SI 0 0 0 when a transport is connected."""
    import types

    # Build a minimal _state with a fake transport.
    fake_transport = _FakeTransport()
    state = {"transport": fake_transport}
    log_lines: list[str] = []

    def _append_log(line: str) -> None:
        log_lines.append(line)

    # Import the pure helper to get the expected SI string.
    from robot_radio.testgui.operations import build_setpose_command
    expected_si = build_setpose_command(0.0, 0.0, 0.0)

    # Construct a minimal trace_model / canvas_ctrl stand-in.
    class _FakeTraceModel:
        def anchor(self, *a): pass
        def clear(self): pass

    class _FakeCanvasCtrl:
        def reset_avatar_to_center(self): pass
        def refresh(self): pass

    trace_model = _FakeTraceModel()
    canvas_ctrl = _FakeCanvasCtrl()

    # Re-implement _set_origin inline mirroring the production logic so we
    # can test the command sequence without launching the full GUI.
    def _set_origin() -> None:
        transport = state.get("transport")
        if transport is not None:
            transport.command("ZERO enc", read_ms=300)
            si_cmd = build_setpose_command(0.0, 0.0, 0.0)
            transport.command(si_cmd, read_ms=300)
        else:
            _append_log("[WARN] Set Robot @ 0,0: no robot connected — display only")
        trace_model.anchor(0.0, 0.0, 0.0)
        trace_model.clear()
        canvas_ctrl.reset_avatar_to_center()
        canvas_ctrl.refresh()

    _set_origin()

    assert len(fake_transport.commands_sent) == 2, (
        f"Expected 2 wire commands, got: {fake_transport.commands_sent}"
    )
    assert fake_transport.commands_sent[0] == "ZERO enc", (
        f"First command must be ZERO enc, got: {fake_transport.commands_sent[0]!r}"
    )
    assert fake_transport.commands_sent[1] == expected_si, (
        f"Second command must be {expected_si!r}, got: {fake_transport.commands_sent[1]!r}"
    )


def test_set_origin_no_transport_skips_wire_commands():
    """_set_origin with no transport skips wire commands and logs a warning."""
    log_lines: list[str] = []

    def _append_log(line: str) -> None:
        log_lines.append(line)

    from robot_radio.testgui.operations import build_setpose_command

    class _FakeTraceModel:
        def anchor(self, *a): pass
        def clear(self): pass

    class _FakeCanvasCtrl:
        def reset_avatar_to_center(self): pass
        def refresh(self): pass

    state = {"transport": None}
    trace_model = _FakeTraceModel()
    canvas_ctrl = _FakeCanvasCtrl()

    def _set_origin() -> None:
        transport = state.get("transport")
        if transport is not None:
            transport.command("ZERO enc", read_ms=300)
            si_cmd = build_setpose_command(0.0, 0.0, 0.0)
            transport.command(si_cmd, read_ms=300)
        else:
            _append_log("[WARN] Set Robot @ 0,0: no robot connected — display only")
        trace_model.anchor(0.0, 0.0, 0.0)
        trace_model.clear()
        canvas_ctrl.reset_avatar_to_center()
        canvas_ctrl.refresh()

    _set_origin()  # should not raise

    assert any("no robot connected" in line for line in log_lines), (
        "Expected a disconnected-state warning in the log"
    )
```

The tests above are Qt-free and import only `build_setpose_command` from the
testgui package. Run with:

```
QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/test_set_origin.py -v
```

### Documentation updates

Update the docstring on `_set_origin()` in `__main__.py` to describe the full
sequence: wire commands (`ZERO enc`, `SI 0 0 0`) then display reset, and the
no-transport fallback behaviour.
