---
id: '010'
title: testgui README and headless CI smoke test
status: open
use-cases:
- SUC-015
depends-on:
- '009'
issue: plan-robot-test-gui-pyside6.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 010 — testgui README and headless CI smoke test

## Description

The final GUI ticket. Write `host/robot_radio/testgui/README.md` covering
launch instructions, transport selection, and how to build the sim lib. Write
headless pytest smoke tests in `tests/testgui/` that run with
`QT_QPA_PLATFORM=offscreen` — no display or hardware required. The tests inject
a fake Transport, push synthetic `TLMFrame` objects, and assert the trace model
and robot marker are updated correctly. They also validate each command row emits
the correct wire string.

Corresponds to item 8 in the approved design's ticket breakdown.

## Acceptance Criteria

- [ ] `host/robot_radio/testgui/README.md` covers:
  - Prerequisites: `uv sync --group gui`
  - Launch: `python -m robot_radio.testgui`
  - Transport selection: Sim (build lib first with `python build.py`), Serial,
    Relay
  - Sim lib build: `cd <project_root> && python build.py`
  - Interactive driving: cursor keys
  - Syncing pose from camera: prerequisites (aprilcam daemon)
- [ ] `tests/testgui/conftest.py` sets `os.environ["QT_QPA_PLATFORM"] = "offscreen"`
  before Qt is imported.
- [ ] `tests/testgui/test_smoke.py` (or equivalent) passes with
  `uv run python -m pytest tests/testgui/ -v`:
  - `test_app_opens`: construct `QMainWindow` with a fake Transport; confirm it
    does not crash.
  - `test_trace_model_feeds_tlm`: push 3 synthetic `TLMFrame` objects to
    `TraceModel.feed()`; assert each trace list has 3 points.
  - `test_robot_marker_moves`: after feeding TLM, assert the robot marker's
    scene position changed from the initial position.
  - `test_command_rows_emit_correct_wire_strings`: for each of S, T, D, R,
    TURN, G — set the row fields programmatically and click Send on a fake
    Transport; assert `transport.last_sent` equals the expected wire string.
  - `test_turn_row_converts_degrees_to_centidegrees`: set TURN row to 90°;
    assert sent string contains `9000` (centidegrees).
- [ ] All headless tests are excluded from `tests/simulation/` (run via
  `uv run python -m pytest tests/testgui/`, not included in the simulation gate).
- [ ] `uv run python -m pytest tests/simulation` still passes (simulation gate
  unaffected).
- [ ] `uv run python -m pytest tests/testgui/` passes (new gate, no hardware
  required if `QT_QPA_PLATFORM=offscreen`).

## Implementation Plan

### Approach

For the fake Transport: create a `FakeTransport(Transport)` in `conftest.py`
or a `tests/testgui/fake_transport.py` module:
```python
class FakeTransport(Transport):
    def __init__(self):
        self.sent: list[str] = []
    def send(self, line): self.sent.append(line)
    def command(self, line, read_ms=200): self.sent.append(line); return "OK"
    def connect(self): pass
    def disconnect(self): pass
```

For the synthetic TLMFrame: construct minimal frames with known `.pose` / `.enc`
/ `.otos` delta values. `TraceModel.feed()` must accept these without crashing.

For TURN centidegree test: programmatically set the TURN row's degree field to 90,
then simulate a button click (or call the handler directly). Assert the wire
string contains `9000`.

### Files to create

- `tests/testgui/__init__.py`
- `tests/testgui/conftest.py` — offscreen platform, FakeTransport
- `tests/testgui/test_smoke.py` — the five smoke tests above

### Files to modify

- `host/robot_radio/testgui/README.md` (new file, technically)

### Testing plan

`uv run python -m pytest tests/testgui/ -v` — all tests must pass. Then run
`uv run python -m pytest tests/simulation` to confirm no regressions.

### Documentation updates

`host/robot_radio/testgui/README.md` is the primary doc artifact of this ticket.
