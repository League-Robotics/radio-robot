---
id: '001'
title: Mode indicator and transport-combo plumbing
status: done
use-cases:
- SUC-001
- SUC-004
depends-on: []
github-issue: ''
issue: live-camera-view-for-the-test-gui.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Mode indicator and transport-combo plumbing

## Description

Add a `QLabel` near the top of the right panel in the Test GUI that shows the
current operating mode. The label updates immediately whenever the transport
combo selection changes, before any connection is made.

The mapping from combo item to label text is a pure, Qt-free function
`transport_name_to_mode_label(name)` so it can be tested without a QApplication.

This ticket also lays the groundwork for tickets 002 and 003 by ensuring the
transport name is reliably accessible (it always has been, but the explicit
helper makes mode-gated behavior in later tickets trivial to write and test).

**Files to modify:**
- `host/robot_radio/testgui/__main__.py`

**Files to create:**
- New test cases in `tests/testgui/test_smoke.py` or a new
  `tests/testgui/test_mode_indicator.py`

## Acceptance Criteria

- [x] A `QLabel` with object name `"mode_label"` is visible in the right panel,
      above the playfield canvas, at all times.
- [x] With "Sim" selected, label text is exactly `"SIM MODE"`.
- [x] With "Serial" selected, label text is exactly `"BENCH MODE"`.
- [x] With "Relay" selected, label text is exactly `"PLAYFIELD MODE"`.
- [x] Label updates immediately on combo change — no connect/disconnect needed.
- [x] `transport_name_to_mode_label("Sim")` returns `"SIM MODE"` (and the color).
- [x] `transport_name_to_mode_label("Serial")` returns `"BENCH MODE"`.
- [x] `transport_name_to_mode_label("Relay")` returns `"PLAYFIELD MODE"`.
- [x] Unknown transport names return a safe fallback (e.g. `"UNKNOWN MODE"`).
- [x] All existing `tests/testgui/` tests pass unchanged.

## Implementation Plan

### Approach

1. Add `transport_name_to_mode_label(name: str) -> tuple[str, str]` as a
   module-level function in `__main__.py`. It maps:
   - `"Sim"` → `("SIM MODE", "color: #808080; font-weight: bold;")`
   - `"Serial"` → `("BENCH MODE", "color: #4080ff; font-weight: bold;")`
   - `"Relay"` → `("PLAYFIELD MODE", "color: #20c020; font-weight: bold;")`
   - anything else → `("UNKNOWN MODE", "color: #ff8000; font-weight: bold;")`

2. In `_build_main_window()`, create the label in the right panel layout,
   before `right_splitter` is added:
   ```
   mode_label = QLabel("SIM MODE")
   mode_label.setObjectName("mode_label")
   mode_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
   right_layout.addWidget(mode_label)
   right_layout.addWidget(right_splitter)
   ```

3. In `_on_transport_changed(index)`, update the label after the existing
   `port_edit.setEnabled(...)` call:
   ```
   name = transport_combo.currentText()
   text, style = transport_name_to_mode_label(name)
   mode_label.setText(text)
   mode_label.setStyleSheet(style)
   ```

4. Because `transport_name_to_mode_label` does not import PySide6, it can be
   placed before `_build_main_window()` at module level.

### Files to create/modify

- `host/robot_radio/testgui/__main__.py`: add `transport_name_to_mode_label()`,
  insert `mode_label` widget in right panel layout, update
  `_on_transport_changed()`.

### Testing plan

Add `tests/testgui/test_mode_indicator.py` (or extend `test_smoke.py`):

```python
# Qt-free tests (no QApplication needed)
def test_transport_name_to_mode_label_sim():
    from robot_radio.testgui.__main__ import transport_name_to_mode_label
    text, _ = transport_name_to_mode_label("Sim")
    assert text == "SIM MODE"

def test_transport_name_to_mode_label_serial():
    from robot_radio.testgui.__main__ import transport_name_to_mode_label
    text, _ = transport_name_to_mode_label("Serial")
    assert text == "BENCH MODE"

def test_transport_name_to_mode_label_relay():
    from robot_radio.testgui.__main__ import transport_name_to_mode_label
    text, _ = transport_name_to_mode_label("Relay")
    assert text == "PLAYFIELD MODE"

def test_transport_name_to_mode_label_unknown():
    from robot_radio.testgui.__main__ import transport_name_to_mode_label
    text, _ = transport_name_to_mode_label("Bluetooth")
    assert "MODE" in text  # safe fallback

# Qt widget test (requires QApplication via conftest offscreen fixture)
def test_mode_label_updates_on_combo_change(qapp):
    from robot_radio.testgui.__main__ import _build_main_window
    from PySide6.QtWidgets import QComboBox, QLabel
    window, _ = _build_main_window()
    combo = window.findChild(QComboBox, "transport_combo")
    label = window.findChild(QLabel, "mode_label")
    combo.setCurrentText("Relay")
    assert label.text() == "PLAYFIELD MODE"
    combo.setCurrentText("Sim")
    assert label.text() == "SIM MODE"
    window.close()
```

### Documentation updates

Update the module docstring in `__main__.py` to mention the mode indicator label
and `transport_name_to_mode_label`.
