---
id: "008"
title: "Camera/relay selection, mode label, and session recorder test port"
status: open
use-cases: [SUC-009, SUC-010]
depends-on: []
github-issue: ""
issue: host-testgui-full-revival.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Camera/relay selection, mode label, and session recorder test port

## Description

This ticket bundles the remaining independent, already-implemented, small
surfaces this sprint's issue scopes in under "the remaining GUI test port":

- **Camera/relay device selection**: `camera_prefs.py`'s
  `select_camera`/`load_camera_pref`/`save_camera_pref` (persisted
  preference → name-heuristic fallback → first available) and
  `transport.py`'s `find_relay_port`/`_relay_probe_banner` (serial-port
  probing for a relay's `!HELLO`-class banner).
- **Mode label**: `__main__.py`'s `transport_name_to_mode_label(name)`,
  mapping a connected transport's name to "SIM MODE"/"BENCH MODE"/
  "PLAYFIELD MODE" text + style.
- **Session recorder**: `recorder.py`'s `SessionRecorder` and
  `direction_from_marker`, persisting session wire traffic as JSONL,
  independent of any transport/runner.

Each is small, self-contained, and already implemented; each changes for a
different reason than the others (device selection vs. UI labeling vs.
logging), but all three are quick, low-risk verification passes with no
shared dependency, so they are grouped into one ticket for sizing (per
`architecture-update.md` Decision 3's cohesion-vs-ticket-sizing tradeoff).

## Acceptance Criteria

- [ ] `tests_old/testgui/test_camera_combo.py`, `test_camera_prefs.py`,
      `test_relay_discovery.py`, `test_mode_indicator.py`, and
      `test_recorder.py` are ported to `tests/testgui/`, updated for any
      API drift, and pass under `QT_QPA_PLATFORM=offscreen`.
- [ ] Camera-combo population and preference persistence/fallback
      (persisted pref → name-heuristic → first available) all hold.
- [ ] Relay-port discovery correctly classifies a relay's `!HELLO`-style
      banner and rejects non-relay serial devices.
- [ ] The mode label maps each known transport name to the correct text
      and style, and handles an unknown name gracefully.
- [ ] `SessionRecorder`'s start/pause/stop state transitions (including the
      raise-on-start-when-already-recording/paused guards) behave as
      documented; every appended line is valid, newline-free JSON; a
      paused session does not record; `direction_from_marker` correctly
      classifies TX/RX vs. status lines.
- [ ] Any genuine bug surfaced by a real run is fixed here and documented.

## Testing

- **Existing tests to run**: full `tests/testgui` suite (regression).
- **New tests to write**: port the five files above.
- **Verification command**: `QT_QPA_PLATFORM=offscreen uv run pytest
  tests/testgui -q`
