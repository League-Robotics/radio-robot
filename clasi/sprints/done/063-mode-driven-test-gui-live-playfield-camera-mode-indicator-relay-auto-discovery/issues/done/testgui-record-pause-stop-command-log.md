---
status: done
sprint: '063'
tickets:
- 063-005
---

# Test GUI: Record / Pause / Stop command+response logging

## Problem / intent

The Test GUI should be able to **record a session** — every command sent to the robot
and every response received — so a run can be reviewed or replayed later.

Add three controls: **Record**, **Pause**, **Stop**.

- **Record** — start capturing. From this point, log every command the GUI sends to the
  robot (the wire string, e.g. `S 200 200`, `TURN 9000`, `SI ...`, `STREAM 50`) and every
  response/telemetry line received, each with a timestamp.
- **Pause** — temporarily suspend capture without ending the session; **Record** (or a
  Resume affordance) continues appending to the same recording.
- **Stop** — end the session and finalize/save the recording to a file.

## Guidance (for planning — not prescriptive)

- **Where to tap the stream:** commands flow through `SerialTransport`/`RelayTransport`/
  `SimTransport` via `transport.command(line)` / `transport.send(line)`; responses/telemetry
  flow back through the `on_log` callback and `on_telemetry`. The GUI already renders TX/RX
  lines to the log pane in `__main__.py` (`_append_log`, `_on_log_from_thread`). The recorder
  should tap the same TX/RX stream so it captures exactly what the operator sees, independent
  of transport type (Sim/Serial/Relay).
- **Format:** timestamped, machine-readable (e.g. JSONL or CSV) with direction (TX/RX), the
  line text, and a monotonic + wall-clock timestamp. Choose a location/naming the operator can
  find (e.g. under a `recordings/` dir with a timestamped filename). Leave exact format to
  planning; keep it simple and replay-friendly.
- **UI:** place Record/Pause/Stop near the other controls (operations panel or a small
  transport-toolbar). Button enable/disable should reflect state (e.g. Pause/Stop only while
  recording). Recording is independent of connection state where possible, but only meaningful
  once a transport is connected.
- **Threading:** TX happens on the GUI thread; RX/telemetry callbacks arrive on background
  threads and are already marshalled to the main thread via the bridge — append to the
  recording on the main thread to avoid races, or use a thread-safe sink.

## Acceptance (behavioural)
- Record/Pause/Stop controls exist with correct enable/disable states.
- After Record, both sent commands and received responses are captured with timestamps and
  direction; Pause suspends and resume/Record continues the same session; Stop writes the file.
- Works across Sim, Serial (bench), and Relay (playfield) transports.
- Headless tests cover the recorder's Qt-free core (append TX/RX, pause gating, serialize to
  the chosen format) via `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui -q`.
