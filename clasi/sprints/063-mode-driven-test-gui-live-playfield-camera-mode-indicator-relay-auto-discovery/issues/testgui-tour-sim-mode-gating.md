---
status: in-progress
sprint: '063'
tickets:
- 063-007
---

# TestGUI: Tour runs in Sim mode / connection gating for Tour is unclear

## Symptom (reported)

"The tour is starting in Sim mode without being connected."

## Code analysis

The Tour click handler guards on `_state["transport"]` being present
(`if transport is None: … return`), and the tour buttons are enabled only after
a successful **Connect** (`for _sb in _send_buttons: _sb.setEnabled(True)` in
`_on_connect`). So from code alone the Tour should not run with *no* transport.

The most likely real behavior: **a `SimTransport` counts as "connected."** When
"Sim" is selected and Connect is pressed, a transport is set, tour buttons
enable, and Tour will happily run its motion sequence against the simulator —
which the operator may perceive as "running without being connected" (i.e. not
on the real robot / playfield).

This needs a **live reproduction detail** to disambiguate:
- Is the Tour button somehow clickable *before* Connect (a stale enable-state
  bug), or
- Is the complaint that Tour should not be runnable in Sim at all?

## Decision needed

Should Tour be:
- gated to hardware transports (Relay/Serial) only,
- allowed in Sim but with a clear warning/log, or
- left as-is (Sim is a valid target for dry-running a tour)?

## Affected code

- `host/robot_radio/testgui/__main__.py` — tour click handler
  (`_make_tour_handler`), `_send_buttons` enable/disable in `_on_connect` /
  `_on_disconnect`.
