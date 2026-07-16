---
id: "007"
title: "Rewire SimTransport onto sim_loop; un-gate Sim tour buttons; verify GUI launches and Tour 1 draws"
status: open
use-cases: ["SUC-042", "SUC-043"]
depends-on: ["006"]
github-issue: ""
issue:
  - "plan-pure-i2cbus-clock-interfaces-a-real-simplant-simulator.md"
  - "binary-bridge-segment-replace-arms-deleted.md"
completes_issue:
  plan-pure-i2cbus-clock-interfaces-a-real-simplant-simulator.md: false
  binary-bridge-segment-replace-arms-deleted.md: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Rewire SimTransport onto sim_loop; un-gate Sim tour buttons; verify GUI launches and Tour 1 draws

## Description

Stage 3 (part c) of the master plan — the ticket that closes the sprint's
headline outcome: pressing **Tour 1** in the TestGUI (Sim) drives the real
compiled firmware against `SimPlant` and draws the trace.

1. Rewire `SimTransport` (`host/robot_radio/testgui/transport.py`) onto
   `sim_loop.py` (ticket 006) instead of the deleted `sim_conn.SimConnection`:
   add `.protocol`, `suspend_telemetry_reader`/`resume_telemetry_reader`,
   keep the existing tick-thread lifecycle contract (`connect()`/
   `disconnect()` owning the `sim_loop` object for the thread's lifetime —
   the same shape `SimConnection` used to have, per `transport.py`'s
   existing docstrings).
2. Repoint `sim_prefs.py`'s `PROFILE_TO_SIM_SETTER` map and its
   special-cased knobs (`encoder_noise`, `enc_scale_err_l/r`) at
   `sim_loop`'s fault-condition setters (ticket 006) instead of the dead
   `SimConnection` ones.
3. Un-gate the Sim Tour buttons: `__main__.py`'s `_TOUR_SIM_TOOLTIP`
   gating in `_on_connect()` (around line 2525) currently disables/
   tooltips-over the Tour buttons for the Sim backend. Remove that gating
   now that `SimTransport` can actually run a tour.
4. **binary-bridge-segment-replace-arms-deleted.md scope for this ticket**
   (verify-only, per architecture-update.md Decision 4 — do NOT expand
   into a rewrite of `binary_bridge.py`'s manual command-row translation,
   that stays a separate, deferred stakeholder call):
   - Confirm `python -m robot_radio.testgui` (Sim backend) launches
     without `ImportError` (107-003's guard should already hold — this is
     a regression check, not new work).
   - Confirm `SimTransport`'s call graph (as rewired in this ticket) never
     calls `binary_bridge.translate_command()`'s `segment`/`replace`
     builders (`R`/`TURN`/`G`). If the rewired `SimTransport` needs a
     single-command path at all this sprint (it may not — tours are the
     acceptance bar), it goes through `sim_loop`'s twist-shaped surface,
     never through `binary_bridge`.

## Acceptance Criteria

- [ ] `SimTransport` is fully rewired onto `sim_loop.py`; no reference to
      `sim_conn`/`SimConnection` remains in `transport.py`.
- [ ] `sim_prefs.py`'s fault-knob keys apply correctly to a live sim
      session (bench/manual check: toggle a knob in the Sim Errors panel,
      confirm the simulated telemetry visibly changes).
- [ ] Sim Tour buttons are un-gated: `_TOUR_SIM_TOOLTIP` gating removed
      from `_on_connect()` for the Sim backend.
- [ ] Headless: a Tour 1 run through `sim_loop` (can reuse ticket 004's
      standalone-driver pattern, now through the Python ABI) completes
      every leg with finite/small closure — this is the master plan's own
      Verification item 5.
- [ ] Manual/bench (final acceptance): `just testgui` → Connect (Sim) →
      press **Tour 1** → the trace draws on the canvas — the master plan's
      own Verification item 6.
- [ ] `python -m robot_radio.testgui` (Sim backend) launches without
      `ImportError`.
- [ ] `grep -n "translate_command" host/robot_radio/testgui/transport.py`
      shows no call reachable from `SimTransport`'s own methods (only from
      `_HardwareTransport`'s, which is unaffected by this ticket).

## Implementation Plan

**Approach**: Reuse `SimTransport`'s existing tick-thread/lifecycle
structure (it already exists and is well-documented in `transport.py`) —
swap only what constructs/owns underneath it, from `SimConnection` to
`sim_loop`'s object. Do not restructure `SimTransport`'s external API
(`Transport` ABC conformance) — `planner/tour.py` and the rest of the GUI
already consume it through that unchanged surface.

**Files to modify**:
- `host/robot_radio/testgui/transport.py` (`SimTransport` class)
- `host/robot_radio/testgui/sim_prefs.py` (setter map repoint)
- `host/robot_radio/testgui/__main__.py` (remove Sim tour gating)

**Testing plan**:
- Existing: `tests/testgui/test_tour1_geometry.py` — un-skip its
  `_LIB_PRESENT` guard now that the sim library and ABI exist; it becomes
  a real, running regression test of Tour 1 geometry against physics.
- New: a headless Tour 1 closure test (finite/small closure assertion) —
  the master plan's Verification item 5, made durable.
- Manual/bench: `just testgui`, Connect (Sim), press Tour 1, confirm trace
  draws — record the result in the ticket's own completion notes since
  this is a human-observed check, not an automated one.
- Verification command: `uv run python -m pytest tests/testgui` (full
  suite) plus the manual bench step above.

**Documentation updates**: `transport.py`'s `SimTransport` docstring
(currently documents `SimConnection` usage) updated to describe
`sim_loop`; `sim_prefs.py`'s module docstring's "083-001" note updated to
reflect the new setter target.
