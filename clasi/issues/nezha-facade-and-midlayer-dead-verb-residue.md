---
status: pending
sprint: '113'
---

# Nezha facade + mid-layer host code still call dead-verb NezhaProtocol methods

From 104-002 (Legacy translator and dead-verb deletion). 104-002's own
Implementation Plan scoped the deletion sweep to `protocol.py`,
`serial_conn.py`, `io/cli.py`, and the four `tests/unit` files exercising
them — the set needed to take `uv run python -m pytest` from 112 failed/5
errors to 0 failed/0 errors. That gate is now green (546/546 across
`tests/sim` + `tests/unit`).

104-002's own acceptance criterion 2 additionally asked for a repo-wide grep
proving zero remaining callers of the retired verb methods. That grep is
NOT clean — a substantial mid-layer of host code still calls the
now-deleted `NezhaProtocol` methods (`ping`/`echo`/`get_id`/`get_ver`/
`get_config`/`grip`/`drive`/`timed`/`distance`/`go_to`/`turn`/`stream`/
`snap`/`stream_drive`/`zero_encoders`/`zero_otos`/`otos_*`/
`set_internal_pose`/`port_*`/`wait_for_evt_done`), because these arms were
retired from `envelope.proto`/`protocol.py` well before this mid-layer was
updated (or even could be, since the P4 wire has no replacement for most of
it — see below).

## Why this was NOT fixed in 104-002

Rewriting this mid-layer is a different-shaped job than a mechanical
dead-code sweep:

- **`host/robot_radio/robot/nezha.py`** (802 lines) — the `Nezha` `Robot`
  facade. Essentially every public method is a thin wrapper over one of
  the now-deleted `NezhaProtocol` methods, INCLUDING `connect()` (uses
  `ping()`+`get_id()` for liveness/identity — both retired, and the P4
  wire has **no replacement liveness/identity mechanism at all**). This is
  not a delete-the-dead-code job; it needs a designed replacement for
  connect/liveness before anything else in the class can be un-broken.
- **`host/robot_radio/robot/nezha_state.py`**,
  **`nezha_kinematic.py`** — same pattern, one layer up.
- **`host/robot_radio/nav/camera_goto.py`**, **`nav/navigator.py`** — call
  `NezhaProtocol.drive()`/`Robot.go_to()` directly; closed-loop nav
  primitives that assumed blocking T/D/G/TURN + EVT-done completion, none
  of which exist on the P4 wire (a velocity/yaw follower with a
  telemetry-only ack-ring return path, not a blocking-command protocol).
- **`host/robot_radio/io/calibrate.py`** — calibration routines
  structurally depend on precise blocking timed/distance drives
  (`proto.distance()`/`.zero_encoders()`/`.zero_otos()`), which have no P4
  binary equivalent; calibration would need to be redesigned atop
  `twist()`-based host-side trajectory generation.
- **`host/robot_radio/io/robot_mcp.py`** — `_robot.grip()`, same
  Robot-interface pattern as the above.
- **`host/robot_radio/testgui/binary_bridge.py`**,
  **`host/robot_radio/testkit/safety.py`** — direct `NezhaProtocol`
  callers (`stream`/`snap`/`get_config_binary`/`ping`). `testgui` is
  already known-parked pending revival (dropped from `pyproject.toml`
  `testpaths` at 102; revival scoped to sprint 107 per team-lead
  direction) — `testkit/safety.py` underlies it.

None of the above have any `tests/unit` coverage today (verified: zero
hits for `nezha`, `calibration`, `nav`, `testgui`, `testkit` imports
across `tests/unit`), so none of this breaks the pytest gate — the break
is a real, but currently silent, runtime one for any script that actually
calls these paths against a P4-firmware robot.

## Why deleting or gutting it unilaterally in 104-002 was rejected

- 104-002's own Implementation Plan didn't name these files, and no other
  ticket in sprint 104 (001/003/004/005/006/007) covers this layer either
  — ticket 006 rewrites the `tests/bench/` script family onto the binary
  plane, establishing that bench-facing capability is meant to survive,
  just via a redesign, not a ticket 002-shaped deletion.
- Some of this (calibration, closed-loop nav) is real, stakeholder-valued
  capability, not merely legacy cruft — deleting it outright without a
  replacement design would be a silent, unreviewed loss of function, not
  a "dead code" cleanup.
- Connect/liveness in particular has NO current wire replacement to fall
  back to (ping/id are both retired) — fixing `nezha.py.connect()`
  requires a P4 wire-level design decision (e.g. "liveness = telemetry
  arriving at all"), not just a code change.

## Direction

A future sprint (105+, likely alongside or after the `sim`/host-fusion
work already on the roadmap) should:

1. Decide the P4-era liveness/identity story (telemetry-arrival-based? a
   new minimal wire arm?) and rewire `Nezha.connect()` on it.
2. Redesign `Nezha`'s motion surface around `twist()`/`wait_for_ack()`
   only — no blocking T/D/G/TURN/RT primitives exist on the wire any
   more; host-side trajectory generation (streaming `twist()` calls) is
   the only path, matching the single-loop firmware's own
   "host computes the trajectory" design (architecture-update.md (103)).
3. Decide per-module fate for `nav/`, `io/calibrate.py`: rebuild atop the
   new `Nezha` motion surface, or retire if the capability's value doesn't
   justify the redesign cost — a stakeholder call, not a default.
4. Fold `testgui/binary_bridge.py`/`testkit/safety.py` into whatever
   sprint 107's testgui revival does — they're already known-broken in
   the same way `testgui` itself is.

Until then: `host/robot_radio/robot/nezha.py` and everything downstream of
it (`nezha_state.py`, `nezha_kinematic.py`, `nav/`, `io/calibrate.py`,
`io/robot_mcp.py`'s `grip()` path, `testgui/binary_bridge.py`,
`testkit/safety.py`) should be treated as **non-functional against a P4
firmware robot** — same status as `testgui` itself, just not yet
formally flagged as such outside this issue.
