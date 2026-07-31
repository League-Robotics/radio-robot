---
status: pending
---

# Rebuild the Nezha driver facade on the protocol-v5 binary surface, with one pose owner and no private-field reach-around

**Source:** code review 2026-07-30, `03-host-core.md` CRITICAL §1, MAJOR §6
(interface bleed), MAJOR §7 (three pose trackers). Supersedes and activates
`clasi/issues/later/nezha-facade-and-midlayer-dead-verb-residue.md`.
**Priority:** P0 — `Nezha` is the class `make_robot()` constructs for the
currently-configured robot (`tovez.json`, `"hardware_model": "DFRobot
Nezha"`), and ~25 of its method calls hit `NezhaProtocol` methods that were
deleted by the v4→v5 cutover. `rogo goto`, `rogo enc`, `rogo grip`, and most
`rogo` verbs raise `AttributeError` on first contact with hardware.
**Goal served:** the driver facade is the layer every CLI/MCP/nav bug hunt
passes through. Today a reader cannot tell which of its 800 lines are real.
Rebuilding it small, on the live surface, with one pose owner, turns the
most-traversed layer in the host tree from a minefield into a map.

## Why the old blocker is gone

The superseded issue said liveness/identity had "no replacement mechanism"
on the wire. That was true under v4. **Protocol v5 restored cleartext
`HELLO`/`PING`/`ID`/`VER`** (docs/protocol-v5.md) — so `connect()` can be
rebuilt properly now, not worked around.

## What to do

1. **Inventory, then rebuild small.** Enumerate `Nezha`'s public methods and
   sort into: (a) has a v5 equivalent → rebuild; (b) capability retired with
   the wire (e.g. `port_read/write`, `snap`) → delete, and delete its CLI/MCP
   callers in the same pass; (c) needs a design decision → list in the PR for
   stakeholder sign-off. Do NOT keep method stubs "for later" — dead verbs
   are what caused this.

2. **Rebuild pattern** — every motion method is a bounded Move plus an ack
   check, the shape `field/geofence.py` and the bench scripts already use:

```python
# BEFORE (dead -- protocol v2/v3 verbs deleted by 104-002/124)
def go_to(self, x, y, speed=150):
    self._proto.go_to(x, y, speed)
    self._proto.stream(80)
    self._proto.wait_for_evt_done("G", timeout=self._timeout)

# AFTER (v5: bounded Move + ack ring; no blocking wire verbs exist)
def move_distance(self, distance, speed=150, *, timeout=8000):  # [mm] [mm/s] [ms]
    """Bounded straight move. Enqueue ack confirms acceptance; completion
    rides the ack ring under the Move's own id."""
    corr_id = self._proto.move_wheels(
        math.copysign(speed, distance), math.copysign(speed, distance),
        stop_distance=abs(distance), timeout=timeout)
    ack = self._proto.wait_for_ack(corr_id)
    if ack is None or not ack.ok:
        raise CommandError(f"move rejected: {ack}")
    return corr_id
```

   `connect()` uses the v5 `HELLO` banner + `PING`; halts use the shared
   `halt_now()` helper (see the estop sweep issue).

3. **One pose owner.** Today three siblings independently re-derive pose
   from raw telemetry with different unit conversions:
   `Nezha._apply_tlm` (nezha.py:587-633, `math.radians(heading/100.0)`),
   `NezhaState._apply_tlm` (nezha_state.py:124-165, `heading/18000.0*pi`),
   and `NezhaKinematic._update_odometry` (a third, WPILib-based estimator).
   Pick ONE owner — `Nezha.robot.state` per its own "canonical" docstring —
   and make the other two read from it or delete them. A single
   `_tlm_to_pose(tlm) -> Pose` function is the only place a unit conversion
   may appear.

4. **Kill the reach-around.** `nezha.py` calls `self._proto._conn.send_fast()`
   / `._conn.read_lines()` in 11 places even though `NezhaProtocol` publicly
   exposes `send_fast()`/`read_lines()`/`read_pending_lines()` — use the
   public methods. `io/cli.py:655,741` and `io/robot_mcp.py:96` fetch
   `getattr(robot, "_proto", None)`: give `Robot` an explicit, documented
   accessor (e.g. a `protocol` property) and convert all three sites.

5. **Fate of `nezha_state.py`/`nezha_kinematic.py`:** rebuild atop the one
   pose owner or delete (`kinematics/differential_drive.py`'s only caller is
   `nezha_kinematic.py`; if it goes, the CW-positive-convention module goes
   with it — see the review's `04` §6).

## Acceptance

- Grep gate (the one 104-002 left failing): zero calls anywhere in
  `src/host` to `ping|echo|get_id|get_ver|get_config|grip|drive|timed|
  distance|go_to|turn|stream|snap|stream_drive|zero_encoders|zero_otos|
  otos_get_*|otos_set_*|set_internal_pose|port_*|wait_for_evt_done` as
  `NezhaProtocol` methods.
- `grep -rn "_proto\._conn\|getattr(robot, \"_proto\"" src/host` returns
  nothing.
- Exactly one telemetry→pose conversion function exists in `robot/`.
- Bench check on the stand: `rogo repl` connect banner, `move_distance`
  drives with climbing encoders, ack-ring completion observed.
- `clasi/issues/later/nezha-facade-and-midlayer-dead-verb-residue.md` is
  moved to done/ (its scope is fully absorbed here and by the nav and
  calibrate issues).
