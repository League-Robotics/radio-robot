---
status: pending
---

# nav/ goto stack is dead code wired to live CLI/MCP surfaces — gate it loudly now, then rebuild on pathplan or delete

**Source:** code review 2026-07-30, `03-host-core.md` CRITICAL §3,
`04-host-planning.md` CRITICAL §1 + MAJOR §5.
**Priority:** P0 for the loud gate (one small commit); the rebuild/delete
decision rides sprint 127's pathplan work.
**Goal served:** five sibling packages currently imply five ways to move the
robot; the true count is zero-live/one-dead/one-differently-scoped/
one-in-progress. Making the dead one *say so* — and then deleting it once
pathplan lands — collapses the search space for every future motion bug.

## What is wrong

- `nav/camera_goto.py:149,295` calls `proto.drive(...)`;
  `nav/navigator.py:211` calls `self._robot.go_to(...)` → both deleted from
  the wire surface. `rogo goto`/`rogo turnto` and the MCP tools
  `navigate`/`follow_path`/`visit_tags` (robot_mcp.py:229,271) crash with
  `AttributeError` three call-frames deep, mid-loop.
- Every failure branch in `go_to_world_camera`/`Navigator.approach()` halts
  with the planned `stop()` — while `spin_to_yaw_camera`, in the same file,
  does it right with `estop()` and documents why.
- `nav/pose.py` is NOT dead — it is live, foundational, imported everywhere.
  Only `camera_goto.py`/`navigator.py`'s motion paths are dead.

## What to do

**Step 1 — immediately (small, standalone commit):** make the dead surface
fail at the front door, not mid-loop:

```python
# top of go_to_world_camera / Navigator.navigate / follow_path / visit_tags
raise NotImplementedError(
    "dead since the v5 wire cutover: NezhaProtocol.drive()/go_to() were "
    "deleted (104-002). The replacement is pathplan.gotoWorld (sprint 127). "
    "Tracked: clasi/issues/nav-goto-stack-is-dead-gate-it-loudly-then-"
    "rebuild-or-delete.md")
```

The MCP tools must catch this and report an honest "not available" result —
an LLM operator must not be advertised a `navigate` capability that
stack-traces. Either deregister the tools or return the message above.

**Step 2 — when sprint 127's pathplan goto lands:** delete
`nav/camera_goto.py` and `nav/navigator.py` outright (gut decisively; the
capability's new owner is `pathplan/`), and repoint `rogo goto`/`turnto` and
the MCP tools at the pathplan implementation. Keep `nav/pose.py` (move it if
`nav/` becomes empty — e.g. to `robot_radio/pose.py` — updating importers).
Do NOT port the old control loops: `go_to_world_camera`'s burst-drive shape
predates the continuous-resolve design pathplan implements.

If any piece of `camera_goto.py` is worth saving, it is
`spin_to_yaw_camera`'s estop-and-explain arrival pattern — carry the pattern
(and its comment) into the pathplan implementation, not the file.

## Acceptance

- Step 1: `rogo goto 30 10` prints the NotImplementedError message
  immediately (no traceback mid-drive); MCP `navigate` returns the honest
  error; a unit test asserts both.
- Step 2: `nav/camera_goto.py`/`nav/navigator.py` are deleted; `grep -rn
  "camera_goto\|Navigator" src/host src/tests` shows only pathplan-era
  code; `rogo goto` drives via pathplan on the playfield with the geofence
  active and camera fixes at segment boundaries per
  `.claude/rules/playfield-testing.md`.
