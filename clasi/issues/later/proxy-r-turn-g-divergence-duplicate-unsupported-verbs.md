---
status: pending
---

# rogo translator proxy still gates R/TURN/G as "unsupported" — drifted duplicate of _ALWAYS_UNSUPPORTED_VERBS

## Description

097's un-gating of R/TURN/G removed the three verbs from
`host/robot_radio/testgui/binary_bridge.py`'s `_ALWAYS_UNSUPPORTED_VERBS`
(they now translate to open-loop `segment`/`replace` envelopes via
`legacy_translate.segment_for_arc()`/`segment_for_turn()`/
`segment_for_goto_relative()`, wired into the **shared**
`legacy_verbs.BINARY_DISPATCH`).

But `host/robot_radio/io/proxy.py` keeps its own copy:

```python
_ALWAYS_UNSUPPORTED_VERBS = frozenset({"QLEN", "G", "R", "TURN", "GRIP"})
```

So the rogo translator proxy — per the stakeholder decision (2026-07-10),
**the** path legacy text clients use to reach the binary-only firmware —
still replies typed `ERR unsupported` for R/TURN/G even though the
translations it needs are already in the `BINARY_DISPATCH` table it shares
with the TestGUI bridge. A rogo client can no longer do something the
TestGUI can. The TestGUI-side comment marks this "out of this ticket's
scope", so it is a known cut — this issue exists so it doesn't get lost.

## Root cause worth fixing, not just the symptom

The drift was made possible by **two hand-maintained copies of the same
constant**. Rather than editing the proxy's copy to match (which just arms
the next drift), derive the gate from `BINARY_DISPATCH` membership — a verb
is "unsupported" exactly when it has no builder in the shared table and is
not in one of the deliberately-gated families (`_POSE_RESET_VERBS` /
OTOS / `DEV`). One source of truth, in `legacy_verbs.py` next to the table,
imported by both `binary_bridge.py` and `io/proxy.py`; delete both local
copies.

## Acceptance sketch

- `R`/`TURN`/`G` through the rogo proxy translate and reply `OK arc`/`OK
  turn`/`OK goto` exactly as `tests/testgui/test_binary_bridge.py` asserts
  for the TestGUI bridge.
- `GRIP`/`QLEN` remain `unsupported` on both paths; pose/OTOS families
  remain gated on both paths.
- No module defines its own private unsupported-verbs set.
