---
id: '006'
title: 'Bench/HITL: port goto_otos.py to GO_TO, stand verification on tovez'
status: open
use-cases:
- SUC-001
- SUC-002
depends-on:
- '005'
github-issue: ''
issue: replaceable-go-to-moves-in-the-motion-library.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Bench/HITL: port goto_otos.py to GO_TO, stand verification on tovez

## Description

Satisfy the standing hardware-verification gate
(`.claude/rules/hardware-bench-testing.md`) for a firmware sprint that
touches motion and the command protocol: deploy this sprint's firmware to
the real robot and exercise `GO_TO` on the stand. Port
`src/tests/bench/goto_otos.py` from driving the goto loop itself to a
thin `GO_TO`-emitting script — its TURN_FIRST/fine-approach/YAW_SIGN
policy knowledge already moved into `Motion::Navigator` (ticket 003/004);
this script becomes a driver and scorer, not a policy owner.

## Hardware target — read this before touching any hardware command

**`tovez` ONLY, addressed BY UID, never by port number or a
default/auto-selected target:**

```
tovez UID: 9906360200052820a8fdb5e413abb276000000006e052820
```

- `vizev` and `togov` are OTHER PEOPLE'S robots sharing the same USB hub.
  Every default-target path (`mbdeploy deploy` with no target, `pyocd`
  with no `-u`) picks the WRONG robot whenever `tovez` happens to be
  unplugged — never fall back to "the only device present."
- Port numbers MOVE on every re-enumeration — never reuse a remembered
  port from a prior session. Take the port fresh from `uv run mbdeploy
  list`'s live view for this session only, and confirm the row literally
  says `tovez` before running anything.
- Deploy with the full UID:
  ```
  uv run mbdeploy deploy --hex ./MICROBIT.hex 9906360200052820a8fdb5e413abb276000000006e052820
  ```
  (or `uv run mbdeploy deploy --build` if flashing from a fresh build,
  confirming the target UID either way per
  `.claude/rules/hardware-bench-testing.md`).
- If `mbdeploy list` does not show `tovez`, it is UNPLUGGED — stop and
  report that. Do not proceed against whatever device IS present.

**Transport this session: DIRECT SERIAL ONLY.** `tovez` is disconnected
from the Raspberry Pi and there is no relay path available this session.
Every verb below runs over `tovez`'s own direct USB serial port (the
`ROLE` column in `mbdeploy list`, NOT a `RADIOBRIDGE` row — there isn't
one to pick from this session). Do not attempt a relay leg; if a relay
happens to be attached later, that is out of THIS ticket's scope (the
relay leg lives in ticket 001, independently, and is conditional there
too).

The robot is mounted on a stand, wheels off the ground — safe to power
motors and spin wheels freely per the standing bench-testing convention.

## Scope

1. Port `goto_otos.py`: replace its own arc-solving/replan loop with
   sending `GO_TO` commands (`NezhaProtocol`, following whatever thin
   wrapper ticket 007 adds to the host if that ticket has landed first —
   if not, send `GO_TO` directly via the raw command plane for this
   ticket's purposes and note the temporary duplication). Keep the
   script's existing CLI shape (`W`/`R` frame args, camera-seed-once
   pattern) — only the loop internals change.
2. **Stand smoke test** (this ticket's actual acceptance gate, scoped to
   what direct-serial testing can show — no camera, no playfield this
   session):
   - Deploy this sprint's firmware to `tovez` (by UID, as above).
   - Confirm sensors alive: OTOS reports plausible, changing position on
     a small commanded arc; encoders increment in the expected direction.
   - Send a `GO_TO` (ROBOT frame, a modest target ahead — e.g. 300mm
     forward) over direct serial; confirm: an enqueue ack, encoders
     climbing roughly in proportion to the commanded arc, and a SINGLE
     completion ack when the goto lands (watch telemetry for the whole
     run — zero spurious acks per Landmine 1, ticket 004).
   - Send a `GO_TO` requiring a stop-then-pivot (a target well behind the
     robot) and confirm the sequence completes without a fault.
   - Send a second `GO_TO` while the first is still in flight (replace
     semantics) and confirm the robot smoothly re-targets rather than
     stopping and restarting.
3. **Explicitly flagged, not attempted this session** (equipment
   unavailable): the full camera-supervised playfield A/B against the
   host `gotoWorld` loop (arrival error at camera truth, per-boundary
   minimum wheel speed as the "never slows" metric) that the linked
   issue's own Verification section describes, and any relay-transport
   leg. Record in this ticket's Completion Notes that these remain owed —
   do not silently drop them; file or point to a follow-up
   `clasi/issues/later/` entry if none already covers it.

## Acceptance Criteria

- [ ] `goto_otos.py` sends `GO_TO` commands instead of driving its own
      arc-solve loop; CLI usage (`W`/`R` args) unchanged for the caller.
- [ ] `tovez` confirmed by UID in `mbdeploy list` before any hardware
      command; port taken fresh from that session's listing.
- [ ] Firmware deployed to `tovez` via `mbdeploy deploy --hex`/`--build`
      targeting the UID above, never a default target.
- [ ] Sensors alive: OTOS position/velocity change plausibly under a
      commanded arc; encoders on both wheels increment in the expected
      direction.
- [ ] A `GO_TO` (robot-frame, modest forward target) over direct serial
      produces: enqueue ack, encoders climbing, exactly one completion
      ack, zero spurious acks observed in telemetry across the whole run.
- [ ] A target-behind `GO_TO` exercises stop-then-pivot-then-arc on real
      hardware without faulting.
- [ ] A replacement `GO_TO` sent mid-goto re-targets smoothly (no
      observable stop-and-restart) rather than completing the first
      target before accepting the second.
- [ ] Completion Notes explicitly record: the full playfield A/B and any
      relay-transport leg were NOT attempted this session (equipment
      unavailable), with a pointer to where that follow-up is tracked.

## Testing

- **Existing tests to run**: none — this ticket is itself the hardware
  verification step; the sim suite (ticket 005) is what gates correctness
  going in.
- **New tests to write**: none in the pytest sense — `goto_otos.py`'s own
  port is the deliverable, exercised manually per the Scope section above.
- **Verification command** (hardware, direct serial — confirm the port
  fresh each session):
  ```
  uv run mbdeploy list
  uv run mbdeploy deploy --hex ./MICROBIT.hex 9906360200052820a8fdb5e413abb276000000006e052820
  uv run python src/tests/bench/goto_otos.py R 300 0 --port <tovez's current port>
  ```
