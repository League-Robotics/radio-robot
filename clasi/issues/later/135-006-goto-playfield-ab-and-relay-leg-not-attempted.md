---
status: pending
filed: 2026-08-06
filed_by: programmer (sprint 135 ticket 006)
related:
- replaceable-go-to-moves-in-the-motion-library.md
tickets:
- '135-006'
sprint: '135'
---

# GO_TO playfield A/B and relay-transport leg — not attempted (135-006)

## Description

Sprint 135 ticket 006 ("Bench/HITL: port goto_otos.py to GO_TO, stand
verification on tovez") explicitly scoped two verification legs OUT of its
own session, for lack of equipment, and its own Scope/Acceptance-Criteria
sections require filing (or pointing to) a follow-up here rather than
silently dropping them. Neither has ever been run against the ported
`GO_TO`-emitting `goto_otos.py` (`src/tests/bench/goto_otos.py`).

1. **The camera-supervised playfield A/B** the linked issue's own
   Verification section describes: run the ported `goto_otos.py`'s `GO_TO`
   waypoints on the real playfield (not the stand), score arrival error
   against camera truth (AprilTag, not OTOS/encoder self-report), and
   compare against the pre-port host `gotoWorld` loop on the same metric —
   including the per-boundary minimum-wheel-speed "never slows" check.
   Ticket 006's own stand session could not do this: OTOS cannot register
   real chassis translation while the robot is mounted on a stand with
   wheels off the ground (measured directly this session — see ticket
   006's Completion Notes: OTOS position/heading stayed frozen at the
   seeded value across a 45s commanded `GO_TO`, encoders climbing the
   whole time), so arrival-quality can only be judged on the actual
   playfield, under camera supervision, per `.claude/rules/
   playfield-testing.md`.
2. **A `GO_TO` smoke test over the radio-relay transport** (the `getez`
   dongle, `mbdeploy list`'s RADIOBRIDGE role) — ticket 006 verified the
   ported script and the wire mechanics (enqueue ack, encoders climbing,
   exactly one completion ack, zero spurious ack-ring entries after a
   135-006 fix to the ack-ring dedup logic, clean mid-goto replacement)
   over DIRECT SERIAL only. The relay leg needs its own session — same
   acceptance shape, different transport, and the relay's own async
   EVT/STREAM drop behavior (`.claude/rules/hardware-bench-testing.md`,
   `relay-transport-and-stand-vs-floor` memory) is worth re-confirming
   doesn't disturb the ack ring specifically.

## What to do

- Bring up the playfield camera (`.claude/rules/playfield-testing.md`:
  lights, `open_camera`/`create_playfield`/`start_detection`), run
  `goto_otos.py`'s WORLD- and ROBOT-frame waypoints with per-boundary
  camera fixes, and produce the same commanded-vs-achieved table the sim/
  bench gates already use, including minimum wheel speed per leg.
- Repeat the same waypoint set through the `getez` relay
  (`SerialConnection`'s auto-detected relay mode, already wired into
  `goto_otos.py`'s `Robot.__init__()` by 135-006 — no code change
  expected, just `--port <relay's own port>`), watching the SAME ack-ring/
  encoder acceptance shape ticket 006 verified over direct serial.
- Not yet assigned to a sprint.
