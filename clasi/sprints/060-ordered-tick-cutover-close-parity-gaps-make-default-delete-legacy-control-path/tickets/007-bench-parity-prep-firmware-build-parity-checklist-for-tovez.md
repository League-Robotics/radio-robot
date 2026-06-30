---
id: '007'
title: 'Bench-parity prep: firmware build + parity checklist for tovez'
status: open
use-cases:
- SUC-007
depends-on:
- '006'
github-issue: ''
issue: make-ordered-tick-the-default-close-parity-gaps.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Bench-parity prep: firmware build + parity checklist for tovez

## Description

This is a HOST-SIDE-SAFE ticket. No autonomous hardware control. The ticket:
1. Builds the firmware with the fully-cutover ordered-tick path and renamed subsystems.
2. Verifies the build succeeds and produces a valid MICROBIT.hex.
3. Produces a documented bench checklist for the stakeholder to run on the tovez robot.

The physical bench run (flashing + executing on tovez) is performed by the
stakeholder/team-lead after the ticket is done.

### Firmware build

Use `python3 build.py --clean` (per the `stale-incremental-build-on-volumes.md`
knowledge note — incremental builds go stale silently on `/Volumes`; always do a
clean build before a HITL flash). Verify the build artifact exists and the hex
size is plausible (same order of magnitude as previous sprints).

The firmware build must produce a clean build with no warnings related to the
deleted or renamed identifiers.

### Bench checklist (to produce as a comment in this ticket's commit or as a file)

The checklist documents the following sequences for the stakeholder to run on tovez
with the new firmware:

**Setup:**
- Flash `MICROBIT.hex` to tovez (micro:bit ID documented in `active_robot.json`).
- Connect via the relay (DTR asserted, `!GO` to enter data plane, per knowledge note).
- Start STREAM at 200ms: `STREAM 200`.

**Check 1 — IDLE TLM structure:**
- Confirm TLM frame arrives at ~5 Hz with `mode=I`.
- Confirm `enc=0,0` (or near-zero).
- Confirm `pose=0,0,0` (or near-zero if OTOS hasn't initialized).

**Check 2 — VW (body-velocity) parity:**
- Send `VW 100 0` (forward 100 mm/s, no rotation).
- Observe TLM: `mode=V`, `enc` values increasing, `twist` ~100,0.
- After 2s send `X` (stop). Confirm `mode=I`, motors stop.
- Expected: same qualitative behavior as pre-cutover legacy build.

**Check 3 — TURN parity:**
- Send `TURN 90` (turn 90 degrees clockwise).
- Observe TLM: `mode=D` or `mode=G`, pose heading changes by ~90 degrees.
- Confirm completion (mode returns to I).
- Expected: same qualitative behavior as pre-cutover legacy build.

**Check 4 — GOTO parity (optional, floor/playfield-only per vision/geofence rules):**
- Only execute this check if on the playfield with camera verification.
- Send a short goto: `GOTO 0 200` (200mm forward from origin).
- Observe pose advancing, goal completion.
- NOTE: Per knowledge notes `vision-geofence-before-driving.md` and
  `playfield-not-floor.md` — NEVER blind-drive on the playfield without reading the
  camera and geofence first. This check requires camera setup. The stakeholder
  decides whether to include it.

**Failure criteria (file a bug if observed):**
- Motors do not respond to VW/TURN.
- TLM shows `enc=0,0` while motors are spinning.
- Robot oscillates or behaves erratically vs. legacy build.
- `mode` never advances past `I` despite a drive command.

## Acceptance Criteria

- [ ] Firmware builds cleanly: `python3 build.py --clean` succeeds with no errors.
- [ ] MICROBIT.hex exists and is non-trivially sized (confirms no silent build failure).
- [ ] Bench checklist is documented (in this ticket or as a committed file).
- [ ] `uv run python -m pytest` — green except the 2 known-baseline failures (final sprint-close verification).
- [ ] Physical bench run is executed by the stakeholder (not part of ticket-done criteria — stakeholder confirms separately).

## Implementation Plan

### Approach

Host-only work. Build the firmware; document the checklist.

### Commands

```bash
# Confirm active_robot.json identifies the correct robot (tovez)
cat data/robots/active_robot.json

# Clean firmware build (per stale-incremental-build knowledge note)
python3 build.py --clean

# Verify hex exists
ls -lh build/MICROBIT.hex
```

### Files to create (optional)

If the bench checklist is longer than fits in a commit message, create:
`tests/bench/060_parity_checklist.md` with the checklist above.

### Testing plan

1. `uv run python -m pytest` — final full suite green check.
2. Firmware build success is the gate for the agent.
3. Physical bench run is stakeholder-gated.

### Documentation updates

If the bench run reveals behavior differences, the stakeholder opens an issue.
No architecture changes from this ticket.
