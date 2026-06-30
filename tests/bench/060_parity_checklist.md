# Sprint 060 — Ordered-Tick Cutover: Tovez Bench Parity Checklist

Sprint 060 completed the ordered-tick cutover: `USE_ORDERED_TICK` is gone (it is now
the only path), and the legacy `Drive2`/`bvc2`/`MotionController2` names have been
deleted and replaced with clean names `Drive`/`bvc`/`MotionController`. This checklist
verifies the physical tovez robot behaves identically to the pre-cutover legacy build.

Firmware built: `MICROBIT.hex` (version `v0.20260630.24`, 930 KB)
Robot: `tovez` (serial last-6: `f137c0`, device name: `tovez`)
Active robot config: `data/robots/active_robot.json` -> `data/robots/tovez.json`

## Setup

1. Flash `MICROBIT.hex` to the tovez micro:bit (serial last-6: `f137c0`).
   - Use `mbdeploy deploy` or drag-and-drop `MICROBIT.hex` to the MICROBIT drive.
2. Place tovez on the stand (or playfield for Check 4 only).
3. Open the relay connection:
   - Assert DTR on the relay serial port.
   - Send `!GO` to enter the data plane (per knowledge note `relay-transport-and-stand-vs-floor.md`).
   - Do NOT use the `>` prefix — data-plane commands have no prefix.
4. Start telemetry stream at 200 ms: send `STREAM 200`
   - Confirm TLM frames begin arriving at ~5 Hz.

## Check 1 — IDLE Telemetry Structure

**Goal:** Confirm the robot is alive and reporting clean IDLE state.

1. After `STREAM 200`, observe several TLM frames.
2. Expected fields:
   - `mode=I` (IDLE)
   - `enc=0,0` (or near-zero if wheel jitter)
   - `pose=0,0,0` (or near-zero if OTOS hasn't initialized — this is acceptable)
3. Pass criterion: TLM arrives regularly at ~5 Hz with `mode=I`.
4. Fail criterion: No TLM received, or `mode` shows an error state.

## Check 2 — VW (Body-Velocity) Parity

**Goal:** Confirm forward drive via the ordered-tick `Drive` subsystem behaves
identically to the pre-cutover legacy build.

**On stand (wheels spin in air — safe).**

1. Send `VW 100 0` (forward at 100 mm/s, no rotation).
2. Observe TLM for 2–3 seconds:
   - `mode=V` (velocity mode)
   - `enc` values increasing (both left and right)
   - `twist` ~`100,0` (forward, zero angular)
3. Send `X` (stop command).
4. Observe TLM:
   - `mode=I` (returns to IDLE)
   - `enc` stops changing
   - Motors audibly stop.
5. Pass criterion: Same qualitative behavior as pre-cutover build.
6. Fail criteria (file a bug if observed):
   - Motors do not respond to `VW`.
   - `enc=0,0` while motors are audibly spinning (encoder disconnect).
   - `mode` never advances past `I` despite the `VW 100 0` command.

## Check 3 — TURN Parity

**Goal:** Confirm heading-hold turn via the ordered-tick path (`Planner` + `Drive`).

**On stand (wheels spin in air — safe).**

1. Send `TURN 90` (turn 90 degrees clockwise).
2. Observe TLM:
   - `mode=D` or `mode=G` (drive/goto mode during turn)
   - Pose heading (`pose` third field) changes by ~90 degrees.
3. Confirm the robot completes the turn and mode returns to `I`.
4. Pass criterion: Heading changes by approximately 90 degrees; mode returns to IDLE.
5. Fail criteria (file a bug if observed):
   - Robot oscillates or behaves erratically vs. legacy build.
   - `mode` never advances past `I` despite the `TURN 90` command.
   - Robot turns but does not stop (runaway).

## Check 4 — GOTO Parity (optional; playfield only)

**SAFETY GATE: Do NOT execute this check unless the robot is on the playfield
with the AprilCam camera active and geofence verified.**
(Per knowledge notes `vision-geofence-before-driving.md` and `playfield-not-floor.md`.)

If camera is not set up, skip this check and mark it deferred.

1. Start the AprilCam daemon and verify the robot's tag (ID 100) is detected.
2. Read pose from camera; confirm robot is within the geofence.
3. Send `GOTO 0 200` (move 200 mm forward from current position).
4. Observe TLM:
   - `mode=G` (GOTO mode)
   - Pose x/y advances toward the goal.
   - Mode returns to `I` at goal completion.
5. Pass criterion: Robot reaches the goal (within ~20 mm); mode returns to IDLE.
6. Fail criteria (file a bug if observed):
   - Robot drives off the playfield edge.
   - `mode` never reaches `G`.
   - Pose does not advance toward goal.

## Failure Criteria (file a GitHub issue if any are observed)

- Motors do not respond to `VW`/`TURN`/`GOTO`.
- TLM shows `enc=0,0` while motors are audibly spinning.
- Robot oscillates or behaves erratically vs. the pre-cutover legacy build.
- `mode` never advances past `I` despite a drive command.
- Build version reported in TLM does not match `v0.20260630.24`.

## Reporting

After the bench run, report results as a comment on GitHub issue for sprint 060.
If all checks pass: close the issue. If a failure is observed: file a bug issue
with the TLM output and the specific failure criterion that was triggered.
