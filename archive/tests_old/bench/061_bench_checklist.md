# Sprint 061 — Planner-Only Bench Checklist (Tovez)

Sprint 061 absorbed `MotionController` entirely into `Planner` and deleted all
`drive2`/`mc2`/`bvc2`/`MotionController2` scaffolding. Every motion mode now
routes through a single `Planner` instance. This checklist verifies the physical
tovez robot works correctly through that unified path.

Firmware built: `MICROBIT.hex` (version `0.20260630.32`, 952 KB, built 2026-06-30)
Robot: `tovez` (serial last-6: `f137c0`)
Active robot config: `data/robots/active_robot.json` -> `data/robots/tovez.json`

## Setup

1. Flash `MICROBIT.hex` to the tovez micro:bit.
   - Use `mbdeploy deploy <uid>` (do NOT use `--build` — it triggers a stale
     incremental build; the repo-root `MICROBIT.hex` is the clean artifact).
   - Drag-and-drop to the MICROBIT drive also works.
2. Place tovez on the stand (wheels clear of the surface) for Checks 1–5.
   - Check 6 (GOTO drive) requires the playfield. See the safety gate below.
3. Open the relay connection:
   - Assert DTR on the relay serial port before opening.
   - Auto-detect mode (`SerialConnection(port)` with no `mode=` arg) will run
     `!GO` to enter the data plane. Do NOT use `mode="direct"` — it bypasses
     the relay and cannot reach the robot.
   - Over the relay, **EVT frames are dropped** (async transport); use SNAP
     polling (`SNAP` + read for `TLM`) to detect mode and confirm completion.
     Direct USB-CDC connections receive EVT frames reliably.
4. Send `STREAM 200` to start telemetry at 5 Hz.
   - Over the relay, use `SNAP` polls instead — relay drops STREAM frames.
   - Confirm TLM frames contain `mode=I` at idle.

## Mode Character Reference

| DriveMode | TLM `mode=` char | When active |
|-----------|-----------------|-------------|
| IDLE      | `I`             | Robot stopped, no active command |
| VELOCITY  | `V`             | `VW` or `R` running |
| DISTANCE  | `D`             | `D`, `T`, `TURN`, or `RT` running |
| GO_TO     | `G`             | `G` go-to running |
| STREAMING | `S`             | Watchdog keepalive active (VW streaming) |

All modes now report through `Planner.mode()` → `RobotTelemetry`.

---

## Check 1 — Idle Telemetry

**Goal:** Confirm the robot is alive, reporting `mode=I`, and the Planner is
correctly wired to the TLM path after the sprint-061 refactor.

| Step | Command / Action | Expected | Done |
|------|-----------------|----------|------|
| 1a | `STREAM 200` | TLM frames begin at ~5 Hz | [ ] |
| 1b | Observe `mode=` field | `mode=I` (IDLE) | [ ] |
| 1c | Observe `enc=` field | `enc=0,0` (or near-zero) | [ ] |
| 1d | No errors in TLM for 5 s | No `ERR` or `EVT safety_stop` | [ ] |

**Pass:** TLM arrives at ~5 Hz with `mode=I`.
**Fail:** No TLM, `mode` shows error state, or `EVT safety_stop` fires immediately.

---

## Check 2 — VW Straight Forward + Stop

**Goal:** Confirm velocity-mode drive via Planner. On stand (wheels in air).

| Step | Command | Expected | Done |
|------|---------|----------|------|
| 2a | `VW 200 0` | Reply: `OK vw`; `mode=V` in TLM | [ ] |
| 2b | Observe TLM 2–3 s | `mode=V`; `enc` both wheels spinning; `twist` ~`200,0` | [ ] |
| 2c | `X` (stop) | Reply: `OK stop`; `mode=I`; motors audibly stop | [ ] |
| 2d | Observe TLM after stop | `mode=I`; `enc` not changing | [ ] |

**Pass:** Motors respond to `VW 200 0`, spin freely, stop cleanly on `X`.
**Fail (file issue):** `enc=0,0` while motors are audibly spinning; `mode` stuck at `I`; motors do not stop on `X`.

---

## Check 3 — TURN (Absolute Heading, Centidegrees)

**Goal:** Confirm TURN command routes through Planner, uses DISTANCE mode.

**On stand — acceptable; on stand OTOS fused heading will not track rotation.**
**Mode progression and EVT confirm Planner wiring; heading closure needs playfield.**

| Step | Command | Expected | Done |
|------|---------|----------|------|
| 3a | `TURN 9000` (90.00°) | Reply: `OK turn`; `mode=D` during turn | [ ] |
| 3b | Poll `SNAP` until `mode=I` | `mode` returns to `I` after turn completes | [ ] |
| 3c | Over USB-direct: observe `EVT done TURN` | `EVT done TURN` received | [ ] |
| 3d | Robot audibly stops after EVT | Motors stop; no runaway | [ ] |

**Note (relay):** Over the relay `EVT done TURN` is an async frame and may be
dropped. Confirm completion by polling `SNAP` and watching `mode` return to `I`.

**Pass:** `mode=D` during spin; `mode=I` after; `EVT done TURN` over direct USB.
**Fail (file issue):** `mode` stuck at `I` despite `TURN 9000`; robot spins without stopping (runaway).

---

## Check 4 — D (Distance Drive)

**Goal:** Confirm timed-distance drive. On stand.

| Step | Command | Expected | Done |
|------|---------|----------|------|
| 4a | `D 500 500 300` (500 mm/s, 500 mm, 300 mm decel) | Reply: `OK drive`; `mode=D` | [ ] |
| 4b | Poll `SNAP` until `mode=I` | `mode` returns to `I` after ~1 s | [ ] |
| 4c | Over USB-direct: observe `EVT done D` | `EVT done D` received | [ ] |
| 4d | Encoder count increased from start | `enc` shows accumulated ticks | [ ] |

**Pass:** `mode=D` during drive; `mode=I` after completion; `EVT done D` over direct USB.
**Fail (file issue):** `mode` stuck at `I`; robot does not stop; `enc` never changes.

---

## Check 5 — RT (Relative Rotation, Centidegrees)

**Goal:** Confirm relative-spin command routes through Planner.

**On stand — rotation observable via `enc`, though OTOS fused heading unreliable off-surface.**

| Step | Command | Expected | Done |
|------|---------|----------|------|
| 5a | `RT 18000` (180.00°) | Reply: `OK rt`; `mode=D` during spin | [ ] |
| 5b | Poll `SNAP` until `mode=I` | `mode` returns to `I` after spin | [ ] |
| 5c | Over USB-direct: observe `EVT done RT` | `EVT done RT` received | [ ] |
| 5d | Differential `enc` reflects expected arc | `enc` left/right differ by ~expected arc | [ ] |

**Pass:** `mode=D` during spin; `mode=I` after; `EVT done RT` over direct USB.
**Fail (file issue):** `mode` stuck at `I`; robot spins without stopping.

---

## Check 6 — G (Go-To, Robot-Relative)

**SAFETY GATE: Do NOT execute this check unless:**
- Robot is on the **playfield** (not the stand, not a table without edge stops).
- The AprilCam camera is active and reporting the robot's tag (ID 100).
- Robot pose has been read and verified within the geofence.
- Per knowledge notes `vision-geofence-before-driving.md` and `playfield-not-floor.md`:
  never blind-drive on the playfield; read camera + geofence first.

If camera is not set up, **skip this check and mark it deferred**.

| Step | Command | Expected | Done |
|------|---------|----------|------|
| 6a | Read camera pose, verify tag 100 visible | Tag 100 detected; pose reported | [ ] |
| 6b | Confirm robot within geofence boundary | Robot position safe to drive | [ ] |
| 6c | `G 200 0 200` (forward 200 mm, 0 lateral, 200 mm/s) | Reply: `OK goto`; `mode=G` | [ ] |
| 6d | Poll `SNAP` during travel | `mode=G`; pose x advancing toward goal | [ ] |
| 6e | Goal reached | `mode=I`; robot within ~25 mm of target | [ ] |
| 6f | Over USB-direct: observe `EVT done G` | `EVT done G` received | [ ] |

**Pass:** Robot reaches goal (~25 mm); `mode=I` after; `EVT done G` over direct USB.
**Fail (file issue):** Robot drives past goal (no stop); `mode` never reaches `G`; robot approaches playfield edge.

---

## Check 7 — SAFE One-Shot Disable + Re-Arm

**Goal:** Confirm safety re-arm fires correctly after Planner refactor.
The `SAFE off` one-shot is handled in `PlannerBegin` — confirm it still re-arms
on the next `begin` call after the sprint-061 changes.

**On stand.**

| Step | Command | Expected | Done |
|------|---------|----------|------|
| 7a | `SAFE off` | Reply: `OK safety off timeout=<ms>` | [ ] |
| 7b | `VW 200 0` (immediately) | Reply: `OK vw`; motion begins; `mode=V` | [ ] |
| 7c | Observe `EVT safety re-armed` in stream | `EVT safety re-armed` received | [ ] |
| 7d | `X` (stop) | `mode=I`; motors stop | [ ] |
| 7e | Send `SAFE` (query) | Reply: `OK safety on timeout=<ms>` (re-armed) | [ ] |

**Note (relay):** `EVT safety re-armed` is async; over the relay, confirm via
`SNAP` that `mode=V` when `VW` is active, and `mode=I` after `X`. Query `SAFE`
after to confirm re-arm.

**Pass:** Motion starts immediately after `SAFE off`; safety re-armed on next begin.
**Fail (file issue):** `VW` rejected despite `SAFE off`; safety not re-armed after begin; `EVT safety re-armed` never received.

---

## Check 8 — TLM Mode-Char Confirmation

**Goal:** Confirm `Planner.mode()` correctly populates the TLM `mode=` field
through the `RobotTelemetry` path for all active modes.

| Motion command | Expected TLM `mode=` | Done |
|----------------|---------------------|------|
| Idle (no command) | `I` | [ ] |
| `VW 100 0` (while running) | `V` | [ ] |
| `D 300 300 100` (while running) | `D` | [ ] |
| `G 100 0 100` (while running; on playfield) | `G` | [ ] |
| After any `X` or completion | `I` | [ ] |

**Pass:** TLM `mode=` char matches the active Planner mode for every command.
**Fail (file issue):** Any mismatch between active command type and reported `mode=` char.

---

## Failure Criteria — File a GitHub Issue If Any Observed

- Motors do not respond to `VW`, `D`, `TURN`, `RT`, or `G`.
- `enc=0,0` while motors are audibly spinning (encoder disconnect or wrong firmware).
- `mode` char never advances past `I` despite a drive command being accepted (`OK`).
- Robot spins or drives without stopping (runaway — no stop condition firing).
- `EVT done TURN/D/RT/G` not received over direct USB.
- Safety not re-armed after `SAFE off` + `VW`.
- TLM `mode=` char does not match the active drive mode.
- Build version in `VER` response does not match `0.20260630.32`.

---

## Reporting

After the bench run, report results as a comment on the sprint-061 issue.

- All checks pass: close the sprint via `close_sprint` (or ask the team-lead).
- Any failure: file a bug GitHub issue with the TLM output, the failing check,
  and the specific failure criterion that was triggered.
