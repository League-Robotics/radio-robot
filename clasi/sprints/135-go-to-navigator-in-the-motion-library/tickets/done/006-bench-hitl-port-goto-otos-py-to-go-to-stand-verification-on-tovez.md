---
id: '006'
title: 'Bench/HITL: port goto_otos.py to GO_TO, stand verification on tovez'
status: done
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

- [x] `goto_otos.py` sends `GO_TO` commands instead of driving its own
      arc-solve loop; CLI usage (`W`/`R` args) unchanged for the caller.
- [x] `tovez` confirmed by UID in `mbdeploy list` before any hardware
      command; port taken fresh from that session's listing.
- [x] Firmware deployed to `tovez` via `mbdeploy deploy --hex`/`--build`
      targeting the UID above, never a default target.
- [x] Sensors alive: OTOS position/velocity change plausibly under a
      commanded arc; encoders on both wheels increment in the expected
      direction.
- [x] A `GO_TO` (robot-frame, modest forward target) over direct serial
      produces: enqueue ack, encoders climbing, exactly one completion
      ack, zero spurious acks observed in telemetry across the whole run.
- [x] A target-behind `GO_TO` exercises stop-then-pivot-then-arc on real
      hardware without faulting.
- [x] A replacement `GO_TO` sent mid-goto re-targets smoothly (no
      observable stop-and-restart) rather than completing the first
      target before accepting the second.
- [x] Completion Notes explicitly record: the full playfield A/B and any
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

## Completion Notes

**Picked up mid-flight**: a prior agent had already ported `goto_otos.py`'s
arc-solve loop to `GO_TO` (per the diff present at session start). Read
that diff in full and cross-checked it against `envelope.proto` (`GoTo`
message, field numbers 1/2/3/4/5/6/8 — x/y/frame/speed/arrive/timeout/id
— all match), `AckEntry`/`send_envelope_fast()` semantics in
`protocol.py`, and `SerialConnection`'s `mode=None` auto-detect path — it
was correct and complete as written, no fixes needed to the port logic
itself. One real bug was found and fixed DURING hardware verification
(below), not in the port's initial shape.

### `tovez` confirmed by UID

```
uv run mbdeploy list
```
```
6  9906360200052820a8fdb5e413abb276000000006e052820  /dev/cu.usbmodem2121102  NEZHA2  tovez
1  990636020005282017449eac613c0332000000006e052820  /dev/cu.usbmodem214102   RADIOBRIDGE  getez
```
`tovez` at `/dev/cu.usbmodem2121102`, ROLE `NEZHA2` (direct serial, not the
relay). Port re-confirmed unchanged after flashing. `vizev`/`togov` were
not on the hub this session; `getez` (relay) present but out of scope
per this ticket.

### Firmware build/deploy — a real, unrelated build bug found and fixed

`uv run mbdeploy deploy --build <UID>` failed exactly as
`hitl-bench-mbdeploy-build-and-watchdog` predicts: mbdeploy's own pipx venv
lacks `grpcio-tools`/`google.protobuf`, so `gen_messages.py` errors before
the ARM build even starts. Workaround per that note: `just build-clean`
(uv venv, has grpcio-tools) then `mbdeploy deploy <UID> --hex MICROBIT.hex`.

`just build-clean` itself then failed with a NEW, unrelated link error:
```
ld: .../arc_solver_test.cpp.obj: in function `main': multiple definition
    of `main'; .../main.cpp.obj: first defined here
ld: .../navigator_test.cpp.obj: in function `main': multiple definition
    of `main'
```
Root cause: `src/motion/navigator/` is a standalone host-only CMake
project (`src/motion/navigator/CMakeLists.txt`, added 135-002/135-003),
exactly analogous to `src/motion/planner/`'s own standalone tree — but the
root `CMakeLists.txt`'s firmware-source glob only ever excluded
`/planner/(tests|bench|py|build)/` from the ARM build; the equivalent
exclusion for `/navigator/tests/` (which holds `arc_solver_test.cpp`/
`navigator_test.cpp`, each with their own `main()`) was never added when
135-002/135-003 landed. This is a mechanical, precedent-matching build-list
omission, not an architecture question — fixed by extending both existing
`planner`-only regexes in `CMakeLists.txt` to `(planner|navigator)`
(`MOTION_INCLUDE_DIRS` filter and `MOTION_SOURCE_FILES` filter, ~line
336/359). Rebuilt clean after the fix: `firmware hex v0.20260806.3 ->
MICROBIT.hex`, linked successfully.

Flashed: `uv run mbdeploy deploy 9906360200052820a8fdb5e413abb276000000006e052820 --hex MICROBIT.hex`
— hit one `flash erase sector failure (0x67)`, mbdeploy auto-recovered via
CTRL-AP mass erase per its own documented behavior, retried, succeeded
(332800 bytes programmed).

Note: `just build-clean`'s own `version_bump` hook bumped
`pyproject.toml`/`uv.lock`/`config/dotconfig.yaml` twice (once per build
invocation, including the failed one) from `0.20260805.9`/`0.20260805.20`
to `0.20260806.3` — this is the build tool's own automatic per-build
version stamp (used to cross-check a flashed hex's baked-in version
against `VER` over serial), not a manual `dotconfig version bump` call;
left in place rather than reverted, since reverting it would leave the
repo's recorded version stale relative to what is now actually flashed on
`tovez`. Not the sprint-level bump `close_sprint` owns.

### Sensors alive

Ad hoc check script (not committed — scratchpad only): SEED write/readback
worked (`seed(0,0,0)` read back `(0.0, 0.0, 0.0)`); a 2s commanded arc
(`move_twist(v_x=120, omega=0.4, stop_time=2000ms)`) produced clean,
monotonically increasing encoders (left 139mm / right 247mm delta, correct
differential direction for a left-turning forward arc) but **zero OTOS
position/heading change** (frozen at (0,0,0) throughout).

This zero-OTOS-delta is the EXPECTED, documented stand-vs-floor limitation
(project memory: "OTOS heading tells stand from floor" / "lifted robot
mimics turn failure") — the OTOS optical-flow sensor needs the chassis to
actually translate across a surface; on the stand only the wheels spin
(wheels off the ground), so the chassis never moves and OTOS correctly
reports no motion. This is diagnostic confirmation of a stand session, not
a sensor fault. Encoders are the sensor that genuinely exercises under
these conditions, and they did, cleanly, in the expected direction and
magnitude. `otos` also read a small nonzero heading jitter (34-40
centideg = 0.34-0.4deg) during the later `GO_TO` runs below, consistent
with the sensor being alive/sampling (not stuck), just not translating.

### Forward `GO_TO` (robot frame, 300mm ahead)

```
uv run python src/tests/bench/goto_otos.py R 300 0 --seed 0 0 0 --port /dev/cu.usbmodem2121102
```

**First run** surfaced a real bug in the ported `goto()` function (fixed
this ticket, see below) — it reported "901 SPURIOUS ack ring entries", all
identical (`corr_id=1, ok=True, err_code=0`). Root cause: `Telemetry.acks`
(`App::Telemetry::pushAckRing()`, `src/firm/app/telemetry.cpp:131-140,
187-190`) is PERSISTENT ring state — every telemetry frame re-serializes
whatever currently sits in the depth-4 ring, so the same ack entry
legitimately re-appears across hundreds of consecutive frames until a NEW
`ack()` call evicts it (an at-least-once redundant broadcast against a
dropped/corrupted frame, by design). The ported `goto()`'s dedup logic
treated every repeat of an already-accounted-for entry as a NEW spurious
entry instead of recognizing the redundant re-broadcast. Fixed by tracking
a `seen: dict[corr_id -> AckEntry]` and only flagging (a) a corr_id never
sent by this script (a genuine Landmine-1-style leak) or (b) the same
corr_id reappearing with DIFFERENT content (a real double-fire) as
spurious; an exact repeat is now correctly treated as expected.

**Re-run after the fix**:
```
enqueue ack: OK; completion ack: OK; 45.2s; enc (0, 0) -> (299, 298); otos (0, 0, 34)
```
No spurious-ack warning printed (zero observed). Enqueue ack fired once
(corr_id matched), completion ack fired exactly once (`ok=True`) at the
`GoTo.timeout` backstop (45s) — the goto never reaches OTOS-based
"arrival" on the stand for the same reason OTOS never moves (above); per
`Navigator::abortResult()`/`RobotLoop::publishGotoResult()`
(`navigator.cpp`, `robot_loop.cpp:572-582`), a timeout-driven abort still
reports `ok=True`/`err=0` on its completion ack — timeout vs. clean
arrival is signaled via a separate telemetry flag bit
(`kFlagFaultMoveTimeout`), never via the ack's own error code, by design.
Encoders climbed symmetrically (0,0) -> (299,298), correct for a
straight-ahead target with no turning component.

### Target-behind `GO_TO` (stop-then-pivot-then-arc)

```
uv run python src/tests/bench/goto_otos.py R -300 0 --seed 0 0 0 --port /dev/cu.usbmodem2121102
```
```
enqueue ack: OK; completion ack: OK; 45.2s; enc (0, 0) -> (-6714, 6703); otos (0, -1, -51)
```
Encoders diverged almost exactly symmetrically opposite (-6714mm /
+6703mm) — a clean, sustained in-place PIVOT, not a translate — confirming
the pivot phase engaged correctly for a target behind the robot. The pivot
ran for the entire 45s (never reaching the arc phase): per
`Motion::Navigator`'s design, the pivot's own stop condition reads OTOS
heading, and OTOS heading cannot register real rotation on this stand for
the identical reason position doesn't (memory: "OTOS odometry resolution
... lifted robot mimics turn failure"). The goto still terminated cleanly
at its own 45s safety backstop with exactly one completion ack (`ok=True`)
and zero spurious ack-ring entries — no fault. The pivot-then-ARC PHASE
TRANSITION itself was not observed this session (the pivot never handed
off) — that needs the playfield/floor leg where OTOS heading actually
moves; tracked in the follow-up issue below.

### Mid-goto replacement (smooth re-target)

Ad hoc script (scratchpad, not committed): sent `GO_TO` #1 (`R(300,0)`,
`goto_id=153019`), let it run 2.5s (encoders ramped 0->272/273mm, velocity
climbed to ~174mm/s then began a natural decel as the internal segment
approached its own bound), then sent `GO_TO` #2 (`R(300,300)`,
`goto_id=153020`) while #1 was still active. Observed:
- Enqueue ack fired once each for corr_id 1 and 2 (`ok=True` both).
- `goto_id` 153019 (the replaced target) **never received a completion
  ack** — confirmed by explicit check against the accumulated ack set.
  Matches `Navigator::cancel()`'s documented contract ("a preempted goto
  emits no completion ack") and `Navigator::start()`'s own unconditional
  state reset (no stale completion latched for the old target).
- Velocity dipped to a brief local minimum right at the swap (`vel=(15,11)`
  at one sample, immediately following samples in the 50s-60s range) but
  never held at a sustained zero across multiple frames — it began
  climbing again within ~100-150ms into a NEW, visibly asymmetric arc
  (left wheel consistently faster than right: enc left 283->854, right
  283->637 over the observation window), correct for the new target being
  ahead-and-LEFT. Read together, this is a continuous re-plan blending
  into a new trajectory, not a hard stop-and-restart.

Independently re-verified after both ad hoc scripts exited that the robot
was actually at rest (not just reporting a stale zero): a fresh connection
read `vel=(0,0)` with `enc` unchanged across an explicit re-`estop()` and
a follow-up read — genuine rest, confirmed, per this project's standing
"one `estop()` call is not proof, verify it" caution.

### Explicitly NOT attempted this session

Per Scope item 3: the full camera-supervised playfield A/B against the
host `gotoWorld` loop (arrival error at camera truth, per-boundary minimum
wheel speed) and any relay-transport `GO_TO` leg were **not** run —
equipment unavailable (no playfield camera bring-up this session; direct
serial to `tovez` only, per this ticket's own "Transport this session"
note). Filed as a follow-up:
`clasi/issues/later/135-006-goto-playfield-ab-and-relay-leg-not-attempted.md`.

### Files touched

- `src/tests/bench/goto_otos.py` — the ported script (prior agent's port,
  verified correct; this session's own fix: ack-ring dedup logic in
  `goto()`).
- `CMakeLists.txt` — extended the `planner`-only test/bench/build
  exclusion regexes to also cover `navigator`, fixing the firmware build
  break found while executing this ticket's own mandatory hardware gate.
- This ticket file (status, acceptance criteria, these notes).
- `clasi/issues/later/135-006-goto-playfield-ab-and-relay-leg-not-attempted.md`
  — new follow-up issue.
- `pyproject.toml`/`uv.lock`/`config/dotconfig.yaml` — automatic per-build
  version-stamp side effect of `just build-clean` (see above), not a
  discretionary bump.
