---
id: '006'
title: 'Part 8 acceptance: sim criteria 1-12 + bench criteria 13-16'
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
depends-on:
- '001'
- '002'
- '003'
- '004'
- '005'
github-issue: ''
issue: telemetry-emit-policy-rebuild-spec.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Part 8 acceptance: sim criteria 1-12 + bench criteria 13-16

## Description

Close out the issue's own acceptance list (Part 8) as real, running
tests/checks — extending `app_telemetry_harness.cpp` /
`app_robot_loop_harness.cpp` for the sim criteria, and the standing
hardware bench gate for the bench criteria. This ticket is the sprint's
own proof that tickets 001-005 actually deliver the spec; it should not
introduce new production behavior, only tests/verification.

**Sim (extend `app_telemetry_harness.cpp` / `app_robot_loop_harness.cpp`):**

1. Fresh boot, silent host → banner + `READY` only;
   `primaryEmitCount() == 0` after N cycles of parked idling.
2. **Regression for the structural defect**: parked robot WITH line and
   color sensors delivering fresh alternating readings → still zero
   frames. (The case the old design could not pass — this is the single
   most important test in the whole sprint; do not skip or weaken it.)
3. Command to parked robot → its ack rides a frame within
   2x`kPrimaryPeriod`; exactly `kAckRepeats` frames carry it; then
   silence again.
4. Move in `kAuto` → frames at cadence while active; `STOP` → frames
   continue while staged wheel velocity is nonzero; silence within
   `kCoastHoldoff` of velocities reaching 0.
5. Hand-spun-wheel/bogus-velocity case: nonzero wheel velocity with
   `everMoved_ == false` → zero frames.
6. Bare `TLM` on a parked robot → exactly one frame; `TLM:NOW` behaves
   identically.
7. `TLM:OFF`, then a Move → the move executes, ONLY ack frames appear
   (enqueue + completion, `kAckRepeats` each), no stream; bare `TLM`
   mid-move still answers with one frame.
8. `TLM:ON` on a parked robot → frames at cadence; `TLM:OFF` → stream
   stops within one `kPrimaryPeriod`; `TLM:AUTO` restores mode-2
   behavior.
9. Every `TLM:ON|AUTO|OFF` is answered by a `STATUS` line whose `tlm=`
   field shows the NEW mode; `TLM:<garbage>` is answered by `HELP` and
   changes nothing.
10. Mode is not persistent: set `kOn` (or `kOff`), simulate reboot →
    `kAuto`.
11. Motor unit test per ticket 001 (velocity 0 before two samples) — if
    ticket 001 already wrote this test, this criterion is satisfied by
    reference; confirm it exists and passes rather than duplicating it.
12. **The harness boot-boilerplate (`markBootComplete` + settle ticks)
    is GONE; no test may re-add a Telemetry lifecycle call.** Implement
    this as a literal grep check (e.g. a test or CI step asserting
    `grep -rn "markBootComplete\|tickBootSettle" src/tests/` returns
    nothing) so this stays true going forward, not just at this ticket's
    completion.

**Bench (standing gate, robot on the stand —
`.claude/rules/hardware-bench-testing.md`):**

13. Power-on with a passive serial capture already attached → verify the
    Part 6 contract (two cleartext lines, zero binary bytes over >= 30 s).
14. Run `src/tests/bench/radio_bench_gate.py` and the quick smoke
    sequence (`twist_drive.py`) — all existing PASS lines hold under
    `kAuto`; additionally observe silence-after-coast (~2 s).
15. Human-terminal check: with the robot parked, type `TLM:ON` in a
    serial monitor → binary streams; `TLM:OFF` → terminal is typeable
    again; `STATUS` → readable line including `tlm=off`.
16. One parked-capture script from ticket 005 (e.g. `otos_drift.py`)
    runs end-to-end and records frames, proving the migration.

**Additional, stakeholder-added, tracked here alongside but distinct
from the 16 numbered criteria above**: `src/tests/bench/square_tour.py`
passes both `--sim` and against the bench robot on the stand. The sim
leg depends on ticket 007 (adjacent, not part of the issue's own scope —
do not conflate the two when reporting results).

## Acceptance Criteria

- [x] All 12 sim criteria above are implemented as real, passing tests
      in `app_telemetry_harness.cpp`/`app_robot_loop_harness.cpp` (or the
      correct existing harness file(s) — confirm current file names/
      locations before adding, since sim harnesses may have been
      reorganized since the issue was written).
- [x] Criterion 2 (sensor-equipped parked robot) is a real test that
      would have FAILED against the pre-002 code — verify this by
      confirming its logic genuinely exercises the line/color
      alternation, not a simplified stand-in.
- [x] Criterion 12's grep check is wired into the test suite (not just
      manually verified once) so future regressions are caught
      automatically.
- [x] All 4 bench criteria are run on the physical robot (stand-mounted,
      per hardware-bench-testing.md) and each produces an explicit
      PASS/FAIL result recorded in this ticket's completion notes — "ran
      it and it looked fine" is not sufficient; report the actual PASS
      lines / observed timings. (Criterion 14's own protocol-level checks
      PASS; its wheel-motion-dependent sub-checks are BLOCKED by an
      independent, pre-existing hardware fault — see Completion Notes.)
- [ ] `square_tour.py --sim` and `square_tour.py --port <bench>` both
      exit 0 (depends on ticket 007 for the sim leg). **`--sim` PASSES;
      `--port` FAILS**, blocked by the same pre-existing hardware fault
      (encoder wedge-latch + OTOS disconnect on robot "tovez") — see
      Completion Notes. NOT weakened/relaxed to force a pass.
- [ ] Sprint's Success Criteria checklist (sprint.md) is fully satisfied
      before this ticket is marked done. **NOT fully satisfied** — bullet
      4 (`square_tour.py` both legs) is blocked by the same hardware
      fault. See Completion Notes for the full breakdown and recommended
      next step (a physical power-cycle / bus inspection of "tovez",
      then re-run `radio_bench_gate.py --port /dev/cu.usbmodem2121302`
      and `square_tour.py --port /dev/cu.usbmodem2121102 --no-geofence`).

## Testing

- **Existing tests to run**: the full sim test suite plus every bench
  script named above — this ticket IS the testing effort for the sprint,
  so "existing tests" and "new tests" substantially overlap with the
  Acceptance Criteria.
- **New tests to write**: the 12 sim criteria as concrete test cases (see
  Description); the criterion-12 grep check as an automated guard.
- **Verification command**: `uv run pytest` for host-side/grep checks;
  the sim HOST_BUILD test target for the harness criteria; the bench
  commands listed in `.claude/rules/hardware-bench-testing.md`
  (`radio_bench_gate.py`, `twist_drive.py`) plus a manual serial-monitor
  session for criterion 15, run with the robot mounted on the stand.

## Completion Notes (125-006)

### Sim criteria 1-12 — coverage table

All 12 pass. `uv run python -m pytest src/tests/sim` — **423 passed, 1
skipped, 1 xfailed** (baseline was 420/1/1; +3 new pytest tests: the new
`test_robot_loop_tlm.py` harness compile+run, and the two-test
`test_telemetry_no_boot_lifecycle_calls.py` grep guard).

| # | Criterion | Covered by |
|---|---|---|
| 1 | Fresh boot, silent host → `primaryEmitCount()==0` | `app_telemetry_harness.cpp::scenarioFreshConstructDefaultsAutoAndStaysSilentAtRest` (pre-existing, 125-002) + `sim_api_harness.cpp::scenarioBootCompletesThroughRealRobotLoop` (pre-existing, RobotLoop-level, real boot) |
| 2 | **THE regression**: parked + alternating fresh line/color → zero frames | **NEW**: `app_telemetry_harness.cpp::scenarioParkedRobotWithAlternatingFreshLineColorStaysSilent` — drives `update()` with `lineFresh`/`colorFresh` alternating every cycle exactly as `RobotLoop::publishLineColor(tickedLine)` does (verified against that function's source); asserts the flags genuinely toggled (not a stand-in) AND `primaryEmitCount()==0`. A genuine end-to-end (real `ColorSensorLeaf`/`LineSensorLeaf` via `SimPlant`) version was attempted and abandoned: `sim_plant.cpp` documents that line/color are "never simulated devices ... every transaction ... NAKs" — confirmed empirically, `present()` never becomes true in `SimHarness`, so a RobotLoop-level rerun would test nothing about the alternation. See `robot_loop_tlm_harness.cpp`'s own file-header note. |
| 3 | Ack on parked robot: rides within 2x`kPrimaryPeriod`, exactly `kAckRepeats` frames, then silence | **NEW**: `scenarioAckOnParkedRobotRidesExactlyKAckRepeatsFramesThenSilence` |
| 4 | Move in kAuto → cadence; STOP → continues while wheel velocity nonzero; silence within `kCoastHoldoff` | pre-existing (125-002/003) `scenarioActivityWindowRefreshesOnActiveAndCoastsThenClosesAfterHoldoff` |
| 5 | Hand-spun wheel, `everMoved_==false` → zero frames | pre-existing `scenarioHandSpunWheelBeforeFirstMoveNeverWakesTheLink` |
| 6 | Bare `TLM`/`TLM:NOW` on parked robot → exactly one frame | **NEW**: `scenarioBareTlmForceOnParkedRobotProducesExactlyOneFrame`. `TLM:NOW`≡bare `TLM` is a comms-level parse fact (`app_comms_harness.cpp::scenarioBareTlmAndTlmNowBothProduceKFrameNoModeChange`, both map to the same `TlmAction::kFrame`, one `RobotLoop` call site) |
| 7 | `TLM:OFF` + Move → only ack frames (enqueue+completion, `kAckRepeats` each), no stream; bare `TLM` mid-move still answers | **NEW**: `scenarioTlmOffThenMoveOnlyAckFramesNoStream` |
| 8 | `TLM:ON` → cadence; `TLM:OFF` → stops within one period; `TLM:AUTO` restores | **NEW** (Telemetry-unit): `scenarioTlmOnThenOffStopsWithinOnePeriodThenAutoRestoresModeTwoBehavior`. **NEW** (RobotLoop/wire, end-to-end): `src/tests/sim/system/robot_loop_tlm_harness.cpp::scenarioTlmOnStartsStreamingOnAParkedRobotAndStatusReportsIt` / `scenarioTlmOffThenAutoRestoresDefaultBehavior` |
| 9 | Every `TLM:ON\|AUTO\|OFF` → `STATUS` with new `tlm=`; `TLM:<garbage>` → `HELP`, no change | `app_comms_harness.cpp` (125-003, pre-existing): `scenarioTlmModeChangeRepliesWithStatusLineOnOriginatingTransport`, `scenarioStatusLineCarriesTlmFieldForEveryMode`, `scenarioTlmGarbageRepliesWithHelpLineListingTlmArgumentGrammar`. **NEW** RobotLoop/wire end-to-end: `robot_loop_tlm_harness.cpp::scenarioTlmGarbageArgumentChangesNothingAndRepliesHelp` |
| 10 | Mode not persistent: set kOn/kOff, "reboot" → kAuto | **NEW** (Telemetry-unit): `scenarioModeNotPersistedAcrossFreshConstruction`. **NEW** (RobotLoop/wire): `robot_loop_tlm_harness.cpp::scenarioTlmModeNeverPersistsAcrossAFreshBoot` |
| 11 | Motor two-sample velocity floor | confirmed existing, from ticket 001: `devices_motor_harness.cpp::scenarioVelocityReadsZeroUntilTwoValidSamplesCollected` — not duplicated |
| 12 | Boot-lifecycle boilerplate gone, enforced by grep | **NEW**: `src/tests/sim/unit/test_telemetry_no_boot_lifecycle_calls.py` — two pytest tests, `grep -rn "markBootComplete\|tickBootSettle"` over `src/tests/` and `src/firm/`, both must return nothing; wired into `pytest`'s normal collection (`testpaths`), so it runs on every CI/test invocation, not just once |

**`app_robot_loop_harness.cpp` / `test_app_robot_loop.py` (KNOWN OBSTACLE,
audited)**: this harness was **already `-DHOST_BUILD` compile-broken
before sprint 125 started** — confirmed by direct compile (~28 errors).
Root cause: an unrelated, independent motion-library rework ("Planner
integration," dated 2026-07-26 per `src/sim/sim_harness.h`'s own
comments) changed `App::RobotLoop`'s constructor from 15 to 16 arguments
(added `App::Configurator`, replaced `Motion::MoveQueue&` with
`Motion::Planner&`) and removed `App::Drive::gainsLeft()/gainsRight()`.
125-003's own 4 TLM scenarios, written into this harness before the
breakage was noticed, therefore never actually ran (the `xfail(strict=
False)` marker hid this). Per this ticket's own explicit "fix (preferred)
or relocate" allowance: fixing this harness's ~20 inline `RobotLoop`
construction call sites for the Planner rework is a substantial, unrelated
motion-library-alignment undertaking — not telemetry-emit-policy work, and
risky to half-do across 2785 lines this sprint doesn't own. Instead, the 4
scenarios were **relocated** to `src/tests/sim/system/
robot_loop_tlm_harness.cpp` (+ `test_robot_loop_tlm.py`), built on
`TestSim::SimHarness` — the composition root `sim_api_harness.cpp` already
uses successfully, confirmed current on the Planner/Configurator shape
(it mirrors `main.cpp`) — where they compile and **pass for real**. The
old harness's dead TLM scenarios were deleted (with a pointer comment) so
they don't sit as misleading, never-executed duplicates; `test_app_robot_
loop.py`'s xfail reason was rewritten to state the CURRENT, confirmed root
cause (the stale 111-002 reorder-experiment reason no longer matches
reality) and point at the relocation. Repairing the rest of that harness
remains open, unrelated follow-up work, not required for this ticket.

One additive, test-infra-only production-adjacent change was needed to
make the relocated harness observable at all: `src/sim/sim_harness.h`
gained `SimHarness::drainReliable()` (mirrors the existing
`drainRawTelemetry()`), because `STATUS`/`HELP` replies ride
`Transport::sendReliable()` into `FakeTransport`'s separate
`sentReliable_` capture, which `SimHarness` had no accessor for before —
without it, `TLM:ON`'s own `STATUS` reply is silently invisible to any
`SimHarness`-based test. This is test scaffolding (`src/sim/`, HOST_BUILD
only, never compiled into `MICROBIT.hex`), not new firmware behavior.

### Bench criteria 13-16 — run on the stand (robot "tovez",
`/dev/cu.usbmodem2121102`; relay "zavaz", `/dev/cu.usbmodem2121302`)

Built clean (`just build-clean`, mandatory after `telemetry.h` changed
this sprint) and flashed (`mbdeploy deploy --hex MICROBIT.hex
9906360200052820a8fdb5e413abb276000000006e052820`) before any bench run.

**13 — PASS.** Passive capture opened BEFORE a forced `pyocd reset -t
nrf52833 -u <uid>` reboot (board does not reset on port open/DTR, per
`.clasi/knowledge`), captured for 32.2s:
```
captured 80 bytes over 32.2s
[0] printable=True len=36 : b'DEVICE:NEZHA2:robot:tovez:2314287040'
[1] printable=True len=36 : b'DEVICE:NEZHA2:robot:tovez:2314287040'
[2] printable=True len=5  : b'READY'
nonprintable/binary-looking byte count across whole capture: 0
```
Zero binary bytes, only cleartext lines — the normative Part 6 content
("a silent host sees zero binary frames") holds. The literal count is 3
lines, not 2: `main.cpp` sends the banner once directly at power-on
(lines 193-196, before `App::Comms`/`RobotLoop` even exist), and
`RobotLoop::boot()` sends it again at its own tail (`robot_loop.cpp:350`)
— a pre-existing, deliberate design from ticket 123-006 ("`RobotLoop::
boot()` emits it a second time when the preamble finishes, so a host that
attached mid-preamble still gets identity at a known instant"), untouched
by this sprint's own Part 6 rebuild (Part 6 only specifies `boot()`'s own
tail: `sendBanner()` then `sendReady()`, which is exactly what happens —
it says nothing about `main.cpp`'s own separate power-on emission). Not a
125 regression.

**14 — PARTIAL** (protocol-level PASS; wheel-motion-dependent checks
BLOCKED by an independent hardware fault — see below).

- `twist_drive.py --port /dev/cu.usbmodem2121102`: found and fixed a real
  gap the script itself had, exposed (not caused) by the correct new
  silence-when-parked behavior — its "before" encoder baseline was
  captured by passively watching a parked robot for 100ms, which can
  never see a frame any more under kAuto (issue Part 8 #1/#5). Fixed by
  requesting one frame explicitly (`proto.tlmNow()`, force=true, honored
  in every mode) before capturing the baseline — the same class of fix
  ticket 005 already applied elsewhere. 5/6 checks now pass:
  `connect()`, `move_twist()` ack, `stop()` ack x2 all PASS; "encoders
  moving during move_twist()" FAILs — see hardware-fault note below.
- `radio_bench_gate.py --port /dev/cu.usbmodem2121302`: same pre-move
  baseline gap found and fixed the same way. HELLO/PING/ID/VER, the
  fresh-connect and mid-session enqueue/completion ack checks, and the
  wire-quality budget (Phase 3: 455 lines demuxed, 99.56% ok, 0.22%
  corrupted, well inside the ≤25% budget, **PASS**) all confirm the
  protocol/comms/telemetry layer is healthy. The move-encoder-climb, some
  active-flag-observation, and 0x0A-repro-completion-ack checks FAIL —
  see hardware-fault note below (radio-link noise compounding with a
  stuck motor bus across repeated back-to-back sessions).
- **Silence-after-coast, measured cleanly**: a 600ms `move_twist` (TIME
  stop, no explicit `STOP`), watched continuously for 6s:
  ```
  last ACTIVE frame at t=0.637s
  last ANY frame at t=2.613s
  silence-after-coast gap: 1.976s
  ```
  This lands right at `kCoastHoldoff`=2.0s, which is the CORRECT bound
  even with frozen (wedged) encoder velocity: `lastActivity_` is granted
  the full `kCoastHoldoff` window from the last cycle `kFlagActive` was
  true regardless of subsequent wheel velocity — velocity only EXTENDS an
  already-open window, it doesn't gate the window's own grant. Positive,
  clean confirmation of the coast-holdoff mechanism on real hardware.

  **Hardware fault (independent of this sprint, NOT a telemetry
  regression)**: `STATUS` on "tovez" reports `wedge=1` and `otos=0`
  continuously — before any commands, after a fresh `pyocd reset`, and
  after TWO independent full mass-erase-and-reflash cycles:
  ```
  STATUS:ready=1:active=0:connL=1:connR=1:otos=0:wedge=1:flags=0xd8:tlm=auto
  ```
  This is `kFlagFaultWedgeLatch` (bit 7) — a well-documented, recurring
  encoder-controller latch condition in this project's own history
  (`docs/knowledge/2026-07-04-encoder-wedge.md`, several `done` sprints),
  plus the OTOS chip not connecting at all. It freezes `enc_left`/
  `enc_right` at 0 regardless of real motor motion, which is the DIRECT
  and SOLE cause of every "encoders climbed"/closure-dependent FAIL in
  this session — the ack ring, mode transitions, `active` flag on
  `kFlagActive`, and the coast-holdoff timing above all behave correctly
  even under this fault, and the fault itself is correctly SURFACED by
  telemetry (`wedge=1`, bit 7) exactly as designed. Attempted recovery: a
  fresh SWD reset, two independent full reflashes (each with its own
  CTRL-AP mass erase), and an idle-wait re-check — all showed the
  identical persistent state. This did not exist before I started this
  session's hardware work in a way I could distinguish from "was already
  present" — it was observed on the FIRST STATUS query after the FIRST
  fresh flash, before any move was ever commanded. The only firmware I
  flashed this session is `master`'s current sprint-125 tip, which I did
  not author beyond this ticket's test-only changes (no production `src/
  firm/`/`src/motion/` file was touched by 125-006). **Recommended next
  step**: a physical power-cycle (full USB unplug/replug, not just an SWD
  reset) and/or a connector/bus inspection of "tovez", then re-run
  `radio_bench_gate.py`/`square_tour.py --port` to complete this
  criterion and the sprint's own Success Criteria checklist.

**15 — PASS**, scripted human-terminal-equivalent check (robot parked,
default kAuto):
```
frames observed while parked/default: 0 (expect 0)
TLM:ON reply: ['STATUS:ready=1:active=0:connL=1:connR=1:otos=0:wedge=1:flags=0xd8:tlm=on']
frames observed during TLM:ON window: 34 (expect several -- streaming)
TLM:OFF reply: ['STATUS:ready=1:active=0:connL=1:connR=1:otos=0:wedge=1:flags=0xd8:tlm=off']
frames observed after TLM:OFF: 0 (expect 0)
STATUS reply: ['STATUS:ready=1:active=0:connL=1:connR=1:otos=0:wedge=1:flags=0xd8:tlm=off']
```
`TLM:ON` → binary streams (34 frames in 0.5s); `TLM:OFF` → link goes
quiet (0 frames) and its own reply confirms `tlm=off`; a follow-up
`STATUS` independently confirms `tlm=off` in a readable line. All three
literal claims in criterion 15 hold.

**16 — PASS**, via `tlm_log.py` (NOT `otos_drift.py`): `otos_drift.py`
imports `rig_dev.Rig`, which targets the separate, quarantined "OTOS-on-
a-servo rig" fixture, not the robot under test here (project convention:
robot is the default target, rig is off-limits unless explicitly
requested — `.claude/rules` / project memory). `tlm_log.py` is also
explicitly named in Part 7's own migration list, targets the robot
directly (`SerialConnection`/`NezhaProtocol`, not `Rig`), and already
self-brackets with `tlmOn()`/`tlmOff()` (125-005):
```
uv run python src/tests/bench/tlm_log.py --port /dev/cu.usbmodem2121102 --duration 8 --csv ...
capturing 8s -> ...
  t=1.0s rows=20 ... t=7.0s rows=159
wrote 182 rows to ...
```
182 CSV rows written end-to-end, confirming the migration (frames are
only present because of the script's own `TLM:ON`/`TLM:OFF` bracket —
under kAuto a parked robot would otherwise write zero rows).

### `square_tour.py` (stakeholder-added, distinct from the 16)

- `--sim`: **PASS**. `path 2066.5 mm (target 2000), heading 361.2 deg
  (target 360), closure 6.0 mm (plant truth)` — depends on ticket 007
  (done), matches the sprint notes' reference figures.
- `--port /dev/cu.usbmodem2121102 --no-geofence` (no camera/playfield in
  this bench session — robot is on the stand, not the playfield, per
  hardware-bench-testing.md): **FAIL**. `path 0.0 mm (target 2000),
  heading 0.0 deg (target 360), closure 0.0 mm (encoder odometry)` — a
  direct, fully-explained consequence of the SAME hardware fault above
  (encoder odometry reads flat because the encoder controller is wedged;
  the tour's own commanded motion sequence executed normally per the
  ack/active-flag evidence in criterion 14). NOT a telemetry-sprint
  regression, NOT weakened/relaxed to force a pass — reported honestly
  per this ticket's own instruction.

### Sprint Success Criteria (sprint.md) — NOT fully satisfied

1. All 16 Part 8 criteria pass: 12/12 sim PASS; 3/4 bench fully PASS
   (13, 15, 16), criterion 14 PARTIAL (protocol-level PASS, wheel-motion
   sub-checks blocked). **Not fully met.**
2. `Devices::Motor::velocity()` two-sample unit test passes: **met**
   (criterion 11).
3. No source reference to the deleted arming-lifecycle symbols anywhere
   in `src/`: **met**, enforced by the new grep-guard pytest tests
   (criterion 12) plus a direct grep against all of `src/` this session
   (clean).
4. `square_tour.py` passes both `--sim` and hardware: **not met** — sim
   passes, hardware is blocked by the hardware fault above.

**This ticket is left `in-progress`, not `done`**, because bullets 1 and
4 are not honestly satisfiable from software alone right now. Every piece
of work that COULD be completed and verified — all 12 sim criteria (with
a defensible, tested reason where the letter of the ticket's own
"fix/relocate" allowance was invoked), the criterion-12 automated guard,
bench criteria 13/15/16, the protocol-level half of criterion 14, and the
`--sim` leg of `square_tour.py` — is complete, tested, and committed. The
single remaining blocker is physical: robot "tovez"'s motor/OTOS I2C bus
needs hardware attention (a genuine power-cycle beyond SWD reset, and/or
a connector check) that this session could not perform, followed by a
re-run of `radio_bench_gate.py` and `square_tour.py --port` to close out
criterion 14 and Success Criteria bullets 1 and 4.
