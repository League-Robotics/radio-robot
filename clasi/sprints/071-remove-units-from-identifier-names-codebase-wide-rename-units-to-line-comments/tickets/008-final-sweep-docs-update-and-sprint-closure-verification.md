---
id: 008
title: Final sweep, docs update, and sprint closure verification
status: in-progress
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
- SUC-007
depends-on:
- '005'
- '006'
- '007'
github-issue: ''
issue: remove-units-from-identifier-names.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Final sweep, docs update, and sprint closure verification

## Description

Close out the sprint: confirm zero remaining unit-suffixed identifiers in
`source/`, zero remaining `FIXME` markers related to this issue,
byte-identical wire output, and a green suite; update the prose
documentation files that quote specific C++ field names renamed by
tickets 002-007. This ticket is the sprint's own acceptance-criteria
closure, mirroring the issue's own acceptance criteria list line-for-line
(`architecture-update.md` Step 3).

This ticket depends on 005, 006, and 007 (every rename ticket must have
landed before the final whole-tree grep can certify the issue's
acceptance criterion: "No identifier in `source/`... embeds a unit
suffix... except documented exclusions").

Scope:
- `docs/protocol-v2.md`, `docs/architecture.md`, `docs/overview.md`,
  `docs/kinematics-model.md`: prose/table mentions of renamed C++ field
  names updated.
- Final `grep -rniE "\b[a-z_][a-z0-9_]*(mm|mms|deg|dps|us|pct|hz)\b"
  source/` (case-insensitive, word-boundary) and `grep -rn "FIXME"
  source/` both return **zero** results (excluding this sprint's own
  planning-doc prose, which lives outside `source/`).
- Full `uv run python -m pytest` run: confirm 2620 (or the then-current
  ticket-adjusted count) passed, 0 failed.
- Confirm no code change is otherwise needed — this is expected to be a
  documentation-only ticket unless the final sweep's grep surfaces a
  residual identifier tickets 002-007's own acceptance criteria didn't
  catch, in which case fix it here and note it.

If the final grep does surface a residual, treat fixing it as within this
ticket's scope (it is exactly the closure check this ticket exists to
run) — but if the residual is large or structurally surprising (e.g. an
entire file family tickets 002-007 missed), stop and flag it rather than
silently absorbing a second sweep's worth of work into a "final sweep"
ticket; report back rather than scope-creeping.

See `architecture-update.md` Step 5 ("008 — Final sweep, docs, closure"),
Step 7 Open Questions (esp. #1, sprint 072 recommendation), the
Architecture Self-Review's "Verdict: APPROVE"; `usecases.md` SUC-001
through SUC-007 (all).

## Acceptance Criteria

- [ ] `grep -rniE "\b[a-z_][a-z0-9_]*(mm|mms|deg|dps|us|pct|hz)\b"
      source/` returns zero results (word-boundary, case-insensitive;
      excludes wire-key string literals per the Wire-Compatibility
      Exclusion Table, which are not identifiers).
      **NOT MET — large/structural residual found, escalated rather than
      absorbed (see "Final Sweep Findings" below).** The literal grep
      returns hundreds of hits. After excluding wire-key string literals
      and comments, a genuine residual of ~90 unique unit-suffixed
      identifiers remains across ~55 `source/` files. This is not the
      "small residual" case the ticket anticipates fixing inline — it is
      the "large or structurally surprising… entire file family tickets
      002-007 missed" case the ticket explicitly instructs to "stop and
      flag… report back rather than scope-creeping" on. Every major
      sub-family in the residual traces to an *explicit, already-adjudicated
      exclusion* recorded in tickets 002/005/006/007's own completion notes
      (e.g. ticket 005: "those exact names belong to the unrelated
      `IPositionMotor` servo/motor-position interface… out of every
      ticket's declared scope in this sprint and was left untouched";
      ticket 002: `RobotGeometry`'s `halfTrackMm`/`halfWheelbaseMm` is "a
      separate struct, never in scope"; ticket 007: `PhysicsWorld`/
      `SimHardware`'s `trackwidthMm()`/`_trackwidthMm` "would cascade into
      three files this ticket does not own"; ticket 006: `Motor.{h,cpp}`'s
      `_lastVelocityMmps`/`_lastTickMs`/`_lastWriteUs`/`kMinWriteIntervalUs`
      etc. "are intentionally untouched"). Renaming all of it in this
      ticket would mean unilaterally reversing several already-completed,
      reviewed ticket-scope decisions without the architecture-review
      surface that established them — thrown as a ticket exception (see
      below) rather than done silently.
- [x] `grep -rn "FIXME" source/` returns zero results. Confirmed: `grep -rn
      "FIXME" source/` returns zero results (unchanged since tickets
      002/005/006/007 closed this out).
- [x] `docs/protocol-v2.md`, `docs/architecture.md`, `docs/overview.md`,
      `docs/kinematics-model.md` updated wherever they quote a C++
      field/identifier name renamed by tickets 002-007. Done:
      `protocol-v2.md` (`mmPerDeg`→`wheelTravelCalib`, `sigmaMm`→`sigma`,
      `_driftPerTickMm`/`_driftPerTickRad`→`_linearDriftPerTick`/
      `_yawDriftPerTick`, `controlPeriodMs`→`controlPeriod`; wire-key
      mentions of `trackwidthMm`/`otosLinDriftMmS`/`otosYawDriftDegS` in
      `SIMSET`/`SIMGET` examples and the Named Key Table left untouched —
      those are wire keys, per the Exclusion Table, not stale identifiers).
      `architecture.md` (`mmPerDegL/R`→`wheelTravelCalibL/R`,
      `trackwidthMm`→`trackwidth`, `arriveTolMm`→`arriveTolerance`,
      `tickMs`→`tick` ×2). `kinematics-model.md` (`mmPerDeg`/`mmPerDegL/R`→
      `wheelTravelCalib`/`wheelTravelCalibL/R` ×6, `trackwidthMm`→
      `trackwidth`, `arriveTolMm`→`arriveTolerance`; `lapsToMm` left as-is —
      a pre-existing, unrelated staleness from the sprint-010 deletion of
      `lapsToMmScale`, not a sprint-071 rename, out of this ticket's scope).
      `overview.md` had no field-name mentions requiring update (confirmed
      by grep — zero hits).
- [x] Every `SET`/`GET`/`SIMSET`/`SIMGET`/`STREAM`/`TLM`/`SNAP` wire byte
      is identical before and after the full sprint (spot-check against
      `tests/_infra/golden_tlm_capture.json`, which requires no
      regeneration). Confirmed: no wire-affecting source change was made by
      this ticket (only a doc-comment fix in `gen_default_config.py`'s
      generated `DefaultConfig.cpp` and prose-doc edits); `test_golden_tlm_
      unchanged` passes as part of the full suite below.
- [x] Full test suite green (`uv run python -m pytest`): 2620 passed (or
      the then-current ticket-adjusted count from 002/005/006/007's own
      test updates), 0 failed. Result: **2621 passed, 0 failed** (matches
      the 002/005/006/007 baseline exactly — no test count change from this
      ticket, since it made no test-affecting change).
- [x] No `data/robots/*.json`, `host/robot_radio/config/robot_config.py`,
      or other `host/robot_radio/` file was modified across the whole
      sprint (Decision 1 and Decision 6 scope boundary — confirm via
      `git diff --stat` against the sprint's base commit). Confirmed via
      `git diff --stat 753e52f..HEAD -- host/robot_radio/ data/robots/`
      (753e52f = sprint 070's merge commit, sprint 071's base): zero
      `host/robot_radio/` changes anywhere in the sprint. The only
      `data/robots/` change across the whole sprint is
      `robot_config.schema.json`'s `firmware.field` values (ticket 002) —
      diffed line-by-line and confirmed every `set_key` value and every
      per-robot data file (`tovez.json`, `togov.json`, `tovez copy.json`)
      is byte-identical. This is the schema's internal field-name mapping
      declaration (Decision 2/3's four-file `RobotConfig` codegen chain),
      not the per-robot JSON config data or pydantic surface Decision 6
      excludes — the two are different things that happen to share a
      directory and a `.json` extension; no violation.
- [x] Sprint-level confirmation that this issue
      (`remove-units-from-identifier-names.md`) is only **partially**
      closed by sprint 071 (the `source/` C++ half) — the host-Python half
      remains open and is recommended for a follow-up sprint (072), per
      `architecture-update.md` Decision 1 and Open Question 1. This ticket
      does not mark the parent issue fully resolved. Confirmed, and
      additionally: even the `source/` C++ half itself is only partially
      closed by this sprint — see the AC #1 residual above and "Final
      Sweep Findings" below. Two follow-ups are recommended: sprint 072
      (host-Python half, per Decision 1, unchanged) and a new sprint/ticket
      for the `source/`-internal residual this final sweep surfaced
      (scope: the ~8 file-families enumerated below).

## Final Sweep Findings (2026-07-03, ticket 008)

The closure grep (`grep -rniE "\b[a-z_][a-z0-9_]*(mm|mms|deg|dps|us|pct|hz)\b"
source/`, filtered to exclude comments and wire-key string literals) surfaces
a residual of roughly 90 unique unit-suffixed identifiers across ~55 files.
Cross-referencing every hit against tickets 002/005/006/007's own completion
notes shows the residual is not scattered oversight — it resolves into about
eight coherent, previously-unscoped (or explicitly out-of-scoped) file
families:

1. **Motor/servo position-and-velocity capability interface** —
   `source/hal/capability/IPositionMotor.h`, `IVelocityMotor.h`
   (`positionMm()`, `setAngleDeg()`, `currentAngleDeg()`) and every
   implementer/caller: `source/hal/real/Motor.{h,cpp}` (remaining names
   ticket 006 explicitly left untouched — `_lastVelocityMmps`,
   `_lastTickMs`, `_lastWriteUs`, `kMinWriteIntervalUs`, `mmPerSec`,
   `now_ms`, `readEncoderMmF*`), `Servo.{h,cpp}`, `hal/sim/SimServo.{h,cpp}`,
   `hal/sim/SimMotor.{h,cpp}` (`_lastPositionMm`, `reportedEncMm()`,
   `setNoiseSigma(sigmaMm)` — explicitly declared out of ticket 007's scope),
   `hal/ReplayHAL.h`, `hal/NoopDevices.h`, `subsystems/gripper/Gripper.h`,
   `hal/real/BenchOtosSensor.{h,cpp}`, `control/ServoController.cpp` (call
   sites). Named explicitly by ticket 005 as "out of every ticket's declared
   scope in this sprint."
2. **`RobotGeometry`/mecanum geometry** — `hal/capability/Pose2D.h`
   (`halfTrackMm`, `halfWheelbaseMm`), `robot/MecanumHAL.{h,cpp}`
   (`_halfTrackMm`, `_halfWheelbaseMm`, `_lastBenchTickMs`),
   `kinematics/MecanumKinematics.cpp`. Ticket 002 explicitly named
   `RobotGeometry` as "a separate struct, never in scope."
3. **`ActualState`/hardware-observation struct** — `state/ActualState.h`
   (`encMm[]`, `velMms[]`) and its pervasive consumers: `subsystems/drive/
   Drive.{h,cpp}`, `control/MotorController.cpp`, `superstructure/
   Planner.cpp`, `control/PlannerBegin.cpp`, `control/StopCondition.cpp`,
   `commands/MotionCommand.cpp`, `commands/SystemCommands.cpp`,
   `commands/ConfigCommands.cpp`, `commands/DebugCommands.cpp`,
   `robot/LoopTickOnce.{h,cpp}`, `robot/Robot.cpp`.
4. **`ValueSet`/stamp lag-tracking struct** — `types/ValueSet.h` (`lagMs`,
   `lastUpdMs`) and consumers: `types/Inputs.h`, `state/PoseEstimate.h`,
   `state/EstimateDump.h` (`ageMs`), `subsystems/sensors/{ColorSensor,
   LineSensor,Ports,Sensors}.cpp`, `subsystems/drive/Drive.cpp`,
   `control/Odometry.cpp`.
5. **`Superstructure::GoalRequest`** — `superstructure/Superstructure.{h,cpp}`
   (`leftMms`, `rightMms`, `durationMs`, `targetMm`, `speedMms`),
   `commands/MotionCommands.cpp`, `robot/Robot.{h,cpp}` (`distanceDrive`'s
   `targetMm` param).
6. **Sim-plant trackwidth/timing remnants** — `hal/sim/PhysicsWorld.{h,cpp}`
   (`_trackwidthMm`, `trackwidthMm()`, `kDefaultTrackwidthMm`),
   `hal/sim/SimHardware.{h,cpp}` (mirrors), `hal/sim/SimColorSensor.{h,cpp}`,
   `SimLineSensor.{h,cpp}` (`_elapsedMs`, `kRowDurationMs`),
   `commands/SimSetters.h` (`getTrackwidth()` wrapping the untouched
   `trackwidthMm()` accessor). Ticket 007 explicitly declined these,
   "would cascade into three files this ticket does not own."
7. **`RobotTelemetry`/`Robot` core timing** — `robot/RobotTelemetry.cpp`
   (`kIdleMinMs`, `kGraceMs`, `kRadioMinMs`), `robot/Robot.{h,cpp}`
   (`_lastTlmMs`, `_lastActiveMs`, `_otosInvalidStartMs`),
   `robot/LoopScheduler.h`/`LoopTickOnce.h` (`watchdogMs`).
8. **Diagnostic/bench-only tools never named by any ticket** —
   `robot/WedgeTest.{h,cpp}` (a bench diagnostic, not production drive
   code — `busyUs`, `rateHz`, `writeMs`, `periodUs`, `kDelayUs` family),
   `com/SerialPort.cpp` (local `deadlineUs`/`drainUs`/`settleUs`),
   `main.cpp` (`lastInputMs`).

Every family above is either (a) never named in any of 002/005/006/007's
"Scope:" sections at all, or (b) explicitly named and explicitly declined by
that ticket's own implementer with a stated reason (usually: renaming it
would cascade into files outside that ticket's declared scope). Absorbing
all eight families into this "final sweep" ticket would mean redoing, without
architecture review, scope calls that were already made and accepted across
four separate ticket closures — exactly the "second sweep's worth of work"
this ticket's own Description explicitly says not to silently absorb. Thrown
as a ticket exception (`thrown_by: programmer`) rather than either (a) doing
the rename unreviewed, or (b) checking off AC #1 as met when it is not.

**What this ticket did complete** (independent of the AC #1 residual):
the required `gen_default_config.py` build fix (see below), the four-doc
prose sweep, the full-suite green confirmation, and the host/data scope
verification.

## Required Build Fix: `scripts/gen_default_config.py`

Confirmed and fixed the pre-existing, cross-referenced bug flagged by
tickets 004 and 006: `gen_default_config.py`'s template still emitted
`p.turnThresholdMm = 50.0f;` / `p.doneTolMm = 5.0f;` — two `RobotConfig`
fields sprint 070 Decision 4 deleted — and `p.tlmFields = 0x1FF;` as a raw
literal (a stale non-symbolic value vs. the committed file's
`TLM_FIELD_ALL`). A fresh `python3 build.py --fw-only` regenerates
`DefaultConfig.cpp` from this generator unconditionally, so the un-fixed
generator was one `build.py` invocation away from a firmware build break
for anyone who ran it without ticket 006's manual `git checkout --`
workaround.

Fix: removed the two stale `p.turnThresholdMm`/`p.doneTolMm` lines (and
their now-orphaned "Legacy go-to tolerances" comment) since `Config.h` has
no such fields (confirmed by direct read — sprint 070 deleted them
end-to-end); changed `p.tlmFields = 0x1FF;` to `p.tlmFields =
TLM_FIELD_ALL;` (the symbol is available — `DefaultConfig.cpp` already
`#include`s `Config.h`), matching the checked-in file's existing form.

Verified: ran `python3 scripts/gen_default_config.py` directly — the
regenerated `DefaultConfig.cpp` is byte-identical to the previously
committed file except for one restored comment (the `tlmFields` line's
explanatory comment, which the committed file was missing — a leftover
of ticket 004's "hand-patched post-070/068" note). Then ran `cmake --build
tests/_infra/sim/build --target clean` followed by a full rebuild (fresh
`libfirmware_host.dylib`, zero compile errors), then `python3 build.py
--fw-only` end-to-end with **no manual neutralization of any kind** —
`gen_default_config.py` ran automatically as part of the normal build,
wrote a compiling `DefaultConfig.cpp`, and the firmware linked and
produced `MICROBIT.hex` (`v0.20260703.7`, 53.25% FLASH). The regenerated
`DefaultConfig.cpp` (now the generator's exact, reproducible output) is
committed as this ticket's diff.

## Testing

- **Existing tests to run**: full suite (`uv run python -m pytest`) as
  the final closure gate; a manual `SET`/`GET`/`SIMSET`/`SIMGET` smoke
  round-trip against a running sim instance is recommended but not
  required if the automated suite already covers the affected keys.
- **New tests to write**: none — this ticket verifies, it does not add
  new behavior.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Run the two closure greps first (unit-suffix, FIXME) across
`source/`. If either returns a result, fix it (small residual) or stop
and report (large/structural residual) before touching docs. Then sweep
the four named prose docs for stale identifier mentions, update them, and
run the full suite one final time as the sprint-closure gate.

**Files to modify**:
- `docs/protocol-v2.md`
- `docs/architecture.md`
- `docs/overview.md`
- `docs/kinematics-model.md`
- (contingently) any `source/` file if the final grep surfaces a small
  residual missed by 002-007

**Testing plan**: full suite run as the closure gate; no isolated test
tier needed since this ticket touches no runtime code path under normal
(no-residual) conditions.

**Documentation updates**: this ticket *is* the documentation-update pass
for the sprint's prose docs.
