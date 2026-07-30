---
id: '125'
title: 'Telemetry emit-policy rebuild: three-mode TLM control, delete arming machinery'
status: planning-docs
branch: sprint/125-telemetry-emit-policy-rebuild-three-mode-tlm-control-delete-arming-machinery
worktree: false
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
issues:
- telemetry-emit-policy-rebuild-spec.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 125: Telemetry emit-policy rebuild: three-mode TLM control, delete arming machinery

## Goals

Replace `App::Telemetry`'s emergent, four-boolean emit-arming machinery
(`changeReportingArmed_`/`bootSettling_`/`stableCycles_`/`lastSeenFlags_`/
`lastEmittedFlags_`, `markBootComplete()`, `tickBootSettle()`,
`kBootStableCycles`) with an explicit, three-mode, host-controllable
policy (`TLM:OFF` / `TLM:AUTO` / `TLM:ON`) per the normative spec in
`clasi/issues/telemetry-emit-policy-rebuild-spec.md`. Fix the source-level
sampling defect (`Devices::Motor::velocity()` reporting a bogus nonzero
value before two real samples exist) that the deleted machinery was
papering over. Delete the DELETIONS this issue names exactly — including
the uncommitted `tickBootSettle()`/`kBootStableCycles`/`bootSettling_`
work in the current dirty tree, which this sprint starts by removing, not
preserving. Migrate the bench scripts that depend on always-on streaming.
Additionally (stakeholder directive on top of the issue): the square tour
(`src/tests/bench/square_tour.py`) must pass in SIM and on the BENCH by
sprint close.

## Problem

The emit policy that shipped is a four-arm predicate patched five times
in two days, and it contains one structural defect, not merely
complexity: the "fresh THIS frame" bits (`kFlagLinePresent`/
`kFlagColorPresent`, bits 13/14) toggle every cycle by design (RobotLoop's
line/color alternation). On a robot with a connected line or color
sensor the flags word can never hold still for `kBootStableCycles`, so
report-on-change never arms and the coast-down arm it gates is
permanently dead — while, had it armed, the same toggling would have
defeated idle silence and streamed at ~25 Hz forever. This is confirmed
by reading `telemetry.h`/`telemetry.cpp` as they stand in the working
tree today (Part 1 of the issue enumerates the exact symbols), not
speculative.

Separately, arm 2 (the coast-down/wheel-velocity arm) was gated behind
`changeReportingArmed_` to suppress a symptom — a bogus nonzero velocity
reading on the very first encoder sample after power-on — whose real fix
belongs in `Devices::Motor`, not in telemetry's arming state.

## Solution

1. **Fix the source-level defect** (Part 2): `Devices::Motor::velocity()`
   returns `0.0f` until the device has collected at least two valid
   samples. No telemetry-side guard is added for this condition — this is
   the belt half of "belt and suspenders" with the new `everMoved_` gate
   below.
2. **Delete the arming lifecycle outright** (Part 1): `kBootStableCycles`,
   `markBootComplete()`, `tickBootSettle()` and its `RobotLoop::cycle()`
   call site, `changeReportingArmed_`/`bootSettling_`/`stableCycles_`/
   `lastSeenFlags_`/`lastEmittedFlags_`, `hasSomethingToSay()` arm 4
   (report-on-change), the arm-2 gate, and `kFlagEventBootReady` (bit 11,
   → RESERVED). No flags, no comment-out — removed.
3. **Rebuild the emit predicate around one explicit mode** (Part 3):
   `TlmMode{kOff, kAuto, kOn}`, one member, one writer, reset to `kAuto`
   every boot, never persisted. Three reasons to emit — request (`force`),
   unsolicited (per-mode: never / activity-window / always), and pending
   ack delivery (honored in EVERY mode, since protocol v5 has no separate
   ack message — the telemetry frame is the ack's only vehicle).
4. **Add the `TLM` command surface** (Part 4): `TLM`/`TLM:NOW`/`TLM:ON`/
   `TLM:AUTO`/`TLM:OFF`, colon-spelled, case-insensitive, mode changes
   reply with `STATUS` (which gains `tlm=off|auto|on`), `HELP` lists the
   new forms, garbage args reply `HELP` and change nothing, nothing is
   persisted. `NezhaProtocol` gains `tlmOn()`/`tlmOff()`/`tlmNow()`.
5. **Document the flags word's true category structure** (Part 5): every
   bit classified as State / Freshness / Event under labeled headers, so
   the category error (treating a Freshness bit as stability evidence)
   cannot recur silently.
6. **Migrate parked-capture bench scripts** (Part 7) to bracket their
   capture with `TLM:ON`/`TLM:OFF`.
7. **Prove it** (Part 8): sim acceptance criteria 1-12 as real tests
   (including the sensor-equipped-parked-robot regression the old design
   could not pass, and a hard "no test may re-add a Telemetry lifecycle
   call" check), plus the bench gate criteria 13-16.
8. **[Adjacent to the issue, stakeholder-added]** Fix the sim-plant
   rotation-calibration gap that currently fails `square_tour.py --sim`
   at the default (ANGLE-stop) corner mode, so the added "square tour
   passes in sim and on the bench" acceptance criterion is achievable —
   see ticket 007 and the Design Rationale entry below for why this is
   in scope despite not being caused by the telemetry defect.

## Success Criteria

- All 16 of the issue's Part 8 acceptance criteria pass (12 sim, 4 bench).
- `Devices::Motor::velocity()`'s two-sample unit test passes.
- No source reference to `kBootStableCycles`, `markBootComplete`,
  `tickBootSettle`, `bootSettling_`, `stableCycles_`, `lastSeenFlags_`,
  `lastEmittedFlags_`, `changeReportingArmed_`, or `kFlagEventBootReady`
  remains anywhere in `src/` (including test harnesses).
- `src/tests/bench/square_tour.py` passes both `--sim` and against the
  bench robot on the stand (stakeholder-added acceptance, tracked
  alongside but distinct from the issue's own Part 8 list).

## Scope

### In Scope

- `src/firm/app/telemetry.h` / `telemetry.cpp` (deletions + rebuild).
- `src/firm/app/robot_loop.cpp` (call-site deletion, two comment fixes).
- `src/firm/app/comms.{h,cpp}` (`TLM:` argument parsing, `STATUS`/`HELP`
  updates, mode-change plumbing to `RobotLoop::cycle()`).
- `Devices::Motor` velocity sampling (whichever concrete leaf(s) own the
  defect — `NezhaMotor` and any sim/fake motor sharing the interface).
- `src/host/robot_radio/robot/protocol.py` (`NezhaProtocol` mirror).
- `src/tests/bench/{otos_drift,tlm_log,velocity_step_response}.py` and any
  other parked-capture script a grep of `src/tests/bench/` turns up.
- `src/tests/sim/unit/app_telemetry_harness.cpp` /
  `app_robot_loop_harness.cpp` (boot-boilerplate deletion + new
  acceptance tests).
- `data/robots/tovez_nocal.json` and/or the sim-plant rotation model
  (ticket 007, adjacent).

### Out of Scope

- Push-on-fault via masked flag comparison (explicitly deferred by the
  issue — a future issue if wanted, scoped to latching fault bits only,
  never freshness or event bits).
- Any wire-format change to the telemetry frame itself. The only protocol
  additions are the cleartext `TLM:` arguments and the `STATUS` `tlm=`
  field, both additive.
- Renaming or restructuring the ack ring, `STATUS`/`HELP`, or the verbs
  table.
- Persisting the telemetry mode in config (deliberately rejected by the
  issue, Part 4).
- Any change to `Motion::WheelSink`, `Motion::MoveQueue`, or the motion
  library's own control law, except as an incidental consumer of
  `Devices::Motor::velocity()`'s corrected boundary behavior.

## Test Strategy

Sim-first: extend `app_telemetry_harness.cpp` and
`app_robot_loop_harness.cpp` with the 12 sim criteria from Part 8,
including the regression case (criterion 2) the pre-rebuild design could
not pass, and a grep-enforceable check (criterion 12) that no test
re-adds a Telemetry lifecycle call. A dedicated `Devices::Motor` unit
test covers the two-sample velocity fix (Part 2/criterion 11). Bench:
the standing hardware bench gate (`.claude/rules/hardware-bench-testing.md`)
covers criteria 13-16 — passive power-on capture, `radio_bench_gate.py` +
`twist_drive.py` under `kAuto`, a human-terminal `TLM:ON`/`TLM:OFF` check,
and one migrated parked-capture script end-to-end. `square_tour.py` is
run both `--sim` and against the bench robot on the stand as an
additional, stakeholder-added acceptance gate, separate from but run
alongside the Part 8 bench criteria.

## Architecture

**Substantial** — 3+ modules touched with a real cross-cutting concern:
`App::Telemetry` (the emit policy itself), `App::Comms` (new command
parsing + STATUS/HELP plumbing), `Devices::Motor`/`NezhaMotor` (the
sampling-defect fix), and the host `robot_radio` protocol mirror. No new
cross-module dependency and no dependency-direction change — the new
`TLM:` plumbing follows the EXISTING bare-`TLM` pattern
(`Comms` parses → stages a request → `RobotLoop::cycle()` consumes and
calls into `Telemetry`) end to end, just widened from a boolean flag to
a small argument. No data-model change on the wire (Part 8's "no wire
change" constraint holds throughout — see Migration Concerns). Per the
sprint-020 precedent for the substantial tier: **no diagram is included**
— nothing NEW is being composed here; the module graph
(`RobotLoop` → `Comms`, `RobotLoop` → `Telemetry`, `Comms` ↛ `Telemetry`
directly) is exactly what it was before this sprint, and a component
diagram would show four boxes and the same two arrows sprint 124 already
documented, clarifying nothing beyond the one-sentence purpose statement
each module gets below.

### Architecture Overview

**Step 1 — the problem restated.** Boot is silent except the `DEVICE`
banner and `READY`; a frame goes out when there is an ack to carry (or
while motion is running); when the robot is parked with nothing to say,
the link is quiet. What exists instead is state spread across five
members and a four-arm predicate, with one arm (report-on-change)
structurally incapable of ever arming safely on sensor-equipped
hardware. The fix is not another patch on the arming machine — it is
deleting the machine and replacing it with a single enum whose three
values are individually simple to reason about.

**Step 2 — responsibilities this sprint touches, grouped by what
changes for the same reason:**

1. **Emit-policy state and predicate** (changes because the mode model
   changed) — `App::Telemetry`'s `mode_`, `everMoved_`, `lastActivity_`,
   `pendingAckDeliveries()`, and the rewritten `emit()`/`hasSomethingToSay`
   equivalent.
2. **Velocity sampling correctness** (changes because it is a distinct,
   pre-existing device-layer defect, not a telemetry concern) —
   `Devices::Motor::velocity()`'s two-valid-samples floor.
3. **Command surface** (changes because a new host-facing control needs
   a wire entry point) — `App::Comms`'s `TLM:` argument parsing,
   `STATUS`'s `tlm=` field, `HELP`'s new line, and the
   `RobotLoop::cycle()` consume-and-apply step.
4. **Documentation of an existing invariant** (changes because the
   category error that caused the defect needs a structural guard against
   recurrence) — the flags-word State/Freshness/Event classification in
   `telemetry.h`.
5. **Bench-script contract** (changes because the default behavior
   parked-capture scripts relied on — always-on streaming — is gone) —
   `otos_drift.py`, `tlm_log.py`, `velocity_step_response.py`, and any
   sibling found by grep.
6. **[Adjacent] Sim-plant rotation fidelity** (changes because it blocks
   an acceptance criterion this sprint now owns, though its root cause is
   unrelated to telemetry) — the ANGLE-stop MOVE overshoot in the sim
   plant that fails `square_tour.py --sim`'s default corner mode.

**Step 3 — modules, purpose (one sentence, no "and"), boundary:**

- **`App::Telemetry`** (`src/firm/app/telemetry.{h,cpp}`) — projects
  `Types::RobotState` into the outbound `msg::Telemetry` frame and decides
  when an unsolicited frame is worth sending. Inside: the mode enum, the
  activity/coast-hold state, the ack ring, cadence pacing. Outside:
  parsing wire text (never touches `char*`), deciding RobotState's own
  contents. Serves SUC-001/002/003 below.
- **`App::Comms`** (`src/firm/app/comms.{h,cpp}`) — turns wire lines into
  staged, RobotLoop-consumable requests and formats the cleartext
  replies. Inside: verb dispatch, `TLM:` argument recognition, `STATUS`/
  `HELP` formatting. Outside: deciding what a mode change DOES (that's
  `Telemetry::setMode()`, called from `RobotLoop::cycle()`, same as the
  existing bare-`TLM` → `takeTelemetryRequest()` pattern). Serves
  SUC-002.
- **`Devices::Motor`** (interface) / **`Devices::NezhaMotor`** (leaf,
  `src/firm/devices/{motor.h,nezha_motor.{h,cpp}}`) — reports a wheel's
  position and velocity from encoder samples. Inside: the two-sample
  floor on `velocity()`. Outside: any policy about what to DO with a
  fresh-vs-stale reading (that is `App::Telemetry`'s `everMoved_`/
  `lastActivity_`, layered on top, not inside the device). Serves
  SUC-001.
- **Host `NezhaProtocol`** (`src/host/robot_radio/robot/protocol.py`) —
  the host-side mirror of the wire verb surface. Inside: three trivial
  wrapper methods sending the three new `TLM:` lines. Outside: any retry/
  backoff policy (unchanged, inherited from the existing `TLM` path).
  Serves SUC-002/SUC-004.
- **`[Adjacent]` sim-plant rotation model** (wherever the ANGLE-stop
  MOVE's systematic overshoot is actually rooted — ticket 007 confirms
  this first; candidates are the `data/robots/tovez_nocal.json`
  `rotation_gain`/`rotation_offset_deg` calibration values themselves, or
  a kinematic mismatch in the sim plant that no calibration value can
  paper over) — serves SUC-005 (the added square-tour acceptance).

**Step 4 — diagram**: omitted; see the Architecture section's opening
paragraph for the one-sentence justification (no new composition, same
module graph as before this sprint).

**Step 5 — What Changed / Why / Impact / Migration**: see Solution above
for What Changed; Problem above for Why. **Impact on Existing
Components**: `App::RobotLoop` loses two lines (the `tickBootSettle()`
call site and the stale arming comment) and gains one (consuming a
mode-change request the same way it already consumes
`takeTelemetryRequest()`); no other RobotLoop behavior changes. Every
OTHER command handler, the ack ring, `STATUS`'s existing fields, and
`HELP`'s existing entries are unaffected — this is additive there.
`Devices::MotorArmor` (the wedge-detection decorator) forwards
`velocity()` unchanged; its own logic does not interpret the value, so
the two-sample floor is transparent to it.

### Design Rationale

**Decision 1 — delete the arming machinery rather than patch the
report-on-change arm to mask Freshness bits.** Context: a masked
comparison that excludes bits 0/13/14 from report-on-change would also
fix the deadlock. Alternatives considered: (a) mask Freshness bits out of
the existing change-comparison; (b) the full deletion the issue
specifies. Why this choice: (a) still leaves a five-member hidden
lifecycle (`bootSettling_`/`stableCycles_`/etc.) whose only job was
serving an arm that report-on-change itself was never actually in the
original stakeholder spec (the issue's own "why" — flag-change push was
never asked for). Deleting the lifecycle removes the whole class of
future masking bugs, not just this instance. Consequences: any future
push-on-fault feature is a NEW, narrowly-scoped addition (masked over
latching fault bits only), not a resurrection of arm 4.

**Decision 2 — fix `Devices::Motor::velocity()` at the source rather than
add a telemetry-side guard.** Context: arm 2's `changeReportingArmed_`
gate existed only to suppress a bogus first-sample velocity. Alternatives
considered: (a) keep a narrow telemetry-side "ignore velocity until N
samples" guard; (b) fix the device layer so the bogus value never exists.
Why this choice: (b) is a correctness fix with one true home — a velocity
is a difference quotient, and with fewer than two samples there IS no
velocity, full stop — versus (a), which is a second place (in addition to
whatever code actually consumes velocity for control) that has to know
about a device-layer artifact. The issue is explicit that no
telemetry-side guard may be added for this condition; `everMoved_` is a
deliberate, independent second line of defense (belt-and-suspenders), not
a substitute fix. Consequences: any other future consumer of
`Motor::velocity()` (not just Telemetry) also gets the corrected boundary
behavior for free.

**Decision 3 — mode changes reply with `STATUS`, not a bespoke ack.**
Context: `TLM:ON`/`AUTO`/`OFF` need SOME reply. Alternatives considered:
(a) a new one-line ack format (e.g. `TLM:OK:on`); (b) reuse `STATUS`.
Why this choice: (b) gives a human immediate, already-known-format
cleartext confirmation and a machine one parse path it already has, at
zero new-reply-shape cost. Consequences: `STATUS`'s formatter grows one
field (`tlm=`); no new message shape enters the protocol.

**Decision 4 [adjacent] — fix the sim-plant rotation discrepancy inside
this sprint rather than defer it to a separate issue.** Context: the
added stakeholder acceptance criterion ("square tour must pass in sim and
on the bench") cannot be met while the sim plant's ANGLE-stop overshoot
stands — the tour's default corner mode is exactly the code path this
defect breaks, independent of anything in the telemetry rebuild.
Alternatives considered: (a) leave it to a future issue and scope this
sprint's acceptance to the bench leg only; (b) fold a fix into this
sprint as an adjacent ticket. Why this choice: the stakeholder's own
added criterion names BOTH legs (sim AND bench) as sprint acceptance, and
(a) would silently fail to deliver what was asked; (b) is one bounded,
separable ticket (007) that does not touch telemetry code at all, so it
carries no risk of confusing telemetry's own review. Consequences: ticket
007 is explicitly flagged everywhere (sprint.md, ticket frontmatter, this
report) as NOT part of the issue's own scope, so a future reader auditing
"what did the telemetry-emit-policy-rebuild-spec issue actually change"
is not misled.

### Migration Concerns

None for the wire format — Part 8's "no wire change" holds: bit
positions and meanings are untouched (bit 11 simply never sets again),
and the two additions (`TLM:` argument tokens, `STATUS`'s `tlm=` field)
are additive cleartext, ignorable by any host that does not parse them.
Deployment sequencing: firmware and host (`NezhaProtocol`) ship together
as usual (protocol v5 already requires matched firmware/host); no host
that speaks the OLD bare-`TLM`-only surface breaks, since bare `TLM` is
kept verbatim. Mode is explicitly non-persistent (Part 4) — a power cycle
always returns to `kAuto`, so there is no stored-config migration to
reason about. `Devices::Motor::velocity()`'s changed boundary behavior
(0 instead of a bogus value pre-second-sample) is a strictly narrowing
correctness fix; no caller depended on the old bogus value (the whole
point of the issue is that nothing should have).

## Use Cases

### SUC-001: Parked robot stays silent regardless of sensor traffic
Parent: (telemetry-emit-policy-rebuild-spec.md, Parts 1-3)

- **Actor**: Host application (or a human at a serial monitor).
- **Preconditions**: Robot has booted (`READY` sent), mode is `kAuto`
  (default), no move has ever been commanded this power cycle, line and
  color sensors are connected and producing fresh alternating readings
  every cycle.
- **Main Flow**:
  1. RobotLoop cycles continuously; `Telemetry::update()` derives
     `kFlagLinePresent`/`kFlagColorPresent` toggling every cycle, as
     designed.
  2. `Devices::Motor::velocity()` may report a nonzero value from a
     hand-spun wheel, but `everMoved_` is still false.
  3. No ack is outstanding.
- **Postconditions**: Zero unsolicited telemetry frames are sent,
  indefinitely, despite the toggling Freshness bits and any bogus
  velocity reading.
- **Acceptance Criteria**:
  - [ ] Sim: parked robot with fresh alternating line/color readings for
        N cycles → `primaryEmitCount() == 0` (Part 8 criterion 2 — the
        regression the pre-rebuild design could not pass).
  - [ ] Sim: hand-spun-wheel/bogus-velocity case with `everMoved_ ==
        false` → zero frames (Part 8 criterion 5).
  - [ ] Bench: passive serial capture at power-on shows two cleartext
        lines and zero binary bytes over >= 30 s (Part 8 criterion 13).

### SUC-002: Host controls the emit mode explicitly via TLM:
Parent: (telemetry-emit-policy-rebuild-spec.md, Part 4)

- **Actor**: Host application or human at a serial monitor.
- **Preconditions**: Robot is connected, any mode.
- **Main Flow**:
  1. Host (or human) sends `TLM:ON` | `TLM:AUTO` | `TLM:OFF` |
     `TLM:NOW`/bare `TLM` | `TLM:<garbage>`.
  2. Firmware applies the mode change (or none, for request/garbage
     forms) and replies per Part 4's table.
- **Postconditions**: Mode reflects the last valid `TLM:` command; a
  garbage argument changes nothing; every mode change is visible via
  `STATUS`'s `tlm=` field.
- **Acceptance Criteria**:
  - [ ] Sim: every `TLM:ON|AUTO|OFF` answered by a `STATUS` line whose
        `tlm=` shows the NEW mode; `TLM:<garbage>` answered by `HELP`,
        changes nothing (Part 8 criterion 9).
  - [ ] Sim: bare `TLM`/`TLM:NOW` on a parked robot → exactly one frame,
        identical behavior for both spellings (Part 8 criterion 6).
  - [ ] Sim: `TLM:OFF` then a commanded Move → move executes, ONLY ack
        frames appear, no stream; bare `TLM` mid-move still answers (Part
        8 criterion 7).
  - [ ] Sim: `TLM:ON` on a parked robot → cadence frames; `TLM:OFF` stops
        the stream within one `kPrimaryPeriod`; `TLM:AUTO` restores
        mode-2 behavior (Part 8 criterion 8).
  - [ ] Sim: mode is not persistent — set `kOn`/`kOff`, simulate reboot
        → `kAuto` (Part 8 criterion 10).
  - [ ] Bench: human types `TLM:ON` in a serial monitor → binary streams;
        `TLM:OFF` → terminal typeable again; `STATUS` → readable line
        including `tlm=off` (Part 8 criterion 15).

### SUC-003: Motion and coast-down keep the link live exactly as long as needed
Parent: (telemetry-emit-policy-rebuild-spec.md, Part 3)

- **Actor**: Host application.
- **Preconditions**: Mode is `kAuto`.
- **Main Flow**:
  1. Host commands a Move; `kFlagActive` sets; frames stream at cadence.
  2. Host sends `STOP` (or the Move completes); `kFlagActive` drops but
     wheels are still coasting.
  3. Frames continue while staged wheel velocity is nonzero, refreshing
     `lastActivity_`.
  4. Wheels reach zero; no more refreshes; `kCoastHoldoff` (2000 ms)
     elapses; frames stop.
- **Postconditions**: The link is quiet again within `kCoastHoldoff` of
  true zero velocity, with no gap in coverage during the coast-down
  (the "velocity frozen at 366 mm/s" harness bug stays fixed).
- **Acceptance Criteria**:
  - [ ] Sim: Move in `kAuto` → cadence frames while active; STOP → frames
        continue while staged velocity nonzero; silence within
        `kCoastHoldoff` of velocities reaching 0 (Part 8 criterion 4).
  - [ ] Bench: `radio_bench_gate.py` + `twist_drive.py` PASS lines hold
        under `kAuto`; observe silence-after-coast (~2 s) (Part 8
        criterion 14).

### SUC-004: Every command's ack is delivered regardless of mode
Parent: (telemetry-emit-policy-rebuild-spec.md, Part 3, reason 3)

- **Actor**: Host application.
- **Preconditions**: Robot is parked, mode is `kOff` (the strictest
  case — if it holds here it holds in every mode).
- **Main Flow**:
  1. Host sends a command (e.g. `CONFIG`).
  2. Firmware pushes an ack ring entry.
  3. The next `kAckRepeats` cadence-due frames carry it, unsolicited
     status notwithstanding.
  4. After `kAckRepeats` deliveries, the link falls silent again (no
     other reason to speak).
- **Postconditions**: The command's outcome is observably delivered even
  though `kOff` suppresses every OTHER unsolicited reason to emit — no
  "acked but nothing happened" gap.
- **Acceptance Criteria**:
  - [ ] Sim: command to a parked robot → ack rides a frame within
        2x`kPrimaryPeriod`; exactly `kAckRepeats` frames carry it; then
        silence again (Part 8 criterion 3).
  - [ ] Bench: one migrated parked-capture script (e.g. `otos_drift.py`)
        runs end-to-end and records frames, proving the Part 7 migration
        (Part 8 criterion 16).

### SUC-005: [Adjacent] Square tour completes with sim-plant-accurate corners
Parent: (stakeholder-added acceptance, not part of
telemetry-emit-policy-rebuild-spec.md)

- **Actor**: Bench/CI operator running `src/tests/bench/square_tour.py`.
- **Preconditions**: The tour's default corner mode (ANGLE-stopped MOVE,
  closed-loop on estimator heading) is used, `--sim` backend.
- **Main Flow**:
  1. Tour runs its self-calibration prelude, then drives 4 sides + 4
     ANGLE-stopped 90 deg corners.
  2. Each corner's commanded stop condition and the sim plant's true
     rotation must agree to within the tour's stated heading/closure
     bounds — today they disagree by a systematic ~13 deg
     (45→56.8, 90→103.0, 180→193.0 measured against plant truth), which
     is what drives the ~288 mm closure failure.
- **Postconditions**: `square_tour.py --sim` exits 0 within its stated
  heading/closure bounds; the same tour also passes against the bench
  robot on the stand.
- **Acceptance Criteria**:
  - [ ] `uv run python src/tests/bench/square_tour.py --sim` exits 0.
  - [ ] `uv run python src/tests/bench/square_tour.py --port <bench
        port>` exits 0 on the stand.
  - [ ] The fix is either a calibration-value correction
        (`data/robots/tovez_nocal.json`'s `rotation_gain`/
        `rotation_offset_deg`) or a sim-plant kinematic fix, whichever
        ticket 007's own investigation confirms is the true root cause —
        not assumed in advance by this planning pass.

## GitHub Issues

(None linked — this sprint's issue is a CLASI issue file, not a GitHub
issue.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning document is complete (sprint.md, including its
      Architecture and Use Cases sections)
- [x] Architecture review passed (or skipped, for changes with no
      architectural impact)
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Devices::Motor velocity() two-sample floor (Part 2) | — |
| 002 | Delete arming machinery; rebuild three-mode emit predicate (Parts 1, 3) | 001 |
| 003 | TLM command surface: TLM:ON/AUTO/OFF/NOW, STATUS tlm=, HELP, NezhaProtocol (Part 4) | 002 |
| 004 | Flags-word State/Freshness/Event documentation (Part 5) | 002 |
| 005 | Bench-script migration to TLM:ON/OFF bracketing (Part 7) | 003 |
| 006 | Part 8 acceptance: sim criteria 1-12 + bench criteria 13-16 | 001, 002, 003, 004, 005 |
| 007 | [Adjacent] Sim-plant rotation calibration for ANGLE-stop MOVE overshoot | — |

Tickets execute in dependency order; 007 has no dependency on the
telemetry chain and may run in parallel with it, but its acceptance
(square tour passing in sim) is verified alongside ticket 006's bench
gate before the sprint closes.
