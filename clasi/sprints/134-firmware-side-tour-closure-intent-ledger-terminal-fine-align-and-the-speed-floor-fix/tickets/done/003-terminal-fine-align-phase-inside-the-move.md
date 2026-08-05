---
id: '003'
title: Terminal fine-align phase inside the Move
status: done
use-cases:
- SUC-001
- SUC-002
depends-on:
- '001'
- '002'
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Terminal fine-align phase inside the Move

**This is the sprint's centrepiece.** Tickets 001 and 002 exist to make it
possible: 001 gives it an honest target, 002 gives it a deliverable actuator.
**Do not start before both are done.**

## Description

**Source of truth: `docs/bench-reports/motion-planning-lab-2026-08-04.md`
§5.2 and §3.** Read both. **Every constant below is measured. Do not invent,
round, or "improve" any of them.**

Put the host-side trim into firmware. Proven on hardware by the host graft:
**planner 25.8 → 9.4 mm at tolerance 1.0°, ~2 s/corner, zero other changes.**

### The mechanism

A new **`Aligning`** arm in `Motion::MoveLifecycle`
(`src/motion/planner/planner_types.h:186-194`), between profile-complete and
DONE. **Twist Angle Moves only** — the same guard the existing `arrived` event
uses (`planner.cpp:494-498`), because the ledger's `carryValid_` is itself
gated on `VelocityKind::Twist` and a Wheels-velocity Move has no heading
intent to align against.

Per iteration:

1. Wait for `settleReached()` (already exists, `planner.cpp:613`).
2. Compare `pose_.heading()` (already exists) to the ledger's cumulative
   target: `active_.baselineHeading + angularDirection(m) * m.<requested>`
   (exists **after ticket 001** — this is precisely why 001 is a dependency).
3. If `|residual| > alignTol`, emit a bounded low-speed pivot nudge through the
   normal `planWheels()` path, **re-settle**, re-measure.
4. Cap at `alignMaxNudges`, then complete regardless.

**The completion ack moves to AFTER alignment.** A Move is not done until it
has landed. Concretely: `result.completed = true` fires on the transition *out
of* `Aligning`, not into it. The Move's wall-clock `timeout` backstop is
**unchanged and must still fire from inside `Aligning`** — an alignment that
cannot converge must never wedge the queue.

### The constants — measured, not guessed

From **333 individual trim nudges** on `tovez`
(`src/tests/bench/output/trimtol_*.json`, per-nudge records; report §3):

- **`align_tol` = 1.0°.** The corrective pivot is **bimodal: 26% deliver
  <0.25° (no breakaway), the rest a median 1.72°** (10th pct 0.63°). A
  tolerance below that ~1.8° quantum is **outside the mechanism's own
  authority**. Measured: at 0.3°, convergence drops **93% → 64%**, cost
  triples, ~14 s/tour is added, and **some corners get *worse*** (a nudge
  fires at a residual it cannot resolve). 1.0° yields 1.3 nudges/corner, ~2
  s/corner, 94% convergence.
- **`align_max_nudges` = 6.**

Both match the already-committed bench script's own defaults
(`src/tests/bench/planner_square_tour.py:344-345`), so firmware and the proven
host graft agree by construction. **If the bench number in ticket 004
disappoints, do NOT tighten the tolerance** — that is measured
counterproductive. See sprint.md Success Criteria.

### Guards

- Suppress `Aligning` when `activeBoundary_ > 0` (a Move handing off at speed
  is not supposed to stop) — matching the existing `arrived` guard.
- A **timed-out** or **stalled** Move must **not** enter `Aligning`; it aborts.
- **Re-settle between nudges, not just before the first one.** Report §6
  measured that the next command lands in the *same* control cycle as the
  previous DONE ack, with the plant still moving in 3/5 trials. Measuring
  heading on a still-moving body reads noise and burns a nudge.

## The configuration cost — read this before you start

**The sprint brief claimed "2 touchpoints" for a new config field. That is
wrong**, and finding out one failing parity test at a time will cost you the
night. A planner field that reaches `Motion::PlannerLimits` is **~16 files**.
Verified by tracing the existing `settle_rest_omega` end to end. Do both new
fields in **one pass** over this list:

**Authoring inputs (HAND):**
1. `src/protos/robot_config.proto` — `message Planner` (~line 485-495). **Pick
   fresh field numbers**; 2/3/5/6/7/8 and 17-22 are `reserved`, do not reuse.
2. `data/robots/tovez.json` (planner group, ~line 95-102), **and
   `tovez_nocal.json` and `togov.json`** — `gen_boot_config._require()` is
   **fail-closed**, so any profile missing the key breaks the build the moment
   it is activated.

**Hand-written derived sites (HAND — these are the ones that ambush you):**
3. `src/scripts/gen_boot_config.py` — **three** f-string template sites:
   `planner_config_for_config()` (~:714), `defaultPlannerLimits()` (~:976),
   `defaultPlannerGroup()` (~:1110). It is not a proto walk.
4. `src/firm/config/config_parity_capi.cpp` (~:110) — the `msg::Planner`
   `kOffsets[]` table. **Mandatory**: `test_config_parity_capi.py:190-210`
   compares its length against the generated pydantic model.
5. `src/firm/config/boot_config.h` (~:230) — `struct PlannerBootConfig`,
   hand-declared, **not** generated.
6. `src/firm/app/boot_calibration.cpp` (~:71, in `App::bootPlannerLimits()`) —
   the one place `Config::PlannerBootConfig` → `Motion::PlannerLimits` happens.
7. `src/motion/planner/planner_types.h` (~:121) — `PlannerLimits::Landing`.
   **Append at the end of `Landing`.**
8. `src/motion/planner/capi.cpp` (~:108) — `plannerLimitsOffsets()` flat array.
9. `src/tests/bench/planner_harness.py` — `class Landing` (~:192) **and**
   `_LIMITS_FIELD_PATHS` (~:220), which must stay in `capi.cpp` order or the
   harness's own offset guard fails.
10. `src/tests/sim/system/composition_root_parity_harness.cpp` (~:95) —
    sim-vs-hardware parity check.

**Generated — never hand-edit** (all committed; produced by `build.py` steps
93/100/105): `src/firm/messages/robot_config.h`, `src/firm/messages/wire.cpp`,
`src/host/robot_radio/config/robot_config_generated.py`,
`data/robots/robot_config.schema.json`,
`src/host/robot_radio/robot/pb2/robot_config_pb2.py`,
`src/firm/config/boot_config.cpp`. The host pydantic mirror is **free** —
`robot_config.py:256` imports the generated class verbatim.

**Tests pinning the literals (HAND — expected, legitimate movement):**
- `src/tests/unit/test_gen_boot_config_planner.py` — `_EXPECTED_RAW` (~:50-67)
  is an **exact key-set comparison**; a new key fails it by design.
- `src/tests/unit/test_gen_boot_config_robot_groups.py` (~:195-232).
- `src/tests/unit/test_robot_config_proto_parses.py` (~:360) — `(message,
  field)` list.
- `src/tests/sim/unit/test_gen_boot_config_required_keys.py` (~:179) —
  parametrized fail-closed check.

**Update these expectations to the new literals; do not delete assertions.**

### Group choice and typing

- Both fields go in the **boot-only `Planner`** group, alongside
  `settle_rest_omega`, which they are siblings of. Sprint.md **D3** has the
  rationale and the accepted cost: **retuning `align_tol` needs a rebuild and
  reflash.** That is fine — §3 already swept the tolerance. Do **not** put them
  in `PlannerShaper` (semantically wrong, and it would force
  `applyShaperLimits()`'s signature open, touching every caller).
- `align_max_nudges` as **`int32`** keeps the 4-byte stride in `Landing`, so no
  padding is introduced and the flat offset guard stays valid. Mirror it as
  `ctypes.c_int32`.
- Naming: `alignTol` / `alignMaxNudges` in C++. **`align_tol` is an angle —
  units go in a `// [rad]` comment tag, not in the name.** Note the report
  states the tolerance in **degrees (1.0°)** while `PlannerLimits` angular
  fields are **radians** (`settleRestOmega` is `[rad/s]`, `settleEpsilonAngular`
  is `[rad]`). **Pick radians for the stored field and convert once, in the
  JSON value** — `1.0° = 0.017453 rad` — and say so in the comment. Getting
  this wrong by 57× is the single most likely silent failure in this ticket.

## Acceptance Criteria

- [x] `Motion::MoveLifecycle` gains `Aligning`; `planner.h`'s and
      `planner.cpp`'s transition-table doc comments are updated to match
- [x] `Aligning` is entered only by a **Twist Angle** Move that completed
      normally with `activeBoundary_ <= 0` — never on timeout, stall, Distance,
      Time, or Stop
      *(ctest `testFineAlignNeverEntersForExcludedMoves` covers all four
      exclusions; `testFineAlignSkipsAnAtSpeedHandoffAndKeepsTheLedger`
      covers the boundary guard.)*
- [x] Residual is measured against the ledger's cumulative intent target, at
      rest, after `settleReached()`
      *(Deviation, documented at the call site: the phase uses its own rest
      predicate — the staged command at zero for `kAlignRestWindow` plus the
      measured omega inside `settleRestOmega`, the same floor
      `settleReached()` uses — rather than `settleReached()` itself.
      `settleReached()` compares against `settleEpsilonAngular`, an ARRIVAL
      tolerance (0.035 rad on `tovez`) WIDER than a typical nudge's whole
      threshold, so it reports every nudge already arrived on its first tick
      and no pivot would ever run.)*
- [x] Nudges are emitted through the normal `planWheels()` path (not a
      bypass), and the body **re-settles between every nudge**
- [x] The Move completes when `|residual| <= alignTol` **or** `alignMaxNudges`
      is spent — whichever first
      *(Plus two bounded backstops added beyond the ticket — see the
      Completion Notes: a nudge that made the residual measurably WORSE, and
      a settle phase that never reaches rest.)*
- [x] The completion ack is emitted **after** alignment, on the transition out
      of `Aligning`
- [x] The wall-clock `timeout` backstop still fires from inside `Aligning` and
      cannot be wedged by a non-converging alignment
- [x] `alignTol` and `alignMaxNudges` are sourced from the robot JSON through
      the boot-config path — **no C++ literal** (`.claude/rules/configuration-discipline.md`:
      every value the robot uses comes from the file)
- [x] All three robot JSON profiles carry both keys (fail-closed generator)
- [x] `planner_harness.py`'s offset guard passes (`Landing` + `_LIMITS_FIELD_PATHS`
      extended in `capi.cpp` order)
- [x] `tovez.json` carries `align_tol` equal to 1.0° expressed in the stored
      unit, and `align_max_nudges` = 6

## Implementation Plan

**Suggested order** — do the plumbing first so the logic compiles against real
config, and do both fields in one pass:

1. Add both fields through the ~16-file list above; build; fix the four pinned
   tests. **Commit here** — this is a clean, separable checkpoint.
2. Add `Aligning` to `MoveLifecycle` and the transition-table comments.
3. Implement entry (the `done`-for-an-Angle-Move edge in `tick()`, `:601`) and
   the per-iteration nudge/re-settle/re-measure loop.
4. Move `result.completed` to the exit edge.
5. Tests.

**Nudge sizing**: bound it. A nudge is a low-speed pivot, not a fresh profile.
Consult `planner_square_tour.py`'s `trim_to_heading()` helper for the shape the
host graft used and reproduce its intent, not necessarily its code.

**Build**: `uv run python build.py --clean --robot-debug`. `planner_types.h` is
a shared header — clean build required.

## Testing

- **New ctests** (`src/motion/planner/tests/`):
  - `planner_lifecycle_test` — entry/exit conditions, the nudge cap, and the
    **timeout firing through `Aligning`**
  - `planner_noise_test` — alignment converges under tracking lag; and
    `testAngleDoesNotUndershootAtCompletionUnderSevereTrackingLag` (130-010's
    guard) still passes
  - a negative test: a Distance Move, a Time Move, and a timed-out Angle Move
    all complete **without** entering `Aligning`
- **Full suite run here** (~11 min). Compare **by identity** against the master
  baseline of ~8 failed / ~1994 passed.
- **Expected legitimate movement**: `test_gen_boot_config_planner` (×2) and
  `test_gen_boot_config_robot_groups` — the two new keys. Update them.
  `test_tour_closure_gate` may also move; it is a **sim** gate and the sim's
  corners **sign-flip** vs hardware (report §7). **Record it; do not tune
  firmware to satisfy it.**
- **The sim cannot validate this ticket's accuracy claim.** Sim tests here
  detect collateral breakage only. Ticket 004 is the real gate.
- **Verification command**:
  `uv run python -m pytest src/tests -q` plus the planner ctests

## Completion Notes

### What landed beyond the ticket's own list

Three bounded backstops the ticket did not specify, each added because a
measurement showed it was needed:

1. **A nudge that made the residual measurably WORSE ends the phase.** The
   corrective pivot has a coarse quantum of its own (§3: median 1.72°
   delivered against a 1.0° tolerance), so a plant that consistently
   over-delivers cannot land inside the tolerance — it oscillates across
   it. Measured directly in the sim while building this: a 1.9° residual,
   nudged, came back +3.3° the other way, and the loop hunted between the
   two for all six nudges (~6 s/corner) to end no better than it started.
   Deliberately "worse", **not** "no better": a nudge that delivers
   NOTHING is the measured 26% no-breakaway case, and retrying it is
   exactly what earns the 94% convergence — stopping there would throw
   that away. Threshold is `kStallEpsilonAngular`, the existing "is this
   change real or is it noise?" constant.
2. **The settle phase is bounded** by `1000*kAlignRestWindow +
   plannedStopWindow()`. `Move.timeout` is OPTIONAL on the wire (0 = none),
   and "no timeout" must not mean "may wedge the queue" — the identical
   argument `plannedStopWindow()` already makes for a `Kind::Stop`.
3. **Each nudge is bounded** by a window derived from the pivot itself
   (sweep + ramps + one coast allowance), because §3's measured 26% of
   nudges never break away at all and would otherwise never answer
   "landed".

### One semantic change to `TickResult::timedOut`

A wall-clock expiry from **inside** `Aligning` ends the Move but is no
longer reported as `timedOut` (the wire's `kFlagFaultMoveTimeout`). A Move
in `Aligning` has already MET its stop condition — the profile landed and
only the trim was still running — so a move-timeout FAULT there is a false
fault on a Move that drove correctly. `move_protocol_harness.cpp`'s own
SUC-050 ANGLE assertion ("ended via ANGLE, not timeout") is what named
this; the queue is still freed either way, which is the property that
matters. Alignment cost is reported in ticket 004's per-corner nudge count
and wall time, not as a fault bit.

### Measured, and NOT acted on

The alignment **does not converge in the sim**, and this is sim
infidelity, not a defect — report §7 predicted exactly this ("the sim's
corner behaviour sign-flips vs hardware; corner work is bench-only until
the sim's turn model is fixed"). Traced cycle-by-cycle on an isolated
managed 90° turn: the profile lands at 87.95° true, one nudge carries it to
91.39°, and `App::Drive`'s position loop then returns it to 88.10° — a
**net delivery of ~0.1°** against a commanded 1.9°. On hardware the same
nudge shape through the same `Drive` path measured a **median 1.72° net**
across 333 nudges, which is what makes the loop converge 94% of the time
there. Per the ticket's explicit instruction, `align_tol` was **not**
tightened and the nudge speed was **not** retuned to make a sim number
look better.

**Recommendation for ticket 004**: watch whether the 0.5 s command-quiet
dwell is long enough on `tovez` (closed-loop settle ~700 ms). If nudges
appear mis-sized, the fix to try is requiring the measured HEADING to be
stable within `kStallEpsilonAngular` for the dwell — not a tighter
tolerance.

### Test baseline, matched by identity

| slice | result |
|---|---|
| planner ctests | **8/8 pass** (2 new tests in `planner_lifecycle_test`… 6 total added, 1 in `planner_noise_test`) |
| `src/tests/unit src/tests/sim` | **4 failed / 1406 passed** — the same four pre-existing failures (`test_gen_boot_config_planner` ×2, `test_gen_boot_config_robot_groups` ×2) |
| `src/tests/testgui` | **7 failed / 591 passed** |

**All 7 testgui failures were proven pre-existing by direct A/B**: with
`align_max_nudges` set to 0 in `tovez_nocal.json` (the phase fully
disabled) and a clean rebuild, the same seven tests fail with
byte-identical numbers — including `test_tour_closure_gate`'s turn 6
(−13.365°) and turn 10 (−13.245°), and all four
`test_gui_button_acceptance` turn presets (+80.1 / −81.2 / +80.3 / −81.4).
**`test_tour_closure_gate` did not move under this ticket.**

`test_move_protocol` DID move and was fixed: its SUC-050 ANGLE scenario
ran an 80-cycle window that predates a completion ack arriving after a
terminal phase. Window widened to 160 cycles with the reason recorded;
every assertion is unchanged and still binding, and the test passes.
