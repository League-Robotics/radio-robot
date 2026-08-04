---
id: '005'
title: 'Turn-accuracy regression: A/B the distribution against the pre-132 merge-base'
status: done
use-cases:
- SUC-004
depends-on:
- '002'
github-issue: ''
issue: A-sprint-132-turn-accuracy-regression.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Turn-accuracy regression: A/B the distribution against the pre-132 merge-base

# Description

**This is a verification ticket first and a debugging ticket only if
verification fails.** Do not open with a bisect.

Sprint 132 shifted the sim turn-error distribution:

| | pre-sprint | post-132 |
|---|---|---|
| turn 2 | −2.6° | **−8.6°** |
| turns 6 / 8 / 10 | ~−12° | **~−21°** |

Early turns got better, later turns got substantially worse. Commanded 90°;
turns 10 and 12 now land at 69.37°. Ticket 132-018 established this is a genuine
regression rather than variance and ruled out two hypotheses, but **did not find
the cause**.

The stakeholder expects ticket 002's wheel-controller work to fix it, and that
expectation is plausible: this is sim-side (testgui runs against `SimLoop`), and
`App::Drive` runs in `SimLoop` too, so the position-domain I term changes sim
turn geometry directly. Hence `depends-on: 002` — run this **after** the
controller work has landed.

## Step 1 — re-run the A/B

Repeat 132-018's method, which is the reason its finding is trustworthy: build a
**second git worktree at sprint 132's pre-sprint merge-base commit** and run the
identical deterministic test there and at HEAD.

```
src/tests/testgui/test_tour_closure_gate.py::test_tour_1_and_tour_2_ninety_degree_turns_land_within_the_shaped_band
```

Record per-turn errors for turns **1–12** at both revisions, under the same run
conditions.

> **`git stash` is forbidden** — two long-lived stashes hold other people's WIP.
> Use a worktree, which is what 132-018 did and what this method requires anyway.
> See the worktree-verification approach: an isolated build and test alongside
> parallel WIP, trusting `git diff` over single file reads.

## Step 2 — judge the distribution, not the verdict

> **"Make it pass" is not the goal.** This test has a **pre-existing failure
> history** — it was already failing before sprint 132, which is exactly why the
> pass/fail tally did not move and the regression went unnoticed. It is one of
> the five named pre-existing failures in the sprint's test baseline.
>
> The goal is **restoring the pre-sprint error distribution** (roughly uniform
> ~−13° across turns), not turning the test green. A green test that got there
> by some other route would not answer the question.

If the distribution has returned to its pre-sprint shape: record the finding
with the per-turn table, attribute it to the controller work, and **stop**. That
is the successful outcome and it costs one ticket.

## Step 3 — bisect ONLY if it has not

If the distribution is still shifted, bisect across sprint 132's own commits
using the same worktree method. The commit range is small and each commit is
coherent.

Already ruled out with evidence, by 132-018 — do not re-litigate:

- **Config-value drift** — a 64-field mechanical diff of the reshaped robot JSON
  against its pre-sprint values came back clean.
- **The new `PLANNER_SHAPER` wire push** — output is byte-identical with it
  disabled.

Still open as candidates:

- **132-009** — the `duty_per_speed` sourcing change (it reversed the hardcoded
  `Drive::kDutyPerSpeed` in favour of the robot JSON; behaviour-preserving for
  `tovez` by construction, but the *path* changed).
- **132-017** — the Planner/PlannerShaper group split.
- The planner-limits reshape.

Both leading candidates touch values that feed turn geometry.

## Scope discipline

This ticket ends when the question is answered — restored, or attributed to a
specific commit. **Fixing** an attributed cause is a separate decision, and if
the fix is not small, file an issue and return the finding rather than growing a
sprint that is deliberately short.

Note the adjacent issue `B-rotation-calibration-vs-live-heading-hold-gain.md`,
which may share a mechanism. Do not adopt it here.

**Another agent may be concurrently triaging `clasi/issues/`.** Read issue files;
do not delete, move, or reorganize them.

## Acceptance Criteria

- [x] A worktree at sprint 132's pre-sprint merge-base is built and the
      deterministic tour-closure test run there and at HEAD, under the same
      conditions.
- [x] Per-turn errors for turns **1–12** are recorded at both revisions, as a
      table, in the completion notes.
- [x] The verdict is stated against the **distribution** — restored to roughly
      uniform ~−13°, or not — and explicitly **not** against the test's
      pass/fail, which has a pre-existing failure history.
- [x] If restored: the finding is recorded and attributed, and no bisect is
      performed. *(N/A — not restored; the bisect below was performed instead.)*
- [x] If not restored: a bisect across sprint 132's commits identifies the
      responsible commit, without re-testing the two hypotheses 132-018 already
      ruled out.
- [x] `git stash` was not used at any point.
- [x] Any fix beyond a trivial one is filed as an issue rather than implemented
      here. *(No fix implemented. Two follow-ups named below for filing —
      `clasi/issues/` was off-limits to this ticket, another agent was
      triaging it concurrently.)*

## Completion Notes

### Method

Three git worktrees off this repo, each with its own `src/sim/build`
configured exactly the way `build.py` configures it (`cmake -S src/sim -B
src/sim/build`, no `CMAKE_BUILD_TYPE` — a first pass with `-DCMAKE_BUILD_TYPE=
Release` produced different float results and was discarded, so every number
below comes from a build matched to the main checkout's). Each needed
`src/scripts/gen_version.py` run first: `src/firm/types/version_generated.h`
is gitignored, so a fresh worktree cannot compile without it.

`src/tests/testgui/test_tour_closure_gate.py` is **byte-identical** between
the pre-132 merge-base and HEAD (`git diff` over that path is empty), so the
A/B drives the same code path at every revision. The measurement harness
called that file's own `_make_loop()` / `_run_tour_capture()` directly,
minus the assertion that aborts the run at the first over-tolerance turn —
otherwise only the first bad turn of the first tour is ever visible.

No `git stash` at any point. All three worktrees removed at the end.

### The three distributions — `TOUR_1/ideal`, commanded +90° every turn

Errors in degrees, sim ground truth (`SimLoop.get_true_pose()`):

| turn | pre-132 `2cb43292` | post-132/pre-133 `9499f286` | HEAD `eb64865d` |
|---|---|---|---|
| 2 | −2.57 | −27.84 | −25.27 |
| 4 | −13.08 | −31.75 | −24.67 |
| 6 | −12.42 | −53.84 | −47.82 |
| 8 | −11.96 | −38.93 | −35.05 |
| 10 | −13.48 | −51.90 | −51.18 |
| 12 | −24.70 | −29.13 | −28.42 |
| **worst \|err\|** | **24.70** | **53.84** | **51.18** |
| closure | 17.4 mm | 119.3 mm | 130.0 mm |

`TOUR_2/ideal` (mixed angles), same three revisions:

| turn | commanded | pre-132 | post-132 | HEAD |
|---|---|---|---|---|
| 2 | +90 | −2.57 | −27.84 | −25.27 |
| 4 | +124 | −13.29 | −32.74 | −25.44 |
| 6 | −217 | −14.33 | −50.75 | −44.31 |
| 8 | +146 | +5.19 | −20.10 | −26.25 |
| 10 | +215 | −16.67 | −42.98 | −51.95 |
| 12 | −90 | −8.15 | −32.78 | −33.83 |
| 14 | −90 | +12.78 | +9.36 | +7.84 |

Both **realistic**-profile runs are measurable only at pre-132 — see the
second finding below. Recorded for completeness:
`TOUR_1/realistic` pre-132: −0.12, −18.56, −4.60, −14.60, −16.08, −13.45.
`TOUR_2/realistic` pre-132: −0.12, −18.66, −2.90, +14.07, −20.89, −13.59, +14.38.

### Verdict — NOT restored

Judged against the **distribution**, not the pass/fail. HEAD is
indistinguishable from post-132 (worst 51.18° vs 53.84°; the ~2° of movement
is ticket 001/002's own, and it is noise at this scale). The pre-132 shape —
turn 2 near −2.6°, the middle turns clustered around −12°/−13° — is not back.
Ticket 002's controller work did **not** fix this, and the stakeholder's
expectation that it would is not borne out. `iMax = 0` on every shipped robot
means Stage B's I term is inert, so 002's change had nothing to act on here;
that reading is confirmed, not assumed.

### Bisect — two separate steps, not one

Probing sprint 132's own commits in a worktree. Commits `bb305ff5` (132-009)
through `4125c0b6` (132-014) **cannot run this test at all** mid-sprint
(`MissingRobotConfigKeyError: control.vel_kp`, then a tour that captures zero
turns), so the range is not finely bisectable; `63f3039d` is the first
runnable commit after the schema cutover.

| revision | | worst \|err\| | turn 2 | turns 6/8/10 |
|---|---|---|---|---|
| `2cb43292` | pre-132 merge-base | 24.70 | −2.57 | −12.42 / −11.96 / −13.48 |
| `63f3039d` | **132-017** JSON reshape | 21.80 | −8.63 | −21.80 / −21.71 / −20.63 |
| `66060662` | 132-018 | 21.80 | −8.63 | unchanged |
| `ad763448` | **132-019** bench acceptance | 53.84 | −27.84 | −53.84 / −38.93 / −51.90 |
| `9499f286` | post-132/pre-133 | 53.84 | −27.84 | unchanged |

`63f3039d` reproduces the issue's own reported numbers **exactly** (turn 2
−8.63 vs the issue's −8.6; turns 6/8/10 ~−21; turns 10/12 landing at 69.37°).
That is the regression 132-018 measured. What 132-018 could not see is that
`ad763448` — committed *after* it — roughly doubled the damage again.

### Root cause of the dominant step (`ad763448`, 132-019) — proven, reversible

132-019 fitted `tovez`'s per-wheel drive correction on the bench and baked it:
`src/firm/config/boot_config.cpp`'s four `wheel_gain_*` values went from
identity `1.0` to `0.9075` (left) / `0.8` (right).

Reverting **only those four values to 1.0f** at HEAD, rebuilding, and re-running:

| turn | HEAD | HEAD, gains reverted | (`63f3039d` for reference) |
|---|---|---|---|
| 2 | −25.27 | −7.97 | −8.63 |
| 4 | −24.67 | −14.05 | −13.60 |
| 6 | −47.82 | −17.05 | −21.80 |
| 8 | −35.05 | −22.10 | −21.71 |
| 10 | −51.18 | −22.02 | −20.63 |
| 12 | −28.42 | −19.39 | −20.63 |
| **worst** | **51.18** | **22.10** | **21.80** |

The whole `ad763448` step reverses cleanly. The mechanism is documented — in
the codebase, as a warning, by the very function that was supposed to prevent
it. `drive_boot_config_for()` (`src/host/robot_radio/calibration/
sim_boot_config.py`) deliberately withholds the wheel correction from the sim:

> "is deliberately NOT installed in the sim: it is a LINEARIZATION of one
> physical drivetrain's gearbox … `TestSim::WheelPlant` is a plain first-order
> linear plant with none of the nonlinearity it corrects for. Installing it
> against that plant does not cancel anything — it just divides every
> commanded speed by its own gain … and bends a straight leg, since the left
> and right gains deliberately differ. An identity correction (`Drive`'s own
> default, gain 1 / intercept 0) is the faithful choice."

That reasoning is still correct and still enforced on the host push path. It
is defeated from the other side: 130-002 unified the sim and hardware
composition roots, so `App::composeRobot()` installs the wheel correction from
`Config::boot_config` — the **bake**, generated from `data/robots/tovez.json` —
for the sim too. The test selects `tovez_nocal.json`, whose `wheel_gain_*`
are held at identity *on purpose*, and never gets them: `calibration_kwargs()`
does not push `DRIVE` fields.

This was invisible until now only because the bake happened to hold identity,
so "`Drive`'s own default" and "the bake" agreed. `ad763448` ended that
coincidence, and the sim silently began running one physical robot's warm
gearbox linearization — with a deliberate 13% L/R asymmetry — against a plant
that has no asymmetry at all.

**Not fixed here.** The fix is a real change (mirror the wheel correction into
`sim_configure_drive()`/`SimHarness`, or make the sim composition root fall
back to identity for it), which is past this ticket's scope bar.

### The residual step (`63f3039d`, 132-017) — attributed to the commit, mechanism still open

Four hypotheses tested and **refuted**, each by rebuild-and-measure, none of
them re-litigating what 132-018 already ruled out:

1. **Shaper ceilings changed source.** `estimator_kwargs()` moved from
   `config.control.*` (a_max 800, a_decel 800, alpha_max 7.0, alpha_decel 7.0,
   j_max 5000, yaw_jerk_max 100) to `config.planner_shaper.*` (300, 250, 6.0,
   5.0, 1500, 30) — genuinely different values, and the reason 132-018's
   "byte-identical with the push disabled" result was a true observation about
   an inert push rather than about the mechanism. Pushing the old values at
   HEAD changed the output but did **not** restore the shape (worst 52.85).
2. **Baked rotation calibration.** `63f3039d` began baking `tovez`'s
   camera-fitted `rotation_gain_pos/rotation_offset/...`. Setting them to `99.0`
   produced byte-identical output — Tier-2's `sim_configure_drivetrain()` push
   from the selected JSON overrides the bake. Inert.
3. **Baked `rotational_slip`.** Same test, same result: inert.
4. **Motor velocity-PID gains** (`vel_kp` 0.0016 / `vel_kff` 0.0008 /
   `vel_kaw` 20.0 baked from `tovez.json` vs `tovez_nocal.json`'s
   0.002/0.002/0.0). Restoring the nocal values in the bake: byte-identical.
   Tier-1's `pid.*` push does reach them.

Stopped at four per the attempt cap. The residual is ~8° of the ~30° total and
is attributed to the commit; the field within it is open.

### Second finding — half this gate has been crashing silently since 132-014

`4125c0b6` (132-014) deleted `NezhaProtocol.otos_config()`. The test's
`_make_loop(realistic_errors=True)` still calls it, so **every
realistic-profile run in this module raises `AttributeError` during setup** and
has since sprint 132. It is invisible from the outside for two reasons: the
`ninety_degree_turns` test asserts on `TOUR_1/ideal` and never reaches the
realistic legs, and the four surrounding tests are
`xfail(strict=False)` — so a setup crash is reported as `xfailed`, under an
xfail reason that claims a turn-accuracy shortfall. The module reports
`1 failed, 5 xfailed` at HEAD either way, which is why the count never moved.

Two of the five per-turn distributions this gate is supposed to measure have
not actually been measured since 132-014.

### Test baseline

No code changed in this ticket; it is verification-only. The failing set is
unchanged: `src/tests/testgui/test_tour_closure_gate.py` reports
`1 failed, 5 xfailed` at HEAD, the same shape as before, with
`test_tour_1_and_tour_2_ninety_degree_turns_land_within_the_shaped_band` the
one hard failure.

### Follow-ups to file (not filed here — `clasi/issues/` was in use)

1. **The sim inherits `tovez.json`'s baked wheel correction** — `composeRobot()`
   installs `Config::boot_config`'s `wheel_gain_*` in the sim, defeating
   `drive_boot_config_for()`'s deliberate exclusion. Owns the −30° step.
   Every future bench fit baked into `tovez.json` will move the sim again.
2. **`otos_config` deletion left both realistic-profile tour runs crashing**,
   masked by `xfail(strict=False)`. Either restore the call against the new
   config surface or make the module fail honestly.

## Testing

- **Existing tests to run**:
  `uv run python -m pytest src/tests/testgui/test_tour_closure_gate.py` at both
  revisions. The test is deterministic, which is what makes the A/B meaningful.
- **New tests to write**: none. This ticket measures an existing test; it does
  not add coverage. If the root cause turns out to be something a unit test
  could pin, note that in the completion notes as a follow-up.
- **Verification command**: `uv run python -m pytest src/tests/testgui/test_tour_closure_gate.py`
- **Baseline note**: this test is one of the sprint's five named pre-existing
  failures. If this ticket changes the failing **set**, name every entry that
  moved and why — a matching count over a changed set is a hidden regression.
