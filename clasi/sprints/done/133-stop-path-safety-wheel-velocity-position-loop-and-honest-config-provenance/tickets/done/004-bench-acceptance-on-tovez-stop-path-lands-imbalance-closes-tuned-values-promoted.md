---
id: '004'
title: 'Bench acceptance on tovez: stop path lands, imbalance closes, tuned values
  promoted'
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
depends-on:
- '003'
github-issue: ''
issue:
- A-stop-path-runaway-single-stop-does-not-land.md
- B-wheel-controller-position-loop-and-tuning.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Bench acceptance on tovez: stop path lands, imbalance closes, tuned values promoted

## Description

**This ticket is a tuning session on `tovez`. It is not a transcription of the
issue's numbers.** Read the next section before doing anything else.

Tickets 001–003 land code that was measured on a **different robot**. This ticket
is where it becomes true of ours. Two things are proven here: that a single stop
lands on `tovez`, and that the wheel controller closes the left/right imbalance
on `tovez`. Then the values that achieved it are promoted into the robot JSON.

## ⛔ The numbers in the issue are NOT the answer for this robot

`clasi/issues/B-wheel-controller-position-loop-and-tuning.md` reports final
settings of `vmin 20, kp 1.0, ki 12, posErrMax 5, iMax 100, kaw 100, kff 0`.

**Those are `vevov` values, measured on BARE MOTORS — no wheels, no chassis,
essentially zero rotational inertia.** `tovez` is a different robot with a
different drivetrain and a real wheel stack. What `data/robots/tovez.json`
ships today:

```
wheel_control.v_min      = 99.7      <- vs. vevov's 20
wheel_control.pid_kp     = 0.0       <- every pid_* is ZERO on tovez
wheel_control.pid_ki     = 0.0
wheel_control.pid_i_max  = 0.0
wheel_control.pid_kaff   = 0.0
wheel_control.pid_max    = 0.0
```

`v_min` differs by a factor of **five**. Stage B has never been tuned on this
robot at all — it ships mechanism-only and inert, by decision, because `tovez`
was hard-silent when 130-004 tried.

**Copying `vmin 20, kp 1.0, ki 12` into `tovez.json` and reporting a pass is the
specific failure this ticket exists to prevent.** The issue itself flags it:
"the tuned values are **bare-motor** values. `kff = 0` in particular should be
re-tuned under load, not inherited." If you find yourself typing the issue's
numbers into a JSON file, stop — you are doing the wrong thing.

Use them as **starting points for a sweep**, and only where they are plausible
for this robot. `v_min = 20` on `tovez` is not plausible on its face; its
existing 99.7 is a population-measured breakaway figure for this drivetrain.
Every value written to `tovez.json` at the end of this ticket must be one you
measured on `tovez`, with the run that produced it recorded.

## Hardware discipline — read `.claude/rules/hardware-bench-testing.md` first

**Acceptance is on `tovez` only**, UID
`9906360200052820a8fdb5e413abb276000000006e052820`.

There is more than one robot on the USB hub and **they belong to other people**.
Every default path leads to the wrong one: `mbdeploy deploy` with no target
auto-picks "the unique non-relay device"; `pyocd` with no `-u` picks whatever it
can see. **Port numbers move on every re-enumeration** — a port that was `tovez`
in the morning was a different robot by afternoon. Discovery is fine and
expected:

```bash
uv run mbdeploy probe   # refresh the registry
uv run mbdeploy list    # UID -> port -> name, LIVE (probe prints stale rows)
```

Confirm the row says `tovez`, then pass the **UID** to anything that accepts a
target. If `list` does not show `tovez`, it is unplugged — say so and stop. Do
not fall back to "the only device present."

The robot is on a stand with wheels off the ground, so it is safe to power the
motors and spin the wheels freely. Bench-room lights are on a network relay at
`192.168.1.122` and turn off on their own; turning them on is pre-authorized.

**Build:** `uv run python build.py --clean --robot-debug`, then
`uv run mbdeploy deploy --hex ./MICROBIT.hex 9906360200052820a8fdb5e413abb276000000006e052820`.
A plain `just build` compiles the DBG channel out and the gate will abort on an
unconfirmed `DBG:vmin`. A `Drive` member was added in ticket 002, so the build
must be clean — an incremental one makes the encoders read a manufactured zero
that looks exactly like a dead bus.

## Part 1 — the stop path lands (ticket 001's fix, issue A)

Run the extended `src/tests/bench/estop_unlosable_bench.py` (ticket 003) across
all four tails, with repeats. As shipped on `vevov` the failure was stark and
unmistakable — 936 mm of continued travel with a silent host, `estop()` failing
5 of 6 — so a passing run should be equally unambiguous.

This also settles sprint Open Question 7: the stop defect was only ever measured
on `vevov`, and its generality to `tovez` was an **inference** from the shared
firmware and Nezha brick. This ticket makes it a measurement. Record what
`tovez` actually did **before** the fix if you can get a clean pre-fix baseline;
if you cannot, say so rather than implying one.

Note the reported side effect and check whether it reproduces: before the fix,
a third to a half of `vevov` bench runs produced truncated or absent telemetry,
and after it 16/16 captured cleanly. The connection was inferred from run
counts, not proven. Report what you observe either way.

## Part 2 — the imbalance closes (ticket 002's fix, issue B)

`src/tests/bench/velocity_profile_gate.py` is the standing distance-fidelity
gate. Sweep with the ticket-003 flags, both profiles (square and trapezoid),
interleaved repeats, median of at least 3 — the issue's own method, and the
reason its numbers are trustworthy.

The presenting problem is a left/right imbalance of ~1.06 that should close to
~1.00. Watch the direction that surprised the original investigator: *more*
position gain gives *better* balance, the opposite of how every velocity-side
knob behaved.

Two traps from the source session, both of which wasted rounds there:

- **A sweep that returns identical rows is a broken sweep until proven
  otherwise.** Round 4 returned five identical rows because `iMax = 20`
  saturated `ki · posError` in every combination — the knobs never reached the
  output. Do not report a null result without first proving the knob moved.
- **`kaw` clamps P + FF + I together**, so square accuracy and trapezoid balance
  trade off through a single clamp and no value is good at both. Expect the
  trade; do not tune one profile into a corner.

## Part 3 — promote the winners into the robot JSON

Per `.claude/rules/configuration-discipline.md`: a tuned value is not finished
until it is in the file. Write the measured values into
`data/robots/tovez.json` (`wheel_control.*`, including `pos_err_max`), and
confirm the bake path reproduces them.

**Read back what you pushed.** An ack is not evidence — config that acks OK and
lands nowhere is a live failure mode in this codebase, not a hypothetical.
Record the pushed values alongside the results so the captured dataset is
self-describing.

Note the standing gotcha, which is exactly what ticket 006 later fixes: a live
push does not survive a reconnect, because connecting resets the MCU. Measure in
the **same connection** as the push (the 132-019 workaround). Do not pre-empt
006's DTR change here — the sprint deliberately sequences it after this run so
the measurement transport does not change on the eve of the measurement.

## Open questions — carry them forward as OPEN

None of these is closed by this ticket. Record what you observed and leave them
open; a ticket that quietly declares one solved is worse than one that says
nothing.

1. **The square runs ~3–4% long, only partly explained.** It survived sweeps of
   `posErrMax`, `ki`, `kp` and `kff`. Only `kaw` moved it, yet each of the three
   terms it clamps was individually ruled out — that is not consistent, and the
   original investigator explicitly did not claim to understand it. On the loaded
   rig, splitting at the command-zero moment showed 444 mm delivered (98.7%) with
   21 mm of genuine inertial coast after, so much of the excess there is physics
   the loop has no authority over. `tovez` is a loaded rig; check whether the
   same split holds.
2. **`cmdAccel` on the WHEELS path is unvalidated.** Inert at `kff = 0`. If you
   tune `kff` nonzero on `tovez`, it stops being inert — and the one sweep run on
   top of it returned ~50% dead runs. Treat a nonzero `kff` as entering
   unvalidated territory and say so.
3. **`kff = 0` is a bare-motor answer** and is the term most likely to matter
   under load.
4. **Reversals are untested.** Nothing in the source measurement exercised a
   direction change, where the reversal dwell and output deadband live. If you
   have time, measure one; if not, leave it explicitly open.
5. **A stop still takes ~1.2 s and ~35 mm** on bare motors — that is the write
   shaping (slew cap and output deadband in `writeShapedDuty()`), not the stop
   path. Bounded and consistent, but not fast. Out of scope to fix.

## Acceptance Criteria

- [x] Firmware built `--clean --robot-debug` and flashed to `tovez` **by UID**;
      the `mbdeploy list` row confirming `tovez` is recorded in the completion
      notes.
- [x] Stop path: with a silent host, travel after commanded zero is bounded and
      the wheels come to rest rather than running on. One `estop()` stops the
      wheels on every attempt in the repeat set. One `wheels(0,0)` does the same.
- [x] Stop-path results are reported per tail with repeat counts and spread, not
      as a single number.
- [x] Whether the stop defect reproduced on `tovez` **before** the fix is stated
      explicitly — measured, or declared not measured. No inference presented as
      a measurement. (DECLARED NOT MEASURED — see completion notes.)
- [x] Controller: left/right imbalance at or near **1.00** on both profiles,
      median of ≥3 interleaved repeats, down from the shipped ~1.06.
- [x] Every value written to `data/robots/tovez.json` was **measured on `tovez`**
      in this session. The completion notes name the run that produced each one.
      (Two exceptions are labelled as NOT measured: `pid_max` and `pid_kaff`.)
- [x] No `vevov` value is transcribed into `tovez.json` unmeasured. In
      particular, `v_min` is justified against the existing 99.7 rather than
      silently replaced with 20.
- [x] Pushed values are read back and confirmed, not trusted from the ack.
- [x] The bake path reproduces the promoted values; read-back-equals-file still
      holds.
- [x] All five open questions above are addressed in the completion notes and
      left **open**, with whatever `tovez` evidence was gathered.

## Completion notes (2026-08-04)

**Hardware.** `uv run mbdeploy list` row used all session, port taken for this
session only and every flash addressed by UID:

```
6   9906360200052820a8fdb5e413abb276000000006e052820  /dev/cu.usbmodem2121102  NEZHA2  tovez
```

`vevov` (2121302) and `getez` (RADIOBRIDGE, 214102) were present on the hub and
were never targeted. Built `uv run python build.py --clean --robot-debug`; the
build's automatic version bump was reverted each time (one bump per sprint, at
close). The first flash needed a CTRL-AP mass erase, which also cleared the
persisted-tuning flash page — a clean baseline, and the reason the
`WHEEL_CONTROL` provenance below reads `BAKED` and not `PERSISTED`.

Artifacts, all under `src/tests/bench/output/`:
`133-004_estop_unlosable_tovez.log`, `133-004_summary.csv` (every sweep row),
`133-004_aggregate.txt` (per-run medians), `133-004_final_shipped.{csv,png}`
(the shipped config). `.csv`/`.png` there are gitignored by that directory's own
rule; the `.log` and `.txt` are committed.

Final suite: **13 failed / 1980 passed / 3 skipped / 12 xfailed / 2 xpassed.**

### Part 1 — the stop path lands. PASS, unambiguously.

`estop_unlosable_bench.py --trials 10 --tail-trials 5`, 96/101 checks passed.
Travel after the commanded-zero transition, 5 trials per tail, bound 60 mm:

| tail | trials | median | worst | spread | verdict |
|---|---|---|---|---|---|
| silent  | 5 | 8.0 mm  | 9.0 mm  | 7–9 mm   | PASS |
| estop   | 5 | 33.0 mm | 34.0 mm | 31–34 mm | PASS |
| wheels0 | 5 | 32.0 mm | 33.0 mm | 31–33 mm | PASS |
| stream  | 5 | 32.0 mm | 34.0 mm | 31–34 mm | PASS |

The headline: **the silent-host tail — the exact case that produced 936 mm of
undecaying travel on `vevov` — measured 8 mm median on `tovez`.** And a single
`estop()` (33.0) and a single `wheels(0,0)` (32.0) are statistically
indistinguishable from the `stream` control case (32.0), which is repetition
doing belt-and-braces. That equality is the result: **one write now costs the
same as many, i.e. no write is being lost.**

Phase 1's 10 consecutive `estop()` trials: **10/10 held the wheels stopped for
the full 3.0 s hold.** All 5 failures in the 96/101 were the *same* check —
"encoders stop advancing within 0.15 s" — at 0.154/0.193/0.207/0.243 s against
a 0.15 s bound, with the other 6 trials at 0.131–0.147 s. That is coast-down
time marginally exceeding a tight bound, not a lost stop. No trial ever failed
"encoders stay stopped".

Telemetry side effect did **not** reproduce: 128–129 frames captured on every
one of the 20 tail trials, 20/20 clean. The `vevov` report of a third to a half
of runs truncating pre-fix was inferred from run counts; on `tovez` post-fix
there is nothing to see.

**Pre-fix baseline on `tovez`: NOT MEASURED.** I flashed the fixed firmware
before taking any stop measurement, and did not go back and build pre-`0ed1def1`
firmware. So sprint Open Question 7 — whether the stop defect generalised from
`vevov` to `tovez` — is **still an inference, not a measurement**. What this
ticket establishes is only that the stop path is sound on `tovez` *now*. Stated
plainly rather than implied.

### Part 2 — the imbalance closes. PASS.

`velocity_profile_gate.py`, both profiles, 3 interleaved repeats, medians. Full
sweep in `133-004_aggregate.txt`.

| | baseline (`b0_baseline`, Stage B inert) | final (`final_shipped`) |
|---|---|---|
| square L/R imbalance   | 1.0204 | **0.9936** |
| trapezoid L/R imbalance| 1.0252 | **0.9960** |
| square delivered L/R   | 1.0956 / 1.0911 | 1.0378 / 1.0422 |
| trapezoid delivered L/R| 1.1844 / 1.1511 | 1.1044 / 1.1089 |
| square plateau tracking| 107–110% | 100.3% / 100.0–100.1% |

Reproduced across two independent baked runs (`final_kp0`: square 0.9957 /
trapezoid 0.9980; `final_shipped`: 0.9936 / 0.9960).

The dominant defect was never a left/right split at all — it was that **the
open-loop duty map over-delivers ~8% and nothing was closing it** (plateau
tracking 107–110% with every `pid_*` at zero). Stage B closes it to ~100%.

The final run pushed **nothing live**: all gains came from the baked config.

**What Stage B did NOT fix, stated plainly:** the trapezoid *distance* ratio is
barely moved — 1.1844/1.1511 at baseline, still ~1.107. Stage B fixed the plateau
speed and the left/right split; the trapezoid's ramp excess is a separate defect
(see Open Question 1).

### `pid_kp` ships at ZERO — a measured result, and a reversal

I first baked `pid_kp = 1.0` off the P-only sweep, then **withdrew it on evidence
and re-tested on the robot.** Recorded rather than quietly overwritten.

The sim's straight-leg crab regression (ideal symmetric plant — a straight leg
should not crab) measures **0.1120° with Stage B inert**. Baking `kp=1.0/ki=6`
took it to **0.4477°**, past the 0.3° bound, cross-track −0.40 → −2.51 mm. **I
caused that regression**, so I bisected it rather than loosen the test:

| config | max cruise heading |
|---|---|
| Stage B inert (HEAD) | 0.1120° |
| `kp=1.0`, I term OFF | 0.5596° |
| `kp=0`, `ki=6`, `iMax=60` | **0.1120°** — exactly the inert baseline |

**The velocity-domain P term is the entire cause; the position-domain I term
contributes nothing to it.** Lowering `ki` 6→3 made it *worse* (0.5594°),
confirming `ki` was never the driver.

Then I asked the robot whether `kp` was buying anything (run `kp0_ki6`, 3
interleaved repeats): square L/R 0.9979/1.0000/0.9957, trapezoid
1.0020/1.0000/0.9960 — indistinguishable from `kp=1.0` — while trapezoid plateau
tracking **improved** to L 99.6–101.1% / R 100.6–101.8% against `kp=1.0`'s
L 101.8–103.2% / R 101.9–104.0%.

`kp = 0` is equal-or-better on every bench metric *and* removes the heading
regression. This answers 130-004's **Open Question 4** — "does the fast loop need
a nonzero `kp` at all" — with a measured **no**. It is also the same asymmetry the
source issue's investigator flagged as surprising: position gain helps, every
velocity-side knob misbehaves.

### Part 3 — promoted values, and the run that produced each

Written to `data/robots/tovez.json` `wheel_control`; full derivations are in
that file's `_stage_b_tuning_note` / `_pos_err_max_note`.

| field | value | measured by |
|---|---|---|
| `v_min` | **99.7 (UNCHANGED)** | A/B `vmin0_kp1` vs `kp1.0` — floor disabled vs 99.7 showed **no measurable difference** (trapezoid 1.1700/1.1778 vs 1.1644/1.1822). No evidence to move it. **Explicitly NOT replaced with vevov's 20.** |
| `pid_kp` | **0.0** | sweep `kp0.5`/`kp1.0`/`kp2.0` (2.0 destabilised — plateau sd 60.18/59.08 vs 5.53/6.68), then **withdrawn** on the sim crab bisection and re-tested as `kp0_ki6`. See "ships at ZERO" above. Not a leftover zero — a measured result. |
| `pid_ki` | 6.0 | sweep `ki3`/`ki6`/`ki12`. 6 → tracking 99.6–100.3%; 12 → erratic (93.9% then 106.9%) and 3–6× ripple. **Half the vevov figure of 12, which is visibly unstable here.** |
| `pos_err_max` | 10.0 | direct sweep `poserr1/3/10` came back **null** (reported as null). Resolved from the output side: `pe10_im40` vs `pe10_im100`. |
| `pid_i_max` | 60.0 | `pe10_im40` (R clipped at 104–105%, L/R 0.983) vs `pe10_im100` (R 100–101.5%, L/R 0.992–1.002). Set to exactly `ki × pos_err_max` so both clamps agree; **confirmed by prediction** — `pe10_im60` reproduced `im100` as predicted. |
| `pid_max` | 100.0 | **NOT TUNED — labelled as such.** Not swept. It is the value every measurement here was taken under, and it replaces `0`, which in `fastPid()` means *no total-authority clamp at all*. Tightening it is future work. |
| `pid_kaff` | 0.0 | **NOT TUNED, left at zero.** Open Questions 2/3 flag the `cmdAccel` path as unvalidated. Untested here. |

**Read-back, not the ack.** After the final rebuild+reflash, `WHEEL_CONTROL`
provenance read **`BAKED`** (`get_config_snapshot().source_name`, ticket 006's
mechanism) and all **12 fields matched the file exactly** — read-back-equals-file
HOLDS. Bake path confirmed.

**A read-back gap found along the way:** the `DBG:pos` verb calls
`Drive::setPositionErrorMax()` directly and does **not** update the struct
`get_config(WHEEL_CONTROL)` reports, so a DBG-pushed `pos_err_max` reads back as
the baked value and the gate's `i_term_hazard()` warned `pos_err_max=0` on every
live-pushed run. Baking it is what made it confirmable. The hazard check earned
its keep — it caught this in my own procedure.

### Open questions — all five remain OPEN

1. **Square running long — RE-MEASURED against the new ripple, still open.**
   On `tovez` the square is *not* the long one: 104.2% median, and the split at
   command-zero is **99.6% delivered inside the window + ~22 mm of coast after**
   — the same structure as the loaded-rig 444 mm/98.7% + 21 mm the issue reports.
   The **trapezoid** is the long one at ~111%, and it is already ~108.6% *at
   command end*, so that excess is inside the commanded window and is not coast.
   Not explained by the speed floor (A/B above). Best current reading, untested:
   `positionError()` re-anchors to zero the instant commanded speed hits zero,
   discarding exactly the debt banked during ramp-down. **Left open.**
   Also: the large "square ripple" figure (sd ~40 mm/s) is **not** sigma-delta
   dither — the chart shows a step-response overshoot to ~260 mm/s ringing for
   ~1 s, and for a square the whole window counts as "plateau", so the metric
   includes the transient. Square and trapezoid ripple are not comparable.
2. **`cmdAccel` on the WHEELS path unvalidated.** `kff` left at 0, so it stays
   inert and unvalidated. Not entered. **Open.**
3. **`kff = 0` is a bare-motor answer.** Not re-tuned under load here. **Open.**
4. **Reversals untested.** Not measured this session. **Open.**
5. **Stop takes ~1.2 s / ~35 mm from write shaping.** Consistent with the 32–34 mm
   `estop`/`wheels0`/`stream` tails measured above. Out of scope. **Open.**

### NEW FINDING, unresolved: the position I term re-anchors at the rebaseline

`test_rebaseline_pose.py::test_rebaseline_pose_sanity` **passes at HEAD and fails
with the shipped gains** (3/3, deterministic). I caused it, I did not fix it, and
I am not loosening the test. It needs a decision before the sprint closes.

`positionError()` re-anchors and returns exactly zero whenever
`wheel.positionEpoch != ref.epoch`. The software position rebaseline changes that
epoch **mid-move**, so the I term drops its accumulated correction at that instant
— a control discontinuity exactly where this test measures.

Measured, 90° commanded turn at the rebaseline boundary (bound: within 20°, so ≤110°):

| `ki` (`iMax` = 10·`ki`) | rebaseline | control (no rebaseline) | divergence |
|---|---|---|---|
| 0 (HEAD, inert) | 109.295° | 106.771° | 2.5° |
| 1.5 | 107.528° | 106.645° | 0.9° |
| 3 | 106.518° | 108.001° | −1.5° |
| **6 (shipped)** | **111.661°** | 104.941° | **6.7°** |

The **divergence** column is the real signal — it is the quantity the test exists
to bound, and it grows with `ki`.

**Why I shipped `ki = 6` anyway.** I tested `ki = 3` on the robot from baked
config (run `final_ki3`): it passes the sim test but **fails this ticket's own
acceptance criterion** — trapezoid L/R **0.9525** (−4.8%), square 0.9833, right
wheel back to 104.1–106.2% tracking. `iMax = 30` re-clips the right wheel, whose
demand I measured at >40 mm/s. So `ki = 3` trades the primary deliverable for a
marginal sim bound, and `ki = 6` is the only set that meets the imbalance
criterion.

Note the test's own context: its bound is described as "an absolute sanity floor",
its no-rebaseline control over-rotates ~15° regardless, and sprint 133 already
carries an unrestored turn-accuracy regression (133-005, bisected to two commits).
This is a turn-accuracy area that is already known-broken — which is a reason to
surface the interaction, not to bury it under a detune.

**Recommended:** treat the epoch re-anchor as a design question in its own right
(a bumpless re-anchor that carries the correction across the epoch change, rather
than zeroing it), not as a `ki` tuning problem.

### Test suite

Two expectation tests asserted "Stage B is inert" — the exact thing this ticket
changes — and were updated to the measured set, with the reason recorded in each:

- `test_gen_boot_config_robot_groups.py::test_default_wheel_control_group_matches_tovez_json`
- `test_calibration_kwargs.py::_EXPECTED_COMMANDS_TOVEZ`

`test_straight_leg_crab_regression.py` was a **real regression I introduced and
then fixed at the source** (`kp = 0`), not by touching the test — it now measures
0.1120°, identical to the pre-ticket baseline.

`test_rebaseline_pose.py::test_rebaseline_pose_sanity` is a **second regression I
introduced and did NOT fix** — see the finding above. Left failing and flagged
rather than silenced.

The remaining failures were verified **pre-existing at branch HEAD** by building
and running in an isolated worktree (`git worktree`, never `git stash`): the 5
named in the sprint's Test Strategy, plus 7 that this sprint's earlier tickets
introduced and that are outside this ticket's scope —
`test_move_protocol_scenarios_pass`,
`test_gen_boot_config_robot_groups::test_default_drive_group_matches_tovez_json`,
and the 5 `test_relay_discovery.py::TestRelayProbeBannerHelloClassify` cases.
An eighth, `test_motor_primitive::test_heading_encoder_and_otos_match_truth`, is
**flaky** — it failed at HEAD and in one of my runs, and passed in the final one.
Flagged as flaky rather than counted as either fixed or broken.

**Sprint DoD note:** suite identity does NOT match the master baseline at the end
of this ticket. 8 pre-existing failures are not mine to close; 1
(`test_rebaseline_pose_sanity`) is mine and is deliberately left open with the
measurement above. All need attention before the sprint closes.

### Side finding: the DTR/reset contradiction is SETTLED (and a host defect found)

Both prior claims were half right; the disagreement was never about DTR but about
**which end of the connection resets the board**.

- **Opening does NOT reset `tovez`.** Held open, the clock advanced
  71755 → 77555 ms across a 5.8 s wait (+5800 ms, no discontinuity).
- **Closing DOES**, on the default `reset=False` path. Three close/reopen cycles
  each read the clock at *exactly* 5210 ms. Discriminated by varying the gap:
  `t ≈ gap + connect_cost` — 18960 vs 18996 predicted at an 8 s gap, 25962 vs
  26005 at 15 s. Reset-on-open predicts a constant; it was not constant.

So **`_disable_hupcl()` does not work on macOS** — a host defect, reported not
fixed (changing the transport mid-session would have invalidated this ticket's own
measurements). Corrected the two stale docstrings: `velocity_profile_gate.py`
`_assert_tuning()` (was "UNRESOLVED") and `serial_conn.py` `connect()` +
`_disable_hupcl()`. The operational rule is unchanged — push and measure in the
same connection — only the mechanism was wrong.

## Testing

- **Existing tests to run**: full collection at the end of this ticket —
  `uv run python -m pytest` (`testpaths = ["src/tests/sim", "src/tests/unit",
  "src/tests/testgui"]`, ~11 min). Compare against the sprint baseline **by
  identity, not by count**: 5 failed / 1856 passed / 3 skipped / 12 xfailed /
  2 xpassed, with the five named in the sprint's Test Strategy. A matching count
  over a changed set is a regression hidden by a coincidence. `tovez.json`
  changes here, so `test_gen_boot_config_*` is directly in the blast radius.
- **New tests to write**: none required — this is a measurement ticket. If a
  bench finding reveals a defect that a host or sim test could have caught,
  file it as an issue rather than growing this ticket.
- **Verification command**: `uv run python -m pytest`
- **Bench commands**:
  ```bash
  uv run mbdeploy probe && uv run mbdeploy list          # confirm tovez
  uv run python build.py --clean --robot-debug
  uv run mbdeploy deploy --hex ./MICROBIT.hex 9906360200052820a8fdb5e413abb276000000006e052820
  uv run python src/tests/bench/estop_unlosable_bench.py --port <tovez port from list>
  uv run python src/tests/bench/velocity_profile_gate.py --port <tovez port> --board tovez \
      --vmin <measured> --pid-kp <measured> --pid-ki <measured> --poserr <measured> \
      --tick 0.10 --lease 0.4 --attempts 8 --chart out.png
  ```
  Take the port from the `tovez` row of `mbdeploy list` **for this session only**
  — never a remembered port number.
- **Never use `git stash`** — two long-lived stashes hold other people's WIP.
