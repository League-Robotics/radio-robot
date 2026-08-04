---
id: '004'
title: 'Bench acceptance on tovez: stop path lands, imbalance closes, tuned values
  promoted'
status: open
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

- [ ] Firmware built `--clean --robot-debug` and flashed to `tovez` **by UID**;
      the `mbdeploy list` row confirming `tovez` is recorded in the completion
      notes.
- [ ] Stop path: with a silent host, travel after commanded zero is bounded and
      the wheels come to rest rather than running on. One `estop()` stops the
      wheels on every attempt in the repeat set. One `wheels(0,0)` does the same.
- [ ] Stop-path results are reported per tail with repeat counts and spread, not
      as a single number.
- [ ] Whether the stop defect reproduced on `tovez` **before** the fix is stated
      explicitly — measured, or declared not measured. No inference presented as
      a measurement.
- [ ] Controller: left/right imbalance at or near **1.00** on both profiles,
      median of ≥3 interleaved repeats, down from the shipped ~1.06.
- [ ] Every value written to `data/robots/tovez.json` was **measured on `tovez`**
      in this session. The completion notes name the run that produced each one.
- [ ] No `vevov` value is transcribed into `tovez.json` unmeasured. In
      particular, `v_min` is justified against the existing 99.7 rather than
      silently replaced with 20.
- [ ] Pushed values are read back and confirmed, not trusted from the ack.
- [ ] The bake path reproduces the promoted values; read-back-equals-file still
      holds.
- [ ] All five open questions above are addressed in the completion notes and
      left **open**, with whatever `tovez` evidence was gathered.

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
