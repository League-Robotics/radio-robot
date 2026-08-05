---
id: '004'
title: 'Bench acceptance on tovez: firmware-only square-tour closure'
status: done
use-cases:
- SUC-001
- SUC-003
depends-on:
- '003'
github-issue: ''
issue: ''
completes_issue: true
exception:
  thrown_by: programmer
  thrown_at: '2026-08-05T11:07:15.965707+00:00'
  attempted: 'Full bench acceptance on tovez, 2026-08-05. Clean --robot-debug build
    from sprint tip 10b7e13e, flashed by UID; align_tol=0.017453 rad and align_max_nudges=6
    read back off the board before any measurement. Ran the gate (planner_square_tour.py
    --sequential, no --trim, no --turn-scale) 14 times. Median closure 35.1 mm (mean
    36.3, range 19.7-48.9). BOTH bars FAIL: sprint acceptance <=8 mm FAIL; mechanism
    acceptance <=12 mm FAIL, >=90% corners inside align_tol FAIL (16%, 9/56), >=3x
    on the 64.1 mm baseline FAIL (1.82x). Also built and flashed the pre-sprint control
    (8aeb3a4a) and ran an interleaved A/B, reflashing between every run with a per-run
    firmware-identity read-back: the CONTROL closes better in 6 of 6 identity-verified
    pairs on the gate invocation (mean delta +19.7 mm). The 64.1 mm premise number
    does not reproduce this session -- pre-sprint firmware closes 8.5-42.5 mm on the
    same invocation. Ticket 002''s speed floor verified on hardware, both halves (teleop
    10 mm/s -> 21.0 measured, boosted; planner Move at 10 mm/s -> 8.6 measured, unboosted;
    DBG:vmin 0 control confirms the boost is the floor). Ticket 003''s sim non-convergence
    risk resolved: on hardware a nudge delivers -1.32 deg net and post-ack drift is
    +0.00 deg, so the mechanism itself works. Test baseline matched by identity: 4
    failed/1406 passed (unit+sim) and 7 failed/591 passed (testgui), both identical
    to ticket 003. No file under src/ was modified and no constant was touched. Full
    data: docs/bench-reports/sprint-134-004-bench-acceptance-2026-08-05.md.'
  conflict: 'sprint.md''s Solution section asserts the property the whole design rests
    on: "A Distance leg''s intent is zero heading change, so the ledger carries its
    heading baseline forward unchanged; the leg''s -1.43 deg curl therefore shows
    up as residual at the NEXT corner, where the align phase drives it out against
    the absolute cumulative target." That property does not hold in the arm the gate
    measures. Planner::activateNext() opens with `carryValid_ = carryValid_ && pendingCount_
    > 0;` (planner.cpp:867, pre-existing since planner v1 4aea58c1 -- sprint 134 neither
    introduced nor touched it), so the cumulative-intent ledger survives ONLY when
    the successor Move is already queued at the predecessor''s completion. A --sequential
    tour waits for each completion ack before sending the next Move, so pendingCount_
    == 0 at every boundary and the carry is dropped every time; every Move re-anchors
    to pose_.heading(). Ticket 001''s restored intent-carry is therefore inert in
    the gate arm. Measured directly (sprint134_ledgerprobe.json): identical leg+turn,
    differing only in whether the turn was queued before the leg''s completion --
    PAIRED (carry live) lands -0.38 deg mean residual, 4/4 corners inside align_tol;
    SEQUENTIAL (carry dropped) lands +1.22 deg mean, 2/4. Worse, with the ledger dead
    the align phase is actively harmful: it drives each turn onto its OWN activation
    heading + 90 deg, removing the pre-sprint firmware''s ~+0.7 deg/corner open-loop
    over-rotation that was partially cancelling the -1.35 deg/leg curl. Per-turn delivery
    sprint +90.10 deg vs control +90.74 deg with identical leg curl, which is exactly
    the 6-of-6 regression. Confirmation that the design is otherwise sound: in the
    PIPELINED arm, where the queue is never empty and the ledger does survive, the
    sprint firmware beats the control 3 of 3 (mean delta -11.2 mm; 2.2/8.1/13.2 vs
    5.4/24.2/27.5). Resolving this is a design decision above this ticket''s authority
    and outside its explicit instruction not to tune: deleting `&& pendingCount_ >
    0` is not obviously safe (the line exists so an unrelated later Move cannot adopt
    a stale heading baseline, and this very tour drives teleop wheels(0,0) between
    segments -- the case it guards). Two candidate directions, neither attempted:
    (a) give the carry a validity rule that survives an idle gap, e.g. valid while
    the body has not moved since it was recorded, checkable inside Planner from pose_
    alone with no new dependency; (b) suppress the Aligning phase when carryValid_
    was false at activation, which would at minimum make the sprint neutral rather
    than negative on the gate arm. Stakeholder decision needed on which, or on whether
    the gate should be the pipelined arm instead.'
  surface: user-visible
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Bench acceptance on tovez: firmware-only square-tour closure

**This ticket is the sprint's acceptance gate.** The firmware closes the tour
alone, or it does not.

## STOP — address the robot by UID, never by port

There is more than one robot on the USB hub, and **every default path leads to
the wrong one**. Ports moved **twice** during the 2026-08-04 session.

```
tovez  9906360200052820a8fdb5e413abb276000000006e052820   <- OURS
vizev  99063602000528205560754f2f401c2f000000006e052820   <- NOT OURS. Never touch.
```

```bash
uv run mbdeploy probe    # refresh the registry
uv run mbdeploy list     # live view: UID -> port -> name. `probe` prints STALE rows.
```

Confirm the row says `tovez`. Take the port from **that row, for this session
only**. Pass the **UID** to anything that accepts a target. If `mbdeploy list`
does not show `tovez`, it is **unplugged** — stop and say so. Do not fall back
to "the only device present".

The robot is on a stand, wheels off the ground. It cannot drive away; drive it
freely.

## The gate

```bash
uv run python src/tests/bench/planner_square_tour.py --port <tovez-port> --sequential
```

**`--trim` OFF. No `--turn-scale`.** Zero host correction — the firmware does
it alone.

- **Today this exact invocation measures 64.1 mm.**
- **Sprint acceptance: ≤ 8 mm.**

The script is already committed with per-boundary pose capture, per-nudge
instrumentation, and CSV/JSON output (`--sequential`, `--trim`, `--trim-tol`,
`--trim-max-nudges` at `:334-345`). You should not need to modify it; if you
do, say so explicitly in the completion note.

### Read the result honestly — the two bars

Per sprint.md Success Criteria, and this matters because the gate sits at the
**optimistic edge** of the measured distribution:

- **Sprint acceptance**: closure ≤ 8 mm.
- **Mechanism acceptance**: closure ≤ 12 mm **and** per-corner residual inside
  `align_tol` at ≥90% of corners **and** ≥3× improvement on the 64 mm baseline.

The report's honest typical band for a trimmed tour is **8–11 mm** (§1, §3);
the **7.3 ± 3.6 mm** best arm was measured after a battery swap left less
residual to absorb. §3 is explicit: "**the only road below ~8 mm is a finer
terminal actuator, not a tighter threshold**" — and that actuator (§5.4) is
**out of scope** for this sprint.

So: landing in 8–12 mm with the mechanism demonstrably converging is a
**measured shortfall of the actuator, not a defect of this sprint's design**.
Report it with the per-corner data. **Do NOT** tighten `align_tol` (measured
counterproductive — convergence 93%→64%, some corners get *worse*), and **do
NOT** add a calibration fudge. Do not churn constants to chase the number.

## What to report

Per-boundary, not just closure — start-and-end only cannot attribute error:

- per-leg length and per-turn angle
- cumulative heading residual at each corner (does it still grow +1.5/+5.3/
  +6.3/+10.6° as in the 64 mm baseline?)
- **nudges per corner** and convergence rate (expect ~1.3/corner, ~94%)
- wall time per corner (expect ~2 s) and total tour time
- the raw CSV/JSON archived under `src/tests/bench/output/`

## Teleop regression check — do not skip

Ticket 002 changed who gets the speed floor. Confirm on hardware that a plain
`wheels()` teleop command **below** `v_min` (20 mm/s) is **still boosted** and
still moves the robot. If ticket 002 disabled the floor for everyone, this is
where it shows up, and it will not show up in the tour.

## Acceptance Criteria

- [ ] `mbdeploy list` confirms `tovez` and the UID is used for every targeted
      command
- [ ] Firmware built `uv run python build.py --clean --robot-debug` and flashed
      to `tovez` by UID
- [ ] `planner_square_tour.py --sequential`, no `--trim`, no `--turn-scale`,
      run and its output archived
- [ ] Closure reported against **both** bars above, with the number stated
      plainly whichever way it lands
- [ ] Per-leg, per-turn, cumulative residual, nudges/corner and wall time
      reported
- [ ] Teleop sub-`v_min` `wheels()` command confirmed still floored on hardware
- [ ] No firmware constant was changed to chase the closure number (or, if one
      was, it is called out explicitly with its justification)

## Implementation Plan

1. `uv run mbdeploy probe && uv run mbdeploy list` — confirm `tovez`, note the
   port for this session, note the UID.
2. Build: `uv run python build.py --clean --robot-debug`. **Plain `just build`
   compiles the DBG channel OUT** — do not use it.
3. Flash by UID:
   `uv run mbdeploy deploy --hex ./MICROBIT.hex 9906360200052820a8fdb5e413abb276000000006e052820`
4. Confirm the robot is alive and the config landed. **`set_config_field` acks
   were flaky on 2026-08-04 (2 of 3 NAK'd) — retry and read back.** An ack is
   not evidence (`.claude/rules/configuration-discipline.md`).
5. Run the tour. Archive output.
6. Teleop floor check.
7. Write up per-boundary results.

### Bench gotchas that cost time on 2026-08-04

- **Closing the serial port resets the robot** (HUPCL is a no-op on macOS).
  Plan captures so a reconnect does not silently re-zero state mid-measurement.
- Do bench work in its **own working directory and venv** — never `just build`
  in the sprint checkout; its version-bump hook edits tracked `pyproject.toml`.
- **NEVER `git stash`** — two long-lived stashes hold other people's WIP.
- If the robot goes silent after a banner and ~2 frames, suspect the **Nezha
  brick losing power** (dead external I2C bus), not a firmware defect — that is
  what ended the 2026-08-04 session, diagnosed by pyocd backtrace. Progressive
  travel decay (496→425→55→0 mm) is what a dying brick battery looks like from
  odometry.

## Testing

- **This ticket is itself the test.** Hardware is the arbiter.
- Per the standing verification gate (`.claude/rules/hardware-bench-testing.md`):
  a sprint touching motor control is not "done" on tests alone — it must be
  seen working on the stand.
- Also run the full suite once here and compare **by identity** against the
  master baseline of ~8 failed / ~1994 passed, accounting for the legitimate
  `test_gen_boot_config_*` movement from ticket 003.
- **Verification command**:
  `uv run python src/tests/bench/planner_square_tour.py --port <tovez> --sequential`
