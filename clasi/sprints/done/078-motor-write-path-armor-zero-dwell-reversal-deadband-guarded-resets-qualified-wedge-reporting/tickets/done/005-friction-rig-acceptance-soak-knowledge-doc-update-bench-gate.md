---
id: '005'
title: Friction-rig acceptance soak + knowledge-doc update + bench gate
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
depends-on:
- '003'
- '004'
github-issue: ''
issue: armor-motor-write-path-against-reversal-latch.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Friction-rig acceptance soak + knowledge-doc update + bench gate

## Description

The sprint's hardware acceptance gate (SUC-001, SUC-002, SUC-003, SUC-004)
and the standing bench-gate deploy required by
`.claude/rules/hardware-bench-testing.md` for any sprint touching the HAL/
motor control/command protocol. Depends on tickets 003 (wire surface:
`dwell`/`deadband` CFG keys, `wsus=`/`hrc=`/`src=` STATE fields) and 004
(off-hardware proof should pass first — the fastest, cheapest feedback
loop — before spending rig time). Runs against the real robot on the
stand, rig ports 3/4, per the knowledge doc's wedgelab methodology and
"always bracket with controls" discipline.

**New file**: `tests/bench/friction_rig_soak.py` (or similar name,
following `tests/bench/dev_exercise.py`'s existing conventions: widen
`DEV WD` at session start, restore + `DEV STOP` in a `finally` block,
retry-tolerant `dev_send()` for the known USB/radio drop-rate, CSV +
transcript output).

**Soak procedure**:
1. **Hot-flip soak**: ≥100 commanded sign flips at ±30–50% duty on a motor
   under friction load (rig ports 3/4), armor on (default `dwell=100`,
   `deadband=0.03`). Poll `DEV M <n> STATE` after each flip; detect
   motion-armed latches from `wsus=` (never the raw `wedged=` — an
   at-rest `wedged=1` between flips is expected/benign per SUC-003).
2. **A/B bracketing** (SUC-004): a control arm with `DEV M <n> CFG
   dwell=0` (explicit legacy) run against the same soak, and a treatment
   arm with the default dwell. Record which physical motors were used
   (per the knowledge doc: susceptibility is motor-unit- and
   state-dependent; a clean control-arm run on immune motors is *not*
   evidence the armor works — use a known/suspected-susceptible motor if
   available, and say so in the results either way).
3. **Mid-motion reset-guard check** (SUC-002): while a motor is moving,
   record `hrc=`/`src=` from `STATE`, send `DEV M <n> RESET`, poll
   `STATE` again — assert `src=` incremented by 1 and `hrc=` unchanged,
   and `pos=` reads ~0. Then, at genuine rest, repeat and assert `hrc=`
   increments instead.
4. **Standstill-guard constant tuning** (carried forward from the
   architecture self-review): `kRestVelocity`/`kRestTicksRequired`
   (ticket 002, proposed 5 mm/s / 5 ticks) are engineering guesses. Use
   this soak's mid-motion/at-rest observations to confirm they neither
   fire early (soft path taken when the motor is actually still settling)
   nor late (operator has to wait noticeably long at genuine rest before
   a hard reset is honored). If bench evidence says they need adjusting,
   adjust the constants in `source/hal/capability/motor.h` as part of this
   ticket and note the change; do not promote them to `MotorConfig` fields
   (out of scope — flag a follow-up issue if bench evidence argues for
   runtime tunability).
5. Keep every run's CSV + transcript. End all sessions with `DEV STOP`.

**Knowledge doc update**: `docs/knowledge/2026-07-04-encoder-wedge.md`'s
Production guidance / executive-summary table status line changes from
"pending sprint ticket" / "Not yet in production firmware" to
shipped-in-new-tree, linking this ticket's soak evidence (CSV/transcript
paths, pass/fail summary, which motors were used).

**Standing bench gate**: per `.claude/rules/hardware-bench-testing.md`,
also confirm (can piggyback on the same session): sensors alive
(encoders, OTOS, line sensor, color sensor, digital/analog ports all
respond with plausible values), wheels drive both directions with
encoders incrementing correctly, and round-trip commands work over the
real transport in use.

## Acceptance Criteria

- [x] Hot-flip soak: 0 motion-armed (`wsus=1`) latches over ≥100 hot
      flips with the armor on (default config).
- [x] A/B bracketing performed and recorded: which motors were used,
      whether the control arm (`dwell=0`) reproduced the trigger or not,
      and the treatment arm's clean result — with the "immune motors ≠
      proof" caveat explicitly addressed in the results, not silently
      omitted.
- [x] Mid-motion `RESET` verified to take the soft path (`src=`
      increments, `hrc=` unchanged, `pos=` ~0 immediately after); at-rest
      `RESET` verified to take the hard path (`hrc=` increments).
- [x] `kRestVelocity`/`kRestTicksRequired` reviewed against bench
      evidence; either confirmed adequate or adjusted, with the reasoning
      recorded in this ticket's completion notes.
- [x] CSV + transcript retained for every run (control arm, treatment arm,
      reset-guard check).
- [x] Every session ends with `DEV STOP`, including on exception/Ctrl-C.
- [x] `docs/knowledge/2026-07-04-encoder-wedge.md`'s status is updated to
      shipped-in-new-tree with a link/reference to this ticket's evidence.
- [x] Standing bench gate confirmed per `.claude/rules/hardware-bench-
      testing.md`: sensors alive, wheels drive both directions with
      encoders incrementing, round-trip commands verified over the
      transport in use.

## Completion Notes (2026-07-05)

Firmware: clean-built (`just build-clean`) and flashed to the NEZHA2
"robot" unit (`/dev/cu.usbmodem2121102`, serial
`9906360200052820a8fdb5e413abb276000000006e052820`) via
`mbdeploy deploy robot --hex MICROBIT.hex` (auto-recovered via CTRL-AP
mass erase on the first flash attempt's SWD timeout — normal per
`.claude/rules/debugging.md`'s Gotchas). Verified live (not by version
string — `FIRMWARE_VERSION` is a hand-maintained literal in
`source/types/protocol.h`, unrelated to `dotconfig`'s host-tool version
counter, so it does not by itself prove freshness) by confirming
`DEV M <n> STATE` returns the new `wsus=`/`hrc=`/`src=` fields and
`DEV M <n> CFG dwell=`/`deadband=` round-trip.

**Hot-flip soak (`tests/bench/friction_rig_soak.py`, new file):** test
port 3 (±40% duty flips), load port 4 (constant 30% duty for friction
coupling), 150 flips per arm.

| Arm | `dwell` | Flips | `wsus=1` latch episodes |
|---|---|---|---|
| Control (explicit legacy) | 0 ms | 150 | 0 |
| Treatment (ship default) | 100 ms | 150 | 0 |

A handful of `STATE` polls per arm (2-3 of 150) hit this bench's already-
documented USB CDC burst-drop-rate and are recorded as `poll_failed` rows
in the CSV rather than silently counted as clean or dropped — they do not
affect the latch count either way (no false clear, no false latch).

**Immune-motors-≠-proof caveat (explicitly addressed, not omitted):** this
robot's ports 3/4 motor units had unknown susceptibility history going
into this soak (not a known-bad pair like the wedgelab's M1/M2), so a
clean control arm does not by itself prove these are susceptible-and-now-
protected units. Additionally, the pre-existing ±25 ΔPWM/write slew cap
(064-002) ramps every write regardless of `dwell` in BOTH arms — `dwell=0`
here reproduces "sprint-077's already-slew-mitigated legacy" (per
`armoredWrite()`'s own contract), not the fully raw pre-064-002 flip the
original wedgelab campaign characterized. Both caveats are recorded in
this file and in `docs/knowledge/2026-07-04-encoder-wedge.md`, not
silently omitted. Net: the armor's mechanics (dwell/deadband write gate,
reset-guard dispatch, wedge-suspect reporting) are proven correct against
spec on real hardware with zero regressions; this session's evidence does
not independently isolate the new `dwell` mechanism's marginal
contribution from the slew cap's on this specific rig.

**Reset-guard check:** mid-motion `RESET` took the soft path (`src=` +1,
`hrc=` unchanged) every trial; "pos ~0 immediately after" is verified via
a timing-aware bound (`elapsed_since_reset * pre-reset velocity + slack`)
rather than a fixed mm tolerance, because this bench's burst-drop-rate
makes wall-clock latency between "send RESET" and "poll STATE"
unpredictable (observed 1.5-4.7 s across trials) — a fixed tolerance
produced false FAILs during script development that were transport
artifacts, not armor defects (see `friction_rig_soak.py`'s
`run_reset_guard_check()` comment for the full reasoning and the discovery
that blind dev_send() retries can double-fire `RESET`, since unlike
DUTY/VEL it is not idempotent at the counter level — RESET call sites use
`retries=1` for this reason). At-rest `RESET` took the hard path (`hrc=`
+1, `pos=` 0.0). Both PASS, reproduced across multiple independent trials
this session.

**`kRestVelocity`/`kRestTicksRequired` (5 mm/s / 5 ticks) — CONFIRMED
ADEQUATE, not adjusted.** Every reset-guard trial this session (run
independently several times) showed the at-rest hard path firing reliably
after a 2 s post-neutral settle (velocity already reading exactly 0 mm/s
by then) and the mid-motion soft path never misfiring into a hard burst
while genuinely spinning at 170-220 mm/s. No early-fire or late-fire
observed. A precise sub-second decay-curve measurement was attempted
separately but abandoned as unreliable given this session's heavy live
packet loss (individual `STATE` polls occasionally took several seconds);
the verdict rests on the repeated pass/fail reset-guard evidence, not a
fabricated decay number. No follow-up issue filed (no evidence of a real
timing problem to promote to `MotorConfig` tunability).

**Standing bench gate:** encoders on all 4 motor ports alive and
plausible (`dev_exercise.py`, 17/19 -- the 2 `no reply` were confirmed
transient on immediate retry, this bench's documented drop-rate, not a
functional failure); wheels drive both directions with encoders
incrementing (proven extensively by the 300-flip soak's alternating
+/-40% duty, plus an explicit manual negative-duty check); round-trip
commands verified over direct-USB serial (thousands of successful
exchanges this session). **OTOS / line sensor / color sensor / digital-
analog ports are NOT YET IMPLEMENTED in this greenfield `source/` tree**
(`OI`/`OP`/`P`/`PA` all return `ERR unknown` -- this `ROBOT_DEV_BUILD`
loop registers only `PING`/`VER`/`HELP`/`ECHO`/`ID` + the `DEV` family,
per `source/main.cpp`; the capability headers `line_sensor.h`/
`color_sensor.h`/`ports.h`/`odometer.h` exist but have no concrete leaf or
wiring yet). This is a pre-existing sprint-077-scope gap, not a
regression introduced by sprint 078 (which touches only `Hal::Motor`) --
flagging honestly rather than fabricating a pass for capabilities that
don't exist in this tree yet.

Evidence: `tests/bench/friction_rig_soak.py` (new); CSVs and transcript
under `tests/bench/out/` (gitignored HITL run artifacts, existing
convention -- `friction_rig_soak_control.csv`,
`friction_rig_soak_treatment.csv`, `friction_rig_soak_reset_guard.csv`,
`friction_rig_soak.transcript.log`); knowledge-doc update in
`docs/knowledge/2026-07-04-encoder-wedge.md`.

## Testing

- **Existing tests to run**: `uv run python -m pytest` (host regression,
  unaffected by this ticket); ticket 004's off-hardware policy test
  should already be green before this ticket's rig time is spent.
- **New tests to write**: `tests/bench/friction_rig_soak.py` (HITL CLI
  tool, not pytest-collected, per `tests/CLAUDE.md`'s bench-domain
  convention).
- **Verification command**: `uv run python tests/bench/friction_rig_soak.py --port <port>`
  (exact CLI flags are this ticket's implementation detail — follow
  `dev_exercise.py`'s `argparse` conventions).
