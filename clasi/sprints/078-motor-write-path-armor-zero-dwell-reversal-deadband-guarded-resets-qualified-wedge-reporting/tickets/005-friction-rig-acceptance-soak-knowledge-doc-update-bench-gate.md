---
id: '005'
title: Friction-rig acceptance soak + knowledge-doc update + bench gate
status: open
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

- [ ] Hot-flip soak: 0 motion-armed (`wsus=1`) latches over ≥100 hot
      flips with the armor on (default config).
- [ ] A/B bracketing performed and recorded: which motors were used,
      whether the control arm (`dwell=0`) reproduced the trigger or not,
      and the treatment arm's clean result — with the "immune motors ≠
      proof" caveat explicitly addressed in the results, not silently
      omitted.
- [ ] Mid-motion `RESET` verified to take the soft path (`src=`
      increments, `hrc=` unchanged, `pos=` ~0 immediately after); at-rest
      `RESET` verified to take the hard path (`hrc=` increments).
- [ ] `kRestVelocity`/`kRestTicksRequired` reviewed against bench
      evidence; either confirmed adequate or adjusted, with the reasoning
      recorded in this ticket's completion notes.
- [ ] CSV + transcript retained for every run (control arm, treatment arm,
      reset-guard check).
- [ ] Every session ends with `DEV STOP`, including on exception/Ctrl-C.
- [ ] `docs/knowledge/2026-07-04-encoder-wedge.md`'s status is updated to
      shipped-in-new-tree with a link/reference to this ticket's evidence.
- [ ] Standing bench gate confirmed per `.claude/rules/hardware-bench-
      testing.md`: sensors alive, wheels drive both directions with
      encoders incrementing, round-trip commands verified over the
      transport in use.

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
