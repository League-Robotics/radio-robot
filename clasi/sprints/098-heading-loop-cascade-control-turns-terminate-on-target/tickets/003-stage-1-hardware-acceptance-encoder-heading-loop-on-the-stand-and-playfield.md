---
id: '003'
title: 'Stage 1 hardware acceptance: encoder heading loop on the stand and playfield'
status: open
use-cases: [SUC-001, SUC-005]
depends-on: ['002']
github-issue: ''
issue:
- heading-loop-cascade-control-turns-terminate-on-target.md
- real-robot-motion-calibration-undershoot.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Stage 1 hardware acceptance: encoder heading loop on the stand and playfield

## Description

The sprint's own acceptance gate. Deploy the encoder-only heading loop
(tickets 001-002) to the robot and confirm on real hardware what sim cannot
fully prove: terminal-reversal safety against the motor armor, and the
actual turn-accuracy improvement the 58-turn dataset's own instrument
(`turn_sweep.py`) measures. Per `architecture-update.md`'s own framing,
Stage 1 alone (encoder heading) satisfies the issue's acceptance criterion
— this ticket IS that satisfaction, not a preview of it.

Depends on 002.

## Acceptance Criteria

**Pre-flash**

- [ ] `just build-clean` succeeds (non-incremental, avoiding the documented
      stale-incremental-build risk — `[[stale-incremental-build-on-volumes]]`).
- [ ] Full `uv run python -m pytest` is green immediately before flashing
      (confirms tickets 001/002's own gates still hold on this exact
      commit).
- [ ] `mbdeploy probe` confirms the connected device's role before flashing
      (`[[verify-microbit-before-flashing]]` — never blind-flash; the robot
      and the RELAY dongle can share the same mount point).
- [ ] `mbdeploy deploy --build` (or `mbdeploy deploy <UID> --hex ...` per
      `[[hitl-bench-mbdeploy-build-and-watchdog]]`) flashes successfully.

**Stand check (wheels off the ground)**

- [ ] Sensors alive: encoders, and any other configured sensors, respond
      with plausible values.
- [ ] Wheels drive both directions; encoders increment in the expected
      direction, roughly proportional to commanded speed.
- [ ] A single in-place turn (e.g. `TURN`/`RT`/`MOVE 0 0 <heading>`)
      completes and visibly lands close to target.
- [ ] **No wedge.** No commanded terminal reversal beyond the `Hal::Motor`
      reversal-dwell/deadband armor's own absorbed window — this is the
      sprint's top flagged risk (`architecture-update.md`'s Risks section)
      and must be explicitly confirmed, not assumed, BEFORE moving to the
      playfield leg.
- [ ] The RX watchdog is fed throughout the session (per
      `[[dev-serial-passive-pump-sampling]]`) so the session does not
      stall on transport starvation unrelated to this ticket.

**Playfield leg (robot moved from USB to the radio relay path — an
explicit location change, not a continuation of the same physical setup)**

- [ ] `tests/bench/turn_sweep.py --relay --both` run across the full
      existing angle × ceiling grid (`ANGLES = [30, 90, 180, 360]`,
      `CEILINGS = [70, 140, 210, 280, 384]` mm/s, both directions per
      cell).
- [ ] Every cell's `overshoot_deg` lands within goal tolerance,
      **`|overshoot_deg| <= ~1°`** (the issue's own "goal ≈ ±1°" acceptance
      bar).
- [ ] The 90° ridge (a same-entry-rate turn overshooting roughly double a
      longer turn's overshoot — the dataset's own named defect) is gone:
      no cell shows the disproportionate-vs-neighboring-cells pattern the
      pre-sprint dataset documented.
- [ ] Run-to-run scatter has collapsed: repeat a representative subset of
      cells (at minimum one low-ceiling and one high-ceiling 90° cell, 3
      repeats each) and confirm σ is well under the pre-sprint ~2° baseline.
- [ ] `[safety] wheels confirmed stopped` prints at the end of the sweep
      (the script's own built-in safety-shutdown confirmation).

**Gain iteration (in scope for this ticket)**

- [ ] If `heading_kp`/`heading_kd` (ticket 001's conservative starting
      values, `3.0`/`0.0`) do not meet the tolerance bar above, this
      ticket ITERATES them (editing `data/robots/tovez.json`, reflashing,
      re-running the relevant cells) — explicitly in scope per
      `architecture-update.md` Decision 2, not a separate ticket. Record
      the final values and the iteration history in this ticket's
      completion notes.
- [ ] Any failure at any step is reported explicitly (which step, what was
      observed, with numbers) — this ticket is not satisfied by a single
      summary verdict.

## Testing

- **Existing tests to run**: `uv run python -m pytest` (pre-flash gate);
  the full `turn_sweep.py --relay --both` grid IS this ticket's own
  acceptance test.
- **New tests to write**: none in the pytest sense — `tests/bench/
  turn_sweep.py` already exists (an HITL CLI tool, not pytest-collected,
  per project convention) and is this ticket's acceptance instrument
  as-is.
- **Verification command**: `uv run python -m pytest` (pre-flash);
  `uv run python tests/bench/turn_sweep.py --relay --both` (the acceptance
  run itself).

## Implementation Plan

**Approach**: A deploy-and-verify ticket, not a code-writing ticket (beyond
iterating `heading_kp`/`heading_kd` in `tovez.json` if tolerance is not met
on the first pass). Follow `.claude/rules/hardware-bench-testing.md` and
the project's bench-verification knowledge (`[[bench-verification-gotchas-088]]`,
`[[binary-vs-text-same-boot-loss-discriminator]]`) throughout.

**Files to modify**: `data/robots/tovez.json` (`heading_kp`/`heading_kd`,
ONLY if iteration is needed — update `_heading_gains_note` to record the
final bench-derived values and drop the "not yet bench-tuned" caveat once
confirmed).

**Files to create**: none expected.

**Testing plan**: as above.

**Documentation updates**: none required structurally; record the final
gain values and the `turn_sweep.py` results in this ticket's own
completion notes for the sprint-close review (mirrors ticket 094-007's
precedent).
