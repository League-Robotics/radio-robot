---
id: '003'
title: 'Stage 1 hardware acceptance: encoder heading loop on the stand and playfield'
status: done
use-cases:
- SUC-001
- SUC-005
depends-on:
- '002'
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

- [x] `just build-clean` succeeds (non-incremental, avoiding the documented
      stale-incremental-build risk — `[[stale-incremental-build-on-volumes]]`).
- [x] Full `uv run python -m pytest` is green immediately before flashing
      (confirms tickets 001/002's own gates still hold on this exact
      commit).
- [x] `mbdeploy probe` confirms the connected device's role before flashing
      (`[[verify-microbit-before-flashing]]` — never blind-flash; the robot
      and the RELAY dongle can share the same mount point).
- [x] `mbdeploy deploy --build` (or `mbdeploy deploy <UID> --hex ...` per
      `[[hitl-bench-mbdeploy-build-and-watchdog]]`) flashes successfully.

**Stand check (wheels off the ground)**

- [x] Sensors alive: encoders, and any other configured sensors, respond
      with plausible values.
- [x] Wheels drive both directions; encoders increment in the expected
      direction, roughly proportional to commanded speed.
- [x] A single in-place turn (e.g. `TURN`/`RT`/`MOVE 0 0 <heading>`)
      completes and visibly lands close to target.
- [x] **No wedge.** No commanded terminal reversal beyond the `Hal::Motor`
      reversal-dwell/deadband armor's own absorbed window — this is the
      sprint's top flagged risk (`architecture-update.md`'s Risks section)
      and must be explicitly confirmed, not assumed, BEFORE moving to the
      playfield leg.
- [x] The RX watchdog is fed throughout the session (per
      `[[dev-serial-passive-pump-sampling]]`) so the session does not
      stall on transport starvation unrelated to this ticket.

**Playfield leg (robot moved from USB to the radio relay path — an
explicit location change, not a continuation of the same physical setup)**

- [x] `tests/bench/turn_sweep.py --relay --both` run across the full
      existing angle × ceiling grid (`ANGLES = [30, 90, 180, 360]`,
      `CEILINGS = [70, 140, 210, 280, 384]` mm/s, both directions per
      cell).
- [x] Every cell's `overshoot_deg` lands within goal tolerance,
      **`|overshoot_deg| <= ~1°`** (the issue's own "goal ≈ ±1°" acceptance
      bar).
- [x] The 90° ridge (a same-entry-rate turn overshooting roughly double a
      longer turn's overshoot — the dataset's own named defect) is gone:
      no cell shows the disproportionate-vs-neighboring-cells pattern the
      pre-sprint dataset documented.
- [x] Run-to-run scatter has collapsed: repeat a representative subset of
      cells (at minimum one low-ceiling and one high-ceiling 90° cell, 3
      repeats each) and confirm σ is well under the pre-sprint ~2° baseline.
- [x] `[safety] wheels confirmed stopped` prints at the end of the sweep
      (the script's own built-in safety-shutdown confirmation).

**Gain iteration (in scope for this ticket)**

- [x] If `heading_kp`/`heading_kd` (ticket 001's conservative starting
      values, `3.0`/`0.0`) do not meet the tolerance bar above, this
      ticket ITERATES them (editing `data/robots/tovez.json`, reflashing,
      re-running the relevant cells) — explicitly in scope per
      `architecture-update.md` Decision 2, not a separate ticket. Record
      the final values and the iteration history in this ticket's
      completion notes.
- [x] Any failure at any step is reported explicitly (which step, what was
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

## Completion Notes (2026-07-12, HITL over the radio relay, team-lead-driven)

**Result: PASS — full-speed turns of every angle terminate on target.**

Firmware `v0.20260711.14` flashed to the robot (NEZHA2 `tovez`, UID
`...a8fdb5e4...`) by full UID after `mbdeploy list` role-verification (never
the RADIOBRIDGE dongle `...e9d16c38...`); the first flash hit a locked-device
`0x67` erase failure and auto-recovered via CTRL-AP mass-erase on the retry
(normal). VER handshake over the relay confirmed `fw_version 0.20260711.14`.

**Turn-accuracy, full grid (30/90/180/360 deg × 70/140/210/280/384 mm/s, both
directions), `turn_sweep.py --relay --both`:**

| gain | mean \|err\| | max \|err\| | within ±1° | 90° full-ceiling ridge |
|---|---|---|---|---|
| open-loop (pre-sprint) | 3.41° | 11.65° | 19% | +6.8..+9.7° |
| heading loop, kp=3.0/kd=0 | 0.53° | 2.39° | 88% | **gone** (−0.4..−1.1°) |
| **heading loop, kp=6.0/kd=0 (SHIPPED)** | **0.27°** | **0.84°** | **100%** | **gone** |

**Gain iteration:** started at 098-001's conservative `kp=3.0/kd=0`. That
already eliminated the ridge but left a residual ~1–2° full-ceiling UNDERSHOOT
(worst −2.4° at 180°@384). Traced it on hardware (`turn_trace_+180_384.csv`):
it is a proportional-control STEADY-STATE ERROR against terminal motor
stiction — near the target the P command (∝ residual error) falls below the
wheel deadband (~7 deg/s ≈ 8 mm/s), so the robot stalls ~1° short and STOP_TIME
ends the phase. Raising `kp` to 6.0 makes the command overcome stiction closer
in; the full grid then landed 100% within ±1°, max 0.84°. `kp=6.0` is
monotonic-safe for a pure-P to-rest loop (no overshoot mechanism) and showed no
ringing. Final shipped values: `heading_kp=6.0`, `heading_kd=0.0` in
`data/robots/tovez.json` (+ note updated). The firmware default stays the
conservative `3.0/0.0` for uncharacterized robots.

**No wedge / terminal-reversal safety:** explicit trace check
(`turn_trace_*90_384.csv`) — the commanded wheel velocity floors at 0 and
NEVER reverses (0 commanded-reversal frames both directions); the encoder-wedge
trigger (a commanded reversal write-train) is structurally absent. 40+ turns
each ended `[safety] wheels confirmed stopped`, no wedge, no runaway.

**Run-to-run scatter collapsed:** repeated 90° cells (n=3–4 each), per-cell
σ = 0.26–0.38°, pooled 90° σ = 0.37° — down from the pre-sprint ~2.0° baseline,
exactly as predicted (a servoed endpoint has no open-transient difference to
scatter).

**Encoder heading was sufficient** (Stage 1): no OTOS needed to hit target —
the residual was tracking/stiction, not slip. OTOS heading (ticket 004) remains
the optional slip-immunity upgrade.

**Remaining (sub-degree, optional):** the last <1° at full ceiling is the
irreducible P-vs-stiction floor; an integral term would null it entirely
(future enhancement, noted in the issue and the tovez `_heading_gains_note`).

Datasets: `tests/notebooks/out/turn_sweep_hl_full.csv` (kp=3),
`turn_sweep_hl_kp6.csv` (kp=6, shipped), `turn_sweep_sigma_*.csv` (σ repeats),
`turn_trace_*_384.csv` (terminal traces).
