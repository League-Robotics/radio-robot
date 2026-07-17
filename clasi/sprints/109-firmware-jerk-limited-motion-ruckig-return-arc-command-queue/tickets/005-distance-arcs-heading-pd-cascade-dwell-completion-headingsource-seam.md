---
id: '005'
title: DISTANCE arcs + heading PD cascade + dwell completion + HeadingSource seam
status: open
use-cases: [SUC-001, SUC-002, SUC-004]
depends-on: ['003']
github-issue: ''
issue: firmware-jerk-limited-motion-ruckig-return-arc-command-queue.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# DISTANCE arcs + heading PD cascade + dwell completion + HeadingSource seam

## Description

This is the turn-accuracy ticket — the sprint's core motivation. It adds
DISTANCE-mode arc commands (the coupled distance + delta-heading curve)
on top of ticket 003's TIMED-mode skeleton, and restores the sprint-098
firmware heading PD cascade that landed 100% of turns within ±1° before
the single-loop rebuild deleted it.

1. Dominant-channel arc planning: Ruckig plans the linear channel for
   `|distance| > 0` (arc or straight leg), or the rotational channel for a
   pure pivot (`distance == 0`). The other channel is slaved:
   `omega_ff(t) = (delta_heading / distance) * v(t)`.
2. Heading reference: `theta(s) = delta_heading * s / |distance|` for
   arcs; direct rotational target for pivots.
3. Heading PD cascade, restored at the firmware's 40 ms cycle:
   `omega_cmd = omega_ff + heading_kp*(theta_des - theta_meas) +
   heading_kd*(omega_des - omega_meas)`, gains from `PlannerConfig`
   (bench-proven `kp=6.0` in `data/robots/tovez.json` — read the gain from
   config, do not hardcode it), gated off during terminal decel.
4. Completion: rest-terminated commands with heading content complete on
   `|err| < 0.5°` AND rate `< 1°/s` held 150 ms (dwell), with a STOP_TIME
   backstop. Distance completion: encoder-relative travel ≥ `|distance|`,
   signed overshoot carried into a same-sign successor (full boundary-
   velocity carry across DISTANCE commands is ticket 006 — this ticket
   only needs single-command completion and overshoot bookkeeping for a
   successor, not the no-decel handoff itself). Chained (non-terminal)
   pivots use encoder/OTOS-accurate handoff without a dwell — only the
   final pivot in a chain dwells.
5. `App::HeadingSource` (`src/firm/app/heading_source.{h,cpp}`): passive
   reader, no bus traffic of its own (reads what the loop already sampled
   — OTOS has a clean 20 ms slot in `kPace`). Policy: OTOS whenever
   `present() && connected() && poseFresh()`; automatic fallback to
   encoder-differential heading `(encR - encL) / trackwidth` after N
   stale cycles; re-promote when OTOS recovers. Visibility: active source
   in every primary TLM frame + an event on fallback transition; TestGUI
   surfaces a non-gyro indicator. Per-robot override via robot JSON
   (`control.heading_source`) → `gen_boot_config.py` →
   `PlannerConfig.heading_source` (new field).
6. `kDeadTime` re-derivation at the 40 ms cycle: the old 120 ms value
   assumed a 20 ms tick (per the issue). This is bench-tune-only — do not
   hand-pick a new constant from the old one; characterize it fresh on
   the stand (sprint.md's Open Question #2 flags this explicitly).

## Acceptance Criteria

- [ ] DISTANCE-mode arcs (`|distance| > 0`, `delta_heading` possibly
      nonzero) plan the dominant (linear) channel and slave the
      rotational channel by the arc ratio.
- [ ] Pure pivots (`distance == 0`, `delta_heading != 0`) plan the
      rotational channel directly.
- [ ] Heading PD cascade implemented exactly per the formula above, gains
      read from `PlannerConfig` (not hardcoded), gated off during
      terminal decel.
- [ ] Rest-terminated heading-bearing commands complete on the dwell
      criterion (`|err| < 0.5°` AND rate `< 1°/s` held 150 ms) with a
      STOP_TIME backstop; chained non-terminal pivots hand off without a
      dwell.
- [ ] Distance completion uses encoder-relative travel with signed
      overshoot carried into a same-sign successor.
- [ ] `App::HeadingSource` implements OTOS-first/encoder-fallback policy;
      fallback transition fires a TLM event; TestGUI shows a non-gyro
      indicator; per-robot `control.heading_source` override wired
      through `gen_boot_config.py` → `PlannerConfig.heading_source`.
- [ ] `kDeadTime` re-derived and bench-characterized at the 40 ms cycle
      (not copied from the old 120 ms/20 ms-tick value) — record the new
      value and how it was measured.
- [ ] `src/firm/motion/DESIGN.md` updated (arc-planning + heading-PD
      design, dwell completion); `src/firm/app/DESIGN.md` updated (new
      `HeadingSource` module); root `src/firm/DESIGN.md` §2 updated if the
      dependency diagram changes (HeadingSource reads Otos/Odometry
      samples already taken by the loop — confirm no new bus-traffic edge
      is introduced, per the single-loop invariant).
- [ ] Bench (`.claude/rules/hardware-bench-testing.md`): arc/pivot
      accuracy sweep (`turn_sweep.py`-style) shows turns landing near ±1°
      on hardware (full sim 1° gate is ticket 009's job; this ticket's
      bench check is a sanity pass, not the decisive gate).

## Testing

- **Existing tests to run**: ticket 003's TIMED-mode tests (must remain
  passing); TWIST/STOP regression.
- **New tests to write**: pivot accuracy vs. sim-OTOS drift + fallback-to-
  encoder transition sim test (asserting TLM `headingSource` visibility);
  dwell-completion unit test (chained vs. terminal pivot); distance-
  completion overshoot-carry unit test.
- **Verification command**: `uv run python -m pytest src/tests/sim/
  system/ -k "pivot or heading or dwell"`.

## Implementation Plan

**Approach**: Layer arc planning and the heading PD directly on top of
ticket 003's Executor/Pilot skeleton — no new top-level module beyond
`HeadingSource`. Read gains from config from day one (never hardcode
`heading_kp=6`) so ticket-009's tuning-if-needed doesn't require a code
change.

**Files to create**:
- `src/firm/app/heading_source.{h,cpp}`

**Files to modify**:
- `src/firm/motion/executor.{h,cpp}` (dominant-channel arc planning,
  heading reference, dwell completion, overshoot carry)
- `src/firm/app/pilot.{h,cpp}` (heading PD cascade computation in
  `tick()`)
- `Config`/`PlannerConfig` (new `heading_source` field), `gen_boot_
  config.py` (per-robot override)
- `src/firm/motion/DESIGN.md`, `src/firm/app/DESIGN.md`,
  `src/firm/DESIGN.md`

**Testing plan**: as above; bench arc/pivot sweep.

**Documentation updates**: as listed in acceptance criteria.
