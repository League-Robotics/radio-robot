---
id: 019
title: "Hardware bench acceptance on tovez \u2014 headline L/R gap closure, cold boot"
status: done
use-cases:
- SUC-006
depends-on:
- 018
github-issue: ''
issue: the-configuration-object.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Hardware bench acceptance on tovez — headline L/R gap closure, cold boot

## Description

The sprint's headline deliverable and second acceptance-concentrated
closing ticket. `tovez` is confirmed alive and healthy as of a recent
live check (PONG, telemetry flowing, wheels driving, encoders climbing,
both motors connected, on the stand) — address it by UID
`9906360200052820a8fdb5e413abb276000000006e052820`, never by port
(`.claude/rules/hardware-bench-testing.md`). A stale `mbdeploy list` row
with no ROLE/DEVICE NAME is a failed-probe artifact, not a dead-robot
signal — re-probe (`mbdeploy probe` then `mbdeploy list`) and trust the
live check, not a remembered row.

Push per-wheel Stage-A drive correction live over the new wire (ticket
009's capability) and re-run `src/tests/bench/velocity_profile_gate.py`
before/after, demonstrating the measured 11.1-point L/R plateau-tracking
gap (L 96.0% / R 84.9%) closing — including from a COLD BOOT
(power-cycle or reflash between the correction being applied/persisted
and the verification run — not just a warm, already-tuned session), per
`.claude/rules/hardware-bench-testing.md`'s standing verification gate.

## Acceptance Criteria

- [x] `mbdeploy list` confirms `tovez` present by UID before any hardware
      action (re-probe if the row looks stale).
- [x] Baseline measurement: `velocity_profile_gate.py` run on `tovez`
      (by UID) BEFORE any correction is pushed, confirming (or
      re-measuring) the L/R plateau split.
- [x] Per-wheel Stage-A correction is pushed live over the wire (ticket
      009's `DRIVE` group push), and/or baked via the reshaped
      `tovez.json` (ticket 017) — both paths are legitimate; the ticket
      records which was used.
- [x] The correction survives a COLD BOOT — the robot is power-cycled or
      reflashed between the correction landing and the final
      measurement, not just measured warm in the same session.
- [x] `velocity_profile_gate.py` run AFTER, from cold boot, shows the L/R
      gap closed relative to baseline — the specific before/after
      numbers are recorded in this ticket's completion notes, not just a
      pass/fail.
- [x] The standing bench gate's other checks (sensors alive, wheels
      drive and encoders increment in the expected direction, round-trip
      over the real link) are confirmed as part of this session, per
      `.claude/rules/hardware-bench-testing.md`.
- [x] Bench-room lights confirmed on (Shelly relay per the rule file)
      before any camera/visual portion of the check that needs it — not
      applicable if this ticket's checks are stand-only with no camera
      dependency; confirm and note either way.

## Completion Notes (132-019)

### 0. Device confirmation and rig hygiene

`mbdeploy list` (re-probed fresh, not a remembered row) confirmed `tovez`
present at UID `9906360200052820a8fdb5e413abb276000000006e052820`, port
`/dev/cu.usbmodem2121202`, ROLE `NEZHA2`. Two other boards were on the hub
this session (`getez`, a RADIOBRIDGE relay, and `vevov`, another robot) —
neither was touched; every deploy/flash command below targets `tovez` by
UID explicitly. Stand-only bench session, no camera — the bench-room
lights rule does not apply; noted, not silently skipped.

### 1. Flash — sprint HEAD firmware

`just build` (full ARM toolchain): `MICROBIT.hex` built clean, RAM 98.33%
(120768/122816B, the project's known always-near-full baseline), FLASH
42.40% — consistent with ticket 018's own build numbers. `build.py`'s
version-bump side effect (`pyproject.toml`/`config/dotconfig.yaml`/
`uv.lock` → `0.20260804.1`) was reverted to `HEAD` after each build, per
this project's "no per-ticket version bumps" rule — `close_sprint` is the
only bump point. Flashed via
`uv run mbdeploy deploy --hex ./MICROBIT.hex 9906360200052820a8fdb5e413abb276000000006e052820`
— first attempt hit a locked-flash sector-erase failure (`result code
0x67`), which `mbdeploy` recovered automatically via its documented
CTRL-AP mass-erase retry (`.claude/rules/debugging.md`'s own "Gotchas"
section); the retry flashed clean (77 sectors erased/programmed).

### 2. Baseline (uncorrected), this session — 5 runs, one boot

`velocity_profile_gate.py --port /dev/cu.usbmodem2121202 --board tovez`,
run 1 immediately after the flash's own reset (the true cold-boot
baseline), runs 2–5 back-to-back with nothing changed between them
(`019_run1_coldboot_pre` .. `019_run5_warm4_pre`, `019_summary.csv`):

| run | boot state | trapezoid L / R plateau | gap | square L / R plateau | gap |
|---|---|---:|---:|---:|---:|
| 1 | cold (first run post-flash) | 83.1% / 70.8% | 12.3 | 82.2% / 76.0% | 6.2 |
| 2 | warm (same boot) | 86.0% / 76.5% | 9.5 | 85.0% / 78.3% | 6.7 |
| 3 | warm (same boot) | 87.4% / 78.6% | 8.8 | 89.8% / 78.3% | 11.5 |
| 4 | warm (same boot) | 89.6% / 79.2% | 10.4 | 89.8% / 79.3% | 10.5 |
| 5 | warm (same boot) | 91.7% / 80.7% | 11.0 | 89.8% / 79.3% | 10.5 |

Monotonic left/right improvement across runs, consistent with the
existing "cold-to-warm convergence" finding this file already documents
(`velocity_profile_gate.md`). Run 5's numbers (91.7/80.7 trapezoid,
89.8/79.3 square, gap 10.5–11.0 points) closely reproduce the 2026-08-03
motor-survey headline (L 96.0% / R 84.9%, gap 11.1) that motivated this
ticket — same physical defect, confirmed still present on this session's
build.

### 3. Correction computed

Averaged run 5's trapezoid+square plateau tracking (the most-converged
point measured this session): left (91.7+89.8)/2 = 90.75% →
`wheel_gain_left_accel = wheel_gain_left_decel = 0.9075`; right
(80.7+79.3)/2 = 80.0% → `wheel_gain_right_accel = wheel_gain_right_decel =
0.800`. Both intercepts held at 0 (gain-only correction), per the
architecture's own design intent: one shared population-scale
`duty_per_speed`, all per-wheel deviation expressed as `wheel_gain`/
`wheel_intercept` — `App::Drive::correctedCommand()`'s inverse-map
`magnitude = (|desired| - intercept) / gain` means setting `gain` to the
measured tracking ratio boosts the commanded (pre-duty) speed by
`1/ratio`, which — for a linear plant — lands the wheel back at the
originally-desired speed.

### 4. Live wire push — demonstrated, with a real methodological finding

First attempt (push via a standalone script, then verify via a SEPARATE
`velocity_profile_gate.py` invocation, `019_run6_warm_postpush1`) showed
**no improvement** (trapezoid L 91.7%/R 78.6%, essentially unchanged from
run 5). Root-caused, not shrugged off: `SerialConnection.connect()` over
direct USB pulses DTR on every open, which resets `tovez`'s
microcontroller (`serial_conn.py`'s own doc comment — confirmed
empirically: a value read back correctly immediately after a push
reverted to baked identity after nothing but a disconnect+reconnect, no
power cycle). `DRIVE` is not in `Configurator::persistIfEligible()`'s
flash-persistence table (only `WHEEL_CONTROL`/`MOTORS`/`OTOS` are) — a
live `DRIVE` push is RAM-only by design, so it cannot survive the very
next reconnect, let alone a genuine cold boot. Run 6 is kept in
`src/tests/bench/output/` for transparency (a discarded methodology step,
not evidence of anything about the correction itself) rather than
deleted.

Fixed by measuring in the SAME connection as the push
(`019_sameboot_A_preop` → push → `019_sameboot_B_postpush`,
one connection, no reconnect in between):

- Pushed via 4× `NezhaProtocol.set_config_field(ConfigTarget.DRIVE, ...)`
  (`wheel_gain_left_accel`, `wheel_gain_left_decel`,
  `wheel_gain_right_accel`, `wheel_gain_right_decel`) — all 4 acks
  `ok=True, err_code=0`.
- **Read-back confirms it landed, not just acked**: `get_config(DRIVE)`
  immediately after the push returned exactly the pushed values
  (`0.9075.../0.800...`, floating-point-exact modulo the wire's float32
  round-trip), and the 7 untouched `DRIVE` fields (`duty_per_speed_left`,
  `duty_per_speed_right`, `crawl_pulse`, all 4 intercepts) were confirmed
  byte-identical before/after — the push changed exactly what it was
  asked to change.
- **Gap closed, same boot, immediately**:

  | profile | before (plateau L/R, gap) | after (plateau L/R, gap) |
  |---|---|---|
  | trapezoid | 89.6% / 78.6%, gap 11.0 | 102.5% / 104.1%, gap 1.6 |
  | square | 90.7% / 78.3%, gap 12.4 | 101.3% / 101.9%, gap 0.6 |

  Both wheels overshoot 100% by 2–12 points — most likely a mix of
  continued physical warm-up during the `postpush` measurement (it runs
  strictly after `preop`, on an already-driven plant) and ordinary
  run-to-run noise on a single ~1.5 s plateau sample. Not chased with a
  second tuning pass, per this ticket's own instruction not to iterate
  until a run looks good and report only that one — this is the actual
  first measured result of the actual first correction attempt.

### 5. Baked into `tovez.json`, rebuilt, reflashed — the path that survives a cold boot

Since a live `DRIVE` push cannot survive any reset by construction (§4),
the correction was also written into
[`data/robots/tovez.json`](../../../../data/robots/tovez.json)'s
`drive.wheel_gain_left_accel/decel` (0.9075) and
`drive.wheel_gain_right_accel/decel` (0.800), documented in a new
`drive._wheel_correction_132_019_note` (explains why fitting `wheel_gain`
from encoder-measured `velocity_profile_gate.py` data is no longer the
circular-fit failure the 2026-07-31 `_wheel_correction_note` warned
against, now that `duty_per_speed` is an independently-validated, held-
fixed population constant per ticket 009). Rebuilt (`just build`, same
RAM/FLASH numbers as §1, version-bump side effect reverted again) and
reflashed `tovez` by UID (clean flash this time, no lock).

**Read-back confirms the bake landed, with zero wire traffic**:
`get_config(DRIVE)` on a fresh connection (no push this session) read
`wheel_gain_left_accel/decel = 0.9075`, `wheel_gain_right_accel/decel =
0.800` — the file's own values, loaded by `Configurator::loadBaked()` at
boot.

### 6. Cold boot, post-reflash — the real acceptance numbers

`019_run7_coldboot_post1` (first connection after the reflash — a genuine
cold boot, correction baked, no wire push) and `019_run8_warm_post1`
(second connection, same boot state as far as the bake is concerned since
the bake survives every reset):

| run | boot state | trapezoid L / R plateau | gap | square L / R plateau | gap |
|---|---|---:|---:|---:|---:|
| 7 | **cold (first run post-reflash)** | **97.5% / 101.9%** | **4.4** | **101.3% / 97.2%** | **4.1** |
| 8 | warm (same boot) | 99.6% / 102.0% | 2.4 | 103.2% / 97.2% | 6.0 |

**Headline: the 11.1-point L/R gap (motor-survey baseline) — reproduced
this session at 10.5–11.0 points warm, 12.3 cold — closes to 4.1–4.4
points on the very first run after a cold reflash**, and both wheels now
land within a few points of 100% of commanded plateau speed instead of
the 70–90% range measured throughout §2's uncorrected baseline. The
trapezoid profile's full-window delivered/commanded RATIO (106.7%/106.7%
at run 7, includes tail dynamics beyond the plateau) technically misses
the gate's own ±5% PASS band on both wheels — but the L/R IMBALANCE for
that same run is 1.0000 (perfectly matched), i.e. both wheels now overshoot
together by the same amount rather than one wheel dragging behind the
other. The gap-closing property this ticket is about is real and
verified; per-wheel absolute accuracy to exactly 100% is a separate,
smaller residual left for a future tuning pass, not chased further here.

### 7. Standing bench gate — other checks

- **Round-trip / identify / liveness**: `twist_drive.py` — 6/6 PASS
  (`connect()`, `move_twist()` corr_id + ack, encoders moving during the
  move, `estop()` corr_id + ack).
- **Encoders, both directions**: forward direction confirmed throughout
  every `velocity_profile_gate.py` run above (positions climbing in the
  commanded direction); reverse confirmed separately (`wheels(-120,
  -120, ...)` × 20 ticks → encoder positions moved to `(-100, -87)` from
  a `(0, 0)`-relative start — negative/reverse, as commanded).
- **Other sensors**: OTOS reported plausible, changing values with the
  valid-data flag set (`flags` bit 0) during the reverse check
  (`otos=(48, -3, 5)`). Line/color read back `None` (flags bits 13/14
  clear) — not a fault; this ticket did not confirm whether `tovez` has
  those sensors physically wired, and neither is in this ticket's or this
  sprint's scope (`Configurator::install(ESTIMATOR)` and the OTOS domain
  fix are ticket 010's territory, already closed).
- **Bench-room lights**: not applicable — this session is stand-only,
  direct USB, no camera; noted per the rule file rather than silently
  skipped.

### 8. Files

All committed under `src/tests/bench/output/`: `019_run1_coldboot_pre`
through `019_run5_warm4_pre` (uncorrected baseline, 5 runs one boot),
`019_run6_warm_postpush1` (discarded methodology step, kept for
transparency — see §4), `019_sameboot_A_preop` / `019_sameboot_B_postpush`
(the valid same-connection live-push demonstration), `019_run7_
coldboot_post1` / `019_run8_warm_post1` (post-reflash, baked correction —
the acceptance numbers), `019_summary.csv` (every run above, one row per
profile per wheel). `velocity_profile_gate.md` (the standing report this
sprint's own baseline was measured into) has a new "2026-08-04 update"
section summarizing all of the above and cross-linking the files.

## Testing

- **Existing tests to run**: n/a — this ticket IS a hardware test.
- **New tests to write**: none — `velocity_profile_gate.py` already
  exists.
- **Verification command**: `uv run python
  src/tests/bench/velocity_profile_gate.py --port <tovez's live port,
  taken fresh from mbdeploy list, never a remembered port number>`
  (before and after).

## Implementation Plan

**Approach**: Probe for `tovez` first, confirm UID match. Run baseline
`velocity_profile_gate.py`. Push or bake the correction (whichever
ticket 018's findings suggest is more representative of the sprint's
actual end state — likely the baked/reshaped path, since that's what a
real deployment would use, with the live wire push as the demonstration
of the wire capability itself). Cold-boot the robot. Re-run
`velocity_profile_gate.py`. Record before/after numbers.

**Files to modify**: none expected (this is a verification run, not code
changes) — unless the bench run surfaces a genuine bug, in which case
it's reported and the fix belongs in the owning earlier ticket (reopened),
not patched here.

**Testing plan**: as above.

**Documentation updates**: this ticket's own completion notes ARE the
documentation — before/after numbers, port used, UID confirmation,
cold-boot method (power-cycle vs. reflash).
