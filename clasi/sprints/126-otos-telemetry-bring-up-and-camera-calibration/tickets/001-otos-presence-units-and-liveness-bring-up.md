---
id: '001'
title: OTOS presence, units, and liveness bring-up
status: in-progress
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: otos-telemetry-bring-up-and-camera-calibration.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# OTOS presence, units, and liveness bring-up

## Description

The OTOS driver, telemetry frame, and boot config are already built and
wired (`Devices::RealOtos`, `TLMFrame`'s `otos*` fields,
`gen_boot_config.py`'s `otos_boot_config_values()`) — verified on hardware
2026-07-29, `STATUS` already reports `otos=1`. What is not known is what
the reported tuple's units are and whether it actually tracks motion; this
ticket establishes both from a measured move, not from source reading.

Create `src/tests/bench/otos_calibration_bench.py`, the shared script this
whole sprint grows one mode at a time (see sprint.md's Architecture
Design Rationale). This ticket adds its skeleton plus a `--mode units`
run:

1. **Preflight**: check playfield lights (`192.168.1.122`, `output: true`
   — `.claude/rules/playfield-testing.md`), open camera / create playfield
   / start detection, confirm tag 100 (robot centre) and tag 1 (field
   origin) are visible.
2. Confirm robot has reached `READY` with `connL=1`/`connR=1` and `STATUS`
   reports `otos=1`; confirm telemetry flag bit 0 is set.
3. Capture a camera-truth rest pose (median-of-7, per
   `.claude/rules/playfield-testing.md`'s segment-boundary convention).
4. Read the OTOS pose tuple at rest over several telemetry frames; confirm
   it holds steady (bounded jitter) while the robot is not moving.
5. Command **one** known straight-line move (tether-safe by construction —
   straight moves never wrap the cable). Capture the camera-truth pose
   again at rest after the move (post-settle, per the same convention).
6. Read the OTOS pose delta over the same interval. Compare the OTOS
   delta's raw magnitude to the camera-measured distance in both mm and cm
   interpretations; whichever interpretation matches the camera distance
   (after accounting for the already-applied `otos_linear_scale`, since
   the chip's on-board scale register is set at `begin()` — the read-back
   value already reflects the committed 1.067) settles the unit.
7. Print a clear units/liveness report: raw units, at-rest jitter bound,
   moving-vs-camera delta comparison, PASS/FAIL against acceptance
   criteria 1 and 2 of the issue.
8. `estop()` (never `stop()`) in a `finally` block on every connection
   opened, per `.claude/rules/playfield-testing.md`.

Reuse `square_tour.py`'s lights-preflight and camera-fix helpers and
`tlm_log.py`'s `otos_*` telemetry-column capture rather than
reimplementing either — do not fork new copies of that logic.

If `otos=0` is seen at any point, first check `connL`/`connR` and whether
`READY` was emitted (per the issue's "blocking hazard" section) before
concluding the chip is missing — a wedged external I2C bus produces the
identical symptom and needs a full power cycle, not debugging as a sensor
fault.

## Acceptance Criteria

- [ ] `otos_calibration_bench.py` exists with a `--mode units` entry
      point, reusing `square_tour.py`/`tlm_log.py` helpers (not forked
      copies).
- [ ] Script confirms `STATUS otos=1` and telemetry flag bit 0 set on a
      `READY` robot with `connL=1`/`connR=1`, and prints the result.
- [ ] Script measures one known straight-line move, compares camera truth
      to the OTOS delta, and prints an explicit units conclusion (mm or
      cm) with the supporting numbers — not asserted from source.
- [ ] Script confirms and prints that the OTOS pose holds steady at rest
      (bounded jitter) and changes under motion.
- [ ] Every connection path calls `estop()` (never `stop()`) in a
      `finally` block.
- [ ] Run against the real robot on the playfield; results (raw output,
      not paraphrased) included in the ticket's completion notes.

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/sim -q`
  (confirm no new regressions beyond the 11 known pre-existing failures).
- **New tests to write**: None — this is a hardware/playfield measurement
  script, not a unit-testable code path. `otos_calibration_bench.py` is a
  bench tool, same category as `tlm_log.py`/`square_tour.py` (not
  pytest-collected, per `src/tests/CLAUDE.md`).
- **Verification command**: `uv run python src/tests/bench/otos_calibration_bench.py --port <robot-port> --mode units`
