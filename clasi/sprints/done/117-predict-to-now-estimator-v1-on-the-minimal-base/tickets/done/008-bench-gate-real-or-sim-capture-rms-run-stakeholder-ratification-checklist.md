---
id: 008
title: "Bench gate \u2014 real-or-sim capture, RMS run, stakeholder ratification checklist"
status: done
use-cases:
- SUC-062
depends-on:
- '004'
- '006'
- '007'
github-issue: ''
issue: predict-to-now-odometry-estimator-ring-capture-dump-validation-trajectory-controller.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Bench gate — real-or-sim capture, RMS run, stakeholder ratification checklist

## Description

Close the sprint's validation loop against real (or, if unavailable,
substitute simulated) data, and hand the stakeholder a durable artifact
to ratify RMS accept thresholds from. The robot's motor I2C bus dropped
during sprint 116's own bench gate (tracked separately in
`clasi/issues/bench-motor-bus-disconnect-during-116-gate.md`, a physical
issue needing a stakeholder reseat — NOT resolved by this ticket, and NOT
linked to this ticket's frontmatter since this ticket does not fix it,
only works around it for validation purposes). This ticket's own gate
logic:

1. Check hardware presence and motor-bus health FIRST (`conn=`/bus-health
   telemetry flags) before attempting to drive anything.
2. If the bus has recovered: run `estimator_capture.py` (ticket 006) for
   real, then `estimator_validation.ipynb` (ticket 007) against the real
   CSV.
3. If the bus is still down: sim-mode capture (`SimLoop` + `tlm_log.py`,
   already a proven path per sprint 115) IS the dataset — run the full
   RMS analysis on it, and record plainly that this is a sim substitute,
   not silently presented as a bench result.
4. Author `docs/bench-checklists/sprint-117-estimator-v1.md` — a
   stakeholder-run real-hardware re-verification checklist for whenever
   the bus is confirmed recovered, mirroring the existing
   `docs/bench-checklists/sprint-116-move-protocol.md` precedent.

RMS accept thresholds are PROPOSED by ticket 007's notebook output — this
ticket presents them for stakeholder review, it does not self-ratify
them.

## Completion Notes (2026-07-22)

**Bus health, checked BEFORE any drive command**: `mbdeploy probe` confirmed
the target robot (UID `9906360200052820a8fdb5e413abb276000000006e052820`,
ROLE=NEZHA2, NAME=robot, port `/dev/cu.usbmodem2121102`). `just build-clean`
+ `mbdeploy deploy` succeeded (firmware v0.20260722.1, boot banner
`DEVICE:NEZHA2:robot:tovez:2314287040`). A passive read of 10 binary TLM
frames (zero drive commands issued) showed `conn_left`/`conn_right`/
`otos_present` all `False`, `flags=2240` — the **exact same** signature
documented in `clasi/issues/bench-motor-bus-disconnect-during-116-gate.md`
and sprint 116's own bench checklist. **The bus has NOT recovered since
sprint 116.** This is a physical/electrical condition; sprint 117 touched
no motor-bus/I2C driver code, so this is not a regression from this
sprint's own work — bus-connectivity observation only, no power/battery
attribution.

**PING `t=<ms>` (ticket 001) verified LIVE against real firmware**:
`SerialConnection.connect()`'s own readiness poll sent a raw, un-suffixed
`PING` and received `OK pong t=<ms>` (observed `t=11204`, `t=19960`,
`t=12040` across three separate connects this session) — confirms ticket
001's firmware change is live and working over the real serial link.
A round trip via `NezhaProtocol.send("PING")` (the corr-id-suffixed path)
was NOT attempted — that is a separate, already-documented gap (ticket
001's own flag, `src/host/robot_radio/DESIGN.md` §6), not worked around
here per this ticket's own instructions.

**Contingency taken (bus still down → sim-mode substitute, explicitly NOT
real bench data)**:
1. `uv run python src/tests/bench/estimator_capture.py --sim --csv
   src/tests/bench/out/estimator_capture_sprint117_sim.csv` — 134 rows,
   8-segment `DEFAULT_PATTERN`, committed as a durable capture artifact.
2. `uv run jupyter nbconvert --to notebook --execute --inplace
   src/tests/notebooks/estimator_validation.ipynb` — ran end-to-end with
   **zero source-cell changes** (`CSV_PATH` left at its committed default
   `None`, which self-generates its own fresh sim capture, 134 rows,
   confirmed via `git diff` that only output cells + the internal capture
   CSV changed).

**RMS one-step-ahead residual, by stream × phase** (full table in
`docs/bench-checklists/sprint-117-estimator-v1.md` §3b):
`enc_left_position`/`enc_right_position` steady ≈0.075mm, ramp/reversal/
pivot ≈1.3–1.8mm; `enc_left_velocity`/`enc_right_velocity` steady ≈1.2mm/s,
ramp/reversal/pivot ≈21–32mm/s; `heading` steady 0.0000rad, pivot 0.0070rad
— order-of-magnitude consistent with ticket 006's own independent
verification run and ticket 005's C++ sim-system harness.

**ZOH lag-signature verdict** (`forward_step` ramp window): velocity error
2.22× theory → **PASS** (within 3×); distance error 5.26× theory → **FAIL**
(exceeds 3×) — the sim plant's dead-time-then-near-step onset (not a smooth
ramp) concentrates residual atypically for the classical `a·k`/`½·a·k²`
formula; evidence against a fit-based ramp predictor buying much over ZOH
here, not a sign the estimator is broken.

**Leg-level projection** (random-walk `√N` bound, not literal
dead-reckoning): straight leg (`forward_step`, 24 steps) → 0.3663mm
projected position error; pivot leg (`pivot_ccw`, 19 steps) → 0.030415rad
(1.74°) projected heading error.

**PROPOSED accept thresholds — explicitly NOT RATIFIED by this ticket or
its executing agent** (2× measured steady/pivot-phase RMS, ONE simulated
dataset, one seed): `enc_left/right_position` 0.1495mm,
`enc_left/right_velocity` 2.4420mm/s, `heading` 0.0140rad. Ratifying,
rejecting, or retuning these against real bench data is the stakeholder's
own call — `docs/bench-checklists/sprint-117-estimator-v1.md` §5 gives the
exact re-run steps for once the bus is confirmed reseated.

**Final sweep**: `just build-clean` PASS (firmware v0.20260722.1, FLASH
37.03%, RAM 98.33%); `uv run python -m pytest` PASS — 1242 passed, 13
skipped, 10 xfailed, 1 xpassed, 1 warning (the one warning is a pre-existing,
unrelated `PytestUnhandledThreadExceptionWarning` in a TestGUI background
thread, `test_set_origin.py`; not touched by this ticket's changes; the
test itself still passes).

Full details, setup commands, and the stakeholder re-verification recipe:
`docs/bench-checklists/sprint-117-estimator-v1.md`.

## Acceptance Criteria

- [x] Hardware presence and motor-bus health (`conn=`/bus-health
      telemetry flags) are checked and the result recorded BEFORE any
      attempt to drive the robot.
- [ ] If the bus has recovered: a real `estimator_capture.py` run
      produces a bench CSV, and `estimator_validation.ipynb` runs
      end-to-end against it, producing real RMS tables.
      **UNCHECKED — bus did NOT recover this session** (same
      `conn_left`/`conn_right`/`otos_present`-all-`False`, `flags=2240`
      signature as sprint 116's gate; see Completion Notes). This branch
      was not executed; honestly left unchecked rather than marked N/A.
- [x] If the bus is still down: a sim-mode capture (via `SimLoop`)
      produces the CSV instead, the notebook runs against it, and this
      substitution is EXPLICITLY recorded in this ticket's completion
      notes and in the bench checklist — never silently presented as
      real bench data.
- [x] `docs/bench-checklists/sprint-117-estimator-v1.md` created,
      structured for a stakeholder-run real-hardware re-verification
      (mirrors `docs/bench-checklists/sprint-116-move-protocol.md`'s
      shape): steps to confirm bus health, run the capture, run the
      notebook, and review the RMS tables.
- [x] `uv run python -m pytest` (full suite) and `just build-clean` both
      pass as the sprint-closing verification, per
      `.claude/rules/hardware-bench-testing.md`.
- [x] This ticket's own completion notes present the RMS tables/ZOH-lag-
      signature verdict for stakeholder review and explicitly state that
      accept thresholds are PROPOSED, not ratified by this ticket or its
      executing agent.

## Implementation Plan

**Approach.** Gate-check first, capture second — never assume hardware
availability. Reuse `mbdeploy probe`/telemetry `conn=`/bus-health flags
(same discipline `.claude/rules/hardware-bench-testing.md` and this
project's `disconnected-bus-signature-tlm-conn` knowledge entry already
establish) to decide real-vs-sim BEFORE issuing any drive command.

**Files to create:**
- `docs/bench-checklists/sprint-117-estimator-v1.md`.
- A captured CSV artifact (real or sim-substitute; path convention
  matching `src/tests/bench/out/` per `tlm_log.py`'s own `DEFAULT_CSV`
  precedent).

**Files to modify:** none expected — this is a verification/documentation
ticket, not a code-change ticket.

**Documentation updates:** `docs/bench-checklists/sprint-117-estimator-v1.md`
(new).

## Testing

- **Existing tests to run**: full `uv run python -m pytest`; `just
  build-clean`; `mbdeploy deploy` (hex by full UID) if hardware is
  present and its bus has recovered.
- **New tests to write**: none (this ticket exercises existing tooling
  from tickets 002-007 against real or sim data, it does not add new
  test code).
- **Verification command**: `uv run python -m pytest`; `just build-clean`;
  bus-health check via telemetry `conn=`/`kFlagConnLeft`/`kFlagConnRight`
  flags before any drive command.
