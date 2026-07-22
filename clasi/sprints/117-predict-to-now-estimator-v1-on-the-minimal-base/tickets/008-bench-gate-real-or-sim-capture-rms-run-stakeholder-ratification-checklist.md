---
id: 008
title: "Bench gate \u2014 real-or-sim capture, RMS run, stakeholder ratification checklist"
status: in-progress
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

## Acceptance Criteria

- [ ] Hardware presence and motor-bus health (`conn=`/bus-health
      telemetry flags) are checked and the result recorded BEFORE any
      attempt to drive the robot.
- [ ] If the bus has recovered: a real `estimator_capture.py` run
      produces a bench CSV, and `estimator_validation.ipynb` runs
      end-to-end against it, producing real RMS tables.
- [ ] If the bus is still down: a sim-mode capture (via `SimLoop`)
      produces the CSV instead, the notebook runs against it, and this
      substitution is EXPLICITLY recorded in this ticket's completion
      notes and in the bench checklist — never silently presented as
      real bench data.
- [ ] `docs/bench-checklists/sprint-117-estimator-v1.md` created,
      structured for a stakeholder-run real-hardware re-verification
      (mirrors `docs/bench-checklists/sprint-116-move-protocol.md`'s
      shape): steps to confirm bus health, run the capture, run the
      notebook, and review the RMS tables.
- [ ] `uv run python -m pytest` (full suite) and `just build-clean` both
      pass as the sprint-closing verification, per
      `.claude/rules/hardware-bench-testing.md`.
- [ ] This ticket's own completion notes present the RMS tables/ZOH-lag-
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
