---
status: in-progress
sprint: '113'
tickets:
- 113-001
- 113-002
- 113-003
- 113-004
- 113-005
- 113-006
- 113-007
---

# Config-as-truth: sim configures-on-open; no tunable defaults in source

**Requirement (stakeholder 2026-07-20):** No tunable/behavioral drive or sim
parameter may have a hardcoded default in the source. Every behavioral
parameter must come from the robot configuration file — the single source of
truth. If you try to run without configuration, the device fails closed and
tells you it is not configured.

## Motivation

The sim is tuned in `src/sim/sim_harness.h::makeExecutorConfig` and the robot's
firmware defaults live in `src/scripts/gen_boot_config.py` — **separate files
with separate values.** During the model-reference motion work, `heading_kp`
was simultaneously `6.0` (firmware default), `2.5` (sim), and `1.0` (robot
JSON). Tuning the sim never reached the robot, and the sim and the TestGUI
diverged (the deadband stall reproduces under one config but not another). The
validated values now live in `data/robots/tovez_nocal.json`, but **nothing
reads them in the sim yet** — the sim boots from the hardcoded
`makeExecutorConfig` and has no config-push surface (see the note in
`src/host/robot_radio/io/sim_loop.py`).

Deleting the code defaults makes the whole class of "I tuned the wrong copy"
bug impossible: one file is the truth, and the sim and the robot both fail if
they don't get it.

## Requirements

1. **No behavioral defaults in code.** Delete hardcoded behavioral defaults:
   `sim_harness.h::makeExecutorConfig`/`makeMotorConfig`, the `*_DEFAULT`
   constants in `gen_boot_config.py`, `nezha_motor`'s `kDefaultOutputDeadband`/
   `kDefaultReversalDwell`, and `App::Pilot`'s `modelTauLin_`/`modelTauAng_`.
   **Boundary:** only *behavioral* parameters (gains, deadband, max speeds,
   trackwidth, modelTau) come from config; structural invariants (array sizes,
   protocol version, `kWheelCount`, math constants) stay compile-time — they
   are the code's identity, not tunables.

2. **Sim configures-on-open** *(this sprint's slice)*. Add a config surface to
   the sim; the host/`SimLoop` sends the configuration — read from the robot
   config file (`data/robots/*.json`) — as part of opening the sim. After
   this the sim runs the same config as the robot, closing the sim/GUI
   divergence and letting motion tuning be tested identically in sim and on
   the bench.

3. **Fail-closed when unconfigured.** An unconfigured device refuses all
   *motion* and answers "not configured". Keep a minimal always-live rump
   (accept the config command, STOP, PING/ID). Config-accept validates
   *completeness* — reject partial config; flip to "ready" only when every
   required parameter is present.

4. **Version-erase persisted config.** The device persists its config but
   stamps it with the firmware version and *wipes* it on a version mismatch —
   a reflash forces reconfiguration, so stale config whose field meanings
   changed between versions can never silently survive a software update.

## Scope note

Items 1, 3, and 4 may be follow-on sprints. **Item 2 (sim configures-on-open)
is the highest-value first slice** and the target of the sprint opened now: it
is what lets the tuning I've validated in the sim actually be the thing the
robot runs, and what makes the deadband-compensation work that follows testable
against the real config.
