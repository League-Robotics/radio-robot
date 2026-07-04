---
id: '007'
title: 'HITL validation: bench scripts and stand/coupled-rig verification'
status: open
use-cases:
- SUC-008
depends-on:
- '006'
github-issue: ''
issue: greenfield-rebuild-faceplate-hal-in-a-fresh-source-old-tree-parked.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# HITL validation: bench scripts and stand/coupled-rig verification

## Description

Run the sprint's exit gate for real: build, flash, and exercise the new
dev-loop firmware on the stand (robot `tovez`, four motors wired) and on the
coupled two-motor bench rig (ports 3 and 4, mechanically linked). This
ticket is the sprint's acceptance gate — per
`.claude/rules/hardware-bench-testing.md`, "a sprint is not 'done' on tests
alone — it must be seen working on the stand." No new source code is
expected from this ticket; if the bench pass surfaces a real bug in tickets
1-6's work, fix it in this ticket (and note which prior ticket's acceptance
criteria the fix corresponds to) rather than deferring — this is the last
ticket in the sprint.

## Acceptance Criteria

All bullets below mirror the linked issue's "Verification" section
verbatim; each must be observed and recorded (log output, TLM excerpt, or
screen capture as appropriate), not just asserted.

- [ ] **Build**: `python build.py --clean` produces the new hex;
      `source_old` is untouched (confirm via `git status`); rollback
      (`codal.json` `application: source_old` + rebuild) still works.
- [ ] **Flash**: `mbdeploy deploy robot --hex …` with the ROLE check passing
      (never a blind `cp` to `/MICROBIT` — see
      `.clasi/knowledge/verify-microbit-before-flashing.md`).
- [ ] **Bench, single motor** (robot on the stand, wheels free):
  - `DEV M 1 DUTY 30` → wheel spins; `DEV M 1 STATE` reports `applied=0.30`
    and `position` climbing.
  - `DEV M 1 VEL 120` → converges; capture applied-duty-vs-measured-velocity
    over the step response (sanity check for the embedded PID — no
    formal tolerance required, just a plausible converging step response).
  - `DEV M 1 VOLT 3` → `ERR unsupported`.
  - `DEV M 1 RESET` → position rezeroes.
- [ ] **Bench, drivetrain**:
  - `DEV DT VW 150 0 0` → both wheels approximately equal.
  - Hand-drag one wheel → **both** wheels slow, ratio held (observable via
    `DEV DT STATE`) — the governor is doing something, not coasting.
- [ ] **Watchdog**: stop sending commands mid-motion → motors reach neutral
      within the configured window (default ~1 s).
- [ ] **Host script — `dev_exercise.py`**: scripts the above sequence over
      `NezhaProtocol.send()`, run once over direct serial and once over the
      relay's `!GO` data plane; both pass.
- [ ] **Interactive — `velocity_chart.py`**: run live while hand-loading
      wheels; visually confirm the wheel-velocity/applied-duty panels track
      real behavior and the vR-vs-vL phase plot shows the ratio governor's
      diagonal. Used at this step to tune the in-motor PID gains if the step
      response from the single-motor bullet above looked implausible —
      record any gain changes made and where (`MotorConfig.vel_gains`
      defaults) they should be persisted for future bench sessions.
- [ ] **Coupled-rig acceptance** (ports 3 and 4, mechanically linked — running
      one loads the other):
  - `pid_hold_speed.py` PASS: motor-3 measured velocity stays inside a
    tolerance band and recovers within a bounded settle time after each load
    step (assist → freewheel → drag → reverse on motor 4), with applied
    duty visibly rising as load increases.
  - `ratio_governor_curve.py` PASS: with `DEV DT PORTS 3 4` and an unequal
    wheel-target curve, the governor lowers BOTH targets so the measured
    wheel-speed ratio holds the commanded ratio within tolerance; re-run
    with the governor off (`sync_gain=0`) and confirm the ratio visibly
    drifts (the required negative control).
- [ ] Any defect found in tickets 1-6's work during this bench pass is fixed
      in this ticket, with a note identifying which prior ticket's
      acceptance criterion was not actually met and why the bench pass
      caught it where the build-only gate did not.

## Testing

- **Existing tests to run**: `uv run python -m pytest` (regression check —
  nothing in tickets 1-6 should have broken the new tree's pytest
  collection); all bench scripts from ticket 6 (`dev_exercise.py`,
  `pid_hold_speed.py`, `ratio_governor_curve.py`, `velocity_chart.py`).
- **New tests to write**: None expected — this ticket exercises what ticket
  6 built. If a bench-pass fix requires new coverage (e.g., a regression the
  bench pass caught that a unit test could have caught earlier), add it to
  `tests/unit/` and note why it wasn't in ticket 3/4's original scope.
- **Verification command**: `python build.py --clean`, `mbdeploy deploy
  robot --hex build/...`, then the manual/scripted bench sequence above. No
  single command captures the full gate — this ticket's acceptance is the
  full checklist, observed on real hardware.
