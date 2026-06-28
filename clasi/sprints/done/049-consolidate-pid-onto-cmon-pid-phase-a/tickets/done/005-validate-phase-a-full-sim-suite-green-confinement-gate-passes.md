---
id: '005'
title: 'Validate Phase A: full sim suite green, confinement gate passes'
status: done
use-cases:
- SUC-004
depends-on:
- '004'
github-issue: ''
issue: consolidate-control-code-onto-vendored-libraries-cmon-pid-tinyekf.md
completes_issue: false
---

# Validate Phase A: full sim suite green, confinement gate passes

## Description

Final validation ticket for Sprint 049 Phase A. After tickets 001-004 are done,
run the full simulation test suite and vendor-confinement gate to confirm no
regressions have been introduced, the cmon-pid integration is clean, and the
tree is ready for Sprint 050 (Phase B).

This ticket introduces no code changes. Its only work is running the suite,
interpreting the results, and — if vendor_baseline.txt needs updating — making
that one-line change.

## Acceptance Criteria

- [ ] `uv run --with pytest python -m pytest tests/simulation -q` exits with
      exactly 2 failures: `test_default_config_pin.py::test_default_robot_config_unchanged`
      and `test_robot_config.py::TestSchemaValidation::test_tovez_validates_against_schema`.
      No other failures.
- [ ] `test_vendor_confinement_zero_hits_empty_baseline` passes (zero CODAL
      tokens above `source/io/`).
- [ ] `test_vendor_confinement_no_new_leaks` passes.
- [ ] `test_velocity_controller.py` is fully green.
- [ ] `test_motor_controller.py` is fully green.
- [ ] `test_body_velocity_controller.py` is fully green.
- [ ] `grep -r 'RatioPidController' source/` returns zero matches.
- [ ] `grep -r 'ratioPid' source/` returns zero matches.
- [ ] `grep -r 'pid\.\(kp\|ki\|kd\|max\)' tests/simulation/ --include="*.py"` returns zero matches.
- [ ] `grep -c '\bdouble\b' libraries/cmon-pid/cmon-pid.h` returns 0.

## Implementation Plan

### Approach

This ticket is a validation-only ticket. No source code is modified. The
programmer agent should:

1. Run the canonical suite and record the output:
   ```
   uv run --with pytest python -m pytest tests/simulation -q 2>&1 | tail -20
   ```

2. Run the targeted confinement test alone for a clean signal:
   ```
   uv run --with pytest python -m pytest tests/simulation/unit/test_vendor_confinement.py -v
   ```

3. Run the key velocity-loop tests:
   ```
   uv run --with pytest python -m pytest \
     tests/simulation/unit/test_velocity_controller.py \
     tests/simulation/unit/test_motor_controller.py \
     tests/simulation/unit/test_body_velocity_controller.py \
     -v
   ```

4. Run the dead-code grep confirmations:
   ```
   grep -r 'RatioPidController' /Volumes/Proj/proj/RobotProjects/radio-robot-elite/source/ || echo "CLEAN"
   grep -r 'ratioPid' /Volumes/Proj/proj/RobotProjects/radio-robot-elite/source/ || echo "CLEAN"
   grep -r 'pid\.\(kp\|ki\|kd\|max\)' /Volumes/Proj/proj/RobotProjects/radio-robot-elite/tests/simulation/ --include="*.py" || echo "CLEAN"
   grep -c '\bdouble\b' /Volumes/Proj/proj/RobotProjects/radio-robot-elite/libraries/cmon-pid/cmon-pid.h
   ```

5. Check `tests/_infra/vendor_baseline.txt`. It should be empty (or contain only
   comment lines). If it has non-comment entries that didn't exist before this
   sprint, that is a regression — investigate and fix in the appropriate earlier
   ticket before closing this one.

### If validation fails

If any test beyond the 2 pre-existing failures is red, this ticket is NOT done.
Diagnose the failure and route back to the appropriate ticket (001-004) for a fix.
Do NOT mark this ticket done with open failures.

### Files to modify

None expected. If `tests/_infra/vendor_baseline.txt` requires a content change
(which is not expected — it should remain empty), update it and document the
rationale.

### Testing plan

The testing plan IS this ticket. Canonical command:

```
uv run --with pytest python -m pytest tests/simulation -q
```

IMPORTANT: Do NOT use bare `uv run pytest` — that uses an ephemeral interpreter
missing project dependencies and falsely reports mass failures.

### Documentation

No documentation changes. The sprint architecture-update.md already documents
the Phase A completion criteria. This ticket's done status signals readiness for
Sprint 050 planning.
