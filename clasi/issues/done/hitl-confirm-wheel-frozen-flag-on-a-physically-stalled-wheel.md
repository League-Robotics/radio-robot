---
status: done
priority: medium
---

# HITL: confirm the wheel-frozen flag on a physically stalled wheel

Sprint 129 ticket 002 landed the wheel-frozen fault flag (telemetry bits
19/20, `Health::wheelFrozenLeft/Right`, host decode in `protocol.py` with
`wheel_frozen_reason()`), with firmware gating tests, host decode unit
tests, and the **negative** bench case verified on hardware: a healthy
700 mm leg at 200 mm/s raised neither flag over 143 telemetry frames
(the ticket's own text calls a false positive here "worse than no flag at
all", so this was the higher-priority check).

The **positive** case was not confirmed: it requires a person to
physically hold a wheel stationary while it is commanded, which an
autonomous agent session cannot do. The agent declined to fabricate a
pass — recorded honestly rather than checked off.

## What to run

A bench script is ready:

```bash
uv run python src/tests/bench/wheel_frozen_bench.py \
    --port /dev/cu.usbmodem2121102 --stall-wheel left
```

Hold the named wheel stationary while it runs; repeat with `right`.
Expected: the matching per-wheel flag sets within ~0.5 s and
`wheel_frozen_reason()` names that wheel.

## Why it matters beyond visibility

Sprint 129 ticket 007's adaptive duty-per-speed learner uses this flag as
a guard — a stalled wheel must never teach the gain learner. If the
positive case does not actually fire, that guard is silently absent.
