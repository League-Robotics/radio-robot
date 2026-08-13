# Heading hold makes a streamed GO_TO momentarily rest — one failing sim scenario

**Status:** open, needs a stakeholder call
**Raised:** 2026-08-13, out of process, playfield session
**Caused by:** `64a55f2f` (heading hold re-enabled on the OTOS, `heading_hold_gain` 0.0 → 1.0)

## The situation in one line

Re-enabling heading hold fixed a large, measured, hardware-verified straight-leg
veer — and broke exactly one sim scenario, which asserts that a streamed GO_TO
never comes to rest. Both facts are real. Which one wins is a product call, not
something to settle by tweaking a threshold.

## What was gained

Measured on `tovez` on the playfield against overhead-camera truth
(median-of-7 rest fixes, per-segment boundary poses):

| | `heading_hold_gain = 0.0` | `= 1.0` |
|---|---|---|
| straight-leg veer, mean | 5.64° | **0.98°** |
| straight-leg veer, worst | 18.66° | **2.20°** |

Six camera-supervised 400 mm square tours after the change closed at
15.8, 22.3, 9.7, 21.5, 40.1 and 8.0 mm, with leg lengths 39.75–40.17 cm
against a commanded 40.0. The 130-011 instability that originally condemned
this gain does **not** reproduce (500 mm leg: 4.86 s vs the 11.35 s failure,
zero wheel reversals on either wheel).

## What broke

`src/tests/sim/system/test_goto_protocol.py::test_goto_protocol_scenarios_pass`

The streamed-GO_TO scenario (EXTERNAL mode) asserts
`checkTrue(!everRestedDuringStream, ...)` — the robot must flow through
waypoints **without** stopping, and a mid-sequence gap in target updates must
neither fault nor halt it. With heading hold on, a genuine at-rest sample
(both wheel velocities under `kMinMovingVelocity`) is observed before the
final target, and the assertion fails.

Causation is established, not assumed:

- gain `1.0` → fails; gain `0.0` (same tree, regenerated `boot_config.cpp`) → passes.
- Reverting the OTOS decode fix (`5bee6ce6`) independently does **not** fix it,
  so that change is not implicated.

Full suite at the time of writing: **1467 passed, 4 xfailed, 1 failed** — this
one.

## What was tried and rejected

Gating `applyHeadingHold()` on the profiled speed being above
`limits_.landing.settleRestVelocity`, on the theory that a differential applied
at zero mean velocity is an in-place pivot rather than a heading hold. It did
**not** fix the scenario, and it was reverted: the hardware verification above
was performed on the un-gated firmware, so shipping the gate would mean
shipping something other than what was measured.

The idea may still be right on its own merits — a correction that spins the
wheels in opposite directions at zero mean velocity is not holding a heading —
but it needs its own evidence, not this scenario's.

## Why there is no cheap fix

The obvious resolution is "don't apply heading hold to Navigator-issued moves."
There is no discriminator to hang that on: `Motion::Move`
(`src/firm/motion/planner/planner_types.h`) carries `id`, `kind`,
`velocityKind` and thresholds — nothing that says who issued it. Adding a
provenance field is a design change touching the Move type, its wire encoding,
and every producer, which is not a 1am unattended edit.

## The options, honestly

1. **Keep heading hold, accept the scenario failure.** Straight-leg veer is a
   daily, visible problem on every tour; streamed GO_TO EXTERNAL mode is not
   currently used by any bench or playfield script in this repo. Cheapest, and
   leaves a red test in the tree — which must not be silently `xfail`ed, because
   the scenario is asserting something real about streaming behavior.
2. **Add Move provenance** and apply heading hold only to directly-commanded
   Distance moves. Correct, more work, touches the Move type.
3. **Understand the rest first.** Nobody has yet established *why* heading hold
   produces a both-wheels-slow sample mid-stream. It may be a short dip at a
   waypoint handoff that a small amount of hysteresis in the scenario's own rest
   detector would ride over — in which case the firmware is fine and the test is
   over-strict. This is the cheapest thing to find out and should probably
   happen before 1 or 2 are chosen.
4. **Revert heading hold.** Restores a green suite and gives back an 18.7°
   worst-case veer. Not recommended.

## Reproduce

```bash
uv run python -m pytest src/tests/sim/system/test_goto_protocol.py -q   # fails

# flip the gain and regenerate to confirm causation
python3 -c "import json,pathlib;p=pathlib.Path('data/robots/tovez.json');d=json.loads(p.read_text());d['planner']['heading_hold_gain']=0.0;p.write_text(json.dumps(d,indent=2))"
uv run python src/scripts/gen_boot_config.py
uv run python -m pytest src/tests/sim/system/test_goto_protocol.py -q   # passes
git checkout -- data/robots/tovez.json && uv run python src/scripts/gen_boot_config.py
```

Note that the flip above rewrites `tovez.json`'s formatting; `git checkout` it
afterwards rather than editing the value back by hand.
