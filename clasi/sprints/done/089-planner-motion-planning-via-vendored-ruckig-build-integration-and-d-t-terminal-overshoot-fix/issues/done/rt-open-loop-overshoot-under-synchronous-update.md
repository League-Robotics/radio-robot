---
status: done
sprint: 089
tickets:
- 089-005
- 089-007
---

# RT open-loop over-rotation under the synchronous-update loop

## Context

Sprint 087's synchronous-update loop (`v0.20260707.1`) introduced a uniform one-tick
(~2-pass) latency on the Planner→Drivetrain→Hardware output path (Decision 6). The
`D`-distance terminal accuracy this shifted was recovered in ticket 087-009 with a
closed-form "stopping distance with a reaction time" retune in
`source/subsystems/planner.cpp`.

`RT` (open-loop rotation) accuracy was **not** recovered and was left as documented,
strict `xfail`s — a deliberate scope decision, not a miss.

## Problem

`RT`'s overshoot is driven by the **smooth ramp-down's post-fire coast** — a different
mechanism than the `D`-distance reaction-time issue the 087 retune addressed — so `RT`
over-rotates (~+9.3° past a commanded 90°, both directions). This compounds with the
pre-existing RT open-loop over-rotation behavior (see prior turn-over-rotation history).

## Currently xfailed (the acceptance targets)

- `tests/sim/unit/test_motion_commands_arc_turn.py::test_rt_rotates_about_90_degrees_and_emits_done_rot`
- `…::test_rt_negative_relangle_rotates_the_opposite_direction`
- `tests/sim/system/test_tour_geometry.py` — both tests (their D-legs now pass; the
  RT-leg heading check still fails)
- (see `tests/sim/unit/test_motion_overshoot_regression.py` for the RT regression bar)

## Desired

Tighten `RT` open-loop rotation accuracy under the new loop — anticipate the ramp-down
coast (analogous to the reaction-time D fix) so a commanded rotation lands within
tolerance — and **un-`xfail`** the tests above. Must not regress the `D`/drive accuracy
009 already recovered, nor the RT/ROTATION closed-loop paths.
