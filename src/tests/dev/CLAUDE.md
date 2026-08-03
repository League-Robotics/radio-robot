# `dev/` — development scratch. Nothing here is a standing test.

Scripts, notebooks, and captured artifacts that were written to answer one
question during one investigation, on hardware or in sim, and have since
stopped earning their keep. Created 2026-08-03 by relocating them out of
`src/tests/bench/`.

**Read this before reviving anything in here.** Most of it targets protocol
surfaces or hardware rigs that no longer exist.

## The rule

- Nothing in `dev/` is pytest-collected, gates anything, or carries a
  maintenance obligation.
- A red or broken file in `dev/` blocks nothing.
- The sweep may prune it. If you need something here, revive it deliberately:
  move it back to `bench/` and fix whatever it targets.
- **`bench/` is the opposite**: current HITL tools and standing gates that we
  use persistently. Do not add scratch there.

## What is here and why it stopped working

### Coupled-rig family — quarantined by standing stakeholder rule

`rig_dev.py` (the `Rig` class), `rig_drive.py`, `rig_soak.py`,
`rig_stress.py`, `friction_rig_soak.py`, `otos_drift.py` (imports `Rig`),
plus the notebooks that drive them (`drivetrain_stress`, `motion_control`,
`turn_hitl`, `sensors`, `friction_coupling_sweep`) and that sweep's own
CSV/PNG.

The rig is a two-motor friction-coupled bench fixture, separate from the
robot. Standing rule: **the rig stays quarantined until the stakeholder says
"rig".** `rig_dev.py` also calls the deleted `NezhaProtocol.twist()`.

One live dependency remains, deliberately: `src/tests/unit/test_rig_dev.py`
covers `waveform()`, a pure function that is still real logic. It loads this
file by path and is still collected. If you move or delete `rig_dev.py`,
fix that test.

### Dead protocol planes

- `dev_exercise.py` — self-declared STALE in its own header. Drives the
  `DEV M …` / `DEV DT …` / `DEV WD` / `DEV STATE` family
  (`docs/protocol-v2.md` §16), which has had **no firmware handler since the
  102-107 single-loop rebuild** — it predates protocol v4, let alone v5.
- `velocity_chart.py` — an interactive dashboard rewired onto that same
  dead `DEV` plane.
- `ratio_governor_curve.py`, `pid_hold_speed.py` — ticket-077-era rig
  acceptance tests, also on `DEV DT`.
- `comms_plane_verify.py` — sprint 093, exercises
  `Communicator -> CommandRouter -> CommandProcessor`, all deleted classes.

### Retired by a later design

- `square_tour_velocity.py` — self-declared RETIRED (130-005). Exercised
  `Motion::WheelTrim`, the planner-side velocity trim that 130-005 deleted.
  Its *data* stayed in `bench/` because live docs cite it as evidence
  (`docs/design/square-tour-chart-spec.md`,
  `docs/code_review/2026-07-30-craftsmanship-review/02-motion.md`).

### Captured run artifacts — all in `output/`

CSV and PNG live in `src/tests/dev/output/`, never beside the program files.
Same rule going forward: if you park something here, the code goes at the
root and anything it produced goes in `output/`.

Superseded speed-map iterations (`speed_map_cal/combined/recal1/recal2/sym/
sym2`, `tovez_map1/2`, `tovez_verify`) and duplicate tour renders
(`square_tour_bench2/3`, `square_tour_goto_sim`, `square_tour_hardware`,
`square_tour_playfield*`, `square_tour_sim`, `rect_fast`, `square_repeat`,
`motionlib_square_tour_sim`, `wheels_square_tour_best`, and the
`square_tour_velocity_*` renders).

**What stayed in `bench/`**: any artifact a live doc, an open issue, or
`data/robots/tovez.json` cites as evidence — e.g. `bias_convergence_150.png`
(cited by tovez.json's own controller note), `duty_sweep.*`,
`speed_map.csv`/`speed_map2.csv`, `speed_map_sym*`, `crawl_sweep.*`,
`speed_sweep.*`, `wheel_controller_ab_bench.*`, `curve_stream_hardware.png`,
`square_tour_bench{,4,5,6}.png`. Measurement evidence that something still
reasons about is not scratch.

## Not moved here, on purpose

`velocity_step_response.py` stayed in `bench/`. It is broken — one dead
`proto.twist()` call — but it is the step-response characterization tool the
sim-fidelity work explicitly wants revived
(`clasi/issues/B-sim-plant-is-idealized-and-biases-belief-not-motion.md`,
post-130 review Part 4). It is one line from working and expensive to lose.
Fix it rather than filing it here.

## Related

- `src/tests/CLAUDE.md` — the three test domains (`sim`/`bench`/`playfield`)
  and why they are never combined.
- `clasi/issues/later/system-test-tests-outside-the-system-test-taxonomy-and-tiers.md`
  — the standing-vs-development policy this directory implements, including
  the `src/tools/` relocation still pending for characterization tools.
