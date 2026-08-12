# Baseline (read this at the start of every sprint)

**Sprint 136 ticket 002** (2026-08-11) established the current trustworthy
baseline, per `clasi/issues/later/A-seven-untriaged-failing-tests-poison-
every-no-regressions-claim.md`'s own proposed fix item 3. Measured with
`uv run python -m pytest src/tests/unit src/tests/sim src/tests/testgui`,
run twice (once before the `xfail` marks below landed, once after, to
confirm they match real failures rather than papering over something that
actually passes):

| tier | passed | failed | xfailed | xpassed | skipped |
|---|---|---|---|---|---|
| `src/tests/unit` | 975 | **0** | 0 | 0 | 0 |
| `src/tests/sim` | 493 | **0** | 4 | 0 | 0 |
| `src/tests/testgui` | 592 | **0** | 19 | 0 | 3 |

**Zero non-`xfail` failures.** Every `xfail` in all three suites is now
`strict=True` (a future accidental fix shows up as a loud XPASS failure,
not a silent no-op) and every reason string names a tracked issue. For the
exact commit this table was measured at, check this file's own `git log`/
`git blame` on this section — pinning a literal hash here would rot the
moment this file is next edited for an unrelated reason.

Measured on clean HEAD before this ticket (`5cf125f0`, sprint-planning
time): `src/tests/unit` 14 failed / 961 passed; `src/tests/sim` 39 failed /
455 passed / 2 xfailed / 1 xpassed; `src/tests/testgui` 10 failed / 590
passed / 3 skipped / 11 xfailed. Ticket 001 (commit `fc2869a2`) root-caused
and fixed the two dominant clusters (a missing `wheel_control.stall_*` key
group cascading through `gen_boot_config.py`'s required-key validation, and
nine tests pinned to literals `tovez.json`'s legitimate re-measurement
history had superseded), taking `unit`+`sim` to 2 remaining failures.
Ticket 002 (this record) triaged everything left: two `sim`-tier harness
scenario failures, and testgui's ten -- two fixed at the root (a stale
"contains '3'" camera-fallback heuristic test, superseded by the current
"arducam" substring convention and never back-filled), eight formally
accepted with strict, issue-referenced `xfail` marks. Every one of the
pre-existing non-strict `xfail`s already in the tree (including the sim
tier's one `xpass`, `test_angle_stop_lands_close_to_target_with_tovez_
nocal_calibration` -- confirmed genuinely passing now, 5/5 repeated runs,
so the `xfail` was removed rather than made strict) was resolved the same
way: fixed, or made strict with an accurate reason.

**Full per-failure triage record**: this section, ticket 002's own commit
message, and the tracked issues below carry the reasoning; ticket
Completion Notes in `clasi/sprints/136-.../tickets/002-....md` could not be
written directly due to a known CLASI tooling defect (`clasi/issues/
clasi-role-guard-sees-every-subagent-as-team-lead.md`) -- the commit
message is the authoritative record for this ticket.

Tracked issues backing every current `xfail` (check each for status before
assuming any of these are still open):

- `clasi/issues/later/A-tour2-146-degree-turn-still-undershoots-after-130-010.md`
  -- turn-accuracy/tour-completion family (`test_tour_closure_gate.py`'s
  shaped-band gate, `test_gui_button_acceptance.py`'s two tour-completion
  tests + the downstream stop-mid-tour precondition failure). Real,
  previously-flagged, unresolved firmware behavior -- out of scope for
  sprint 136 per its own `sprint.md`.
- `clasi/issues/later/test-canvas-playfield-tour-fixtures-deleted-by-e4bffd8e.md`
  -- `test_canvas.py`'s three tests: their fixture files were deleted by an
  unrelated commit (`e4bffd8e`, ~200K lines, no archive path named in its
  own commit message) -- needs a stakeholder call (restore vs. rewrite the
  tests), not a triage fix.
- `clasi/issues/later/otos-live-config-push-does-not-converge-in-sim.md` --
  `test_otos_calibration_convergence.py`: a pushed, acked OTOS
  compensating-scale config does not measurably converge the decoded
  reading back to truth in sim. Leading (unconfirmed) hypothesis: a
  sim-fidelity gap in `OtosPlant`'s burst-read packing, not a firmware
  defect.
- `clasi/issues/later/app-preamble-and-devices-otos-harness-scenario-failures.md`
  -- `src/tests/sim`'s two harness tests (`test_app_preamble.py`,
  `test_devices_otos.py`): both compile clean; both fail at RUNTIME
  (compiled-binary scenario assertions), long-documented pre-existing,
  unrelated to sprint 135/136.
- Everything else already carried a tracked issue before this ticket
  (`B-rotation-calibration-vs-live-heading-hold-gain.md`,
  `chain-advance-reset-defeats-same-axis-compatible-leg-continuity.md`,
  and the two permanently-disabled-`goto_btn`/`origin_btn`/fused-pose
  `xfail`s dating to sprint 097/098's binary-plane gating) -- only made
  strict here, reasons otherwise unchanged except where a cross-referenced
  issue had itself moved (`land-at-zero-at-orthogonal-chain-boundaries.md`
  closed at sprint 121; `test_tour_closure_gate.py`'s two reason strings
  that cited it now point at the still-live tracking issue above instead).

# tests/ — three test domains (sprint 077 greenfield rebuild)

This tree was rebuilt from scratch alongside `src/firm/` (sprint 077's
greenfield rebuild: `source_old`/`tests_old` are the parked pre-rebuild
trees — see `tests_old/CLAUDE.md` and the sprint issue for history). It is
**not** a copy of `tests_old/`'s tiering; the domains below are a different
split, organized around *where the test runs*, not around CI-gate tiering.

## The three domains — never combined

`sim/`, `bench/`, and `playfield/` are three independent test regimes that
run on three different machines and are **never combined** into one run or
one CI gate:

- **`sim/`** — runs on a developer laptop against a firmware simulator, no
  hardware involved. Merges the old `tests_old/sim/` + `tests_old/simulation/`
  pair (both wrapped the same simulated firmware; the split was an artifact
  of history, not a real domain boundary). 077-006 created only the skeleton
  (`unit/`, `system/`, `conftest.py`); sprint 105 built the real harness on
  top of it:
  - `unit/` — per-module `Core::`/`Devices::` host-build harnesses (one
    `*_harness.cpp` + `test_*.py` pair per module, e.g.
    `test_app_robot_loop.py`, `test_devices_motor.py`) — each compiles its
    own throwaway binary via `subprocess`, no shared build step.
  - `plant/` — `TestSim::WheelPlant`/`OtosPlant` (105-003, SUC-020): a
    deterministic, seeded duty->velocity->position first-order model
    standing in for one physical wheel + Nezha channel, plus three
    fault-injection knobs (motor disconnect, encoder wedge, encoder
    dropout — 105-005, SUC-022).
  - `support/` — `TestSim::SimApi` (`sim_api.h`, 105-004, SUC-021): wires
    the REAL `Core::RobotLoop` against the REAL plant and a scripted
    `Devices::I2CBus` into one reusable, steppable harness other test
    binaries link against, plus `TestSupport::FakeTransport` (the
    `Core::Transport` HOST_BUILD double, 105-002) and a hand-written wire
    codec (`wire_test_codec.h`) for decoding outbound telemetry / encoding
    inbound commands in tests.
  - `system/` — whole-robot scenario tests built on `SimApi`: the
    acceptance harness for `SimApi` itself, the three fault-injection
    scenarios, and the scripted-twist demo (105-006, SUC-023) — this
    sprint's own Definition of Done, a readable narrated end-to-end story
    (boot -> twist -> real plant ramp -> stop -> velocity heads back
    toward zero) runnable both under pytest and standalone. See
    `src/tests/sim/system/README.md` for the full layout and how to add a new
    scenario.
  - `conftest.py` — no fixtures (105-006 removed the stale `build_lib`/
    `sim` fixtures that referenced deleted `tests/_infra/sim` — see the
    file's own header for the ticket-time call). Every harness compiles
    its own binary ad hoc; there is no shared Python-level fixture.

  `uv run python -m pytest` collects this domain (see `pyproject.toml`'s
  `testpaths`) and, as of sprint 105, actually runs a real simulator
  against the current `src/firm/` tree end to end.
- **`bench/`** — runs against a real robot on the bench/stand, wired over USB
  or the radio relay. HITL (human-in-the-loop): a person is present to
  hand-load wheels, watch for runaways, and read dashboards. These are
  **Python CLI tools, not pytest tests** — nothing here is pytest-collected.
  Drives the robot over the **protocol-v5 binary plane** via `NezhaProtocol`
  (`docs/protocol-v5.md`) — bounded `Move`s with ack-ring completion. (This
  line used to name the `DEV` command family from `docs/protocol-v2.md` §16;
  that plane has had no firmware handler since the 102-107 rebuild, and every
  script that still drove it moved to `dev/` on 2026-08-03.)
  **`bench/` holds things we use persistently** — current tools and standing
  gates. Scratch does not belong here; it belongs in `dev/`.
- **`playfield/`** — runs against a real robot driving on the camera-covered
  playfield (never call it "the floor" — see
  `.clasi/knowledge/playfield-not-floor.md`; driving off the table is a
  failure, not a synonym). Also HITL Python tools, also not pytest-collected.
  Currently **parked**: both scripts here need motion/odometry that only
  existed in the pre-rebuild firmware (`source_old`) — see each file's header
  note for what has to come back before they run again.

Why never combined: a sim run proves the *control logic* is correct against
an idealized plant; a bench run proves the *real motor/encoder/PID* behaves
under load with no camera or floor risk; a playfield run proves the *whole
robot* holds a world-frame task in the one environment that can actually
fail badly (driving off the table). Collapsing them into one suite would
either force sim-only assertions onto hardware (flaky, unsafe) or force
HITL/camera setup onto the CI sim gate (impossible to run unattended).

## Kept categories

- **`unit/`** — host-side unit tests that aren't scenario/domain-specific
  (e.g. `robot_radio` protocol parsing). Skeleton only this ticket.
- **`tools/`** — test tooling/helpers shared across domains. Skeleton only
  this ticket.
- **`dev/`** — development scratch (added 2026-08-03). Scripts, notebooks,
  and captured artifacts written to answer one question during one
  investigation and since retired: the quarantined coupled-rig family, the
  dead `DEV`/protocol-v2 plane tools, and superseded run outputs. **Nothing
  in `dev/` is collected, gates anything, or carries a maintenance
  obligation**; a broken file there blocks nothing and the sweep may prune
  it. See `src/tests/dev/CLAUDE.md` for what is there and why each piece
  stopped working. This is the "development stock" half of the
  standing-vs-development policy — the standing lists live in
  `clasi/issues/later/system-test-tests-outside-the-system-test-taxonomy-and-tiers.md`.

## How to run

```
uv run python -m pytest
```

Collects `src/tests/sim/` only (`pyproject.toml`'s `testpaths`) — the always-run,
no-hardware gate. `src/tests/bench/` and `src/tests/playfield/` are HITL CLI tools
invoked directly, e.g.:

```
uv run python src/tests/bench/radio_bench_gate.py --port /dev/cu.usbmodemRELAY123
uv run python src/tests/bench/twist_drive.py --port /dev/cu.usbmodem2121202
```

Take the port from `mbdeploy list` **for that session only** — port numbers
move on every re-enumeration, and more than one robot shares the hub. Address
the robot by UID wherever a tool accepts a target
(`.claude/rules/hardware-bench-testing.md`).

`tests_old/` and `source_old/` are excluded from collection
(`norecursedirs`) and must never be touched by anything under `tests/`.

## Conventions

See `.claude/rules/` for project-wide coding conventions that apply here too
— in particular `coding-standards.md` (units in `# [unit]` trailing comments,
never in identifier names) and `naming-and-style.md`.

**Stale — do not follow, kept for git-blame context only**: this section
used to say bench/playfield scripts must widen the `DEV` serial-silence
watchdog (`DEV WD 3000`) at session start and restore it (`DEV WD 1000`)
in a `finally` block. The entire `DEV`/`SET`/`GET` command family
(`docs/protocol-v2.md` §16) has had no firmware handler since the sprint
102-107 single-loop rebuild — `dev_exercise.py` (which this guidance was
written for) is itself flagged stale for the same reason, see that
file's own header. **Current safety contract (protocol v5,
`docs/protocol-v5.md` §5, "no deadman")**: there is no session-wide
serial-silence watchdog to widen/restore at all — every `Move` carries
its own bounded `timeout` safety backstop by construction. What a bench/
playfield script must still do: call `estop()` (the binary `ESTOP` verb —
via the shared `robot_radio.robot.halt.halt_now()` helper, 128-001) in
a `finally` block on every connection it opens, so motors are never left
running on an exception or Ctrl-C — see `src/tests/bench/radio_bench_gate.py`
for the current reference implementation of that pattern. `stop()` is a
PLANNED stop that queues behind whatever is already in flight (measured:
39.8cm/5.9s to take effect mid-leg, vs. 2.9cm/0.10s for `estop()`) — never
use it in a Ctrl-C/exception cleanup path.
