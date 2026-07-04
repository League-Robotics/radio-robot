# tests/ — three test domains (sprint 077 greenfield rebuild)

This tree was rebuilt from scratch alongside `source/` (sprint 077's
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
  of history, not a real domain boundary). **This ticket (077-006) creates
  only the skeleton** — `unit/`, `system/`, and a `conftest.py` — because a
  fresh simulator harness for the new `source/` tree does not exist yet
  (later-ticket work). `uv run python -m pytest` currently collects this
  domain (see `pyproject.toml`'s `testpaths`) and finds nothing to run until
  that harness lands.
- **`bench/`** — runs against a real robot on the bench/stand, wired over USB
  or the radio relay. HITL (human-in-the-loop): a person is present to
  hand-load wheels, watch for runaways, and read dashboards. These are
  **Python CLI tools, not pytest tests** — nothing here is pytest-collected.
  Drives the robot via the `DEV` command family (`docs/protocol-v2.md` §16).
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

## How to run

```
uv run python -m pytest
```

Collects `tests/sim/` only (`pyproject.toml`'s `testpaths`) — the always-run,
no-hardware gate. `tests/bench/` and `tests/playfield/` are HITL CLI tools
invoked directly, e.g.:

```
uv run python tests/bench/dev_exercise.py --port /dev/cu.usbmodem2121102
```

`tests_old/` and `source_old/` are excluded from collection
(`norecursedirs`) and must never be touched by anything under `tests/`.

## Conventions

See `.claude/rules/` for project-wide coding conventions that apply here too
— in particular `coding-standards.md` (units in `# [unit]` trailing comments,
never in identifier names) and `naming-and-style.md`. Bench/playfield scripts
must be resilient to the `DEV` serial-silence watchdog (default 1000 ms,
`docs/protocol-v2.md` §16): widen it (`DEV WD 3000`) at session start, and
always restore it (`DEV WD 1000`) plus send `DEV STOP` in a `finally` block —
motors must never be left running on an exception or Ctrl-C.
