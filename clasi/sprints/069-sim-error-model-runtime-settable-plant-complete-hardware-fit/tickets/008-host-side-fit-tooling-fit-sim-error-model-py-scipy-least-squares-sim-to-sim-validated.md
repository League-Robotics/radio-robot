---
id: '008'
title: 'Host-side fit tooling: fit_sim_error_model.py (scipy least_squares, sim-to-sim validated)'
status: open
use-cases:
- SUC-008
depends-on:
- '004'
- '006'
github-issue: ''
issue: sim-error-model-runtime-settable-hardware-fit.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host-side fit tooling: fit_sim_error_model.py (scipy least_squares, sim-to-sim validated)

## Description

Builds `host/robot_radio/calibration/fit_sim_error_model.py`, the sprint's
acceptance vehicle for "the simulator and the real robot must be tunable to
behave identically." The `host/robot_radio/calibration/` package already
exists (`helpers.py`, `linear.py`, `angular.py`, `push.py`,
`_conn_helpers.py`) — this ticket adds one new module to it, following the
package's existing convention of pure, hardware-decoupled math functions
(`helpers.py`'s docstring: "Canonical implementations; do not duplicate
these elsewhere").

**Inputs**: a recorded run (JSONL: `{t, cmd}` for issued commands, `{t,
encpose, otos, pose}` for parsed TLM frames — transport-agnostic; this
sprint's recorder helper produces it from a `Sim()` instance only) and a
candidate `SIMSET` parameter-name list (defaults to the full
deterministic/bias-shaped subset: scale errors, drift, body scrub,
trackwidth, actuation offset — NOT noise sigmas, which don't bias a mean
trajectory and would only add variance to the least-squares residual). The
three TLM poses (`pose=`, `otos=`, `encpose=`) are already parsed by
`host/robot_radio/robot/protocol.py`'s `TLMFrame`/`parse_tlm()`
(`protocol.py:43-80,219`) — reuse this, do not re-implement TLM parsing.

**Algorithm**: `scipy.optimize.least_squares` (bounded, Trust Region
Reflective — see `architecture-update.md` Design Rationale Decision 5 for
why `scipy` over a hand-rolled optimizer), minimizing summed squared
position + heading residual (heading wrapped to `(-π, π]` before
differencing — headings are circular quantities, an unwrapped difference
near the ±180° boundary would otherwise dominate the residual spuriously)
across all three TLM poses at every recorded timestamp, replaying the
recorded command sequence into a FRESH, zero-error sim instance for each
candidate parameter vector.

**Output**: a JSON parameter file (`SIMSET` key → fitted value) and a small
CLI to load it (`SIMSET`-batch over a live connection, using the same
`sim.command(...)`-style raw-wire call `transport.py` uses).

**Dependency**: `scipy` is a NEW host dependency (only `numpy` is present
today). Per the recorded stakeholder note on this sprint's gate: add it to
the CURRENT, root-level `pyproject.toml`'s `[dependency-groups]` — the
user recently folded `host/pyproject.toml` into the root; do NOT create or
edit a `host/pyproject.toml` (confirmed absent — `ls host/pyproject.toml`
returns nothing). The `calibrate` group already exists (`pyproject.toml`,
"Bench calibration / camera-ground-truth tooling (tests/calibrate/)", listing
`matplotlib>=3.8`) and is the natural fit for `scipy` too — add it there,
and confirm `calibrate` remains in `[tool.uv].default-groups` (it already
is, alongside `codegen`/`dev`) so `uv sync`/`uv run` pick it up without an
explicit `--group` flag.

**Explicitly out of scope**: the real-hardware Tour-1 record→fit→replay
demo (the issue's ultimate acceptance) — deferred to a follow-up HIL task
per `architecture-update.md` Open Question 1 (needs a physical robot, a
live-serial/relay wire-log recorder, and bench time, none of which fit this
autonomous sim-only sprint). This ticket validates SIM-TO-SIM only: inject
known `SIMSET` values into one sim instance, record, fit against a second
fresh sim, and confirm the fit recovers the injected values.

## Acceptance Criteria

- [ ] `host/robot_radio/calibration/fit_sim_error_model.py` (new): CLI
      script with (at minimum) a `record` mode (drive a `Sim()` instance
      through a maneuver, emit the JSONL recording) and a `fit` mode
      (consume a recording + a candidate parameter-name list, emit the
      fitted-parameter JSON file).
- [ ] Recording format: JSONL, transport-agnostic — `{"t": <ms>, "cmd":
      "<wire command text>"}` for each issued command, `{"t": <ms>,
      "encpose": [x,y,h], "otos": [x,y,h], "pose": [x,y,h]}` for each parsed
      TLM frame (reuse `TLMFrame`/`parse_tlm()` from `protocol.py`, do not
      hand-roll a second TLM parser).
- [ ] Fit uses `scipy.optimize.least_squares` with explicit bounds per
      parameter (not unbounded — the whole point of TRF here is respecting
      each `SIMSET` key's valid range, e.g. `(0, 1]` for scrub factors per
      ticket 002's `clampScrub()`), minimizing summed squared position +
      wrapped-heading residual across `pose=`/`otos=`/`encpose=` at every
      recorded timestamp.
- [ ] Default candidate parameter set is the deterministic/bias-shaped
      `SIMSET` subset: `bodyRotScrub`, `bodyLinScrub`, `trackwidthMm`,
      `motorOffsetL`, `motorOffsetR`, `encScaleErrL`, `encScaleErrR`,
      `otosLinScaleErr`, `otosAngScaleErr`, `otosLinDriftMmS`,
      `otosYawDriftDegS` — explicitly EXCLUDING noise-sigma keys
      (`encNoiseL`/`R`, `otosLinNoise`, `otosYawNoise`, `encSlipL`/`R` if
      treated as a pure-variance term rather than a bias term — confirm
      `encSlipL`/`R`'s bias-vs-variance character by inspection of
      `PhysicsWorld::update()`'s sub-step A' before deciding which bucket it
      belongs in).
- [ ] Emitted parameter file: JSON, `{"<SIMSET key>": <fitted value>, …}`.
- [ ] A CLI mode (or a small companion function) loads a parameter file into
      a live connection by sending one batched `SIMSET k1=v1 k2=v2 …`
      command (reuses the same wire mechanism ticket 007's TestGUI panel
      uses — a single `SIMSET` string, not per-key round trips).
- [ ] **Sim-to-sim validation test**: `SIMSET` known values into Sim A
      (e.g. `bodyRotScrub=0.90`, `encScaleErrL=0.03`), drive a maneuver,
      record; replay the same command sequence against fresh Sim B instances
      across the fit's search; recover each injected parameter within a
      stated tolerance (e.g. ±10% relative, or an absolute floor for
      near-zero true values — pick and document the exact tolerance in the
      test).
- [ ] **Replay-fidelity check**: load the fitted parameter file into a THIRD
      fresh sim instance (Sim C), replay the same command sequence, and
      confirm its recorded trajectory agrees with Sim A's original
      recording within a stated tolerance.
- [ ] `pyproject.toml`: `scipy` added to the `calibrate` dependency group
      (NOT a new group, NOT a stale `host/pyproject.toml` — there is none).
      `uv sync` succeeds and `import scipy.optimize` works from a fresh
      sync.
- [ ] Real-hardware Tour-1 record→fit→replay is explicitly NOT attempted by
      this ticket — note this in the module's docstring, pointing at the
      Open Questions in `architecture-update.md` for the follow-up HIL
      task's scope.
- [ ] Full default suite green: `uv run python -m pytest`.

## Testing

- **Existing tests to run**: full default suite (unaffected by an additive
  new host module + dependency).
- **New tests to write**:
  - `tests/host/calibration/test_fit_sim_error_model.py` (or
    `tests/calibrate/`, matching whichever location the existing
    `host/robot_radio/calibration/` package's tests already use — check
    `tests/calibrate/` first, per `architecture-update.md` Open Question 7's
    "prefer reusing an existing format/location over inventing a third"
    guidance): the sim-to-sim injection→record→fit→recovery test and the
    replay-fidelity check, both described above.
  - A unit test for the JSONL recording format itself (round-trips a
    synthetic `{t, cmd}`/`{t, encpose, otos, pose}` sequence).
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Build the recorder as a thin, sim-only helper inside this same
new module (a HIL recorder producing the identical JSONL shape from a live
serial/relay connection is explicitly a follow-up task's concern, not this
ticket's). Keep the fit function itself transport-agnostic — it only
consumes the recording format and a `Sim()`-constructing replay function, so
a future HIL recorder can feed it without any change to the fit logic
itself. Reuse `protocol.py`'s `TLMFrame`/`parse_tlm()` rather than
re-parsing TLM text.

**Files to create**:
- `host/robot_radio/calibration/fit_sim_error_model.py`.
- `tests/host/calibration/test_fit_sim_error_model.py` (or the matching
  existing test location for this package — confirm at implementation
  time).

**Files to modify**:
- `pyproject.toml` — add `scipy` to the `calibrate` dependency group.
- `uv.lock` — regenerated by `uv sync` after the `pyproject.toml` change.

**Testing plan**:
- Sim-to-sim injection→record→fit→recovery test with a stated, documented
  tolerance.
- Replay-fidelity test (fitted params loaded into a fresh sim reproduce the
  original recording's trajectory within tolerance).
- Confirm `uv sync` picks up `scipy` and the full suite runs green including
  the new tests.

**Documentation updates**: module-level docstring in
`fit_sim_error_model.py` documenting the recording format, the candidate
parameter set and why noise sigmas are excluded, and the explicit
sim-to-sim-only scope of this sprint's validation (pointing at
`architecture-update.md` Open Question 1 for the deferred real-hardware
follow-up).
