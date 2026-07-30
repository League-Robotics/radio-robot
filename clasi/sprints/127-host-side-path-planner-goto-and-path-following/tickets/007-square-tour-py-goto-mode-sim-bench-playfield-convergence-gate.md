---
id: '007'
title: 'square_tour.py goto-mode: sim/bench/playfield convergence gate'
status: open
use-cases:
- SUC-007
depends-on:
- '002'
- '006'
github-issue: ''
issue: sprint-127-host-side-path-planner-goto-path-following.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# square_tour.py goto-mode: sim/bench/playfield convergence gate

## Description

Design issue T6+T7, **re-expressed per sprint.md Architecture Decision
2**: the issue's own text proposes new files under `src/tests/sim/
system/`; that directory is scheduled for deletion by a separate, later
sprint (`square-tour-is-the-one-system-test-sim-bench-playfield.md`),
which also states plainly that new coverage belongs in a unit test or an
expansion of the square tour, not a new system test. This ticket adds a
`goto`-driven mode to the existing `src/tests/bench/square_tour.py`
instead — same 8-segment square, driven through `gotoWorld()` (ticket
006) rather than open-loop `WHEELS` commands, reusing the script's
existing `_Backend` sim/hardware split. **No new file under
`src/tests/sim/system/` may be created by this ticket.**

Depends on ticket 002 (this loop replaces the in-flight move
continuously, at rate — it must not run against unverified dedup) and
ticket 006 (the loop itself).

**Decision 4's feedback loop — this ticket's own defining requirement,
not an optional stretch goal**: ticket 006 shipped a *provisional*
termination-tolerance/give-up constant, isolated at one commented
definition site and marked `# PROVISIONAL -- pending ticket 007`. This
ticket:

1. Measures the real actuation floor — **minimum reliable move distance
   and minimum reliable turn angle** — from playfield-tier camera-verified
   data (below what distance/angle does the robot's own deadband/
   actuation lag make a commanded move unreliable, i.e. it doesn't
   reliably move at all or lands with error comparable to the move
   itself).
2. **Updates ticket 006's provisional constant** at its single definition
   site with the measured value, removing the `# PROVISIONAL` marker (or
   replacing it with a comment citing this ticket and the measurement).
3. Records the measurement itself — raw data, not just the final number —
   in this ticket's Completion Notes.

A ticket that only reports the number without writing it back into
ticket 006's constant does **not** satisfy this ticket's acceptance.

**Three tiers**:

1. **Sim** (`square_tour.py --sim --mode goto`): convergence to a stated
   closure tolerance, bounded overshoot, no stall, through the real
   firmware loop compiled into `libfirmware_host.dylib`
   (`just build-sim`), ground truth `SimLoop.get_true_pose()`. Also
   exercise `SimLoop.set_otos_drift()`/`set_enc_slip()` injection and
   confirm sane (non-catastrophic) behavior under both.
2. **Bench** (stand, wheels free): same square, `--mode goto`, encoder-
   judged closure. Robot currently reachable only via the RADIOBRIDGE
   relay (`/dev/cu.usbmodem2121302`, dongle `zavaz`) — confirm current
   ROLE with `mbdeploy list` before running, do not assume this port is
   still current.
3. **Playfield** (camera-supervised): `--mode goto` against camera-
   verified world targets, per-segment-boundary camera fixes, PNG chart,
   the minimum-reliable-distance/angle measurement above, and the
   encoder-vs-OTOS divergence rate (from ticket 004's `WorldPose`) — this
   is what feeds a re-anchor-cadence recommendation.

**Safety obligations** (`.claude/rules/playfield-testing.md`, mandatory
for the bench and playfield legs of this ticket):

- **Lights**: confirm `GET http://192.168.1.122/rpc/Switch.GetStatus?id=0`
  reports `"output": true` *before* arming any playfield run — do not
  infer from a prior session. If off,
  `GET http://192.168.1.122/rpc/Switch.Set?id=0&on=true`, then re-confirm
  with `GetStatus` (the `Set` reply's own `was_on` is not fully
  trustworthy per the standing rule).
- **Geofence checked INSIDE the ~10 Hz time-advance primitive**, never
  between segments — reuse the promoted `field.Geofence`/
  `drainFrames(proto, seconds, geofence)` idiom, do not poll only at
  segment boundaries.
- **`estop()`, never `stop()`, in a `finally` block on every connection
  path** — `stop()` is a planned stop that waits behind whatever move is
  already queued (measured elsewhere: 39.8 cm of travel on a 40 cm leg
  before a mid-leg `stop()` took effect); `estop()` clears Drive targets
  and the planner queue in the same cycle.
- **Camera fix at every segment boundary, at REST** (after the settle
  dwell, so there is no velocity lag) — not just at start and end. An
  8-leg square with turns yields many boundary fixes; log every one.

**Files**:
- Modify: `src/tests/bench/square_tour.py` — add a `--mode goto` (or
  equivalent) that drives the same 8 segments through `gotoWorld()`
  instead of `WHEELS`; reuse the existing `_Backend`, `Geofence` (now
  imported from `field/`, ticket 003), and camera-fix helpers unchanged.
- Modify: `src/host/robot_radio/pathplan/planner.py` (ticket 006's file)
  — update the provisional termination constant per the feedback-loop
  requirement above.
- New: PNG chart output (per-run, following the existing bench-script
  chart convention `square_tour.py` already has for its wheel-command
  mode).

**Coding standards**: lowerCamelCase functions/variables, UpperCamelCase
types; no units in identifiers in any new code added to `square_tour.py`
or `planner.py`.

## Acceptance Criteria

- [ ] Sim: `square_tour.py --sim --mode goto` closes the square within a
      stated tolerance (specific number, not "close enough"), bounded
      overshoot, no stall.
- [ ] Sim: behavior under `set_otos_drift()` and under `set_enc_slip()`
      (run separately) stays sane — no runaway, no unhandled exception,
      explicit pass/fail against a stated bound for each.
- [ ] Bench: `square_tour.py --port /dev/cu.usbmodem2121302 --mode goto`
      (or the port `mbdeploy list` currently reports) completes with
      encoder-judged closure reported numerically.
- [ ] Playfield: camera-verified closure reported numerically; per-
      segment-boundary camera fixes logged for every boundary, not just
      start/end; PNG chart produced and attached/referenced in Completion
      Notes.
- [ ] Minimum reliable move distance and minimum reliable turn angle are
      each reported as a specific measured number, with the raw data that
      established them in Completion Notes.
- [ ] Ticket 006's provisional termination-tolerance/give-up constant is
      updated in place with the measured value from the criterion above,
      and its `# PROVISIONAL` marker is removed or replaced with a
      citation to this ticket.
- [ ] Encoder-vs-OTOS divergence rate is reported from a playfield run,
      with an explicit re-anchor-cadence recommendation.
- [ ] `estimator.weight_heading_otos` / `weight_omega_otos` confirmed
      still `0.0` and `geometry.otos_untrusted` confirmed untouched
      (grep-checked, not assumed) — sprint success criterion 5.
- [ ] No new file under `src/tests/sim/system/`.
- [ ] Lights confirmed via `Switch.GetStatus` before every playfield run;
      `estop()` (never `stop()`) confirmed in a `finally` on every path
      exercised.

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/sim -q`;
  `square_tour.py`'s existing (non-`goto`) mode still passes unmodified,
  both `--sim` and on hardware.
- **New tests to write**: none new at the pytest level beyond what
  tickets 001/002/004/005/006 already added — this ticket is primarily a
  bench-script expansion plus HITL runs, per the standing
  no-new-system-tests directive.
- **Verification command**:
  `just build-sim && uv run python src/tests/bench/square_tour.py --sim --mode goto`,
  then (confirm ROLE with `mbdeploy list` first)
  `uv run python src/tests/bench/square_tour.py --port /dev/cu.usbmodem2121302 --mode goto`,
  then the same command against the playfield with camera bring-up
  completed first.
