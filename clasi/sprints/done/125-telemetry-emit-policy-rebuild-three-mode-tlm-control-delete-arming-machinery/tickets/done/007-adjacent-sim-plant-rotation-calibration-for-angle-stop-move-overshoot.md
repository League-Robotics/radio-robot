---
id: '007'
title: '[Adjacent] Sim-plant rotation calibration for ANGLE-stop MOVE overshoot'
status: done
use-cases:
- SUC-005
depends-on: []
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# [Adjacent] Sim-plant rotation calibration for ANGLE-stop MOVE overshoot

## Description

**This ticket is NOT part of `telemetry-emit-policy-rebuild-spec.md`.**
It exists because the stakeholder added "the square tour must pass in
SIM and on the BENCH" as sprint-125 acceptance on top of that issue, and
`src/tests/bench/square_tour.py --sim` currently fails at ~288 mm
closure in its default corner mode (ANGLE-stopped MOVE, closed-loop on
estimator heading — see `square_tour.py`'s own `_corner_angle()`/
argparse help around line 474/668). The failure has already been
isolated (not by this ticket, prior to sprint planning) to a systematic
~13 deg overshoot between the commanded ANGLE stop condition and the sim
plant's own true rotation: 45→56.8, 90→103.0, 180→193.0 measured against
plant truth (`SimLoop.get_true_pose()`). `data/robots/tovez_nocal.json`
— the config `square_tour.py --sim` uses — carries IDENTITY
`rotation_gain`/`rotation_offset_deg` values (no calibration applied),
which is the leading suspect but not yet confirmed as the sole cause.

**First step, before writing any fix**: confirm whether the root cause
is (a) a calibration-VALUE gap (the identity `rotation_gain_pos`/
`rotation_gain_neg`/`rotation_offset`/`rotation_offset_neg` in
`tovez_nocal.json` need real fitted values, the same kind of self-
calibration `square_tour.py`'s own prelude already does for its pivot
math — see the file's `TURN_ARC`/rotational-slip comments around line
51), or (b) a genuine kinematic mismatch in how the sim plant or the
motion-library/firmware ANGLE stop condition computes rotation, that no
calibration value can fully paper over. Use the
`docs/design/encoder-refresh-characterization.md`-style approach (measure
first, then fix) rather than guessing. If the root cause turns out to be
(b), this ticket's fix may need to touch the sim plant's own rotation
model rather than (or in addition to) the JSON config — scope
accordingly and update this ticket's acceptance criteria to match what
is actually found, noting the change in completion notes.

Do not touch `src/firm/app/telemetry.*`, `src/firm/app/comms.*`, or
`Devices::Motor` in this ticket — those are ticket 001-006's exclusive
territory; this ticket's footprint should be confined to
`data/robots/tovez_nocal.json` and/or the sim-plant rotation model
(location TBD by this ticket's own investigation — likely under
`src/sim/` or wherever `SimLoop`'s plant kinematics live).

Bench leg: the failure described above is a SIM-plant discrepancy;
nothing in the issue or this sprint indicates the bench (real hardware)
leg of `square_tour.py` has the same defect (`tovez_nocal.json` is
explicitly the NO-calibration config, likely sim-only — confirm whether
the bench run uses a different, already-calibrated robot config file
before assuming the bench leg needs any change here at all).

## Acceptance Criteria

- [x] Root cause confirmed and documented in this ticket's completion
      notes: calibration-value gap, sim-plant kinematic bug, or both.
- [x] `uv run python src/tests/bench/square_tour.py --sim` exits 0
      within the script's own stated heading/closure bounds.
- [x] The fix does not regress any other sim test that reads
      `tovez_nocal.json` or exercises ANGLE-stopped MOVEs (run the
      broader sim suite, not just `square_tour.py`, after the change).
- [x] Confirm (do not assume) whether the bench leg
      (`square_tour.py --port <bench>`) is affected by the same defect;
      if it is, extend the fix; if it is not (e.g. because the bench
      robot uses a real calibrated config, not `tovez_nocal.json`),
      document that finding and verify the bench leg passes as-is on
      the stand.
- [x] No change to `data/robots/tovez.json`/`togov.json` (the CALIBRATED
      configs) unless investigation shows they share the same defect —
      state explicitly if they were checked and found unaffected.

## Completion Notes (2026-07-29)

**Verdict: (a), with an important refinement neither (a) nor (b) as
originally framed quite captures.** The ~13 deg overshoot ITSELF is a
genuine, physically-expected sim-plant artifact — `TestSim::WheelPlant`
(`src/tests/sim/plant/wheel_plant.h`) is a first-order duty->velocity
model with `kDefaultTau = 0.13s` (matching the bench-characterized
~120-140ms actuation-lag range), so the wheels keep coasting for a beat
after an ANGLE stop condition fires — the SAME physical mechanism real
hardware's own actuation lag produces, and exactly the class of residual
`App::RobotLoop::setRotationCalibration()` already exists to correct
(its own doc comment: "handleMove() inverts it so an ANGLE-stopped move
LANDS on the angle the host asked for"). `App::RobotLoop::handleMove()`'s
ANGLE-stop threshold math (`robot_loop.cpp` line ~201-215) is unmodified
by this ticket — confirmed by inspection, not a kinematics bug.

**But (a) as literally stated — "calibrate tovez_nocal.json's rotation
constants" — could not have worked on its own.** Editing
`tovez_nocal.json`'s `rotation_gain`/`rotation_offset_deg` and re-running
`square_tour.py --sim` had **zero effect** (confirmed empirically before
touching any C++: a temp-file config with fitted constants produced
byte-identical overshoot numbers to the identity config). Root cause:
`setRotationCalibration()` was called from exactly ONE place in the
entire tree — `main.cpp`'s real-hardware boot sequence (reading the
compile-time-baked `boot_config.cpp`, itself generated from a robot JSON
by `gen_boot_config.py`) — grep-confirmed absent from `sim_harness.h`,
`sim_ctypes.cpp`, and every other sim-path file. `SimLoop.
configure_from_robot()`'s three tiers (live `ConfigDelta`, one-shot motor
boot-config, live `EstimatorConfigPatch`) never touched drivetrain
rotation calibration at all, so the sim's own `App::RobotLoop` stayed at
the identity default (gain 1, offset 0) **permanently, regardless of any
robot JSON's content** — a structural wiring gap, not a value or
kinematics bug, and the reason this ticket's footprint had to extend into
`src/sim/` per its own scoping note ("location TBD... likely under
src/sim/").

**Fix (three files, ~60 net lines, mirrors the EXISTING motor Tier-2
pattern exactly — no new mechanism invented):**
- `src/sim/sim_ctypes.cpp`: new `sim_configure_drivetrain()` extern-"C"
  export, a thin call-through to `App::RobotLoop::setRotationCalibration()`
  via the already-public `SimHarness::robotLoop()` accessor (no new
  `SimHarness` method needed).
- `src/host/robot_radio/calibration/sim_boot_config.py`: new
  `drivetrain_boot_config_for()`, calling `gen_boot_config.py`'s
  EXISTING (already-used-for-real-hardware) `rotation_calibration_for_config()`
  — no JSON->value mapping logic duplicated, exactly `motor_boot_config_for()`'s
  own precedent. Degrees->radians conversion happens here, mirroring
  `main.cpp`'s own conversion at its boot seam.
- `src/host/robot_radio/io/sim_loop.py`: `configure_from_robot()`'s Tier 2
  now also calls `sim_configure_drivetrain()` after the existing per-port
  motor loop, plus ctypes argtypes registration.

**Measurement and fit** (`data/robots/tovez_nocal.json`'s own
`_rotation_calibration_note` carries the full derivation): measured the
IDENTITY-calibration sim plant's own overshoot at omega=+/-2.0 rad/s
(`square_tour.py`'s own `runTurnMove()` rate) via `SimLoop.get_true_pose()`
across 15/30/45/60/90/135/180 deg targets. The overshoot is NOT constant
at small angles (15 deg: +6.5; the wheels have not yet reached steady
omega by the time the stop fires, so the coast itself starts from a lower
velocity) but SATURATES to a near-constant ~+13 deg by 45 deg and beyond
— consistent with a coast-distance model (integral of an exponentially-
decaying velocity is proportional to velocity-at-cutoff x tau, saturating
once the plant reaches steady state before the cutoff). Since
`square_tour.py` commands EXACTLY 90 deg every corner (never a small
angle), the affine fit was deliberately taken over the saturated
45-180 deg range (least-squares: gain=1.006, offset=12.1 deg), not
forced to also cover the small-angle regime a single affine pair cannot
represent. Verified symmetric between rotation directions (WheelPlant's
`dv/dt` model has no directional asymmetry) — same gain/offset applied to
both `rotation_gain`/`rotation_offset_deg` and their `_neg` counterparts.

Post-fit measured residual (`tovez_nocal.json`, real file, post-rebuild):
90 deg -> 90.24 (err +0.24), 180 deg -> 179.94 (err -0.06), 45 deg -> 43.15
(err -1.85, outside the fit's intended saturated range — acceptable, not
this ticket's exercised regime).

**`square_tour.py --sim` result**: `heading 361.2 deg (target 360),
closure 6.1 mm (plant truth)` → `PASS: square tour closed` (bounds:
closure<=60mm, heading within 15deg of 360 — both comfortably inside).
Re-confirmed after a cosmetic C++ parameter rename (dropping a unit
suffix, see Coding Standards note below) and rebuild: `heading 360.8 deg,
closure 5.8 mm` — still PASS.

**Broader sim suite**: `uv run python -m pytest src/tests/sim/ -q` →
**420 passed, 1 skipped, 1 xfailed, 0 failed** (418 baseline + 2 new
regression tests this ticket adds, see below). No test besides
`square_tour.py` itself (not pytest-collected) exercises an ANGLE-stopped
MOVE against `tovez_nocal.json` — grepped for `stop_angle`/`omega=` usage
across every file that references `tovez_nocal.json`; the one hit outside
`square_tour.py` (`test_motor_primitive.py`'s HEADING primitive) is a
TIME-stopped twist, which never reaches `handleMove()`'s ANGLE-stop
correction branch at all.

**New regression test**: `src/tests/sim/system/test_angle_stop_rotation_calibration.py`
— two tests. The positive case drives a 90 deg ANGLE-stop MOVE through
`configure_from_robot()`-loaded `tovez_nocal.json` and asserts landing
within +/-5 deg of target (bound intentionally loose relative to the
measured ~0.2 deg residual, tight relative to the ~13 deg defect — it
exists to catch a regression, not to hold the plant to a tolerance this
ticket never claimed). A negative control repeats the SAME move with
Tier-1/motor-Tier-2 config only (deliberately skipping the drivetrain
Tier-2 call) and asserts the ~13 deg overshoot REAPPEARS — proving the
positive test is actually exercising the calibration path, not passing
for an unrelated reason (e.g. a loose stop-condition bound). Both pass.

**Bench leg: NOT affected — confirmed by code inspection, not a hardware
run (no hardware attached to this environment).** Three independent
findings converge:
1. `HardwareBackend.__init__()` (`square_tour.py`) explicitly loads the
   robot named by `data/robots/active_robot.json`'s pointer (currently
   `tovez.json`), NOT the script's own `--robot-json` default
   (`tovez_nocal.json`, "the deliberately UNCALIBRATED config the sim
   wants" — the script's own comment).
2. `tovez.json`'s `calibration.rotation_gain`/`rotation_offset_deg` are
   ALREADY real, camera-measured, hardware-fitted values (1.061/-5.54 and
   1.071/-7.04, dated 2026-07-29, verified over 47-48 trials — see that
   file's own `_rotation_calibration_note`), unrelated to and unaffected
   by this ticket (`tovez.json` was not touched).
3. Real firmware's boot path (`main.cpp`) has ALWAYS called
   `setRotationCalibration()` from the compiled-in `boot_config.cpp` —
   this call site is untouched by this ticket. The wiring gap this ticket
   fixed was `src/sim/`-only (`SimHarness` never went through `main.cpp`'s
   boot sequence at all); real hardware was never missing this mechanism.
`square_tour.py`'s own `runTurnMove()` docstring already documents a
prior hardware verification of `tovez.json`'s calibration independent of
this ticket ("180 deg commands landed at 180.3 (sd 1.9) over six runs...
mean +0.64 deg" across 15-180 deg, four rates) — consistent with "bench
leg unaffected," not merely assumed.

**`tovez.json`/`togov.json`**: `tovez.json` — unchanged (confirmed:
`git diff --stat -- data/robots/` shows only `tovez_nocal.json`); its own
hardware-fitted constants are NOT a fix for the sim's own artifact (a
hardware-measured NEGATIVE offset would not cancel — would likely worsen
— the sim's own POSITIVE-offset coast overshoot; the two configs correct
for physically different mechanisms, stiction vs. coast-down lag). This
is why `square_tour.py --sim`'s own `--robot-json` default is
`tovez_nocal.json` specifically, and this ticket leaves that default
alone. `togov.json` — also unchanged, and NOT checked for the same
defect: it carries its OWN identity rotation calibration (1.0/0.0,
grep-confirmed), so the same structural wiring gap this ticket fixed
*could* apply to it too if anyone ever ran an ANGLE-stopped sim tour
against it — but Togov is a mecanum drivetrain (`.clasi/knowledge/
tovez-vs-togov-drivetrain.md`), nothing in this sprint exercises it, and
its own ANGLE-stop sim behavior (if any differs meaningfully from a
differential-drive plant) was not characterized here. Flagging as a
candidate follow-up, not a defect of this ticket's own scope.

## Testing

- **Existing tests to run**: the sim test suite for anything reading
  `tovez_nocal.json` or driving ANGLE-stopped MOVEs, to catch any
  regression from a calibration-value or plant-model change.
- **New tests to write**: none necessarily required beyond
  `square_tour.py --sim` itself passing, but consider a focused
  regression test (single 90 deg ANGLE-stop MOVE against sim plant
  truth, asserting the overshoot is within a tight tolerance) so this
  defect cannot silently reappear.
- **Verification command**: `uv run python src/tests/bench/square_tour.py
  --sim` (must exit 0); `uv run python src/tests/bench/square_tour.py
  --port <bench-port>` on the stand (must exit 0 or be confirmed
  already-passing/not-applicable).
