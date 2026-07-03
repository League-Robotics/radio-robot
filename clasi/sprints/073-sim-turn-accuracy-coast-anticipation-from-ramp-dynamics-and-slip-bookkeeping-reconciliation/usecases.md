---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 073 Use Cases

`clasi/issues/sim-turn-undershoot.md` measured three independent, composing
defects behind SIM turn inaccuracy (90°→~87°, 180°→~3° short, 300°→2.8–6°
short): a mistuned/stale coast-anticipation constant in
`Planner::beginRotation()` (firmware, shared ARM+sim), a plant that has never
modeled the body-truth chassis scrub the firmware's `rotationalSlip`
calibration compensates for (sim-only), and a `PhysicsWorld::setSlip()` side
effect that silently couples an encoder-report-error knob to the same
body-truth channel (sim-only). Two SUCs below narrow the existing **UC-020:
Configure Simulator Plant/Error Model at Runtime** (proposed by sprint 069,
`docs/architecture/done/architecture-update-069.md` — not yet consolidated
into `docs/usecases.md`); two propose a **new UC-023: Rotate In Place by a
Relative Angle (RT command)**, since the `RT` wire command (added ~sprint
024-006) is not documented anywhere in `docs/usecases.md`'s current UC-001
through UC-019. Flagged for the stakeholder/consolidation pass to confirm
both the UC-020 narrowing and the UC-023 mint (mirrors 068/069/072's
recurring "UC mint-vs-narrow" open item — see this sprint's
`architecture-update.md` Step 7). Note: several other sprints have
tentatively claimed UC-022/034/039/047/048 for unrelated concerns
(grep-confirmed against `clasi/sprints/done/*/usecases.md`); UC-023 is chosen
to avoid an obvious collision, but final numbering is a consolidation-time
decision, not this planning pass's to fix.

---

## SUC-001: RT Lands on the Commanded Angle (Coast Anticipation From Ramp Dynamics)
Parent: UC-023 (new)

- **Actor**: Python host (real hardware or sim) issuing an `RT <cdeg>` command.
- **Preconditions**: Robot (real or sim) is stationary and idle. Firmware
  `RobotConfig` has live values for `trackwidth`, `yawRateMax`, `yawAccMax`
  (all already `SET`/`GET`-able and already live-read by `Planner`, per
  sprint 067's live-reference guarantee — no new config surface needed).
- **Main Flow**:
  1. Host sends `RT <cdeg>`.
  2. `Planner::beginRotation()` computes the per-wheel encoder-arc target
     `arc` from the requested angle, `trackwidth`, and
     `effectiveSlip(cfg.rotationalSlip)` (unchanged math), then subtracts a
     coast-anticipation arc computed from the ACTUAL SOFT-ramp-down dynamics
     of the commanded spin rate (`min(cfg.yawRateMax, kRtRate)`) and the live
     `cfg.yawAccMax` — replacing today's hand-tuned, stale `kRtCoastArc`
     constant (8.0 mm, derived for an assumed 100°/s cruise rate the
     `yawRateMax=70°/s` default has not matched since at least sprint 071).
  3. The `ROTATION` stop condition (`StopCondition::Kind::ROTATION`, signed
     per sprint 072) fires once the encoder differential reaches the
     (now-accurate) stop arc; the BVC SOFT-ramps ω to 0, coasting almost
     exactly the anticipated remainder.
  4. Firmware emits `EVT done RT`.
- **Postconditions**: The commanded body rotation (per `sim.get_true_pose()`
  in sim; per an external ground-truth reference on real hardware, HIL
  follow-up) matches the commanded angle within a small, angle-independent
  tolerance — not a constant few-degree shortfall regardless of angle.
- **Acceptance Criteria**:
  - [ ] Clean-sim (default `RobotConfig`, default plant, no operator
        configuration) `RT` lands within ~1° of commanded across a 45°–300°
        sweep (45/90/180/300), once combined with SUC-002's fix.
  - [ ] The coast-anticipation quantity is computed from live
        `cfg.yawAccMax`/spin-rate, not a hardcoded constant — so a future
        change to either config field self-corrects the anticipation instead
        of silently re-introducing a stale-constant defect.
  - [ ] No change to `RT`'s wire grammar, `StopCondition`, or
        `MotionBaseline` — this SUC is a pure internal recompute inside
        `beginRotation()`.
  - [ ] Real-hardware turn behavior is expected to change (this file is
        ARM-and-sim-shared); flagged for HIL validation, not asserted by this
        sprint's own sim-only test suite (see architecture-update.md Open
        Questions).

---

## SUC-002: Simulated Plant Models Real-Chassis Rotational Scrub by Default
Parent: UC-020 (existing, sprint 069)

- **Actor**: Any sim consumer (pytest fixture, TestGUI, a future HIL-fit
  tool) that constructs a fresh `Sim()`/`SimHandle` without explicit
  plant-error configuration.
- **Preconditions**: `PhysicsWorld` has an independent, wire-settable
  body-rotational scrub channel (`_bodyRotationalScrub` /
  `SIMSET bodyRotScrub`, landed sprint 069) that defaults to 1.0 (neutral) at
  the class level. `RobotConfig.rotationalSlip` defaults to 0.92 (real,
  bench-calibrated per-robot value baked into `DefaultConfig.cpp` and used
  identically by `Planner::beginRotation()`'s arc inflation and
  `Odometry::predict()`'s dead-reckoning correction).
- **Main Flow**:
  1. A test or the TestGUI constructs a fresh `Sim()` (`SimHandle`).
  2. `SimHandle`'s constructor seeds the plant's body-rotational scrub from
     the SAME loaded `RobotConfig.rotationalSlip` it already uses to seed
     the plant's trackwidth (`hal.setTrackwidth(cfg.trackwidth)`,
     pre-existing pattern) — i.e. `hal.plant().setBodyRotationalScrub(
     effectiveSlip(cfg.rotationalSlip))` — so the plant, from construction,
     genuinely scrubs by the same factor the firmware's arc-inflation
     assumes.
  3. A `SIMSET bodyRotScrub=<v>` call (manual, or via TestGUI's "From
     Calibration" button, sprint 070-004) still overrides this seeded value
     — the seed is a DEFAULT, not a floor or a locked value.
  4. `RT <cdeg>` is issued; the Planner's `1/effectiveSlip(0.92)` inflation
     and the plant's `effectiveSlip(0.92)` scrub cancel, landing on the
     commanded angle instead of over-rotating ~+8.7%.
- **Postconditions**: A freshly-constructed sim, with zero explicit
  plant-error configuration ("clean sim"/"neutral profile"), reproduces the
  commanded RT angle — not the current ~95.2° (RT 9000) over-rotation.
- **Acceptance Criteria**:
  - [ ] `Sim()` constructed with zero configuration, `RT 9000` → true heading
        within ~1° of 90° (combined with SUC-001's coast fix).
  - [ ] `SIMSET bodyRotScrub=1.0` (explicit override back to neutral) +
        `SET rotSlip=1.0` (identity) still reproduces the pre-existing
        "no correction needed" identity behavior — the seed is overridable,
        not a hidden floor.
  - [ ] Zero change to any ARM-firmware-linked file — `SimHandle`/
        `PhysicsWorld` are HOST_BUILD/sim-only; real-robot `rotationalSlip`
        calibration and `Planner::beginRotation()`'s inflation logic are
        completely unaffected (satisfies the sprint brief's "keeps the
        real-robot path unchanged" constraint for this specific defect).
  - [ ] `PhysicsWorld`'s own class-level default (`_bodyRotationalScrub =
        1.0f`) is unchanged — bare `PhysicsWorld` unit tests that construct
        the class directly (not via `SimHandle`), e.g.
        `test_physics_world_basic.py`, are unaffected.

---

## SUC-003: Body-Truth Scrub Configuration Is Decoupled From Encoder-Report-Error Configuration
Parent: UC-020 (existing, sprint 069)

- **Actor**: Developer / test-infrastructure author configuring
  `PhysicsWorld` error models.
- **Preconditions**: `PhysicsWorld::setSlip(straight, turnExtra)` currently
  derives `_rotationalSlip = straight + turnExtra` as a side effect, even
  though `_rotationalSlip` feeds ONLY the body-rotation term (sub-step B) and
  `turnExtra` is conceptually an encoder-report-only, turn-rate-modulated
  defect (sub-step A′). Every current caller that wants a genuine body-truth
  effect via this channel (`test_sim_otos_lever_arm.py`'s 066-001 test,
  `test_physics_world_basic.py`, `test_physics_world_body_scrub.py`) passes
  `turnExtra=0.0`; the TestGUI's `slip_turn_extra` knob (an encoder-only
  control) is the only caller that ever passes a nonzero `turnExtra`, always
  with `straight=0.0` — and today relies on `effectiveSlip()`'s `<=0` clamp
  to accidentally neutralize the resulting negative sum, rather than the
  channel being structurally unreachable-by-accident-only.
- **Main Flow**:
  1. A caller invokes `setSlip(straight, turnExtra)` to configure
     encoder-report slip (sub-step A′).
  2. `_slipStraight`/`_slipTurnExtra` are set exactly as today (unchanged;
     substep A′ behavior is byte-identical).
  3. `_rotationalSlip` (feeding substep B only) is derived from `straight`
     ALONE, not `straight + turnExtra` — removing `turnExtra`'s ability to
     perturb body truth even in principle, while preserving every existing
     test's observed behavior (all pass `turnExtra=0.0` when they want a
     body-truth effect via this channel).
- **Postconditions**: An encoder-report-only configuration (`slip_turn_extra`
  via TestGUI/`set_field_profile()`) can never, even accidentally, perturb
  the plant's true chassis rotation. A genuine body-truth configuration via
  this legacy channel (`straight` alone) is unchanged.
- **Acceptance Criteria**:
  - [ ] `test_sim_otos_lever_arm.py::test_turn_with_slip_otos_matches_truth_encoder_diverges`
        (066-001) passes unmodified (uses `straight=0.7, turnExtra=0.0` —
        arithmetic result identical under the new derivation).
  - [ ] `test_physics_world_basic.py` and `test_physics_world_body_scrub.py`
        pass unmodified (same `turnExtra=0.0` pattern).
  - [ ] `PhysicsWorld::setSlip(0.0, <any turnExtra>)` produces
        `_rotationalSlip == 0.0` (was: `0.0 + turnExtra`, sign- and
        magnitude-dependent) — verified by a new direct unit assertion.
  - [ ] `docs/architecture/architecture-update-069.md`'s Open Question 4
        ("consolidating `_rotationalSlip`/`setSlip()` with the new
        `bodyRotScrub`/`bodyLinScrub` fields") is narrowed, not fully closed
        — full consolidation (retiring `_rotationalSlip` from substep B
        entirely) remains a documented future option, not attempted here
        (see architecture-update.md Design Rationale).

---

## SUC-004: TestGUI's Default Sim-Error Profile Reflects Reconciled Calibration
Parent: UC-020 (existing, sprint 069)

- **Actor**: TestGUI operator running a fresh install / a profile with no
  persisted `data/testgui/sim_error_profile.json`.
- **Preconditions**: `sim_prefs.DEFAULT_PROFILE` currently hardcodes
  `slip_turn_extra: 0.26` (a field-realism default that, combined with
  `body_rot_scrub`'s neutral `1.0` default, under-rotates turns ~14% net) and
  `body_rot_scrub: 1.0` (neutral — requires the operator to manually click
  "From Calibration", sprint 070-004, to reconcile it against the active
  robot's `rotational_slip`).
- **Main Flow**:
  1. TestGUI starts with no persisted sim-error profile (fresh install, or
     an operator explicitly resets to defaults).
  2. `sim_prefs.load_sim_error_profile()`'s fallback-default computation
     (shared with `__main__.py`'s existing "From Calibration" button logic,
     factored into one helper) resolves `body_rot_scrub` from the active
     robot's `calibration.rotational_slip` (falling back to neutral `1.0`
     with the same WARN-and-fallback semantics "From Calibration" already
     uses if no active robot config is found) and sets `slip_turn_extra` to
     `0.0` (no encoder-report defect baked into the factory default).
  3. TestGUI connects to Sim and applies this profile automatically (as it
     already does today for whatever `DEFAULT_PROFILE` holds).
  4. Clicking the Tour 1 button, or issuing a manual `RT`, now lands near the
     commanded angle out of the box — without the operator needing to
     discover and click "From Calibration" first.
- **Postconditions**: A fresh TestGUI install's default behavior matches
  what "From Calibration" already produces manually. Operators with an
  EXISTING persisted profile file are unaffected until they reset/delete it
  (a documented migration note, not a silent behavior change for existing
  installs).
- **Acceptance Criteria**:
  - [ ] `load_sim_error_profile()` with no persisted file returns
        `body_rot_scrub` matching the active robot's `rotational_slip` (or
        neutral `1.0` with a logged fallback if no active robot config is
        found) and `slip_turn_extra == 0.0`.
  - [ ] `__main__.py`'s "From Calibration" button is refactored to call the
        SAME shared helper (`sim_prefs`), not a second, duplicated
        implementation of the same lookup/fallback logic.
  - [ ] `tests/testgui/test_070_004_sim_errors_from_cal.py` and
        `tests/testgui/test_sim_prefs.py` are updated for the new defaults
        (deliberately, documented before/after — these tests currently
        assert or exercise the OLD `0.26`/`1.0` hardcoded defaults).
  - [ ] `tests/testgui/test_tour1_geometry.py::test_tour1_traces_the_tour_at_zero_error`'s
        `xfail(strict=True)` marker is removed once the combined fix (this
        SUC + SUC-001 + SUC-002) makes the test pass — `strict=True` means an
        unexpected pass is itself a failure until the marker is removed.
