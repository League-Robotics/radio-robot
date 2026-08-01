---
status: in-progress
sprint: '129'
tickets:
- 129-009
---

# Move ALL tuning constants out of main.cpp into per-robot configuration

**Source:** code review 2026-07-30, `01-firm.md` MAJOR §4, expanded by
stakeholder direction 2026-07-31: not just `PlannerLimits` — **all** constants
hardcoded in the composition root move into proper configuration.
**Priority:** P1 — this directly contradicts the project's own sprint-114
convention ("config is fail-closed truth from `data/robots/*.json` — no
behavioral defaults baked into source"), and it recreates the exact
"one robot's gearboxes become every robot's" failure we already paid for
once with `Config::DriveBootConfig`.
**Goal served:** when a motion bug appears, the first question is "what were
the gains?" Today the answer is "read a 100-line C++ literal block and hope
it matches the flashed build." With config-as-truth, the answer is one JSON
file, diffable per robot, pushable without a reflash.

## What is wrong

`src/firm/main.cpp:341-433` assembles the entire `Motion::PlannerLimits`
struct as C++ literals — while the surrounding lines source
`defaultMotorConfigs`/`defaultDriveConfig`/`defaultDrivetrainConfig`/
`defaultOtosBootConfig` from real per-robot JSON. The hardcoded inventory:

- Profile ceilings: `vMax 400`, `aMax 300`, `aDecel 250`, `omegaMax 3.0`,
  `alphaMax 6.0`, `alphaDecel 5.0`, `jerkMax 1500`, `yawJerkMax 30`
- Loop timing: `controlPeriod 47`, `actuationDelay 47` (measured, per-robot)
- Settle/rest: `requireSettle false`, `settleRestVelocity 10`,
  `settleRestOmega 0.16`, `settleWindow 2500`, `settleEpsilonLinear 4`,
  `settleEpsilonAngular 0.035`
- Plant model: `kPlantGain 1370`, `kPlantTau 0.23` (plant-ID measured —
  the definition of per-robot data)
- Duty-stage PID: `velKff`, `velKp 0.0009`, `velKi 0.004`, `velIMax 0.25`,
  `velKaff`, `velIAccelGate 50`, `dutyFloor 0.18`
- Trim loop: `trimKp 0.15`, `trimKi 0.4`, `trimIMax 40`, `trimKaff`,
  `trimMax 80`, `decelPlanFraction 0.4`, `headingHoldGain 2.0`

The comment block explains *why* the old JSON `vel_gains`/`shaper` values
must not reach this controller (they were tuned for the deleted NezhaMotor
PID and limit-cycled the wheels) — that reasoning is correct, but the fix is
a **new planner-domain config block**, not source literals. The comment even
says so: "A planner-domain config surface can supersede these constants
later." This issue is that supersession.

## What to do

1. **Add a `planner` block to `data/robots/*.json`** carrying every value
   above, with the same `[unit]`-commented documentation style the other
   blocks use:

```json
"planner": {
  "_domain_note": "Motion::Planner limits+gains (plant ID 2026-07-26, gain ~1370 mm/s per duty, tau ~230 ms). NOT the retired vel_gains/shaper block -- those fed the deleted NezhaMotor PID.",
  "v_max": 400.0,           "a_max": 300.0,          "a_decel": 250.0,
  "omega_max": 3.0,         "alpha_max": 6.0,        "alpha_decel": 5.0,
  "jerk_max": 1500.0,       "yaw_jerk_max": 30.0,
  "control_period": 47.0,   "actuation_delay": 47.0,
  "require_settle": false,
  "settle_rest_velocity": 10.0, "settle_rest_omega": 0.16,
  "settle_window": 2500.0,
  "settle_epsilon_linear": 4.0, "settle_epsilon_angular": 0.035,
  "plant_gain": 1370.0,     "plant_tau": 0.23,
  "vel_kp": 0.0009,         "vel_ki": 0.004,         "vel_i_max": 0.25,
  "vel_i_accel_gate": 50.0, "duty_floor": 0.18,
  "trim_kp": 0.15,          "trim_ki": 0.4,          "trim_i_max": 40.0,
  "trim_max": 80.0,         "decel_plan_fraction": 0.4,
  "heading_hold_gain": 2.0
}
```

   Derived values (`velKff = 1/plant_gain`, `velKaff = plant_tau/plant_gain`,
   `trimKaff = plant_tau/2`) are computed in the loader from the measured
   primitives — store the measurement, derive the gain, so the derivation
   stays in one reviewed place.

2. **Add `Config::defaultPlannerLimits()`** mirroring the sibling loaders
   (`defaultDriveConfig` etc.), and replace the literal block in `main()`:

```cpp
// Planner limits: fail-closed config truth from data/robots/*.json --
// no behavioral defaults in source (sprint 114 convention). A missing
// planner block is a boot fault, not a silent fallback.
Motion::PlannerLimits plannerLimits = Config::defaultPlannerLimits();
plannerLimits.trackWidth = kEffectiveTrack;
plannerLimits.velocityFilterWeight = ...;  // from drivetrainConfig, as today
static Motion::Planner planner(plannerLimits);
```

3. **Fail closed.** A robot JSON without the `planner` block does not boot
   into a driveable state (same treatment as sprint 114's unconfigured-device
   rule) — it must NOT inherit another robot's plant measurements. Update the
   generator/schema so `tovez.json`, `tovez_nocal.json`, `togov.json` all
   carry the block (togov's values initially copied and marked
   `"_uncalibrated": true` until its own plant ID runs).

4. **Preserve the provenance.** The measurement history in the deleted
   comment block (plant ID dates, sweep results, limit-cycle warning) moves
   into the JSON `_domain_note`s and/or a short doc — it must not be lost.

5. Wire the block through the existing `config()` push path where live
   tuning already exists (`applyVelGains`/`applyShaperLimits`) so bench
   tuning and boot config share one vocabulary.

## Acceptance

- `main.cpp` contains no numeric planner/tuning literal (grep for
  `plannerLimits.` assignments — only `trackWidth`/`velocityFilterWeight`
  plumbing from other config remains).
- Booting with a `planner`-less JSON raises the configured boot fault;
  booting `tovez.json` yields byte-identical `PlannerLimits` to today's
  literals (assert in a host-side test comparing loader output to the
  recorded values).
- Full test suite + clean build + deploy; standing bench smoke unchanged;
  one square-tour sim run matches the pre-change closure numbers.
