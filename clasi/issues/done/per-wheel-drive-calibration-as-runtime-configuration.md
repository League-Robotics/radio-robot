---
status: done
sprint: '132'
---

# Per-wheel drive calibration as runtime configuration

> **CLOSED AS SUPERSEDED, 2026-08-03 — the wheels are still uncorrected.** No
> code was written against this issue and `tovez`'s measured 11.1-point L/R
> plateau-tracking gap is still open. The *need* is real and carried forward;
> only the mechanism changed. Under [[the-configuration-object]] the 8
> `wheel_gain_*`/`wheel_intercept_*` values are simply fields of the drive
> config group, so per-wheel calibration arrives by sending that group — no new
> patch fields, no side-selection, no presence flags, no read-merge-write, none
> of the accessor pairs this issue specced.
>
> **Do not lose from here:** the measured baseline and the traps below still
> apply — the flash chunk-budget ceiling, `setDutyPerSpeed` doubling as the
> actuation gate, NaN defeating bounds validation, and the sim-contamination
> risk in `calibration_kwargs()`. They are carried into
> [[the-configuration-object]]; this issue is their provenance.

## Description

`App::Drive`'s per-wheel Stage A affine correction — `wheel_gain_{left,right}_{accel,decel}`
and the matching `wheel_intercept_*` — is boot-baked with no live wire arm, so
every trial value costs a full rebuild and reflash. It also ships at identity
(gain 1.0, intercept 0.0) on every robot in `data/robots/`, which leaves
mismatched wheels uncorrected.

Make the per-wheel correction settable at runtime over the protocol-v5 binary
CONFIG plane, the way Stage B's PID gains already are. This is a general
capability for all robots; `tovez` is the first robot to be characterized and
corrected with it (stakeholder, 2026-08-03).

Measured on the bench 2026-08-03 (`src/tests/bench/output/motor_survey_20260803/`,
4 robots / 8 motors, all flashed from one image so only hardware varied): motors
reach 84.9–99.1% of commanded plateau speed, a 14.2-point spread. Within-robot
L/R gaps are 2.0–4.0 points on three robots but **11.1 points on `tovez`** — the
robot every calibration in `data/robots/tovez.json` was measured against.

**Governed by [[configuration-discipline-one-file-authors-every-value]]**
(stakeholder rule, 2026-08-03): one file authors every value, everything baked
is configurable, and push and bake read that same file. These 8 Stage A fields
are the first consumers of that discipline and 8 of its 27 outstanding fields.
Two consequences for this issue specifically: the never-persist decision below
is the right one under that rule (bake, don't persist — the file is the durable
record), and the bench tooling must push **from the robot JSON**, never from
literals at a call site.

## Cause

Three of the wheel-speed controller's four correction mechanisms are inert as
shipped:

| stage | state | evidence |
|---|---|---|
| **A** — per-wheel affine map | **at identity on every robot** | `wheel_gain_* = 1.0`, `wheel_intercept_* = 0.0` in every `data/robots/*.json` |
| **B** — fast PID | **off** — all five gains 0 | `wheel_pid_* = 0.0`; `drive.h`: "every field 0 produces exactly zero PID contribution" |
| **C** — slow adaptive bias | running, but clamped to ±23.8 mm/s | live telemetry: `pid_*` exactly 0.0, `bias_*` climbing |
| deficit flag | **cannot fire** | needs `pidMax > 0` AND `deficitThreshold/Window > 0`; all are 0 |

The only feedback on a wheel today is Stage C, a 30-second adaptation capped at
±23.8 mm/s. At 200 mm/s commanded, `tovez`'s right wheel needs ~30 mm/s of
correction — **more than Stage C is allowed to give** — so it converges and
stops short, while the other three robots (needing 12–22 mm/s) reach setpoint.
This also explains the universal cold-start deficit: Stage C's bias starts at
zero every boot and takes ~10–15 s of driving to converge, reproduced on all
four boards (+2.5 to +6.2 points cold→warm).

**This is not a motor defect and does not call for rematching hardware.** At
full duty the two wheels measure L 571.7 / R 532.5 mm/s (2026-08-02) — ~7%
apart, and 2.6× the 200 mm/s being commanded. The wheels differ in *gain*,
which is exactly what a per-wheel affine map corrects.

The Stage A path is already fully wired — `data/robots/*.json` →
`gen_boot_config.py:523-548` → `Config::DriveBootConfig` →
`boot_calibration.cpp:89-92` → `Drive::setWheelCorrection()`. Only the runtime
arm is missing.

## Proposed fix

### Design decisions

**Carrier: extend `MotorConfigPatch`, do not add a `WHEELMAP` PatchKind.** That
message is already side-selected (`side` = LEFT/RIGHT, `config.proto:124-126`,
today selecting only `travel_calib`), making it the natural per-wheel carrier.
The repo already decided this question the same way: `EstimatorConfigPatch` was
widened rather than "inventing a fifth ConfigTarget/Patch-message pair … for no
behavioral gain" (`config.proto:224-246`). Free field numbers are **8–11**.
Name them for what they are — `gain_accel`, `intercept_accel`, `gain_decel`,
`intercept_decel` — matching `Config::DriveBootConfig` and the JSON keys. Do
**not** repeat the `pid.kff→kaff` / `pid.kaw→pidMax` wire-name/meaning mismatch
(`configurator.cpp:145-146`).

Adding side-selected fields makes `pid.*`'s shared-ness the visible anomaly,
which is correct — it *is* the anomaly. Pay it down structurally: add
`mergeMotorSideFieldsPatch()` beside the existing `mergeMotorGainsPatch()`
(`configurator.cpp:11-18`) and move `travel_calib` into it, so the mirrored and
side-selected families are two named functions rather than an inline block a
future field lands in the wrong half of by omission.

**`dutyPerSpeed` stays shared and boot-baked — deliberately.** The 2026-07-31
"MEASURED, NOT CONFIGURED" decision hardcoded one constant for both wheels
(`boot_calibration.cpp:88`) to stop `duty_per_speed` and `wheel_gain` being
fitted against each other. Exposing it per-wheel at runtime would reopen exactly
that. Keep **one population-scale constant**, express **all per-wheel deviation
in gain/intercept**. This is also the form `duty_sweep.py:236-247`'s
`map_gain_intercept()` already computes.

**Live-apply only — do not persist to flash.** Not merely a preference: the
flash chunk budget cannot take it. `persisted_tuning.h:99-107` gives
`kBlobSize = 2*kMotorPatchFields*5 + 25`; `persisted_tuning.cpp:143-151` gives
`kNumChunks = ceil((4 + kBlobSize)/32)` with `static_assert(kNumChunks <= 4)`.
Four new persisted fields per side takes `kMotorPatchFields` 6→10, blob 85→125,
chunks 3→**5**, and the assert fires. The ceiling is `kMotorPatchFields <= 9` —
**three** new fields, not four. Worse, that assert sits inside `#ifndef
HOST_BUILD` (`persisted_tuning.cpp:131`), so every host test stays green and the
break appears only at ARM/bench build time.

Design accordingly: merge the new fields into `persistedTuning_.motorL/R` for
live present-field merge semantics, but do **not** add them to
`putMotorPatch()`/`takeMotorPatch()` (`persisted_tuning.cpp:64-82`) and do
**not** bump `kMotorPatchFields`. `serializeSnapshot()` is explicit
field-by-field, so this works, and a reboot correctly reverts to baked JSON.
`kConfigSchemaVersion` then needs **no bump** — the blob layout is byte-identical.

#### If persistence is wanted later, the budget is solvable

The hard ceiling is **128 B** of payload: CODAL's `KEY_VALUE_STORAGE_VALUE_SIZE`
= 32 B/key × (`KEY_VALUE_STORAGE_MAX_PAIRS` = 5 − 1 for radiochan) = 4 usable
keys. Verified options against it:

| option | fields | payload | chunks | fits |
|---|---:|---:|---:|---|
| today | 17 | 89 | 3 | yes |
| +4/side, current 5 B/field encoding | 25 | 129 | 5 | **no** |
| +2/side (decel only), current encoding | 21 | 109 | 4 | yes, 19 B spare |
| **+4/side, presence bitmap** | 25 | **108** | **4** | **yes, 20 B spare** |

1. **Presence bitmap (preferred).** Replace the per-field has-byte with one
   packed bitmap: `ceil(n/8) + 4n` instead of `5n`. Fits all four fields per
   side with headroom, and is a strict improvement even today (17 fields:
   85 B → 71 B). Costs a `kConfigSchemaVersion` bump 2→3 and a wipe of stored
   tuning on upgrade — which `shouldWipe()` already exists to handle. Do it as
   its OWN change, and fix [[B-persisted-tuning-schema-version-not-bumped]] in
   the same one rather than stacking a second unbumped meaning-change.
2. **Persist the decel pair only.** Fits with no format change, and is
   defensible on its own terms: steady state always selects decel
   (`drive.cpp:156`) and accel may ship = decel anyway. Cheapest, but bakes a
   "we ran out of room" compromise into the data model.
3. **Raise the CODAL key budget** via `codal.json`'s `config` block, which
   already overrides four CODAL constants. **Unverified** — CODAL is not
   vendored in this checkout, so whether `KEY_VALUE_STORAGE_MAX_PAIRS` is
   `#ifndef`-guarded and overridable must be checked at build time. Also
   changes the flash KV layout, invalidating stored data.

**None of these is the reason to persist, though.** The byte budget is a
symptom; the argument against persistence is observability, and it does not go
away when the bytes fit — see Risk 5.

**Steady state uses the DECEL pair.** `correctedCommand()` picks accel vs decel
by `|desired| > |previous|` against last tick's *commanded* speed
(`drive.cpp:156`, `:399-403`), so holding any speed always selects decel. The
accel pair is exercised only during ramps and will fit poorly on the stand —
consider shipping accel = decel initially rather than a badly-conditioned
separate fit.

### 1. Runtime wire arm for the per-wheel map — the core change

- **`src/protos/config.proto:114-132`** — add fields 8–11 to `MotorConfigPatch`,
  with `(min)`/`(max)` on gains and `(abs_max)` on intercepts (see Safety
  below). Rewrite the message doc comment into an explicit two-column
  mirrored/side-selected table, and state the never-persisted contract citing
  `EstimatorConfigPatch`'s precedent (`config.proto:186-191`).
- **Regenerate** via `python build.py` (`build.py:95-102` runs `gen_messages.py`
  then `gen_pb2.py`). `src/firm/messages/config.h`, `wire.cpp`, `wire.h` and
  `src/host/robot_radio/robot/pb2/config_pb2.py` are **tracked** — commit the
  churn, never hand-edit. Codegen itself needs no changes (it is
  descriptor-driven). Envelope budget is a non-issue:
  `kCommandEnvelopeMaxEncodedSize = 55` against 240 (`wire.h:69-73`).
- **`src/firm/app/drive.h:220-226`** — add a `WheelCorrection` struct plus
  `wheelCorrection(bool leftWheel) const` **and** `setWheelCorrection(bool
  leftWheel, const WheelCorrection&)`. The getter is essential: routing must be
  **read-merge-write**, or a patch carrying only `gain_decel` silently resets
  `gain_accel`. Mirrors `controlGains()`/`setControlGains()` exactly. Keep the
  existing 8-arg overload (`boot_calibration.cpp:89-92` is its only caller) and
  implement it in terms of the new pair.
- **`src/firm/app/configurator.cpp:106-156`** — route the four fields per side
  in `applyMotorConfigPatch()`, using the `patch.side == msg::BoundMotorSide::LEFT`
  discriminator already used at `:150-154`. `DRIVETRAIN` keeps returning
  `ERR_UNIMPLEMENTED` (`:82-84`), unchanged.
- **`src/host/robot_radio/robot/protocol.py:845-889`** — **eight** wire keys,
  not four: `set_config()`/`config()` route by key *name*, not by a side
  argument, so a side must be in the key. Follow the `ml`/`mr` precedent
  (`:876-878`) with `…L`/`…R` suffixes. Union into `_ALL_SET_KEYS` (`:887-889`)
  and route in `set_config()` (`:1071-1081`) into the existing
  `motor_left_patch`/`motor_right_patch` split, which already fans out to two
  envelopes at `:1090-1097`.
- **`config()` needs a decision and exposes an existing bug.** `config()`
  (`:1486-1509`) collapses everything into ONE `motor_patch` with a single
  last-write-wins `motor_side`, so `config(ml=…, mr=…)` **silently drops the
  left value today**. Adding four more side-selected keys multiplies that.
  Prefer raising `ValueError` on mixed-side motor keys (consistent with its
  existing multi-target `ValueError` at `:1500-1504`); file the `ml`/`mr` bug
  separately.
- **Lints** — `check_config_sync.py:182-229` `PATCH_TO_PYDANTIC` raises
  `unmapped-patch-field`, a `FORCED_FAIL_CATEGORIES` member (`:249`), for any
  unmapped Patch field: add four entries in the two-target list form
  `travel_calib` uses (`:206-209`). Then **delete** the eight now-false
  `pydantic-field-no-patch` entries at `config_sync_allowlist.json:42-49`, which
  assert "Boot calibration with NO live wire arm -- deliberately."
- **Do NOT add these keys to `calibration_kwargs()`** (`calibration/push.py:180-191`)
  — see Risk 1. Expose them only through explicit `set_config()` calls from the
  characterization tooling.

### 2. Safety — three distinct hazards

`correctedCommand()` (`drive.cpp:157-158`) computes `(|desired| - intercept) / gain`.

- **gain ≤ 0 → divide-by-zero or sign inversion.** Handle in the wire schema:
  the generated decoder already enforces `(min)`/`(max)` on `Opt<T>` fields
  (`wire.cpp:585` → `ERR_RANGE` with the offending field number), so
  `optional float gain_accel = 8 [(min) = 0.05, (max) = 20.0];` is fail-loud
  validation for free. This would be the first `(min)`/`(max)` on a config-plane
  Patch field — `config.proto:24-43` documents that gap and offers exactly this
  choice. `gen_boot_config.py:544-547` already aborts codegen on `gain <= 0`;
  the runtime path must mirror it.
- **NaN bypasses bounds validation entirely.** `validateBounds()`
  (`wire.cpp:316-330`) uses `v < min` / `v > max`, **both false for NaN**, so a
  NaN gain passes every bound, reaches `corrGain_`, and NaNs every duty on both
  wheels. Proto options cannot catch this — add an explicit `std::isfinite()`
  rejection at the routing site returning `ERR_BADARG`.
- **A large intercept silently kills a wheel.** `drive.cpp:159`:
  `if (magnitude <= 0.0f) return 0.0f;` — an intercept above the operating speed
  range returns zero for every command, with no fault and no flag. Same bug
  class as sprint 114's sub-deadband dead zone. Bound the intercept and say so
  in the field comment.

Reject loudly; do **not** clamp in the `Drive` setter — a silently-clamped
calibration is a wrong number reporting success, precisely the unobservability
failure [[A-no-firmware-to-host-config-readback]] is about.

### 3. Observability — cheap and high-value

`Telemetry` already carries `duty_per_speed_left/right` (fields 17–18,
`telemetry.proto:441-442`) and the reply frame has ~48 B of margin
(`kReplyEnvelopeMaxEncodedSize = 192` vs 240). Add the two live **decel** gains
as fields 23–24 for partial read-back of the values actually being tuned. This
serves the "ships with the feature, not after it" mandate `telemetry.proto:130-146`
invokes for the 130-005 block, and without it a pushed map is invisible until
reboot while `duty_sweep.py` reports against JSON values the firmware isn't
running (`duty_sweep.py:154`).

### 4. Fix the deficit flag

`drive.cpp:338-350` latches only when bias **and** PID are both saturated, so
`pidMax = 0` makes it permanently false — and `wheel_deficit_threshold/window`
are *also* 0, disabling it a second time. A bias-pinned wheel must speak.
Without this, the next silent 15% deficit hides exactly as this one did.

### 5. Turn Stage B on — a real gain sweep, no reflash needed

Stage B is *already* live-tunable; needs no firmware change. Wire keys (note the
two name mismatches, `configurator.cpp:129-146`):

```python
proto.config(**{"pid.kp": …, "pid.ki": …, "pid.iMax": …,
                "pid.kff": …,   # -> ControlGains::kaff  (accel feedforward)
                "pid.kaw": …})  # -> ControlGains::pidMax (authority clamp)
```

Sprint 130-006 tried **one** point (kp=0.3, ki=0.02, iMax=20, kaff=0.23,
pidMax=30): rise 0.59→0.18 s (cleared its bound) but ripple worsened 24→33.7
mm/s and the L/R split stayed at 19.7. Reverted as "a mixed result, not a clean
win", with a real sweep recommended. Do the sweep, not another point. Watch for
the 2–3 Hz duty-domain limit cycle `drive.h` warns about.

Harness: drive from `src/tests/bench/velocity_profile_gate.py`, which already
reports plateau tracking and distance fidelity and appends to a shared summary
CSV (`--summary-csv/--board/--run`); a sweep driver mirrors
`src/tests/bench/motor_survey.py`'s structure. Add a **ripple** metric to the
gate's summary — the velocity samples are already captured, and ripple is the
limit-cycle tell.

### 6. Characterize and correct `tovez`

1. **Re-measure the plant first.** `duty_sweep.py` gives per-wheel,
   per-direction `speed = gain*duty + offset` plus a constant-free saturation
   anchor; `--from-csv` re-analyzes without hardware. This also settles an open
   anomaly: the 2026-08-02 saturation reading (L 571.7 / R 532.5) is ~24% below
   the 2026-07-31 baseline (L 760-795 / R 696) that `Drive::kDutyPerSpeed`
   descends from, never root-caused. **Do not calibrate against a plant we
   haven't just measured.**
2. Compute candidate gain/intercept via `map_gain_intercept()` (decel pair).
3. Push live over the new wire arm, re-run the gate, iterate — no reflash.
4. Bake the converged values into `data/robots/tovez.json` once settled.

Only `tovez` is characterized here; the other boards stay on the shared image as
an unmodified control group.

## Verification

- **Establish the red-test baseline FIRST** —
  [[A-seven-untriaged-failing-tests-poison-every-no-regressions-claim]]; without
  it "no regressions" is unprovable. `uv run python -m pytest src/tests/sim -q`
  was 484 passed / 2 known failures (`test_clock_sync_activation.py`,
  `test_fake_transport.py`) as of sprint 130.
- **Do NOT add coverage to `app_robot_loop_harness.cpp`** — it looks like the
  CONFIG-motor harness but is compile-broken under HOST_BUILD and xfailed
  (`test_app_robot_loop.py:110`, [[B-app-robot-loop-harness-never-compiled]]).
  It will not run.
- **The live CONFIG→Drive path is covered at**
  `src/tests/sim/system/move_protocol_harness.cpp:250-255, 1041, 1066` via
  `armorMotorConfigPatchCommand()`. Extend that hand-rolled encoder with fields
  8–11 and assert through the new `Drive` getter, including a per-side scenario.
- **`src/tests/sim/unit/app_drive_harness.cpp` has zero Stage A coverage** —
  `setWheelCorrection` and `correctedCommand` appear nowhere in the tree outside
  bench scripts, which is how Stage A shipped at identity unnoticed. Add unit
  scenarios for accel/decel branch selection off the *commanded* previous speed,
  the steady-state-always-decel property, the `magnitude <= 0` guard, and
  reverse-command sign restoration. Worth doing regardless of the runtime arm.
- **`src/tests/sim/unit/test_wire_differential.py:254`** — an exact-equality
  assert on the field-number map (`{"side":1,"travel_calib":2,…}`) that fails the
  moment a field is added. Add the four entries; extend `env_config_motor()`
  round-trips (`:397-411`) with the new kwargs and a per-side case.
- **Wire:** `src/tests/fixtures/wire_golden_vectors.txt` is a COBS/CRC
  *primitive* fixture (payload_hex → wire_hex, no message schema) — a proto
  field addition cannot invalidate it. No regeneration.
- **Bench, on the stand** (`tovez`, wheels off the ground):
  - `velocity_profile_gate.py --port <tovez>` before/after. Target: both wheels
    inside ±5%, L/R gap collapsing from 11.1 points.
  - Confirm the live push takes effect — telemetry `pid_*` non-zero and the
    plateau moving within one run. The failure mode to rule out is a
    silently-ignored patch (precedent: `ERR_UNIMPLEMENTED` on every DRIVETRAIN
    key; estimator fields that ack OK and land nowhere, `configurator.cpp:58-60`).
  - Re-run from a **cold boot** — the cold-start deficit is the real acceptance
    case and reproduced on all four boards.
  - **Build for ARM before declaring done** — the chunk-budget `static_assert` is
    HOST_BUILD-invisible.
- **Playfield:** the standing `_wheel_correction_note` in `data/robots/tovez.json`
  gates refitting this pair on *camera-measured* travel, not encoder data —
  matching two wheels by their own encoders is only as good as each wheel's
  `mm_per_wheel_deg`. Validate with the 06-issue criteria: net heading change
  ≤3°, cross-track ≤30 mm.

## Risks

1. **Sim contamination (highest).** `SimLoop.configure_from_robot()` pushes
   `set_config(**calibration_kwargs(config))` (`sim_loop.py:681`), and
   `testgui/binary_bridge.py` does the same on hardware connect.
   `TestSim::WheelPlant` is **linear**, so identity Stage A is *correct* there —
   stated at `sim_harness.h:123-129` and `sim_ctypes.cpp:610-613` ("a
   hardware-gearbox linearization is deliberately NOT mirrored here"). The
   instant `tovez.json` carries a non-identity gain, adding these to
   `calibration_kwargs()` pushes a gearbox linearization into a plant with no
   gearbox — structurally the `pid.kff` accident (`configurator.cpp:107-119`:
   one wire key onto two unrelated quantities, wheels at 43% of commanded).
2. **ARM-only build break** from the chunk budget, invisible to host tests.
   Avoided entirely by not persisting.
3. **NaN bypasses wire validation** — cheap to fix, easy to forget, NaNs both
   wheels' duty if missed.
4. **Silent wheel death** from an out-of-range intercept, with no fault path.
5. **Unobservable third tuning layer** — even live-only, a pushed map is
   invisible until reboot. Partially mitigated by the telemetry gains above.
6. **Generated-artifact churn** in tracked files; regenerate via `build.py`.
7. **The accel branch is nearly untestable on hardware** — steady state is
   always decel, so accel fits only during ramps.

## Open questions

1. **Accel pair:** `duty_sweep.py` characterizes decel only. Ship accel = decel
   initially, or extend the sweep to fit it separately?
2. **Telemetry read-back scope:** two decel gains (fields 23–24) is the cheap
   version. Is partial read-back acceptable, or should this wait for the full
   [[A-no-firmware-to-host-config-readback]] fix?

## Related

- [[B-observability-contract-is-inert-as-shipped]] — the dead deficit flag
  (section 4 is where it gets fixed).
- [[B-one-owner-per-constant-speed-floor-and-duty-per-speed]] — `duty_per_speed`
  has 3–5 owners; the "one shared constant" decision depends on that cleanup not
  reversing it.
- [[A-no-firmware-to-host-config-readback]] — bears directly on Risk 5 and Open
  question 2.
- [[B-persisted-tuning-schema-version-not-bumped]] — must be fixed first if the
  never-persist decision is ever overruled.
- [[B-app-robot-loop-harness-never-compiled]] — why the obvious test home is
  unusable.
- [[A-seven-untriaged-failing-tests-poison-every-no-regressions-claim]] —
  baseline before landing.
- Bench evidence: `src/tests/bench/output/motor_survey_20260803/README.md`
  (4-robot survey) and `src/tests/bench/output/velocity_profile_gate.md`
  (cold-boot vs converged).
- Sprint 130 ticket 006 completion notes (the one-point Stage B trial) and
  `clasi/issues/done/plus500-transient-criteria-and-plant-gain-drift-followup.md`
  (the recommended sweep, and the unexplained plant-gain drop).
