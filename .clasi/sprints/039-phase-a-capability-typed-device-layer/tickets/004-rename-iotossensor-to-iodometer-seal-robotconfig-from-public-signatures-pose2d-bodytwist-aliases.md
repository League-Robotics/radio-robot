---
id: "004"
title: "Rename IOtosSensor to IOdometer; seal RobotConfig from public signatures; Pose2D/BodyTwist aliases"
status: open
use-cases:
  - SUC-039-005
depends-on:
  - "039-001"
  - "039-003"
github-issue: ""
issue: ""
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# T4 — Rename IOtosSensor to IOdometer; seal RobotConfig from public signatures; Pose2D/BodyTwist aliases

## Description

Wire up the `IOdometer` capability header (scaffolded in T1). The T1 shim
`source/hal/IOtosSensor.h` already defines `using IOtosSensor = IOdometer` and the
`OtosPose` / `OtosVelocity` / `OtosAccel` type aliases. This ticket:

1. Updates `OtosSensor` (the concrete impl) to derive from `IOdometer` and adopt the
   new `readTransformed(Pose2D&, float)` signature (no `RobotConfig&` parameter) by
   storing `const RobotConfig& _cfg` as a constructor-injected member.
2. Updates `MockOtosSensor` and `BenchOtosSensor` to implement `IOdometer` with the
   new signatures.
3. Updates all callers of `readTransformed(cfg, poseOut, heading)` to use
   `readTransformed(poseOut, heading)` — the alias `OtosPose = Pose2D` means the
   struct types are already compatible; only the function signatures change.
4. Updates `Odometry` / `Robot::otosCorrect` at the call sites for the three
   `RobotConfig&`-carrying methods.

The raw-register and calibration methods (`getPositionRaw`, `setPositionRaw`,
`calibrateImu`, `resetTracking`, `init`, `getLinearScalar`, `setAngularScalar`) are
already on `IOdometer` without `RobotConfig&` and do not change.

The golden-TLM canary must stay byte-exact (the internal math is unchanged; only the
`cfg` delivery path changes from parameter to member).

**Host-verifiable:** Yes — `OtosSensor` is excluded from the host build (CODAL-
dependent `#ifndef HOST_BUILD` or via CMake exclusion), but `MockOtosSensor` and
`BenchOtosSensor` are included. At least `MockOtosSensor` and all callers compile in
the host build.
**ARM files touched:** `OtosSensor.h/.cpp`, `NezhaHAL.h/.cpp` (constructor change).

## Approach

### Step 1 — Update OtosSensor

**`source/hal/OtosSensor.h`**:
- Remove `IOtosSensor` parent (it was an alias — clean it up); add `IOdometer`.
- Add `const RobotConfig& _cfg;` private member.
- Constructor signature: `OtosSensor(I2CBus& i2c, const RobotConfig& cfg)`.
- Update all method declarations to drop `const RobotConfig& cfg` parameter from the
  four affected methods:
  - `readTransformed(Pose2D& poseOut, float headingRad = 0.0f) const`
  - `readVelocityTransformed(BodyTwist& velOut, float headingRad = 0.0f) const`
  - `readAccelTransformed() const` → returns `BodyAccel`
  - `setWorldPose(float x_mm, float y_mm, float h_rad)` (no cfg)

**`source/hal/OtosSensor.cpp`**:
- Update constructor to add `, _cfg(cfg)` to initializer list.
- Replace every `cfg.xxx` access inside the method bodies with `_cfg.xxx`. Bodies are
  VERBATIM — only the cfg delivery path changes.
- Remove `cfg` from all parameter lists where it was passed in.

**NezhaHAL** constructs `OtosSensor` — update the constructor call to pass `cfg`:
```cpp
// NezhaHAL.h:
OtosSensor _otos;
// NezhaHAL constructor: , _otos(_bus, cfg)
```
`NezhaHAL` already receives `const RobotConfig& cfg` at construction time (it stores
`_cfg` already — confirm); this is a straightforward member-init addition.

### Step 2 — Update MockOtosSensor

**`source/hal/mock/MockOtosSensor.h`**:
- Derive from `IOdometer` (not `IOtosSensor`).
- Update method signatures to remove `const RobotConfig& cfg` parameters.

**`source/hal/mock/MockOtosSensor.cpp`**:
- Update implementations. For methods that previously used `cfg` (e.g., to apply
  `mmPerDegL` scale), the mock can either:
  (a) Store a dummy config internally, or
  (b) Apply no scaling (since mock values are already in mm/rad).
  Check the current `MockOtosSensor` implementation — it likely stores pose directly
  in mm and does not apply calibration. Remove the `cfg` parameter usage.

### Step 3 — Update BenchOtosSensor

**`source/hal/BenchOtosSensor.h/.cpp`**:
- Derive from `IOdometer` (not `IOtosSensor`).
- Update method signatures to remove `const RobotConfig& cfg` parameters.
- `BenchOtosSensor` currently integrates commanded velocity; it may or may not use
  `cfg.mmPerDegL` etc. If it does, add `const RobotConfig& _cfg` as a member with
  the same constructor-injection pattern as `OtosSensor`. Or it may already store
  whatever scalars it needs.

### Step 4 — Update callers of readTransformed / readVelocityTransformed / readAccelTransformed

Grep for all call sites:
```
grep -rn "readTransformed\|readVelocityTransformed\|readAccelTransformed\|setWorldPose" source/
```

Expected callers:
- `source/robot/Robot.cpp` (`otosCorrect` method) — uses `readTransformed(config, ...)`,
  `readVelocityTransformed(config, ...)`, `readAccelTransformed(config)`.
  Update to remove `config` argument: `readTransformed(poseOut, heading)` etc.
- `source/control/Odometry.cpp` — may call `otos.readTransformed(cfg, ...)`. Update.
- OTOS command handlers (`OI`, `OZ`, `OR`, `OV`, `OL`, `OA`, `OP`) — in
  `source/robot/Robot.cpp` or `source/app/` — use calibration/raw methods that do NOT
  take `cfg`. Verify these are untouched.
- `source/hal/BenchOtosSensor.cpp` may call `_otos.readTransformed(cfg, ...)` internally.
  Update if needed.

For `OtosPose` / `Pose2D` usage: since `using OtosPose = Pose2D;` is in `IOtosSensor.h`
(the shim), any code using `OtosPose p` still compiles. Local variable names can stay
`OtosPose` or `Pose2D` — both are the same type after the alias.

### Step 5 — Update Hardware::otos() return type

`Hardware::otos()` currently returns `IOtosSensor&`. Canonicalize to `IOdometer&`:
```cpp
virtual IOdometer& otos() = 0;
```
`IOtosSensor.h` shim makes existing callers compile. Update `NezhaHAL::otos()` and
`MockHAL::otos()` return types canonically.

### Step 6 — Confirm vendor-confinement gate

The `IOtosSensor.h` shim introduces `OtosPose = Pose2D` etc. — these are in the
`hal/` layer (below the confinement boundary), not above it. The confinement gate
checks ABOVE `source/hal/` for OTOS raw register patterns. After sealing `cfg` from
public signatures, no OTOS LSB patterns cross the interface. Confirm the baseline
is unchanged (no new hits).

## Files to Modify

- `source/io/capability/IOdometer.h` — finalize interface (already scaffolded in T1)
- `source/hal/IOtosSensor.h` — shim: `using IOtosSensor = IOdometer` + type aliases (done in T1; verify)
- `source/hal/OtosSensor.h` — derive from `IOdometer`; add `_cfg` member; update signatures
- `source/hal/OtosSensor.cpp` — constructor init + bodies verbatim; `cfg` params → `_cfg`
- `source/hal/NezhaHAL.h/.cpp` — `OtosSensor` constructor call updated to pass `cfg`
- `source/hal/mock/MockOtosSensor.h/.cpp` — derive from `IOdometer`; update signatures
- `source/hal/BenchOtosSensor.h/.cpp` — derive from `IOdometer`; update signatures; possibly add `_cfg` member
- `source/hal/Hardware.h` — `otos()` return type → `IOdometer&`
- `source/hal/mock/MockHAL.h/.cpp` — `otos()` return type → `IOdometer&`
- `source/hal/NezhaHAL.h/.cpp` — `otos()` return type → `IOdometer&`
- `source/robot/Robot.h/.cpp` — update `otos` ref type; update `otosCorrect` call sites (remove `config` arg)
- `source/control/Odometry.h/.cpp` — update any `readTransformed(cfg, ...)` call sites

## Acceptance Criteria

- [ ] `OtosSensor` derives from `IOdometer`; `readTransformed` takes no `RobotConfig&` parameter.
- [ ] `OtosSensor` stores `const RobotConfig& _cfg` as a constructor-injected member.
- [ ] `NezhaHAL` passes `cfg` to `OtosSensor` constructor.
- [ ] `MockOtosSensor` implements `IOdometer` with updated signatures.
- [ ] `BenchOtosSensor` implements `IOdometer` with updated signatures.
- [ ] `Hardware::otos()` returns `IOdometer&`.
- [ ] No caller of `readTransformed`, `readVelocityTransformed`, or `readAccelTransformed` passes `RobotConfig&`.
- [ ] `using OtosPose = Pose2D;` and `using OtosVelocity = BodyTwist;` aliases present in `IOtosSensor.h` shim.
- [ ] Vendor-confinement canary passes (no new hits; baseline unchanged from T1).
- [ ] Golden-TLM canary passes byte-exact.
- [ ] `defaultRobotConfig()` field-pin unchanged.
- [ ] Simulation tier green: `uv run --with pytest python -m pytest -q`.

## Testing Plan

- Run `uv run --with pytest python -m pytest tests/simulation/unit/test_golden_tlm.py -v` — this confirms the cfg-delivery-path change is purely mechanical.
- Run `uv run --with pytest python -m pytest tests/simulation/unit/test_vendor_confinement.py -v`.
- Run `uv run --with pytest python -m pytest -q` (full suite).
- **OTOS command handler tests** (if any test `OI`, `OZ`, `OR` verbs through the sim) — run explicitly to confirm calibration/raw-register access still works.
- If ARM toolchain present: `python3 build.py` (confirms `OtosSensor.cpp` compiles).
- **ARM-only files:** `OtosSensor.cpp` — verify `_cfg` member access is consistent.
