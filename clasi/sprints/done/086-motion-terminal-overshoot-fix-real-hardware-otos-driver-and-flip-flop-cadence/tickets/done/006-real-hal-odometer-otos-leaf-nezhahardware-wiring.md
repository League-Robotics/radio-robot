---
id: "006"
title: "Real Hal::Odometer (OTOS) leaf + NezhaHardware wiring"
status: done
use-cases: [SUC-005, SUC-006, SUC-007]
depends-on: ["005"]
github-issue: ""
issue: nezha-hardware-otos-driver-for-new-source-tree.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Real Hal::Odometer (OTOS) leaf + NezhaHardware wiring

## Description

Implement the real `Hal::Odometer` leaf for the SparkFun OTOS sensor and
wire it into `Subsystems::NezhaHardware`. Depends on ticket 005 (lever-arm
math + boot-config surface).

**Grounding (architecture-update.md facts 4/5) — read before starting:**
- `Subsystems::Hardware::odometer()` defaults to `nullptr`; `NezhaHardware`
  does not override it today. `source/dev_loop.cpp`'s pose-estimation step
  ALREADY calls `hardware.odometer()` and, generically, for any non-null
  result, calls its `tick(now)`/`pose()` every pass before
  `PoseEstimator::tick()` runs — unconditionally, with no per-owner special
  casing. `source/commands/otos_commands.cpp` ALREADY resolves
  `hardware.odometer()` live on every one of the seven OTOS verb dispatches
  and already replies `ERR nodev` gracefully when null. **Do not modify
  `dev_loop.cpp` or `otos_commands.{h,cpp}` — they need zero changes.**
- The new leaf's `tick()` is NOT part of `NezhaHardware`'s brick flip-flop
  motor scheduler (`REQUEST_DUE`/`COLLECT_DUE`, address `0x10`) — the OTOS
  chip is a different address (`0x17`) and is driven by `dev_loop.cpp`'s own
  separate per-pass call. Do not fold OTOS scheduling into the flip-flop
  phase state machine.
- Working directory: `source/hal/otos/` (a new top-level HAL device
  directory, parallel to `source/hal/nezha/`, `source/hal/sim/`,
  `source/hal/capability/` — NOT nested under `hal/nezha/`, since the OTOS
  sensor is not a Nezha-brand device; it just happens to be orchestrated by
  the same `NezhaHardware` owner in this single-hardware-owner tree).
- Reuse `I2CBus`'s existing per-device `preClear`/`postClear` lazy-clearance
  mechanism (already generic over any 7-bit address) for the leaf's own
  register writes/reads — do not invent a second bus-safety mechanism, and
  do not become a new source of bus contention (issue 3's non-goal applies
  here too, even though this is a different issue).
- Port the register map / read sequencing from `source_old/hal/real/
  OtosSensor.{h,cpp}` (product ID detect, `init()`, `resetTracking()`,
  position/velocity burst reads, linear/angular scalar registers) —
  conforming to this tree's naming/coding standards (CamelCase, no units in
  identifiers), not copied verbatim syntax.

## Acceptance Criteria

- [x] A new leaf (working name `Hal::OtosOdometer`, `source/hal/otos/
      otos_odometer.{h,cpp}`) implements all five `Hal::Odometer` primitives
      (`init()`, `resetTracking()`, `setPose()`, `setLinearScalar()`,
      `setAngularScalar()`) plus `pose()`/`connected()`/`tick()`/`begin()`.
- [x] `pose()` applies the lever-arm compensation (ticket 005's math) using
      the same-instant heading from the same read burst — not a lagged one.
- [x] The leaf is constructed with ticket 005's boot-config values (offset,
      linear/angular scalar) — no new live `SET`/wire surface.
- [x] `Subsystems::NezhaHardware` gains one new member (the leaf instance)
      and overrides `odometer()` to return its address — the flip-flop
      scheduler (`tick()`'s `REQUEST_DUE`/`COLLECT_DUE`) is untouched.
- [x] `source/main.cpp` constructs the new leaf alongside existing hardware
      construction, wired with ticket 005's boot-config values.
- [x] `source/dev_loop.cpp` and `source/commands/otos_commands.{h,cpp}` are
      confirmed UNCHANGED (diff shows zero lines touched in either file).
- [x] Unit tests exercise the leaf's register sequencing against a scripted
      `I2CBus` fake (mirroring `NezhaMotor`'s own existing test precedent),
      without requiring real hardware.
- [x] `Hal::Odometer`'s public interface is unchanged (no new virtual, no
      signature change) — the leaf conforms to the existing five-primitive
      contract.

## Completion Notes

Implemented exactly per plan, host-tested via a scripted `I2CBus` fake, no
real hardware touched (deferred to ticket 007's HITL pass).

- **`source/hal/otos/otos_odometer.{h,cpp}`** (new) — `Hal::OtosOdometer`, a
  new top-level `source/hal/otos/` device directory (parallel to `hal/nezha/`,
  `hal/sim/`, `hal/capability/`, NOT nested under `hal/nezha/`). Ports the
  register map (`PRODUCT_ID`=0x00, `LINEAR_SCALAR`=0x04, `ANGULAR_SCALAR`=0x05,
  `IMU_CALIBRATION`=0x06, `RESET`=0x07, `SIGNAL_PROCESS_CFG`=0x0E,
  `POSITION_XL`=0x20, `VELOCITY_XL`=0x26) and read sequencing from
  `source_old/hal/real/OtosSensor.{h,cpp}` — concept/math, renamed to this
  tree's CamelCase/no-units-in-identifiers standards (e.g. `scaleToInt8` ->
  `scaleToRegister`, `sensorHrad`/`odomYaw` -> tagged-comment quantities).
  I2C address 0x17 (`Hal::kOtosDeviceAddr`), a separate device slot from the
  Nezha flip-flop's 0x10 (`Hal::kNezhaDeviceAddr`) — the two never contend on
  I2CBus's per-address clearance timers.
  - **`begin()`**: product-ID probe (expects 0x5F) gates `initialized_`
    permanently (mirrors `source_old`'s `is_initialized()` — never re-probed).
    On success: `init()` (signal processing + Kalman reset + IMU-calibration
    kick-off), boot-config linear/angular SCALE MULTIPLIERS converted to the
    chip's raw int8 register domain via `scaleToRegister()` (ported
    `scaleToInt8()` formula) and applied via the same `setLinearScalar()`/
    `setAngularScalar()` primitives OL/OA use live, then zeroes
    position+heading (matches `OtosSensor::begin()`'s rationale: the chip
    retains its tracked pose across a reflash).
  - **Deliberate deviation from `source_old`**: `init()` does NOT block-poll
    for IMU calibration completion (the old `fiber_sleep`-based ~0.77 s busy
    wait). This tree's main loop has no scheduler-yield primitive (HOST_BUILD
    has none at all), and blocking the whole dev loop for the better part of
    a second on every `OI` command is exactly the class of stall sprints
    078/079 spent real stand time eliminating from the Nezha path. This leaf
    only WRITES `REG_IMU_CALIBRATION` (fire-and-forget, matching the chip's
    own documented async behavior) and never polls for completion. Flagged
    for the stakeholder to assess on the stand (does OI's calibration still
    land adequately without the wait?) — see report below.
  - **`tick()`**: burst-reads `POSITION_XL` then `VELOCITY_XL` (each a
    register-address write + 6-byte read), applies the boot-config
    `offsetYaw` mounting-rotation to the linear components (`geometry.
    odometry_offset_mm.yaw_rad`, analogous to `source_old`'s `odomYaw` — the
    OTOS chip's own rotation relative to the robot's forward axis; heading
    and omega pass through unrotated, matching the ported rationale), then
    applies `source/hal/lever_arm.h`'s `LeverArm::sensorToCentre()` using the
    SAME-INSTANT heading `hF` from THIS burst (never a stale one — the
    db11b7c contract). Caches into `cachedPose_`; a burst failure (either
    half) holds the previously-cached pose but sets `stamp.valid = false` so
    `Subsystems::PoseEstimator::tick()` skips fusion that pass
    (`pose_estimator.cpp:115` checks `otosObs->stamp.valid`) — verified by
    reading that call site directly rather than assuming. Every `tick()`
    attempts the read regardless of the previous call's outcome (live
    per-tick `connected_`, mirrors `Hal::NezhaMotor`'s own always-retry
    semantics) — a transient bus glitch does not permanently disable further
    attempts, proven by harness scenario 6 (fail then recover).
  - **`setPose()`** (OZ/OV): exact inverse of `tick()`'s read transform,
    ported from `OtosSensor::setWorldPose()`, using `LeverArm::
    centreToSensor()`.
  - **`setLinearScalar()`/`setAngularScalar()`** (OL/OA): write the caller's
    value directly to the raw int8 register — confirmed against
    `docs/protocol-v2.md` §11 ("Gets or sets the OTOS linear scalar
    calibration register (`int8_t`)") that OL/OA operate on the raw register
    domain, NOT the 1.0-based JSON multiplier; the boot-config multiplier
    (`Config::OtosBootConfig.linearScale`/`angularScale`) is converted via
    `scaleToRegister()` once, only at `begin()`.
- **Constructor**: `Hal::OtosOdometer(I2CBus& bus, const Config::
  OtosBootConfig& config)` — takes ticket 005's boot-config struct directly
  (not a `msg::` message type, since there is deliberately no wire-serialized
  equivalent for the offset fields — this is the one Hal:: leaf with a
  Config:: dependency, noted in the header comment; `Config::` has no Hal::
  dependency of its own, so no cycle results).
- **`Subsystems::NezhaHardware`** (`nezha_hardware.{h,cpp}`): gained the
  `otosOdometer_` member (constructed alongside `motor1_..4_` from the same
  shared `I2CBus&`), `begin()` now also calls `otosOdometer_.begin()`, and a
  new `Hal::Odometer* odometer() override` returns `&otosOdometer_` — the
  flip-flop `tick()` method itself (`REQUEST_DUE`/`COLLECT_DUE`) has ZERO
  lines changed (confirmed by reading the diff: only the constructor,
  `begin()`, and a new `odometer()` method were touched). The new
  constructor parameter (`const Config::OtosBootConfig& otosConfig`) is
  DEFAULTED (`= Config::OtosBootConfig()`) so every pre-086-006 two-argument
  construction (`main.cpp` aside, four `tests/sim/unit/*_harness.cpp`
  fixtures that construct a `NezhaHardware` but never call `begin()`/
  `odometer()` on it) keeps compiling unchanged — verified none of those
  harnesses call `begin()`, so the default's identity values are
  behaviorally inert for them (confirmed by grep before relying on it).
- **`source/main.cpp`**: constructs `hardware` with a third argument,
  `Config::defaultOtosBootConfig()`. Also updated a stale doc comment
  (~line 258) that said "`odometer()` resolves nullptr ... no real-hardware
  OTOS driver this program" — no longer true; `main.cpp` is in this ticket's
  explicit file scope so this was fair game (unlike `dev_loop.cpp`'s own
  now-stale identical comment, which the ticket's hard constraint forbids
  touching — see "surprises" below).
- **Unit tests**: new `tests/sim/unit/otos_odometer_harness.cpp` +
  `test_otos_odometer.py`, mirroring `nezha_flipflop_harness.cpp`'s
  established scripted-`I2CBus` pattern (ticket 001's HOST_BUILD fake,
  `-DHOST_BUILD`, compiled against the real `otos_odometer.cpp`). Seven
  scenarios, all passing: (1) product-ID match runs the full
  probe+init+scalar+zero-pose sequence (asserted transaction count = 8);
  (2) product-ID mismatch leaves the leaf permanently uninitialized after
  only the 2-transaction probe; (3) never-`begin()`'d — every primitive
  setter and `tick()` is a zero-bus-traffic no-op; (4) `tick()`'s lever-arm-
  only transform (offsetYaw=0, tovez-realistic offset -47.7/3.5) matches
  `LeverArm::sensorToCentre()` called directly, isolating that step; (5)
  `tick()`'s mounting-yaw-rotation-only transform (zero offset, non-zero
  offsetYaw) matches the rotation formula directly, isolating that step;
  (6) a burst-read failure (induced I2C status -1 on the velocity half)
  holds the previously-cached pose, marks `stamp.valid=false`, flips
  `connected()` false, then a third clean tick() proves recovery (no
  permanent latch); (7) `setPose()`/`setLinearScalar()`/`setAngularScalar()`/
  `resetTracking()` each issue exactly one write when initialized.
- **Existing-suite fallout (necessary, not scope creep)**: `nezha_hardware.cpp`
  now hard-links against `otos_odometer.cpp` (a real member, not a pointer),
  which broke FOUR pre-existing host harnesses that compile+link
  `nezha_hardware.cpp` directly: `test_dev_command_outbox.py`,
  `test_hardware_seam.py`, `test_nezha_flipflop.py`,
  `test_otos_commands_nodev.py` (all failed with an "undefined symbols:
  `Hal::OtosOdometer::begin()`/constructor" linker error on first full-suite
  run). Fixed by adding `source/hal/otos/otos_odometer.cpp` to each Python
  wrapper's compiled-sources list. Separately, `test_otos_commands_nodev.py`
  /`otos_commands_harness.cpp`'s ENTIRE PREMISE ("`hardware.odometer()`
  inherits `Subsystems::Hardware`'s defaulted-nullptr override — never set
  here" / all seven verbs reply `ERR nodev`) is now factually false — that is
  the whole point of this ticket. Rather than deleting the test, I rewrote
  its assertions to lock in the NEW invariant (all seven verbs now reply
  `OK ...`, verified against the exact `CommandProcessor::replyOK()` format
  for each), updated both files' header comments/docstrings to explain the
  086-006 change, and left the file/function names alone (grepped: nothing
  else in the repo references either by name) to minimize churn. This
  harness deliberately never calls `hardware.begin()` (no I2C scripting), so
  it proves the wire-dispatch-level guard (`otosReady()`'s `odometer ==
  nullptr` check, itself UNCHANGED in `otos_commands.cpp`) rather than real
  I2C behavior — that's `otos_odometer_harness.cpp`'s job.
- **`otos_commands.{h,cpp}` and `dev_loop.cpp`**: confirmed ZERO lines
  changed via `git diff --stat` immediately before every commit in this
  ticket (not just once) — both files' own comments about `odometer()`
  returning nullptr for `NezhaHardware` are now stale, but the ticket's hard
  constraint explicitly forbids touching either file, so the staleness is
  accepted as a known, intentional follow-up (a future doc-only ticket can
  refresh `dev_loop.cpp`'s comment at line ~109).
- **`Hal::Odometer`'s public interface** (`source/hal/capability/odometer.h`):
  confirmed zero lines changed via `git diff --stat`.
- **CMake**: no `CMakeLists.txt` change needed — `RECURSIVE_FIND_FILE` globs
  `source/**/*.cpp` with no exclusion regex matching `hal/otos/`, so
  `otos_odometer.cpp` is automatically picked up by the real ARM build.
- **Sim shared lib** (`tests/_infra/sim/build/libfirmware_host.dylib`): NOT
  rebuilt — confirmed via its `CMakeLists.txt`'s explicit (non-glob) source
  list that `subsystems/nezha_hardware.cpp` (and therefore the new
  `hal/otos/otos_odometer.cpp`) is deliberately absent; the Python sim suite
  only ever exercises `Subsystems::SimHardware`/`Hal::SimOdometer`, both
  untouched by this ticket.
- **Full host suite**: `uv run python -m pytest -q` — 620 passed (619 after
  ticket 005 + 1 new `test_otos_odometer.py`), 0 regressions, ~147s. Also
  independently re-ran the five directly-affected harnesses (`test_dev_
  command_outbox.py`, `test_hardware_seam.py`, `test_nezha_flipflop.py`,
  `test_otos_commands_nodev.py`, `test_otos_commands.py`) plus the two new
  ones (`test_otos_odometer.py`, and ticket 005's `test_lever_arm.py`/
  `test_gen_boot_config_otos.py`) individually — all 25 green.
- **Surprises / things for the stakeholder to verify on the stand (ticket
  007)**:
  1. **IMU calibration no longer blocks** (see `begin()`/`init()` above) —
     confirm the OTOS still reports a sane, low-drift heading shortly after
     boot without the old ~0.77 s blocking wait; if drift is bad, the fix is
     narrow (e.g. delay the first real `tick()` sample, not re-add blocking).
  2. **`geometry.odometry_offset_mm.yaw_rad` is a NEW interpretation** by this
     ticket — I read it as the OTOS chip's mounting-rotation angle (analogous
     to `source_old`'s `odomYaw`, historically in degrees; the new schema's
     `yaw_rad` name implies radians already). `tovez.json`'s value is `0.0`,
     so this is functionally untested on real geometry; worth a sanity check
     if a robot is ever mounted with the OTOS rotated relative to the chassis
     forward axis.
  3. **`geometry.odometry_chip_upside_down` is intentionally NOT wired** —
     ticket 005's boot-config surface only covers `odometry_offset_mm` and
     the two scale multipliers (per its own explicit description), so this
     leaf never reads or applies the upside-down flip. `tovez.json` has it
     `false`, so no behavior change for the target robot, but flagging this
     as an explicit scope narrowing, not an oversight.
  4. **No live `SET`/wire surface added** — grepped `config_commands.cpp`;
     untouched. `OL`/`OA` already existed and are now reachable end-to-end.

## Implementation Plan

**Approach**: Read `source_old/hal/real/OtosSensor.{h,cpp}` for the register
map and read/write sequencing, and `source/hal/sim/sim_odometer.{h,cpp}`
(081-003) for this tree's `Hal::Odometer` leaf conventions (constructor
shape, how `pose()`/`tick()` are structured). Build the leaf host-testable
first (scripted `I2CBus` fake, same pattern `NezhaMotor`'s own tests use),
then wire it into `NezhaHardware`/`main.cpp`.

**Files to create/modify**:
- `source/hal/otos/otos_odometer.{h,cpp}` (new).
- `source/subsystems/nezha_hardware.{h,cpp}` — new member + `odometer()`
  override.
- `source/main.cpp` — construct the new leaf with ticket 005's boot-config
  values.

**Testing plan**:
- New unit test file exercising the leaf's register protocol against a
  scripted `I2CBus` (product-ID detect, init sequencing, position/velocity
  burst read + lever-arm-corrected `pose()`, linear/angular scalar
  set/read-back).
- Confirm `dev_loop.cpp`/`otos_commands.{h,cpp}` are untouched via `git
  diff` (this is an explicit acceptance criterion, not just an aspiration).
- Full existing `tests/sim/unit/` suite re-run to confirm no regression
  (this ticket adds a new leaf; it should not change any existing sim
  behavior since `Hal::SimOdometer` is a separate, untouched leaf).

**Documentation updates**: None required at the wire/protocol level (the
seven OTOS verbs already document this behavior in `docs/protocol-v2.md`
§11 — once this leaf is live, no prose there needs to change, since it
already describes the intended behavior, only previously unreachable on
real hardware).
