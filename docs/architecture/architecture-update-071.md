---
sprint: "071"
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Architecture Update — Sprint 071: Remove units from identifier names (firmware; host Python split to sprint 072)

## Sprint Changes Summary

Strip physical-unit suffixes (`Mm`, `Mms`, `Deg`, `Dps`, `Ms`, `Us`, `Pct`,
`Hz`) from **C++ firmware/sim-library identifiers** in `source/` — struct
fields, member variables, locals, parameters — replacing the unit with a
standard leading `// [unit]` comment tag. This resolves the 8 remaining
units-suffix `FIXME` markers (7 in `source/types/Config.h`, 1 whole-struct
marker in `source/state/DesiredState.h`) deferred here by sprint 070, plus
every unit-suffixed peer identifier this planning pass found across
`control/`, `state/`, `superstructure/`, `hal/`, `hal/sim/`, `commands/`,
`robot/`, and the `protos/`-generated message structs. **No behavioral
change; no wire-format change.** The full pre-existing test baseline
(**2620 passed, 0 failed, 51.89s**, confirmed this planning pass on the
current checkout) must remain green after every ticket.

**Scope decision made this pass, mirroring sprint 070's own precedent:**
`host/robot_radio/`'s unit-suffixed Python identifiers (`_mm`, `_mms`,
`_deg`, `_dps`, `_pct`, `_hz` snake_case suffixes — `read_ms` alone at ~269
call sites, plus ~101 unique `_mm`-suffixed, ~78 `_deg`-suffixed, ~77
`_ms`-suffixed, ~24 `_mms`-suffixed identifiers found by this pass's own
grep across `host/`, `tests/`, `scripts/`) are **not** in this sprint. This
sprint is scoped to `source/` (the firmware/sim C++), the `protos/`
codegen chain, and the minimum `tests/`/`docs/`/`data/robots/` touches
needed to keep the suite green given the C++ renames. See Decision 1 for
the full rationale; **sprint 072 is recommended for the host-Python half**.

The second major deliverable of this sprint is governance, not code: a
**wire-compatibility exclusion table** (below) settling, for every
unit-suffixed name that is also a wire/serialized string, whether it is
renamed or explicitly excluded — and a **comment convention** (`// [unit]`
/ future `# [unit]`) recorded in a new `docs/coding-standards.md`.

---

## Step 1: Understand the Problem

**What changes:** ~150 C++ identifiers across `source/` lose their
unit-suffix and gain a leading `// [unit]` comment. Two `RobotConfig`
peer-struct rename chains ripple through their consumers. Three
proto-generated internal message structs (`msg::PlannerConfig`,
`msg::DrivetrainConfig`, `msg::MotorConfig`) lose unit-suffixed
snake_case fields, requiring `scripts/gen_messages.py` to regenerate
`source/messages/*.h`. A handful of `tests/simulation/`/`tests/_infra/`
Python files that mirror a renamed C++ identifier by name (mock
`RobotConfig`-shaped test doubles, `SIMSET`/`SET` assertions, docstrings)
move in lock-step, mechanically, to stay green.

**What does not change:** any `SET`/`GET`/`SIMSET`/`SIMGET` wire key
string; any `TLM`/`SNAP` field-name token (`enc=`, `pose=`, `otos=`,
`vel=`, `twist=`, `line=`, `color=`, `ekf_rej=`, `wedge=`, `encpose=` —
confirmed by direct read of `RobotTelemetry.cpp`/`SystemCommands.cpp`'s
`kFieldNames[]`/`protocol.py`'s `TLMFrame`: **these are already unit-free
strings, nothing to rename**); any `data/robots/*.json` config key or the
`host/robot_radio/config/robot_config.py` pydantic fields that mirror them
1:1; any `RobotConfig` default value, EKF gain, or control-loop constant.
Sprints 067 (live-SET propagation), 068 (three-pose TLM), 069 (SIMSET
surface), and 070 (FIXME cleanup, `PhysicalStateEstimate` de-threading) are
all preserved verbatim — this sprint touches *names*, never the logic
those sprints landed.

**Why now:** the 8 `FIXME` markers are direct, current stakeholder
instructions (`Config.h`: 7, `DesiredState.h`: 1 whole-struct), explicitly
deferred from sprint 070 (`architecture-update-070.md` Decision 1) because
issue 1 is "an order-of-magnitude larger, differently-shaped change ...
that deserves its own architecture-review attention." This planning pass
confirms that assessment and narrows it further: even *within* issue 1,
the C++ half (firmware, ~150 identifiers, wire-decoupled by construction —
see Step 6 Decision 2) and the Python half (host, ~280+ identifiers,
larger and independently testable) are themselves two differently-shaped
problems that should not share one sprint's review surface.

**Codebase-alignment reading done this pass (not just the issue text):**
- `grep -rn "FIXME" source/` — confirmed exactly the 8 markers named in the
  brief remain (the 2 Config.h markers for `turnThresholdMm`/`doneTolMm`
  named in the *original* issue text no longer exist — sprint 070 Decision
  4 already deleted both fields end-to-end; this planning pass verified
  their absence directly rather than assuming the issue text was current).
- `source/robot/ConfigRegistry.cpp`'s full `CFG_*` table read in full: **every
  row binds a wire key string literal to a field-name literal as two
  separate macro arguments** (`CFG_FI("tw", trackwidthMm)`) — the wire key
  is never derived from the field name by stringification. This is the
  single most important structural fact this sprint's plan depends on (see
  Decision 2): renaming the *second* argument (the C++ field) never
  touches the *first* (the wire key), for any of the ~40 registry rows,
  with one deliberate exception (`minWheelMms`, where the two literals
  happen to be spelled identically today — see the Exclusion Table).
- `data/robots/robot_config.schema.json`'s `"firmware": {"field": ...,
  "set_key": ...}` blocks read in full (30 rows): confirms the identical
  two-literal pattern one layer up, in JSON — `set_key` is the wire key
  (excluded), `field` is the C++ field name (renamed, and the schema's
  `field` value must move in lock-step with `Config.h`, `DefaultConfig.cpp`,
  and `ConfigRegistry.cpp`'s field argument — an internal, build-time
  three-way consistency requirement, not a runtime wire constraint).
- `scripts/gen_default_config.py` and `scripts/gen_messages.py` read in
  full: both hold literal-string mapping tables that reference C++ field
  names (`ov('arriveTolMm', ...)`, `("DrivetrainConfig", "mm_per_deg_l"):
  "RobotConfig::mmPerDegL"`) — every one of these must be updated in the
  same ticket as the C++ field it names, or the generator silently falls
  back to a stale default / produces a stale doc comment.
- `host/robot_radio/config/robot_config.py` read in full: pydantic models
  use the bare attribute name as the JSON key (no `Field(alias=...)`
  anywhere in this file) — `data/robots/*.json`'s unit-suffixed keys
  (`wheel_diameter_mm`, `tag_offset_mm`, `min_wheel_mms`,
  `rotation_offset_deg[_neg]`, `max_drive_mm_s`, etc.) are therefore
  genuinely serialized identifiers, not just Python-side convenience names
  — confirmed this is a real exclusion, not a cautious over-exclusion.
- `tests/_infra/sim/sim_api.cpp`, `drive_api.cpp`, and
  `host/robot_radio/io/sim_conn.py` read/grepped: every `extern "C"`
  function **name** ctypes callers bind to is already unit-free
  (`sim_set_encoder_noise`, `sim_set_true_velocity`); only their
  **parameter** names (`float mm`, `uint32_t now_ms`) carry units, and
  ctypes calls are positional — parameter names are not part of the ABI a
  Python caller depends on. This C-ABI surface is therefore renamable
  entirely within `source/`/`tests/_infra/`, zero `host/` changes required.
- `source/commands/SimCommands.cpp`'s `kSimRegistry[]` (17 rows) read in
  full: same two-literal pattern as `ConfigRegistry` — wire key string vs.
  `simsetters::` function name are independent literals.
- `docs/usecases.md` (19 UCs) read for parent-UC narrowing; sprint 069's
  proposed-but-not-yet-consolidated UC-020/021 noted to avoid a numbering
  collision for this sprint's one new UC (UC-022).

---

## Step 2: Identify Responsibilities

| Responsibility | Owning surface | Why it changes independently |
|---|---|---|
| Document the unit-comment convention for all future identifiers (C++ and, later, Python) | New `docs/coding-standards.md` | A durable, cross-cutting convention outliving any one sprint; every other responsibility below conforms to it, so it must land first. |
| Declare and default `RobotConfig`'s calibration/timing/geometry fields; expose them via `SET`/`GET` | `source/types/Config.h`, `source/robot/DefaultConfig.cpp`, `source/robot/ConfigRegistry.cpp`, `data/robots/robot_config.schema.json` | Sole owners of the field-name ↔ wire-key ↔ JSON-schema three-way mapping; every field rename is a closed edit across exactly these four files plus that field's direct C++ consumers. |
| Represent commanded (`DesiredState`) and actuator-output (`OutputState`) per-tick state | `source/state/DesiredState.h`, `source/state/OutputState.h` | Both are the issue's own worked examples (`wheelMms`, `tgtMms`); both are small, narrowly-consumed structs from the same 047-001 refactor, independent of `RobotConfig`'s much larger consumer set. |
| Project `RobotConfig`/internal state into codegen'd internal message structs | `protos/*.proto`, `scripts/gen_messages.py`, `source/messages/*.h` (generated), `source/superstructure/PlannerConfig.{h,cpp}`, `source/subsystems/drive/DriveConfig.cpp` | Sole owners of the protobuf-codegen pipeline; a proto field rename requires regenerating headers and updating the generator's own literal mapping table in the same edit, independent of whether the `RobotConfig` field it mirrors was itself renamed in the same ticket. |
| Compute dead-reckoned/EKF-fused pose from explicit inputs/outputs | `source/control/Odometry.{h,cpp}`, `source/state/EKFTiny.{h,cpp}`, `source/state/PhysicalStateEstimate.{h,cpp}` | Sole owners of the (070-003-narrowed) per-call estimator contract; this sprint renames its already-unit-suffixed parameter names (`encLeftMm`, `v_otos_mmps`, `trackwidthMm`, …) introduced fresh by 070, independent of `RobotConfig`'s own rename. |
| Drive the per-wheel velocity inner loop and body-twist profiling | `source/control/MotorController.{h,cpp}`, `source/control/VelocityController.{h,cpp}`, `source/control/BodyVelocityController.{h,cpp}`, `source/control/BodyKinematics.{h,cpp}` | Sole owners of the motor-control locals/members with unit suffixes (`velLMms`, `trueVelLMms`, `kAtRestVelEpsilonMms`); independent of the goal-closure math in `Planner`. |
| Compute goal-closure math and drive the RT/TURN/G state machines | `source/superstructure/Planner.{h,cpp}`, `source/control/PlannerBegin.cpp`, `source/control/StopCondition.{h,cpp}`, `source/control/MotionCommand.{h,cpp}` | Sole owners of the arc/rate/angle locals (`arcMm`, `rateDps`, `currentAngleDeg`, `kRtCoastArcMm`); independent of the inner velocity loop. |
| Read/convert real-hardware sensors (encoder ticks, OTOS chip) | `source/hal/real/Motor.{h,cpp}`, `source/hal/real/OtosSensor.{h,cpp}`, `source/robot/NezhaHAL.{h,cpp}` | Sole owners of the derived-unit calibration factor (`mmPerDeg`) and hardware-register-adjacent locals; independent of the sim plant. |
| Model the sim plant's true/reported physical behavior and expose it over `SIMSET`/`SIMGET` | `source/hal/sim/PhysicsWorld.{h,cpp}`, `source/hal/sim/SimOdometer.{h,cpp}`, `source/commands/SimSetters.{h,cpp}`, `source/commands/SimCommands.cpp` (internal names only), `tests/_infra/sim/*.cpp` (parameter names only) | Sole owners of the sim-only error-model internals; the `kSimRegistry[]` wire-key strings are untouched (Exclusion Table), so this responsibility is purely internal renaming, independent of every real-hardware surface above. |
| Keep prose documentation and generated inventories consistent with the renamed identifiers | `docs/protocol-v2.md`, `docs/architecture.md`, `docs/overview.md`, `docs/kinematics-model.md`, `docs/design/message-inventory.md` | These files quote specific C++ field names in prose/tables; each becomes stale the instant the identifier it quotes is renamed, so a final sweep is its own responsibility, run last. |

No responsibility spans more than one row's file set except where a field
flows through the codegen chain (Config.h → DefaultConfig.cpp →
ConfigRegistry.cpp → schema.json), which is intentionally treated as **one**
coordinated edit per field, not four independent ones — see Decision 3.

---

## Step 3: Subsystems and Modules (this sprint's ticket sequence)

| Module (ticket) | Purpose (one sentence, no "and") | Boundary | Use cases served |
|---|---|---|---|
| **001 — Coding-standards convention doc** | Documents the `// [unit]` comment convention that every subsequent ticket conforms to. | Inside: `docs/coding-standards.md`'s new "Units in Identifiers" section. Outside: any code change. | SUC-007 |
| **002 — `RobotConfig` field renames** | Renames `Config.h`'s unit-suffixed fields (the 7 FIXME'd + peers) and their build-time codegen chain. | Inside: `Config.h`, `DefaultConfig.cpp`, `ConfigRegistry.cpp` (field arguments only), `robot_config.schema.json` (`firmware.field` values only), and every direct call site reading a renamed field. Outside: any `set_key`/wire-key string (Exclusion Table). | SUC-001 |
| **003 — `DesiredState`/`OutputState` renames** | Renames the commanded-state and actuator-output structs' unit-suffixed fields. | Inside: `DesiredState.h`, `OutputState.h`, and their direct consumers (`PlannerBegin.cpp`, `Planner.cpp`, `MotionCommandHandlers`, `BodyVelocityController`). Outside: `RobotConfig` (ticket 002's concern). | SUC-003 |
| **004 — Proto-generated message renames** | Renames unit-suffixed snake_case fields in the three affected `.proto` files and regenerates their C++ headers. | Inside: `planner.proto`, `drivetrain.proto`, `motor.proto`, `gen_messages.py`'s mapping table, `source/messages/*.h` (regenerated), `PlannerConfig.{h,cpp}`, `DriveConfig.cpp`, `message-inventory.md`. Outside: the `RobotConfig` fields these proto fields are projected from (already renamed by 002; this ticket updates the projection's *other* side). | SUC-004 |
| **005 — Estimation, motion, and goal-closure identifier sweep** | Renames remaining unit-suffixed identifiers in the pose-estimation, motor-control, and goal-closure call chain. | Inside: `Odometry`, `EKFTiny`, `PhysicalStateEstimate`, `MotorController`, `VelocityController`, `BodyKinematics`, `Planner`/`PlannerBegin.cpp`, `StopCondition`, `MotionCommand`. Outside: `hal/real/` (006's concern), `hal/sim/` (007's concern). | SUC-002, SUC-004 |
| **006 — Real-hardware HAL identifier sweep** | Renames unit-suffixed identifiers in the real-sensor/motor drivers. | Inside: `Motor.{h,cpp}` (incl. the `mmPerDeg` derived-unit rename), `OtosSensor.{h,cpp}`, `NezhaHAL.{h,cpp}`. Outside: the sim plant (007's concern). | SUC-005 |
| **007 — Sim-library identifier sweep** | Renames internal (non-wire) unit-suffixed identifiers in the sim plant and its `SIMSET`/`SIMGET` glue. | Inside: `PhysicsWorld`, `SimOdometer`, `SimSetters`, `SimCommands.cpp` (function/local names only — never `kSimRegistry[]`'s key strings), `tests/_infra/sim/*.cpp` (parameter names only). Outside: any `SIMSET`/`SIMGET` wire key string. | SUC-006 |
| **008 — Final sweep, docs, and closure** | Confirms zero remaining unit-suffixed identifiers in `source/`, zero remaining `FIXME` markers, byte-identical wire output, and green suite; updates prose docs. | Inside: `docs/protocol-v2.md`, `docs/architecture.md`, `docs/overview.md`, `docs/kinematics-model.md`; the final `grep`-based acceptance checks. Outside: no source code change expected (a documentation-only ticket, unless the sweep finds a residual the earlier tickets' automated tests didn't catch). | SUC-001–007 |

Every module addresses at least one SUC; no module is speculative — each
row names files this planning pass read directly. Ticket 008 acts as the
sprint's own acceptance-criteria closure, mirroring the issue's own
acceptance criteria list line-for-line.

---

## Step 4: Diagrams

### 4a. Ticket dependency graph

```mermaid
graph TD
    T001["001: coding-standards doc"]
    T002["002: RobotConfig fields\n(Config.h + codegen chain)"]
    T003["003: DesiredState / OutputState"]
    T004["004: proto message fields\n(depends on 002's new field names)"]
    T005["005: estimation / motion / goal-closure\n(consumes 002, 003, 004's renamed names)"]
    T006["006: real HAL sweep\n(consumes 002's renamed fields)"]
    T007["007: sim-library sweep\n(consumes 002's renamed fields;\nnever touches SIMSET/SIMGET key strings)"]
    T008["008: final sweep, docs, closure"]

    T001 --> T002
    T001 --> T003
    T002 --> T004
    T002 --> T005
    T003 --> T005
    T004 --> T005
    T002 --> T006
    T002 --> T007
    T005 --> T008
    T006 --> T008
    T007 --> T008
```

No cycles. `001` is the sole root (every ticket's `// [unit]` comments must
follow its convention). `002` is the widest fan-out node (4 dependents) —
expected, since `RobotConfig` is read by nearly every subsystem; this
is pre-existing fan-in on `RobotConfig` (unrelated to this sprint) surfacing
as ticket-ordering fan-out, not a new architectural coupling.

### 4b. Wire-boundary component diagram — what is renamed vs. excluded

```mermaid
graph TD
    subgraph "Renamed this sprint (source/, internal only)"
        CFG["RobotConfig fields\n(Config.h)"]
        DESIRED["DesiredState / OutputState\nfields"]
        PROTOFIELDS["msg::PlannerConfig /\nDrivetrainConfig fields"]
        ESTIM["Odometry / EKFTiny /\nPhysicalStateEstimate\nparams+locals"]
        MOTION["MotorController / Planner /\nBodyKinematics locals"]
        HALREAL["Motor / OtosSensor /\nNezhaHAL locals"]
        SIMLIB["PhysicsWorld / SimOdometer /\nSimSetters function+field names"]
    end

    subgraph "Excluded this sprint (wire/serialized strings — unchanged)"
        SETKEY["SET/GET key strings\n(ConfigRegistry.cpp arg 1,\nschema.json set_key)"]
        SIMKEY["SIMSET/SIMGET key strings\n(SimCommands.cpp kSimRegistry[] arg 1)"]
        TLMTOK["TLM/SNAP field tokens\n(enc=, pose=, otos=, encpose=, ...)\n-- already unit-free, no-op"]
        JSONCFG["data/robots/*.json keys +\nrobot_config.py pydantic fields\n(host_config schema — deferred)"]
    end

    subgraph "Deferred to sprint 072 (out of this sprint)"
        HOSTPY["host/robot_radio/ Python\nidentifiers (_mm, _ms, _deg, ...)"]
    end

    CFG -->|"exposed via (unchanged)"| SETKEY
    SIMLIB -->|"exposed via (unchanged)"| SIMKEY
    ESTIM -->|"feeds (unchanged text)"| TLMTOK
    CFG -.->|"NOT renamed together\n(explicit exclusion)"| JSONCFG
```

No cycles. The diagram's point is structural: every renamed box has an
outbound edge to an unchanged wire-string box, never the reverse — nothing
downstream of a wire key or JSON schema key depends on the C++ identifier
that happens to resemble it.

No entity-relationship diagram is included — no data model or persisted
schema changes shape this sprint (Migration Concerns).

---

## Wire-Compatibility Exclusion Table

Per-name disposition for every unit-suffixed name that is also a
wire-visible or serialized string. Default is **exclude** (keep the wire
string stable, rename only the internal identifier); every row states why.

| Wire/serialized name | Surface | Disposition | Rationale |
|---|---|---|---|
| `tw`, `ml`, `mr`, `arriveTol`, `lag.otos`, `lag.line`, `lag.color`, `lag.ports`, `ctrlPeriod`, `tlmPeriod`, `minSpeed`, `sTimeout`, `tick`, `turnGate`, `rotSlip`, `rotGainPos/Neg`, `rotOffPos/Neg`, `odomYaw`, `vel.*`, `sync`, `ekfRHead`, `ekfQ*`, `ekfROtos*`, `ekfREncV`, and every other `ConfigRegistry.cpp` `CFG_*` key string (~40 rows total) | `SET`/`GET` wire key | **EXCLUDE.** Already unit-free (confirmed by reading every row — see Step 1). No rename needed at all; listed to make the "already clean" fact explicit and auditable. |
| `minWheelMms` | `SET`/`GET` wire key **and** current C++ field name (the one row where both literals are spelled identically) | **EXCLUDE the key; RENAME the field.** `CFG_F("minWheelMms", minWheelMms)` becomes `CFG_F("minWheelMms", minWheelSpeed)` — the wire key string is a separate macro argument and stays exactly `"minWheelMms"`; only the second argument (the C++ field) changes. This is the one row where a careless whole-word rename could accidentally touch the wire key — flagged explicitly for ticket 002's implementer. |
| `robot_config.schema.json`'s `firmware.set_key` values (mirrors the `ConfigRegistry.cpp` keys one layer up in JSON) | Schema-declared wire key | **EXCLUDE.** Same string, same reason, second declaration site. |
| `trackwidthMm`, `motorOffsetL`, `motorOffsetR`, `encScaleErrL/R`, `encSlipL/R`, `encNoiseL/R`, `otosLinScaleErr`, `otosAngScaleErr`, `otosLinNoise`, `otosYawNoise`, `otosLinDriftMmS`, `otosYawDriftDegS`, `bodyRotScrub`, `bodyLinScrub` | `SIMSET`/`SIMGET` wire key (`SimCommands.cpp` `kSimRegistry[]` argument 1) | **EXCLUDE.** Sim-only wire vocabulary consumed by the TestGUI Sim Errors panel and `tests/simulation/unit/test_simset_profile_chunking.py`/`test_sim_commands_registry.py`. `trackwidthMm`'s SIMSET key is a **different, independently-declared string constant** from `RobotConfig::trackwidthMm`'s C++ field name (renamed) and from `tw`'s SET key (also excluded, also already unit-free) — three separate namespaces that happen to share a substring; ticket 007's implementer must not conflate them when grepping. `otosLinDriftMmS`/`otosYawDriftDegS` are the two sim keys that most resemble the internal function names beside them (`simsetters::otosLinDriftMmS`) — the **function name is renamed**, the **key string is not**. |
| `enc=`, `pose=`, `otos=`, `vel=`, `twist=`, `line=`, `color=`, `ekf_rej=`, `wedge=`, `encpose=`, `mode=`, `seq=`, `t=` and the `STREAM fields=<name>` vocabulary (`kFieldNames[]` in `SystemCommands.cpp`) | `TLM`/`SNAP` wire field tokens | **EXCLUDE — no-op.** Confirmed by direct read: none of these tokens embed a unit suffix today. Listed so the sprint's acceptance criteria don't misfire looking for a rename that was never needed. |
| `data/robots/*.json` keys (`wheel_diameter_mm`, `tag_offset_mm`, `drive_axle_offset_mm`, `odometry_offset_mm`, `wheelbase_mm`, `mm_per_wheel_deg_left/right`, `rotation_offset_deg[_neg]`, `min_wheel_mms`, `max_drive_mm_s`, `max_turn_deg_s`, `min_drive_mm_s`, `crawl_threshold_mm_s`, `cmd_to_mm_per_s`, `gripper_offset_mm`, `max_rot_accel_dps2`) and the `host/robot_radio/config/robot_config.py` pydantic fields that mirror them 1:1 | Per-robot JSON config (serialized on disk, read by both `gen_default_config.py` and the host) | **EXCLUDE (whole surface).** `robot_config.py`'s pydantic models have no `Field(alias=...)` anywhere — the Python attribute name **is** the exact JSON key. Renaming any of these fields breaks deserialization of every `data/robots/*.json` file unless done in lock-step across every JSON file, the schema, and every reader — a disproportionate, purely-cosmetic risk for a "no behavioral change" sprint, and it is data-file content, not a source-code identifier, under the issue's own framing. Recommended as an optional future sprint if desired (Decision 6), not required by this issue's acceptance criteria. |
| `extern "C"` function **names** in `tests/_infra/sim/sim_api.cpp`/`drive_api.cpp` (`sim_set_encoder_noise`, `sim_tick`, `sim_command`, …) | C-ABI symbol name, called by `host/robot_radio/io/sim_conn.py` via `ctypes` | **NOT AN EXCLUSION — already unit-free.** Confirmed by reading every exported function signature: none of the ~30 exported names carry a unit suffix (only their *parameter* names do, e.g. `sim_set_enc_l(void* h, float mm)`, and ctypes calls are positional — parameter names are not part of the ABI). Parameter names ARE renamed (ticket 007); function names are untouched because they were already clean. |

**Summary:** exactly one wire key (`minWheelMms`) and one JSON-mirroring
pydantic surface (the whole per-robot config schema) required a genuine
exclusion decision. Every other wire/serialized name this sprint's own
grep found was **already unit-free** — this sprint's C++ field renames are
wire-safe by construction, because `ConfigRegistry`/`SimCommands`/the
JSON schema all bind wire-key literals as separate macro/JSON arguments
rather than deriving them from the field name by stringification (see
Decision 2).

---

## Comment Convention (`docs/coding-standards.md`, Ticket 001)

**Format:** a leading, bracketed unit tag as the *first token* of the
declaration's trailing (or block) comment:

```cpp
// Before
float tgtMms[kWheelCount] = {};  // all-wheel speed targets, mm/s

// After
float tgtSpeed[kWheelCount] = {};  // [mm/s] all-wheel speed targets
```

```python
# Before
def send(self, cmd: str, read_ms: int = 500) -> dict: ...

# After (sprint 072)
def send(self, cmd: str, read_timeout: int = 500) -> dict:  # [ms]
    ...
```

- **Unit vocabulary:** reuse whatever unit text the surrounding prose
  already uses elsewhere in the file (`mm`, `mm/s`, `mm/s²` or `mm/s^2`,
  `deg`, `deg/s`, `deg/s²`, `ms`, `us`, `%`, `Hz`, `rad`, `rad/s`,
  `rad²/s`, `mm²/s`) — do not invent a second vocabulary; grep for the
  unit already used in the block comment above the field being renamed.
- **Grep-ability:** the tag's fixed leading position means `grep -rn "//
  \[mm/s\]" source/` (or `grep -rn "# \[ms\]" host/` in sprint 072) finds
  every declaration of that unit, independent of identifier spelling. This
  is the convention's whole purpose, per the issue's own design point 1.
- **No tag for dimensionless/boolean/enum fields** — `rotationalSlip`,
  `kFF`, `velKp`, `odomUpsideDown`, `drivetrain` never had a unit suffix
  and get no tag (nothing to disambiguate).
- **Ambiguity-resolution rule (issue design point 3):** where stripping a
  unit suffix would collide two previously-distinguished names (e.g. a
  `Mm`-suffixed float position vs. a raw-ticks integer counterpart),
  ticket authors choose a descriptive replacement for the *kind* of
  quantity (`positionLinear` vs. `positionTicks`) rather than a bare
  strip that produces a collision or an ambiguous bare word. This
  planning pass found no confirmed same-scope collision case in the
  Config.h/DesiredState.h FIXME set itself, but flags the rule for
  tickets 005/006 (`Odometry`/`Motor`) where raw-tick and mm-scaled
  siblings are more likely to coexist — a per-ticket judgment call, not
  resolved exhaustively here (module-level planning, not a symbol table).
- **Derived-unit names (issue design point 4):** rename to what the
  quantity *is*, with the unit in the comment — e.g. `mmPerDegL`/`mmPerDegR`
  (and the mecanum siblings `mmPerDegFR/FL/BR/BL`) → `wheelTravelCalibL`/
  `wheelTravelCalibR`/`wheelTravelCalibFR/FL/BR/BL`  `// [mm/deg] wheel
  linear travel per motor-shaft degree of rotation`. Simply stripping
  `Deg` from `mmPerDegL` would leave `mmPerL`, which still embeds `mm` and
  reads worse, not better — this is exactly the case the issue's design
  point 4 anticipates.

---

## Step 5: What Changed / Why / Impact / Migration

### What Changed (by ticket)

**001 — `docs/coding-standards.md` (new file):** the convention above,
plus a short "why" paragraph citing the issue and this document.

**002 — `RobotConfig` field renames:**
- `source/types/Config.h`: `trackwidthMm`→`trackwidth`, `minWheelMms`→
  `minWheelSpeed`, `rotationOffsetDeg`→`rotationOffset`,
  `rotationOffsetDegNeg`→`rotationOffsetNeg`, `arriveTolMm`→
  `arriveTolerance`, `tlmPeriodMs`→`tlmPeriod`, `lagOtosMs`→`lagOtos`,
  `halfTrackMm`→`halfTrack` (all 7 FIXME'd fields + the untagged
  `rotationOffsetDegNeg` sibling) plus peers: `lagLineMs`→`lagLine`,
  `lagColorMs`→`lagColor`, `lagPortsMs`→`lagPorts`, `minSpeedMms`→
  `minSpeed`, `tickMs`→`tick`, `sTimeoutMs`→`sTimeout`,
  `controlPeriodMs`→`controlPeriod`, `odomYawDeg`→`odomYaw`,
  `halfWheelbaseMm`→`halfWheelbase`, `mmPerDegL/R`→`wheelTravelCalibL/R`,
  `mmPerDegFR/FL/BR/BL`→`wheelTravelCalibFR/FL/BR/BL`. All 8 `FIXME`
  comments removed; every renamed field gains a `// [unit]` tag.
- `source/robot/DefaultConfig.cpp`: every `p.<oldField>` assignment
  updated to `p.<newField>`; every `ov('<oldField>', ...)` call updated to
  `ov('<newField>', ...)`.
- `source/robot/ConfigRegistry.cpp`: every `CFG_*` row's **second**
  argument (the field name) updated; **first** argument (the wire key)
  untouched (Exclusion Table).
- `data/robots/robot_config.schema.json`: every `"firmware": {"field":
  "<oldField>", ...}` value updated to `"<newField>"`; `"set_key"` values
  untouched.
- Direct consumers updated (mechanical, same-file-set as the field's
  existing readers): `Planner`/`PlannerBegin.cpp`, `Drive.cpp`,
  `MotorController`, `Motor.cpp`, `OtosSensor`, `BodyVelocityController`,
  `NezhaHAL`.
- `tests/_infra/default_config_golden.json`, `tests/simulation/unit/
  test_config_registry.py`: regenerated/updated (field names in Python
  dict literals and mock-config kwargs mirroring `RobotConfig`, e.g.
  `tests/simulation/unit/test_body_velocity_controller.py`'s local
  `RobotConfig` test-double class's `trackwidthMm` kwarg → `trackwidth`).

**003 — `DesiredState`/`OutputState` renames:**
- `source/state/DesiredState.h`: `wheelMms`→`wheelSpeeds`,
  `targetSpeedMms`→`targetSpeed`, `distanceTargetMm`→`distanceTarget`,
  `deadlineMs`→`deadline`. Whole-struct `FIXME` comment removed.
- `source/state/OutputState.h`: `tgtMms`→`tgtSpeed` (the issue's own
  worked example, applied verbatim).
- Consumers updated: `PlannerBegin.cpp`, `Planner.cpp`,
  `MotionCommandHandlers`, `BodyVelocityController`.

**004 — Proto-generated message renames:**
- `protos/drivetrain.proto`: `mm_per_deg_l`→`travel_calib_l`,
  `mm_per_deg_r`→`travel_calib_r` (and any `half_track_mm`/
  `half_wheelbase_mm`/`arrive_tol_mm` fields present — confirmed at ticket
  time against the current `.proto` source, not assumed from this
  document).
- `protos/motor.proto`: `mm_per_deg`→`travel_calib`.
- `scripts/gen_messages.py`: the field-name mapping table's literal pairs
  updated (`("DrivetrainConfig", "mm_per_deg_l"): "RobotConfig::mmPerDegL"`
  → `("DrivetrainConfig", "travel_calib_l"):
  "RobotConfig::wheelTravelCalibL"`); `source/messages/*.h` regenerated.
- `source/superstructure/PlannerConfig.{h,cpp}`, `source/subsystems/drive/
  DriveConfig.cpp`: accessor call sites updated to the new proto field
  names.
- `docs/design/message-inventory.md`: regenerated.

**005 — Estimation/motion/goal-closure sweep:**
- `source/state/PhysicalStateEstimate.{h,cpp}`, `source/control/
  Odometry.{h,cpp}`: `encLeftMm`/`encRightMm`→`encLeft`/`encRight`,
  `trackwidthMm`→`trackwidth` (parameter, mirrors 002's field rename),
  `v_otos_mmps`→`vOtos`, `omega_otos_rads`→`omegaOtos`,
  `vy_otos_mmps`→`vyOtos`, `x_mm`/`y_mm`/`h_cdeg`→`x`/`y`/`h` (each with a
  `// [unit]` tag), `now_ms`→`now` `// [ms]`.
- `source/control/MotorController.{h,cpp}`, `VelocityController.{h,cpp}`:
  `velLMms`/`velRMms`→`velLeft`/`velRight`, `trueVelLMms`/`trueVelRMms`,
  `kAtRestVelEpsilonMms`, and peers renamed with `// [mm/s]` tags.
- `source/control/BodyKinematics.{h,cpp}`, `BodyVelocityController.{h,cpp}`:
  remaining `Mms`-suffixed locals renamed.
- `source/superstructure/Planner.{h,cpp}`, `source/control/
  PlannerBegin.cpp`: `arcMm`→`arc`, `rateDps`→`rate`,
  `currentAngleDeg`/`setAngleDeg`→`currentAngle`/`setAngle`,
  `kRtRateDps`→`kRtRate`, `kRtCoastArcMm`→`kRtCoastArc`, each `// [unit]`
  tagged.
- `source/control/StopCondition.{h,cpp}`, `MotionCommand.{h,cpp}`:
  remaining `Mm`/`Ms`-suffixed fields renamed.

**006 — Real-hardware HAL sweep:**
- `source/hal/real/Motor.{h,cpp}`: local `mmPerDeg`→`wheelTravelCalib`
  (mirrors 002's derived-unit field rename), `_lastPositionMm`→
  `_lastPosition` `// [mm]`.
- `source/hal/real/OtosSensor.{h,cpp}`, `source/robot/NezhaHAL.{h,cpp}`:
  remaining unit-suffixed locals/members renamed.

**007 — Sim-library sweep:**
- `source/hal/sim/PhysicsWorld.{h,cpp}`: `_driftPerTickMm`→`_driftPerTick`,
  `sigmaMm`→`sigma`, `encoderScaleErrL/R` locals, etc. renamed; internal
  method names mirroring `SIMSET` keys (e.g. `simsetters::otosLinDriftMmS`)
  renamed to e.g. `simsetters::otosLinearDrift` — **`kSimRegistry[]`'s key
  string argument in `SimCommands.cpp` is untouched.**
- `source/hal/sim/SimOdometer.{h,cpp}`, `source/commands/
  SimSetters.{h,cpp}`: mirrors the above.
- `tests/_infra/sim/sim_api.cpp`, `drive_api.cpp`: parameter names only
  (`float mm`→`float distanceMm`... resolved per the convention, e.g.
  `float mm` → `float distance` `// [mm]`); function names untouched
  (already clean, per the Exclusion Table).

**008 — Final sweep, docs, closure:**
- `docs/protocol-v2.md`, `docs/architecture.md`, `docs/overview.md`,
  `docs/kinematics-model.md`: prose/table mentions of renamed C++ field
  names updated.
- Final `grep -rniE "\b[a-z_][a-z0-9_]*(mm|mms|deg|dps|us|pct|hz)\b"
  source/` (case-insensitive, word-boundary) and `grep -rn "FIXME"
  source/` both return **zero** results (excluding this document's own
  historical prose).
- Full `uv run python -m pytest` run: 2620 passed (or the then-current
  ticket-005/006/007-adjusted count, since each ticket may add a small
  number of new/renamed test functions), 0 failed.

### Why

- Tickets 002/003 exist because they are the issue's own named FIXME
  markers — direct stakeholder instructions, deferred from 070 for scope
  reasons (070 Decision 1), now due.
- Ticket 004 exists because `RobotConfig`'s renamed fields (002) are
  projected into `msg::` structs by name via `scripts/gen_messages.py`'s
  literal mapping table — leaving the proto/generated side unrenamed would
  leave the mapping table (and hence the generated header's field name)
  silently referencing the *old* `RobotConfig::` name in a comment/mapping
  that no longer resolves, and leave a units-suffixed name on the
  `msg::` side that the issue's "everywhere" scope explicitly covers.
- Ticket 005 is ordered after 002/003/004 because its files are the
  **consumers** of all three surfaces (`Planner` reads `RobotConfig` via
  `_cfg` per sprint 067, reads `DesiredState` per sprint 047, and reads
  `msg::PlannerConfig`) — renaming its own locals before its inputs are
  renamed would mean touching every call site twice.
- Tickets 006/007 are ordered after 002 (both read renamed `RobotConfig`
  fields) but are mutually independent of each other and of 005 (real HAL
  and sim HAL are parallel implementations of the same interfaces, never
  calling into each other) — listed sequentially only because tickets
  execute serially in this project, not because of a real dependency.
- Ticket 008 exists because the issue's own acceptance criteria are
  phrased as a whole-codebase invariant ("no identifier ... embeds a unit
  suffix ... except documented exclusions") that only a final,
  whole-`source/`-tree grep can certify; the docs sweep exists because
  five prose documents were found, by this planning pass's own reading, to
  quote specific C++ field names that would otherwise silently go stale.

### Impact on Existing Components

| Component | Impact |
|---|---|
| `source/types/Config.h`, `DefaultConfig.cpp`, `ConfigRegistry.cpp` | **Modified** (ticket 002). Field names change; wire key strings, defaults, and validation logic do not. |
| `data/robots/robot_config.schema.json` | **Modified** (ticket 002, `firmware.field` values only). `set_key`, all other schema content unchanged. |
| `data/robots/*.json` (`tovez.json`, `togov.json`, …), `host/robot_radio/config/robot_config.py` | **Unaffected** (Exclusion Table) — explicitly excluded from this sprint. |
| `source/state/DesiredState.h`, `OutputState.h` | **Modified** (ticket 003). Both `FIXME` markers resolved. |
| `protos/*.proto`, `source/messages/*.h`, `PlannerConfig.{h,cpp}`, `DriveConfig.cpp` | **Modified** (ticket 004). Internal message schema only — never wire-transmitted (confirmed by 070's own architecture reading, reconfirmed here). |
| `source/control/Odometry.{h,cpp}`, `EKFTiny.{h,cpp}`, `PhysicalStateEstimate.{h,cpp}`, `MotorController.{h,cpp}`, `VelocityController.{h,cpp}`, `BodyKinematics.{h,cpp}`, `Planner.{h,cpp}`, `PlannerBegin.cpp`, `StopCondition.{h,cpp}`, `MotionCommand.{h,cpp}` | **Modified** (ticket 005). Identifiers only; no algorithm change. |
| `source/hal/real/Motor.{h,cpp}`, `OtosSensor.{h,cpp}`, `NezhaHAL.{h,cpp}` | **Modified** (ticket 006). Identifiers only. |
| `source/hal/sim/PhysicsWorld.{h,cpp}`, `SimOdometer.{h,cpp}`, `SimSetters.{h,cpp}`, `SimCommands.cpp` (internal names) | **Modified** (ticket 007). `kSimRegistry[]` key strings unchanged. |
| `source/commands/ConfigRegistry.cpp`/`SimCommands.cpp`'s wire key strings; `TLM`/`SNAP` field tokens; `SET`/`GET`/`SIMSET`/`SIMGET` grammar | **Unaffected.** Zero wire-format change anywhere in this sprint. |
| `tests/_infra/golden_tlm_capture.json` | **Unaffected — no regeneration needed.** No TLM field or format change. |
| `tests/_infra/default_config_golden.json` | **Regenerated** (ticket 002) — field-name keys change, values do not. |
| `tests/simulation/`, `tests/_infra/sim/*.cpp` | **Modified, mechanically**, wherever a test mirrors a renamed C++/proto identifier by name (mock `RobotConfig` test doubles, assertion strings quoting an old field name, docstrings). Not a host-Python cleanup — a required consequence of keeping the suite green. |
| `host/robot_radio/` | **Unaffected.** Explicitly out of this sprint's scope (Decision 1). |
| `docs/protocol-v2.md`, `architecture.md`, `overview.md`, `kinematics-model.md`, `design/message-inventory.md` | **Modified** (ticket 008/004). Prose/table updates only. |

### Migration Concerns

- **No wire-protocol change of any kind.** Every `SET`/`GET`/`SIMSET`/
  `SIMGET`/`STREAM`/`TLM`/`SNAP` byte is identical before and after this
  sprint — verified by the Exclusion Table's per-surface audit and by the
  unmodified golden-TLM fixture requirement.
- **No `RobotConfig`/`DesiredState`/`OutputState` layout or default-value
  change.** Field renames do not change `sizeof()`, offsets, or defaults;
  `ConfigRegistry`'s `offsetof()`-based dispatch is unaffected by a field
  rename (offsets are structural, not name-based).
- **No data/schema migration.** `data/robots/*.json` is untouched
  (Exclusion Table); no persisted format changes.
- **Deployment sequencing (firmware/codegen build).** `scripts/
  gen_messages.py` must be re-run (regenerates `source/messages/*.h`)
  after ticket 004's `.proto` edits, before that ticket's `--clean` sim
  build. `scripts/gen_default_config.py` runs automatically as part of the
  normal build and picks up ticket 002's `robot_config.schema.json`
  `firmware.field` changes with no manual step. Per project knowledge
  (stale incremental builds on `/Volumes`), every ticket that touches
  `source/` requires a `--clean` sim build before its test run.
  Ticket ordering (Step 4a) already sequences codegen-producing tickets
  (002, 004) before their consumers (005, 006, 007).
- **`docs/design/message-inventory.md` regeneration (ticket 004)** must
  land in the same ticket as the `.proto` edits it documents, mirroring
  the golden-fixture-same-ticket discipline sprint 068 established for
  TLM.
- **Host/sprint-072 boundary.** This sprint deliberately leaves `host/
  robot_radio/` untouched; sprint 072 (recommended, not created by this
  planning pass) will need to re-derive its own baseline test count and
  wire-compat exclusions independently — this document's Exclusion Table
  is a reusable reference for that future planning pass but is not itself
  binding on it (072 may find additional host-side exclusions this
  planning pass, scoped to `source/`, did not need to check).

---

## Step 6: Design Rationale

### Decision 1: split the host-Python half of the issue to a recommended sprint 072; scope 071 to `source/` (firmware/sim C++) and its minimal test/doc/codegen footprint

**Context:** the team-lead's brief explicitly asks this planning pass to
decide whether the full issue fits one sprint or should split (offering
"071 firmware, 072 host" as the example split), noting the stakeholder
wants the work finished but preferring a complete, independently-green
ticket sequence. This mirrors sprint 070's own Decision 1, which split
this same issue out of 070 for being "an order-of-magnitude larger,
differently-shaped change."

**What this pass found, quantitatively:** grepping `host/`, `tests/`, and
`scripts/` directly (not estimating from the issue text) for the six
Python unit-suffix patterns found **101** unique `_mm`-suffixed, **78**
unique `_deg`-suffixed, **77** unique `_ms`-suffixed (excluding `_mms`),
**24** unique `_mms`-suffixed, plus smaller `_dps`/`_pct`/`_hz` sets —
roughly **280+ unique Python identifiers**, before counting occurrences
(`read_ms` alone has ~269 call sites, not the issue's originally-cited
121 — the issue's own count was already stale). This is **larger**, not
smaller, than the ~150-identifier C++ firmware surface this sprint covers.

**Alternatives considered:**
- *One sprint, phased tickets (firmware tickets, then host tickets), per
  the brief's "prefer a complete sequence in 071 if each ticket is
  independently green" instruction.* Technically buildable — the two
  halves are wire-decoupled (Exclusion Table), so there is no *technical*
  ordering requirement forcing them into the same sprint. But bundling
  ~280+ Python identifiers across `host/robot_radio/{robot,io,testgui,
  calibration,sensors,nav,path,kinematics,config}` and `tests/testgui/`
  into the same architecture-review surface as the firmware half would
  produce a single review pass covering two genuinely different risk
  profiles: the firmware half's risk is almost entirely "did the codegen
  chain move in lock-step" (a small number of well-understood coordinated
  edits); the host half's risk is almost entirely "did every one of ~280
  call sites, spread across a much larger and more loosely-coupled
  package tree with no `offsetof`-style structural indirection protecting
  it, get caught" — a breadth-of-execution risk, not a design risk, but a
  real one given `read_ms` alone is a *keyword argument name* at ~269 call
  sites (a missed rename there is a `TypeError`, not a silent bug, but
  still a large mechanical surface to review in one pass).
- *Two sprints, 071 = firmware, 072 = host (chosen).* Matches the brief's
  own example split verbatim. Each sprint gets its own architecture review
  and its own stakeholder-approval gate, appropriately sized to its own
  risk profile — exactly sprint 070's precedent, applied a second time to
  the same parent issue.

**Why this choice:** the brief itself names this exact split as the
example outcome, and this pass's own quantitative reading confirms the
host half is if anything the *larger* of the two — the same scope-
discipline argument that justified 070's split applies with at least
equal force here, one level down. Keeping 071 to `source/` also means its
own review surface (this document) stays focused on the wire-compatibility
question that matters *only* for the firmware half (the C-ABI/proto/JSON-
schema codegen chains) — sprint 072 will need its own, different
wire-compat analysis (e.g., are any Python identifiers *themselves* the
serialized form, as `read_ms` almost is via keyword-argument binding,
even though it is not wire/persisted?).

**Consequences:** the issue's acceptance criterion "No identifier in
`source/` or the host Python embeds a unit suffix" is only **half**
satisfied at the close of 071 — the `source/` half. The team-lead should
create sprint 072 for `host/robot_radio/` (and the pure-Python-only
identifiers in `tests/`/`scripts/` not already dragged along by 071's
mechanical test-fixture updates) once 071 closes, per Open Question 1.

### Decision 2: rename `RobotConfig`/proto/`SIMSET` field identifiers freely; exclude only the wire-key-string literals, by construction

**Context:** the issue's design point 2 asks planning to decide, per
name, whether a wire-visible identifier is renamed in lock-step or
excluded. This pass found that `ConfigRegistry.cpp`, `SimCommands.cpp`,
and `robot_config.schema.json` **already** bind every wire-key string as
an independent literal argument, never derived from the C++/JSON field
name by stringification (`CFG_FI("tw", trackwidthMm)` — two separate
tokens, not `#field` stringification of one).

**Alternatives considered:**
- *Treat every field with a registered `SET`/`GET`/`SIMSET` key as
  wire-visible and therefore excluded from rename*, per the issue's
  stated default ("Default to EXCLUDING wire/serialized names"). This
  would have excluded essentially the entire `RobotConfig` struct (nearly
  every field has a key) — defeating 6 of the 7 Config.h FIXME markers,
  which is not what the FIXME authors (the stakeholder) asked for, and not
  what the codebase's own structure requires.
- *Verify, per name, whether the wire-visible string is the field name
  itself or an independent literal, and exclude only where it genuinely
  is* (chosen). Reading the actual registry/registry-adjacent code (rather
  than assuming coupling from the issue's cautious framing) found exactly
  one field (`minWheelMms`) and one schema surface (the per-robot JSON,
  Decision 6) where the two are genuinely the same string.

**Why this choice:** the issue's "default to excluding" instruction is a
safety default for the *unverified* case; this planning pass did the
verification the issue anticipated ("For EACH wire-visible name, decide")
and found the codebase's own registry pattern already provides the
decoupling the issue was worried might not exist. Applying the cautious
default anyway, after confirming it is unnecessary, would leave 6 FIXME
markers unresolved for no risk-reduction benefit.

**Consequences:** ticket 002's implementer must still grep carefully
around `minWheelMms` (the one field/key spelling collision) and must
never touch a `CFG_*` macro's *first* argument — a discipline this
document's Exclusion Table and Step 5 "What Changed" call out explicitly
so it is not left to tribal knowledge.

### Decision 3: treat a `RobotConfig` field rename as one coordinated, same-ticket edit across `Config.h` → `DefaultConfig.cpp` → `ConfigRegistry.cpp` → `robot_config.schema.json`, not four independent tickets

**Context:** these four files form a closed, internal (never externally
wire-transmitted) codegen/consistency chain: the schema's `firmware.field`
value must name a real `RobotConfig` field, `DefaultConfig.cpp`'s
generator script reads that same field name via `ov('<field>', ...)`, and
`ConfigRegistry.cpp`'s macro references it via `offsetof`. A field rename
that touches only one of the four leaves the chain broken at build time
(a `-Wundeclared` compile error, not a silent bug — but still a broken
intermediate state).

**Alternatives considered:**
- *Split into per-file tickets (one for `Config.h`, one for the schema,
  one for the registry).* Would produce three tickets that cannot each
  independently pass the suite (a `Config.h`-only rename fails to compile
  against `DefaultConfig.cpp`'s still-old field references) — directly
  violating this sprint's own hard contract that every ticket ends green.
- *One ticket per field-rename chain* (chosen) — ticket 002 renames the
  full FIXME + peer set together, since they are all instances of the
  identical four-file edit shape and share no reason to be split further
  (unlike, say, `DesiredState` vs. `RobotConfig`, which are genuinely
  different structs with different consumer sets — Decision 4 explains
  why *those* stay separate).

**Why this choice:** "coherent implementation ticket ... completable in
one focused session" (per this project's ticket-authoring convention) is
about the *shape* of the work, not the *file count* — a dozen fields all
moving through the identical four-file mechanical edit is one focused
session; a `Config.h` field and a `DesiredState.h` field moving through
two entirely different consumer sets are two.

**Consequences:** ticket 002 is the largest single ticket in this sprint
by file-touch count (Config.h + DefaultConfig.cpp + ConfigRegistry.cpp +
schema.json + ~7 direct-consumer files + 2-3 test files) — flagged for the
ticketing pass to confirm it still fits "one focused session"; if not, the
ticketing pass may split ticket 002 by Config.h section (e.g., "OTOS/
rotation calibration fields" vs. "timing/lag fields") while preserving the
four-file-per-field coordination rule within each split.

### Decision 4: `DesiredState`/`OutputState` get their own ticket (003), separate from `RobotConfig` (002)

**Context:** both structs carry unit-suffixed fields and both have a
`FIXME` marker, but they are structurally unrelated: `RobotConfig` is a
SET/GET-configurable calibration blob read by nearly every subsystem;
`DesiredState`/`OutputState` are per-tick commanded/actuator state written
by the motion-command layer and read by the control loop, with a small,
disjoint consumer set (5 references outside their own header, per this
pass's grep).

**Alternatives considered:**
- *Fold into ticket 002* (larger, single "state structs" ticket). Would
  mix two consumer sets with zero file overlap, growing ticket 002's
  already-largest footprint for no cohesion benefit — a "coincidence of
  both having FIXME markers" is not a shared responsibility.
- *Separate ticket (chosen).*

**Why this choice:** matches Step 2's Responsibilities table — cohesion
(shared reason to change) determines ticket boundaries, not superficial
similarity (both being "state structs with FIXMEs").

**Consequences:** none beyond the ticket split itself; both tickets can
run in either order relative to each other (both depend only on 001)
though this document sequences 002 first since it is the larger,
higher-attention change.

### Decision 5: derived-unit calibration names (`mmPerDeg*`) get a full semantic rename (`wheelTravelCalib*`), not a partial suffix strip

**Context:** the issue's design point 4 calls this out explicitly:
stripping only the trailing suffix from `mmPerDegL` (removing `L`? or
`Deg`?) does not produce a valid unit-free name — `mmPerDeg` itself
embeds `mm` (still a forbidden token) regardless of which trailing letter
is stripped.

**Alternatives considered:**
- *Rename to `travelPerRotL/R`* — better than `mmPerDegL` but `PerRot`
  is vague about what's being measured per rotation of what.
- *Rename to `wheelTravelCalibL/R` (chosen)* — names the quantity as what
  it functionally is (a per-wheel calibration factor converting motor
  rotation to linear travel), consistent with how the codebase already
  names other calibration factors (`otosLinearScale`, `otosAngularScale`
  — scale/calib nouns, not unit ratios, are the established local idiom).

**Why this choice:** matches the issue's own instruction ("rename to what
the quantity *is*") and the codebase's own naming idiom for calibration
factors, rather than inventing a new one.

**Consequences:** the proto-generated `mm_per_deg_l`/`mm_per_deg_r`/
`mm_per_deg` fields (ticket 004) are renamed to `travel_calib_l`/
`travel_calib_r`/`travel_calib` to stay consistent with the C++ side they
project to/from.

### Decision 6: exclude the entire per-robot JSON config schema (and its 1:1-mirroring pydantic model) from this sprint, rather than lock-stepping it

**Context:** `data/robots/*.json` keys and `host/robot_radio/config/
robot_config.py`'s pydantic field names are the same string (no alias
layer) — a genuine serialization coupling, unlike the `RobotConfig`/`SET`
key relationship (Decision 2), where the coupling turned out not to
exist.

**Alternatives considered:**
- *Lock-step rename*: touch every `data/robots/*.json` file (`tovez.json`,
  `togov.json`, `tovez copy.json`), the schema, `robot_config.py`, and
  `gen_default_config.py`'s `_get(...)` string-literal lookups, together.
  Technically closed and low-behavioral-risk (it's a config format only
  this repo's own tooling reads), but touches a *data* file format for a
  sprint whose entire premise is "no behavioral change" to *code* — a
  scope expansion the issue's own framing ("identifier names," not
  "config file schemas") does not require, and one that would need its
  own careful audit of any out-of-repo tooling or saved robot configs
  this project's `docs/` doesn't mention (a risk this planning pass cannot
  fully rule out from static reading alone).
- *Exclude (chosen)*, consistent with the issue's own default and this
  sprint's `source/`-scoped boundary (Decision 1).

**Why this choice:** the marginal benefit (a few dozen JSON/pydantic
identifiers made unit-free) is small and purely cosmetic, while the risk
surface (every saved robot config file, any external tooling reading
`data/robots/*.json` directly) is the one part of this sprint's search
space this planning pass could not fully bound by reading the repo alone.
Consistent with the issue's own stated default.

**Consequences:** `data/robots/*.json`'s unit-suffixed keys remain
un-renamed indefinitely unless a future sprint explicitly takes this on
as its own scoped, reviewed decision — flagged as Open Question 3.

---

## Step 7: Open Questions

1. **Sprint 072 (host-Python rename) should be created next**, per
   Decision 1. This sprint recommends the team-lead create it (roadmap
   mode) once 071 closes, scoping it to `host/robot_radio/` plus any pure
   host-Python identifiers in `tests/`/`scripts/` not already touched
   mechanically by 071. The ~280+ unique identifier count found this pass
   (vs. ~150 in `source/`) suggests 072 may itself warrant a similar
   ticket-count sequence, split by host package (`robot/`, `io/`,
   `testgui/`, `calibration/`, `sensors/`, `nav/`, `path/`, `kinematics/`).
2. **Ticket 002's size** (the `RobotConfig` four-file coordination across
   ~15 fields) is flagged in Decision 3 as the largest ticket in this
   sprint; the ticketing pass should confirm it still fits "one focused
   session" or split it by Config.h section while preserving the
   four-file-per-field rule.
3. **The per-robot JSON config schema exclusion (Decision 6)** is a
   deliberate scope cut, not a permanent architectural position — a future
   sprint could revisit lock-stepping `data/robots/*.json` if the
   stakeholder wants full codebase unit-suffix elimination including data
   files, not just source identifiers. Not blocking.
4. **The ambiguity-resolution rule** (Comment Convention section, and the
   ↑issue's design point 3) is stated as a rule for ticket implementers to
   apply case-by-case; this planning pass did not find a confirmed
   same-scope name collision in the Config.h/DesiredState.h FIXME set
   itself, but flags `Odometry`/`Motor` (tickets 005/006) as the most
   likely place a raw-ticks vs. mm-scaled sibling pair could collide after
   a naive strip. Not blocking — a normal implementation-time judgment
   call, module-level planning intentionally does not pre-resolve it.
5. **`protos/planner.proto`/`drivetrain.proto`/`motor.proto`'s exact
   current field list** was read at a summary level (grep) rather than
   the full file text for every message; ticket 004's implementer should
   re-confirm the complete field list against the live `.proto` source
   before editing, in case a field this document didn't enumerate also
   carries a unit suffix.

---

## Architecture Self-Review

**Consistency.** The Sprint Changes Summary's claims (8 FIXMEs resolved,
host split to 072, wire-compat-safe-by-construction) match Step 5's
per-ticket "What Changed" one-to-one. The Wire-Compatibility Exclusion
Table's dispositions match every claim made in Step 1's codebase-alignment
reading and Decision 2's rationale — no exclusion is asserted without a
corresponding "why" row, and no rename is proposed for a name the
Exclusion Table lists as excluded. Design Rationale Decisions 1–6 each
trace to a specific claim made earlier in the document (Decision 1 → the
Scope Decision paragraph and Step 4a's ticket graph; Decision 2 → the
Exclusion Table's summary line; Decision 5 → SUC-005 and ticket
006's "What Changed" `mmPerDeg` line).

**Codebase Alignment.** Every structural claim was checked against the
actual current source, not the issue text alone: the 8 `FIXME` markers
were re-grepped fresh (finding 2 of the issue's originally-cited markers
already resolved by sprint 070); `ConfigRegistry.cpp`'s full `CFG_*` table
(all ~40 rows) and `robot_config.schema.json`'s full `firmware`/`set_key`
block (all 30 rows) were read in full, not sampled; `gen_default_config.py`
and `gen_messages.py` were read in full to confirm their literal-mapping
dependency on field names; `robot_config.py` was read in full to confirm
the no-alias pydantic coupling that justifies Decision 6;
`PhysicalStateEstimate.h`'s current (070-003-modified) signatures were
read directly, not assumed from the 070 architecture doc's prose; the
full test suite was run fresh (2620 passed, 51.89s) rather than trusting
the 070 document's stale 2612 figure.

**Design Quality.**
- *Cohesion:* every ticket in Step 3's table passes the one-sentence,
  no-"and" test — each names one struct/subsystem family and one reason
  to change (a unit-suffix rename within that family's own file set).
- *Coupling:* the ticket dependency graph (4a) has one root (001) and one
  sink (008); the widest fan-out (002, 4 dependents) reflects
  `RobotConfig`'s pre-existing, unrelated-to-this-sprint fan-in — not a
  new coupling this sprint introduces. No ticket's files overlap with a
  sibling ticket's files at the same graph depth (002 vs. 003; 006 vs.
  007) — verified by this pass's own file-path reading, not asserted.
- *Boundaries:* the Exclusion Table is itself a boundary specification —
  it draws a hard line between "C++ identifier" (mutable this sprint) and
  "wire/serialized string" (immutable this sprint) with zero ambiguous
  rows left unresolved.
- *Dependency direction:* unchanged from the existing architecture
  (`app/`/`commands/` → `subsystems/`/`superstructure/` →
  `state/`/`control/` → `hal/`) — this sprint renames symbols within that
  existing direction, never adds or reverses an edge.

**Anti-Pattern Detection.**
- *God component:* none — no module in Step 3 grows; several (
  `PhysicalStateEstimate`, `SimCommands`) only have their internal names
  cleaned up, no new methods or responsibilities added.
- *Shotgun surgery:* ticket 002 (Decision 3) is the one ticket that
  touches many files for a single underlying reason (the four-file
  codegen-chain coordination, repeated per field) — judged, per the same
  reasoning sprint 070's Ticket 003 self-review applied, as **justified**
  multi-file change (every touch traces to the one interface/chain being
  renamed, not scattered unrelated edits), not true shotgun surgery.
- *Feature envy:* none — no module reaches into another's private state;
  this sprint's edits are entirely name substitutions at existing,
  already-public call sites.
- *Circular dependencies:* none in either diagram (checked explicitly per
  diagram, per node).
- *Leaky abstraction:* none newly introduced — if anything, the
  `RobotConfig`/`SIMSET` registries' wire-key/field-name decoupling
  (Decision 2) is an *existing* clean abstraction this sprint's plan
  explicitly relies on and confirms, rather than erodes.
- *Speculative generality:* none — every rename targets a name that
  exists today with a unit suffix; nothing is renamed "in case" a future
  need arises, and no new abstraction layer is introduced anywhere.

**Risks.**
- *Data migration:* none — no persisted format changes (Exclusion Table,
  Decision 6).
- *Breaking changes:* none on any wire surface (Exclusion Table, verified
  per-name). The only "breaking" surface is internal C++/proto source
  compilation, self-contained within each ticket's own green-suite
  requirement.
- *Performance:* none — pure renames, zero logic change, zero new
  indirection.
- *Security:* none — internal firmware/sim refactor, no new external
  surface.
- *Deployment sequencing:* covered in Migration Concerns (`--clean` sim
  build after each `source/`/`.proto`-touching ticket; `gen_messages.py`
  re-run after ticket 004's proto edits).

**Verdict: APPROVE.**

No structural issues (no circular dependencies, no god components, no
inconsistency between this document's summary and its body). The one
substantial scope decision (Decision 1, splitting host Python to a
recommended sprint 072) is explicitly justified with fresh quantitative
evidence (this pass's own grep counts, not the issue's stale estimate)
and mirrors an already-accepted project precedent (070's own split of this
same issue). The one deliberate, larger-than-usual ticket (002, Decision 3)
is justified by the four-file codegen-chain coordination it must preserve
in one edit to stay green, and is flagged in Open Questions for the
ticketing pass to size-check. Proceed to ticketing.
