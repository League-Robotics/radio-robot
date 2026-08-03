---
status: in-progress
sprint: '132'
tickets:
- 132-001
- 132-002
- 132-003
- 132-004
- 132-005
- 132-006
- 132-007
- 132-008
- 132-009
- 132-010
- 132-011
- 132-012
- 132-013
- 132-014
- 132-015
- 132-016
- 132-017
- 132-018
- 132-019
- 132-020
---

# The configuration object

## Description

Build one configuration object, owned by the configuration subsystem, fed from
baked values or from the wire, which then sends config out to each subsystem.

Stakeholder direction, 2026-08-03:

> The configuration subsystem holds the single configuration object, which it
> gets from baked values or from the wire, and it sends it out to each
> subsystem.

One struct, one owner, two sources, one fan-out:

```
data/robots/tovez.json
        │
        ├── baked at build time ──┐
        │                         ▼
        │                  ┌─────────────────┐
        └── sent at run ──►│  Configurator   │
                           │  Config::Robot  │◄── the one object
                           └────────┬────────┘
                                    │ install()   (the converters)
              ┌──────────┬──────────┼──────────┬──────────┐
              ▼          ▼          ▼          ▼          ▼
            Drive     Planner    Motors      OTOS     RobotLoop
```

`GetConfig` reads the object straight back out — no per-subsystem getters, no
reassembling state from scattered private members.

**This issue supersedes and absorbs [[simplify-configuration-to-a-struct]]**,
which carries the same direction at one less level of detail; that issue should
be closed when this one is planned. It also supersedes
[[per-wheel-drive-calibration-as-runtime-configuration]], whose whole feature
becomes fields of the drive group rather than new patch fields.

## Cause

Configuration is **defined twice and owned twice**, and every piece of
machinery that made the previous design unintelligible — sparse patches,
presence flags, string wire keys, patch-kind routing, read-merge-write — is
glue between the two definitions.

- **Defined four times, not twice.** The `.proto` generates *only* the wire
  messages. Verified 2026-08-03:

  | definition | source | generated from `.proto`? |
  |---|---|---|
  | wire messages | `.proto` → `gen_messages.py` | — |
  | C++ boot structs | robot JSON → `gen_boot_config.py` | **no** |
  | `data/robots/robot_config.schema.json` | **hand-maintained** | **no** |
  | `robot_config.py` pydantic model | **hand-written** | **no** |

  `check_config_sync.py` plus a 58-entry allowlist exists purely to notice the
  resulting drift. **Adding one config field touches 16 places, 9 hand-edited,
  across 5 languages** (`7256e22f` added 4 fields and touched 29 files).

  Worse, the JSON schema's own `$comment` declares itself "the single source of
  truth that maps a JSON value to a firmware RobotConfig field" via a custom
  `firmware` keyword — and points at `source/robot/DefaultConfig.cpp`,
  `scripts/gen_default_config.py` and `source/robot/ConfigRegistry.cpp`, **all
  of which were deleted in the sprint-077 rebuild.** It is a fourth definition
  that documents a pipeline that no longer exists.
- **Owned twice:** `RobotGraph::Resolved` (`boot_wiring.h:185-195`) holds the
  baked values and is `private` with no accessor, never read after the
  constructor body ends; `Configurator::persistedTuning_` (`configurator.h:77`)
  holds the wire values. Neither is authoritative, which is why nothing can
  answer "what is this robot running?"

**What the missing read-back already cost** (from the sprint-130 post-mortem,
which ranked this "the single highest-leverage missing capability"): twice in
one sprint a routine measurement produced a confident wrong answer. A duty sweep
computed its x-axis from `tovez.json`'s `duty_per_speed` while the firmware had
switched to a baked constant (`Drive::kDutyPerSpeed`) — a **~1.6x error,
undetectable from the host**. It reported a 28% L/R gain mismatch and a 0.24
breakaway; the truth was 1.9% and ~0.10, and it produced a recommendation to
inspect a wheel mechanically that had nothing wrong with it. Two independent
sweeps hit it separately.

There are **three layers of tuning truth and none is observable**: the robot
JSON baked at build time; firmware constants that deliberately ignore the JSON;
and flash-persisted `pid.*` that silently overrides the JSON at boot. The third
is sharpest — a robot can boot tuning nobody in the room knows is there, and
there is no way to look.

Measured drift, 2026-08-03: the host pydantic model has 36 `control` fields
where the JSON has 53 and silently drops 18 — including
`control.output_deadband` and `control.reversal_dwell_ms`, which the firmware
generator *requires* and refuses to build without. 17 `control` keys are read by
nothing. `ShaperBootConfig` is generated, baked, and read by no code.

Full diagnosis: [[simplify-configuration-to-a-struct]].

## Proposed fix

### The object holds RAW file values

`Config::Robot` mirrors the robot JSON exactly — same fields, same units. It
does **not** hold derived quantities: track width is the configured
`trackwidth`, not the scrub-corrected `trackwidth / rotational_slip` that Drive
and Odometry actually use; rotation offsets are degrees, not the radians
`RobotLoop` stores.

That is what makes read-back meaningful — `config()` serialized can be **diffed
directly against the file**. Derivation stays in the converters where it already
lives, so all 14 install-time transformations are untouched.

### The object

Generated from the `.proto` schema, never hand-edited. Grouped by **consumer** —
what the converters take, and what fits one wire envelope each.

```cpp
namespace Config {

struct Robot {
  // --- host-only: never sent to a robot ---
  Identity     identity;      // name, uid, model, drivetrain type
  Connection   connection;    // ports, addresses
  Vision       vision;        // tag id, tag offsets

  // --- robot config: one ConfigTarget / wire message / converter each ---
  Geometry     geometry;      // trackwidth, wheel diameter, axle+odometry offsets
  Motors       motors;        // per port: travel calib, fwd sign, deadband, dwell, slew
  Drive        drive;         // dutyPerSpeed L/R, crawl pulse, 8x gain/intercept
  WheelControl wheelControl;  // Stage B gains, Stage C bounds, deficit policy
  Planner      planner;       // ceilings, plant, landing, tracking
  Otos         otos;          // scales + lever-arm offsets
  Estimator    estimator;     // fusion weights
};

}  // namespace Config
```

Of the file's 13 sections the firmware reads only about six; `connection`,
`vision`, `wheels`, `encoders`, `gripper`, `peripherals` and — despite its name
— the entire `drive` section are read by no firmware code. Those stay in the
file under the one-file rule but never cross the wire.

Sizes measured against the 240 B envelope: largest group is 16 fields / ~80 B.
A single all-in-one message does **not** fit (118 fields, ~590 B) and must not
be attempted — there is no fragmentation anywhere in the wire layer.

The JSON is reshaped to match (stakeholder-approved 2026-08-03), retiring
today's `control` — a 53-field dumping ground feeding five destinations. This
breaks all three existing robot JSONs and needs a one-time migration script plus
a re-bake.

### The interface

```cpp
class Configurator {
 public:
  const Config::Robot& config() const;   // read-back: one call, whole truth

  void    loadBaked();                                   // generated defaults
  ErrCode applyGroup(ConfigTarget, const uint8_t* wire, size_t len);
  ErrCode applyField(ConfigTarget, uint16_t fieldNumber, float value);

  void install();               // send everything
  void install(ConfigTarget);   // send just the group that changed
};
```

Every flow is two calls: boot is `loadBaked()` → `install()`; a push is
`applyGroup(...)` → `install(target)`; read-back is `config()`.

**Deleted by this:** presence flags on config, string wire keys, `PatchKind`
routing, the merge accumulator, and read-merge-write. `Configurator` stops
reading values back out of `Drive`/`Planner` to apply partial updates
(`configurator.cpp:65-77`, `:141-147`) — it already holds every value.

### How a subsystem gets configured while the robot is running

**Subsystems take the whole object and pull out what they want** (stakeholder
preference, 2026-08-03): *"I don't need to have something outside of a subsystem
know how it has to be configured."* Every configurable subsystem exposes one
method:

```cpp
void configure(const Config::Robot& config);
```

#### 1. The message arrives and is routed

Unchanged from today — `robot_loop.cpp` already routes CONFIG to the
configurator and acks whatever it returns:

```cpp
case msg::CommandEnvelope::CmdKind::CONFIG:
  tlm_.ack(cmd.env.corr_id, configurator_.apply(cmd.env));
```

#### 2. The Configurator updates its object, then hands it out whole

```cpp
ErrCode Configurator::apply(const msg::CommandEnvelope& env) {
  const ConfigTarget target = env.config.target;

  // Boot-only groups are refused, not silently ignored. See the
  // re-appliability table below -- ~14 values have no setter at all.
  if (!isLiveConfigurable(target)) return ErrCode::ERR_NOT_LIVE;

  // Decode straight into the object. No patch, no presence flags, no merge:
  // a group message carries the whole group.
  const ErrCode decoded = decodeGroup(target, env.config.body, &config_);
  if (decoded != ErrCode::OK) return decoded;   // ERR_RANGE / ERR_BADARG

  return install(target);
}

ErrCode Configurator::install(ConfigTarget target) {
  switch (target) {
    case ConfigTarget::DRIVE:
    case ConfigTarget::WHEEL_CONTROL:  drive_.configure(config_);   break;
    case ConfigTarget::PLANNER:        planner_.configure(config_); break;
    case ConfigTarget::GEOMETRY:       robotLoop_.configure(config_); break;
    case ConfigTarget::MOTORS:
      // The one guarded case: refuses while the robot is moving.
      if (!motorL_.configure(config_) || !motorR_.configure(config_)) {
        return ErrCode::ERR_BUSY;    // caller retries at rest
      }
      break;
    default: return ErrCode::ERR_NOT_LIVE;
  }
  return ErrCode::OK;
}
```

`install()` with no argument is the same switch over every target — used once at
boot.

#### 3. The subsystem pulls what it needs

Nothing outside `Drive` knows what `Drive` reads:

```cpp
void Drive::configure(const Config::Robot& config) {
  // Stage A -- per-wheel affine map, reshaped into [wheel][accel|decel]
  corrGain_[0][0]      = config.drive.wheelGainLeftAccel;
  corrIntercept_[0][0] = config.drive.wheelInterceptLeftAccel;
  // … the other six …

  // Stage B / Stage C
  gains_  = { config.wheelControl.pidKp, config.wheelControl.pidKi,
              config.wheelControl.pidIMax, config.wheelControl.pidKaff,
              config.wheelControl.pidMax };
  bounds_ = { config.wheelControl.vMin, config.wheelControl.biasMax, … };

  crawlPulse_ = config.drive.crawlPulse;

  // Derived values are METHODS on the object, not fields -- see below.
  trackWidth_ = config.effectiveTrackWidth();
}
```

#### Updating one value: `(target, field number, value)`

The wire carries **numbers, not strings** — the protobuf field number, which is
already a stable identifier the schema assigns:

```proto
message SetConfigField {
  ConfigTarget target = 1;   // which group
  uint32       field  = 2;   // protobuf field number WITHIN that group
  float        value  = 3;
}
```

~11 bytes on the wire against ~25 for a string key like
`"wheel_gain_left_decel"`, and no hand-maintained name vocabulary to drift.

**Field numbers are unique only within a message**, so the address must be the
pair — `MotorConfigPatch.kp` and `EstimatorConfigPatch.staleness_ms` are both
field 3 today. `ConfigTarget` supplies the stable group id. (Do **not** use
`kMessageTables[]`'s array index: its ordering is BFS-derived from schema
reachability and changes when the schema does.)

Firmware side is nearly free, because the wire decoder is **already** a
schema-generic, table-driven, offset-writing walker (`wire.cpp:546-597`): it
looks a field up by number, writes at `base + fd->offset`, and validates bounds
inline. The setter is that loop minus tag decoding:

```cpp
ErrCode Configurator::applyField(ConfigTarget target, uint16_t field, float value) {
  const FieldDesc* fd = findField(tableFor(target), field);   // same lookup decode does
  if (fd == nullptr)              return ErrCode::ERR_BADARG;  // unknown field
  if (!std::isfinite(value))      return ErrCode::ERR_BADARG;  // NaN defeats bounds checks
  if (!validateBounds(*fd, &value)) return ErrCode::ERR_RANGE; // reuses wire.cpp:316-331

  writeScalar(*fd, groupBase(target, &config_), value);        // base + fd->offset
  return install(target);                                      // fan out, same as a group push
}
```

The `isfinite` check is not optional: `validateBounds()` compares with `<` and
`>`, both of which are false for NaN, so a NaN would pass every bound and reach
the object.

This must be **emitted by the generator** — `FieldDesc`, the field tables, and
`validateBounds()` all live in an anonymous namespace in `wire.cpp` and cannot
be reached from outside. That is a change to the generator's fixed engine text,
not a per-message change.

Host side gets it free from the real protobuf descriptors, so **a human still
types a name and the wire still carries a number**:

```python
field = ConfigDrive.DESCRIPTOR.fields_by_name["wheel_gain_left_decel"].number
proto.set_config_field(ConfigTarget.DRIVE, field, 1.043)
```

This replaces the string-key surface entirely, and structurally kills the
`pid.kff → kaff` class of bug: the number comes from the schema rather than
from a hand-written mapping table that can drift from what it names.

#### Derived values are methods on the object, not stored fields

This is what makes "everyone takes the whole object" work without either
duplicating derivations across consumers or polluting the object with values
that exist in no config file. The object stores **raw**; it *computes*
**derived**:

```cpp
struct Robot {
  Geometry geometry;   // trackwidth, rotational_slip — raw, as in the file
  // …

  // Derived: computed, never stored. One definition of each derivation, so
  // every consumer necessarily gets the same answer, and read-back still
  // serializes only the raw fields.
  float effectiveTrackWidth() const {          // [mm]
    return geometry.rotationalSlip > 0.0f
         ? geometry.trackwidth / geometry.rotationalSlip
         : geometry.trackwidth;                // 0 == uncalibrated sentinel
  }
  float rotationOffsetPos() const;             // [rad] from configured degrees
};
```

Today that same derivation is computed once in `boot_calibration.cpp:25-29` and
fanned into four different subsystems' private members, where nothing can check
they agree. As a method it cannot drift.

#### Why push, not pull

The third option — subsystems asking the configurator for updates — was
considered and rejected. A pull means every subsystem holds a pointer to
configuration and re-reads it on some cadence, which puts config on the hot path,
makes the moment a value takes effect nondeterministic, and reintroduces
mutable shared state. Push keeps configuration an *event*: it happens at a known
instant, on the caller's thread, and the return value says whether it landed.

#### The one place this collides with layering

`devices/` may include neither `messages/` nor `config/` — which is why device
leaves take narrow structs (`Devices::MotorConfig`) today. Two ways to resolve
it, and the choice should be explicit:

- **Give `Config::Robot` a `<cstdint>`-only dependency floor** and relax the
  rule for that one header. This is precedented: `types/robot_state.h` is
  includable from the sibling `src/motion` tree for exactly this reason. Then
  device leaves take the whole object too, and the rule holds everywhere.
- **Keep device leaves on narrow structs.** `Devices::NezhaMotor` stays reusable
  hardware code that knows nothing about a robot config, and one small adapter
  at the app boundary translates. Costs one place that "knows how a device is
  configured" — the thing the stakeholder preference is trying to eliminate.

Recommendation: the first. A POD config header with no dependencies is not the
coupling the layering rule exists to prevent, and it makes the rule uniform
rather than carved out.

### The boundary that shapes the design: boot-only vs live

**"Send config to subsystems" cannot cover everything, and pretending otherwise
is how config silently fails to take effect.** An audit of every install entry
point found:

| | count | examples |
|---|---:|---|
| **safely re-appliable at runtime** | 8 setters | `setWheelCorrection`, `setCrawlPulse`, `setControlGains`, `setAdaptationBounds`, `applyShaperLimits`, `applyTravelCalib`, `setRotationCalibration` |
| **guarded — may refuse** | 1 | `NezhaMotor::reconfigure()` returns `[[nodiscard]] bool` and refuses while the robot is moving |
| **no setter exists at all** | ~14 values | `trackWidth` (in Drive, Odometry *and* Planner), Planner's `vMax`/`omegaMax`/`controlPeriod`/`actuationDelay`/`landing.*`/`headingHoldGain`, the OTOS lever arm and scales, `ColorConfig`, `LineConfig` |

So the object is **complete before construction**, and the fan-out has two modes:

- **at boot — total.** Subsystems are constructed *from* the object; constructor
  injection stays exactly as it is for the un-settable values.
- **at runtime — partial.** `install(target)` re-pushes only groups whose values
  have safe setters.

**Every `ConfigTarget` declares its re-appliability.** A wire push to a
boot-only group returns a clear error — "takes effect on reboot" — rather than
acking OK and doing nothing. That failure mode is not hypothetical; it is the
current behaviour twice over (traps 1 and 2 below).

### Traps — each is a live landmine for this design

1. **Persisted OTOS tuning is silently discarded today.**
   `loadPersistedTuning()` runs at `main.cpp:166`, *before* `boot()` at `:171`.
   Every `RealOtos` setter is a no-op until `begin()` sets `initialized_`, and
   `begin()` then overwrites the scalars with baked values anyway. Existing bug,
   and exactly the class of silent no-op this design must make impossible.
2. **Estimator config acks OK and lands nowhere** (`configurator.cpp:58-60`).
3. **OTOS scale is two domains with one name.** `begin()` converts the config
   *multiplier* through `scaleToRegister()` (`otos.cpp:45-46, 275`) before
   calling `setLinearScalar()`, which takes the **raw int8 −127..127 register**
   value; the wire path passes the patch value straight through. A config object
   holding `linearScale = 1.0` pushed through `setLinearScalar()` installs a
   1-LSB scalar, not unity. Reconcile before any whole-group OTOS push.
4. **`setDutyPerSpeed` doubles as the actuation gate** — it sets `calibrated_`,
   and an uncalibrated `Drive` writes *no duty at all* (`drive.h:208`, `:300-302`).
   A whole-group push whose drive section is zero-filled silently stops the
   robot. Reject incomplete group pushes.
5. **Limit changes do not re-clamp existing state.** `setControlGains` does not
   re-clamp the running PID integrator; `setAdaptationBounds` does not re-clamp
   the learned bias. Lowering either at runtime leaves out-of-bounds state live.
6. **`kEncodeScratchCap = 220`** (`wire.cpp:684`) is an unasserted ceiling; a
   nested message over it makes `encode()` return 0 — the frame is silently
   never sent, at runtime, with a clean compile.

### Where the code moves

- **`Configurator` owns the object and dispatches; it performs no
  transformation.** Derivations stay in the free-function converters, keeping
  the class small and preserving the rule that `config/` may depend only on
  `messages/` and `devices/` may include neither.
- **The converter layer gets consolidated.** It is currently split three ways:
  `boot_calibration.cpp` (7 functions), plus port selection and the OtosConfig
  remap stranded in `boot_wiring.cpp:14-15, 17-26`, plus `scaleToRegister()`
  inside `otos.cpp:275`.
- **`RobotGraph::Resolved` disappears**, replaced by the one object;
  `boot_wiring.cpp:102-105`'s install calls become `Configurator::install()`.
- **`BootOverrides` is subsumed.** It exists so the sim can diverge on exactly
  the values that have no setter (`trackWidth`, `otosConfig`,
  `controlPeriod`/`actuationDelay`). With one object built before construction,
  a divergence is just a different object rather than a pointer bag. The sim's
  seven enumerated divergences must each survive that translation.

### Sequencing

Each step is independently useful; stopping after any leaves the tree better.

1. **One schema — `.proto` generates everything.** Today it generates only the
   wire messages; after this step it also generates the C++ config structs
   (replacing `gen_boot_config.py`'s hand-declared `boot_config.h`), the host
   pydantic model (replacing hand-written `robot_config.py`), and
   `robot_config.schema.json` (replacing the hand-maintained file whose
   `firmware` keyword points at deleted generators). Build order becomes
   `.proto` → everything, one pass. No behaviour change. Delete
   `check_config_sync.py` and its allowlist — with one definition there is
   nothing left to keep in sync. **This step alone takes a new config field
   from 16 touchpoints to 2** (edit the proto, edit the robot JSON).
2. **`Config::Robot` + `Configurator` ownership.** Build the object, move the
   install calls into it, delete `Resolved`. Boot behaviour identical.
3. **Whole-group set/get over the wire**, with per-target re-appliability
   declared and boot-only groups rejected loudly. Read-back lands here; so does
   per-wheel calibration, as fields of the drive group.
4. **Generic single-value setter** addressed by `(target, field_number)`,
   emitted by the generator on top of the already-table-driven decoder
   (`wire.cpp:546-597` is already a schema-generic, offset-writing walker).
5. **Retire the patch surface**; migrate OTOS calibration and TestGUI.
6. **Cleanups the audit surfaced**: the pre-`begin()` OTOS ordering bug, the
   `scaleToRegister` domain mismatch, `extra='forbid'` on the host model, the 17
   unread `control` keys, dead `ShaperBootConfig`, dead
   `CONFIG_PLANNER`/`CONFIG_WATCHDOG`, and the missing
   `output_deadband`/`reversal_dwell_ms` in the host model.

## Verification

- **Read-back equals the file.** `config()` serialized diffs clean against the
  robot JSON it was baked from. This is the property that makes the object worth
  having, so it is the headline test.
- **Bake/push parity.** Build an image from a robot JSON, push the same JSON to
  a robot running a *different* baked config, confirm identical behaviour.
- **No silent no-ops.** Push to a boot-only target and assert a *rejection*, not
  an OK. Regression-test trap 1 specifically: persisted OTOS tuning must either
  land or report, never be discarded.
- **Parity guard, generated.** Reuse the working pattern —
  `plannerStructSizes()`/`plannerLimitsOffsets()`
  (`src/motion/planner/capi.cpp:69,93`) export sizes and per-field offsets, and
  `planner_harness.py:207-212` walks them to prove the mirror has not drifted.
  Generate the same for `Config::Robot`, so "one definition" is enforced rather
  than asserted.
- **Composition-root parity.** `composition_root_parity_harness.cpp` must still
  pass, with each of the sim's seven divergences preserved or explicitly retired.
- **Add the missing `static_assert` for `kEncodeScratchCap`** and prove it fires.
- **Hardware, on the stand** (`tovez`, addressed by UID):
  `src/tests/bench/velocity_profile_gate.py` before/after, L/R plateau gap
  closing from its measured 11.1 points, from a cold boot.
- **Build for ARM** — several constraints here are `HOST_BUILD`-invisible.
- Sim baseline is 484 passed / 2 known failures
  (`test_clock_sync_activation.py`, `test_fake_transport.py`); establish it
  first or "no regressions" is unprovable.

## Related

- [[simplify-configuration-to-a-struct]] — **superseded by this issue**; close
  it when this is planned. Holds the fuller diagnosis of the two-definitions
  problem.
- [[per-wheel-drive-calibration-as-runtime-configuration]] — **superseded**; its
  feature becomes fields of the drive group. Sprint 132 was planned against that
  older design and sits at `stakeholder-review` with zero tickets created, so
  nothing is wasted; it should be closed and re-planned against this issue.
- [[configuration-discipline-one-file-authors-every-value]] — the stakeholder
  rule this design enforces structurally instead of by lint. Its 27-field work
  list collapses once one schema drives both paths. Note the production/
  development split recorded there: production boot comes from the file;
  ad-hoc single-value pushes are expected during development, which is what
  `applyField()` serves.
- [[A-no-firmware-to-host-config-readback]] — satisfied by `config()` /
  `GetConfig` in step 3.
- [[B-one-owner-per-constant-speed-floor-and-duty-per-speed]] — the
  multiple-owners catalogue; one definition is the structural fix.
- [[B-observability-contract-is-inert-as-shipped]] — config that acks OK and
  lands nowhere is the same disease as traps 1 and 2.
