---
status: done
---

# Simplify configuration to a struct

> **CLOSED AS SUPERSEDED, 2026-08-03 — not completed.** No code was written
> against this issue. It is fully absorbed by
> [[the-configuration-object]], which carries the same direction at the next
> level of detail (the concrete `Config::Robot` shape, the `Configurator`
> interface, and the boot-only-vs-live boundary that an install-surface audit
> later established). Read that issue instead; this one is kept only for the
> diagnosis below, which it cites.

## Description

Configuration should be a struct that the file, the boot path, and the wire all
share. Sending configuration means sending the struct — or one field of it,
addressed by its protobuf field number. No parallel key vocabulary, no
presence-flag merge semantics, no patch-kind routing.

Stakeholder direction, 2026-08-03, on reading sprint 132's architecture:

> I can barely understand whether we're decoding patches: motor patch routing
> and read-back arms and accessor pairs. This is way too complicated. […] The
> whole system should be one big struct that has parameters in it, right? […]
> when you send configuration, you should be able to send the whole packet, or
> parts of the packet.

Two decisions taken at the same time: **replace** the existing string-key patch
surface rather than running both (one way to do config), and **unify** so a
single struct definition drives both the boot bake and the wire.

## Cause

**Adding one config field today touches 16 places, 9 of them hand-edited,
across 5 languages/formats** (proto, C++, Python, JSON schema, JSON data).
Real commits bear this out: `7256e22f` added 4 fields and touched 29 files;
`b0f329a9` added 2 fields and touched 27.

The same configuration is **defined twice, independently**:

- `src/scripts/gen_boot_config.py` reads the robot JSON and emits C++ boot structs;
- `src/scripts/gen_messages.py` reads `.proto` and emits the wire messages.

Neither knows about the other, so a hand-written lint (`check_config_sync.py`
plus a 58-entry allowlist) exists purely to notice when they drift. Every piece
of vocabulary that made the design unintelligible — patches, presence flags,
string keys, patch-kind routing, merge semantics — is glue between two
definitions that should be one.

Concretely, the shapes do not line up:

| layer | shape |
|---|---|
| the file (`data/robots/tovez.json`) | a struct — nested sections of named values |
| boot config (`Config::*BootConfig`) | a struct — generated from that file |
| **the wire** (`MotorConfigPatch` &c.) | **sparse patches** — presence flags, string keys, oneof routing |

The drift is not hypothetical. Measured 2026-08-03:

- the host pydantic model has **36** `control` fields; the JSON has **53**.
  Pydantic defaults to `extra='ignore'`, so **18 keys are silently dropped** —
  including `control.output_deadband` and `control.reversal_dwell_ms`, which
  `gen_boot_config.py` *requires* and refuses to build without. The host model
  cannot see two keys the firmware cannot build without.
- **17 `control` keys are read by nothing at all.**
- `ShaperBootConfig` is generated, baked, and **read by no code**.
- `msg::DrivetrainConfig` declares 43 fields of which **6** are ever baked.
- `CONFIG_PLANNER` and `CONFIG_WATCHDOG` are dead `ConfigTarget` values.
- string keys have drifted from their meanings: `pid.kff` sets `kaff`,
  `pid.kaw` sets `pidMax` (`configurator.cpp:145-146`).

**The aggregate struct already exists.** `App::RobotGraph::Resolved`
(`src/firm/app/boot_wiring.h:185-195`) already collects every config struct into
one object, computed once at boot and consumed by whole-struct install calls. It
simply never crosses a wire.

## Proposed fix

### Scope boundary: unify the definition, keep the install step

The request splits in two, and they have opposite answers:

| | verdict |
|---|---|
| **(a) config is one struct on the wire** — send the whole thing or one field | **Yes.** This is the simplification. |
| **(b) config is one live object every subsystem reads** | **No.** Deliberately retired; unsafe to re-adopt today. |

(b) existed in this repo — `src/archive/source_old/` has `struct RobotConfig`
injected by const reference into Motor, OtosSensor, SimOdometer and onto
interfaces — and was removed on purpose. `IOdometer.h:23-28` records it: sprint
039-004, *"RobotConfig seal — `RobotConfig&` no longer appears in the public
read signatures."*

It also cannot be a drop-in, because **14 transformations happen at install
time**. The load-bearing ones:

- **track width is derived**, not configured: `trackwidth / rotational_slip`
  (`boot_calibration.cpp:25-29`), then fanned to Drive, Odometry, PoseTracker
  and FakeOtos. The raw JSON number is not what anyone uses.
- **degrees → radians** for rotation offsets (`boot_calibration.cpp:77-80`).
- **OTOS scale factors are written into chip registers** at `begin()`
  (`otos.cpp:45-46`) — after boot the value physically lives in the sensor;
  there is nothing to read live.
- **three latches derive from the act of installing**, not from any value:
  `Drive::calibrated_`, `Planner::shaperConfigured_`, `RobotLoop::configured_`.
  They gate whether the robot actuates at all.
- 8 flat gain/intercept floats reshape into `corrGain_[2][2]`;
  `output_deadband` is cached in three objects; Odometry seeds its baseline
  from live encoder reads at construction.

Plus a hard layering rule: `config/` may depend only on `messages/`, and
`devices/` may include neither — which is *why* each `*BootConfig` has a
converter. `robot_state.h` escapes it only via a `<cstdint>`-only dependency
floor, and its own header excludes config on purpose: *"What stays OUT: config,
PID gains, calibration … RobotState is per-cycle dynamics only."*

So: the install fan-out stays, because it does real work. What gets deleted is
the *second definition* and the glue that kept two definitions in sync.

### Ownership: one object, one owner, two sources, one fan-out

Stakeholder direction, 2026-08-03:

> The configuration subsystem holds the single configuration object, which it
> gets from baked values or from the wire, and it sends it out to each
> subsystem.

**This is the target, and it is not what the code does today.** Today there are
two partial aggregates, owned by two different objects, fed by two paths:

| | boot path | wire path |
|---|---|---|
| aggregate | `RobotGraph::Resolved` (`boot_wiring.h:185-195`) | `Configurator::persistedTuning_` (`configurator.h:77`) |
| owner | `RobotGraph` (composition root) | `App::Configurator` |
| fan-out | install calls (`boot_wiring.cpp:102-105`) | direct setters on `Drive&`/`Planner&` |

`Resolved` is `private` with **no accessor** — only `trackWidth()`
(`boot_wiring.h:170`) escapes — and is never read again after the constructor
body ends (`boot_wiring.cpp:106`). `Configurator` never sees it. Neither
aggregate is authoritative, which is why nothing can answer "what is this robot
running?"

The target collapses these into one: `Configurator` owns **the** config object;
both the baked defaults and the wire write into it; and it is the single thing
that fans out to subsystems.

**Why this is safe where "subsystems read config live" was not.** The
distinction is that the configuration subsystem *sends* rather than subsystems
*reading*. The send **is** the install step, so all 14 install-time
transformations survive untouched — the derivation of track width, degrees→
radians, the OTOS chip-register writes, the `calibrated_`/`shaperConfigured_`
latches. Nothing has to become live-readable, and the `config/`-may-only-depend-
on-`messages/` layering rule is unaffected because subsystems keep receiving
their own narrow structs.

**Two things this deletes outright:**

- **Read-merge-write disappears.** `Configurator` today reads values back *out*
  of subsystems to apply a partial update — `planner_.limits()` then
  `applyShaperLimits()` (`configurator.cpp:65-77`), `drive_.controlGains()`
  then `setControlGains()` (`:141-147`). With one owned object there is nothing
  to read back: the configurator already holds every value, mutates its own
  copy, and re-sends. The accessor pairs this issue's predecessor needed are
  not needed at all.
- **Read-back becomes trivial.** `GetConfig` returns the configurator's object.
  No per-subsystem getters, no reconstructing state from scattered members.

### The design

**One schema.** The `.proto` files become the single definition; everything else
is generated from them — the C++ config structs (replacing hand-written
`boot_config.h`), the wire messages (already), the host model (replacing
hand-written `robot_config.py`), and the JSON schema. The robot JSON becomes
literally a serialized config message. Push and bake are then the same bytes
because they are the same definition, so `check_config_sync.py` and its
allowlist can be **deleted** — there is nothing left to keep in sync.

**Grouping: one message per consumer**, using the existing `ConfigTarget` enum
as the stable message id. `src/firm/config/boot_config.h` already defines
exactly this grouping, and every struct fits one envelope with ~3x headroom:

| struct | fields | ~bytes encoded | fits 240 B |
|---|---:|---:|---|
| `EstimatorBootConfig` | 3 | 15 | yes |
| `OtosBootConfig` | 5 | 25 | yes |
| `ShaperBootConfig` | 6 | 30 | yes |
| `DriveBootConfig` | 11 | 55 | yes |
| `WheelControllerBootConfig` | 11 | 55 | yes |
| `PlannerBootConfig` | 16 | 80 | yes |

A single all-in-one message does **not** fit (297 B against a 240 B budget;
118 config fields total) and must not be attempted.

**Three operations, replacing the entire patch surface:**

| operation | shape |
|---|---|
| set a whole group | `SetConfig{target, <the struct>}` |
| set one value | `SetField{target, field_number, value}` |
| read a group back | `GetConfig{target}` → `ConfigSnapshot{target, <the struct>}` |

The single-value setter is cheap: `decodeInto()` (`src/firm/messages/wire.cpp:546-597`)
is **already a schema-generic, table-driven, offset-writing walker** — it finds
a field by number, writes at `base + fd->offset`, and validates bounds inline.
The setter is that loop minus tag decoding. It must be *emitted by the
generator* (the tables live in an anonymous namespace in `wire.cpp`), not
bolted on from outside.

**Deleted:** the four `*Patch` messages, `Opt<T>` presence flags on config, the
string wire-key vocabulary, `PatchKind` oneof routing, the `persistedTuning_`
merge accumulator, read-merge-write, and the config-sync lint plus allowlist.

**Precedent already in this repo:** `planner_harness.py` passes
`Motion::PlannerLimits` — a nested struct-of-structs — across the ctypes
boundary as one binary unit, guarded by generated size/offset parity exports
(`plannerStructSizes()`, `plannerLimitsOffsets()`, `src/motion/planner/capi.cpp:69,93`).
That is the shape *and* the safety mechanism to reuse.

### Traps — each would bite silently

1. **`kEncodeScratchCap = 220`** (`wire.cpp:684`) is a hardcoded, *unasserted*
   ceiling on nested-message encoding. Exceed it and `encode()` returns 0 — the
   frame is **silently never sent, at runtime, with a clean compile**. Breaks at
   exactly 40 optional float fields. Add a `static_assert` tying it to the
   computed worst case.
2. **There is no fragmentation anywhere** in the wire layer. Per-group messages
   are mandatory, not a preference; oversize frames are dropped whole.
3. **The reply path is tighter than the command path** — the `tlm` arm already
   occupies 188 B of the 240 B reply budget. Per-group snapshots (largest 81 B)
   are fine; a combined snapshot is not.
4. **Field numbers are unique only within a message.** Address the setter by
   `(target, field_number)`. `kMessageTables[]`'s BFS index is *not* stable and
   must not be used as an identifier; `ConfigTarget` is.
5. **No config field declares any bounds today** (`config.proto` has zero
   `(min)`/`(max)`), so "validation comes free" is currently vacuous. Bounds
   must be added to the schema for the generic setter to protect anything.
6. `comms.h:151`'s `kFramedMaxBytes = 200` is a hand-picked literal needing
   adjustment if the command envelope grows.

### Sequencing

Each step is independently useful and verifiable; stopping after any of them
leaves the tree better than before.

1. **Schema unification** — `.proto` becomes the single definition, generating
   the C++ config structs, host model, and JSON schema. No behaviour change;
   the win is that two generators become one and drift becomes impossible.
   Delete `check_config_sync.py` and its allowlist. **This step alone takes a
   new config field from 16 touchpoints to 2** (edit the proto, edit the JSON).
2. **Whole-group set/get over the wire** — `SetConfig`/`GetConfig` +
   `ConfigSnapshot`, one message per `ConfigTarget`. Per-wheel calibration
   arrives here for free as fields of `DriveBootConfig`.
3. **Generic single-value setter**, emitted by the generator, addressed by
   `(target, field_number)`.
4. **Retire the patch surface** — delete the `*Patch` messages, string keys,
   `PatchKind` routing and the merge accumulator, migrating OTOS calibration
   and TestGUI onto the new operations.
5. **Cleanups this audit surfaced** — add the missing
   `output_deadband`/`reversal_dwell_ms` to the host model; delete the 17
   unread `control` keys, dead `ShaperBootConfig`, dead `CONFIG_PLANNER` and
   `CONFIG_WATCHDOG`; set `extra='forbid'` on the pydantic model so a typo in a
   robot JSON fails loudly instead of being silently dropped.

## Verification

- **The parity guard is the acceptance test.** Follow the working pattern:
  `plannerStructSizes()`/`plannerLimitsOffsets()` export C++ sizes and per-field
  offsets, and `planner_harness.py:207-212` walks them to prove the mirror has
  not drifted. Generate the same guard for every config struct — that is what
  makes "one definition" enforced rather than asserted.
- **Add the missing `static_assert` for `kEncodeScratchCap` and prove it
  fires**: temporarily add fields past the limit and confirm a build failure
  rather than a silent runtime `encode()` returning 0.
- **Round-trip every group**: push a struct, read it back, compare
  field-by-field. An ack is not evidence — `EstimatorConfigPatch` fields ack OK
  and land nowhere today (`configurator.cpp:58-60`).
- **Bake/push parity**: build an image from a robot JSON, push the same JSON to
  a robot running a *different* baked config, confirm identical behaviour. This
  is the invariant the design exists to guarantee, so test it directly rather
  than inferring it.
- **Hardware, on the stand** (`tovez`, addressed by UID):
  `src/tests/bench/velocity_profile_gate.py` before/after, showing the L/R
  plateau-tracking gap closing from its measured 11.1 points, from a cold boot.
- **Build for ARM** — several constraints here are `HOST_BUILD`-invisible.
- Sim baseline is 484 passed / 2 known failures
  (`test_clock_sync_activation.py`, `test_fake_transport.py`); establish it
  before starting or "no regressions" is unprovable.

## Related

- [[per-wheel-drive-calibration-as-runtime-configuration]] — its feature is
  absorbed into step 2 as fields of `DriveBootConfig`, rather than as new patch
  fields. Sprint 132 was planned against the old design and sits at
  `stakeholder-review` with **zero tickets created**, so nothing is wasted; it
  should be re-planned against this issue or closed and replaced.
- [[configuration-discipline-one-file-authors-every-value]] — the stakeholder
  rule this design enforces structurally instead of by lint. Its 27-field work
  list collapses once one schema drives both paths.
- [[A-no-firmware-to-host-config-readback]] — satisfied by `GetConfig`/
  `ConfigSnapshot` in step 2.
- [[B-one-owner-per-constant-speed-floor-and-duty-per-speed]] — the
  multiple-owners catalogue; one definition is the structural fix.
- [[B-observability-contract-is-inert-as-shipped]] — config that acks OK and
  lands nowhere is the same disease.
