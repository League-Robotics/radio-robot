---
id: '003'
title: SIMSET/SIMGET wire-command surface (SimCommands, sim-build-only)
status: open
use-cases:
- SUC-003
- SUC-004
- SUC-006
depends-on:
- '002'
github-issue: ''
issue: sim-error-model-runtime-settable-hardware-fit.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# SIMSET/SIMGET wire-command surface (SimCommands, sim-build-only)

## Description

This is the pivotal decision of the sprint (`architecture-update.md` Design
Rationale Decision 1): give the simulator a `SET`/`GET`-equivalent for
plant/error parameters via a NEW, sim-build-only verb pair (`SIMSET`/
`SIMGET`), NOT a reserved key namespace inside the shared `ConfigRegistry`/
`RobotConfig` — because `RobotConfig` (`source/types/Config.h`) and
`ConfigRegistry.cpp` are compiled into BOTH the ARM firmware target and the
sim library (confirmed: `Config.h` is a plain POD struct with no `#ifdef
HOST_BUILD`, and `ConfigRegistry.cpp`'s `kRegistry[]` macros expand to
`offsetof(RobotConfig, field)` — a compile-time requirement that the field
exist in the one shared struct for both targets). Adding 17 sim-only fields
to `RobotConfig` would bloat ARM firmware flash/RAM with fields meaningless
on real hardware and require `#ifdef HOST_BUILD` scattered through files
that today have none.

The codebase already has the exact extension point needed:
`Robot::buildCommandTable()` (`source/robot/Robot.h:219-221`,
`source/commands/SystemCommands.cpp:1079-1080`) already takes an optional
`DebugCommands* dbg = nullptr` and, at `SystemCommands.cpp:1115`, does
`if (dbg) append(dbg->getCommands());` — a proven "optional, separately-owned
`Commandable*` the ARM build can omit and the sim/bench build can supply"
pattern. `DebugCommands` (`source/commands/DebugCommands.h:37`) is the
existing instance of this pattern to mirror structurally.

`ConfigCommands.cpp` already demonstrates the exact generic-KV parsing
machinery this ticket reuses verbatim:
- `parseSet` (`ConfigCommands.cpp:66-105`) — converts `key=value` tokens into
  `"key=value"` STR args, custom `parseFn` reading `kvs[]` not `tokens[]`.
- `getSchema` (`ConfigCommands.cpp:63`, `{nullptr, 0, 0, true, nullptr}`) — a
  variadic `ArgSchema` where every token becomes `args[i].sval` (a key name),
  used by `GET`.
- `appendConfigCommands()` (`ConfigCommands.cpp:106-111`) — registers `GET`
  via `makeSchemaCmd(&getSchema, ...)` and `SET` via `makeCmd(parseSet, ...)`.

`SIMSET`/`SIMGET` reuse this SAME `parseSet`/`getSchema` shape (no new
parsing code), dispatching through a NEW registry (`kSimRegistry[]`) that —
unlike `ConfigRegistry`'s `offsetof`-into-a-POD-struct approach — dispatches
through NAMED SETTER/GETTER FUNCTION POINTERS, because `PhysicsWorld`/
`SimOdometer` are encapsulated classes with invariants, not POD structs
(Design Rationale Decision 3 — this also avoids the stale-snapshot
antipattern sprint 067 eliminated from `Planner`/`Drive`).

This ticket wires the FIRST registry rows: `bodyRotScrub` → ticket 002's
`PhysicsWorld::setBodyRotationalScrub()`, `bodyLinScrub` →
`setBodyLinearScrub()`, `trackwidthMm` → `SimHardware::setTrackwidth()`
(`source/hal/sim/SimHardware.h:74`, already forwards to
`_plant.setTrackwidth()`), `motorOffsetL`/`motorOffsetR` →
`PhysicsWorld::setOffsetFactor()` (`PhysicsWorld.h:132-136`, existing).
Remaining knobs (encoder-report error, OTOS error) are ticket 004's scope,
added to the SAME registry once this mechanism exists.

## Acceptance Criteria

- [ ] New files `source/commands/SimCommands.h`/`.cpp`: a `Commandable`
      subclass (mirrors `DebugCommands`'s shape) constructed from
      `SimHardware&`. Holds `kSimRegistry[]`: an array of `{key, setterFn,
      getterFn}` rows, function-pointer dispatch over `SimHardware&` (NOT
      `offsetof` — see Design Rationale Decision 3). `getCommands()` returns
      two `CommandDescriptor`s:
      - `SIMSET` — reuses the existing `parseSet` grammar (same
        `key=value…` token parsing `SET` uses; either call the existing
        file-local `parseSet` if made reusable, or an equivalent local copy
        with identical behavior).
      - `SIMGET` — reuses `GET`'s variadic `ArgSchema` shape (`{nullptr, 0,
        0, true, nullptr}`).
      Reply shapes mirror `SET`/`GET` exactly: `OK simset <applied
      key=value…>` on success, `ERR badkey <key>` for an unregistered key,
      `ERR badval <key>=<value>` for an unparsable value (atomic — a `SIMSET`
      with one bad key/value applies NONE of the keys in that command, same
      as `SET`'s existing all-or-nothing semantics — confirm by reading
      `handleSet`'s actual behavior in `ConfigRegistry.cpp:404+` and match
      it, don't assume). `SIMGET` with no args dumps every registered key as
      one or more `SIMCFG key=value…` reply lines (mirrors `GET`'s `CFG`
      chunking at `ConfigRegistry.cpp:171-254` if the sim-only dump risks
      exceeding one line — confirm actual byte budget at implementation
      time); `SIMGET <key>…` returns only the named keys, unknown key →
      `ERR badkey <key>`.
- [ ] First `kSimRegistry[]` rows: `bodyRotScrub` (→
      `PhysicsWorld::setBodyRotationalScrub`/`bodyRotationalScrub`, from
      ticket 002), `bodyLinScrub` (→ `setBodyLinearScrub`/
      `bodyLinearScrub`), `trackwidthMm` (→
      `SimHardware::setTrackwidth()`/new getter — `SimHardware.h` has no
      trackwidth getter today; add one forwarding to
      `_plant.trackwidthMm()`), `motorOffsetL`/`motorOffsetR` (→
      `PhysicsWorld::setOffsetFactor(0/1, f)`/new getters
      `offsetFactorL()`/`offsetFactorR()` — `PhysicsWorld.h` has no such
      getters today; add them, mirroring the existing `rotationalSlip()`
      accessor shape at line 224).
- [ ] `source/robot/Robot.h`: forward-declares `class SimCommands;` (near
      the existing `class DebugCommands;` forward declaration, line 43);
      `buildCommandTable()`'s signature (line 219-221) gains a third
      optional parameter: `SimCommands* sim = nullptr`.
- [ ] `source/commands/SystemCommands.cpp`: `Robot::buildCommandTable()`
      (definition at line 1079) accepts the new `sim` parameter and, at
      line 1115 (immediately after `if (dbg) append(dbg->getCommands());`),
      adds `if (sim) append(sim->getCommands());`.
- [ ] ARM build path unaffected: the ARM target's `main.cpp` call site
      passes only `dbg` (or nothing), `sim` defaults to `nullptr`. Confirm by
      inspection that no ARM-build translation unit `#include`s
      `SimCommands.h`.
- [ ] `tests/_infra/sim/sim_api.cpp`: `SimHandle` (struct, line ~120) gains a
      `SimCommands _simCmds` member constructed from `hal` (the existing
      `SimHardware` instance); the `SimHandle` → `Robot::buildCommandTable()`
      call site passes `&_simCmds` as the third argument.
- [ ] On real (non-sim) firmware — verify via a targeted test on a
      command table built WITHOUT a `SimCommands*` (`sim=nullptr`) — `SIMSET`
      and `SIMGET` reply `ERR unknown SIMSET` / `ERR unknown SIMGET`, exactly
      like any other unrecognized verb.
- [ ] `docs/protocol-v2.md`: new `## 15. Sim-Only: SIMSET / SIMGET` section,
      grammar mirroring §7 exactly, with an explicit note that these verbs
      exist ONLY in sim/`HOST_BUILD` binaries.
- [ ] Ticket 002's `tests/simulation/system/test_069_rt_90deg_body_scrub.py`
      is REBASED from its ticket-002 direct-ctypes setup
      (`sim_set_body_rot_scrub`/`sim_set_body_lin_scrub`) onto `SIMSET
      bodyRotScrub=… bodyLinScrub=…` sent as a normal command through the
      sim's command dispatch — same two acceptance assertions (RT 9000 →
      90° with scrub=0.92; RT 9000 → exactly 90° with all defaults +
      `rotSlip=1.0`), different transport. Do not delete the ctypes forwards
      added in ticket 002 (`sim_set_motor_offset`-style back-compat is
      explicitly preserved — see `architecture-update.md` Migration
      Concerns); they simply become an alternate, still-valid entry point,
      formally rebased in ticket 005.
- [ ] New test `tests/simulation/unit/test_sim_commands_registry.py`:
      `SIMSET`/`SIMGET` grammar coverage — unknown key → `ERR badkey`,
      atomic all-or-nothing apply on a mixed valid/invalid `SIMSET`, bare
      `SIMGET` dumps all registered keys, `SIMSET`/`SIMGET` are `ERR unknown`
      on a command table built with `sim=nullptr`.
- [ ] Full default suite green: `uv run python -m pytest`.

## Testing

- **Existing tests to run**: `test_069_rt_90deg_body_scrub.py` (rebased, see
  above); `test_sim_otos_lever_arm.py` (066-001, confirm unaffected); full
  default suite.
- **New tests to write**: `test_sim_commands_registry.py` (grammar/dispatch
  coverage, described above); rebase of `test_069_rt_90deg_body_scrub.py`.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Add `SimCommands` as a second instance of the existing
`DebugCommands`-shaped optional-`Commandable*` extension point — no new
architectural idea, a structural copy of a proven pattern. Reuse `SET`/
`GET`'s existing generic KV-parsing machinery (`parseSet`, the variadic
`ArgSchema`) rather than writing new parsing code. Dispatch via named
setter/getter function pointers, not `offsetof`, to respect
`PhysicsWorld`/`SimOdometer`'s encapsulation (Decision 3).

**Files to create**:
- `source/commands/SimCommands.h`/`.cpp` — new `Commandable` subclass,
  `kSimRegistry[]`, `SIMSET`/`SIMGET` descriptors.

**Files to modify**:
- `source/robot/Robot.h` — forward declaration, `buildCommandTable()`
  third parameter.
- `source/commands/SystemCommands.cpp` — `buildCommandTable()` definition,
  one new conditional `append()` call.
- `source/hal/sim/SimHardware.h` — new `trackwidthMm()` getter forwarding to
  `_plant.trackwidthMm()`.
- `source/hal/sim/PhysicsWorld.h` — new `offsetFactorL()`/`offsetFactorR()`
  getters mirroring `rotationalSlip()`.
- `tests/_infra/sim/sim_api.cpp` — `SimHandle` gains a `SimCommands` member
  and updated `buildCommandTable()` call site.
- `tests/simulation/system/test_069_rt_90deg_body_scrub.py` — rebase onto
  `SIMSET`.
- `docs/protocol-v2.md` — new §15.

**Testing plan**:
- New unit test for `SIMSET`/`SIMGET` grammar and dispatch correctness
  (including the ARM-equivalent `sim=nullptr` "unknown verb" case).
- Rebase ticket 002's system test onto the new wire surface; confirm
  identical pass/fail behavior to the ctypes-based version.
- Confirm 066-001 and the golden-TLM fixture remain unaffected (no new
  field is on the TLM path).
- Full `uv run python -m pytest`.

**Documentation updates**: `docs/protocol-v2.md` — new §15 `Sim-Only:
SIMSET / SIMGET`, grammar mirroring §7.
