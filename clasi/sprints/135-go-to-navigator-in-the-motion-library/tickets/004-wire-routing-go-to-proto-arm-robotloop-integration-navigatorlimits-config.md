---
id: '004'
title: 'Wire + routing: GO_TO proto arm, RobotLoop integration, NavigatorLimits config'
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
depends-on:
- '003'
github-issue: ''
issue: replaceable-go-to-moves-in-the-motion-library.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Wire + routing: GO_TO proto arm, RobotLoop integration, NavigatorLimits config

## Description

Wire `Motion::Navigator` (ticket 003) into the real robot: a new `GO_TO`
wire arm, `App::RobotLoop` routing (accept, cancel, own), and
`NavigatorLimits` sourced from the robot JSON through both the runtime
push path and the bake path (configuration-discipline: one file, both
paths read it).

## Wire schema changes

**`src/protos/envelope.proto`** — new `GoTo` message + `CommandEnvelope`
oneof arm. Field numbers verified directly against the CURRENT file at
sprint-planning time (2026-08-06) — re-verify at ticket time, since the
schema may have moved:

```proto
message GoTo {
  float  x = 1;        // [mm]
  float  y = 2;        // [mm]
  uint32 frame = 3;    // 0 = WORLD (OTOS/SEED frame), 1 = ROBOT (resolved once at acceptance)
  float  speed = 4;    // [mm/s] cruise; 0 = config default
  float  arrive = 5;   // [mm] arrival tolerance; 0 = config default
  float  timeout = 6;  // [ms] REQUIRED whole-goto backstop
  uint32 id = 8;       // echoed in the ONE completion ack
}
```

`CommandEnvelope.cmd` oneof gains `GoTo go_to = 26;` — **26 is the correct,
verified next-free number**: `CommandEnvelope`'s reserved list is
`2, 3, 4, 5, 7 to 12, 14 to 18, 19, 20` (retired pre-102 arms) and the
currently-used numbers are `1` (`corr_id`), `6` (`config`), `13` (`stop`),
`21` (`move`), `22` (`wheels`), `23` (`estop`), `24` (`get_config`), `25`
(`set_field`) — 26 is the first number that is neither. `go_to` is spelled
with an underscore (not `goto`, a C++ keyword) because the arm name is
LOAD-BEARING the same way `wheels`/`estop`/`get_config`/`set_field` are:
`WhichOneof("cmd").upper()` derives the wire prefix on the host side
(`io/serial_conn.py`/`io/sim_config.py`'s `_envelope_command_name()`), so
this arm's name must uppercase to exactly `GO_TO`.

**`src/protos/commands.proto`** — new `Verb` enum value. Also verified
directly: `Verb` currently runs `0` through `23` with no gaps (the linked
issue does not state this number explicitly — do not guess it, use this
verified value):

```proto
GO_TO = 24 [(binary) = true];
```

**Codegen**: run `python3 src/scripts/gen_messages.py` (see `build.py`'s
own `_gen_msgs` invocation for the canonical call) after editing both
proto files AND `robot_config.proto` (see NavigatorLimits section below —
one regeneration pass covers both). This regenerates
`src/firm/messages/envelope.h`, `src/firm/messages/commands.h` (the `Verb`
enum + `kVerbTable[]`), `src/host/robot_radio/io/wire_commands.py`, and —
because `robot_config.proto` is also touched — `data/robots/
robot_config.schema.json` and `src/host/robot_radio/config/
robot_config_generated.py`. Confirm `GO_TO` appears in
`kVerbTable[]`/`wire_commands.py`'s `VERBS` after regeneration; do not
hand-edit any of these generated files.

## NavigatorLimits configuration group

Follow the `Planner`/`PlannerShaper` group pattern exactly
(`src/protos/robot_config.proto:496` is `message Planner { ... }` — the
template to copy):

1. Add `NAVIGATOR = 9;` to `enum ConfigGroupTarget`
   (`robot_config.proto:281`, currently `0` through `8`).
2. Add `message Navigator { ... }` with ticket 002's `NavigatorLimits`
   fields (behind-angle, pivot-first threshold, omega slew rate,
   approach-taper parameters, arrival-tolerance default, cruise-speed
   default) — snake_case field names, `[(min) = 0.0]` annotations matching
   the `Planner` message's own style, units in trailing `//` comments per
   this project's units-in-identifiers rule.
3. `src/firm/config/boot_config.cpp`: add `defaultNavigatorGroup()`
   mirroring `defaultPlannerGroup()` (boot_config.cpp:299) — reads the
   robot JSON's new `navigator` block (auto-generated schema key from
   step 2, no hand-written JSON Schema edit needed) and bakes it into
   `msg::Navigator`. Wire it into the composition root
   (`App::composeRobot()`/`RobotGraph`, `src/firm/app/boot_wiring.h`) the
   same way the existing seven groups are wired, so it is genuinely part
   of the SAME bake pass, not a bolt-on.
4. Confirm the existing runtime config-push machinery (`GET_CONFIG`/
   `SET_FIELD`/`CONFIG` arms, `Configurator::applyGroup()`/`applyField()`)
   picks up `NAVIGATOR` with no special-casing — it shouldn't need any;
   these are generic over `ConfigGroupTarget`. If you find yourself adding
   a `NAVIGATOR`-specific branch anywhere in `Configurator`, that's a
   signal something is wrong, not a step to push through.
5. Add a `navigator` block with real values to `data/robots/tovez.json`
   (required for ticket 006's bench pass — bake and pushed config must
   agree, configuration-discipline invariant 2) and, as convenient,
   `gopiv.json`/`togov.json`.

## RobotLoop integration

- `App::RobotLoop::handleGoto(const msg::CommandEnvelope& env)` — new
  handler, same shape as `handleMove()`/`handleWheels()`
  (`robot_loop.cpp:151`/`257`). Rejects (does not accept) the goto if
  `!state_.otos.connected` — issue's own explicit gate. Pick the wire
  error code from the EXISTING `ErrCode` enum
  (`src/protos/envelope.proto:49-59`) rather than inventing one;
  `ERR_NOT_CONFIGURED` (8) is the closest existing fit ("composition root
  refused because a precondition isn't satisfied") — use it unless you
  find a better existing match, and record which you chose in Completion
  Notes (this was an open question at sprint-planning time, resolved
  here).
- `routeCommand()` gains a `GOTO` case (`robot_loop.cpp:94-110`'s switch),
  calling `handleGoto()`.
- `Navigator` is wired into `cycle()` (`robot_loop.cpp:454`) just before
  `planner_.tick(state_)` (currently line 515) — the Navigator's own
  per-cycle update (read pose, solve, maybe replace) runs first so its
  `move()` call is visible to the SAME cycle's `planner_.tick()`.
- `drive_.takeover()`: `handleGoto()` calls it, the same call
  `handleMove()` already makes at `robot_loop.cpp:228`.

### Landmine 1 — spurious `ack(0)` per internal segment

`publishMoveResult()` (`robot_loop.cpp:449-451`) acks
UNCONDITIONALLY on `moveResult.completed`:

```cpp
if (moveResult.completed) {
  tlm_.ack(moveResult.moveId, 0);  // timeout signaled by flag, not ack_err
}
```

Every internal segment Move the Navigator issues carries `id = 0`
(ticket 002/003's field mapping). Without a filter, EVERY internal
replacement that completes (which happens routinely — that's how the
Navigator works) fires a spurious `ack(0)` into the ack ring, alongside
the genuine wire-command acks that also legitimately use id 0 in some
paths. `telemetry.cpp:127-139`'s ack-ring emission does NOT filter this
out on its own. Fix: route `TickResult`s that originate from an
internal Navigator-issued Move away from `publishMoveResult()`'s
unconditional path — either the Navigator consumes its own segment's
`TickResult` directly (never handing it to `publishMoveResult()` at
all) and only `RobotLoop` emits the ONE real completion ack when the
Navigator itself declares the goto Done/Aborted, or `publishMoveResult()`
gains an explicit "this id belongs to the Navigator, suppress" check.
Prefer the former — it matches ticket 003's own "Navigator owns its
one-ack-per-goto contract" design and keeps `publishMoveResult()`
itself unchanged for ordinary wire-originated Moves.

### Landmine 2 — Aligning phase fires on internal pivots

The Planner's terminal fine-align phase (`planner.cpp:766-770`) holds a
completed Twist Angle Move open for ~2 s of low-speed trim nudges before
firing its completion ack:

```cpp
if (done && !aligning && alignApplies(m, timedOut, stalled)) {
  enterAligning(now);
  alignSettleCommand(dt);  // stages this tick's command (and rolls history)
  return result;           // result.completed stays false
}
```

A Navigator-issued pivot (SUC-004's stop-then-pivot) is a Twist Angle
Move and WILL trigger this. If the Navigator naively waits for that
pivot's own completion before issuing its next replace, it stalls for
up to ~2 s per pivot. The design intent (sprint.md Architecture) is that
the Navigator replaces the pivot with the outbound cruise arc BEFORE the
Aligning trim completes — i.e., once the pivot has landed close enough
(not necessarily Aligning-settled), issue the next `move(..., replace=
true)`, which flushes the Aligning state along with everything else
`replace=true` already flushes (`planner.cpp:243-282`). Verify this
explicitly with a ctest scenario: a pivot followed immediately by a
cruise replace should NOT incur the ~2 s Aligning delay. If ticket 003's
own state machine didn't already prove this (it may not have exercised
the real Aligning path if its scripted `RobotState` never satisfied
`alignApplies()`), add the scenario here.

### Landmine 3 — rotation-calibration inversion is skipped when OTOS present, by different reasoning than usual

`robot_loop.cpp:210-221`:

```cpp
if (m.kind == Motion::Move::Kind::Angle && !state_.otos.present) {
  // ... pre-distorts m.threshold using rotGainPos_/rotOffsetPos_ etc ...
}
```

This correction only runs when OTOS is ABSENT (open-loop rotation
calibration; when OTOS is present, the loop is closed on optical truth
and pre-distorting would make it stop at the wrong place). Navigator
pivots bypass `handleMove()` entirely (they're issued internally via
`Planner::move()`, never through the wire-command handler this
correction lives in) — this is CORRECT, not an oversight, but only
because goto acceptance is already gated on `otos.connected` (this
ticket's own `handleGoto()` gate, above): a Navigator pivot only ever
happens with OTOS present, which is exactly the condition under which
this correction would have been a no-op anyway. Do not add this
correction to the Navigator's pivot path — it would be wrong (the OTOS
closes the loop already) and it isn't reachable in the uncorrected case
regardless (no goto without OTOS). Document this reasoning at the
pivot call site so a future reader doesn't "fix" it.

### Landmine 4 — omega sign

Commanded ω is opposite world-CCW: `YAW_SIGN = -1.0` in
`src/tests/bench/goto_otos.py`, reconciled on the host today by the
OTOS-heading negation at `planner.cpp:499-513`. `ArcSolver` (ticket 002)
works in the codebase's OTOS/world convention; the ONE point where this
sign flip must be applied is at the Navigator's command boundary — where
it hands `omega` to `Planner::move()` — commented clearly so it is
flipped together with the deferred kinematics fix this constant is
standing in for, not scattered across multiple call sites. Verify: a
goto curving toward a target with a known world-frame bearing produces
the correct-sign commanded `omega`, checked against `goto_otos.py`'s own
`YAW_SIGN` convention (a ctest or a sim scenario, whichever ticket 003's
existing harness makes cheaper to extend).

### Ownership: estop/MOVE/WHEELS cancel an active goto, and vice versa

- `handleMove()`, `handleWheels()`, `handleEstop()`: each must cancel an
  active Navigator target (no completion ack — preempted, not completed)
  before proceeding with their own existing behavior, the same way they
  already cancel each other.
- `handleGoto()`: must cancel active Drive teleop the same way
  `handleMove()` already does (`drive_.takeover()`).
- `handleEstop()` in particular must clear the Navigator's target in the
  SAME cycle it clears the Planner's queue — no completion ack, no
  fault ack, just gone, matching ESTOP's existing "halt now, everywhere"
  contract.

## Build-list touch points (this ticket is where they become real)

New files from tickets 002/003 (`src/motion/navigator/arc_solver.{h,cpp}`,
`navigator.{h,cpp}`) become part of the actual firmware/sim image for the
first time in this ticket (RobotLoop now calls `Navigator`). Root
`CMakeLists.txt` needs no edit (confirmed glob-based, see ticket 002).
Two touch points DO need edits:

1. **`src/sim/CMakeLists.txt`'s `MOTION_SOURCES`** (~line 135-142): add
   `"${MOTION_DIR}/navigator/arc_solver.cpp"` and
   `"${MOTION_DIR}/navigator/navigator.cpp"` alongside the existing
   `planner/*.cpp` entries.
2. **Every `src/tests/sim/*` pytest file whose `_APP_SOURCES` list names
   planner sources.** Re-derive the CURRENT list at ticket time — do not
   trust the count from sprint planning:
   ```
   grep -rln "planner.cpp" src/tests/sim/
   ```
   (22 files matched at sprint-planning time 2026-08-06; treat that as a
   floor to verify against, not a number to stop at). Each matching
   file's `_APP_SOURCES` list needs the same two new entries, in the same
   `_REPO_ROOT / "src" / "motion" / "navigator" / "arc_solver.cpp"` /
   `"navigator.cpp"` path style `test_app_robot_loop_replace.py` already
   uses for `_MOTION_PLANNER_DIR / "planner.cpp"` and
   `_REPO_ROOT / "src" / "motion" / "odometry.cpp"`. A missing file here
   is a LINK error, not a compile error — it will not show up until that
   specific test file is run.

## Acceptance Criteria

- [x] `GoTo` message + `CommandEnvelope.go_to = 26` added to
      `envelope.proto`; `Verb.GO_TO = 24 [(binary) = true]` added to
      `commands.proto`; codegen re-run; `GO_TO` present in generated
      `kVerbTable[]` and `wire_commands.py`'s `VERBS`.
- [x] `ConfigGroupTarget.NAVIGATOR = 9` + `message Navigator` added to
      `robot_config.proto`, following the `Planner` message's pattern;
      `defaultNavigatorGroup()` added to `boot_config.cpp` and wired into
      the composition root; `data/robots/robot_config.schema.json`
      regenerated (not hand-edited) with a `navigator` block;
      `tovez.json` gains real tuned values.
- [x] `RobotLoop::handleGoto()` + `routeCommand()` GOTO case; goto
      rejected with a named `ErrCode` (not a new one) when
      `!state_.otos.connected`; `drive_.takeover()` called on acceptance.
- [x] `Navigator` ticks once per `cycle()`, before `planner_.tick()`.
- [x] Landmine 1 fixed: a sim/ctest scenario streaming several internal
      replacements shows ZERO spurious `ack(0)` entries in the ack ring,
      and exactly ONE completion ack per completed goto id.
- [x] Landmine 2 verified: a pivot-then-cruise scenario shows the
      Navigator's next replace is NOT blocked behind the Planner's ~2 s
      Aligning trim.
- [x] Landmine 3: documented at the pivot call site, not "fixed" (there
      is nothing to fix — verify no rotation-calibration correction is
      applied to Navigator-issued pivots).
- [x] Landmine 4: a scenario with a known world-frame target bearing
      confirms the commanded `omega` sign matches `goto_otos.py`'s
      `YAW_SIGN = -1.0` convention.
- [x] MOVE/WHEELS/ESTOP each cancel an active goto with no completion
      ack; a GO_TO cancels active WHEELS teleop via `drive_.takeover()`;
      ESTOP clears the Navigator's target in the same cycle it clears
      the Planner's queue.
- [x] `src/sim/CMakeLists.txt` and every pytest file found by the
      `grep -rln "planner.cpp" src/tests/sim/` re-derivation (this
      ticket's own fresh count, not sprint-planning's) updated with the
      two new navigator source files.

## Testing

- **Existing tests to run**: full sim suite (this ticket touches
  `RobotLoop`, the widest-blast-radius file in the base):
  ```
  uv run python -m pytest
  ```
  Also re-run `planner_tests` and ticket 002/003's own navigator ctest
  target to confirm the wiring didn't regress the standalone builds.
- **New tests to write**: the four landmine scenarios above, plus a basic
  end-to-end "GO_TO command accepted, Navigator drives, completion ack
  observed" sim/unit test in `src/tests/sim/unit/` (mirroring
  `test_app_robot_loop_replace.py`'s harness style) — ticket 005 owns the
  fuller system-level wire-codec test, this ticket's own test just proves
  the routing itself is correct.
- **Verification command**:
  ```
  uv run python -m pytest
  ```

## Completion Notes

### ErrCode choice for goto rejection

`handleGoto()` uses `ERR_NOT_CONFIGURED` (8) for BOTH of its two reject
paths: `!configured_` (the pre-existing composition-not-ready gate every
other handler already uses) and `!state_.otos.connected` (this ticket's
own new gate). No new `ErrCode` was added. `ERR_NOT_CONFIGURED` was the
closest existing fit named in the ticket, and on review there wasn't a
better alternative in the existing enum (`ERR_BADARG` is reserved for
malformed/out-of-range wire fields — used here for `timeout <= 0`, which
IS a genuinely separate malformed-argument case; `ERR_BUSY`/`ERR_FULL`
don't apply to a precondition failure; there is no `ERR_NO_OTOS` or
similar). The two `ERR_NOT_CONFIGURED` paths are reached via separate
`if` blocks (not merged into one condition) so a future ticket that wants
to split them onto distinct codes can do so without re-deriving the
control flow.

### Landmine 4 — NOT already handled by ticket 003's OTOS negation

This was flagged in the ticket as worth resolving carefully rather than
quickly, and it needed the careful treatment: ticket 003's existing
single OTOS-heading negation (`planner.cpp`'s reconciliation, referenced
in the ticket body) does **not** fully resolve the omega-sign question
for the Navigator's own commanded `omega`. I verified this by direct
numeric comparison (Python, non-degenerate angles) between "apply only
ticket 003's existing negation" and `goto_otos.py`'s own proven
`YAW_SIGN = -1.0` formula — they only coincidentally agreed at
`raw_heading = 90°` (a degenerate angle where `cos(heading) = 0` trivially
satisfies both) and disagreed in magnitude/sign elsewhere (e.g.
`raw_heading = 0°`). A hardcoded `kNavYawSign = -1.0` constant was tried
first and confirmed (by an actual build+run) to break
`testConvergesFromRestAndSettles` in the standalone navigator ctest —
proving the sign flip is not simply "always negate" but depends on
context the test plants can't represent.

The actual fix is `Motion::NavigatorLimits::yawSign` (`arc_solver.h`), a
new configurable field (default `1.0`, identity) applied at exactly two
points in `navigator.cpp`: building `worldPose.heading` from the OTOS
pose before every `ArcSolver::solve()` call, and on the final
`m.omega = limits_.yawSign * solution.omega` / pivot-omega assignment.
Because the default is identity, every pre-existing navigator/planner
test that never touches `yawSign` is byte-for-byte unaffected — zero
regression risk. `tovez.json`'s baked `navigator.yaw_sign` is `-1.0`
(measured, matching `goto_otos.py`); `tovez_nocal.json`/`togov.json`/
`gopiv.json` bake `1.0` (unmeasured/identity).

This surfaced a second, structural finding: baking the real hardware's
`yaw_sign = -1.0` into the SAME robot JSON used for sim composition
breaks `TestSim::SimPlant`'s closed-loop convergence, for the identical
structural reason `TestNav::IdealPlant` can't represent the quirk either
— both plants read the commanded wheel velocity directly as their own
ground truth, so a sign flip that is physically real on hardware becomes
a self-fighting oscillation in a plant with no independent physical
truth to diverge against. Fixed with a 5th `BootOverrides` field,
`navigatorYawSign` (`boot_wiring.h`/`.cpp`), following the exact
precedent already set by `otosConfig`/`wheelCorrection`/`trackWidth`/
`controlPeriod`+`actuationDelay`: a real robot passes `nullptr` (gets the
baked, measured value); `TestSim::SimHarness` passes
`&kIdentityNavigatorYawSign` (`1.0f`). Applied in `boot_wiring.cpp`'s
constructor body immediately after `configurator_.install()` (install()
is what writes `navigatorLimits_.yawSign` from the baked config in the
first place, so the override has to land after it, not before).

### Landmine 2 — also needed a real fix, not just verification

The ticket allowed for "verify, and fix if needed." It needed the fix.
`Navigator::tick()`'s `PivotPhase::Pivoting` branch originally waited on
`moveResult.completed` alone to advance past the pivot. Because a pivot
is a `Move::Kind::Angle`, it satisfies `alignApplies()`
(`planner.cpp`'s terminal fine-align, ticket 134-003) and so
`moveResult.completed` can stay false for up to ~2s while the Planner
holds it in `MoveLifecycle::Aligning` for low-speed trim nudges — exactly
the stall the ticket described. Fixed by also advancing on
`planner_.lifecycle() == MoveLifecycle::Aligning`, which is available the
moment the pivot has landed close enough (before the Aligning trim
settles), letting the Navigator issue its `replace = true` cruise arc
immediately — that `replace` call flushes the Aligning state on its own
(`planner.cpp`'s existing replace semantics), so nothing is left stranded.
Verified via a new navigator ctest, `testPivotThenCruiseNotBlockedBehindAligning`,
using the REAL Aligning parameters (not a relaxed test double):
`alignTol = 0.017453` rad, `alignMaxNudges = 6` — measured
`DELTA_TICKS = 0` against a `<= 3` bound (i.e., the replace fires within
the same handful of ticks it would without Aligning in the picture at
all, not after a ~2s delay).

### Landmine 3 — documented only, no fix (as expected)

Confirmed by reading the call site: Navigator-issued pivots go through
`Planner::move()` directly, never through `handleMove()`'s wire-command
path, so the rotation-calibration pre-distortion
(`robot_loop.cpp`, gated on `!state_.otos.present`) never runs for them
regardless. This is safe specifically because `handleGoto()` gates goto
acceptance on `state_.otos.connected` — a Navigator pivot only exists
in a world where OTOS is present, which is exactly the branch this
correction already skips on its own. A doc comment recording this
reasoning was added at the pivot call site in `robot_loop.cpp` (ahead of
the `if (m.kind == Motion::Move::Kind::Angle && !state_.otos.present)`
block) so a future reader doesn't try to extend the correction to cover
Navigator pivots.

### Landmine 1 — spurious ack(0), fixed via mutual exclusion + boolean ack checks

Fixed by construction, not by a suppression filter: `cycle()` now routes
to EITHER `navigator_.tick(state_)` (when `navigator_.active()`) OR
`planner_.tick(state_)`/`planner_.update(state_)`, never both in the
same cycle. `publishGotoResult()` (new, mirrors `publishMoveResult()`'s
shape) only calls `tlm_.ack()` when `navResult.completed` is the WHOLE
goto completing (Navigator-internal segment replacements never reach
`publishMoveResult()`'s unconditional per-segment ack path at all, since
that path is only reached from the `else` branch). Verified in the new
`test_app_robot_loop_goto_harness.cpp` basic end-to-end scenario using
boolean "did this key ever appear in the ack ring" checks
(`sawEnqueueAck`/`sawCompletionAck`), per the established
`scenarioDuplicateIdSanityNoOp()` convention (the ack ring retains
entries across several consecutive telemetry frames by design, so a
raw per-cycle count double/triple-counts a single logical ack — a count
is not the right check here, presence is).

### Build-list re-derivation: 22 files, not the sprint-planning estimate

`grep -rln "planner.cpp" src/tests/sim/` at ticket time returned 26 raw
hits; 4 were comment-only references (no `_APP_SOURCES` list to touch),
netting exactly **22** files whose `_APP_SOURCES` actually needed the two
new navigator source-file entries plus a `src/motion/planner` include
flag (`navigator.h`'s own bare `#include "planner.h"` needed it, same
reason the standalone navigator ctest project already adds that include
dir). This matches the sprint-planning-time floor of 22 exactly; it was
still re-derived fresh per the ticket's instruction rather than assumed.
`src/sim/CMakeLists.txt`'s `MOTION_SOURCES` got the same two files plus
the same include dir. Five additional harness `.cpp` files (not pytest
files — files with a direct, hand-written `App::Configurator`
construction) needed a local `Motion::NavigatorLimits` added at 6
construction call sites, an unanticipated but mechanical consequence of
`Configurator`'s constructor gaining a 6th parameter.

### Test results (final, clean run)

- Standalone navigator ctest: 12/12 checks passing, including the 3 new
  checks added this ticket (`testYawSignMatchesGotoOtosConvention`,
  `testPerGotoSpeedOverrideAppliesToCruise`,
  `testPivotThenCruiseNotBlockedBehindAligning`).
- `arc_solver_test`: 17/17 (unchanged by this ticket, run to confirm no
  regression from the `NavigatorLimits` field additions).
- `planner_tests` (standalone): 8/8.
- Full sim suite (`uv run python -m pytest`): **487 passed, 3 failed, 2
  xfailed, 1 xpassed**. The 3 failures
  (`test_motor_primitive.py::test_heading_encoder_and_otos_match_truth`,
  `test_app_preamble.py::test_app_preamble_harness_compiles_and_passes`,
  `test_devices_otos.py::test_devices_otos_harness_compiles_and_passes`)
  are confirmed pre-existing and unrelated to this ticket's changes —
  reproduced identically against a `git stash`-clean baseline (same
  failure text before and after this ticket's diff), and two of the three
  were already flagged pre-existing in ticket 008's own Completion Notes.
  Zero new regressions; the new `test_app_robot_loop_goto.py` file (5
  scenarios, collected as 1 pytest test) is included and passing.

### Files touched

See the diff on this branch (`sprint/135-go-to-navigator-in-the-motion-library`)
for the complete list. Highest-signal files: `src/protos/envelope.proto`,
`src/protos/commands.proto`, `src/protos/robot_config.proto`,
`src/firm/app/robot_loop.{h,cpp}`, `src/firm/app/boot_wiring.{h,cpp}`,
`src/firm/app/configurator.{h,cpp}`, `src/firm/app/boot_calibration.{h,cpp}`,
`src/motion/navigator/navigator.{h,cpp}`, `src/motion/navigator/arc_solver.h`,
`src/tests/sim/unit/test_app_robot_loop_goto_harness.cpp` (new),
`src/tests/sim/unit/test_app_robot_loop_goto.py` (new),
`data/robots/tovez.json`, `data/robots/{tovez_nocal,togov,gopiv}.json`.
