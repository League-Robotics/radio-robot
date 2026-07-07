---
id: "007"
title: "Cyclic-executive main loop: delete dev_loop, rewire main.cpp and sim_api.cpp, preserve serial-silence watchdog"
status: open
use-cases: [SUC-001, SUC-005, SUC-006]
depends-on: ["002", "003", "004", "005", "006"]
github-issue: ""
issue:
- preserve-serial-silence-safety-watchdog-in-greenfield-loop.md
- plan-file-a-design-issue-blackboard-architecture-state-objects-command-queues.md
# completes_issue: Controls whether linked issues are archived when this ticket
# is moved to done. Default: true (archive when all referencing tickets are done).
# Set to false (scalar) to suppress archival for ALL linked issues on this ticket.
# Set to a mapping {filename.md: false} to suppress archival per issue filename.
# Use false for tickets that partially address a multi-sprint umbrella issue.
completes_issue:
  preserve-serial-silence-safety-watchdog-in-greenfield-loop.md: false
# This ticket only PARTIALLY addresses preserve-serial-silence-safety-watchdog-
# in-greenfield-loop.md: the sim-side behavior and the same-pass/queue-bypass
# correctness are delivered here, but the issue's own Bench/HITL acceptance
# criterion (radio-path comms-silence neutralize on the stand) is only closed
# out by ticket 009. The per-issue mapping above suppresses archival for the
# watchdog issue specifically on THIS ticket (it archives once 009 completes
# instead); the design issue (the other entry in `issue:` above) is unaffected
# and keeps the default archive-when-all-referencing-tickets-are-done behavior.
# exception: Written by a lower agent when it cannot proceed (see architecture §exception-protocol).
# exception:
#   thrown_by: "programmer"          # "programmer" | "sprint-planner"
#   thrown_at: "2026-05-07T14:23:00Z"
#   attempted: |
#     Description of what was attempted before giving up.
#   conflict: "architecture-update.md §3 — reason the agent is blocked"
#   surface: "internal"              # "user-visible" | "internal"
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Cyclic-executive main loop: delete dev_loop, rewire main.cpp and sim_api.cpp, preserve serial-silence watchdog

**Note (architecture-update-r1.md Decision 10):** `comm.takeStatement()`'s
return type, `Subsystems::CommunicatorToCommandProcessorStatement`, is
unchanged in name but now defined in `source/subsystems/statement.h`
(ticket 002), included via `communicator.h` — no change needed at this
ticket's own call site (`main.cpp`/`sim_api.cpp` just call
`comm.takeStatement()` and pass the result to `router.route()`, same as
before); flagged only so the type's new home isn't a surprise if this
ticket's implementer greps for it.

## Description

Delete `source/dev_loop.{h,cpp}` entirely and rewrite `source/main.cpp`'s
loop body as the cyclic executive from `architecture-update.md`'s Reference
code: **mandatory tick** (`Hardware`, `Drivetrain`, `PoseEstimator`,
`Planner`) -> **commit** (bulk-copy each subsystem's `state()` into the
Blackboard, sample the odometer, `routeOutputs`, call `Telemetry.tick()` at
its existing call site — `Telemetry`'s *own* internals reading the
blackboard is ticket 008's scope; this ticket only needs to preserve the
call site) -> **best-effort slack** that **yields via `uBit.sleep(1)` every
iteration** before ingest/route/configure (Decision 9 — the
stakeholder-mandated correction to the design issue's original busy-wait
reference code; routing wins over config application per Decision 8).
`tests/_infra/sim/sim_api.cpp` is rewired identically **in this same
ticket** (the 1:1-mirror invariant — both wiring sites change together,
never split across two tickets).

**NON-NEGOTIABLE — safety-critical, see the linked watchdog issue.** The
serial-silence safety watchdog (`SerialSilenceWatchdog`, today in
`dev_commands.h`) must survive this rewrite intact: same-pass `check()`
every mandatory pass; fed on **any** statement arrival, on **any** channel,
during slack ingest; **immediate same-pass neutralize on fire**, bypassing
`driveIn`/`motorIn` entirely (the one sanctioned exception to synchronous
update, per Decision 6); `EVT dev_watchdog` emitted on fire; window
settable via `DEV WD`. This ticket implements the sim-side/same-pass
correctness; the issue's Bench/HITL radio-path acceptance is closed out by
ticket 009 (hence `completes_issue: false` above for the watchdog issue on
this ticket).

## Acceptance Criteria

**Loop rewrite:**

- [ ] `source/dev_loop.h` and `source/dev_loop.cpp` are deleted; no file in
      `source/` references `DevLoopState`/`DevLoop`/`devLoopTick` after this
      ticket.
- [ ] `source/main.cpp` constructs `Communicator`, `NezhaHardware`,
      `Drivetrain`, `PoseEstimator`, `Planner`, `Telemetry`, one
      `Configurator` (holding the four subsystem refs), one
      `CommandRouter`, and one `Rt::Blackboard`, then runs the loop exactly
      as in `architecture-update.md`'s Reference code (mandatory tick ->
      commit -> slack).
- [ ] `tests/_infra/sim/sim_api.cpp` is rewired identically in this same
      ticket/commit, substituting `SimHardware` for `NezhaHardware` and its
      own boot-config/reply packaging — confirmed by diffing the two
      files' loop bodies for structural equivalence.
- [ ] The slack loop's first statement each iteration is `uBit.sleep(1)`
      (Decision 9); comms ingestion, statement routing, and Configurator
      application all happen only after that yield.
- [ ] Decision 1's producer-side authority gate (ticket 006) and Decision
      2's per-port `motorIn[]` unpack are both exercised correctly by
      `routeOutputs` at this integration point (`Drivetrain`'s addressed
      output command is split into `bb.motorIn[p.left]`/`[p.right]`;
      `Planner`'s output only reaches `driveIn` when `Drivetrain`'s
      published authority state allows it).

**Safety watchdog (non-negotiable):**

- [ ] The watchdog's `check()` runs in the loop's **mandatory** portion,
      every pass, same-pass deterministic (not in the slack phase, not
      deferred).
- [ ] On fire, motors are neutralized **immediately, same-pass**, via a
      narrow bypass path that does **not** route through
      `bb.driveIn`/`bb.motorIn`'s one-tick queues. Document the chosen
      bypass mechanism (e.g. a preserved narrow `apply()`-style immediate
      method, or a new dedicated `emergencyNeutralize()`-style method) in
      the ticket's implementation notes.
- [ ] The watchdog is fed (`feed(now)`) on arrival of **any** statement, on
      **any** channel, regardless of content, during slack ingest — fed
      **before** the `sleep(1)`-gated routing branch decides what to do
      with the statement, so feeding is never delayed by routing/
      config-application priority.
- [ ] `DEV WD <window>` remains settable and is routed like any other
      command. Since the watchdog is loop-owned (not one of the four
      Configurator-managed subsystems), confirm/document the chosen
      mechanism (e.g. a small dedicated Blackboard mailbox drained directly
      by the loop's mandatory section, rather than through
      `ConfigDelta`/the Configurator).
- [ ] `EVT dev_watchdog` is still emitted exactly once per silence episode,
      matching today's `check()`'s fire-once contract.
- [ ] `tests/sim/unit/test_watchdog_policy.py`'s two existing scenarios
      (fires-after-window-expires-and-neutralizes; does-not-fire-while-
      commands-keep-arriving) pass with unchanged asserted behavior (the
      test file itself may need mechanical updates for any sim-harness API
      change).
- [ ] This ticket does not introduce any behavior that would make ticket
      009's radio-specific bench check fail (e.g. `feed()` must not be
      delayed beyond the same slack iteration a statement arrived in,
      regardless of the yield).

## Implementation Plan

**Approach.** This ticket is a single, indivisible cutover: `source/main.cpp`
and `tests/_infra/sim/sim_api.cpp` are rewritten together, in the same
commit, per the 1:1-mirror invariant Grounding confirms holds today. The
watchdog's emergency-bypass mechanism (exact method name/shape) is an
implementation-time decision — `architecture-update.md` deliberately stays
at module level and does not prescribe it. The programmer may choose
between (a) keeping a narrow `apply()`-style immediate-write method on
`Hardware`/`Drivetrain` reserved for this one caller, or (b) a dedicated
`emergencyNeutralize()`-style method added to each faceplate; either is
acceptable as long as it demonstrably bypasses `driveIn`/`motorIn`.

**Files to delete:**
- `source/dev_loop.h`, `source/dev_loop.cpp`

**Files to modify:**
- `source/main.cpp`
- `tests/_infra/sim/sim_api.cpp`
- `source/subsystems/{drivetrain,hardware,nezha_hardware,sim_hardware}.h`/`.cpp`
  (to add/preserve the emergency-bypass method)
- `source/commands/dev_commands.h` (`SerialSilenceWatchdog` itself is
  unchanged internally — confirm it lifts as-is into the new loop; only
  `DEV WD`'s routing mechanism changes, per the acceptance criteria above)

**Testing plan:**
- Run `tests/sim/unit/test_watchdog_policy.py`, `test_determinism.py`
  (synchronous-update order-independence, SUC-001), and the full
  `tests/sim/unit/` + `tests/sim/system/` suites for regression.
- Add a new test asserting the watchdog's neutralize is visible in the
  **same pass** the window expires in, not the next one (bypasses one full
  tick of `driveIn`/`motorIn` latency).
- **Verification command**: `uv run pytest tests/sim/unit/test_watchdog_policy.py tests/sim/unit/test_determinism.py` then the full suite `uv run pytest tests/sim`

**Documentation updates:** None to `docs/protocol-v2.md` (`DEV WD`'s wire
contract is unchanged). Optionally refresh
`.claude/rules/hardware-bench-testing.md`'s stale pre-v2 quick-smoke table
against the post-rearchitecture command surface — not required for this
ticket's acceptance (explicitly out of scope per that file's own note,
unless the team-lead requests it separately).
