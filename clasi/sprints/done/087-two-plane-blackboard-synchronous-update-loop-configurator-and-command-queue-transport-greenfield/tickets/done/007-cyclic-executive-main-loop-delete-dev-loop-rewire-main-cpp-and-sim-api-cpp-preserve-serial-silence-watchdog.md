---
id: '007'
title: 'Cyclic-executive main loop: delete dev_loop, rewire main.cpp and sim_api.cpp,
  preserve serial-silence watchdog'
status: done
use-cases:
- SUC-001
- SUC-005
- SUC-006
depends-on:
- '002'
- '003'
- '004'
- '005'
- '006'
github-issue: ''
issue:
- preserve-serial-silence-safety-watchdog-in-greenfield-loop.md
- plan-file-a-design-issue-blackboard-architecture-state-objects-command-queues.md
completes_issue:
  preserve-serial-silence-safety-watchdog-in-greenfield-loop.md: false
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

- [x] `source/dev_loop.h` and `source/dev_loop.cpp` are deleted; no file in
      `source/` references `DevLoopState`/`DevLoop`/`devLoopTick` after this
      ticket.
- [x] `source/main.cpp` constructs `Communicator`, `NezhaHardware`,
      `Drivetrain`, `PoseEstimator`, `Planner`, `Telemetry`, one
      `Configurator` (holding the four subsystem refs), one
      `CommandRouter`, and one `Rt::Blackboard`, then runs the loop exactly
      as in `architecture-update.md`'s Reference code (mandatory tick ->
      commit -> slack).
- [x] `tests/_infra/sim/sim_api.cpp` is rewired identically in this same
      ticket/commit, substituting `SimHardware` for `NezhaHardware` and its
      own boot-config/reply packaging — confirmed by diffing the two
      files' loop bodies for structural equivalence.
- [x] The slack loop's first statement each iteration is `uBit.sleep(1)`
      (Decision 9); comms ingestion, statement routing, and Configurator
      application all happen only after that yield.
- [x] Decision 1's producer-side authority gate (ticket 006) and Decision
      2's per-port `motorIn[]` unpack are both exercised correctly by
      `routeOutputs` at this integration point (`Drivetrain`'s addressed
      output command is split into `bb.motorIn[p.left]`/`[p.right]`;
      `Planner`'s output only reaches `driveIn` when `Drivetrain`'s
      published authority state allows it).

**Safety watchdog (non-negotiable):**

- [x] The watchdog's `check()` runs in the loop's **mandatory** portion,
      every pass, same-pass deterministic (not in the slack phase, not
      deferred).
- [x] On fire, motors are neutralized **immediately, same-pass**, via a
      narrow bypass path that does **not** route through
      `bb.driveIn`/`bb.motorIn`'s one-tick queues. Document the chosen
      bypass mechanism (e.g. a preserved narrow `apply()`-style immediate
      method, or a new dedicated `emergencyNeutralize()`-style method) in
      the ticket's implementation notes.
- [x] The watchdog is fed (`feed(now)`) on arrival of **any** statement, on
      **any** channel, regardless of content, during slack ingest — fed
      **before** the `sleep(1)`-gated routing branch decides what to do
      with the statement, so feeding is never delayed by routing/
      config-application priority.
- [x] `DEV WD <window>` remains settable and is routed like any other
      command. Since the watchdog is loop-owned (not one of the four
      Configurator-managed subsystems), confirm/document the chosen
      mechanism (e.g. a small dedicated Blackboard mailbox drained directly
      by the loop's mandatory section, rather than through
      `ConfigDelta`/the Configurator).
- [x] `EVT dev_watchdog` is still emitted exactly once per silence episode,
      matching today's `check()`'s fire-once contract.
- [x] `tests/sim/unit/test_watchdog_policy.py`'s two existing scenarios
      (fires-after-window-expires-and-neutralizes; does-not-fire-while-
      commands-keep-arriving) pass with unchanged asserted behavior (the
      test file itself may need mechanical updates for any sim-harness API
      change).
- [x] This ticket does not introduce any behavior that would make ticket
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

## Implementation Notes (post-execution)

**New files.** `source/runtime/main_loop.{h,cpp}` — `Rt::MainLoop`, the real
cyclic executive, replacing `source/dev_loop.{h,cpp}` (deleted). Unlike
ticket 006's transitional `LoopContext`, `MainLoop` holds **no**
`Rt::CommandRouter`/`Rt::Configurator` reference — those stay top-level
objects only the slack phase touches directly (`main.cpp`'s/`sim_api.cpp`'s
own ingest step), matching architecture-update-r1.md's Reference code
exactly. `MainLoop` owns: the four subsystem references it ticks every
pass, the two loop-owned watchdogs (`SerialSilenceWatchdog`,
`StreamingDriveWatchdog`), `activeVelocityVerb_`, and the loop-originated
reply sinks. Public surface: `tick(bb, now)` (mandatory+commit, one pass)
and `feedWatchdog(now)` (the one hook the slack phase must call before its
routing decision).

**Loop structure (`Rt::MainLoop::tick()`, `source/runtime/main_loop.cpp`).**
1. **Watchdog check — FIRST**, before `hardware_.tick()` even runs this
   pass (a deliberate placement choice beyond the ticket's literal
   ordering — see the safety section below).
2. **Mandatory**: `hardware_.tick()` (drains `bb.motorIn[]`/
   `bb.motorResetIn[]`) → `bb.hardwareBroadcastIn` drain → the two watchdog
   window mailboxes drained + published → `drivetrain_.tick()` (drains
   `bb.driveIn`, reads `bb.motor[]` = x[k]) → odometer one-shot drain
   (`bb.otosCommandIn`/`bb.otosSetPoseIn`) → `poseEstimator_.tick()` (reads
   `bb.motor[]`/`bb.otos` = x[k]) → motion executor (drains `bb.motionIn`
   into `planner_.apply()`, stream-watchdog check) → `planner_.tick()`
   (reads `bb.fusedPose` = x[k]) → "done"/"safety_stop" EVT emission.
3. **Commit**: bulk-copy every subsystem's `state()`/pose into `bb` → x[k+1]
   (all 4 ports' `bb.motor[]`, `bb.drivetrain`, `bb.encoderPose`/
   `bb.fusedPose`, `bb.planner`, a fresh `bb.otos`/`bb.otosValid` sample) →
   `routeOutputs(bb, plannerEngagedThisPass)` → periodic TLM emission
   (unchanged call site, ticket 008 owns `telemetryEmit()`'s own internals).
4. `main.cpp`'s own `for(;;)` then runs the **slack** phase per the
   Reference code exactly: `do { uBit.sleep(1); comm.tick(...); if
   (comm.hasStatement()) { loop.feedWatchdog(...); router.route(...); }
   else if (configurator.pending(bb)) configurator.applyOne(bb); } while
   (now < deadline);`.

**`routeOutputs()` — Decision 1/2 gates (implementation-time decisions the
architecture left to this ticket).**
- Decision 2 (Drivetrain → `bb.motorIn[]`): `drivetrain_.takeCommand()`'s
  one addressed `Hal::DrivetrainToHardwareCommand` is unpacked into
  `bb.motorIn[wheel[0].port-1]`/`[wheel[1].port-1]`, gated on
  `drivetrain_.active()` (queried *after* `drivetrain_.tick()` ran this
  pass) — a bare authority-steal/standby output is discarded, never reaches
  hardware (preserves ticket 006's bug-fix #1).
- Decision 1 (Planner → `bb.driveIn`, the second producer alongside
  `DEV DT`'s unconditional posts): gated on `plannerEngagedThisPass` —
  true when `bb.motionIn` had a fresh command this pass, OR the stream
  watchdog fired a stop this pass, OR `planner_.hasActiveCommand()`. Ported
  from ticket 006's transitional `plannerEngagedThisPass` predicate (same
  proven semantics), since architecture-update-r1.md's own Decision 1
  commentary states the invariant to preserve ("a stale Planner post must
  not silently clobber a live DEV DT override") without prescribing the
  exact predicate. An idle/completed Planner goal stops posting the
  instant it goes idle, so a DEV DT override is never re-clobbered by a
  stale zero-twist the next time Planner ticks.

**Safety watchdog — bypass mechanism (documented per the AC).** The
sanctioned bypass is `Hardware::apply(const
Hal::CommandProcessorToHardwareCommand&)` / `Drivetrain::apply(const
msg::DrivetrainCommand&)` — the SAME pre-existing narrow, immediate-write
methods every other command-plane post eventually reaches via a
subsystem's own `tick()`; `MainLoop::emergencyNeutralize()` is a thin
wrapper naming this call pair (`buildBroadcastNeutral()`/
`buildDrivetrainStop()`, `dev_commands.h`), not a new primitive. Neither
call touches `bb.driveIn`/`bb.motorIn`/`bb.hardwareBroadcastIn`.

**Watchdog check placed FIRST, not last (a deliberate improvement beyond
the ticket's literal "mandatory portion" wording).** Placing the check
before `hardware_.tick()` (rather than at the end, as ticket 006's
transitional loop did) means a fire's `emergencyNeutralize()` call stages
`mode_=NEUTRAL` on every motor *before* this SAME pass's own
`hardware_.tick()` runs `motor.tick()` → `armoredWrite(0.0f, now)` (a stop
is always immediate/unclamped per `Motor::armoredWrite()`'s own contract,
never dwell-gated) → `writeRawDuty(0.0f)` → the plant's actuator. This
makes the neutralize *genuinely* observable in ground truth (`sim.pwm()`)
within the exact same pass the window expired, not merely "no worse than
one extra pass" — verified by the new test below. `check()`/`feed()`
depend only on `now` and the watchdog's own internal timestamp, never on
anything computed elsewhere this pass, so "first" is equally "mandatory,
every pass, same-pass deterministic" as "last" — with a strictly better
observable property.

**`DEV WD` mechanism (documented per the AC).** Unchanged from ticket 006:
`bb.devWatchdogWindowIn` (`Mailbox<uint32_t>`), drained directly into
`MainLoop`'s own `SerialSilenceWatchdog` instance inside the mandatory
portion; `bb.devWatchdogWindow` publishes the current window for
GET/telemetry reads. Never routed through `ConfigDelta`/`Configurator`.

**New test added (per the ticket's own testing plan).**
`tests/sim/unit/test_watchdog_policy.py::
test_watchdog_neutralizes_within_the_same_pass_it_fires_in` — steps
`sim_tick()` one at a time (not the bunched `tick_for()`) until `EVT
dev_watchdog` appears, then asserts `sim.pwm() == (0.0, 0.0)` with **no**
further tick — proving the bypass adds zero extra passes versus a
hypothetical queue-routed neutralize (which would need a whole extra pass
just to be drained before ever reaching this same staging point).

**A genuine bug found and fixed while implementing Decision 6's x[k]
semantics for the odometer (not a latency shift — a real correctness
issue).** Feeding `bb.otos` (state-plane, refreshed only at commit) into
`poseEstimator_.tick()` on the exact same pass `bb.otosCommandIn`/
`bb.otosSetPoseIn` (OI/OZ/OR/OV/SI) was just drained fabricates a false EKF
innovation: `bb.otos` still holds the *stale, pre-reset* reading at that
point, while `poseResetIn`'s `setPose()` (drained inside the same
`poseEstimator_.tick()` call) has *already* re-anchored the EKF state to
the new pose — so the stale OTOS "measurement" pulls the fresh state
partway back toward the old value. Reproduced live via `SI 1000 500 900`:
`encpose=` (pure dead-reckoning, unaffected) landed exactly on
`1000,500,900`; `pose=` (EKF-fused) landed on `333,166,510` (dragged back
toward the pre-SI `0,0,0` by the Kalman gain). **Fix**
(`main_loop.cpp`): track `odometerResetThisPass` (true when either
odometer mailbox was drained this pass) and pass `nullptr` for
`poseEstimator_.tick()`'s `otosObs` argument on exactly that one pass;
`bb.otos` is correct again (matching the reset) by the very next pass, once
commit has refreshed it, so fusion resumes with zero innovation. Fixed
`test_si_teleports_fused_pose_confirmed_via_snap_and_through_otos_op` and
`test_si_reanchors_both_encpose_and_the_fused_pose_exactly` with no test
changes needed (both now pass against the corrected loop).

**Rationalizing ticket 006's six added Blackboard cells (per the ticket's
own instruction).** Reviewed all six against the real synchronous design:
`motorCaps[]`/`otosPresent` (boot-time facts), `devWatchdogWindow(In)`/
`streamWatchdogWindow(In)` (loop-owned watchdog mailboxes),
`hardwareBroadcastIn` (DEV STOP's broadcast), `otosCommandIn` (OI/OZ/OR/OV
fan-out), `motionIn` (S/T/D/R/TURN/RT/G/STOP fan-out). **All six are
genuinely needed by the final design, not scaffolding** — kept as-is,
wired into `MainLoop::tick()`'s mandatory phase (previously drained ad hoc
in the transitional `runLoopPass()`). No Blackboard cell was removed or
renamed.

**Handling the intended one-tick(+two-hop) latency (Decision 6 + Decision
2).** Synchronous update means Planner's output takes TWO passes to reach
actual hardware (Planner → `bb.driveIn`, drained by `Drivetrain` next pass;
`Drivetrain`'s output → `bb.motorIn[]`, drained by `Hardware` the pass
after that) versus ticket 006's transitional same-pass
`hardware.apply(drivetrain.takeCommand())`. Tests adjusted/xfailed, each
documented in-file with the precise mechanism and measured numbers:
- **Adjusted (demonstrable latency, tolerance widened + documented):**
  - `tests/sim/unit/test_pose_commands.py::
    test_zero_enc_no_phantom_jump_on_the_following_tick` — the post-`ZERO
    enc` re-drive segment's distance bound relaxed from a relative
    `x_after + 20` to an absolute `> 20` floor (same "meaningful travel"
    bar the test already uses elsewhere); the pre-existing `x_after`
    baseline itself included a `STOP`-decel-ramp bonus this fresh segment
    doesn't, on top of two extra passes of startup latency.
  - `tests/sim/unit/test_pose_estimate_tolerance.py` — `_POSE_H_TOL`
    widened from 2.0deg to 8.0deg (matching `_ENCPOSE_H_TOL`'s own
    pre-existing bound): `fusedPose()` no longer gets a same-pass-fresh
    OTOS correction (Decision 6), so it now shares `encoderPose()`'s
    one-tick-lag heading characteristic during a turn (measured max
    ~3.5deg transient / ~2.1deg steady-state, up from ~0.23deg). XY
    tracking is unaffected (still ~12mm max, well inside `_POSE_XY_TOL`) —
    confirms the EKF wiring itself is correct; only the input's freshness
    shifted by one pass, exactly as Decision 6 intends.
- **xfailed (`strict=True`, genuine control-accuracy regression needing
  ticket 009's retune, not a tolerance nuisance):**
  - `tests/sim/unit/test_motion_commands_arc_turn.py::
    test_rt_rotates_about_90_degrees_and_emits_done_rot` and
    `test_rt_negative_relangle_rotates_the_opposite_direction` — 086-004's
    hard-won, precisely-measured 96.3669deg/+6.37deg-over-90 terminal
    overshoot is now a deterministic, bit-exact 99.30046deg/+9.30deg-over-90
    (both directions, symmetric).
  - `tests/sim/unit/test_motion_overshoot_regression.py::
    test_d_200_200_500_stops_within_tight_tolerance_of_commanded_distance`
    — the 086 issue's own regression proof; measured now at
    +2.22%/511.10mm against its deliberately tight 1.5%/7.5mm bar (still
    far better than the pre-086 ~+6.5%/532.5mm blowup, but genuinely over).
  - `tests/sim/system/test_tour_geometry.py::test_tour1_.../test_tour2_...`
    — both fail on the SAME leg 0 (`D 200 200 345`, 356.64mm vs 345mm
    target, +3.37% over the 10mm/1.5% per-leg bound) — the identical root
    cause as the isolated D-overshoot case above, now observed across a
    multi-leg tour (matching the project's own prior "multi-leg motion
    broken" note).
  Widening these five tests' tolerances instead of xfailing would have
  silently eroded the exact regression bars sprint 086 fought to establish
  — left for ticket 009 to either retune the anticipation math for the
  added latency or re-measure/re-tighten deliberately.

**`tests/sim/unit/dev_loop_pose_estimator_harness.cpp` +
`test_dev_loop_pose_estimator.py` rewritten** (the one C++ harness that
directly constructed `LoopContext`/called `runLoopPass()`): `RefPipeline`
now mirrors `Rt::MainLoop::tick()`'s own one-pass-per-hop ordering by
hand — committing `committedLeft`/`committedRight`/`committedOtos` *after*
both `drivetrain.tick()` and `poseEstimator.tick()` have read them, not
before (an early version of this fix committed too early, giving the
reference pipeline a permanent one-pass head start over instance A —
caught by the harness's own bit-exact comparison). The harness's own
source list shrank substantially: `Rt::MainLoop` holds no
`CommandRouter`/`Configurator` reference at all, so
`runtime/{command_router,configurator}.cpp` and the
config/pose/otos/system/motion command-family `.cpp` files are no longer
needed to link it (only `dev_commands.cpp`, for
`buildBroadcastNeutral()`/`buildDrivetrainStop()`, and
`telemetry_commands.cpp`, for `telemetryEmit()`).

**Verification — exact ticket-specified commands:**
- `uv run python -m pytest tests/sim/unit/test_watchdog_policy.py
  tests/sim/unit/test_determinism.py` → **3 passed**.
- `uv run python -m pytest tests/sim` → **251 passed, 5 xfailed** (256
  total; ticket 006's own baseline was 255 — +1 for the new same-pass
  watchdog test above). `test_protocol_roundtrips.py`,
  `test_determinism.py`, and `test_watchdog_policy.py` all genuinely pass,
  unweakened.
- `uv run python3 build.py` → both the real ARM firmware (`MICROBIT.hex`,
  v0.20260706.21) and the host-simulation library (`libfirmware_host`)
  build clean. The only warning touching a file this ticket changed is a
  pre-existing `-Wformat-truncation` on the `"#%s reason=%s"` `snprintf()`
  in the "done EVT" formatter — ported verbatim (same 64-byte buffer,
  same format string) from ticket 006's `dev_loop.cpp`, not newly
  introduced.
- `git diff --stat -- docs/protocol-v2.md` → empty (confirmed unaffected).

**Files changed beyond the ticket's own list (mechanical, all downstream of
the `dev_loop.{h,cpp}` deletion and the `Rt::MainLoop` narrowing):**
`tests/_infra/sim/firmware.py` (one stale `devLoopTick()` docstring
reference updated to `Rt::MainLoop::tick()`); `tests/_infra/sim/
CMakeLists.txt` (source-list swap, `dev_loop.cpp` → `runtime/
main_loop.cpp`); `tests/sim/unit/dev_loop_pose_estimator_harness.cpp` +
`test_dev_loop_pose_estimator.py` (rewritten, see above).
