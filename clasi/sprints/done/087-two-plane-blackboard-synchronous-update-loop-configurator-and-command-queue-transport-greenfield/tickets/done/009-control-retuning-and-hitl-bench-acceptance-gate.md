---
id: 009
title: Control retuning and HITL bench acceptance gate
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
depends-on:
- 008
github-issue: ''
issue:
- plan-file-a-design-issue-blackboard-architecture-state-objects-command-queues.md
- preserve-serial-silence-safety-watchdog-in-greenfield-loop.md
completes_issue:
  preserve-serial-silence-safety-watchdog-in-greenfield-loop.md: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Control retuning and HITL bench acceptance gate

## Description

Close out sprint 087. This ticket directly depends on ticket 008 and
therefore, transitively, on every prior ticket (001-008) being done. Five
things happen here:

1. **Verify/adjust control tuning** for the added one-tick synchronous-update
   latency (Decision 6, Open Question 1 in `architecture-update.md`) against
   the sim and the hardware bench — adjust `Drivetrain`/`Planner` gains or
   thresholds **only if** a real regression is observed, not pre-emptively.
2. **Run the full existing regression suite** (`tests/sim/unit/`,
   `tests/sim/system/`) to confirm the whole rearchitecture is
   behavior-preserving at the wire level.
3. **Execute the hardware-bench gate** per
   `.claude/rules/hardware-bench-testing.md` **and** this sprint's own
   radio-specific requirement (Decision 9): deploy to the robot on the
   stand, confirm every sensor responds, confirm wheels drive and encoders
   increment in both directions, and round-trip at least one command over
   the **radio relay** specifically (not only serial — a serial-only pass
   cannot catch a missing slack-loop yield).
4. **Confirm the serial-silence safety watchdog** neutralizes on the stand
   under comms silence, exercised over the radio path specifically — this
   is the linked watchdog issue's own Bench/HITL acceptance criterion, and
   this ticket is what finally closes that issue out (ticket 007 delivered
   the sim-side/same-pass correctness; `completes_issue` was set `false`
   for that issue on ticket 007 for exactly this reason).
5. **Confirm sim/hardware parity** — the same command sequence produces
   equivalent behavior in `tests/_infra/sim` and on the real robot, within
   the tolerances the existing bench scripts already use.

## Acceptance Criteria

- [x] Full `tests/sim/unit/` and `tests/sim/system/` suites pass. (253
      passed, 4 xfailed, 0 failed — see completion notes; the 4 remaining
      xfails are an honest, documented partial recovery, not failures.)
- [x] Determinism/order-independence (SUC-001) is re-confirmed against the
      **full rebuilt loop** (not just the isolated primitives from ticket
      007) — re-ordering the mandatory-tick call sequence produces
      bit-identical `x[k+1]` for a fixed `x[k]` and fixed inputs. (New
      `tests/sim/unit/main_loop_order_independence_harness.cpp` +
      `test_main_loop_order_independence.py` — see completion notes.)
- [x] Control tuning is verified against the sim first; any regression
      attributable to the one-tick latency is documented with before/after
      values and the specific gain/threshold changed, in this ticket's
      completion notes.
- [ ] Hardware-bench gate (`.claude/rules/hardware-bench-testing.md` items
      1-3) passes on the stand: sensors alive (encoders, OTOS, line, color,
      digital/analog), wheels drive both directions with encoders
      incrementing proportionally, round-trip over the actual transport.
      (deferred — stakeholder bench on master; see Bench Checklist below)
- [ ] The round-trip-over-transport check is performed specifically over
      the **radio relay** (not serial-only), confirming Decision 9's yield
      is present and effective in the shipped firmware.
      (deferred — stakeholder bench on master; see Bench Checklist below)
- [ ] Comms-silence safety watchdog: on the stand, with no statement
      arriving for longer than the configured window over the **radio**
      path, the wheels neutralize and `EVT dev_watchdog` is observed;
      feeding a statement re-arms it. (Closes the linked watchdog issue's
      Bench/HITL acceptance criterion.)
      (deferred — stakeholder bench on master; see Bench Checklist below)
- [ ] Sim/hardware parity: a representative command sequence (e.g. a short
      drive + turn + `SI` + `ZERO enc` sequence) produces equivalent
      encoder/pose behavior in sim and on the bench, within existing
      tolerance bands.
      (deferred — stakeholder bench on master; see Bench Checklist below)
- [x] Sprint 087's full acceptance bar (every SUC-001 through SUC-006
      acceptance criterion in `usecases.md`) is confirmed met end-to-end,
      not just per-ticket, **for everything verifiable off-hardware**; the
      HITL-only criteria (sensors/drive/radio-relay/watchdog/parity) are
      the stakeholder's own deferred bench step — see Bench Checklist.

## Implementation Plan

**Approach.** This ticket is verification-and-tuning, not new production
architecture — it may complete with zero source changes beyond tuning
constants, which is an expected, successful outcome (per sprint 085's own
precedent for verification-first tickets), **not** a sign of misscoping.

**Files to modify:** possibly `source/subsystems/drivetrain.{h,cpp}` or
`planner.{h,cpp}`'s gain/threshold constants only, if retuning proves
necessary; no structural changes expected.

**Testing plan:**
- Run the full existing automated suite.
- Execute the hardware-bench gate live (radio + serial + watchdog +
  sensors + drive), per `.claude/rules/hardware-bench-testing.md`'s
  "Standing verification gate" — this sprint is explicitly one of the
  firmware sprints that gate applies to (it touches the HAL, motor
  control, sensing, and the command protocol's internal transport).
- Confirm sim/bench parity with a scripted comparison run.
- **Verification command**: `uv run pytest tests/sim` then the live bench sequence per `.claude/rules/hardware-bench-testing.md`.

**Documentation updates:** Consider refreshing
`.claude/rules/hardware-bench-testing.md`'s stale pre-v2 quick-smoke table
(flagged stale in the file itself) against the post-rearchitecture command
surface — optional stretch, not required for this ticket's acceptance
unless the team-lead requests it. Not done this ticket (out of the
autonomous scope given to the implementer); the Bench Checklist below
covers the current command surface directly instead.

## Implementation Notes (post-execution)

### Xfail recovery — before/after values and exact constants changed

**Root cause, common to all 5 xfails.** `Planner::applyStopAnticipation()`'s
STOP_DISTANCE/STOP_ROTATION caps (086-003) compute `vCap/omegaCap =
sqrt(2*a*remaining)` — the classic "you can still stop within `remaining`
at deceleration `a`" bound, correct ONLY if the commanded speed takes effect
immediately. Ticket 087-007's synchronous-update rearchitecture added a
**two-pass** dead time between Planner computing that cap and it actually
reaching the wheel (Planner's output -> `bb.driveIn`, drained by Drivetrain
next pass; Drivetrain's own output -> `bb.motorIn[]`, drained by Hardware
the pass after THAT) — so the un-compensated cap lets the robot travel
`remaining` further than it should before the lower speed ever lands,
producing the added overshoot ticket 007 xfailed.

**The fix.** `Planner::applyStopAnticipation()` (`source/subsystems/
planner.cpp`) now solves the closed-form "stopping distance with a reaction
time" formula (same shape as the highway stopping-sight-distance formula,
`d = v*T + v^2/(2a)`, solved for `v` given `d = remaining`):

```
reach = a * kDeadTime
vCap  = -reach + sqrt(reach^2 + 2*a*remaining)
```

where `kDeadTime = kOutputHops(2.0) * kAssumedPassPeriod(0.020s) = 0.040s`
— a **fixed constant** matching `main.cpp`'s own `kPeriod` (NOT the tick's
own measured `now` delta — see "A genuine bug found" below for why that
matters). Applied to both the STOP_DISTANCE branch (`a = a_decel`) and the
STOP_ROTATION branch (`a = yaw_acc_max`, the same pre-existing
documented-approximation domain 086-003 already used). STOP_HEADING (TURN)
is left unmodified — its own tests already pass without it.

**A genuine bug found and fixed while developing this formula (not just a
tuning miss — a real algorithmic error).** The first version of this fix
subtracted `measuredWheelVelocity * deadTime` from `remaining` directly (a
"predict the coast distance from the CURRENTLY MEASURED speed" idea). This
closes a feedback loop through the plant's own delayed response: as the cap
drives commanded speed down, the measured-speed term shrinks, which relaxes
the cap, which lets speed climb back up. Caught by tracing `D 200 200 700`'s
terminal approach tick-by-tick: velocity dipped to `-0.54mm/s` at
`t=3288ms`, then REBOUNDED to `72.5mm/s` by `t=3432ms`, with the DISTANCE
stop firing mid-rebound at `66.4mm/s` — a genuine limit-cycle oscillation,
not smooth braking. Root cause: using the CURRENTLY MEASURED speed as an
input to the same cap that determines that speed, through a 2-pass delay,
is textbook dead-time-induced instability. Fixed by switching to the
closed-form quadratic above, which depends only on `remaining`
(monotonically shrinking, from real encoder progress) and fixed constants —
no plant-velocity feedback path, hence no oscillation (re-verified via the
same dense trace: monotonic approach, no dip/rebound). A second, related
bug in the SAME first attempt: using the tick's own *measured* `dt` (rather
than a fixed constant) as the pass-period proxy broke `planner_harness.cpp`'s
own scenarios, which deliberately advance `now` by non-representative
(1-second) steps to force convergence within a single call — `kDeadTime` is
a fixed constant for exactly this reason.

**Recovered (2 of 5):**

| Test | Before (087-007, un-fixed) | After (this ticket) | Bound |
|---|---|---|---|
| `test_motion_overshoot_regression.py::test_d_200_200_500_stops_within_tight_tolerance_of_commanded_distance` | 511.10mm / +2.22% | **502.27mm / +0.45%** | 1.5% (7.5mm) |
| `tests/sim/system/test_tour_geometry.py` — every D leg, both tours (e.g. Tour 1 leg 0, `D 200 200 345`) | 356.64mm / +3.37% (leg 0) | passes every D leg's distance/heading check in both tours (confirmed via `--runxfail`) | `max(10mm, 1.5%)` per leg |

**Left xfail — honest partial recovery (3 of 5), with reasons:**

| Test | Value (unchanged by this fix) | Why the fix doesn't reach it |
|---|---|---|
| `test_motion_commands_arc_turn.py::test_rt_rotates_about_90_degrees_and_emits_done_rot` | 99.30046deg / +9.30deg-over-90 | See below |
| `test_motion_commands_arc_turn.py::test_rt_negative_relangle_rotates_the_opposite_direction` | -99.30046deg / -9.30deg-over-90 | See below |
| `tests/sim/system/test_tour_geometry.py` (both) | fails on the same RT-leg heading check as the isolated case above (Tour 1/2 leg 1, `RT 9000`, 99.30deg) | See below |

The SAME closed-form STOP_ROTATION cap was applied (it's in the shipped
code — see planner.cpp's own comment) but makes **zero** measurable
difference for RT: at RT's commanded rate (`kRotationOmega`,
`motion_commands.cpp`, ~1.745 rad/s) and this config's `yaw_acc_max` (20
rad/s^2), the cap only binds inside the last ~0.076mm of a ~100mm per-wheel
arc (90deg at the default 128mm trackwidth) — far below one tick's own
~2.7mm of arc travel — so it is provably a no-op both before and after this
fix, confirming 086-003's own completion notes ("does not close RT's own
overshoot to near-zero the way it closed D 200 200 500's"). RT's actual
overshoot driver — both the original 6.37deg (086-004) and this sprint's
added +2.93deg — is the **SMOOTH ramp-down's post-fire coast**: `Planner`
reports "done" the instant its OWN ramp converges to `(0,0)`
(`ramp_.atTarget()`), but the actuator is still 2 passes behind (Decision
6), so the wheel keeps coasting a bit further before it physically catches
up. Compensating that would mean changing **when** `done`/EVT fires
relative to ramp convergence — shared machinery every goal kind's SMOOTH
stopping phase uses, not a Planner/Drivetrain gain or threshold — a
materially bigger, higher-blast-radius change than this ticket's scoped
retuning, risking the hard constraint against regressing the passing
suite. Rescaling STOP_ROTATION's cap to bind meaningfully at these config
values would require an arbitrary, dimensionally-unmotivated fudge factor
(the already-mixed mm/rad-per-s^2 units have no natural scale to anchor
one) — exactly the "aggressive over-tuning" this ticket is authorized to
decline. Left xfail, `strict=True` markers kept, reasons updated in-file
with this full explanation (`test_motion_commands_arc_turn.py`,
`test_tour_geometry.py`) so the next attempt starts from this diagnosis
instead of re-deriving it.

**Mechanical test updates (not a masked regression — the harness itself
tests the EXACT formula this ticket intentionally changed).**
`tests/sim/unit/planner_harness.cpp`'s two 086-003 scenarios
(`scenarioDistanceGoalAnticipatesStopWithSpeedCap`,
`scenarioRotationGoalAnticipatesStopWithRateCap`) hard-code expected cap
values computed from the un-compensated `sqrt(2*a*d)` formula; their Tick-3
expected values are updated to the new closed-form's own hand-computed
numbers (`63.2456->34.8331` mm/s; `1.09545->1.01836` rad/s — both
recomputed and documented in-file with the exact arithmetic), with the "cap
does not bind far from the stop" Tick-2 checks confirmed unaffected either
way.

### Determinism / order-independence (SUC-001) re-confirmed against the full loop

New: `tests/sim/unit/main_loop_order_independence_harness.cpp` +
`tests/sim/unit/test_main_loop_order_independence.py`. Ticket 002's own
proof (`runtime_blackboard_harness.cpp`) only round-trips the
`Mailbox`/`WorkQueue` **types**; ticket 007's own proof
(`dev_loop_pose_estimator_harness.cpp`) runs the REAL `Rt::MainLoop::tick()`
against a hand-mirrored reference that copies its **same** fixed call
order, for PoseEstimator only — neither actually swaps call order and diffs
the full result. Since `Rt::MainLoop::tick()` itself has one hardcoded
internal order (can't be permuted without invasive surgery), this new
harness builds two independent, by-hand-driven pipelines (`OrderedPipeline`,
wiring the four REAL subsystems: `SimHardware`/`Drivetrain`/
`PoseEstimator`/`Planner`) from byte-identical configs — one ticking
FORWARD (`hardware, drivetrain, poseEstimator, planner` — matching
`Rt::MainLoop::tick()`'s own order) and one REVERSE (`planner,
poseEstimator, drivetrain, hardware`) — with `routeOutputs`/commit run
strictly after all four ticks in both, exactly like the real loop. Drives a
real Drivetrain WHEELS-authority twist AND a real Planner DISTANCE goal (so
this ticket's own retuned STOP_DISTANCE anticipation runs under the test,
too) for 120 passes (20ms apart), asserting BOTH ports' `MotorState`,
`fusedPose`, `DrivetrainState.active`, and `PlannerState`
(`active`/`body_twist`) are bit-identical between FORWARD and REVERSE after
**every single pass** — confirmed: **0 assertion failures across 120
passes**, plus a sanity check that the goal actually ran to completion and
produced real motion (not a trivially-passing idle setup).

### Full regression

- `uv run python -m pytest tests/sim` -> **253 passed, 4 xfailed** (0
  failed) — up from ticket 007/008's own 251 passed/5 xfailed baseline:
  +1 recovered xfail (the D-mode case), +1 new test (the determinism
  harness above).
- `uv run python3 build.py` -> both the real ARM firmware (`MICROBIT.hex`,
  v0.20260706.21 — unbumped per this ticket's own commit instructions) and
  the host-simulation library (`libfirmware_host`) build clean. No new
  warnings touching any file this ticket changed (the linker warnings
  present are pre-existing vendor-libc noise, `_fstat`/`_isatty` not
  implemented, unrelated to this ticket).

### HITL bench acceptance — deferred to the stakeholder (see Bench Checklist)

Per this ticket's own dispatch instructions, the physical hardware-bench
ACs above are the stakeholder's authorized on-stand step, not something
this agent flashes/drives. `completes_issue` is set to
`{preserve-serial-silence-safety-watchdog-in-greenfield-loop.md: false}` —
that issue's own Bench/HITL acceptance criterion is exactly what the Bench
Checklist below exists to close out; it must not auto-archive until the
stakeholder actually runs it. The design issue (`plan-file-a-design-issue-
blackboard-architecture-state-objects-command-queues.md`) completes
normally with this ticket (unlisted in the `completes_issue` map).

## Bench Checklist (stakeholder's on-master HITL step)

Run this on the stand, after merging to `master`, per
`.claude/rules/hardware-bench-testing.md` and `docs/protocol-v2.md`. The
robot is wheels-off-the-ground on the stand — safe to spin freely.

### (a) Deploy

```bash
just build-clean                                   # clean ARM build -> MICROBIT.hex
mbdeploy probe                                      # refresh the connected-device registry
mbdeploy deploy <full-UID> --hex MICROBIT.hex        # flash the ROBOT (not a relay dongle --
                                                     #   mbdeploy list's ROLE column confirms)
```

(`mbdeploy deploy --build` is currently broken — its own venv lacks
`grpcio-tools`/`protobuf`; use the explicit `--hex` form above, per
`.clasi/knowledge/hitl-bench-mbdeploy-build-and-watchdog.md`.)

### (b) Sensors respond

Open the serial port (or a `DEV`-family session) and confirm every sensor
gives plausible, CHANGING values:

```
PING                          -> PONG (or protocol v2's equivalent liveness reply)
GET otos                      -> position/velocity reads, changing as the wheels turn (below)
GET line                      -> 4 channel values
GET color                     -> RGBC values
SNAP                          -> one TLM line with enc=/pose=/otos=/vel= all populated
```

### (c) Wheels drive both directions, encoders increment

```
ZERO enc                      -> ACK; SNAP shows enc=0,0 (or equivalent zeroed report)
D 200 200 300                 -> wheels spin forward; SNAP/STREAM enc values climb
                                  proportionally to commanded speed
D -200 -200 300                -> wheels spin the OTHER direction; encoders move the
                                  opposite way
STOP  (or D-mode's own stop)   -> EVT done; wheels settle
```

Confirm encoder deltas are proportional to commanded speed/distance in
BOTH directions (not just sign-correct).

### (d) Round-trip over the RADIO RELAY specifically (Decision 9's yield)

Serial-only is not sufficient — a loop that starves radio (a busy-wait
regression in the slack loop's `uBit.sleep(1)`) would still pass a
serial-only check, since serial RX is IRQ-driven and radio RX is not (it
needs a fiber yield to run `Radio::onData`). Connect over the radio relay
specifically (open the relay with DTR asserted, send `!GO`, then plain
commands — no `>` prefix; see `.clasi/knowledge/` for the relay `!GO`
protocol) and repeat a subset of (b)/(c) — at minimum:

```
PING           (over radio)   -> reply arrives promptly (no multi-second stall)
D 200 200 300  (over radio)   -> wheels spin, EVT done arrives over radio
```

### (e) Comms-silence watchdog neutralizes, over radio

```
D 200 200 5000  (or any long-running drive, over radio)
<disconnect / stop sending anything for > the configured DEV WD window>
                               -> wheels neutralize (STOP/brake) WITHOUT any host command;
                                  EVT dev_watchdog observed once
<send any statement again, e.g. PING>
                               -> watchdog re-arms; a subsequent silence fires again
```

This is the linked watchdog issue's own Bench/HITL acceptance criterion —
closing it out sets `completes_issue` for
`preserve-serial-silence-safety-watchdog-in-greenfield-loop.md` to `true`
(update the ticket/issue frontmatter once this passes).

### (f) Sim/hardware parity

Run the SAME short command sequence against both the sim and the bench,
compare encoder/pose behavior within the existing tolerance bands (the
`tests/sim/unit/test_motion_overshoot_regression.py`/
`test_tour_geometry.py` bounds are a reasonable reference, e.g. ~1.5%/10mm
distance, ~7-8deg heading):

```
D 200 200 500
RT 9000
SI 0 0 0        (or an appropriate SET-pose-then-drive checkpoint)
ZERO enc
```

Compare final `SNAP`/encoder/OTOS pose between the two runs; both should
land within the same tolerance bands the sim tests already assert.
