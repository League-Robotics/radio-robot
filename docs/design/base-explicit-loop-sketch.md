# Sketch: the explicit-dataflow base loop (discussion draft)

**Status:** DRAFT for stakeholder discussion, 2026-07-24 — not an issue yet.
**Companions:** `clasi/issues/extract-motion-library-to-src-motion.md`,
`clasi/issues/firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md`.

## What the stakeholder asked for, as three principles

1. **No reference webs.** Objects in the main loop are constructed
   independently (a motor knows its bus and config, nothing else) and
   coordinated AT the loop. No `Drive(motorL, motorR)`, no
   `Odometry(motorL, motorR)`, no `MoveQueue(drive, odom, clock, estimator)`.
2. **Loop-visible dataflow.** The commanded wheel velocity is a local
   variable in `cycle()` at the line it is sent. Reading the loop top to
   bottom tells you everything that happened this cycle, in order.
3. **The wheel layer makes no velocity decisions.** The motion library
   decides the wheel velocity; the base tracks it and reports truthfully.
   The only wheel-side interventions are basic plausibility and physical
   brick protection.

This is the blackboard's decoupling without the blackboard: a central state
object lets anyone write anytime (hidden writers, unordered reads). Passing
plain value structs THROUGH the loop gives the same decoupling with ordered,
visible, printable dataflow — the loop is the coordinator, and every
cross-object value is a local you can log.

## Inventory: everything currently between `setVelocity()` and the wire

`NezhaMotor` is 885 lines per channel. What's actually in there, and the
verdict per item:

| # | Mechanism (where) | What it is | Verdict |
|---|---|---|---|
| 1 | Split-phase 0x46 request/collect, hardReset median-of-3, encoder offset, failure-hold | Brick protocol truths | **KEEP** — how to talk to this chip; not velocity decisions |
| 2 | fwdSign, clamp ±100%, integer-% quantization, write-on-change, NAK retry, write-rate throttle | Bus/write hygiene | **KEEP** — protocol, not policy |
| 3 | Reversal dwell (100 ms) + output-deadband boost (`writeShapedDuty`) | Wedge protection: an instant H-bridge sign flip under way latches the 0x46 readback (documented hardware failure, 2026-07-04) | **KEEP but make visible** — physical protection; telemetry reports `dwelling` and the actually-written duty so it never surprises anyone again |
| 4 | Velocity PID + kff open-loop mapping (`MotorVelocityPid`) | speed → duty tracking | **MOVE to motion** (stakeholder decision 2026-07-24): moving it up gives ONE velocity estimate (the observer's) feeding ONE controller, tunable in `motion_tests`. (The original "encoder freshness ~80 ms bounds the PID rate anyway" argument was measured false 2026-07-26 — `encoder-refresh-characterization.md`; see Resolved question 1's correction note.) The base's command primitive becomes per-wheel DUTY; anti-windup reads `appliedDuty` back from the sample |
| 5 | Per-write duty slew cap (`slewRate`, default 25%/write) | Hidden actuator shaping | **DECIDE: characterize or delete.** It reshapes the response underneath the PID and would silently corrupt the observer's model. Either the bench shows the brick tolerates steps (delete), or it stays and the observer's characterized model includes it. Not both, not hidden |
| 6 | Duty boxcar smoothing (`dutyAvgWindow`, bench knob, default off) | Cosmetic jitter filter | **DELETE** — a bench experiment that shipped; adds lag, answers nothing the observer won't |
| 7 | Freshness gate (guards against repeated raw counts — measured 2026-07-26: NOT an ~80 ms register refresh, which is false; repeats come from interposed-traffic sample invalidation, `encoder-refresh-characterization.md`) | Measurement conditioning | **FOLD INTO OBSERVER** — between fresh samples (rare on a clean schedule), the estimate rides the command model instead of starving |
| 8 | Glitch rejection (`kMaxPlausibleStepSpeed` 1200 mm/s, streak-of-3 re-accept) | Outlier heuristic | **FOLD INTO OBSERVER** — becomes an innovation bound: a sample wildly off prediction is rejected+counted; three ad-hoc mechanisms (7, 8, 9) become one principled one |
| 9 | Velocity estimator A/B (EMA `velFiltAlpha` vs least-squares line-fit ring, live-switchable) | Two competing filters + a knob | **FOLD INTO OBSERVER** — the observer IS the velocity estimate; delete the A/B machinery |
| 10 | `wheelTravelCalib` (ticks → mm per wheel) | Calibration | **KEEP** — applied once at sample ingest, reported |
| 11 | `kMaxPlausibleSpeed` (600 mm/s) sanity clamp | Plausibility | **KEEP** — the stakeholder's "basic plausibility limits," applied visibly in the loop's clamp step |
| 12 | MotorArmor decorator (wedge observe/recover, rest thresholds, reconfigure guards) | Protection policy wrapper | **SIMPLIFY** — wedge detection folds into the observer's innovation logic (commanded ≠ moving for N samples = wedge event, reported); recovery stays an explicit, loop-visible action |

Net: `NezhaMotor` shrinks to protocol + PID + dwell/deadband + clamp
(~300 lines); the conditioning stack (7-9) is replaced by the per-wheel
observer; the knobs (5 decided, 6 and 9 deleted) stop existing.

## The loop (LOGICAL dataflow — read this box before transcribing anything)

> **This code block is a dataflow diagram, not the cycle implementation.**
> Do not write `cycle()` this way. The real cycle CANNOT sample the two
> motors back-to-back: the brick's encoder-select latch and settle windows
> force request→wait→collect per port, the waits are real time, and the
> loop uses that time for other work (comms pump, telemetry emit, OTOS,
> line/color) exactly as today's schedule does. What is NORMATIVE in the
> sketch below is (a) the order VALUES flow — sample before observe before
> decide before act before report, within a cycle; (b) every cross-object
> value is a cycle-local struct you can print; (c) there is exactly ONE
> write point per wheel, fed by ONE visible variable, with plausibility as
> the only intervention. What is NOT normative: temporal adjacency of the
> statements, the number of blocks, or where the borrowed work goes. The
> implementer keeps the interleaved request/settle/collect schedule and
> threads these values through it.

```cpp
void RobotLoop::cycle() {
  uint64_t now = clock_.nowMicros();

  // 1 SENSE — the only place motors are read
  WheelSample left  = motorL_.sample(now);   // raw ticks->mm, fresh flag; NO velocity math
  WheelSample right = motorR_.sample(now);
  OtosSample  otos  = otos_.sample(now);

  // 2 OBSERVE — pure; predict from last command, correct on fresh samples
  WheelEstimate leftEst  = observerL_.update(lastCmd_.left,  left,  now);
  WheelEstimate rightEst = observerR_.update(lastCmd_.right, right, now);

  // 3 DECIDE — the motion library, pure; ALL control decisions live here,
  //   including the per-wheel velocity PID (duty out, per stakeholder
  //   2026-07-24: rate-bound by the loop either way, so it lives with the
  //   rest of control and tunes in motion_tests)
  MotionOutputs out = motion_.tick({leftEst, rightEst, otos, now},
                                   comms_.takeCommand());

  // 4 ACT — the one visible write; plausibility + zero-on-silence only
  lastCmd_ = clampPlausible(out.duty);       // |duty| <= 1, NaN -> 0
  motorL_.drive(lastCmd_.left,  now);        // dwell/deadband -> wire; nothing else
  motorR_.drive(lastCmd_.right, now);

  // 5 REPORT — exactly what happened, all three views
  tlm_.emit({left, right, leftEst, rightEst, otos,
             lastCmd_, out.pose, out.events}, now);
}
```

Construction: `NezhaMotor(bus, config)` ×2, `WheelObserver(model)` ×2,
`Motion::Controller(config)`, `Telemetry(comms)`. Nothing holds anything
else. `Drive` and the estimator-reference plumbing cease to exist as wiring;
their logic lives inside `motion_` (twist→wheels) and the observers.

**What the real cycle looks like (illustrative, not prescriptive):** the
five logical stages interleave with the bus timing exactly as today's
schedule interleaves its work —

```
requestSample(L)                      // SENSE-L begins
  [settle window — borrowed: comms pump]
left = collectSample(L)               // SENSE-L ends
  [clear window — borrowed: telemetry emit of LAST cycle's snapshot]
requestSample(R)                      // SENSE-R begins
  [settle window — borrowed: leftEst = observer update, command decode]
right = collectSample(R)              // SENSE-R ends
otos  = otos.sample(now)              // pace block, as today
rightEst = ...; out = motion_.tick(...)   // OBSERVE + DECIDE
lastCmd_ = clampPlausible(out.duty)       // ACT decided...
motorL_.drive(...); motorR_.drive(...)    // ...and written (may land next
                                          // cycle's write slot per the bus
                                          // schedule — the implementer owns
                                          // that placement)
stage snapshot for next emit              // REPORT
  [pace window — borrowed: line/color alternation]
```

The invariant that must survive any scheduling: within a cycle, the VALUES
flow sample → estimate → decision → clamped command → write → report, each
as a visible cycle-local; where the statements sit among the settle/clear/
pace windows is the implementer's schedule to own, same discipline as
today's robot_loop.cpp.

**Why not request-request-collect-collect (asked and answered, 2026-07-24):**
the brick has ONE latched encoder-select register (0x46) shared by all
channels at one I2C address — it holds a single pending read. Issuing both
selects before either collect makes both collects read the LAST-selected
wheel: the documented 2026-07-18 glued-encoder incident ("actual L ==
actual R"), modeled deliberately in SimPlant. The per-port
request→settle→collect interleave is a protocol constraint, not a style
choice. The legitimate goal behind the question — time-aligned L/R state —
is met differently: (a) the L→R collect gap can be squeezed toward its
~one-settle-window floor (~4 ms) by choosing which borrowed work sits
between them; (b) decisively, the OBSERVER owns time alignment — every
`WheelSample` carries its own timestamp, and the observer projects BOTH
wheels' estimates to a common epoch (the decide instant) before anything
kinematic consumes them. That makes the 4–8 ms collect offset — and any
residual freshness skew (measured 2026-07-26: the register is live at
≤ 16 ms; see `encoder-refresh-characterization.md` — repeats come from
schedule faults, not a refresh timer) — a solved
mathematical problem instead of a scheduling constraint. Same-generation
telemetry pairing (121-005) remains a base invariant at the FRAME level;
common-epoch projection is the estimator-level guarantee on top.

## Boundary structs (the whole interface)

```cpp
struct WheelSample   { float position; bool fresh; float appliedDuty; uint64_t t; bool busOk; };  // [mm] [-1,1] [us] — appliedDuty is what the shaping actually wrote (anti-windup + observer input)
struct WheelEstimate { float position; float velocity; float age; uint32_t rejects; bool wedged; };  // [mm] [mm/s] [s]
struct DutyCommand   { float left; float right; };                              // [-1,1] per wheel
struct MotionOutputs { DutyCommand duty; Pose pose; MotionEvents events; };     // pose [mm,mm,rad]
```

## Resolved and open questions

1. **PID placement — RESOLVED (stakeholder, 2026-07-24): in the motion
   library.** Rationale AS DECIDED: encoder freshness (then believed
   ~80 ms) bounds the PID's effective rate at or below the loop rate
   wherever it lives, so motor residency buys nothing. [CORRECTION
   2026-07-26: the freshness premise is false — the register is live at
   ≤ 16 ms (`encoder-refresh-characterization.md`), so a motor-resident
   PID COULD run faster than the loop. The decision's other grounds
   below still apply; revisit only if a faster inner velocity loop is
   ever wanted.] Motion residency gives one velocity estimate
   (observer → PID), unit-speed tuning in `motion_tests`, and a motor object
   that is a pure bus adapter (~200 lines). Consequences, accepted: the
   base's command primitive is per-wheel DUTY; the bounded WHEEL-SPEED move
   (MoveWheels + stop conditions + timeout) moves up into the motion
   library's lowest tier (speed-stop tracking needs the PID); the base's
   safety property becomes zero-on-silence (no drive() call this cycle ⇒
   duty 0) + the plausibility clamp; the observer stays in the base
   (per-wheel, hardware-characterized truth reporting) and consumes
   commanded duty + encoder samples; `appliedDuty` is fed back in
   `WheelSample` so the PID's anti-windup sees actuator truth through the
   dwell/deadband shaping. The sim boundary gets cleaner: `WheelPlant`
   already consumes duty natively.
2. **Slew cap (#5):** characterize into the observer model, or bench-test
   and delete. Recommend delete-if-bench-allows: fewer hidden shapers.
3. **Telemetry width:** commanded duty + observed + raw per wheel per frame
   is ~12 extra bytes; worth it for "the base never lies."
4. **Migration order:** extraction issue first (motion library exists), then
   this loop rewrite as the first base-hardening ticket, then the observer
   replaces 7-9.
