# Turn-Execution Review — why "turn 90°" doesn't land on 90°

**Reviewer:** Claude (team-lead session, stakeholder-authorized out-of-process — CLASI MCP unavailable)
**Date:** 2026-07-22/23 (overnight run)
**Scope:** full execution trace of a turn command through the sim path; root-cause of the
overshoot; correctness assessment; bloat inventory. Read-only — no source changes, no tickets.
**Verification:** firmware sim library built from this checkout (Linux host build) and driven
headlessly through the real `SimLoop`/`run_tour()` path. All numbers below are measured on this
checkout against SimPlant ground truth, not quoted from prior notes.

---

## 1. Summary

Three findings, in decreasing order of the damage they did:

**F1 — The fix existed but was silently OFF in every live GUI sim session until today.**
`App::MoveQueue` is constructed with anticipation and velocity shaping disabled
(`stopLead = 0`, `ShaperLimits{}` all-zero) unless an `EstimatorConfigPatch` is pushed over the
wire (`src/sim/sim_harness.h:169-185`, by design — the "sim/production boundary"). The closure
gate test pushes it; the live TestGUI did not until
`clasi/issues/wire-testgui-live-push-of-estimator-stop-lead.md` was closed (build
v0.20260722.x, today). Your bowtie tour and the +22.4° isolated 180° are byte-for-byte the
no-push baseline: I measure **+17.5° mean per 90° turn with the push absent** on this checkout,
and **+0.98° (isolated 90°) / 1.65° worst (TOUR_1)** with it present. Your post-update
screenshots (isolated 360° landing at +0.9°) confirm the push is now live in your GUI.

**F2 — The residual error is structural: the stop decision is an open-loop time-lead guess.**
The firmware stops a turn when a *predicted* heading crosses the threshold, where the
prediction is current-heading + ω × `stop_lead_ms`. `stop_lead_ms` is a hand-tuned constant
that has been re-tuned four times in three weeks (200 → 90 → 60 → 45 ms, the archaeology is in
`data/robots/tovez_nocal.json`'s `_estimator_note`) because every new pipeline stage changed
the lag it was tuned to cancel. It is correct only at ω = 2 rad/s, only at the sim's 50 ms
cycle, and only for the current shaper settings. That is why "exact" never arrives: the design
compensates latency instead of removing it, and lands while still rotating instead of arriving
at zero speed. §6 quantifies the latency budget; §7 gives the two small changes that make turns
exact by construction instead of by tuning.

**F3 — Two of the ~three cycles of stop latency are self-inflicted scheduling, not physics.**
`MoveQueue::tick()` runs *before* `odom_.integrate()` in the cycle
(`src/firm/app/robot_loop.cpp:586` vs `:619`), so every stop decision uses heading data one
full cycle stale — and the decision's `Drive::stop()` doesn't reach the motors until the *next*
cycle's `drive_.tick()` (`:542`). The anticipation lead exists in large part to cancel an
ordering choice one screen away.

The command pipeline itself (host Move → wire → MoveQueue → StopCondition → Drive → plant) is
correct, compact, and traceable. The problem was never "the robot can't turn 90°" — it is that
the completion decision runs on stale data, fires while the robot still carries ω, and the
compensation for that was (a) disabled in your GUI and (b) a tuned constant rather than a
derived quantity.

---

## 2. Execution trace: what happens when you ask for "turn 90°"

Sim path, tour leg or GUI managed-move — the only motion path that exists post-116 (every
motion is a bounded MOVE; there is no separate turn controller).

### 2.1 Host side (Python, one-shot — no host control loop)

1. **Leg definition.** `RT 9000` (centidegrees) in `TOUR_1`
   (`src/host/robot_radio/planner/tour.py:189-203`) → `parse_tour()` → `TourLeg(kind="turn",
   value=90.0)` (`tour.py:310-347`).
2. **Move construction.** `_move_kwargs_for_leg()` (`tour.py:454-491`):
   `omega = ±PlannerParams.omega_max = ±2.0 rad/s` (`planner/model.py:79`),
   `stop_angle = radians(90) = 1.5708 rad`, `timeout = max(2000, expected·3) ms`,
   `replace=False` (tours enqueue one leg ahead — `run_tour()`, `tour.py:704-706, 763-764`).
3. **Wire encode.** `NezhaProtocol.move_twist()` (`robot/protocol.py:951`) builds
   `CommandEnvelope{move: Move{twist:{v_x:0, omega:2.0}, stop_angle:1.5708, timeout, id}}`
   (protobuf arm 21), base64-armored, injected into the sim
   (`SimLoop.move()` → `sim_ctypes` → `SimHarness::injectCommand()`).
4. **From here the host only polls.** `run_tour()` sleeps in 50 ms polls waiting for a
   telemetry frame whose single ack slot echoes `Move.id` (`tour.py:494-563`). The host plays
   **no part** in deciding when the turn ends. (The "23 ticks" in your `[TOUR]` log are host
   poll iterations, nothing more.)

### 2.2 Firmware cycle (every cycle; sim advances 50 ms virtual time per cycle — see §5.3)

Cycle schedule, `RobotLoop::cycle()` (`src/firm/app/robot_loop.cpp:526-633`), in execution
order:

| # | Work | Lines | Notes |
|---|------|-------|-------|
| 1 | `drive_.tick()` — convert *last* staged twist → wheel targets | :542 | targets decided in cycle N−1 are written in cycle N |
| 2 | motor request/collect — write duty, read encoders | :548-551 | encoder positions reflect plant state at cycle start |
| 3 | `comms_.pump()` — decode ≤1 command | :554 | the MOVE arrives here |
| 4 | `updateTlm()` + `tlm_.emit()` — stage enc/twist, **emit frame** | :563-564 | emitted *before* this cycle's MoveQueue tick → a completion ack rides the **next** frame |
| 5 | `processMessage()` → `handleMove()` → `MoveQueue::enqueue()` | :573, :232-252 | validation: velocity variant + stop variant + timeout>0 |
| 6 | `moveQueue_.tick(now, odom_)` — **the stop decision** | :586 | uses odometry integrated at the END of cycle N−1 |
| 7 | final block: `applyOtosSample()`, `odom_.integrate()`, `frame_.pose staged`, `stateEstimator_.update()` | :612-632 | fresh heading exists only *after* the decision ran |

### 2.3 Activation (cycle the MOVE arrives, or chain-advance)

`MoveQueue::activate()` (`src/firm/app/move_queue.cpp:18-109`): stages the twist on `Drive`
(from carried-over shaper state when shaping is on — SUC-051 hand-off), captures the
StopCondition baseline `(now, pathLength, theta)` from **raw** odometry. For a tour leg the
activation is usually `tick()`'s chain-advance (`move_queue.cpp:261-265`) at the instant the
previous leg completes, reusing that tick's `(now, odom)` readings.

### 2.4 Every subsequent cycle until the stop fires

`MoveQueue::tick()` (`move_queue.cpp:215-282`):

1. Reconstruct `Motion::StopCondition` from the stored baseline (pure precompute).
2. **Anticipation** (`:230-240`): if `stopLead_ > 0`, replace the current readings with
   `stateEstimator_.bodyAt(nowMs + stopLead_)` — ZOH extrapolation
   `heading = basis.heading + basis.omega·age` (`state_estimator.cpp:87-107`), where the basis
   is last cycle's `frame.pose.h`/`frame.twist.omega` (encoder-derived; OTOS blend weights are
   committed 0.0). `age ≈ one cycle + stopLead` ≈ 95-100 ms in sim.
3. `StopCondition::tick()` (`motion/stop_condition.cpp`): Angle kind →
   `|theta − theta₀| ≥ 1.5708`? Timeout backstop second; kind wins ties.
4. `Continue` → `shapeAndStage()` (`move_queue.cpp:138-184`): `remainingAngular = threshold −
   |theta_pred − theta₀|`; `VelocityShaper::next()` (`motion/velocity_shaper.cpp:57-133`) —
   slew toward cruise at ≤ α_max, cap |ω| to `√(2·α_decel·remaining)` (the decel-into-goal
   taper), jerk-clamp the implied accel at ≤ yaw_jerk_max; restage via `Drive::setTwist()`.
5. `StopConditionMet` → report completion (ack `Move.id` on the **next** frame), chain-advance
   the next pending Move with the same readings, or `Drive::stop()` + shaper resets.

### 2.5 Plant (sim) side

`Drive::tick()` → `BodyKinematics::inverse(v=0, ω=2, b=128)` → wheel targets ±128 mm/s →
`NezhaMotor` velocity PID/FF (`vel_kff = 0.002` ⇒ duty ≈ target/500) → `SimPlant` `WheelPlant`
first-order duty→velocity response, **τ = 0.13 s** (bench-characterized 120-140 ms,
`src/tests/sim/plant/plant_harness.cpp:133-134`) → `OtosPlant` integrates ground truth with the
same 128 mm trackwidth. Sim geometry, firmware kinematics, and odometry all agree — there is no
kinematic scale error anywhere (confirmed: firmware pose tracks sim truth to <0.1° through a
full tour).

### 2.6 Measured timeline (this checkout, deterministic sim, ideal chip, 90° at 2 rad/s)

Anticipation + shaper OFF (what your GUI ran until today):

| t [s] | truth [deg] | TLM pose.h [deg] | event |
|-------|------------|------------------|-------|
| 0.85 | 86.0 | 80.2 | |
| 0.90 | **91.8** | 86.0 | truth crosses 90 |
| 0.95 | 97.5 | **91.8** | MoveQueue sees ≥90 (data is 1 cycle old) → completes; stop staged |
| 1.00 | 103.2 | 97.5 | duty 0 written this cycle; **completion ack emitted** |
| 1.05-2.5 | 107.2 → **110.3** | | plant coasts out on τ=0.13 s |

**Result: +20.3° overshoot.** Budget: ~1.8° threshold quantization + 5.7° stale-data cycle +
5.7° decision-to-duty cycle + ~7.1° plant coast ≈ +20.3°. The same budget at hardware's 20 ms
cycle: ~2.3+2.3+2.3 + ~15 (same τ, higher ω tail fraction) ≈ +17-22° — which is exactly the
"61-73% on 0.5 rad" (+17-21° absolute) and the "+10-16° on 90°" hardware bench tables in
`clasi/issues/angle-stop-overshoot-61-73-percent-on-hardware.md`. One mechanism explains every
measurement in that issue.

Anticipation (45 ms) + shaper ON:

| t [s] | truth [deg] | ω_cmd [rad/s] | event |
|-------|------------|----------------|-------|
| 0.10-0.55 | 0 → 32.5 | 0 → 2.0 | jerk-limited S-curve ramp-up |
| 0.60-0.90 | 38.2 → 72.7 | 2.0 | cruise |
| 0.95-1.10 | 78.1 → 89.1 | 1.9 → 0.85 → 0.31 | taper: ω bleeds as remaining → 0 |
| 1.10 | 89.1 | | predicted heading crosses 90 → complete, ack |
| 1.15+ | 90.0 → **91.0** | 0 | small tail |

**Result: +0.98°.** The taper does the real work (the tail shrinks with ω²); the 45 ms lead
covers what's left of the detection+write latency.

### 2.7 A/B results (all measured tonight on this checkout)

| Scenario | push OFF | push ON |
|----------|---------|---------|
| Single 90° turn (deterministic) | **+20.34°** | **+0.98°** |
| TOUR_1, 6 turns (deterministic) | +15.6 to +21.3°, mean **+17.5°** | −0.83 to +1.65°, mean **+0.04°** |
| Mini-tour, real-time tick thread | — | −0.76°, +1.30° |

Your first screenshots (bowtie, fused θ +724° after 540° commanded ≈ +30°/turn; isolated 180°
→ 202.4°) are the push-OFF row plus live-GUI conditions. Your post-update screenshots (360° →
+0.9°) are the push-ON row. The mid-session improvement you reported is real and is F1 closing.

---

## 3. Direct answers to the review questions

**"Why does 90° become ~120-135°?"** Because until today the live sim GUI ran the raw
threshold-stop with zero compensation: the decision fired ~2 cycles late on a robot still
turning at 114°/s, and the plant coasted another τ·ω after the duty-0 write. 90° was never
mis-measured — firmware odometry tracked truth to <0.1° the whole time. It was measured
correctly and *acted on* 150-250 ms too late.

**"I told it to reduce speed approaching the goal — why didn't that work?"** It was
implemented (`Motion::VelocityShaper`, correctly, 94 lines) and then never engaged in your GUI
sessions, because `SimHarness` deliberately boots with shaping disabled and nothing pushed the
config until today's connect-time push. The same applies to the forward-prediction stop. Both
features tested green in the harness (which pushes its own config) while every interactive
session ran without them. **This is the single most expensive defect in the project right now
and it's a process defect, not a control defect: a correctness-critical feature whose default
is silently-off in exactly one environment — the one you demo in.**

**"Is the predictive system itself right?"** Half. Predicting forward to cover *unavoidable*
latency is legitimate. But (a) one of its cycles of latency is avoidable (§1-F3), and (b) the
lead is a tuned scalar, not `ω × measured_pipeline_delay` — so it silently encodes ω = 2 rad/s,
the 50 ms sim cycle, and the current shaper curve. Change any of those and it's wrong again;
that is precisely the 200→90→60→45 ms history, and why hardware (20 ms cycle, different tail)
still shows a 4-8° residual with a sim-tuned 45 ms.

---

## 4. What is actually good

Worth saying plainly, because the fix should preserve it: the post-116 motion architecture is
the right shape. One bounded-MOVE primitive; completion decided where the data is freshest
(firmware), host reduced to enqueue-and-wait; `StopCondition` and `VelocityShaper` are small,
pure, host-testable modules; `BodyKinematics` is the only place the diff-drive math lives; the
sim drives the *real* firmware graph through the real wire codec. The tour path has no host
control loop left to fight the firmware one. `velocity_shaper.cpp` after your "two clamps and
an integrator" correction is exactly as simple as it should be.

---

## 5. Defect list (correctness)

**D1 — Silent-off config boundary (root cause of weeks of confusion).** `SimHarness`
constructs `MoveQueue(drive, odom, clock, stateEstimator)` with no stopLead/ShaperLimits
(`sim_harness.h:169-185`) and deliberately does not link `boot_config.cpp`. Anything not
pushed over the wire is off. Now papered by the GUI connect push — but any *other* entry point
(a new script, a test, MCP) silently reverts to the uncompensated plant. There is no telemetry
bit, log line, or fault flag that says "shaping/anticipation inactive."

**D2 — Stop decision consumes cycle-stale state by schedule order.** `moveQueue_.tick()` at
`robot_loop.cpp:586` runs before `odom_.integrate()` at `:619`. Cost: one full cycle of heading
(5.7° at cruise in sim). The StateEstimator basis is likewise last-cycle. The anticipation lead
then pays this back. Decision-to-duty adds a second cycle (`drive_.tick()` at the top of the
next cycle writes what the stop staged).

**D3 — `stop_lead_ms` is a tuned fudge with four generations of archaeology.** It conflates
three physical quantities (data staleness, actuation delay, coast tail) into one scalar valid
at one ω, one cycle time, one shaper setting. Its own config note admits each retune was forced
by an unrelated stage changing. On hardware the same constant is wrong again (4-8° residual,
no better value found in the 45-90 ms bracket — because no single value exists).

**D4 — Sim runs the control loop at 50 ms; firmware runs it at 20 ms.**
`SimHarness::kCycleDtUs = 50000` (`sim_harness.h:472`) — chosen to dodge `NezhaMotor`'s duty
write-rate throttle — vs `kCycle = 20` (`robot_loop.cpp:27`). Every sim-tuned millisecond
constant and every "N cycles of latency" result is measured on a plant with 2.5× the shipped
control period. The sim is deterministic, but it is deterministic about *a different robot*.

**D5 — Tour-embedded turns start from hand-off state, isolated turns from rest.** Chain-advance
carries shaper state (deliberate, SUC-051) and the next leg's activation re-stages the *old*
axis speeds; a D→RT boundary also crosses `NezhaMotor`'s 100 ms reversal dwell on one wheel
only (asymmetric). This is the measured isolated-vs-tour gap (0.3° vs ~1.4-1.7°). Not a bug per
se, but nobody has written down what the hand-off *should* do to heading, so it's tuned around
instead of specified.

**D6 — Stale/staleness hazards around the realtime path.** The one real-time-threaded test is
xfailed with a reason citing `clasi/issues/cycle-order-reorder-experiment-ab-before-hardware.md`
— a file that no longer exists — describing alternating/stale encoder reads corrupting
`frame.twist` under the real tick thread on this very library. My sandbox real-time runs were
clean (±1.3°), but if your live GUI tours still measure worse than ±2°/turn after F1, this is
the first suspect, and right now its tracking issue is a dangling reference. (Second suspect:
the fused/camera overlay's start-heading bookkeeping — verify with `Set Robot @ 0,0`, run
TOUR_1, read the `[TOUR]` closure line and final pose θ against 540°.)

---

## 6. Bloat inventory (agent-accretion to delete or demote)

The pattern across sprints 109-117 is: each campaign **adds** a compensator and **retunes** the
previous one; nothing is ever removed. Concrete list:

1. **`stop_lead_ms` + its tuning saga** — delete after R1/R2 below; the constant plus ~60 lines
   of derivation notes across `tovez.json`/`tovez_nocal.json`/`test_tour_closure_gate.py`
   comments exist to justify a number that shouldn't exist.
2. **`StateEstimator` speculative surface** — firmware consumes exactly `bodyAt()` (one call
   site, `move_queue.cpp:233`). `wheelAt()`, `wheelNow()`, `whereAmI()`, `Innovations`, and the
   OTOS complementary-blend path (weights committed 0.0, "wired but not trusted") are dead
   weight on the hot path with live tuning arms, config plumbing, and doc surface. Keep the
   8-line ZOH `bodyAt`; delete or quarantine the rest until something consumes it.
3. **Orphaned `control.*` config keys** — `heading_kp/kd`, `heading_source`,
   `heading_dwell_tol_deg`, `heading_dwell_rate_dps`, `heading_lead_bias`, `plan_lead`,
   `terminal_lead`, `distance_kp`, `distance_tol`, `actuation_lag`, `model_tau_lin/ang`,
   `turn_gate`, `arrive_dwell`, `arrive_tol_mm`, `sync`, `min_speed`, `yaw_rate_max`,
   `max_rot_accel_dps2` — most fed the motion stack deleted in 115-002. "Config as source of
   truth" has become config-as-attic; every dead key is a future agent's invitation to "wire it
   back up." Audit each for a living consumer; delete the rest from schema + JSONs.
4. **`run_tour()` back-compat kwargs** — `a_max`, `alpha_max`, `cadence`, `inter_leg_settle`
   all documented UNUSED (`tour.py:611-617`), kept for one bench-script caller. Fix the caller.
5. **Header archaeology** — `move_queue.h` is 375 lines for ~90 lines of declaration;
   `sim_harness.h`, `tour.py`, and the closure-gate xfail strings carry multi-page sprint
   narratives (the ideal-chip xfail reason alone is ~115 lines). This is exactly the "I can't
   read my own code" complaint. The contract belongs in the header; the history belongs in
   `DESIGN.md`/git. Mechanical relocation, zero behavior risk.
6. **Stale references** — CLAUDE.md points at `source/commands/CommandProcessor.cpp` and
   `docs/protocol-v2.md` §13 (path-rotted; the tree is `src/firm`, protocol is v4);
   `.claude/rules/hardware-bench-testing.md`'s smoke table is marked STALE and predates v2, two
   protocols ago; the GUI panel still says "Managed — Ruckig" (Ruckig was never shipped; the
   shaper is deliberately not Ruckig); D6's dangling issue link. Each one misdirects the next
   agent session — this is how wrong-spot fixes start.
7. **`DEFAULT_INTER_LEG_SETTLE` and friends** — constants retained solely so an old call
   signature doesn't change, with 10-line comments explaining they do nothing.

None of these are behavior bugs today. All of them are surface area that agents keep
re-reading, re-explaining, and re-tuning. The comment-to-code ratio in the motion path is the
single best predictor of where the next hack will land.

---

## 7. Recommendations, ranked

**R1 — Reorder the cycle so the stop decision sees this cycle's odometry.** Move
`odom_.integrate()` (+ `applyOtosSample`/estimator update) ahead of `moveQueue_.tick()`, or move
the MoveQueue tick into the final block after integration. Deletes one of the two latency
cycles outright; the decision then runs on data ≤1 ms old in sim and ≤ a few ms on hardware.
Small diff in `robot_loop.cpp`; the bus-discipline constraint (no bus traffic in settle
windows) doesn't apply to MoveQueue::tick (pure compute). Re-verify with the existing gate.

**R2 — Replace the tuned lead with a derived one, or remove the need for it.** Two acceptable
end states; pick one:
  a. *Derived lead:* `lead = kCycle·(pipeline cycles) + κ·τ_plant·|ω|/ω` — i.e., compute the
     anticipation from named constants that already exist (`kCycle`, plant τ from calibration)
     instead of a swept scalar. Portable across ω, cycle time, and hardware/sim by
     construction.
  b. *(Preferred, simpler)* *Land at zero:* the taper already commands
     ω = √(2·α_decel·remaining); let the shaper finish — declare completion when
     `remaining ≤ ε AND |ω_cmd| ≤ ε_ω` (StopCondition keeps the threshold as its safety
     backstop). The robot arrives *at* the target *at* ~zero speed; there is no tail to
     predict, so `stop_lead_ms` is deleted rather than derived. This is what "reduce speed as
     you approach the goal" was always for.
With R1+R2b the whole anticipation block in `MoveQueue::tick()` (and the StateEstimator
dependency edge it dragged in) can go.

**R3 — Kill the silent-off boundary.** The sim composition root already configures motors from
the active robot JSON; estimator/shaper being excluded "to preserve a boundary" is an
inconsistency that cost weeks. Either bake `defaultShaperConfig()`/`stopLead` into the sim
graph from the same JSON, or add a loud indicator (telemetry flag + GUI banner + log line on
every MOVE accepted while shaping is disabled). A feature that changes accuracy by 20× may not
have an invisible off state.

**R4 — Make the sim run the shipped control period.** Fix the duty write-rate throttle
interaction (the reason for 50 ms stepping) and set the sim cycle to `kCycle = 20 ms`, so
sim-validated timing constants transfer to hardware. Until then, treat every sim-tuned ms value
as sim-only.

**R5 — Specify the leg hand-off.** One paragraph in `motion/DESIGN.md`: at a chain-advance
boundary, what should the carried shaper state be for an axis the next Move doesn't command
(current answer: decay from carry-over), and is the D→RT reversal-dwell asymmetry acceptable?
Then assert it in the boundary test instead of tuning around it.

**R6 — Delete per §6**, and adopt one standing rule for agent work in this repo: *a change that
adds or retunes a numeric constant must name the physical quantity it models and derive it from
named constants; if adding a stage forces retuning an existing constant, the default action is
to delete the constant, not retune it.* That rule would have converted this month's four
stop-lead sweeps into R2 directly.

---

## 8. Verification appendix

Built `libfirmware_host.so` from this checkout (cmake, Linux sandbox), drove it through the
real `SimLoop` → `NezhaProtocol` → `run_tour()` stack (`start_tick_thread=False` deterministic,
and `=True` real-time), robot config `tovez_nocal.json`, ideal chip (all sim error knobs
explicitly zero). Scripts preserved in the session outputs (`turn_experiment.py`,
`realtime_tour.py`); re-runnable on macOS against `src/sim/build/libfirmware_host.dylib`
unchanged. Full per-cycle traces in §2.6 were captured at 50 ms cycle resolution from
`get_true_pose()` vs the emitted TLM frames.

Not verified here: hardware timing (no robot attached to this sandbox), the live macOS GUI's
real-time behavior under Qt/camera load, and TOUR_2 (time-boxed; the gate covers it). The
one open empirical question is D6: whether your live GUI tours still exceed ±2°/turn now that
F1 is closed — the two candidate explanations and the discriminating test are listed there.

---

## 9. Addendum (2026-07-23 morning) — straight-leg crab introduced by 118-001

> Filed as its own issue (stakeholder-directed):
> `clasi/issues/straight-leg-crab-118-001-actuation-and-telemetry-pairing-skew.md`

Observed live (v0.20260723.1): a 700 mm straight leg where truth/fused/OTOS end at y ≈ +31 mm
while the host encoder trace stays perfectly straight (`enc L +708 R +708`, encpose y 0 θ 0).
Reproduced headlessly on this checkout and root-caused to **two defects in the 118-001 schedule
restore** (`robot_loop.cpp`, commit 3189086f):

**A — One-cycle L/R actuation skew.** `drive_.tick()` now sits in the R-settle block, *between*
`motorL_.tick()` and `motorR_.tick()`. L therefore writes duty from the target staged **last**
cycle; R writes **this** cycle's fresh target (the block's own comment says so: "−1 cycle" for
L). During any commanded ramp, R physically leads L by one cycle. Predicted yaw transient
`Δθ = v_cruise · kCycle / b = 150 · 0.040 / 128 = 2.69°`; measured truth heading during cruise
**+2.685°**. The decel ramp restores it (final θ 0.00°), so the body crabs sideways:
`y ≈ 660 · sin(2.69°) ≈ +31 mm` — measured **+32.5 mm** over x +708. Every accel/decel on every
Move (straight or turn) injects this kick; hardware inherits it identically at kCycle = 40 ms.

**B — Telemetry pairs fresh L with stale R.** `updateTlm()` + `emit` run in the kClear block,
after collect L but *before* collect R, so every frame carries this cycle's L against last
cycle's R. Measured host-visible per-frame deltas: `dL − dR = +0.00` on every frame — the
pairing skew exactly cancels the physical skew, so host encoder dead-reckoning (encpose, the
orange trace, frame.twist) reports a perfect straight line while the robot crabs. The firmware's
own odometry (same-generation pairs, `odom_.integrate()` after both collects) sees the truth —
which is why the TLM `pose` row agrees with OTOS/truth (+31) and only the host encoder view
lies.

**Fixes (both required, both preserve the per-port select→settle→collect interleave):**
1. Stage wheel targets once per cycle at a point where **both** motor ticks write the same
   generation — e.g. `drive_.tick()` above `motorL_.requestSample()` (both wheels then apply
   this cycle's stage; symmetrically one cycle old). Note: this is what the retired 112-005
   hoist did. 118-001 threw it out along with the select-ordering fix, but the glued-encoder
   bug was select ordering only; the hoist was the part keeping L/R actuation symmetric.
2. Move `updateTlm()`/`emit` after `motorR_.tick()` (start of the pace block) so frames carry
   same-generation encoder pairs. Fixing (1) without (2) leaves twist/encpose skewed during
   ramps; fixing (2) without (1) makes the crab *visible* but still present.

Repro script: `docs/code_review/2026-07-22-turn-execution-review-scripts/straight_drift_repro.py`.
