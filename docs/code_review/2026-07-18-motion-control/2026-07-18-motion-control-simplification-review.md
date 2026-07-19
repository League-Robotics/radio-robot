# Motion Control Review: Why the Blips Exist and What to Delete

**Date:** 2026-07-18
**Scope:** `src/firm/motion/` (executor, jerk_trajectory), `src/firm/app/`
(pilot, drive, robot_loop), `src/firm/devices/` (nezha_motor, velocity_pid),
`src/sim/` (sim_plant, sim_harness), `src/tests/sim/plant/` (wheel_plant),
`src/firm/config/boot_config.cpp`, `src/firm/messages/planner.h`.
**Trigger:** D700 straight and 360° pivot wheel-speed traces showing (a) a
slow actual-vs-commanded ramp, (b) a ~44 mm/s velocity hump after the
straight should have finished, (c) a ±15 mm/s tail after the pivot should
have finished.

---

## 1. Verdict

Both of the positions in the room are partly wrong.

**The agent's position** — "this is normal, better than before, can't do
better" — is wrong. The end-of-move blips are not physics. Each one is a
specific compensation mechanism in `executor.cpp`/`pilot.cpp` firing, and
each of those mechanisms exists to patch a hole left by a missing feedback
term. The architecture can land these moves with no visible terminal
artifacts, using *less* code than it has now.

**The stakeholder's position** — "it's a simulator, there's no friction, it
should be perfect" — is also wrong, on one point. The simulator is not an
ideal plant, *by deliberate design*. `WheelPlant` models a first-order motor
with a 130 ms time constant (`kDefaultTau = 0.13`, matched to the
bench-measured 120–140 ms actuation lag), and the sim runs the motor stack
under the full production write shaping — integer-percent duty quantization,
3% output deadband, 100 ms reversal dwell, 40 ms write throttle — per the
2026-07-18 "PARITY" decision in `sim_harness.h`. The wheels in the sim do
**not** respond instantly to commands, on purpose, so that the controller
that works in sim works on the robot.

Given that plant, the correct conclusion is not "the sim should be perfect"
and not "the blips are inevitable." It is: **the controller is open-loop in
the one dimension that determines where the move ends, and every terminal
artifact on the traces is a hand-tuned patch standing in for the missing
loop.** Close the loop, delete the patches.

---

## 2. What the traces actually show, line by line

### 2.1 The plotted signals

"cmd L/R" is the TLM `cmd_vel` field — the per-wheel velocity setpoint
handed to the motor PID (post-profile, post-lead, post-heading-PD,
post-floor). "actual L/R" is `NezhaMotor::velocity()` — the *EMA-filtered*
measured wheel velocity (`velFiltAlpha`). With `tovez.json`'s
`vel_filt = 0.3`, the displayed "actual" carries ~100+ ms of pure display
lag on top of real plant lag. Part of the ramp gap you see is the estimator,
not the wheel.

### 2.2 The straight (D700 @ 150 mm/s)

Planned duration at a=800, j=8000 is ~5.0 s; the trace spans ~5.3 s. The
sequence that produces the tail:

1. `Executor::plan()` solves the linear channel **once** to
   `effectiveDistance_` — plus a hand-fit "terminal straight-lead" of
   `1.5 + 0.102·v_cruise` mm (`kStraightLeadBias/kStraightLeadSlope`,
   executor.cpp:123–124). At 150 mm/s that's **16.8 mm of deliberate
   over-plan**, sized so the lagging plant's rest lands on the true target.
2. `tick()` samples the profile velocity **0.2 s in the future**
   (`plan_lead`, the `peek(elapsed + linLead)` machinery,
   executor.cpp:901–916) to pre-compensate actuation transport lag.
3. The wheel loop converges at whatever rate the motor gains + filter give
   it (with `tovez.json`'s bench-tuned kff=0.0008/ki=0.005 the crawl to
   cruise takes ~1 s; with `tovez_nocal.json`'s kff=0.002 it is ~5× faster).
   The profile does not know or care — nothing feeds tracking error back.
4. The profile runs out. If measured distance is short by more than the 3 mm
   settle epsilon and the plant has stopped coasting, tick() requests a
   **terminal top-up**: a fresh from-rest mini-profile over the remainder
   plus a 3 mm "cross-bias" (executor.cpp:937–971, 639–650). A from-rest
   jerk profile over ~7 mm at j=8000 peaks at **46 mm/s** — that is the
   ~44 mm/s hump on the trace. If the first top-up lands short again, it
   fires again (the second, smaller bump).
5. Completion fires on the *crossing* test mid-top-up, while the commanded
   velocity is still nonzero. `Pilot::tick()` stops staging twists the
   moment `state() == kIdle` (pilot.cpp:70–72) — but **nothing zeroes
   Drive**. `Drive::tick()` keeps writing the last staged twist every cycle
   until the deadman lease expires up to **300 ms later**
   (robot_loop.cpp:463–477). That is the ~8–10 mm/s shelf after the hump.
   This one is a plain bug, independent of any architecture opinion.

### 2.3 The pivot (360°)

Planned duration at ω=4 rad/s is ~2.0 s (cruise wheel speed 256 mm/s + PD
trim ≈ the observed ~290); the trace spans ~2 s of turn plus ~1 s of tail.
The tail:

1. At cruise, the plant runs ~0.5–0.6 rad *behind* the planned heading
   (lag × rate — the code's own comment at executor.cpp:485–497 says
   exactly this). The pivot-overshoot lead (`kPivotOvershootLeadSlope`,
   executor.cpp:138) pre-brakes to cancel the rate-dependent bulk.
2. The profile ends; a residual heading error of a degree or three remains.
   The heading PD (`Pilot::tick`, kp=1–6) tries to close it, but a small
   error × kp commands a wheel speed **below the 3% duty deadband**
   (~15 mm/s), so the plant doesn't move — the PD stalls.
3. The **minimum-command floor** (`min_speed = 16 mm/s`, pilot.cpp:58–65)
   punches through the deadband: ω is floored at 2·16/128 = 0.25 rad/s —
   **±16 mm/s per wheel**, the ±15 tail on the trace — until the error is
   inside the 3° dwell tolerance, where the floor disengages and the plant
   coasts.
4. The dwell gate then has to confirm the landing: tolerance test + an
   EMA-filtered rate test + a leaky hold counter for 150 ms, with a
   `2×duration + 6 s` timeout backstop behind it
   (executor.cpp:988–1113).

Every feature of the tails is one of these mechanisms. None of it is noise,
none of it is floating-point, none of it is inevitable.

---

## 3. Root cause: feedforward without feedback on the dominant channel

The design tracks a time-based reference with velocity feedforward only:

```
v_cmd(t) = v_ref(t + lead)                 ← linear channel: NO feedback
ω_cmd(t) = ω_ff(t + lead) + PD(θ_err)      ← heading: feedback exists
```

A first-order plant with time constant τ following a velocity ramp sits a
distance `v·τ` behind the reference — **19.5 mm at 150 mm/s with τ=130 ms**.
Nothing in the linear channel ever measures or closes that gap during the
move; the code instead predicts it, pre-compensates it, and mops up the
residue afterward. Count the mechanisms that exist *only* because that one
feedback term is missing:

| # | Mechanism | Where | Tuned constants |
|---|-----------|-------|-----------------|
| 1 | `plan_lead` velocity peek + ramp-in + short-profile gate | executor.cpp:901–916 | 0.2 s |
| 2 | Terminal straight-lead (over-plan the distance) | executor.cpp:104–124, 681–684 | 1.5 mm, 0.102 s |
| 3 | Pivot overshoot lead (scale lead with cruise rate) | executor.cpp:127–138, 910–911 | 0.009 s/(rad/s) |
| 4 | `heading_lead_bias` + measurement-age projection (`headingLead()`, `thetaMeasLead`) | heading_source.*, executor, pilot | −0.05 s |
| 5 | `terminal_lead` (predicted-heading dwell test) | executor.cpp:977–989 | 0.0 s (unused today) |
| 6 | Distance settle epsilon + cross-bias | executor.cpp:45–71, 639–645 | 3 mm, 3 mm |
| 7 | Terminal top-up mini-profiles | executor.cpp:937–971 | 5 mm/s rest gate |
| 8 | Same-sign overshoot carry between commands | executor.cpp:264–282, 328–337 | — |
| 9 | Minimum-command floor | pilot.cpp:40–65 | 16 mm/s |
| 10 | Dwell rate EMA + leaky hold counter | executor.cpp:991–1113 | α=0.3, 150 ms |
| 11 | STOP_TIME backstops (two of them) | executor.cpp:73–79, 1111, 1159 | 2×, 6 s |
| 12 | 40 mm gross-divergence reanchor (the surviving tier of a deleted 3-tier system) | executor.cpp:87–100, 454–482 | 40 mm, 60 ms |

Twelve mechanisms, ~15 tuned constants, several hundred lines of code and
comment — standing in for one line of control law. The straight-lead fit
(`0.102 s × cruise`) is the plant time constant itself, re-measured
empirically and frozen into a planner constant. It holds only for the exact
gains, filter, cycle rate, and τ it was fit against; retune any of them
(as `tovez.json`'s bench history shows happens regularly) and the fit drifts
→ the top-up returns → the blips return. That is why this has consumed
weeks: **each patch is calibrated to the residue the other patches leave**,
so every change reopens the tuning.

The repository's own history says the missing loop used to exist: the dead
`track_k_s`/`track_k_theta`/`track_k_cross` config fields are the gains of
the deleted `source/drive` tracker. The 102–107 greenfield rebuild kept the
feedforward and dropped the feedback, and sprints 109-005 through 109-010
have been re-inventing the feedback's effects one incident at a time.

---

## 4. The fix: track the reference, then delete the patches

Ruckig already produces exactly what a tracking controller needs: a
time-parameterized reference `(s_ref(t), v_ref(t))` per channel. Use it.
Each tick:

```
s_err = s_ref(t) − s_meas                    // measuredPathSinceActivation_ already exists
v_cmd = v_ref(t) + k_v · s_err               // clamp to ±v_ceiling; k_v ~ 2–4 [1/s]

θ_err = θ_ref(t) − θ_meas                    // already exists verbatim
ω_cmd = ω_ff(t) + k_h · θ_err (+ k_d·rate)   // already exists verbatim
```

Completion, both modes:

```
done = (t ≥ duration + margin) AND |s_err| < s_tol AND |θ_err| < θ_tol
timeout = t ≥ duration·2 + margin            // single backstop, kTimeout
```

Why this eliminates the artifacts rather than relocating them:

- **During cruise** the position loop drives the lag error to zero in steady
  state (the plant integrates velocity, so a constant `k_v·s_err` trim wipes
  out the `v·τ` deficit; the error is transient and bounded, not
  accumulated). Nothing lands 17 mm short anymore, so there is nothing for
  a straight-lead to predict or a top-up to mop up.
- **At the end** `v_ref → 0` and `s_ref → target`, so the command decays to
  `k_v·s_err` — a smooth, proportional approach that is *part of the same
  commanded curve*, not a separate from-rest mini-profile. No hump, by
  construction. When `|s_err| < s_tol` (which the loop actively drives it
  to), completion fires with the command already at rest — no crossing test,
  no stale nonzero twist.
- **The deadband** stops needing a floor in the planner, *provided the gains
  and tolerances satisfy one inequality*: the P-term at the tolerance
  boundary must exceed the deadband-equivalent speed, i.e.
  `k_h·θ_tol ≥ ω_deadband` and `k_v·s_tol ≥ v_deadband`. With the 3% duty
  deadband (≈15 mm/s ≈ 0.25 rad/s at track 128): heading needs
  `k_h ≥ 0.25/0.052 ≈ 5` at 3° tolerance — `tovez.json`'s bench value 6.0
  satisfies it, the boot default 1.0 does not (raise it or widen the
  tolerance); linear needs `k_v ≥ 5` at 3 mm, or `s_tol = 5 mm` at `k_v = 3`.
  If the inequality can't be met, keep the floor — but then it is ten lines
  implementing a stated constraint, not a discovered workaround. On
  hardware, stiction belongs to the wheel PID's integral term — where
  `tovez.json` already has `vel_ki` — not to the trajectory layer.
- **Leads become unnecessary** rather than mistuned: feedback replaces
  prediction. `plan_lead`, `terminal_lead`, `heading_lead_bias`, the
  straight-lead, and the pivot lead all approximate `k·(ref − meas)` with
  `k·(ref(t+Δ) − ref(t))` — the open-loop Taylor expansion of the closed
  loop. Keeping the OTOS measurement-age projection is defensible on
  hardware; it should be a HeadingSource detail, not three separately
  fitted planner constants.

### Why this does not resurrect the 087-009 / terminal-reversal failures

The two documented disasters this codebase is (rightly) scarred by were
both caused by **re-solving trajectories from measured state near the
target** — a time-optimal solve from nonzero velocity to a nearly-zero
remainder is an overshoot-and-reverse plan, and each re-solve reset the
clock (the ±100 mm/s terminal ringing described at executor.cpp:469–481 and
485–497). The tracking law above never solves from measured state — the
JerkTrajectory seeding contract stays intact, solves stay
plan-state-seeded, one per command. Feedback enters only as a *bounded
velocity trim* on the sampled reference: near the target its authority is
`k_v · s_tol` (≈ 10 mm/s at k_v=3, tol=3 mm) — below the deadband, incapable
of commanding the old full-authority reversal. The gross-slip reanchor
(40 mm) can stay as the recovery path for genuine wheel-stall, unchanged.

---

## 5. What to delete

### 5.1 In `motion/executor.{h,cpp}` (≈ 300–400 lines incl. comments)

| Delete | Lines (cpp) | Replaced by |
|---|---|---|
| Terminal top-up: `pendingLinearRetarget_` path, `kTopUpMeasuredRestVelocity`, cross-bias, `retarget()` call site | 96–102, 617–651, 937–971 | P-term terminal approach |
| Straight-lead: `kStraightLeadBias/Slope`, the `linPosTarget` padding | 104–124, 666–684 | P-term cruise/decel tracking |
| Pivot overshoot lead: `kPivotOvershootLeadSlope`, `rotTargetLead` | 127–138, 904–911 | same |
| `plan_lead` peek machinery: `linLead/rotLead`, ramp-in, short-profile gate, `peek()` call pair | 873–916 | sample at `t`, add feedback |
| `terminal_lead` / `thetaErrLead` predicted-dwell | 975–989 | plain `θ_err` test |
| Overshoot carry: `pendingOvershoot_`, `effectiveDistance_` adjustment, clamp edge case | 264–282, 328–337 | each leg ends within tol; carry is sub-tolerance by construction |
| Dwell machinery: `dwellRateFilt_`, leaky counter, `withinRate`, `crossedTarget` special case | 991–1137 | `|θ_err| < tol` held for one `arrive_dwell`, plus the single timeout |
| One of the two STOP_TIME backstop branches (keep a single timeout in the shared completion test) | 1140–1164 | unified completion |
| `checkDivergence()` retarget tier remnants and its cached-measured-state plumbing (`lastMeasuredVelocity_` stays only if the 40 mm reanchor stays) | 430–500 | optional: keep reanchor only |

`Executor::Twist` shrinks from 8 fields with a documented mode-dependent
overload on `omega` (executor.h:196–254) to
`{v_ff, omega_ff, s_ref, theta_ref, active}` — the overload, `headingActive`,
`withinTolerance`, `omegaDes`, and `thetaMeasLead` all go. The 119-line file
header shrinks with them.

### 5.2 In `app/pilot.{h,cpp}`

- The minimum-command floor block (pilot.cpp:40–65) and `minSpeed_`.
- The `thetaMeasLead` plumbing (use `heading()`; keep age projection inside
  HeadingSource if hardware needs it).
- Add the one new term: `v = twist.v_ff + kv_ * (twist.s_ref − odomPath_)`,
  and **fix the stale-twist bug**: when `executor_.state()` transitions to
  `kIdle`, stage `drive_.setTwist(0, 0)` once. Today the robot creeps at the
  last staged twist for up to 300 ms after every terminal command until the
  deadman flushes it (robot_loop.cpp:463–477). This fix is worth making
  *today* regardless of everything else in this review.

### 5.3 In `msg::PlannerConfig` (35 fields → ~14)

Dead already — consumed by nothing but the patch-merge and the wire table
(verified by grep): `arrive_tol`, `turn_in_place_gate`, `v_wheel_max`,
`steer_headroom`, `wheel_step_max`, `track_k_s`, `track_k_theta`,
`track_k_cross`, `trim_v_max`, `trim_omega_max`, `replan_err_pos`,
`replan_err_theta`, `replan_hold`, `replan_min_period`, `replan_max`,
`handoff_tol_pos`, `handoff_tol_v`, `arrive_vel_tol` — **18 fields**, plus
their `PlannerConfigPatch` arms, `Pilot::applyPlannerPatch` merge lines, and
`wire.cpp` rows (reserve the field numbers in the proto instead).

Deleted by the redesign: `plan_lead`, `terminal_lead`, `heading_lead_bias`
(or fold into HeadingSource), `min_speed` (as a planner field).

Kept (~14): the seven kinematic limits (`a_max`, `a_decel`, `v_body_max`,
`j_max`, `yaw_rate_max`, `yaw_acc_max`, `yaw_jerk_max`), two tracking gains
(`track_k_v` new/revived, `heading_kp`, optionally `heading_kd`), two
tolerances (`s_tol` — rename of the settle epsilon into an honest config
field — and `heading_dwell_tol`), `arrive_dwell`, `heading_source`.

### 5.4 What to keep, explicitly

- **`JerkTrajectory`** — a clean Ruckig wrapper with a well-designed seeding
  contract. Untouched.
- **Queue, ring, completion events, enqueue outcomes, flush semantics** —
  wire-protocol machinery, orthogonal to the control problem. Untouched.
- **Mode classification (kTimed/kArc/kPivot) and the arc slaving**
  (`headingRatioPerMm_`, θ_ref = ratio·s_ref) — good design; with a linear
  tracking loop it gets *more* accurate, since s tracks s_ref.
- **`computeExitVelocity()` boundary carry** — this is what makes chained
  tours smooth; it is independent of the compensator stack. Keep.
- **`resolveFromRest()`**, **emergency stop on solve-fail**, **the unwrapped
  heading accumulator** (109-009) — all correct fixes for real failure
  modes. Keep.
- **NezhaMotor write shaping and MotorArmor on hardware** — the reversal
  dwell/deadband protect against a documented, reproducible encoder-wedge
  hardware fault. Not gold-plating. (One stale comment: motor_armor.h:16
  claims the sim composes bare motors; sim_harness.h wraps them in armor
  since the PARITY change.)

---

## 6. On "the sim should be perfect"

Two coherent stances exist; pick one deliberately instead of arguing with
the traces:

1. **Keep the realistic plant** (recommended). τ=130 ms and the write
   shaping are what the real robot does; a controller that lands clean under
   them lands clean on hardware. With the tracking loop, the residual
   tracking error at cruise decays to ~0 and the terminal approach is
   monotone — the traces will *look* essentially perfect at plot scale, lag
   in the ramp notwithstanding (and half the visible ramp lag is the
   `vel_filt` display filter anyway).
2. **Add an ideal-plant sim preset** (τ→10 ms, deadband/dwell explicitly 0,
   kff exact) as a *diagnostic mode*, so "controller is wrong" and
   "controller is mistuned for the plant" are separable questions in future
   debugging. Cheap: every knob already exists (`MotorConfig`
   reversalDwell/outputDeadband take explicit 0; WheelPlant takes τ in its
   constructor — only a SimPlant constructor parameter is missing).

What is *not* coherent is the current middle position: a deliberately laggy
plant driven by a controller that pretends lag doesn't exist, reconciled by
a dozen fitted constants.

---

## 7. Numeric cross-checks (claims vs. traces)

| Claim | Computation | Trace |
|---|---|---|
| D700@150 duration | 700/150 + ramp ≈ 4.95 s | ~5.3 s (incl. slow wheel-loop ramp) ✓ |
| Straight-lead magnitude | 1.5 + 0.102·150 = 16.8 mm ≈ v·τ_eff | matches the open-loop lag deficit 19.5 mm at τ=0.13 ✓ |
| Top-up hump peak | from-rest S-curve, j=8000: 7 mm → 46 mm/s | ~44 mm/s hump ✓ |
| Pivot duration | 2π at ω=4, a=20, j=80 → 2.0 s | ~2.0 s ✓ |
| Pivot cruise wheel speed | 4.0·128/2 = 256 + PD trim | ~290 ✓ |
| Turn-tail floor | 2·min_speed/track = 0.25 rad/s → ±16 mm/s | ±15 mm/s tail ✓ |
| Post-completion shelf | deadman lease 300 ms at last staged twist | ~0.3–0.4 s shelf at 8–10 mm/s ✓ |

---

## 8. Suggested execution order

Small, independently verifiable steps; the wheel-speed trace and the
existing sim tour gates are the acceptance instrument for each.

1. **Fix the stale-twist-on-idle bug** in Pilot (zero Drive on the
   running→idle transition). One line; removes the post-completion shelf on
   every terminal command immediately.
2. **Add linear position feedback** (`k_v·(s_ref − s_meas)`, clamped) in the
   kArc path, `plan_lead` set to 0. Verify D700 endpoint error < 3 mm with
   the straight-lead constants forced to 0.
3. **Delete** top-up, straight-lead, cross-bias, overshoot carry, and the
   epsilon-widening history; keep `s_tol = 3 mm` as an honest tolerance.
4. **Simplify the pivot terminal**: keep the existing heading PD as the
   tracker; delete the pivot-overshoot lead, `terminal_lead`, the
   min-command floor, and the leaky/EMA dwell machinery; completion =
   profile done + `|θ_err| < tol` for `arrive_dwell`, one timeout.
5. **Delete the `plan_lead` peek machinery**; evaluate whether hardware
   still wants the OTOS age projection (keep it inside HeadingSource if so).
6. **Strip the 18 dead PlannerConfig fields** end-to-end (proto, generated
   structs, patch merge, wire rows — reserve field numbers).
7. **Optional:** ideal-plant sim preset for future diagnostics.

Expected net effect: `executor.cpp` ≈ 1170 → ~700 lines, `executor.h`
≈ 640 → ~300, `pilot.cpp` gains ~10 and loses ~30, PlannerConfig 35 → ~14
fields, tuned constants in the motion path ~15 → ~4 (two gains, two
tolerances) — and the traces stop being embarrassing for the structural
reason that there is no longer any mechanism whose job is to twitch the
wheels after the move is over.
