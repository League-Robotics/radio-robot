# Distance-Trim Limit Cycle: Sprint 112 Follow-Up Review

**Date:** 2026-07-19
**Scope:** the sprint-112 result (tickets 112-001…004, merged at `2c7a6fd0`):
`src/firm/app/pilot.{h,cpp}` (bounded position trim), `src/firm/app/drive.cpp`
(acceleration feedforward), `src/firm/app/robot_loop.cpp` (cycle ordering),
`src/firm/config/boot_config.cpp` / `src/scripts/gen_boot_config.py`
(`distance_kp`, `distance_tol`, `actuation_lag` defaults).
**Trigger:** D700 wheel-speed trace after the sprint: commanded speed
oscillates rail-to-rail between 100 and 200 mm/s at ~1.15 s period for the
whole cruise, then holds −50 mm/s for ~0.8 s after the profile ends.
**Prior doc:** `2026-07-18-motion-control-simplification-review.md`
(architecture rationale; not needed to act on this one).

---

## 1. Summary

The sprint fixed what it set out to fix — the ramp now tracks (actual
reaches cruise in ~0.6 s, courtesy of the new `actuation_lag·a`
feedforward), and the patch stack is gone. What replaced the old terminal
blips is a textbook control instability: **the new position-feedback trim
is tuned past the loop's ultimate gain, and its ±50 mm/s clamp converts the
divergence into a sustained relay oscillation.** One number is wrong
(`distance_kp = 8`), one structural property makes that number fragile
(~4.5 cycles of dead time in the measure→act path), and one gap in the
acceptance tests let it ship (endpoint-only grading). All three are cheap
to fix.

Not defects, for the record: the ~260 mm/s spike at motion start is the
acceleration feedforward's inversion kick (0.13 s × 800 mm/s² ≈ 104 mm/s on
top of v_ref) — it is the thing that fixed the ramp. Leave it alone.

## 2. Reading the trace

Every feature of the plot is accounted for:

- **Rails at exactly 100 and 200 mm/s.** Cruise is 150; the trim clamp
  (`App::kDistanceTrimCeiling`) is ±50. `cmd = v_ref + clamp(kp·s_err)`
  pinned alternately at both rails means the trim is saturated
  essentially all the time — a relay, not a trim.
- **Period ~1.15 s.** Matches the loop's phase condition (below) at the
  measured dead time. This is not encoder noise or the write quantizer;
  those produce cycle-rate dither, not a 1 Hz wave.
- **Actual oscillating ~105–195, lagging cmd ~0.3 s.** The wheel loop's
  gain at 5.5 rad/s is ~0.93 with ~0.3 s of combined lag — consistent.
- **−50 mm/s shelf for ~0.8 s after the profile ends.** The last
  oscillation lobe: the plant crossed the target while the trim rode the
  +50 rail into the decel, leaving ~30–40 mm of overshoot; with `v_ref = 0`
  the trim rails at −50 and backs the robot up until `s_err` re-enters
  tolerance. It is the same instability's terminal symptom, not a separate
  bug.

## 3. Diagnosis: gain past the ultimate, clamp turning it into a relay

The trim closes this loop:

```
s_err → distance_kp → v_cmd → [dead time L] → wheel lag (τ) → ∫ → s_meas
```

**Dead time, counted against `robot_loop.cpp`'s actual cycle order** (sim
cycle = 50 ms):

1. Pilot computes the trim in cycle N (block 3) from `odom_.lastDistance()`
   — which was integrated in cycle N−1's kPace block from encoders
   collected at the top of N−1 (≈ 1.5 cycles of measurement staleness).
2. `drive_.tick()` at the top of N+1 stages the wheel targets — *after*
   that cycle's motor ticks have already run, so (its own comment) they are
   "consumed on the NEXT cycle's ticks": duty is written at the top of N+2.
3. The plant integrates the new duty in the tick before N+3; the effect
   appears in measurements Pilot sees at N+3½–N+4.

Total: **≈ 4.5 cycles ≈ 0.225 s of dead time**, plus the wheel loop's
~65 ms first-order lag (τ_plant 0.13 s, halved by the velocity PID with the
active `tovez_nocal` gains).

**Phase condition for sustained oscillation** (−180° around the loop):
the velocity→position integrator contributes −90°, so oscillation sits
where `ω·L + atan(ω·τ) = π/2`:

| L (dead time) | ω_u | period | k_u = ω_u/\|G(jω_u)\| |
|---|---|---|---|
| 0.15 s | 7.5 rad/s | 0.84 s | 8.3 /s |
| 0.20 s | 6.0 rad/s | 1.05 s | 6.4 /s |
| **0.225 s** | **5.5 rad/s** | **1.15 s** | **5.8 /s** |
| 0.25 s | 5.0 rad/s | 1.25 s | 5.3 /s |

The observed 1.15 s period lands on L ≈ 0.225 s — independently confirming
the dead-time count — and puts the ultimate gain at **k_u ≈ 5.8 /s. The
shipped `distance_kp = 8.0` is ~1.4× the ultimate gain.** The loop
diverges; the ±50 clamp bounds the divergence; the result is a steady
limit cycle at exactly the clamp rails. Once saturated, the clamp *is* the
loop gain, which is why the oscillation neither grows nor dies.

### Why the 112-004 sweep passed it

Two reasons, both worth internalizing:

- The sweep's own data brackets this edge — kp 10 fails 1/40, kp 12 fails
  4/40, kp 15 fails 10/40. A gain whose neighbors fail intermittently is
  not "the highest passing gain," it is the edge of the unstable region
  measured with too few samples. Tuning rules exist precisely for this:
  Ziegler–Nichols-style backoff puts kp at 0.4–0.5·k_u, not 1.0·k_u.
- The sweep graded *endpoint completion* on a short same-boot leg. This
  failure mode lives in *sustained cruise* — the scenario never held cruise
  long enough to let the ring build, and a completion check can't see a
  cruise oscillation that still happens to complete.

### Why kp was pushed this high in the first place

Ticket 003 sized the gain against the deadband inequality
(`distance_kp · distance_tol ≥ v_deadband`, 15 mm/s for the active config)
at a fixed `distance_tol = 3 mm`, which demands kp ≥ 5 — right at k_u.
The inequality itself is correct; solving it with gain instead of
tolerance is what created the conflict. The tolerance is the free variable
here: the gain is bounded above by stability, the tolerance is not.

## 4. Fixes

Ordered; the first alone removes the oscillation and the shelf.

1. **Retune the pair: `distance_kp` 8 → 2.5–3, `distance_tol` 3 → 6 mm.**
   kp ≈ 0.4–0.5·k_u gives ~50° of phase margin; kp·tol = 15–18 mm/s keeps
   the deadband inequality satisfied. With the acceleration feedforward now
   carrying the tracking load, the trim's job is residuals — a few mm/s in
   cruise, never the rail. 6 mm on a 700 mm leg is 0.9%, and it is a
   completion *gate*, not the expected accuracy; a converged loop normally
   lands well inside it. (If 6 mm offends, recover 3 mm after fix 2 raises
   k_u.)
2. **Cut ~2 cycles of dead time in `robot_loop.cpp`.** (a) Move
   `drive_.tick()` above the two motor ticks, so targets staged last cycle
   are written *this* cycle instead of next (−1 cycle). (b) Move
   `odom_.integrate()` from the kPace block to immediately after the two
   encoder collects, so Pilot reads this-cycle measurement (−1 cycle).
   Both are pure-math moves — neither relocates a bus transaction, so the
   settle/clearance discipline is untouched. Dead time drops to
   ~0.10–0.125 s → k_u ≈ 10–12: kp 4–6 becomes safe if ever wanted, tuning
   stops being delicate, and the heading PD (kp = 6, same loop shape, same
   dead time, currently enjoying less margin than anyone has measured)
   inherits the improvement for free.
3. **Add a cruise-quality assertion to acceptance**, next to the endpoint
   ones: over the cruise phase of a D700 run, the trim must not sit on its
   clamp rail for more than a few consecutive cycles (equivalent form:
   cruise-phase `cmd_vel` variance below a small bound). This is the
   assertion that would have caught this before merge, and it directly
   encodes the design intent — "the trim is a trim."
4. **Cleanup:** `plan_lead`/`terminal_lead` are unread since 112-001 but
   still set in `boot_config.cpp` — fold them into the dead-field strip
   already planned for the 18 legacy PlannerConfig fields.

## 5. Verification checklist for the retune

- D700 @ 150: cruise `cmd_vel` stays within a few mm/s of 150 after the
  accel-FF kick decays; no rail contact; endpoint |error| < tol; no
  commanded reversal after `v_ref` reaches 0; robot stationary within one
  cycle of completion.
- Same run under `tovez.json` gains (kff detuned, ki active): same
  qualitative trace — this is the config whose deadband equivalent
  (~37.5 mm/s) the inequality does not clear, so confirm the wheel PID's
  integrator, not the trim, digests the last few mm.
- 360° pivot: unchanged from the 112-004 result (the heading loop was not
  retuned here), then re-checked after fix 2 for reduced terminal trim
  activity.
- Sweep kp ∈ {2, 3, 4, 6, 8} × the cruise assertion: pass/fail boundary
  should now sit ≥ 2× the shipped gain.
