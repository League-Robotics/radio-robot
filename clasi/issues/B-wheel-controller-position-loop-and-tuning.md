---
status: pending
priority: medium
---

# Wheel controller: the I term is a position term, and the actuator quantizes at 8.46 mm/s

**Measured 2026-08-03 on `vevov`, bare motors and a loaded wheel stack.
Findings and a working-but-unmerged implementation. NOT fixed on master.**

Built and verified on a throwaway branch (`position-pid` in the
`radio-robot-elite-vmin` worktree), deliberately not merged. §7 of this
document carries every firmware change as readable code, so the branch is
not needed — take the patch.

Patch: `clasi/issues/attachments/wheel-controller-2026-08-03/firmware-changes.patch`, `clasi/issues/attachments/wheel-controller-2026-08-03/gate-changes.patch`.
Charts: `clasi/issues/attachments/wheel-controller-2026-08-03/kff00_r3.png` (bare motors, final settings),
`clasi/issues/attachments/wheel-controller-2026-08-03/inertia_tuned.png` (loaded wheel stack).
Working copy at time of writing:
`/Volumes/Proj/proj/RobotProjects/radio-robot-elite/src/tests/bench/output/tuned_20260803/`
(bench output, may be cleaned — attachments above are durable).

Related: [[A-stop-path-runaway-single-stop-does-not-land]] — that defect had
to be fixed first, because before it every "overshoot" number here included
a tail that was a runaway rather than a control error.

## Sprint-sized decisions this raises

1. Does the position-reading I term land as-is, or as an explicit outer
   stage? An outer stage composes onto a HiWonder/Yahboom board whose own
   velocity PID becomes the inner loop; the in-place version does not.
2. `iMax` clamps `ki · error`, so it is in the wrong units and shrinks the
   position memory as `ki` rises. `posErrMax` (mm) replaces it — that is a
   config-surface change.
3. `aSteady` and the deficit-flag policy become vestigial for Stage B.
4. The tuned values are **bare-motor** values. `kff = 0` in particular
   should be re-tuned under load, not inherited.

---

## Full write-up

### Wheel controller — findings and transfer package

**vevov (Nezha brick). 2026-08-03. Every number is a median of 3
interleaved repeats of the standing velocity-profile gate, both profiles,
450 mm commanded over 3 s.**

**This code is NOT being checked in.** §7 carries every firmware change in
full so this document alone is enough to re-apply them. Patches are beside
this file: `firmware-changes.patch`, `gate-changes.patch` (in
`src/tests/bench/output/tuned_20260803/`).

Three defects and a tuning result. The tuning result is the smallest of them.

---

### 1. Where it started and where it ended

Bare motors, both wheels:

| | square L / R | imbal | trapezoid L / R | imbal |
|---|---|---|---|---|
| shipped | 111.3 / 105.1 | 1.059 | 113.3 / 106.7 | 1.063 |
| final | **104.2 / 103.8** | **1.004** | **101.1 / 100.9** | **1.000** |

Loaded wheel stack (4–5 wheels, one motor):

| | square | trapezoid |
|---|---|---|
| shipped | 102.2% | 109.3% |
| final | 103.3% | **100.7%** |

**Final settings:** `vmin 20, kp 1.0, ki 12, posErrMax 5, iMax 100, kaw 100,
kff 0`

The left/right imbalance — the presenting problem — is gone, 1.06 → 1.00 on
both profiles. The trapezoid lands within ~1% on both rigs. The square runs
~3–4% long, only partly explained (§6).

### 2. The stop never reached the motor

Full write-up: [[A-stop-path-runaway-single-stop-does-not-land]]. A stop issued **once** and
not repeated did not land: host silent meant 936 mm of continued travel with
no decay, and `estop()` failed 5 of 6 attempts. The brick latches its last
speed and does not reset on an nRF52 reset, so a lost zero write is
permanent. Both existing defences gate on the **encoder**, so both are
disarmed by a wheel that reads at rest.

Fixed by §7-A and §7-B. 16/16 runs stop afterwards, every path.

Read it as a **measurement** fix too: before it, every "square overshoot"
number included a tail that was a runaway, not a control error.

### 3. The I term was a position term computed the worst possible way

`integral += ki · err · dt`, with `err` in mm/s and `dt` in s, accumulates
**millimetres** — commanded position minus measured position. The velocity
loop always contained a position controller. It just built its position
estimate by summing a *derived, quantized* velocity (sd 8.0 mm/s, §4)
instead of reading the encoder's position register, so every dropped or
manufactured-zero sample permanently deleted real distance from the sum.

Replaced with a direct read (§7-D). Same control law, same units (`ki` stays
1/s), same output — the loop still commands velocity, which is all the motor
takes. No accumulator, so no windup, no `steady` gate to freeze it, no reset
to lose it.

This is what fixed the imbalance: 1.05–1.07 → 1.000–1.006. Note the
direction — *more* position gain gives *better* balance, the opposite of how
every velocity-side knob behaved.

**A unit-domain bug fell out of it.** `iMax` clamps `ki · error`, so the
maximum position error the loop could REMEMBER was `iMax / ki` mm — at
ki=6/iMax=20 that is **3.3 mm**, smaller than every residual being chased.
Raising `ki` made the memory *shorter*. That is why the ki sweep plateaued
between 2 and 6. `posErrMax` replaces it with a clamp in millimetres.

### 4. The actuator's resolution is the floor

Duty reaches the brick as an integer percent and `kDutyPerSpeed = 0.001182`,
so **one duty count is 8.46 mm/s** — 5.6% of a 150 mm/s command. Measured:
plateau velocity clusters at exactly −1/0/+1 counts, sd 8.0 mm/s against an
8.46 mm/s step. The imbalance being chased was 0.43 of ONE count.

No gain can command a value the output cannot represent, which is why the
ki/kff/aSteady sweeps returned nothing at that stage. Fixed with a
sigma-delta on the rounding residual (§7-C).

Ripple, plateau sd in duty counts:

| rig / config | square | trapezoid |
|---|---|---|
| bare, tuned | 0.80 | 0.73 |
| loaded, tuned | 0.69 | 0.55 |
| loaded, shipped | 1.41 | 1.04 |

Most of the smoothing is the sigma-delta plus the position loop; inertia
adds a further ~15%.

### 5. What the sweeps said

Nine rounds. Four hypotheses died, and the dead ones were the informative
ones.

| round | hypothesis | verdict |
|---|---|---|
| 2 | ki closes the steady-state error | **confirmed** — plateau 1.058 → 1.013, stop 1.069 → 1.000 |
| 3 | residual banked in the ramp window (aSteady/kff) | **refuted** — both inside repeat spread |
| 5 | posErrMax/ki trade square against trapezoid | partly — fixed balance, not the common mode |
| 6 | kaw caps the square's sprint, trapezoid unaffected | **half** — square 107→101 confirmed; trapezoid balance degraded 1.002 → 1.091, refuting the rest |
| 7 | tight posErrMax + high ki separates them | **refuted** — square 104–108% across posErrMax 3..15, ki 6..20 |
| 8 | the sprint is the P term | **refuted** — kp 1.0 → 0.1, square unchanged |
| 9 | the sprint is the feedforward | **weakly supported** — square 105.1 → 104.2, trapezoid 99.8 → 100.9 |

Round 6 is load-bearing: square accuracy and trapezoid balance trade off
through a single clamp, so no value of `kaw` is good at both.

**Two null results were instrument failures, not physics.** Round 4 returned
five identical rows because `iMax = 20` saturated `ki · posError` in every
combination; the knobs never reached the output. And the earlier kff/aSteady
sweeps measured nothing because `Drive::update()` never set `cmdAccel`, so
`kaff · cmdAccel` was identically zero and `steady` was pinned true. **A
sweep that returns identical rows is a broken sweep until proven otherwise.**

### 6. What is still wrong

- **The square runs ~3–4% long, only partly explained.** It survived sweeps
  of posErrMax, ki, kp and kff. Only `kaw` moved it, and `kaw` clamps
  P + FF + I together — yet each of the three was individually ruled out.
  That is not consistent and I do not claim to understand it. On the LOADED
  rig, splitting at the command-zero moment shows 444 mm delivered
  (**98.7%**) with 21 mm of genuine inertial coast after — so there at
  least, much of the excess is physics the loop has no authority over.
- **`cmdAccel` on the WHEELS path (§7-E) is unvalidated.** Correct on its own
  terms — the field genuinely was never set — but the sweep on top of it
  returned ~50% dead runs. With `kff = 0` it is inert, which is the only
  reason it is safe to leave in.
- **`kff = 0` is a bare-motor answer.** The feedforward is the term most
  likely to matter under load. Re-tune it; do not inherit it.
- **Reversals are untested.** Nothing here exercised a direction change,
  where the reversal dwell and output deadband live.
- **`aSteady` and the deficit-flag policy are now vestigial** for Stage B —
  there is no accumulator to gate. Stage C still uses `aSteady`.
- **All tuning is RAM-only.** A power cycle reverts to the baked values.

---

### 7. The code

Against `master`. Full patches: `firmware-changes.patch` (528 lines),
`gate-changes.patch`. Comments are stripped here for length — the patches
carry them, and they explain the reasoning at each site.

### A. Unowned-motion guard — `app/robot_loop.cpp`, top of `cycle()`

Immediately before `drive_.tick(state_)`:

```cpp
if (!planner_.active() && !drive_.owns()) {
  state_.wheelLeft.cmdVelocity = 0.0f;
  state_.wheelRight.cmdVelocity = 0.0f;
}
```

`Drive::update()` publishes one zero pair on the expiry cycle then returns
early forever (`if (!owned) return;`); `Planner::update()` republishes only
its own command. Nothing said "no one is driving, so the speed is zero" on
the cycles between. Can only ever remove motion.

### B. Arm the stop re-assertion on every stop — `app/drive.cpp`, `tick()`

After `alreadyQuiet` is computed, before the `return`:

```cpp
if (commandedStop && !alreadyQuiet) stopEnforceCountdown_ = kStopEnforceTicks;
```

The window existed but was armed only by `estop()`, and its other half
trusted the encoder. Keying it on the commanded duty pair makes
re-assertion depend on what was commanded, not on what the encoder claims.
Adds writes only.

### C. Sigma-delta duty quantizer — `devices/nezha_motor.cpp`, `writeRawDuty()`

Replaces `int8_t pct = lroundf(duty * 100.0f);`:

```cpp
int8_t pct;
if (duty == 0.0f) {
    dutyCarry_ = 0.0f;
    pct = 0;
} else {
    const float wanted = clampf(duty * 100.0f + dutyCarry_, -100.0f, 100.0f);
    pct = static_cast<int8_t>(lroundf(wanted));
    dutyCarry_ = clampf(wanted - static_cast<float>(pct), -1.0f, 1.0f);
}
if (pct > 100) pct = 100;
if (pct < -100) pct = -100;
```

New member in `nezha_motor.h`:

```cpp
float dutyCarry_ = 0.0f;  // [percent] fractional, [-1, 1]
```

The carry is **discarded on commanded zero** — a residual left from the last
nonzero duty would round to ±1 and creep a stopped wheel, re-creating §2.

### D. The I term reads position — `app/drive.cpp`

```cpp
float Drive::fastPid(float posError, float err, float aCmd) const {
  const float proportional = gains_.kp * err;
  const float feed = gains_.kaff * aCmd;

  float integral = gains_.ki * posError;          // was: += ki*err*dt
  if (gains_.iMax > 0.0f) {
    if (integral > gains_.iMax) integral = gains_.iMax;
    if (integral < -gains_.iMax) integral = -gains_.iMax;
  }

  float pid = proportional + feed + integral;
  if (gains_.pidMax > 0.0f) {
    if (pid > gains_.pidMax) pid = gains_.pidMax;
    if (pid < -gains_.pidMax) pid = -gains_.pidMax;
  }
  if (!std::isfinite(pid)) return 0.0f;  // fail closed, never inject NaN
  return pid;
}

float Drive::positionError(float speed, const Types::RobotState::Wheel& wheel,
                           PositionRef& ref, float dt) const {
  if (speed == 0.0f || dt <= 0.0f || !wheel.connected ||
      wheel.positionEpoch != ref.epoch || !ref.armed) {
    ref.armed = (speed != 0.0f) && wheel.connected;
    ref.epoch = wheel.positionEpoch;
    ref.origin = wheel.position;
    ref.reference = 0.0f;
    return 0.0f;
  }
  ref.reference += speed * dt;                                  // [mm]
  float error = ref.reference - (wheel.position - ref.origin);  // [mm]
  if (bounds_.posErrMax > 0.0f) {
    if (error > bounds_.posErrMax) error = bounds_.posErrMax;
    if (error < -bounds_.posErrMax) error = -bounds_.posErrMax;
  }
  return error;
}
```

Call sites in `tick()`:

```cpp
const float pidLeft = (speedLeft == 0.0f)
    ? 0.0f
    : fastPid(positionError(speedLeft, state.wheelLeft, posRefLeft_, dt),
              errLeft, state.wheelLeft.cmdAccel);
```

`estop()` resets `posRefLeft_ = PositionRef{};` where it used to zero the
integrators.

`drive.h`:

```cpp
struct PositionRef {
  float reference = 0.0f;  // [mm] integral of commanded speed since anchor
  float origin = 0.0f;     // [mm] Wheel::position when anchored
  uint8_t epoch = 0;       // Wheel::positionEpoch when anchored
  bool armed = false;
};
mutable PositionRef posRefLeft_;
mutable PositionRef posRefRight_;

float posErrMax = 0.0f;    // [mm] added to ControlBounds; 0 = unclamped
void setPositionErrorMax(float posErrMax) {  // [mm]
  bounds_.posErrMax = (posErrMax > 0.0f) ? posErrMax : 0.0f;
}
```

Three guards earn their place: commanded zero passes through (stop is stop);
epoch change or disconnect re-anchors without correcting (a rebaseline steps
`position`, a disconnected wheel reports a manufactured zero); and the error
is clamped in **mm** so a blocked wheel cannot bank unbounded catch-up debt
and then sprint to repay it.

### E. `cmdAccel` on the WHEELS path — `app/drive.cpp`, `update()` — UNVALIDATED

```cpp
const float dt = static_cast<float>(state.time.cyclePeriod) * 1e-6f;  // [s]
if (dt > 0.0f) {
  const float rawLeft = (targetLeft_ - previousTargetLeft_) / dt;   // [mm/s^2]
  const float rawRight = (targetRight_ - previousTargetRight_) / dt;
  cmdAccelLeft_ += kAccelSmoothing * (rawLeft - cmdAccelLeft_);
  cmdAccelRight_ += kAccelSmoothing * (rawRight - cmdAccelRight_);
}
previousTargetLeft_ = targetLeft_;
previousTargetRight_ = targetRight_;
state.wheelLeft.cmdAccel = cmdAccelLeft_;
state.wheelRight.cmdAccel = cmdAccelRight_;
```

with `static constexpr float kAccelSmoothing = 0.35f;` and the four float
members. Smoothed because the host re-arms slower than `kCycle`, so the raw
command is a staircase and a bare finite difference alternates a
double-size spike with a zero. **Inert at `kff = 0`.** See §6.

### F. DBG verbs (ROBOT_DEBUG builds) — `comms.h`, `comms.cpp`, `robot_loop.cpp`

Four RAM-only tuning verbs, all following the existing `kMark`/`kWedge`
pattern — enum arm, parser arm in `classifyDbgArg`, apply arm in
`applyDbgAction`:

| verb | effect |
|---|---|
| `DBG:vmin <mm/s>` | `drive_.setSpeedFloor()` |
| `DBG:gain <L> <R>` | scales the baked `kDutyPerSpeed` per wheel |
| `DBG:asteady <mm/s^2>` | `drive_.setASteady()` |
| `DBG:pos <mm>` | `drive_.setPositionErrorMax()` |

Each echoes `"... applied"`, which is what the gate waits for before it will
report a run. `comms.h` gains `float value2` (and the enum arms).

### G. Bench tooling — `src/tests/bench/`

`velocity_profile_gate.py` (modified):
- `--vmin --pid-kp --pid-ki --pid-kff --pid-imax --pid-kaw --poserr
  --asteady --gain-left --gain-right`, asserted **after connect** — opening
  the port resets the board, so config set by any other process is already
  gone. This is the only place it survives.
- `--wheel {both,left,right}` for single-motor rigs.
- `--attempts N`: discards truncated captures and reconnects to clear a
  wedged board. A short capture reads as a short distance (77 mm of 450 mm
  once) and is indistinguishable from real under-travel.

New: `tail_forensics.py` + `plot_tail_forensics.py` (§2 evidence),
`plot_runaway_fix.py`, `plot_square_vs_trapezoid.py`,
`gain_balance_sweep.py` (drives the gate over a point list, interleaved
across repeats, reports per profile).

### 8. Reproducing

```bash
uv run python build.py --clean --robot-debug
uv run mbdeploy deploy --hex ./MICROBIT.hex <UID>

uv run python src/tests/bench/velocity_profile_gate.py --port <port> \
    --board vevov --vmin 20 --pid-kp 1.0 --pid-ki 12 --poserr 5 \
    --pid-imax 100 --pid-kaw 100 --pid-kff 0 \
    --tick 0.10 --lease 0.4 --attempts 8 --chart out.png
```

Two build traps, both of which cost real time here:

- **A plain `just build` compiles the DBG channel OUT.** The gate then aborts
  on an unconfirmed `DBG:vmin`. Always `--robot-debug`.
- **Adding a member to `Drive` in `drive.h` needs a CLEAN build.** An
  incremental one links stale objects against the old class layout; the
  encoders then read a manufactured zero that looks exactly like a dead bus.
  This cost an hour of hunting a hardware fault that did not exist.
