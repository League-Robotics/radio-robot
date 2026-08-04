---
status: pending
priority: high
---

# A single commanded stop does not reach the motor; `estop()` failed 5 of 6

**Measured 2026-08-03 on `vevov`. Safety defect — surfaced here for a future
sprint, NOT fixed on master.** The fix below was built and verified on a
throwaway branch (`position-pid` in the `radio-robot-elite-vmin` worktree)
and is deliberately not merged; take the patch, not the branch.

Patch: `clasi/issues/attachments/wheel-controller-2026-08-03/firmware-changes.patch` (hunks A and B — the file also carries
the unrelated controller work from [[B-wheel-controller-position-loop-and-tuning]]).
Chart: `clasi/issues/attachments/wheel-controller-2026-08-03/runaway_fix.png`.
Working copy at time of writing:
`/Volumes/Proj/proj/RobotProjects/radio-robot-elite/src/tests/bench/output/tuned_20260803/`
(that tree is bench output and may be cleaned — the attachments above are
the durable copies).

## Why this is filed as an issue rather than a fix

The change is a hack placed for demonstrability, not architectural fit: the
unowned-motion guard writes `cmdVelocity` from the loop, violating the
"one writer owns cmdVelocity" rule. The proper home is an explicit idle
owner. It also leaves the lowest-layer defence encoder-dependent. Both are
sprint-sized decisions, not a drive-by.

---

## Full write-up

### The stop never reached the motor

**Measured 2026-08-03 on `vevov` (Nezha brick + two BARE motors, no wheels,
no chassis). Reproduced 16/16 before the fix and 16/16 after.**

A commanded stop does not reliably stop the wheels. As shipped, if the host
issues a stop **once** and then goes quiet, the motors keep turning at the
last commanded speed **indefinitely** — measured 936 mm of continued travel
in a 6 s window at ~150 mm/s with no decay, still going when the capture
ended. `estop()` did not stop them either, failing 5 of 6 attempts.

Only *repetition* stops the wheels. A host that streams `wheels(0, 0)` at
10 Hz gets a stop; a host that sends one of anything does not.

This was found while investigating an unexplained "bounce" after the
commanded zero on a square velocity profile. The bounce was the visible
symptom; this is the cause.

Chart: `src/tests/bench/output/tuned_20260803/fixed/runaway_fix.png`.
Raw data: that directory's `confirm/`, `mechanism/`, `fixed/`.

---

### 1. What was measured

One 150 mm/s square, 3 s long, then a 6 s tail in which the host does
exactly one of five things. Everything else is identical.

Travel accumulated **after** the command reached zero, median across
repeats (range in brackets):

| tail treatment | as shipped | patched |
|---|---|---|
| host sends **nothing** | **936 mm** [927–937], never stops | **81 mm** [79–87], stops in 1.5 s |
| one `estop()` | **897 mm** [407–922], 5 of 6 never stop | **39 mm** [32–41], stops in 1.2 s |
| one `wheels(0,0)` | **~1079 mm**, never stops | **~35 mm**, stops in 1.2 s |
| `estop()` every 100 ms | 223 mm, stops in 1.65 s | — |
| `wheels(0,0)` every 100 ms | 81 mm [42–131] | 32 mm [31–40] |
| one `stop()` (planner) | ~1078 mm, never stops | — (expected: `stop()` is a *planned* stop) |

Two facts fall straight out:

1. **A single stop write does not land.** One `wheels(0,0)` and one
   `estop()` both leave the motors running. The same command repeated
   every tick stops them.
2. **Nothing re-asserts it.** With the host silent there is no second
   attempt from anywhere, so the runaway is permanent.

The motors are bare — no wheels, no chassis, essentially zero rotational
inertia. None of this is coast-down, and it does not decay.

### 2. Why it happens

The Nezha brick **physically latches its last commanded speed** and does not
reset on an nRF52 reset, only on power loss. So a lost zero write is not a
transient glitch — it is permanent, and the wheel keeps its old speed
forever. `nezha_motor.h` already documents this trap one layer down
("LOAD-BEARING (129-001, issue 07)"), and mitigates it inside
`writeRawDuty()` by re-issuing a commanded zero whenever the wheel is still
measurably moving.

**That mitigation reads the encoder.** So does `App::Drive::tick()`'s own
stop re-assertion window, whose `wheelsMoving` half is
`|velocity()| > kRestVelocity`. Both defences are disarmed by the same
condition: an encoder that reports at rest. A lost zero write plus an
encoder reading zero is a permanent runaway with no witness.

Above that, the ownership handoff leaves a gap that nothing covers:

```cpp
// App::Drive::update()
const bool owned = commandActive_;        // sampled BEFORE the expiry test
if (commandActive_ && expired) {
  commandActive_ = false;
  targetLeft_ = targetRight_ = 0.0f;      // publishes ONE zero pair
  ...
}
if (!owned) return;                       // ...and never publishes again
```

On the expiry cycle `owned` is still true, so one zero pair is published.
On every cycle after that `owned` is false and `update()` returns
immediately. `Motion::Planner::update()` runs unconditionally, but it
republishes only *its own* idea of the command. **Nothing states "no one is
driving, so the speed is zero"** on the cycles in between.

Every individual link looked correct in isolation. That is precisely why
this survived: each component was defensible on its own, and the defect
lived in what none of them was responsible for.

### 3. The fix

**This code is not being checked in** -- the two hunks below are the whole
change, and [[B-wheel-controller-position-loop-and-tuning]] §7 carries the rest of
the session's firmware changes in the same form. Full patch:
`clasi/issues/attachments/wheel-controller-2026-08-03/firmware-changes.patch`.

Two changes, both of which can only ever *remove* motion. Neither touches a
nonzero command.

**(a) Unowned-motion guard** — `App::RobotLoop::cycle()`, immediately ahead
of the single actuation path:

```cpp
if (!planner_.active() && !drive_.owns()) {
  state_.wheelLeft.cmdVelocity = 0.0f;
  state_.wheelRight.cmdVelocity = 0.0f;
}
```

If neither decider owns motion, the commanded speed is zero — asserted
every cycle, so the wheels cannot inherit a stale target from a decider
that has stopped publishing.

**(b) Arm the stop re-assertion on every stop, not just `estop()`** —
`App::Drive::tick()`:

```cpp
if (commandedStop && !alreadyQuiet) stopEnforceCountdown_ = kStopEnforceTicks;
```

The window existed but was armed only by `estop()`, and its other half
trusted the encoder. Arming it on the nonzero→zero transition of the *duty
pair* makes re-assertion depend on what was **commanded** rather than on
what the encoder claims happened — 30 cycles (~1.5 s) of repeated zero
writes after every stop.

Fix (a) addresses the silent/expired case; fix (b) addresses the lost
single write. They are independent, and both are needed: (a) alone would
still lose a stop whose one write was dropped, and (b) alone would still
let an expired command run on.

### 4. Result

16 of 16 runs stop, on every path, with no failures:

- **Silent host**: 936 mm → **81 mm**, and it stops rather than running on.
- **One `estop()`**: 897 mm → **39 mm**, 4 of 4 instead of 1 of 6.
- **Streaming zeros**: 81 mm → **32 mm**.

Run-to-run variance also collapsed: travel at command end went from
255–404 mm to 290–301 mm.

**Unexpected side effect worth noting.** Before the fix, roughly a third to
a half of all bench runs produced truncated or absent telemetry, which had
been costing entire measurement sessions and which I had been treating as a
separate, unexplained rig problem. After the fix, **16 of 16 runs captured
cleanly** — no truncation at all. A motor left running between runs appears
to have been the cause. This was not the bug being hunted, and the
connection is inferred from the run counts rather than proven, but the
before/after difference is stark.

### 5. What is still not right

- **A stop still takes ~1.2 s and ~35 mm** on bare, unloaded motors, which
  is far longer than the plant requires. That is the write shaping — slew
  cap and output deadband in `writeShapedDuty()` — not the stop path. It is
  now bounded and consistent, but it is not fast.
- **Both defences remain encoder-dependent at the lowest layer.**
  `writeRawDuty()`'s `stopNotTaken` exemption still gates on measured
  velocity. Fix (b) covers it from above for 30 cycles; a stop lost *after*
  that window, with a dead encoder, would still be permanent. The real
  answer is a stop the brick acknowledges, or a periodic zero heartbeat
  while at rest.
- **This is a hack, by agreement.** It is placed for demonstrability, not
  for architectural fit. The unowned-motion guard writes to the blackboard
  from the loop, which is a layering violation against the "one writer owns
  `cmdVelocity`" rule; the proper home is an explicit idle owner.
- **Only `vevov` was tested.** The mechanism is generic to the firmware and
  the Nezha brick, so other robots are very likely affected, but that is an
  inference — it has not been measured on `tovez` or any other machine.

### 6. Documentation that is now wrong

`NezhaProtocol.wheels()`'s docstring states:

> `duration` is REQUIRED and must be positive … a wheel command is always
> time-bounded, so a dead host can never mean a runaway.

Measured, as shipped: a dead host means exactly a runaway. The duration
expires correctly in `Drive`, but the expiry does not reach the motor.

`.claude/rules/playfield-testing.md` records `estop()` as measured at
2.9 cm / 0.10 s and instructs every geofence, Ctrl-C handler, and "halt now"
path to call it. On `vevov`, as shipped, a single `estop()` stopped nothing
in 5 of 6 attempts. Any halt path that calls it **once** should be treated
as unverified until re-measured on the machine in question.

### 7. Reproducing

```bash
uv run python src/tests/bench/tail_forensics.py --port <vevov> \
    --cases silent,estop,zero_once,zeros --repeats 4 --tail 6.0 --outdir <dir>
uv run python src/tests/bench/plot_runaway_fix.py <before_dir> <after_dir>
```

Note a measurement bug that was live for the first pass and is now fixed:
`t` is anchored on the baseline frame, captured during the leading settle
window, so the profile ends near `t = 4.0`, not `t = DURATION`. The original
table charged the profile's own last second to the tail and inflated every
tail figure by roughly 150 mm. The numbers in this document are computed
from the actual commanded-zero transition. The qualitative findings —
never-stops vs stops — are unaffected either way.
