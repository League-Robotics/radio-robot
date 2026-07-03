---
status: done
sprint: '072'
tickets:
- 072-002
- 072-004
---

# SAFETY: DISTANCE stop uses fabsf(traveled) — backward runaway "completes" a forward drive

## Description

`StopCondition::evaluate`, `Kind::DISTANCE`
(`source/control/StopCondition.cpp:96-105`):

```cpp
float enc_avg = (s.encMm[1] + s.encMm[0]) * 0.5f;
float traveled = enc_avg - base.enc0Mm;
if (traveled < 0.0f) traveled = -traveled;  // fabsf
return traveled >= a;
```

The absolute value means a drive that runs away **backwards** satisfies the
stop condition once it is `target` mm *behind* its baseline. The same
`fabsf` pattern exists in the Planner's D decel hook (`d_traveled =
fabsf(enc_avg - _dEnc0)` in `Planner.cpp` `driveAdvance`), which makes the
decel cap symmetric in the wrong way too.

### Demonstrated consequence

In a forced-stall sim experiment against the real firmware code (encoders
pinned 2.5 mm short of a `D 200 200 500` target), the controller wound up,
flipped negative, committed to **−100 PWM full reverse, drove more than a
meter backwards, and emitted `EVT done D reason=dist`** when |traveled| hit
500 — i.e. 500 mm *behind* the start, reported as success. On the playfield
this failure mode is a robot backing off the table at full speed while the
host believes the move completed normally.

This is the unbounded end-state of the terminal-instability issue
(`d-drive-terminal-instability-reversal-thrash.md`); on hardware the chaotic
thrash happened to swing forward before reaching it, but nothing in the code
prevents the sim outcome from occurring on the real robot.

### Fix

Make the DISTANCE stop signed/direction-aware: completion requires traveling
`>= target` in the **commanded direction** (sign from the commanded wheel
speeds / body v at `beginDistance`). A reverse drive (`D -200 -200 500`)
should complete on backward travel — so keep the magnitude semantics but
gate on the commanded sign rather than blanket `fabsf`. Consider the same
treatment for the ROTATION stop's `fabsf(diff)` (wrong-direction spin) and
for the decel hook's `d_traveled`.

Also worth a wire-visible safety net: if signed traveled goes more than some
margin *negative* during a forward D (robot demonstrably moving the wrong
way), abort with `EVT safety_stop` rather than waiting for the TIME net.
