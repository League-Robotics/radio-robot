---
status: done
priority: high
---

# Unmanaged drive: leases expiring mid-leg, and a terminal pivot that hid a 27 deg heading error

Two defects in the TestGUI's host-side unmanaged distance drive
(`src/host/robot_radio/testgui/transport.py`), both measured on the bench
2026-07-31 and both introduced by earlier work in the same session.

## 1. The wheels halted mid-leg — lease expiry

Each `wheels` command is a **bounded lease**: `App::Drive::command()` sets
`commandDeadline_ = now + duration` (`src/firm/app/drive.cpp`). The host drive
loop used a 150 ms lease refreshed on a fixed 120 ms sleep — only 1.25x of
margin. Any cycle where the serial send plus frame handling overran 150 ms let
the deadman fire and **both wheels dropped to zero** until the next lease
landed. Observed as dropouts at t = 0.7, 2.8, 3.8 and 5.5 s of a 700 mm drive.

Fix: lease 300 ms, re-arm every 60 ms (5x margin). Measured over 419 leases:
mean gap 70 ms, one 312 ms scheduler hiccup. Keep the rule "refresh interval
<= lease/4" documented at the constant — a refresh interval close to the lease
is not merely tight, it visibly halts the robot.

## 2. The leg curved — a terminal pivot, from the per-wheel distance equalizer

To close an earlier 351 mm L/R split (open loop, L 879 / R 528 over 700 mm), the
loop was given a **per-wheel independent** ease-out: each wheel ran its own
distance profile. The wheel that got ahead therefore eased out *first*, and the
end of every leg became a pivot. From the bench serial log:

```
corr_id: 53  wheels { v_left: 150.0  v_right: 90.0 }
corr_id: 54  wheels { v_left: 125.0 }      <- v_right omitted = 0
corr_id: 57  wheels { v_left:  90.0 }      <- still pivoting
```

### The arithmetic, which matters because the encoders looked clean

- **Veer:** commands symmetric, but *actual* R ran ~130-140 mm/s while L ran
  ~115-125 for ~5.5 s. R outran L by ~66 mm. Over a 140.4 mm effective track
  that is 0.47 rad = **27 deg of left turn.**
- **Pivot:** left-only at ~100 mm/s for 4 leases ~= 0.6 s -> omega = 0.71 rad/s
  -> **~24 deg of right turn.**

They cancel in heading and add in displacement. Predicted: final heading ~0,
lateral offset ~700*sin(13 deg) ~= 160 mm, encoder counts equal. Observed:
**theta −0.6 deg, y +149 mm, enc L+714 R+713.** All three.

**The 1 mm encoder split was the symptom, not evidence of a straight line.** The
pivot erased the heading error from the encoder record while leaving it in the
physical path. This is the same signature `src/firm/app/DESIGN.md:152` documents
for the 119-005 crab bug — lateral offset, zero final heading error, invisible
to `encpose` — from a different cause. Worth treating as a recognised trap.

Fix: **common speed plus a bounded differential trim**, never independent
per-wheel profiles. Both wheels always driven; the correction is applied
continuously from the first frame instead of dumped at the end. Against a fake
with a worse mismatch than the real robot: 0 pivot commands, slowest wheel while
moving 70 mm/s, worst L/R ratio 1.57x.

With gain 2.0 mm/s per mm and a +/-40 mm/s limit, against the measured ~12 mm/s
wheel mismatch the loop settles at ~6 mm of standing split ~= 2.4 deg of
heading — roughly **29 mm of lateral offset over 700 mm instead of 149.**

## Note on the underlying cause

The ~12 mm/s L/R difference under identical commands is real and separate. No
host trim removes it; it needs per-wheel calibration — see
[[duty-per-speed-and-wheel-gain-disagree-with-the-plant]].

## Acceptance

- 700 mm unmanaged leg: zero frames where both wheels report < 5 mm/s mid-leg.
- Zero commands where one wheel is 0 while the other is nonzero.
- Camera-measured cross-track <= 30 mm, net heading <= 3 deg.

Related: [[estop-did-not-stop-write-on-change-vs-latching-brick]],
[[testgui-drive-loop-starved-telemetry-traces-and-graphs]]
