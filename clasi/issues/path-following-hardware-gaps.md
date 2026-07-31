---
status: pending
---

# Path following: curvature feed-forward, and the streaming demo that does not run on hardware

## Description

Sprint 127 built a real lookahead pure-pursuit path follower
(`solver.pursuitTarget()` / `planner.followPath()`, commit `8e38b2b6`, 64 unit
tests) and relaxed the firmware's at-speed hand-off so consecutive arcs of
differing curvature no longer land at rest (`c20cabc0`). Both work in sim. Two
gaps remain before this is usable on hardware.

## Gap 1 — tight corners are physically marginal, and no advance rule fixes it

Steering lag is 150 ms of dead time plus a ~230 ms plant tau, about **57 mm of
travel at cruise**. Pure pursuit needs a lookahead of roughly 2x that (~114 mm)
to damp. A 62.5 mm-radius fillet — what a 250 mm leg forces — is only **98 mm of
arc**, so a damping lookahead cuts ~26 mm off every corner. The two bounds nearly
touch.

Measured completion at 10% command loss: L=60 mm → 1/6, L=120 mm → 4/6,
L=150 mm → 4/6, L=180 mm → 2/6. There is a narrow optimum and it is not robust.

**The fix is curvature feed-forward**: command the path's own known curvature
directly and use pursuit only as a *correction*, which removes the lookahead
dependence entirely. The path generator already knows each segment's curvature —
`curve_stream.py` derives it from the heading delta over arc length — so the
information is available and simply is not being used.

Alternatives, both weaker: wider corners (changes the path, not the follower), or
slowing through fillets only (costs the speed this was built for).

## Gap 2 — the streaming demo does not run on hardware

`src/tests/bench/curve_stream.py` streams a 4-petal cloverleaf through the
planner queue. In sim: 70 of 71 interior hand-offs flow, mean per-boundary
minimum wheel speed 75 mm/s against a 300 mm/s cruise, the single stop being a
genuine wheel-direction reversal that `shapeDirectionsAgree()` correctly still
requires rest for.

On the robot (stand, direct USB, firmware `0.20260731.1` with the relaxed
hand-off flashed): **19 of 71 segments completed**, mean per-boundary minimum
11.1 mm/s, **16 of 18 boundaries landed fully at rest**, and the wheel-speed
trace shows two brief bursts of motion separated by **45 seconds of complete
stillness** (`src/tests/bench/curve_stream_hardware.png`).

That is not the "stops at every boundary" shape — it is the robot not executing
most of what it was sent. The streaming loop (`stream_segments()`) is
backend-agnostic and passes in sim, so the divergence is hardware-specific.

Candidates, none confirmed:
- Segments are being rejected and the script counts a boundary anyway — it warns
  on `ack.ok == False` but does not stop.
- `Move.id` collisions against the firmware's 16-slot dedup ring across a
  71-segment stream (a duplicate is silently re-acked OK).
- 60 mm segments at 300 mm/s are shorter than the ~69 mm the plant covers in one
  time constant, so the wheels are in permanent transient.

Needs an instrumented run logging every enqueue ack, `ERR_FULL`, and queue depth
— not a parameter sweep.

## Verification

- Curvature feed-forward: corner tracking that does not degrade as the fillet
  radius shrinks toward the lag distance, measured over the same L sweep.
- Streaming: a hardware run where per-boundary minimum wheel speed stays off zero
  across a continuous-curvature path, comparable to the sim figure.

## Related

- Sprint 127 ticket 008 — its exception block records the same two gaps.
- `clasi/issues/measure-actuation-floor-and-set-termination-tolerance.md` — the
  tolerance window's other wall.
- `docs/bench-reports/2026-07-30-square-tour-dead-time.md`.
