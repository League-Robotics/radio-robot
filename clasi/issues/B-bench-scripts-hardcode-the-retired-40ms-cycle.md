---
status: pending
priority: medium
---

# Three live bench scripts still hardcode the retired 40 ms cycle, with comments claiming it equals `kCycle`

## Description

Sprint-130 midpoint finding **#3 (MAJOR)**. It was recommended for fixing
*before* ticket 011 ran, because 011's own bench gate rides one of these
scripts. That did not happen — verified still present today:

```
src/tests/bench/square_tour.py:105            CYCLE_S = 0.04    # [s] one SimLoop.step() -- App::RobotLoop::kCycle
src/tests/bench/curve_stream.py:79            POLL_SLICE = 0.04 # [s] one SimLoop cycle (App::RobotLoop::kCycle)
src/tests/bench/turn_prediction_capture.py:94 _CYCLE_S = 0.04   # [s] ... matches firmware's
```

`kCycle` is 50 (and delivers 54 — see
[[A-nominal-50ms-vs-delivered-54ms]]). Every cycle-derived number in these
scripts is **25% off**, and each comment asserts the equality that makes it
wrong.

## Why it matters beyond arithmetic

`square_tour.py` is the vehicle for ticket 011's bench re-verification gate.
A gate that scores against 25%-wrong cycle math is not a gate. Any conclusion
drawn from those three scripts since the 40→50 cutover should be re-checked
before it is trusted — including timing figures quoted in
`square_tour.py`'s own header comments (`:229-255`).

## What to do

Do **not** just change `0.04` to `0.05`. That reproduces the defect one
generation later — which is precisely what happened here, and what the
post-mortem's root cause 1 is about. Take the period from a single source:

- ideally the config read-back ([[A-no-firmware-to-host-config-readback]]),
- otherwise the generated boot config the host already parses,

and delete the local literal in all three. Then grep for other copies —
these three were found by a review, not by a search, so there may be more.

## Verification

- `grep -rn "0\.04\b" src/tests/bench/ src/tests/system/` finds no
  cycle-period literal.
- The three scripts derive the period from one shared source and agree with the
  firmware.
- `square_tour.py`'s header timing figures are recomputed or removed.

## Related

- `docs/code_review/2026-08-02-sprint-130-midpoint.md` finding 3, and its
  "recommended sequencing" item 2.
- `docs/post-mortem/2026-08-02-sprint-130-wheels-solid.md` root cause 1 —
  "the period *was* 50 ms because a constant said so."
