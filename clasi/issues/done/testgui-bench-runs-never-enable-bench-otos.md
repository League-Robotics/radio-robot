---
status: done
tickets:
- NONE
---

# TestGUI hardware ("BENCH MODE") connections never enable the bench OTOS

## Problem

The TestGUI labels the Serial transport "BENCH MODE" but never sends
`DBG OTOS BENCH 1` (there is no `DBG` string anywhere in
`host/robot_radio/testgui/`). A tour run against the real robot on the
stand therefore fuses the **real** OTOS chip. On the stand the chip is
frequently **status-clean (`status=0x00`)** — the "warn bits are always
set on a stand" assumption does not hold in practice — so the sprint-074
fusion gate happily admits it. The chip reports "stationary at wherever
`OZ`/`SI` anchored it," and the EKF pins the fused pose to the origin
while the encoders report the commanded motion.

## Hardware evidence (2026-07-03, tovez on stand, fw 0.20260703.19)

Tour 1 run with the exact GUI wire sequence (`STOP; ZERO enc; OZ;
SI 0 0 0;` then `D`/`RT` steps with SNAP polling), real OTOS active:

- Step 1 `D 200 200 345` ends at pose `(220,-8,14.8°)`; by the end of the
  next `RT 9000` the fused pose is dragged back to exactly `(0,0,0.0°)`.
  Every subsequent step ends at `(0,0,~0°)`.
- Straight `D` legs show the BVC fighting the phantom OTOS heading:
  `D 700` ended with per-wheel encoders `(601, 978)`; `D 480` with
  `(245, 191)` — wildly asymmetric wheel travel on a straight drive.
- Moves still "complete" (D stops on raw encoder sum, RT on encoder
  differential), so the tour runs to the end, but the fused pose — what
  the GUI plots and what D-direction/TURN/heading-hold consume — is
  garbage. This is the observed "complete failure" of bench tours.

Log: scratchpad `out/rerun_console.txt` (Phase A) from the 2026-07-03
diagnosis session.

## Proposed fix

When the TestGUI connects over the Serial ("BENCH MODE") transport (and
after robot-change re-pushes), send `DBG OTOS BENCH 1` (optionally with
the Sim Errors panel's OTOS noise knobs, mirroring
`robot_radio/testkit/target.py:make_target("bench")`), and send
`DBG OTOS BENCH 0` on disconnect. Consider a visible "Bench OTOS"
indicator/toggle so playfield (production) runs can keep the real
sensor.

## Dependencies

- Requires firmware with sprint 074 (Drive reads the live `hal.otos()`;
  fusion-gate re-admission). Pre-074 firmware ignores the swap in the
  fusion path entirely.
- Related: [oz-si-do-not-reanchor-bench-otos.md] — without that fix,
  only the first tour after boot works.
