---
status: pending
---

# Bench finding — right encoder wedges / under-counts during drives, corrupting odometry

## Context

Found during sprint 032 hardware bench validation (firmware v0.20260612.17, robot `tovez`, on the stand).
During straight `D` drives the RIGHT encoder repeatedly wedged or severely under-counted:

- `EVT enc_wedged wheel=R enc=0 n=10` fired mid-`D`.
- Saw `enc=22,0` (R stuck at 0 while L advanced) and, over one `D 600`, `enc=736,57` — left wheel
  counted 736 mm, right only 57 mm.

Because firmware odometry integrates the L/R differential, the dead right encoder produced a **phantom
heading swing** (fused pose ran to ~131 deg with large Y drift on what should have been a straight drive).
The corruption is driven by the HARDWARE encoder, not a software bug — the velocity ramps themselves were
clean and `ekf_rej` stayed 0.

This is the long-known nRF52 encoder-wedge / L-R-imbalance issue (see memory
`encoder-wedge-...`, `D command drive findings`), re-confirmed on the bench against the current firmware.
The IRQGUARD/begin-placement fixes did not fully eliminate it under sustained drive load.

## To investigate

- Whether the wedge is the TWIM/I2C errata recurring under the current IRQ load, a specific right-channel
  hardware fault, or a battery-droop L/R imbalance (memory notes imbalance grows with battery drain).
- Whether `enc_selftest` / `enc_watch` reproduce it standalone (run those FIRST per memory before blaming
  the encoder — but here the EVT enc_wedged + the 736-vs-57 split are strong direct evidence).
- Whether odometry should reject / hold pose when one encoder is flagged wedged (defensive: don't
  integrate a known-dead wheel into heading).

## Acceptance

- Right encoder counts symmetrically with left on a straight drive (within tolerance), or odometry is
  made robust to a single wedged encoder (no phantom heading swing). Reproduce + characterize with
  `enc_selftest` before/after any fix.
