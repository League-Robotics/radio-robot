# Square-tour verification in the main repo — findings, 2026-07-28

Session goal: verify the `radio-robot-elite-pidfree` work survived the merge to
master, and get the square tour running on tovez.

## 1. Merge verification — clean

All five worktree commits (`5065775a`, `3238bdaf`, `2f5f8b71`, `5c86c87a`,
`f6dbf598`) are ancestors of master. Verified present and byte-identical to the
worktree: `data/robots/tovez.json`, `drive.cpp`, `drive.h`, `main.cpp`.
Functionally confirmed: `Drive::correctedCommand`/`setWheelCorrection`,
`WHEELS`=13 / `ESTOP`=14 verbs, `kCmdRingDepth = 12` command ring,
`plannedStop`/`estop` planner routing, both bench scripts.

The flashed image is internally coherent — tovez's encoder ruler
(`mm_per_wheel_deg` 0.7165/0.7077, baked via `travel_calib_for_ports()`) paired
with tovez's own gains. The `0.487` `TRAVEL_CALIB_PLACEHOLDER` lands only on the
undriven ports 3/4. **Nothing was lost in the merge.**

## 2. Calibration was stale — recalibrated

The tour's calibration prelude read L 0.577 / R 0.739 instead of ~1.0.

Ruled out along the way: encoder-ruler mismatch (config is coherent);
battery (a fresh pack changed essentially nothing); measurement-window
artifact (`speed_map.py` samples at 0.42–0.58 s against τ≈0.23 s, which biases
fitted gains *low* — the wrong direction to explain this).

Also floated and **withdrawn**: a cold-drivetrain/thermal explanation. It
rested on two-point fits whose uncertainty (±47 on slope, ±24 on intercept,
propagated from ~8 mm/s scatter across a 0.171 duty lever arm) is larger than
the difference being explained. Not resolvable from that data.

Recharacterized with identity correction flashed, 599 trials over two batches,
526 after filtering (drop cmd > 450 and cmd = 0), classified per-trial by
each trial's own `prev`:

| wheel | direction | a | b [mm/s] | rms | n | implied plant G |
|---|---|---|---|---|---|---|
| left | accel | 1.4703 | +5.34 | 7.1 | 262 | 785 |
| left | decel | 1.4668 | +9.22 | 6.8 | 264 | 783 |
| right | accel | 1.4345 | −6.91 | 10.0 | 262 | 766 |
| right | decel | 1.4188 | +2.12 | 8.3 | 264 | 757 |

rms matches the original study's 6–9 mm/s floor, so the plant is no noisier
than before. The accel/decel structure is preserved (decel intercept above
accel on both wheels) — the physics is unchanged; only the gain moved.

Written to `tovez.json`, regenerated, flashed. Post-flash verification:

| commanded | 100 | 150 | 250 | 350 | 450 |
|---|---|---|---|---|---|
| achieved/commanded, L | 0.930 | 0.977 | 0.988 | 0.997 | 0.990 |
| achieved/commanded, R | 0.921 | 0.982 | 0.976 | 0.991 | 0.989 |

Unity from 150 mm/s up; the droop at 100 is the known dead zone.

## 3. Duplicate move enqueue — found and fixed

Both post-recalibration tours failed, differently, and both logged an enqueue
retry:

- **Run A** — `retry 1 for move 9005`. ~3 s runaway at turn 3 (left wheel
  +410 mm/s sustained, heading spiking 270°→1030° then decaying). Path 2505 mm
  vs 2000 target; the glitch's own mean speed × duration ≈ 465 mm accounts for
  nearly all of the +505 excess.
- **Run B** — `retry 1 for move 9006`. **Five** 90° turns instead of four:
  heading 450.3°, wheel differential dR−dL = 1006 mm against the five-turn
  theoretical 1005 mm. Straights were accurate (+0.5%).

Cause: `planner_square_tour.py` retries an unacked enqueue with a fresh
`corr_id` but the **same `Move.id`**, and `RobotLoop::handleMove()` had no
idempotency check. A lost *ack* (rather than a lost command) therefore executes
the move twice. The script's own comment names this hazard and accepts it as
"rare-by-construction" — it fired in 2 of 2 runs, on direct USB, not the relay.

Fix: `handleMove()` now suppresses a repeat non-zero `Move.id`, acking success
(the move genuinely is enqueued/running/done, and an error would make the host
abandon a move that actually ran). `Move.id == 0` is exempt — it is the
host-side default meaning "unset", and deduping it would drop every default-id
move after the first. The accepted-id window outlives completion, and
`ERR_FULL` rejections are not recorded. Issue:
`clasi/issues/duplicate-move-enqueue-on-ack-loss-retry.md`.

Result — three post-fix runs, all with exactly four turns (dR−dL ≈ 800–813
against the 804 mm four-turn theoretical):

| run | retries | path (target 2000) | heading | closure | duration |
|---|---|---|---|---|---|
| C | 4 (move 9005) | 2046 (+46) | 358.1° | **25.1 mm** | 110.6 s |
| D | none | 1834.5 (−165.5) | 363.9° | 233.6 mm | 52.7 s |
| E | 3 (9005/6/7) | 2008.5 (+8.5) | 361.2° | **4.1 mm** | 27.3 s |

Against the pre-fix 850.1 mm and 722.7 mm closures. The extra-turn failure mode
is gone, including on runs that took four retries.

## 4. Test status

- **Sim domain** (`src/tests/sim`, the no-hardware gate): **418 passed**,
  1 skipped, 1 xfailed. Green.
- **TestGUI domain**: 9 failed / 598 passed — an **identical** failure set with
  the dedup change stashed and rebuilt, so it is pre-existing, not a
  regression. These are also **flaky**: an earlier full-suite run failed 12,
  with different parametrizations (`[-180]/[270]/[360]` vs `[90]/[-90]`).
  They are sim turn-accuracy assertions (turns undershooting 7–10° against an
  8° tolerance), untouched per the standing "leave TestGUI alone" direction.

## 5. Open problems

Four things remain unexplained. See the chat discussion for candidate causes;
none of these are resolved.

1. **Plant gain apparently ~10% lower** than the stored calibration implied
   (L 876→784, R 833→762 mm/s per duty). Note the 876 figure is *not* a
   measurement — it is inferred from the stored gain on the assumption that
   identity correction was live during the original fit. That assumption is
   unverified.
2. **~27–30 s dead gaps** in the tour, wheels at exactly zero, making run
   duration vary 27 s → 110 s for identical work. Host-side, not a firmware
   move timeout.
3. **Leg-distance variance** across runs: path ranged −165 mm to +46 mm
   (−8% to +2%) with turns accurate throughout.
4. **`wheels_square_tour.py` metric bug**: reported path 591 mm while plotting
   a correct ~500 mm square, and heading −918.5° = exactly −229.6°/turn across
   four turns. The drive traces in that run were clean; the metrics are wrong.
