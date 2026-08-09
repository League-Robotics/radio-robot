# Motion planner (host ctypes library) bench measurements

Hardware characterization the host-only planner cannot derive for itself.
This doc, and the scripts it covers (`encoder_refresh.py`, `hil_drive.py`,
`hil_square_tour.py`, `plant_id.py`, `square_tour_sim.py`,
`square_tour_velocity.py`, `planner_harness.py`), relocated here (128-010)
from `src/firm/motion/planner/bench/` and `src/firm/motion/planner/py/` — bench/
tooling artifacts inside the C++ library tree, joining the rest of the
bench-tool catalog. `square_tour_sim.py`'s own committed capture is named
`motionlib_square_tour_sim.png` (not `square_tour_sim.png`) to avoid
colliding with the unrelated, pre-existing `square_tour_sim.png` this
directory already had (127-007's `square_tour.py` goto-mode sim capture) —
two entirely different scripts that happened to want the same output name
once co-located.

## `encoder_refresh.py` — encoder refresh interval (issue §7 item 3)

**Status: NOT YET RUN — no hardware attached.** As of 2026-07-25 this
checkout sees no CMSIS-DAP probe (`pyocd list` → "No available debug
probes are connected") and no robot serial device (`/dev/cu.usbmodem*`
does not exist). The capture half of the script is therefore unverified
against a real robot; the analysis half is exercised and correct (it was
validated against a synthetic capture, and it correctly reports a flat
p50 across speeds as the fixed-timer signature).

Run it when the robot is back on the stand:

```bash
uv run python src/tests/bench/encoder_refresh.py \
    --port /dev/cu.usbmodem2121102
```

It drives 60 / 150 / 300 mm/s for 60 s each (wheels off the ground — see
`.claude/rules/hardware-bench-testing.md`), writes `out/encoder_refresh.csv`
and `out/encoder_refresh.md`, and prints the per-wheel, per-speed histogram.

An existing `tlm_log.py` capture can be analyzed instead of taking a new
one — the loader accepts either CSV shape:

```bash
uv run python src/tests/bench/encoder_refresh.py \
    --analyze src/tests/bench/out/tlm_log.csv
```

### Why it (used to) block things — ANSWERED 2026-07-26

The underlying question is settled: a dedicated bench firmware
(`src/tests/firmware/encoder_rate/`) measured the register LIVE at
≤ 16 ms — the "~80 ms refresh" folklore is false; historical staleness
was interposed-traffic sample invalidation under the pre-118 loop
schedule. Authoritative write-up:
`docs/design/encoder-refresh-characterization.md`. This script remains
useful as a THROUGH-THE-LOOP confirmation (expect fresh samples every
50 ms cycle on the current schedule).

Two planner numbers were originally set from that folklore rather than
measurement:

* `PlannerLimits.velocityFilterWeight` — the EMA weight on fresh encoder
  velocity. Too high and the noise passes through; too low and the filter
  lags the motion.
* the noise tier's `NoisyPlant::sampleDivisor` (currently 2, i.e. a fresh
  sample every 100 ms) — the staleness the planner is *tested* against.

The measured distribution sets both. Its **p99**, not its p50, is what
`PlannerLimits.settleWindow` has to tolerate, since a Move confirms its
arrival at low speed, which is exactly where a per-count encoder refreshes
slowest.
