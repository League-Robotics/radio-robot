# The square-tour chart — canonical specification

**Status: normative.** This describes the chart produced by
`src/motion/planner/bench/square_tour_velocity.py`
(`square_tour_velocity_trim.png`). That chart is the reference rendering for
square-tour results. Every other tour chart in this project — the hardware
one in `src/tests/bench/planner_square_tour.py` included — conforms to this
spec or documents exactly where and why it cannot.

The point of writing it down: these charts have been re-invented ad hoc for
each run, so colours, panel order, and worst of all *what a dashed line
means* have drifted between them. A reader should be able to look at any
tour chart in this repo and know what they are seeing without re-reading the
plotting code.

## The prime rule

**A trace labelled "commanded" MUST be the planner's commanded value. Never
a measured signal, and never a measured signal transformed.**

This is first because it has already been violated. The hardware tour chart
plotted a dashed trace labelled "commanded" that was reconstructed from
`Telemetry.twist` — which `telemetry.proto` defines as *"body twist from
measured wheel velocities"*. It was the measured signal, drawn twice. It sat
exactly on top of the solid measured trace and was repeatedly misread as
evidence of perfect tracking, including by me.

If commanded data is not available, the chart must **omit the trace and say
so** — not substitute something that looks like it. See "Hardware
divergences" below.

## Figure geometry

| property | value |
|---|---|
| figure size | 15 × 11 in |
| dpi | 130 |
| save | `bbox_inches="tight"` |
| grid | `GridSpec(3, 2)`, `height_ratios=[1.25, 1.25, 1.0]` |
| spacing | `hspace=0.34`, `wspace=0.22` |
| backend | `Agg` (headless; never require a display) |

Panel placement:

```
+-----------------------------------------------+
|  1  Wheel speed (full width, grid[0, :])      |
+---------------------+-------------------------+
|  2  Path            |  3  Trim                |
|     grid[1, 0]      |     grid[1, 1]          |
+---------------------+-------------------------+
|  4  Tracking error (full width, grid[2, :])   |
+-----------------------------------------------+
```

Every panel: `grid(alpha=0.25)`, legend `fontsize=8`, title `fontsize=10`,
axis labels carrying units in brackets (`"wheel speed  [mm/s]"`).

## The colour system

Two rules govern every panel, and they are what make the chart readable at a
glance:

1. **Hue is the wheel.** Blue `#1f77b4` is ALWAYS left; red `#d62728` is
   ALWAYS right. This holds in all four panels. No panel may reuse these
   hues for anything else.
2. **Saturation is the signal's authority.** Full-strength hue = ground
   truth or command; washed-out hue = the noisy measurement.

| role | colour | style | lw | zorder |
|---|---|---|---|---|
| left true (plant) | `#1f77b4` | solid | 1.7 | 4 |
| right true (plant) | `#d62728` | solid | 1.7 | 4 |
| left commanded | `#1f77b4` | dashed | 1.1 | 3 |
| right commanded | `#d62728` | dashed | 1.1 | 3 |
| left measured (encoder) | `#9ecae9` | solid | 0.8 | 2 |
| right measured (encoder) | `#f2b8b5` | solid | 0.8 | 2 |

The z-order is deliberate: the quantised encoder trace is the busiest line
and must sit *behind* both the command and the truth, so the control story
reads over the noise rather than under it.

### Phase shading — the legs and turns

The accel/hold/decel bands are shaded behind everything (`zorder=0`,
`lw=0`), one `axvspan` per contiguous run of equal phase:

| phase | colour | reads as |
|---|---|---|
| `ACCEL` | `#e8f4ff` | pale blue |
| `HOLD` | `#eafaea` | pale green |
| `DECEL` | `#fff0e6` | pale orange |

`IDLE` is **not** shaded — unshaded means "no active move", which is
meaningful whitespace, not a gap in the rendering.

These three tints are the single highest-value feature of the chart. The
relative widths of the blue and orange bands are a direct visual readout of
`decelPlanFraction`: at the current 0.4 the orange bands are visibly ~2.5×
the blue ones, which is how the deceleration asymmetry was spotted at all.
Do not drop the shading to reduce clutter.

### Move boundaries

At each completion: a vertical `#999999` dotted line (`lw=0.7`,
`ls=":"`, `zorder=1`), annotated with the move's name rotated 90°,
`fontsize=7`, colour `#666666`, anchored to the top of the axes with a
`(2, -10)` point offset.

Naming is derived from move-id parity — odd = `leg`, even = `turn`,
numbered `(moveId + 1) // 2`, giving `leg 1, turn 1, leg 2, turn 2, …`.
**Note the id bases differ between harnesses** (sim numbers from 1, the
hardware tour from 9001), so each computes parity against its own base. The
rendered labels must come out identical either way.

Zero reference on every time-series panel: `axhline(0.0, "#cccccc",
lw=0.6)`.

## The four structures

The four panels are not four views of the same thing — they answer four
different questions, in causal order. Someone debugging a bad run should be
able to walk them top to bottom and localise the fault.

### Panel 1 — Wheel speed: commanded vs actual

**Question: did the planner ask for the right thing, and did the plant do
it?**

Full width, six traces (the table above) over the phase shading, with move
boundaries. Title states the shading legend inline:
`"Wheel speed: commanded vs actual   (shading: blue = accel, green = hold,
orange = decel)"` — the shading is explained in the title rather than the
legend so the legend stays a pure trace key.

This is the diagnostic panel. Profile shape, dead-zone tails, overshoot,
missing plateaus, and pivot asymmetry are all read here.

### Panel 2 — Path (ground truth)

**Question: where did the robot actually end up?**

- driven path: `#2b8a3e` green, `lw=1.8`, `zorder=3`
- ideal square: `#bbbbbb` dashed, `lw=1.0`, `zorder=2`, labelled
  `"ideal square"`
- start: `o` marker, `#2b8a3e`, `ms=7`, `zorder=4`
- finish: `X` marker, `#d62728`, `ms=10`, `zorder=5`
- closure callout: text `"closure {n:.1f} mm"` at the finish point,
  `(10, -14)` point offset, `#d62728`, `fontsize=9`
- **`set_aspect("equal", adjustable="datalim")`** — mandatory. A square
  drawn on unequal axes is worse than no chart; corner geometry and drift
  are the whole content of this panel.

Green is used here and nowhere else, precisely because this panel is about
the body rather than a wheel — it must not read as "left" or "right".

### Panel 3 — Closed-loop correction added to the profile

**Question: how hard did the feedback have to work?**

Left/right trim in the standard wheel hues at `lw=1.2`. Title switches to
**`"Trim DISABLED (open loop)"`** when trim is off — a silent empty panel
would be indistinguishable from a trim that happened to stay near zero, and
that distinction matters.

Read this against Panel 1: large trim excursions mean the profile and the
plant disagree, and the trim is papering over it.

### Panel 4 — Velocity tracking error

**Question: where exactly did the plant fail to follow?**

`commanded − actual` per wheel (`cmdLeft − velLeft`, `cmdRight − velRight`),
standard wheel hues, `lw=0.9`. Y-label is literally
`"commanded - actual  [mm/s]"` so the sign convention is unambiguous:
**positive = the plant is behind the command.**

This is Panel 1's residual made legible. Structured error concentrated at
phase boundaries (as opposed to broadband noise) indicates a profile the
plant cannot physically track.

## Title block

Two lines, `fontsize=12`:

```
Square tour, velocity plane -- {trim ON|trim OFF}, {plant}
closure {n} mm ({p}% of the {N} mm perimeter)   heading error {+d} deg
```

where `{plant}` is `"symmetric plant"` or
`"asymmetric plant (L {gainLeft} / R {gainRight} mm/s per duty)"`.

The rule the title encodes: **the configuration that produced the run is
part of the chart.** A chart whose plant asymmetry and trim state are not on
its face cannot be compared against another chart later — and this session
produced a dozen charts that needed exactly that comparison.

## Companion CSV

Every chart writes a sibling per-tick CSV with columns:

```
t, phase, profLeft, profRight, trimLeft, trimRight,
cmdLeft, cmdRight, measLeft, measRight, velLeft, velRight, x, y, heading
```

This is not optional. The chart is for seeing; the CSV is for measuring.
Phase-duration analysis, decel/accel ratios, and tail quantification all
come from the CSV — the chart should never be pixel-measured to recover a
number the CSV already holds.

Note `phase` is written as an integer enum: `0=idle, 1=accel, 2=hold,
3=decel`.

## Hardware divergences (`planner_square_tour.py`)

The hardware tour cannot currently meet this spec, and must be explicit
about it rather than approximating:

| spec element | hardware status |
|---|---|
| commanded traces | **ABSENT.** No commanded-velocity telemetry exists — `Command::v_x/omega` is unwired, and `cmd_vel` lived on the deleted `TelemetrySecondary`. |
| true (plant) velocity | **ABSENT.** There is no ground truth on hardware; the encoder measurement is all there is. |
| phase shading | **ABSENT.** `MovePhase` is not published in telemetry. |
| trim panel | **ABSENT.** Trim is not published. |
| tracking error panel | **ABSENT** — requires commanded. |
| path panel | present, conforms |
| move boundaries | present, conforms |

So the hardware chart is legitimately a two-panel reduction (speed + path,
plus a heading panel it adds because encoder heading is the only accuracy
signal it has). Its speed panel must label its traces as measured and must
NOT synthesise a "commanded" line — currently satisfied by labelling the
twist-derived trace `"twist-derived (MEASURED, not commanded)"`.

**Closing that gap is what would let the hardware chart converge on this
spec**: publishing per-wheel commanded velocity and `MovePhase` in telemetry
would enable the commanded traces, the phase shading, and the tracking-error
panel in one change. Until then, sim is the only place the full chart
exists — which is a reason to do profile work in sim, not a reason to fake
the traces on hardware.
