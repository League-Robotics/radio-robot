---
status: pending
filed: 2026-07-28
filed_by: team-lead (stakeholder directive)
tickets: []
---

# The square tour is THE system test — one test, three tiers (sim / bench / playfield)

## Stakeholder directive (2026-07-28)

> Make square tour the default system test. Let's remove every other system
> test... We might want to expand it a bit here and there to test more things,
> but generally, I think this should be our main system test. That means you
> can get rid of other system tests, and let's make a note not to create new
> ones. **I don't want to have a bazillion system tests that we have to work
> through every time.**
>
> We're going to expand this a bit to ensure that it has a sim, bench and
> playfield version. A full system test would involve all three of them and
> accepting that something is finally complete. It would have to require
> having the playfield test run with the camera and the odometer so you can
> validate independently that the robot's doing what it's supposed to.
>
> Additionally, we need to work a few stops into the test. The main square
> should have two planned stops on two opposite legs. And then, if it
> completes the full square and closes it within tolerance, then we can go on
> and do the initial or another curved plot, which will be like a circle
> rather.

## The shape

**One** system test, run at three tiers. "Complete" means all three passed —
not any one of them.

| Tier | Runs on | Truth source |
|---|---|---|
| **sim** | laptop, no hardware | the simulated plant |
| **bench** | robot on the stand, wheels off the ground | encoders + OTOS |
| **playfield** | robot driving the playfield | **camera + odometer, independently** |

The playfield tier is the one that actually proves anything: the camera and
the odometer validate the robot's real position independently of what the
robot believes about itself. Every other tier can only confirm the robot is
self-consistent.

## The test itself

1. **Square, with two planned stops on opposite legs.** The stops are part of
   the test, not an interruption of it — they exercise stop/restart and (per
   the pending OTOS-at-rest issue) are exactly where an at-rest optical-flow
   correction would be taken.
2. **Closure gate**: the square must close within tolerance.
3. **Only if closure passes**, continue to a **curved plot — a circle**. The
   curve is gated behind the square, so a robot that cannot drive straight and
   turn does not get scored on arcs.

## Delete the rest, and stop making new ones

Current `src/tests/sim/system/` holds a scattered collection:
`test_move_protocol.py`, `test_scripted_twist_demo.py`, `test_sim_api.py`,
`test_sim_boot_config_parity.py`, `test_sim_configure_from_robot.py`,
`test_state_estimator_tracking.py`, `test_straight_leg_crab_regression.py`,
`test_straight_twist.py`, plus their harnesses. The square tour subsumes what
these are really asking.

**Record the no-new-system-tests rule** somewhere it will actually be read —
`src/tests/CLAUDE.md` and/or `.claude/rules/` — so a future sprint does not
quietly grow the collection back. New coverage belongs either in a unit test
or as an expansion of the square tour, not as a new system test.

**One carve-out to weigh, not assume:** `test_sim_wire_loopback.py` is not a
motion scenario — it is the byte-level codec path built in sprint 124 as one
of two explicitly non-negotiable structural fixes, because `sim_ctypes`
bypasses the real codecs and both of sprint 123's wire bugs reached hardware
as a result. Deleting it re-opens that hole. It lives in `sim/system/` by
location but is really wire infrastructure. Recommend keeping it (possibly
relocated out of `system/`) and saying so explicitly rather than losing it to
a directory sweep.

## Existing material to build on

Already present, not starting from scratch:

- `src/tests/bench/square_tour.py`, `wheels_square_tour.py`,
  `planner_square_tour.py` — three overlapping square-tour drivers that should
  converge into one.
- `src/tests/playfield/plot_square.py`, `pose_fix_convergence.py`,
  `world_goto_chart.py` — the playfield tier exists but is **parked**; per
  `src/tests/CLAUDE.md` these need motion/odometry that only existed in
  pre-rebuild firmware. Reviving them is part of this work.
- AprilTag camera tooling is available via the aprilcam MCP server.

## Open questions

1. **What is "within tolerance"?** The closure gate needs a number per tier —
   sim, bench, and playfield will not share one. Note the standing
   ~41% turn over-rotation and the parked ±90° accuracy work; pick numbers
   that gate real regressions without being unreachable today.
2. **Is the bench tier meaningful for a square?** Wheels are off the ground on
   the stand, so the robot cannot translate. The bench tier presumably scores
   commanded-vs-encoder wheel travel rather than geometric closure — decide
   what it actually asserts.
3. **Where do the two stops go, exactly**, and what is asserted at each — dwell
   time, position hold, restart behavior, and (if the OTOS-at-rest change
   lands) the correction taken there.
4. **Circle parameters**: radius, direction, single or both, and its own
   closure tolerance.
5. **What runs in CI?** The sim tier is the only unattended one. Bench and
   playfield are HITL by definition (`src/tests/CLAUDE.md`: bench/playfield are
   CLI tools, not pytest-collected). Keep that split.
6. **What coverage genuinely dies** with the deleted tests? Walk each one
   before deleting and say what it asserted that the square tour does not —
   then either fold it in or accept the loss deliberately. Silent coverage
   loss is the risk here.

## Why

A single, real, three-tier test that has to pass end-to-end is worth more than
a broad shallow suite nobody reads. It also makes "done" mean something
specific: the playfield run, camera-verified, is the acceptance — and this
project has repeatedly shipped things that were green in sim and broken on
hardware (sprint 123's two wire bugs, sprint 125's boot race).

## Golden traces — stakeholder-verified baselines, compared programmatically

Second directive, same session (2026-07-28):

> I'd like there to be some recording of the traces. Maybe we've got some
> canonical form of the traces as an image or as a dataset. If I look at the
> traces and verify that that's what I want, then we can mark that as being
> the golden image, so to speak, and then future tests get tested against
> that.
>
> This should be the wheel speeds. You want to do each of those lines
> separately: each wheel speed, each commanded velocity, each measured
> velocity, the image of the trace of the X-Y trace, location of the robot.
> There may be some others, but you'll produce each of those as a simple
> image in a canonical form. Maybe what you'll do is draw the line on that
> image a bit wider, account for error variation, and then do an XOR or an
> AND with the trace you got from last run. You can just do a quick binary
> test, a quick programmatic operation, to check how much the last run varies
> from the golden image.

### The signals — one plot per line, never combined

- each **wheel speed** (left, right — separate images)
- each **commanded velocity** (per wheel — separate)
- each **measured velocity** (per wheel — separate)
- the **X-Y trace** (the path image)
- **robot location** over time

Deliberately one signal per image: a combined plot cannot be compared
pixel-wise, because a divergence in one line contaminates the whole frame and
you lose which signal actually regressed.

Likely additions worth considering: heading vs. time, per-wheel encoder
position, and `cycleBusy`/`cyclePeriod` (loop-timing health already rides the
wire and would catch a schedule regression the motion signals would not).

### The comparison

The stakeholder's proposal: render the golden trace with a **deliberately
widened stroke** — the width *is* the tolerance band — then do a binary
raster operation against the new run and count pixels.

Concretely, and this matters for which operator to use:

- `new AND NOT golden_band` → new-run pixels **outside** tolerance. **This is
  the failure metric.** Count them; a threshold gates pass/fail.
- `golden_band AND NOT new` → band the new run never visited. Not automatically
  a failure (a shorter or faster run legitimately covers less), so treat it as
  a separate coverage signal, not part of the same number.
- A plain `XOR` conflates those two, so it is the wrong single operator despite
  being the obvious one. Recommend reporting both terms separately.

### What makes this actually work (and how it silently breaks)

Pixel comparison is only meaningful if the two images are rendered
identically. Pin all of it, and treat any of these drifting as invalidating
every golden image at once:

- **fixed axis limits** — never autoscale. Two runs with different data ranges
  produce visually similar but pixel-incomparable images, and the test would
  pass or fail for reasons unrelated to the robot.
- fixed figure size, DPI, stroke width, and margins; no legend, title, ticks,
  or grid in the comparison layer (render those, if wanted, into a separate
  human-readable image).
- **1-bit rendering** for the comparison — antialiasing puts grey pixels at
  every edge and makes the count depend on the renderer version.
- pinned matplotlib (or whatever renderer) version, since a minor release can
  shift line rasterization by a pixel.

### Store the dataset too, not only the image

Keep the raw series (CSV/parquet) beside each golden image. The image is what
the stakeholder eyeballs and signs off; the dataset is what makes a failure
diagnosable — "12,431 pixels outside band" tells you something regressed but
not what, and it cannot be re-analyzed at a different tolerance without
re-running the robot. The dataset also lets a numeric check (RMS error,
max deviation) run alongside the raster check, which is a better gate for
signals like wheel speed where a small constant offset matters more than
pixel area.

### Process

- Goldens are **stakeholder-approved artifacts**, not auto-generated. A run
  produces candidate images; Eric looks at them; approval promotes them to
  golden. Nothing self-blesses.
- Goldens are **per-tier** — sim, bench, and playfield have genuinely different
  noise floors, so a single band cannot serve all three.
- Version goldens in-repo alongside the test, and require the commit that
  changes a golden to say **why** it changed. A silently re-blessed golden is
  how a real regression becomes the new baseline.
- The tolerance width per signal is itself a decision, and should start from
  measured run-to-run variation rather than a guess — take N runs with no code
  change and let the observed spread set the initial band.

Related: [`otos-sampled-only-at-rest-not-integrated-during-motion.md`](otos-sampled-only-at-rest-not-integrated-during-motion.md)
(the planned stops are where an at-rest OTOS fix would be taken).

**Extended by (2026-07-31):**
[`minimal-system-test-one-program-tour-files-one-jsonl-dataset-dbg-fault-injection.md`](system-test-minimal-system-test-one-program-tour-files-one-jsonl-dataset-dbg-fault-injection.md)
— the concrete mechanism for this charter: tour files (a text version of the
protocol), the one `systest.py` program, the per-run JSONL dataset every
analysis runs on, `DBG` fault-injection wire commands, in-tour `EXPECT`
assertions over the record stream, and `CAMFIX` camera position validation.

[`tests-outside-the-system-test-taxonomy-and-tiers.md`](system-test-tests-outside-the-system-test-taxonomy-and-tiers.md)
— the keep-list for everything that legitimately remains outside the system
test (wire robustness, protocol races, link gates, safety latency,
characterization tools), each classified sim/bench/playfield. Answers this
issue's open question 6 ("what coverage genuinely dies").
