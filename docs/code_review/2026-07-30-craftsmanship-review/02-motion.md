# Craftsmanship + Correctness Review — `src/motion` (motion library)

**Date:** 2026-07-30 · **Reviewer:** programmer-agent (review-only pass) ·
**Branch:** `sprint/127-host-side-path-planner-goto-and-path-following`
(WIP from a parallel session; `git diff` against the sprint's merge-base
confirms sprint 127 itself has not yet touched `src/motion` — everything
below is pre-existing tree state, not this sprint's WIP) · **Scope:**
every `.cpp`/`.h` under `src/motion` (~8.2k lines of C++ across the six
DESIGN.md-documented modules plus the entire `src/motion/planner/`
subtree, which DESIGN.md does not mention at all) plus the
`Motion::WheelSink` boundary from the motion side. Guidelines:
`docs/code_review/GUIDELINES.md`. Companion review:
`01-firm.md` (the firmware-base side of the same `WheelSink`/`Planner`
finding — read together, not independently).

## Verdict

**Rework, on the documentation and ownership questions; the live math is
in good shape.** `src/motion/DESIGN.md` ("last reviewed 2026-07-24")
describes a tree that no longer exists in production: it names
`Motion::MoveQueue` as "the one consumer that ties everything else
together," holds up `Motion::WheelSink` as the load-bearing boundary, and
never mentions `src/motion/planner/` — a second, larger, independently
built-and-tested motion-decision system (its own CMake project, its own
ctypes harness, 9 `ctest` executables, HIL bench scripts) that has
**already replaced** `MoveQueue` as the robot's actual motion decider
(`main.cpp:322-441`, `robot_loop.h:14-16`). Both systems are compiled
into the same firmware image today; only one is wired to the wheels. This
is the exact "if both lines of code are compiled together, one of them is
unwired" signature GUIDELINES §3 names, and it recurs at three different
layers of this tree simultaneously: the move-queue/shaping layer
(`MoveQueue`+`WheelSink`+`StopCondition`+`VelocityShaper`, dead),
the pose/estimation layer (`StateEstimator`, computed every cycle but
provably unconsumed, alongside `Odometry`, alive, alongside the
planner's own independent third implementation, alive and different
math), and the wheel-control layer (`WheelVelocityPid`, dead;
`WheelPid`'s own duty output live but explicitly discarded by the
composition root's own comment). None of this is exotic to find — the
codebase's own comments say so at every layer — which is itself the
finding: the tree has been carrying the receipts for its own accretion
for weeks without anyone walking back to update `DESIGN.md`, delete the
dead modules, or pick a single pose owner. The actual planner math
(`profile.cpp`'s discrete-exact braking accounting, `shape.cpp`'s ratio
lock) is dense but careful, extensively self-documented with measured
failure scenarios, and I found no arithmetic defect in it.

---

## Findings

### 1. MAJOR — accretion: `Motion::MoveQueue`/`WheelSink`/`StopCondition`/`VelocityShaper` are fully dead in production; `DESIGN.md` still calls `MoveQueue` the tree's central class

**Category:** accretion / interface-bleed

**Evidence:** `src/motion/DESIGN.md:120-131` ("`MoveQueue` is the one
consumer that ties everything else together... `Odometry` calls
`BodyKinematics::forward()`... `StateEstimator` is currently a peer");
`src/firm/main.cpp:29` still `#include`s `motion/move_queue.h` but never
constructs a `Motion::MoveQueue` anywhere in `main()` — the only motion
decider instantiated is `Motion::Planner` at `main.cpp:434`;
`src/firm/app/robot_loop.h:14-16`'s own routing-table comment: `MOVE ->
Motion::Planner (the motion queue)` — `MoveQueue` is not named at all;
`src/firm/app/drive.h:125-131`, verbatim: `"--- Motion::WheelSink (legacy
boundary) --- ... RobotLoop no longer routes anything through it -- the
live path is command()/tick()/update() above -- but the interface is
still implemented so a MoveQueue-era harness keeps compiling... nothing
in the live loop calls either method."` The companion firm-side review
(`01-firm.md`) independently found the same dead boundary from
`App::Drive`'s side; this is the motion-side half of one finding, not two.

**Concrete mechanism:** `move_queue.{h,cpp}` (~1000 lines, the land-at-zero
margin-factor derivation alone is a 250-line comment with three sprints of
sweep data) and `stop_condition.{h,cpp}`/`velocity_shaper.{h,cpp}` remain
fully compiled, fully unit-tested (`src/motion/CMakeLists.txt`'s
`motion_tests` target, plus `test_app_move_queue.py`), and fully described
in `DESIGN.md` as the architecture — but zero bytes of that machinery
execute on the robot. A reader who trusts `DESIGN.md` (which a reviewer,
a new contributor, or an agent following `CLAUDE.md`'s pointer to this
file is explicitly told to do) will build a completely wrong mental model
of how the robot actually decides motion.

**Design answer:** this needs a real go/no-go, not a doc patch — either
(a) `MoveQueue`/`WheelSink`/`StopCondition`/`VelocityShaper` are
confirmed retired and get deleted (their own tests included) with the
land-at-zero sweep history preserved in a dated design-history doc rather
than live code, or (b) if `WheelSink` is meant to survive as the eventual
duty-sink boundary (per `wheel_sink.h`'s own 125-002 comment, see finding
5), `DESIGN.md` needs to say which system is live TODAY and which is
scaffolding for a future cutover — not describe the dead one as current
truth. This is the stakeholder conversation GUIDELINES §7 asks for, not
a unilateral cleanup.

### 2. MAJOR — correctness/interface-bleed: `RobotState::pose` has two writers with contradictory contracts

**Category:** correctness

**Evidence:** `src/firm/types/robot_state.h:164-169`, the field's own
doc comment: `"--- Pose --- writer: Motion::Odometry::integrate()
(dead-reckoned world pose)... Encoder-only -- never OTOS-blended (that
blend lives one level up, in 'estimate' below)."` But
`src/motion/planner/planner.cpp:1247-1252` (`Planner::update()`) ALSO
writes `state.pose.x/y/heading/v_x/v_y/omega` — from `Planner`'s own
`pose_` member (`Motion::PoseTracker`, `planner/estimation.{h,cpp}`),
which blends OTOS heading directly (`planner.cpp:465-468`:
`pose_.blendHeading(state.otos.heading, limits_.headingOtosWeight)`)
whenever `limits_.headingOtosWeight > 0.0f`. That is precisely the case
the field's own doc comment declares cannot happen.

**Failing scenario:** within one `RobotLoop::cycle()`
(`src/firm/app/robot_loop.cpp:491-601`), `publishPose()` writes
`state_.pose` from `odom_` at line 534; `stateEstimator_.update(state_,
...)` at line 536 reads that odom-based pose as its `BodyPeer` basis
(correct, per its own contract); then `planner_.tick(state_)` /
`planner_.update(state_)` at lines 596-597 **overwrite** `state_.pose`
with `Planner`'s own, differently-integrated (and potentially
OTOS-blended) pose. Telemetry (`tlm_.update(state_)`, line 539) runs
BEFORE the planner ticks, so today's telemetry frame happens to carry
the odom-only value, and the planner's overwrite is itself clobbered by
`publishPose()` at the top of the next cycle before anything reads it —
the current behavior is a coincidence of block order, not an enforced
contract. `limits_.headingOtosWeight` is a live `PlannerLimits` field
that `main.cpp`'s own construction block does not set (stays at its
struct default `0.0f`), so the OTOS-blend divergence is dormant in
today's boot config — but nothing stops a future live-tune path (the
project already has exactly this precedent for `Motion::StateEstimator`'s
own `FusionWeights.headingOtos`, wired through `Configurator` at
`configurator.cpp:36`) from setting it nonzero, or a future refactor
moving `tlm_.emit()` later in the cycle, at which point telemetry starts
silently alternating between two disagreeing pose sources with no
config flag or log line marking the switch.

**Design answer:** `RobotState::pose` needs exactly one writer. If
`Planner`'s `PoseTracker` is now the canonical pose (which the "live
path" framing in finding 1 suggests it should be), `Odometry`'s write in
`publishPose()` should be deleted and the field's own doc comment
corrected to say so; if `Odometry` stays canonical, `Planner::update()`
should stop writing `state.pose` and instead publish its internal
pose through a distinctly-named field (or the existing `state.estimate`
section) so a reader can tell the two apart. Either way, "whichever
subsystem happens to run last in `cycle()` wins" cannot be the design.

### 3. MAJOR — accretion: pose/estimation logic is independently implemented three times in this tree

**Category:** accretion

**Evidence:** `Motion::Odometry` (`odometry.{h,cpp}`, midpoint-arc
dead-reckoning, encoder-only); `Motion::StateEstimator`
(`state_estimator.{h,cpp}`, ZOH predict-to-now + v1 complementary OTOS
blend, layered on `Odometry`'s output) — `DESIGN.md:127-131` and
`state_estimator.h:14` both say its `whereAmI()`/`wheelAt()` output has
"no consumer in this tree or the base," and `robot_loop.cpp:536` still
calls `stateEstimator_.update(state_, ...)` every single cycle,
computing a value nobody reads; `Motion::WheelChannel` +
`Motion::PoseTracker` (`planner/estimation.{h,cpp}`) — a THIRD,
independent implementation of the same two ideas (per-wheel ZOH velocity
filtering and a complementary OTOS heading blend), using different math
(`PoseTracker::integrate()` is a constant-curvature chord/arc integration,
not `Odometry`'s midpoint-arc approximation) and the one that actually
drives `Planner`'s decisions.

**Concrete mechanism:** this is GUIDELINES §1's own cited history
recurring inside the tree that history was presumably meant to prevent:
"the 2026-06-11 review found three independent go-to-point stacks and
three pose estimators with no owner." Here it is again, three modules
doing ZOH-extrapolation-plus-OTOS-blend, two of them (`Odometry` feeding
telemetry, `StateEstimator` computing and discarding) alive in the loop
for no functional reason, one (`PoseTracker`) actually driving the robot
and undocumented anywhere as the tree's real estimator.

**Design answer:** one estimator. If `PoseTracker`/`WheelChannel` is now
the design (it is the one solving the sprint's actual acceptance
criteria), `StateEstimator`'s `update()` call should stop running every
cycle for a value nobody reads, and `Odometry` should either be deleted
or explicitly repositioned as "the telemetry-only encoder trip odometer,
not the planning pose" — a real, statable design decision, not an
emergent side effect of the current call order (see finding 2).

### 4. MAJOR — accretion: three generations of the wheel velocity-control law coexist; the live one's duty output is explicitly discarded

**Category:** accretion

**Evidence:** `Motion::WheelVelocityPid` (`wheel_velocity_pid.{h,cpp}`,
relocated from `Devices::MotorVelocityPid` per 125-003's own header
comment, "App::Drive holds the interim instances this sprint") has
**zero instantiations anywhere in `src/firm`** — `grep -rn
"WheelVelocityPid" src/firm` returns nothing but two historical doc-comment
mentions (`device_config.h:14`, `nezha_motor.h:21`); `main.cpp:70-75`
confirms it directly: `"toMotionGains -- DELETED... It fed App::Drive's
interim Motion::WheelVelocityPid pair, which is gone: Drive is open-loop
duty from calibrated speed and holds no controller at all."` Meanwhile
`Motion::WheelPid` (`planner/wheel_pid.h`) is the live duty-plane
controller `Planner::stageDuty()` runs every tick — but `main.cpp:408-412`
says, in its own words: `"The loop's one actuation contract is a wheel
VELOCITY (RobotState::Wheel::cmdVelocity)... the duty-stage gains above
are computed every tick and DISCARDED."`

**Concrete mechanism:** `Planner::stageDuty()` (`planner.cpp:248-337`) runs
a full per-wheel PID (rest-clamp, actuation-lead compensation, filter-lag
compensation, stiction breakaway kick — real, carefully-derived control
logic, not a stub) on every one of the ~21 cycles/second the robot runs,
and the composition root's own comment says the result is thrown away.
`WheelVelocityPid` is dead weight with no call site at all. Only
`Motion::WheelTrim` (the velocity-domain correction) is both live and
actually reaches the wheels.

**Design answer:** delete `WheelVelocityPid` outright (no caller, no
transition plan referencing it) or state explicitly why it is being kept
warm. Decide whether the duty-plane stage (`WheelPid`/`stageDuty()`/
`dutyLeft_`/`dutyRight_`/`commandedDutyLeft()`) is provisional
infrastructure for a future duty-sink cutover (in which case say so in
`DESIGN.md` and mark it explicitly experimental/inert) or should stop
running until that cutover actually lands — right now it is real,
tested, tuned control-law code (`wheel_pid_test.cpp`,
`planner_duty_scenarios_test.cpp`) computing a value the loop's own
comment says is discarded, which is exactly the "half-retired code is
worse than either state" signature from GUIDELINES §3.

### 5. MAJOR — doc drift: `DESIGN.md` doesn't mention `src/motion/planner/` at all

**Category:** accretion

**Evidence:** `src/motion/DESIGN.md`'s file table (§2, lines 52-61) lists
exactly six `.cpp`/`.h` pairs plus `wheel_sink.h` and one `CMakeLists.txt`
— the "motion_tests" build description (§4) enumerates exactly three
`ctest` scenarios (StopCondition, VelocityShaper, the chained-Move
end-to-end). None of that mentions `src/motion/planner/`, which has its
own, entirely separate CMake project (`src/motion/planner/CMakeLists.txt`,
`project(motion_planner CXX)`) building a `planner` static library from
six `.cpp` files, a `motionplanner` shared library (the ctypes surface for
`py/planner_harness.py`), and 9 `ctest`-registered executables
(`profile_test`, `shape_test`, `estimation_test`,
`planner_scenarios_test`, `planner_noise_test`, `ratio_lock_test`,
`wheel_trim_test`, `wheel_pid_test`, `planner_duty_scenarios_test`) —
this is the larger of the two CMake projects under `src/motion` by every
measure (13 source files vs. 6, 9 test executables vs. 3), and it is the
one that is actually wired to the robot (finding 1). `wheel_velocity_pid.
{h,cpp}` (finding 4) is likewise absent from `DESIGN.md`'s file table
despite being folded into the SAME `motion` static library
(`src/motion/CMakeLists.txt:76`). `CLAUDE.md`'s own architecture section
compounds this: it still lists "the velocity PID" as part of the frozen
firmware base (`src/firm`), which was true before sprint 125 relocated it
into `src/motion` (per `wheel_velocity_pid.h`'s own header) — itself now
superseded again (finding 4).

**Design answer:** this is squarely GUIDELINES §3's "fixes that don't
update the design docs" — `DESIGN.md` needs a `planner/` row (or a
dedicated `src/motion/planner/DESIGN.md`, mirroring the pattern
`src/firm/motion/DESIGN.md` already uses for pre-122 history) covering
the second CMake project, the ctypes/Python harness, and the bench
script catalog; `CLAUDE.md`'s architecture paragraph needs its "the
velocity PID" claim corrected or removed.

### 6. MINOR — placement: bench/diagnostic Python + measurement data artifacts live inside the product tree, not `src/tests/`

**Category:** placement

**Evidence:** `src/motion/planner/bench/` (`hil_drive.py`,
`hil_square_tour.py`, `plant_id.py`, `square_tour_sim.py`,
`square_tour_velocity.py`, `encoder_refresh.py` — 2,229 lines of Python
total, all `git ls-files`-tracked) and `src/motion/planner/py/
planner_harness.py` (the ctypes test harness) sit directly under
`src/motion/planner/`, the C++ library tree, rather than under
`src/tests/bench/`. Several multi-megabyte measurement artifacts are
committed alongside the scripts: `square_tour_velocity_open.csv` (2.4 MB),
`square_tour_velocity_trim.csv` (2.4 MB),
`square_tour_velocity_open_sym.csv` (1.9 MB),
`square_tour_velocity_trim_sym.csv` (2.1 MB), plus four PNG plots
(confirmed via `git ls-files -s`, not gitignored build output like
`planner/build/` correctly is).

**Design answer:** per this project's own stated convention ("Test code
belongs in src/tests/ — never in src/host/robot_radio/", and GUIDELINES
§2's identical rule for sim/bench/playfield domains), `bench/` and `py/`
belong under `src/tests/bench/` (alongside the project's other HIL/bench
scripts, e.g. `src/tests/bench/move_protocol_bench.py`), not inside the
library directory whose own `CMakeLists.txt` a reader would expect to
define only the shipping `planner`/`motionplanner` libraries. The
committed CSV/PNG captures are measurement output, not source — they
belong in a bench-results location outside the C++ source tree (or
regenerated on demand and left untracked) rather than inflating `git
clone`/`git blame` on the library directory indefinitely.

### 7. NOTE — style: `body_kinematics.{h,cpp}` predates the CamelCase/no-embedded-units naming convention

**Category:** style

**Evidence:** `body_kinematics.h:12-89` uses Doxygen `/** ... */` blocks
(not the project's `//` file-header convention used everywhere else in
this tree) and Hungarian-style output-parameter names (`vL_out`, `vR_out`,
`v_out`, `omega_out`) rather than plain quantity names
(`.claude/rules/naming-and-style.md` rule 3 requires lowerCamelCase
variables/parameters; `vL_out` is neither a math subscript like `v_x`
nor a trailing-underscore member — it's a distinct "_out" suffix
convention this file invented for itself).

**Design answer:** low priority — `DESIGN.md` and the file's own header
both describe this as "moved verbatim... zero behavior change," and
`.claude/rules/coding-standards.md`'s legacy-code carve-out applies in
spirit even though the letter of the carve-out is written for
"already lowerCamelCase" code. Bring into conformance the next time this
file is touched for any other reason, per that same rule's general
instruction, rather than as a standalone pass.

### 8. NOTE — correctness: unwrapped float32 heading accumulators have no documented precision bound

**Category:** correctness

**Evidence:** `Motion::Odometry::theta_` (`odometry.h:99`, `theta_ +=
headingDelta` in `odometry.cpp:34`) and `Motion::PoseTracker::heading_`
(`estimation.cpp:55`, `heading_ += dTheta`) are both deliberately
unwrapped, monotonically-accumulating `float` radians — correct and
necessary for multi-rotation cumulative tracking (`move_queue.h`'s own
"TOUR_1 net heading 540±1deg" commentary depends on exactly this), and
`PoseTracker::blendHeading()`'s `std::remainder`-based residual
(`estimation.cpp:63`) is a genuinely wrap-correct comparison against this
unwrapped accumulator (see "What's good" below). Neither `odometry.h`
nor `estimation.h` documents a precision bound on how large the
accumulator may grow before a single-precision `float`'s ~7 significant
decimal digits can no longer represent a per-cycle increment (`dTheta`
on the order of 1e-3–1e-2 rad) against an accumulated base — e.g. at a
base of ~10,000 rad, float32 epsilon is already comparable to a typical
per-cycle increment.

**Concrete mechanism:** not observed as an active failure (mission
durations in this project's own bench/playfield docs are short), and
this is explicitly a NOTE, not a confirmed defect — but no comment in
either file states what "very long session" means for this accumulator
or whether a periodic re-wrap (subtracting a multiple of 2π when safely
between moves) is planned. Worth a one-line acknowledgment in
`odometry.h`/`estimation.h` given how deliberately every other numeric
edge case in this tree is called out.

---

## What's good

- **`profile.cpp`'s discrete-exact braking accounting** (`brakeDistance()`,
  `maxEntryVelocity()`, `profileStep()`) is genuinely careful work: the
  feasibility test is the exact discrete staircase sum rather than the
  continuous `v²/2a` approximation the pre-planner `MoveQueue` land-at-zero
  logic had to fudge with three empirically-swept margin constants
  (`move_queue.cpp:47-327` — itself a good illustration of why the
  discrete-exact approach in `profile.cpp` is a real improvement, not
  just a rewrite). Every non-obvious branch (the ratio-lock tie-break,
  the ZOH anticipation split into "already elapsed" vs. "not yet elapsed"
  spans) carries a comment naming the specific measured failure it fixes.
- **`shape.cpp`'s ratio lock** (`Planner::planWheels()`,
  `planner.cpp:1058-1150`) is a clean, well-justified answer to a real
  problem (per-wheel slip corrupting a commanded turn radius) with a
  correctness argument (the homogeneous-degree-1 property making the tie
  exact on a tracking plant) spelled out in the comment rather than
  asserted.
- **`Planner::move()`'s `ERR_FULL` path** (`planner.cpp:362-393`) is
  exactly the explicit, caller-visible bounded-resource contract
  GUIDELINES §5 asks for: the full/empty check runs before any mutation,
  so a rejected `move()` call provably leaves the queue unchanged —
  no partial-state hazard.
- **`estop()`/`plannedStop()` semantics** (`planner.h:42-59`) correctly
  preserve the project-wide "estop is immediate and total, a planned stop
  is a sequenced queue entry" distinction (`.claude/rules/
playfield-testing.md`'s "Halting" section) — `estop()` clears both active
  and pending state in one call with no partial-completion acks.
- **`PoseTracker::blendHeading()`'s wrap handling**
  (`estimation.cpp:59-65`) is a correct, minimal wrap-safe comparison
  (`std::remainder` against an unwrapped accumulator) — the textbook
  version of the "every modular quantity needs a wrap-correct comparison"
  rule GUIDELINES §5 asks for, and a good contrast with the historical
  ~272° attractor bug that rule references.
- **`StopCondition`/`VelocityShaper`** (dead in production per finding 1,
  but reviewed on their own terms) are still clean, single-purpose,
  zero-collaborator classes exactly matching their own stated contracts —
  the code itself is not the problem; where it sits in the tree's
  current architecture is.
