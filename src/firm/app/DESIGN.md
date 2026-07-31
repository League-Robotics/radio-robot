---
root: ../../../docs/design/design.md
---

# App — Loop and Passive App Modules

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-25 · **Status:** in-flux

---

## 1. Purpose

`app/` is the single cooperatively-timed control loop (`App::RobotLoop`) and
the BASE-side passive modules it owns:

* `Comms` (wire framing — protocol v5's uniform packet grammar,
  `<COMMAND>[':' <data>]'\n'`, dispatched by generated command-registry
  lookup, 124-005; COBS+CRC binary frames demuxed from the HELLO/PING
  text rump by a transport-level heuristic, 123-002, before that; base64
  line-armor before that, see §4 below),
* `Telemetry` (outbound frames, including the `cycle_busy`/`cycle_period`
  loop-timing fields — now on the PRIMARY frame every cycle, 123-004;
  landed on the secondary frame as an interim placement at 122-003, see §4
  below),
* `Drive` (122-002, NARROWED — the wheel-target sink only:
  `setDuty()`/`stop()`/`tick()` (125-002 renamed `setWheels()` ->
  `setDuty()`, `Motion::WheelSink`'s own velocity-sink -> duty-sink retool
  — a placeholder, unclamped pass-through until ticket 007's real duty
  implementation lands; see `src/motion/wheel_sink.h`/`drive.h`'s own
  doc comments), implementing `Motion::WheelSink`
  (`src/motion/wheel_sink.h`); it lost `setTwist()`/its `BodyKinematics`
  dependency to `Motion::MoveQueue`, see below), and
* `Preamble` (boot-time device detection).

**`RobotLoop` also constructs and drives three MOTION-LIBRARY objects
through the velocity-sink boundary** (sprint 122's two-layer base/motion
split — `docs/design/design.md` §2/§5): `Motion::MoveQueue` (the
1-active + 4-pending bounded-motion queue — every `Move` is self-bounding
by construction, so there is no separate staleness gate), `Motion::
Odometry` (dead reckoning, plus cumulative path length), and `Motion::
StateEstimator` (117 — predict-to-now wheel/body peer estimates,
zero-order-hold extrapolation, v1 complementary blend against OTOS).
These three live in `src/motion/` (a sibling tree, not a child of
`app/`) as of sprint 122 ticket 002 — `RobotLoop` holds them by reference
exactly like its own base-side modules (constructed at the composition
root, `main.cpp`/`SimHarness`) and calls into them at specific points in
its own schedule, but they are motion-library types, not `app/`'s own.
See this file's own "122 (motion-library extraction)" note at the end of
this section for the full before/after, and
[`src/motion/DESIGN.md`](../../motion/DESIGN.md) for their current
orientation, boundary contract, and standalone `motion_tests` build.

This is the seam that owns the robot's *timing* — every I2C
transaction, every wait, every cadence decision lives here or is called from
here in visible order. It exists as its own subsystem because timing
discipline on a shared single-master I2C bus, and cooperative yielding to the
CODAL fiber scheduler, are the two hard realtime problems this firmware has;
drawing the boundary around "the thing that owns the schedule" keeps that
problem in one place instead of smeared across device leaves.

**115-005 (gut S1) deletion note.** `Pilot` (109-003/109-005 — bridged
`Motion::Executor` into the loop's cycle and computed the heading PD
cascade on top of it) and `HeadingSource` (109-005 — decided which sensor
was truth for heading) are DELETED wholesale, along with `Motion::Executor`/
`Motion::JerkTrajectory`/`vendor/ruckig` (`motion/DESIGN.md` and its own
Pilot/Executor/HeadingSource sections go with them — see that directory's
own history if it still exists, or the git tag below if it does not). This
sprint's own tag `pre-gut-motion-stack` preserves the full pre-deletion tree
(files, DESIGN.md prose, tests) for recovery — deleting the DOCUMENTATION of
a deleted subsystem here is not a loss of information, only a redirect to
where the real historical record lives. `RobotLoop` lost the `Pilot&`
constructor parameter, the MOVE dispatch case, and every `pilot_.*` call
site; `Drive` lost its `configure(msg::PlannerConfig)`/acceleration-
feedforward staging (112-002) entirely, since `msg::PlannerConfig` itself no
longer exists. The command surface through this sprint (S1) is TWIST+STOP+
CONFIG{motor,otos}+deadman only — S2 (sprint 116) replaces TWIST+deadman
with the bounded MOVE protocol.

**116-005/116-006 (S2, MOVE protocol cutover) — landed.** `App::Deadman`
(`app/deadman.{h,cpp}`, both test harnesses) is deleted in turn — the
same wholesale-deletion treatment 115-005 gave `Pilot`/`HeadingSource`
above. `RobotLoop::handleTwist()` is replaced by `handleMove()`; a new,
small `App::MoveQueue` (`app/move_queue.{h,cpp}`) owns the 1-active +
4-pending queue and drives one `Motion::StopCondition`
(`motion/stop_condition.{h,cpp}` — a fresh, much smaller `motion/`
directory than the one S1 deleted, and NOT a revival of `Pilot`/
`Motion::Executor`) per active `Move`. The command surface is now
MOVE+STOP+CONFIG{motor,otos} — no deadman.

**117 (predict-to-now estimator v1) — landed.** A new passive module,
`App::StateEstimator` (`app/state_estimator.{h,cpp}`), is added
alongside `Odometry` — NOT a replacement for it: `Odometry`'s dead-
reckoned `x_`/`y_`/`theta_` still feed `frame_.pose` exactly as before,
and `StateEstimator` reads that same per-cycle `Frame` data as an
independent, additive consumer, greenfield in the same sense
`StateEstimator`'s own source issue used for the deleted `Pilot`/
`HeadingSource` era: it does not yet drive motion (no consumer wires its
output into `Drive`/`MoveQueue` this sprint — that is the later
trajectory-controller sprint, gated on this one being bench-proven).
`StateEstimator` holds per-wheel and body state as PEER estimates (each
independently valid/stale), computes zero-order-hold "predict to now"
extrapolation from the newest basis reading — generalizing the deleted
`HeadingSource::headingLead()` equation (`heading = basis.heading +
basis.omega × age`) to the full body pose (x, y, heading, v_x, v_y,
omega) — and blends a v1 complementary weight against OTOS heading/omega
whose weights are fail-closed baked config (`Config::
defaultEstimatorConfig()`), defaulting to 0.0 (encoder-only output this
sprint, per stakeholder decision) and live-tunable via a new
`ConfigDelta.estimator` (`EstimatorConfigPatch`) arm dispatched by
`RobotLoop::handleConfig()`, mirroring `OtosConfigPatch`'s existing
merge-then-apply pattern — see §3/§4 below for the full detail. Pure
computation: never touches the I2C bus, never sleeps, no `Devices::
Clock&` collaborator of its own (every query takes an explicit `now`/`t`
argument, mirroring `Motion::StopCondition`'s "hand-fed readings, no
owned collaborator" shape — see that module's own file-header precedent).

**118 (loop schedule truth) — landed.** Restores `cycle()`'s schedule to
what this file already claimed it was: `kSettle`/`kClear` back to their
genuine 4ms vendor-settle/clearance budget (regressed to 0 by commit
`5f5a2ba7`, which had been satisfying the vendor's mandatory settle as a
*blocking* sleep hidden inside `motorL_.tick()`/`motorR_.tick()` instead —
tripping the I2C clearance safety-net fault bit every cycle), `kCycle`
40ms/~25Hz (was a fictional 20ms/~50Hz under the regression), and
`Telemetry::kPrimaryPeriod` coupled back to `kCycle` (40). Two call-order
changes ride along: `drive_.tick()` moves back inside the R-settle block
(retiring the 112-005 "hoist `drive_.tick()` above both motor ticks"
experiment, which had been tracked only in project memory, not in an
issue — the interleaved order restored here is the one this file's §2/§4
already described); and `moveQueue_.tick()` — the stop decision — moves
from the R-settle block into the trailing pace block, evaluated AFTER
`applyOtosSample()`/`odom_.integrate()`/`stateEstimator_.update()` rather
than before them, so a MOVE's completion decision reads odometry
integrated in the SAME cycle, not the previous one (closes a full cycle
of avoidable heading/distance staleness the `stop_lead_ms` anticipation
constant had been partly compensating for — see the turn-execution review
`docs/code_review/2026-07-22-turn-execution-review.md` D2/F3).

**119 ticket 005 (straight-leg crab fix — corrects 118's own restore) —
landed.** 118's restore above threw out the good half of the 112-005
hoist along with the bad: `drive_.tick()` sitting BETWEEN `motorL_.tick()`
and `motorR_.tick()` made L always write duty from a target staged ONE
CYCLE OLDER than R's — a genuine per-cycle L/R actuation skew during
every commanded ramp (measured +2.685° cruise yaw on a straight leg,
exact match to the predicted `v_cruise·kCycle/trackWidth` transient;
decel canceled it, so the net signature was lateral crab with ZERO final
heading error — +32.5mm over a 700mm straight, endpoint-only heading
checks provably blind to it). A second, independent defect compounded
it: `updateTlm()`/`tlm_.emit()` ran in the kClear block (between L's
collect and R's), pairing THIS cycle's fresh L against LAST cycle's stale
R in every outbound frame — a pairing skew that numerically CANCELED the
physical skew above, so every host-visible encoder view (`dL−dR`,
`encpose`, `frame.twist`) reported a perfectly straight leg while the
robot's true path crabbed. See
`clasi/issues/straight-leg-crab-118-001-actuation-and-telemetry-pairing-skew.md`
and `docs/code_review/2026-07-22-turn-execution-review.md` §9 for the
full derivation and measured numbers.

Both fixed together (fixing one without the other either leaves the crab
in place while making it visible, or removes it while still hiding a real
defect for any OTHER per-frame L/R consumer): `drive_.tick()` now runs at
the very TOP of `cycle()`, before EITHER motor's own select — restoring
the 112-005 hoist's one genuinely good half (same-generation actuation
staging) WITHOUT reintroducing the select-ordering bug 118's restore was
actually about (moving `drive_.tick()`'s position never touches select
ordering — the two are orthogonal, see §2's own interleave description
below). `updateTlm()`/`tlm_.emit()` moved to the START of the trailing
pace block, immediately after `motorR_.tick()`'s own collect — every
emitted frame now pairs same-generation L/R encoder samples. Measured
post-fix (`straight_drift_repro.py`'s own scenario, isolated 700mm
straight at 150mm/s, ideal chip): cruise heading 0.000° (was +2.685°),
final y +0.0mm (was +32.5mm) — an exact zero, not merely "reduced."

**Ack-latency consequence of the telemetry-emit move (documented, not
accidental).** `updateTlm()`/`emit()` now runs AFTER `processMessage()`'s
own command-dispatch call (R-settle block, still unchanged position) in
the SAME cycle, where 118's kClear placement ran BEFORE it. An
enqueue/command ack (CONFIG/MOVE-enqueue/STOP, staged via `tlm_.ack()`
inside `processMessage()`'s own handlers) therefore now typically rides
THIS SAME cycle's emitted frame instead of the next one — see §7.2 of
`docs/protocol-v5.md` for the wire-level statement. The MOVE COMPLETION
ack is UNAFFECTED: `moveQueue_.tick()` (the stop decision) still runs
AFTER `updateTlm()`/`emit()`, later in the SAME pace block, so a
completion ack staged there is still not visible until the NEXT cycle's
own `emit()` call — "ack rides the next frame," unchanged from before
this ticket.

**Both `MoveQueue::landAtZero()` completion-margin constants re-swept
(`move_queue.cpp`) — the actuation-staging fix shifts BOTH regimes, not
just the already-known-narrow chain-advance one.** `drive_.tick()`'s new
top-of-cycle position changes the plant's exact per-cycle response and
the average commanded-to-duty latency (both leaves now lag their own
freshly-staged target by 1 cycle, symmetric — was 0/1 split, averaging
0.5), which shifts both of `landAtZero()`'s own margin factors:

- `kStoppingMarginFactorChain` (`pendingCount() > 0`, 118 ticket 003's own
  narrow-pocket constant) — the shipped 0.60 value re-measured 3.457°
  worst-case at this schedule (TOUR_2/ideal turn 10,
  `test_tour_closure_gate.py`), over its own 2.5° gate. A fresh 1-D sweep
  at THIS schedule found a genuinely broad plateau (unlike 118-003's own
  narrow-pocket finding) at `[0.40, 0.50]` — 0.48 ships as the new default
  (worst=2.218°, 0.282° of margin).
- `kStoppingMarginFactorFinal` (`pendingCount() == 0`) — NOT anticipated
  by this ticket's own plan (118-003 had found this regime cadence-robust
  and left it untouched at 1.00); only surfaced by re-running the FULL
  gate set this ticket's own acceptance criteria require.
  `test_gui_button_acceptance.py`'s own isolated ±90° managed-turn presets
  (`test_managed_angle_preset`/`test_managed_seg_0_cdeg_turn`) went RED at
  the old 1.00 value — a genuine 3.267°/3.178° UNDERSHOOT (settle-based),
  over their own 3.0° gate. A fresh sweep found a broad plateau at
  `[0.88, 0.96]` (worst=0.316° throughout, identical at every sampled
  point in that range) — 0.92 ships as the new default, replacing 1.00.

Full sweep data for both in `move_queue.cpp`'s own updated
anonymous-namespace comments and ticket 119-005's own file.

**118 ticket 004 (land-at-zero, pulled forward from sprint 119,
2026-07-23) — landed.** Once the stop decision reads this-cycle odometry
(above), the unchanged `stop_lead_ms=45` lead OVERcompensates — a 0-120ms
sweep against the closure gate found no value with real margin (fresh
data confirming the constant's own multi-retune history: no single value
exists). Per the turn-execution review's own R6 rule, `stop_lead_ms` and
the anticipation block are DELETED rather than re-tuned. `MoveQueue::tick()`
gains a land-at-zero completion predicate instead — but NOT the static
`remaining ≤ ε AND |ω_cmd| ≤ ε_ω` form this note originally described
sketching: empirical tracing (printf-instrumented ticks against the sim
tour-closure gate) showed a static `ε_ω` set near the deadband floor
never binds before the raw backstop's own `remaining ≤ 0` already does —
the jerk-limited taper's own commanded speed doesn't cross a fixed floor
early enough to matter. What shipped instead is a DYNAMIC,
self-referential stopping-distance check: `remaining ≤
(commandedSpeed² / (2·decelCeiling)) · marginFactor` — "have we already
entered our own braking envelope for our current commanded speed," the
same closed-form the taper's own decel ceiling already uses. `marginFactor`
takes one of two empirically-swept values, selected by `pendingCount() >
0` (a chain-advance is already queued behind this MOVE — only the
ack-instant reading matters, since `Drive::stop()` never runs) vs.
`pendingCount() == 0` (this MOVE drains the queue to a genuine
`Drive::stop()`, so the real post-stop motor/PID coast reaches the plant
before it settles): this split was necessary because the sim
tour-closure gate (ack-instant measurement) and
`test_gui_button_acceptance.py` (settle/quiescence measurement) disagreed
about which single scalar "worked" — no value in [0.20, 1.10] satisfied
both until the predicate was made aware of which completion regime it was
in. See `move_queue.cpp`'s own anonymous-namespace comment for the full
sweep data (chain: 0.82-0.84 plateau, worst=2.398° against the gate's
2.5° band; final: 0.90-1.10 plateau, worst=1.189° settle-based against
the button-acceptance suite's 3.0° tolerance). The `StopCondition`
threshold/timeout comparison remains the always-armed backstop
(unchanged, and the ONLY completion path when shaping is off — an
all-zero `ShaperLimits` makes `shapeAndStage()` early-return, so the
commanded speed never bleeds and the land-at-zero gate never fires).
Scope is TWIST Angle/Distance stops only; TIME/WHEELS moves are
unaffected. `App::StateEstimator`'s `bodyAt()` — the anticipation block's
one call site — now has no firmware production consumer: the module,
`update()`, and its tests are QUARANTINED (kept, not deleted) as the
planned consumer for future fake-OTOS/fusion bench work, per the same
"greenfield, not yet wired to motion" posture the 117 note above already
established for the estimator as a whole.

**121 ticket 003 (land at zero at orthogonal chain boundaries — landed;
stakeholder decision, 2026-07-23) — splits the land-at-zero completion
predicate above by AXIS RELATIONSHIP at a chain boundary, not merely by
whether one is imminent.** With the 119-005 crab fix landed, TOUR_1/TOUR_2
per-leg TRUE-heading measurement isolated the remaining tour error to
chain-advance boundaries: every straight FOLLOWING a turn gained
+1.34–4.24° (mean ~+2.9°/boundary) because the ending turn completed on
the CHAIN margin with residual ω that decays into the next `Move` (which
does not command ω), arcing the straight's entry. `MoveQueue::
landAtZero()`/`tick()` now classify every chain-advance boundary
(`pendingCount_ > 0`) into one of two kinds via a new pure predicate,
`MoveQueue::sameAxisCompatible(next)`: a **same-axis compatible** boundary
(the incoming pending `Move` continues the ending `Move`'s own stop-kind
axis — `v_x` for Distance, `ω` for Angle — in the SAME direction, e.g. two
Distance legs both forward) keeps the existing CHAIN margin
(`kStoppingMarginFactorChain`/`kDiscretizationCyclesChain`, UNCHANGED,
sprint 122's own deferred velocity-carry scope); an **orthogonal**
boundary (turn→straight, straight→turn — the incoming `Move` does NOT
continue this axis) now lands the ending axis at zero via a THIRD,
dedicated constant, `kStoppingMarginFactorOrthogonal`, structurally
shaped like the drain-case FINAL branch (no discretization term) but NOT
numerically equal to it: the plan's own default proposal — reuse
`kStoppingMarginFactorFinal` (0.92) verbatim — was verified against the
closure gate, not assumed, and measurably FAILED (worst |turn error|
8.043°/7.863° ideal/realistic, against the shaped-band gate's 2.5°) —
TOUR_1/TOUR_2 alternate Distance/Angle unconditionally, so EVERY boundary
in both tours is orthogonal, and reusing the settle-based drain margin for
an ack-instant, never-settles chain-advance recreates the exact
measurement-convention mismatch that originally justified
`kStoppingMarginFactorChain`'s own existence, one level down. A dedicated
1-D sweep of `kStoppingMarginFactorOrthogonal` against the SAME gate found
a genuinely broad plateau at [0.665, 0.674]; 0.67 (mid-plateau) ships.
`kStoppingMarginFactorChain`/`kDiscretizationCyclesChain` now govern
ONLY a same-axis-compatible chain boundary — decoupled from orthogonal
accuracy, their remaining tuning is sprint 122's concern.

**Honest residual (the issue's own "if a residual remains" clause).**
Even at the shipped 0.67, the sprint's own aspirational SUC-074 targets —
straight-following-turn gain ≤0.3°, turn |error| ≤0.5°, TOUR_1 net
heading 540°±1° — are NOT met. Measured against `test_tour_closure_gate.py`
at 0.67: turn |error| 2.314° (ideal, TOUR_1)/2.100° (realistic, TOUR_2);
straight-leg cruise |delta| 4.104° (ideal)/9.852° (realistic); TOUR_1/ideal
net heading closure residual ~+21° over the 540° commanded. This is
COMPARABLE TO, not dramatically better than, the pre-ticket baseline
(chain margin applied uniformly, since 100% of TOUR_1/TOUR_2 boundaries
are orthogonal): turn 2.195°/2.218°, cruise 4.254°/9.307°, net closure
~+17.9°/+34.2° — this ticket's margin-only mechanism avoids the
disastrous naive-reuse regression and keeps every EXISTING hard gate
(shaped-band 2.5°, cruise 5.5°/10.5°) passing with real margin, but does
NOT deliver the hoped-for cruise/closure improvement. Root cause, and why
no further margin sweep can fix it: the residual ω/`v_x` that "decays
into the next Move" is the REAL PLANT's own post-reset momentum
(`tick()`'s own unconditional `shaperOmega_.reset()`/`shaperVX_.reset()`
zeroes the KINEMATIC shaper target instantly, but the physical
wheel/velocity-PID plant that had been tracking the PREVIOUS nonzero
target does not stop instantly) — a separate physical effect from "how
much of the taper's own v²/(2·a) remaining distance is left," which is
all a `marginFactor` scale on that formula can ever adjust. The full
[0.00, 1.00] sweep confirms this structurally: low margins minimize
cruise-leak but blow turn-error up via raw-backstop-driven OVERSHOOT;
high margins minimize net-heading closure (crossing ~0 around 0.85–0.90)
but blow turn-error up via a large systematic UNDERSHOOT that a following
straight leg's own compensating overshoot happens to cancel — a
two-wrongs-cancel artifact, rejected on that basis, not a genuine fix.
Closing this residual properly needs the issue's own analytic
`remaining ≤ |ω_measured|·(kCycle/2 + τ_plant)` form using an ACTUALLY
MEASURED velocity (not this predicate's own kinematic `cmd`) and an
independently-characterized `τ_plant` (measured via an isolated
step-response test, not fitted against this same closure gate) — both
are new capability beyond this ticket's authorized scope and are flagged
for a follow-up ticket rather than rushed into a second fitted constant.
Full sweep table and derivation in `move_queue.cpp`'s own
anonymous-namespace comment.

**120 (bench tour bring-up: ack ring + build-selectable fake OTOS + I2C
safety-net diagnosis — all three tickets LANDED).** Three independent,
phase-B bench-observability fixes. See "Telemetry's ack ring" (§4) for
the ack-slot→ack-ring change and the `kFlagFaultI2CSafetyNet` paragraph
(§4) for the bit-6 diagnosis (ticket 3, LANDED: diagnosis-only, no code
fix — see that paragraph for the confirmed root cause and why no fix
ships). The third change (ticket 2, LANDED): a compile-time `FAKE_OTOS` bench
build variant that reports the dead-reckoned `Odometry` pose in place of a
real OTOS burst read. **This mechanism was later reshaped by the
otos-fake-seam refactor** (see §2's "Otos call site" and
[`devices/DESIGN.md`](../devices/DESIGN.md)): what shipped in 120-002 as a
`Devices::Otos::feedSyntheticSample()` method + a per-cycle `#ifdef
FAKE_OTOS` branch in `RobotLoop::cycle()` is now `Devices::Otos` as an
abstract interface with two implementations — `Devices::RealOtos` (the
chip) and `App::FakeOtos` (the synthesizer) — selected once at the
`main.cpp` composition root. The bench property it established is
unchanged: this is the first FIRMWARE PRODUCTION CONSUMER of the "OTOS is
present and reads a meaningful pose on a stand" property the previous
paragraph's quarantine note anticipated — NOT yet a consumer of
`StateEstimator::bodyAt()` itself (that stays quarantined; the fake feeds
`frame.otos` one layer below the estimator, and `StateEstimator`'s own
OTOS-fusion weights stay 0.0/0.0 in `defaultEstimatorConfig()`).
`odom_.integrate()`/`frame_.pose` staging runs immediately BEFORE the Otos
call site so a `FakeOtos` reports THIS cycle's genuinely fresh pose — see
§2's own "Otos call site" paragraph for why this reorder is
side-effect-free for the real build.

**Hardware verification (2026-07-23, robot "tovez",
`/dev/cu.usbmodem2121102`).** Flashed via `mbdeploy deploy <uid> --hex
MICROBIT.hex` (built with `uv run python3 build.py --fw-only --fake-otos
--clean`). Confirmed `frame.otos` tracks commanded motion exactly:
forward drive (300mm/s, 2s) took `pose`/`otos` from `(0,0,0)` to
`(555,-119,-22.9deg)` on BOTH fields identically; a 90° turn continued to
`(548,-123,+67.2deg)` on both, again identically. A bench tour
(`src/tests/bench/fake_otos_tour_bench.py`, TOUR_1, 13 legs, driven with
bounded enqueue-ack retry over the known lossy link —
`bench-move-commands-intermittently-never-reach-firmware.md` —
independently confirmed still ~8-12% one-way loss even with ticket 1's
ack ring proven solid) closed twice in a row (13/13 legs completed each
run); `frame.otos` matched `frame.pose` on every one of 435-436 polled
frames per run, 0.00mm/0.00deg deviation, `otos_present` true on 100% of
frames. The real (table) build's own physical symptom is confirmed
UNCHANGED (not just via code diff): the identical forward-drive command
against the real build gave `pose=(569,-58,-12.3deg)` (encoders counting)
against a near-static `otos=(47,-3,0.0deg)` — the exact "useless on a
stand" behavior the source issue describes, proving the real
`Devices::Otos::tick()`/`begin()` path is genuinely untouched, not merely
textually unchanged. Full per-leg numbers and the retry mechanism's own
design (including a real single-consumer-queue bug the bench script's
first draft hit and fixed) are recorded in ticket 002's own file.

**122 (motion-library extraction — landed, 2026-07-24, reconciled ticket
004).** `App::MoveQueue`/`App::Odometry`/`App::StateEstimator` — all
three described above and throughout this file's history as `app/`'s own
modules through sprint 121 — MOVE to `src/motion/` (a new sibling tree of
`src/firm`, not a child of it) and are renamed `Motion::MoveQueue`/
`Motion::Odometry`/`Motion::StateEstimator`. `App::Drive` narrows to the
wheel-target sink only (`setWheels()`/`stop()`/`tick()`), losing
`setTwist()`/its `BodyKinematics` dependency to `Motion::MoveQueue`,
which now calls `BodyKinematics::inverse()` itself and hands already-
decomposed wheel targets down through a new boundary interface,
`Motion::WheelSink` (`src/motion/wheel_sink.h`) — `Drive` implements it;
nothing else in `app/` does. `BodyKinematics` itself moves out of
`src/firm/kinematics/` to `src/motion/body_kinematics.{h,cpp}` (flat, no
nested `kinematics/` under `src/motion`) for the same reason. Every
mention of `App::MoveQueue`/`App::Odometry`/`App::StateEstimator`,
`app/move_queue.*`/`app/odometry.*`/`app/state_estimator.*`, or
`src/firm/motion/`/`src/firm/kinematics/` earlier in this file's own
history (115-121) is an accurate PRE-122 record of where that code lived
and what it was called AT THE TIME — not restated or renamed throughout
this document — see [`src/motion/DESIGN.md`](../../motion/DESIGN.md) for
the current orientation, module list, and boundary contract, and
`docs/design/design.md` §2/§5 for the two-layer split at the system
level. Zero behavior change (this was a pure mechanical move, sprint
122's own refactor gate); `cycle_busy`/`cycle_period` (122-003, §4 below)
is the one independent, additive change riding in the same sprint.
`velocity_pid.*` and `Devices::NezhaMotor`'s PID ownership are UNCHANGED,
still base-side (sprint 122 Design Rationale Decision 2 — the duty-sink
rewrite that would move them is deferred to sprint 2).

**123 (COBS+CRC binary framing + telemetry migration) — landed.**
`Comms` (`comms.{h,cpp}`) and the two transports (`com/serial_port.*`,
`com/radio.*`) cut over from `*B<base64>\r\n` line armor to a
binary-clean byte stream that demuxes a `0x00`-delimited COBS+CRC frame
from the `\r`?`\n`-terminated HELLO/PING text rump on the same stream
(`App::FrameKind`) — a flag-day cutover, no dual-stack, base64 armor
call sites removed from `app/` entirely (`WireRuntime::base64Encode()`/
`Decode()` themselves are retained only for an unrelated debug harness,
see `messages/DESIGN.md` §3). `wire.h`'s envelope-size budget is
recomputed from 186 to 240 bytes, restoring the primary frame's
headroom; ticket 004 uses that headroom to migrate `cycle_busy`/
`cycle_period` off `TelemetrySecondary` (where 122-003 landed them as an
interim placement) onto the primary `Telemetry` frame every cycle. Zero
schema change beyond that one field relocation and the two envelope-size
constants — every `CommandEnvelope`/`ReplyEnvelope` field shape is
otherwise untouched. See §4 below for the full technical detail and
`docs/protocol-v5.md` §2/§8 for the wire-level reference (superseded — the
COBS delimiter/CRC scope/reply grammar it describes were themselves
replaced by sprint 124's protocol v5 cutover, see this section's own
"124-005"/"AS OF 124" paragraphs below).

**124-005 (protocol v5 Part A, "framing grammar cutover") — landed.**
123's own `App::FrameKind`-based transport-level demux (a heuristic
guess, per completed line, about which of two incompatible framings it
was — a `0x00`-delimited binary frame or a `\r`?`\n`-terminated cleartext
line) is DELETED wholesale, not adapted — the same "supersedes, does not
port" treatment 115-005 gave `Pilot`/`HeadingSource` and 116-005 gave
`App::Deadman`. In its place: ONE uniform packet grammar in both
directions, `<COMMAND>[':' <data>]'\n'` (issue
`protocol-v5-one-line-packets-command-prefix-and-newline-cobs.md` §1),
made possible by 124-003's delimiter-parameterized COBS primitive now
being called with `delimiter=0x0A` on the live wire (`App::
kCobsDelimiter`, `comms.h`) — a COBS-encoded binary body can never
contain a literal `0x0A` by construction, so `\n` is a genuine,
unconditional line terminator for BOTH transports (see
[com/DESIGN.md](../com/DESIGN.md) §2), and `Transport::readLine()`
collapses from returning `FrameKind` to a plain `bool`. `Comms::
dispatchLine()` parses the `<COMMAND>` prefix off the already-`\n`-
delimited line the transport hands it, looks it up in the generated
command registry (`messages/commands.h`'s `kVerbTable[]`, ticket
124-001), and dispatches by that lookup's OWN `binary` flag — never by
inspecting `<data>`'s own bytes — to either `decodeBinaryFrame()` (the
existing COBS+CRC dearmor path, now handed the parsed command name as
the CRC's scope-extension input, ticket 124-003's own primitive) or the
new `dispatchCleartext()`. An unrecognized `<COMMAND>` increments
`malformedCount_` exactly like a CRC/COBS failure already did — one
fault-bit source, not two.

**Relay control-plane sigil carve-out (124-010,
relay-handshake-trips-comms-malformed.md).** `dispatchLine()` drops a
line whose first byte is `#`/`!`/`?` — the radio-relay dongle's own
control-plane sigils (a status/comment reply, a dongle command, or the
dongle's config query, respectively) — BEFORE the registry lookup,
uncounted (`isRelayControlPlaneLine()`, comms.cpp). This closes a
one-shot-at-connect false trip of `kFaultCommsMalformed` over the radio
relay path only (never direct USB, which never runs the dongle's
`!ECHO OFF`/`!MODE RAW250`/`!GO` handshake at all): a fragment of that
host↔dongle handshake traffic could reach the robot's `radioLink_`
before/at the moment the dongle commits to transparent pass-through. No
registered v5 verb starts with any of these three bytes, so the
carve-out is narrow by construction and cannot mask a genuine malformed
command — it mirrors a tolerance `host/robot_radio/io/serial_conn.py`'s
own `_handle_wire_line()` already had for `#` lines, extended to the
firmware side (which had none) and to the two other sigils the dongle's
own handshake actually uses. The exact mechanism by which dongle
control-plane bytes reach the robot's radio receiver is NOT confirmable
further from this repository — the dongle ("gozop") is a separate,
external firmware with no source in this tree; see the ticket's own
completion notes for the full root-cause discussion and what a
hardware-side wire capture would need to show to settle it definitively.

`dispatchCleartext()` answers four verbs: `HELLO` → the existing
`banner_` (`sendBanner()`'s own content, unchanged, issue §8 confirms
`DEVICE:NEZHA2:...` already conforms to the grammar with no edit);
`PING` → `PONG:t=<ms>` (replaces the pre-124-005 `"OK pong t=<ms>"`
shape); `ID` → `idLine_`, a caller-owned string set once at construction
(sprint 124 architecture Decision 4: CONFIGURED identity — drivetrain
type + calibration-profile name — distinct from `banner_`'s hardware
identity; `main.cpp` builds it from two new generated constants,
`Config::kDrivetrainType`/`Config::kRobotProfileName`
(`scripts/gen_boot_config.py`, baked from the robot JSON's own
`identity.drivetrain_type`/filename stem — deliberately NOT derived from
any wire-level `msg::DrivetrainConfig` field, since
`defaultDrivetrainConfig()` never bakes `half_track`, which stays at its
wire default `0.0f` for every profile); `VER` → `"VER:" +
FIRMWARE_VERSION_STR`, reading the existing generated
`version_generated.h` constant directly (Decision 4: zero new
version-tracking infrastructure). All four reply via `sendReliable()`,
matching the pre-124-005 HELLO/PING reply path's bounded-wait policy.

`Comms::sendReply()`'s signature simplifies to take only the
`ReplyEnvelope` — the outbound verb name (for both the wire line's own
`<COMMAND>':'` prefix and the CRC's scope-extension input) is now derived
INTERNALLY from `reply.body_kind` (`TLM`/`OK`/`ERR` map 1:1 onto
`messages/commands.h`'s `Verb::TLM`/`OK`/`ERR`) rather than threaded in
by the caller — see §5 below. **Superseded by 124-009**: at the time
124-005 landed, `Telemetry::emitSecondary()` still existed and reused the
same `TLM` verb/CRC-scope for `TelemetrySecondary`'s own independently-
framed line (no registry entry existed for the secondary diagnostic frame,
so the host structurally disambiguated the two shapes by trial-decode).
124-009 deleted `TelemetrySecondary` — frame type, `emitSecondary()`, and
the trial-decode disambiguation — outright (see this section's own
"AS OF 124" paragraph below), so there is only ever one outbound `TLM`
shape now and this consideration no longer applies. `kFramedMaxBytes`/
`kMaxCrcPayloadBytes` (`comms.h`) are unchanged by this ticket (the
command prefix lives OUTSIDE the COBS-encoded region); a new
`kMaxLineBytes = kFramedMaxBytes + kMaxCommandPrefixBytes` constant
replaces 123-002's `kArmoredBufSize` as the whole-line scratch-buffer
size, `kMaxCommandPrefixBytes` itself derived at compile time from
`messages/commands.h`'s own longest verb name rather than hand-picked.

## 2. Orientation

`RobotLoop` has two phases. `boot()` steps `Preamble` until every device
leaf reaches a terminal state (present-and-ready or confirmed-absent),
emitting a boot telemetry frame each pass; commands are not consumed during
boot. `cycle()` is the steady-state loop body. It opens with `Drive::tick()`
(119 ticket 005 — pure computation, before either motor's own select, so
both leaves apply the SAME staged target this cycle — see §4's own
"same-generation actuation staging" note), then interleaves per port (118 —
select L → collect L → select R → collect R, the schedule this section
always claimed for the request/collect halves): request/settle(borrow:
`Comms::pump`)/collect/PID for the left motor, a post-duty clearance window
(119 ticket 005: no borrowed work left here — see below), request/
settle(borrow: `processMessage`)/collect/PID for the right motor
(immediately followed by the wheel section's own `state_` publish —
124-009, see below), then a trailing pace block that integrates odometry
(`Odometry::integrate`), samples OTOS inline (uniform across builds; the
sensor behind it is a real chip or an `App::FakeOtos`, chosen at
construction), polls line/color at a rate-limited, alternating cadence,
refreshes `Motion::StateEstimator`'s predict-to-now estimates from that
same cycle's published `state_` (117), and ONLY THEN calls
`tlm_.update(state_)`/`tlm_.emit(now)` (124-009 — the one assembly point,
LAST among the pace-block's own publishers so every section it projects
is already coherent; superseding this paragraph's former "FIRST stages
and emits telemetry" ordering, which predates the RobotState blackboard),
evaluates the `MoveQueue`'s unconditional per-cycle stop decision
(`moveQueue_.tick(now, odom_)` — 118: AFTER odometry/estimator refresh
AND AFTER `tlm_.update()`/`emit()`, so the decision reads THIS cycle's
data and a completion ack still rides the NEXT frame, protocol-v5 §7.2),
and paces the whole cycle. `Drive`, `Odometry`, and `MoveQueue` are pure,
bounded, non-bus-touching helpers that `RobotLoop` calls at specific
points in its own schedule; `MoveQueue::tick()` is called unconditionally
once per cycle and drains to `Drive::stop()` once its queue empties.
See `robot_loop.cpp` for the exact call order — it is the schedule's single
source of truth.

**RobotState assembly + `Telemetry::update()`/`emit()` (124-009,
robot-state-blackboard-...md, superseding the `updateTlm()`/scattered-
`setFlag()`/secondary-tie-break shape this paragraph used to describe).**
`RobotLoop` now owns a persistent `Types::RobotState state_` member (the
blackboard, `src/firm/types/robot_state.h`, ticket 007) instead of a bare
`Telemetry::Frame frame_`. Each section publishes into `state_` at its own
earliest point of coherence: the wheel section (`state_.wheelLeft/Right`
— position/velocity/`sampleTime()`/connected/positionEpoch/cmdVelocity,
plus the position-rebaseline clamp) publishes immediately after
`motorR_.tick()`'s own collect (same-generation L/R, unchanged reasoning
from the 119-005 paragraph above); otos/perception/pose/command/health
publish in dependency order inside the trailing `kPace` block (sensors →
odom → estimate, matching `Motion::StateEstimator::update(state_, now)`'s
own input needs — `state_` IS `Motion::StateEstimator::Input` now, ticket
007's alias, so there is no more hand-copied `estimatorInput` local
either). `tlm_.update(state_)` is the ONE call that projects the whole
blackboard into the wire frame AND derives every flag (replacing both the
old `updateTlm()`-style field staging and the ten scattered
`tlm_.setFlag()` calls with one method) — `RobotLoop::cycle()` itself
never calls `tlm_.setFlag()` at all any more (grep-enforceable:
`grep setFlag src/firm/app/robot_loop.cpp` returns nothing).
`kFlagFaultMoveTimeout`/`kFlagFaultShapingDisabled` are the one documented
exception (their defining condition, `MoveQueue::tick()`'s own outcome,
isn't known until after `tlm_.update()`/`emit()` run) — `RobotLoop` sets
those via `tlm_.setLiveFlag(bit, active)`, a narrow, deliberately
NOT-named-`setFlag` escape hatch (telemetry.h's own doc comment).
`Telemetry::emit(now)` sends the primary frame when its own cadence gate
says it's due — there is no secondary frame or tie-break to arbitrate any
more (`TelemetrySecondary` is deleted outright, not deprecated — see §5's
own updated entry). `age` fields (`EncoderReading.age`/`OtosReading.age`,
issue §B2/SUC-006) are computed inside `Telemetry::update()` as
`(cycleStart + cycleBusy) - sampleTime`, genuinely independent per reading
(`Devices::Motor::sampleTime()`/`Devices::Otos::sampleTime()`, ticket
002) — not the shared, always-zero stand-in ticket 008 left in place.

**TLM action application + STATUS projection now live in `Telemetry`/
`Comms`, not the loop (128-012, `tlm-mode-switch-belongs-in-telemetry-
not-the-loop.md` / `status-projection-belongs-in-comms-not-the-loop.md`).**
Two adjacent blocks that used to be inlined in `RobotLoop::cycle()`
right after `tlm_.update(state_)` moved out to the modules whose policy
they actually are — the same "the loop says WHEN, the modules say WHAT"
split every other section of this design doc already follows.
`Telemetry::applyAction(Comms::TlmAction)` absorbs the mode-change switch
(`kSetOff`/`kSetAuto`/`kSetOn` → `setMode()`) AND the "is this a
force-a-frame request" answer (`kFrame` → returns `true`); `RobotLoop`
passes its return value straight into `tlm_.emit(now, forceFrame)` — 
`emit()`'s own `force` parameter is unchanged, only WHERE the decision is
computed moved (telemetry.h's own doc comment on `applyAction()` records
the dependency-direction choice: `Telemetry` takes `Comms::TlmAction` by
value, since `comms.h` was already an unconditional dependency of
`telemetry.h`). `Comms::updateStatus(state, tlm)` absorbs the 8-field
`Comms::Status` projection RobotLoop used to hand-assemble field by
field — the same "one struct, filled once" idiom `Telemetry::update()`
already uses for its own wire frame, with the two Telemetry-sourced
fields (`flags`/`tlmMode`) read straight off an explicit `const
Telemetry&` parameter rather than a stored member (`comms.h` forward-
declares `Telemetry` to avoid the include cycle `telemetry.h`'s own
`#include "app/comms.h"` would otherwise create). `status.ready` — the
one genuinely loop-owned fact ("past `boot()`") — moved onto
`state_.health.ready`, set exactly once at the tail of `RobotLoop::
boot()`, rather than hard-coded `true` at the old projection call site.
Two orderings stay load-bearing and are still `RobotLoop::cycle()`'s own
job to preserve (not something either new method can enforce on its
own): `applyAction()` before `updateStatus()` (a same-cycle mode change
must already be reflected in `tlm_.mode()` when STATUS is assembled),
and `updateStatus()` before `comms_.sendTlmReply()` (the STATUS/HELP
reply must report the mode just applied, not last cycle's).

**Line/color polling (a plain inline block in `RobotLoop::cycle()`'s own
trailing `kPace` block, 115-005; 124-009 folded the former
`updateLineColor()` method into that block directly — no separate method
exists any more).** Runs once per cycle. Ticks EXACTLY ONE of
`Devices::LineSensorLeaf`/`Devices::ColorSensorLeaf` per call — never both —
alternating which one on the NEXT call, so at most one of the two is even
OFFERED a chance to check its own `readDue()` in any given cycle (the
098-004 per-pass-read regression precedent: a per-cycle sensor read must
never disrupt the motor request/collect cadence). Each leaf's own
`tick()`/`readDue()` rate-limits the actual bus transaction further (the
same `Otos::readDue()` pattern `Devices::Otos` already uses). A fresh
reading packs into `state_.perception.line`/`.color` (4 raw grayscale
bytes, ch1 low byte; RGBC scaled 16→8 bits, R low byte, via `packLine()`/
`packColor()`, still file-local to `robot_loop.cpp`'s own anonymous
namespace — `Types::RobotState` may only ever hold cstdint-level data, so
the packing step cannot move into `Telemetry::update()`) and sets
`state_.perception.lineFresh`/`.colorFresh` for THIS cycle only —
`Telemetry::update()` derives the corresponding `flags` bit (13/14) from
those, clearing the OTHER leaf's own bit the same cycle (it was not even
touched), matching the wire spec's "line/color word fresh" (fresh THIS
frame, not merely "known at
some point") semantics.

**Otos call site (uniform across builds; `FAKE_OTOS` chosen at
construction).** Runs once per cycle from the trailing `kPace` block,
immediately after `odom_.integrate()`/`state_.pose` staging (this pair is
hoisted to run BEFORE the Otos call so an `App::FakeOtos` reports THIS
cycle's fresh pose — a side-effect-free reorder for the real leaf, since
`Odometry::integrate()` reads neither `otos_` nor any `state_.otos*`
field and vice versa). There is **no `#ifdef` here** anymore: the loop
holds a plain `Devices::Otos&` and calls `otos_.tick(nowUs)` directly,
publishing `state_.otos.present`/`connected`/`x`/`y`/`heading`/`v_x`/
`v_y`/`omega`/`sampleTime` inline in `RobotLoop::cycle()` itself (124-009
— the former `applyOtosSample()` free function, `app/odometry.*`,
predates the RobotState blackboard and no longer exists; this call site
is now the single place that translates `otos_`'s own
`connected()`/`present()`/`poseFresh()`/`pose()`/`sampleTime()` into the
blackboard). Which implementation backs `otos_` — the real SparkFun leaf
(`Devices::RealOtos`, a rate-limited I2C burst read) or the bench
synthesizer (`App::FakeOtos`, which reports the dead-reckoned `Odometry`
pose + `BodyKinematics`-fused wheel twist) — is chosen ONCE at the
`main.cpp` composition root under `#ifdef FAKE_OTOS`, the only place that
macro appears (otos-fake-seam refactor, superseding 120-002's per-cycle
branch + the deleted `Devices::Otos::feedSyntheticSample()`). See
[`devices/DESIGN.md`](../devices/DESIGN.md) for the `Otos` interface /
`RealOtos` split and [`app/fake_otos.h`](fake_otos.h) for the fake.

**Predict-to-now estimation (`RobotLoop`'s `StateEstimator::update()`
call, 117; 124-009 threads `state_` directly).** Runs once per cycle from
the trailing `kPace` block, immediately after `state_.pose` is staged and
the Otos call site above has run (120 ticket 002 reordered which of the
two stages first — see that paragraph above — `update()`'s own position
in the schedule, relative to both being done, is unchanged).
`stateEstimator_.update(state_, now)` reads `state_.wheelLeft`/
`state_.wheelRight` (position, velocity, their own `sampleTime`) to
refresh each wheel's peer `WheelEstimate` basis, and `state_.pose`
(already fused by `Odometry`/`BodyKinematics::forward()` earlier the same
cycle) plus `state_.otos` (when `present`) to refresh the body peer's
`BodyEstimate` basis via the v1 complementary blend. `Types::RobotState`
IS `Motion::StateEstimator::Input` (ticket 007's alias) — no hand-copied
intermediate struct exists any more. Pure computation over already-staged
data — no I2C access, no sleep, bounded work, same posture
`Odometry::integrate()` and the Otos call site above already keep in this
same block.

## 3. Constraints and Invariants

- **Single-loop bus ownership:** every I2C transaction happens from
  `RobotLoop::cycle()`'s own call sequence. No app module ever initiates bus
  traffic from its own `tick()`/staging methods on its own timing — see the
  system doc's "single-loop bus ownership" invariant (`docs/design/design.md`
  §5). `Odometry::integrate()`, the `otos_.tick()` call site, and the
  line/color alternation (124-009: both now plain inline blocks in
  `RobotLoop::cycle()`'s own trailing pace block, not separate methods —
  see §2's own "Otos call site"/"Line/color polling" paragraphs) are
  called only from that block, never from inside a motor request→collect
  window.
- **The timing schedule is exactly `robot_loop.cpp`'s `runAndWait` calls:**
  `grep 'runAndWait\|sleepUntil' app/robot_loop.cpp` must remain the
  firmware's complete list of waits. A sleep hidden inside any other
  function (a module's `tick()`, a handler, a helper) silently breaks the
  cycle's timing budget and starves the CODAL fiber scheduler — the radio
  looks dead when the loop fails to yield, even though nothing is wrong
  with the RF.
- **`runAndWait` bodies other than the final block never touch the bus and
  never sleep:** they exist only to spend an already-mandatory clearance
  window on other bounded work (comms pump, telemetry assembly, command
  dispatch). Moving bus traffic into one of these bodies reintroduces the
  shared-bus timing collisions a single-master I2C bus cannot tolerate.
- **Command dispatch is bounded to at most one per cycle:** `Comms::pump()`
  decodes at most one frame per call by construction (at most one
  `readLine()` per transport, first transport to have something wins), so
  `processMessage()` needs no separate "already handled" flag.
- **No deadman — every `Move` is structurally self-bounding:**
  `App::MoveQueue::tick()` runs unconditionally once per cycle and drains
  to `Drive::stop()` once the active `Move`'s `Motion::StopCondition` or
  `timeout` fires and nothing is pending — an emergent property of every
  queued `Move` carrying its own bound, not a second, independently-timed
  staleness timer. `App::Deadman` does not exist in this tree. Do not add
  an ad hoc watchdog anywhere in `app/`.
- **Telemetry always carries the last staged snapshot, not a diff:** a
  cycle that doesn't update a `Frame` field still sends whatever was last
  staged. Nothing here is "only send on change" — a dropped or unread frame
  never loses data because the next one repeats it.
- **`Frame` fields written late in a cycle are read by the NEXT cycle's
  emit, not lost:** pose/OTOS/line/color are staged at the end of the cycle
  they are computed in, and picked up by the following cycle's
  `updateTlm()` + `emit()`. This is deliberate — treat a one-cycle
  staleness on those fields as normal, not a bug.
- **Devices isolation still applies inside `app/`:** wire-plane `msg::*`
  types are converted to/from `Devices::*` types only in `main.cpp`
  (outside this directory); no app module should reach around that.
- **Line/color are now sampled in steady state (115-005), OTOS is not the
  only one any more.** The previous version of this note said the
  opposite — `Preamble` detects presence at boot; `updateLineColor()` (§2
  above) now also samples both in steady state, rate-limited and
  alternating. There is still no full 3-way round-robin abstraction
  (otos|line|color) — each sensor is its own bounded step, not a unified
  scheduler class.
- **Config patches cover `MotorConfigPatch`, `OtosConfigPatch`
  (109-004), and `EstimatorConfigPatch` (117) only.** `RobotLoop::
  handleConfig` replies `ERR_UNIMPLEMENTED` for `DRIVETRAIN`/`WATCHDOG`/
  `NONE` (`DrivetrainConfigPatch` has no on-robot fusion consumer).
  `PlannerConfigPatch` is GONE, not merely out of scope — 115-005 (gut
  S1) deleted the type and `ConfigDelta`'s own `PLANNER` oneof arm
  entirely, along with `Pilot`/`Motion::Executor`, the only things that
  ever consumed it. `OtosConfigPatch` (issue
  `otos-calibration-config-message.md`) restores a RUNTIME path to
  `Devices::Otos::setLinearScalar()`/`setAngularScalar()`/`setOffset()`/
  `init()` — previously only ever called once at boot from baked
  `boot_config` — applied immediately and synchronously inside
  `handleConfig()` (still "the loop's own cycle" per the single-loop bus
  ownership invariant above: a rare, command-triggered I2C/config
  transaction sandwiched into the existing schedule, not a new per-cycle
  bus consumer). `EstimatorConfigPatch` (117) merges present
  `weight_heading_otos`/`weight_omega_otos`/`staleness_ms` fields onto
  `StateEstimator`'s own live weight state — a pure in-memory update, NOT
  an I2C transaction (unlike the OTOS branch above), and NOT persisted
  into `persistedTuning_`/flash (Design Rationale Decision 4, overlay
  `design.md`'s sibling — a reboot reverts to the baked JSON default).

## 4. Design

**Why one loop.** `RobotLoop::cycle()` is deliberately one function with
every bus transaction and every wait visible in call order, rather than a
dispatch graph of modules each with their own timing. The alternative
(subsystems/fibers each owning a slice of the schedule) hides the bus
schedule and the sleeps inside layers, which makes both hard-realtime
problems — bus discipline and fiber-scheduler yielding — undebuggable.
Modules (`Drive`, `Odometry`, `Telemetry`, `Comms`, `MoveQueue`, `Preamble`)
were factored *out* of that one function only as passive, bounded helpers;
none of them run their own timing loop.

**The timing primitive.** `runAndWait(gap, body)` marks time, runs `body`,
then sleeps until at least `gap` has elapsed since its own mark. Each block
anchors to its *own* mark rather than a shared cycle-start mark, so a slow
body degrades gracefully — its sleep shrinks toward (never below) 1ms
instead of stacking on top of an unrelated deadline. The schedule has four
such blocks: left-motor settle, post-duty clearance, right-motor settle,
and a final perception+odometry+pace block. The four gaps
(`kSettle`, `kClear`, `kSettle`, `kPace`) are sized so their sum equals the
whole-cycle target `kCycle` (40ms / ~25Hz — 118: restored from a fictional
20ms/~50Hz that `kSettle=kClear=0` had been faking, see §1's "118 (loop
schedule truth)" note) — `kPace` is *derived* as
`kCycle` minus the other three, not a second independent `kCycle`-sized
sleep, specifically so the schedule's total holds even under a
zero-real-time-cost virtual clock (anchoring the final block to the cycle
start instead of its own mark was a diagnosed defect: it double-counted the
first three blocks' time against the target). `kCycle` matches
`Telemetry::kPrimaryPeriod` by construction (115-005: primary period now
EQUALS the cycle period — every loop iteration emits a frame, closing
`kcycle-kprimaryperiod-mismatch.md`) so the primary-frame throttle and the
loop's own pace agree.

**Command dispatch.** `processMessage` reads the `Cmd` populated (or not)
by this cycle's single `Comms::pump()` call and switches on `cmd_kind`:
`MOVE` validates the envelope's shape (velocity variant present, stop
variant present, `timeout > 0`) and the config-completeness gate, then
delegates to `moveQueue_.enqueue()` (`replace=true` flushes pending and
preempts the active `Move`; `replace=false` enqueues, or acks `ERR_FULL`
past 4 pending); `STOP` stops `Drive` (immediate, safety-critical) and
flushes `moveQueue_`; `CONFIG` merges present wire fields into each
motor's *own* current gains (never blanket-copies one motor's gains onto
the other — their calibration can legitimately differ) and applies
`travel_calib` to whichever motor `side` names, or (OTOS arm) applies
scale/offset/init directly. Every path that applies a command acks via
`Telemetry::ack(corrId, errCode)` (115-005: a single ack slot, not a
ring — see "Telemetry's ack slot" below); `moveQueue_` additionally emits
a completion ack against `Move.id` (the same `Telemetry::ack()` call) when
the active `Move` ends, whether by its stop condition or by `timeout` —
the latter also sets `kFlagFaultMoveTimeout` (bit 15, see below).
`Comms`'s dearmor path itself never replies synchronously — a malformed
frame is silently counted (`Comms::malformedCount()`) and surfaced as a
telemetry flags bit instead of answered inline. This keeps replies
flowing through one channel (the ack slot) rather than two.

**Telemetry has one send path (historical: it used to have two).** The
primary frame (`msg::Telemetry`, ack ring + `flags` + pose/enc/vel/otos/
line/color + `cycle_busy`/`cycle_period`, 123-004 — see below) rides a
`ReplyEnvelope` through `Comms::sendReply()`. Through 123, a SECOND,
independently-COBS+CRC-framed diagnostic frame (`msg::TelemetrySecondary`)
also existed: not a `ReplyEnvelope` oneof arm, so `Telemetry` held its own
`Transport&` pair and performed its own frame-and-broadcast for it
(123-002 — base64 armor+broadcast before that), and `emit()` alternated
between the two when both were due in the same call rather than always
favoring primary (at the real loop period, ~40ms/118, primary is due on
essentially every call, so an unconditional "primary wins ties" rule would
have starved secondary to 0Hz). **124-009 deleted `TelemetrySecondary`
outright** — the frame type, `emitSecondary()`, `Telemetry`'s own
`Transport&` pair, and the alternation/tie-break logic are all gone, not
merely unused (issue's own "Telemetry is a lean projection — and
TelemetrySecondary dies"). `Telemetry` now holds a `Comms&` only (no
direct transport references at all) and `emit()` is a plain "primary due
since last send" gate — there is no second frame type to arbitrate against
any more. See this section's own "AS OF 124" paragraph below for the full
disposition.

**`cycle_busy`/`cycle_period` loop-timing fields — landed on the PRIMARY
frame (123-004, `Telemetry` fields 15/16, ADDITIVE).** Originally landed
on the SECONDARY frame as an interim placement (122-003,
`TelemetrySecondary` fields 11/12): the issue that motivated these
(`telemetry-report-loop-cycle-duration.md`) always targeted the PRIMARY
per-cycle frame, but at that time `msg::Telemetry`'s own
`ReplyEnvelope`-wrapped worst case sat 1 B under the shared 186-byte
base64-armored envelope budget, and the serial transport carried a
second, tighter, independent ceiling underneath that budget (CODAL's
`Serial` TX ring buffer caps at 254 usable bytes; the 186-byte budget's
armored+CRLF wire line already consumed 252 of those) — raising the
primary budget at all guaranteed mid-line truncation on serial.
`TelemetrySecondary`'s own worst case had no such problem, so the two
fields landed there first.

Sprint 123's COBS+CRC binary framing (tickets 001/002) replaced the
base64 armor's ~33% expansion with COBS+CRC's fixed ~4-byte overhead
(CRC + one COBS code byte + the trailing `0x00` delimiter, independent of
payload content), letting the envelope budget be recomputed from 186 to
**240 bytes** (`wire.h`, `messages/DESIGN.md` §3) — restoring the primary
frame's headroom. Ticket 004 then migrated `cycle_busy`/`cycle_period`
onto the primary `Telemetry` frame (fields 15/16, `188 B` standalone /
`194 B` wrapped as the `tlm` arm, 46 B under the new budget — see
`docs/protocol-v4.md` §8.3) and marked `TelemetrySecondary`'s former
fields 11/12 `reserved`, not reused (Open Question 4 resolved: remove
from secondary rather than keep both, since two frames independently
reporting the same fields at different cadences risks silent
divergence with no benefit). Staged EXACTLY as before — RobotLoop's own
`previousCycleStartUs_`/`everCycled_` bookkeeping (independent of
`markTime()`'s `[ms]`-truncated `cycleStart` — a separate
`clock_.nowMicros()` read gives the `[us]` resolution these diagnostics
need) — only the destination frame moved, so every primary frame now
carries this cycle's own fresh timing data instead of secondary's ~5 Hz
sample (whichever cycle RobotLoop last staged before secondary happened
to be the frame `emit()` sent).

**Telemetry's ack ring (120 ticket 001, LANDED — replaces the 115-005
single-slot design, which itself had replaced the original depth-3
`AckEntry` ring).** Bench measurement at the real 40ms cycle / ~15Hz host
read rate (`bench-single-ack-slot-observability-collapses-at-40ms.md`)
showed the 115-005 single-slot design's own "rare at bench rates"
assumption no longer holds: `move_protocol_bench.py` lost 12/43 checks,
every miss a transient enqueue/STOP/CONFIG ack overwritten before the
host's next read. `Telemetry::ack(corrId, errCode)` now pushes onto BOTH
the pre-120 scalar pair (`ackCorr_`/`ackErr_`, UNCHANGED behavior — see
below) AND a small, bounded ring (`ackRing_[kAckRingDepth]`, depth 4,
`telemetry.h`) — a plain circular buffer (`pushAckRing()`,
`telemetry.cpp`): while the ring has spare capacity the new entry lands
in the next free slot; once full, the new entry overwrites the OLDEST
slot and the head pointer advances, so only the single oldest entry is
ever evicted, never a mid-ring one. `emitPrimary()` serializes the ring's
CURRENT contents (oldest-to-newest) into the new, additive wire field
(`telemetry.proto` field 14, `repeated AckEntry acks`) every call — the
same "last staged snapshot, not a diff" contract every other `Frame`
field already has (§3's own invariant, extended here): a ring entry
persists across emits with no new `ack()` call, it is not cleared after
being sent once.

`ack_corr`/`ack_err` (the pre-120 scalar pair) and `flags` bit 5
(`kFlagAckFresh`) keep their EXACT prior meaning — "the freshest ack" —
for any reader that never looked past them; the ring is purely additive,
so no existing host consumer needs to change to keep working. A command
acked within the same primary period as 4 OTHER commands still overwrites
the ring's oldest entry (a saturated-ring tradeoff, not the old
single-slot tradeoff). `flags` bit 5 remains a ONE-SHOT pulse Telemetry
tracks internally: true on the very next `emitPrimary()` call after ANY
`ack()` push since the last emit, then cleared — this pulse governs ONLY
the scalar pair; no equivalent freshness bit exists (or is needed) for
the ring, since a ring entry is either genuinely present (a real,
once-pushed ack) or not there at all — there is no "stale leftover value"
ambiguity for a repeated field the way there is for a persisting scalar
pair.

Wire-size consequence (AS OF 120, pre-123 base64-armored budget):
`Telemetry` standalone grows from 144 B to a worst-case 179 B (a full
4-entry ring, each entry at its own declared bound — `corr_id` up to
65535, `err` up to 7); wrapped as `ReplyEnvelope.body`'s `tlm` arm, the
whole envelope's worst case grows from 153 B to **185 B**, exactly 1 B
under the then-186-byte envelope budget — the tightest margin in the
schema at that time.

**AS OF 123 (historical — superseded by 124, below):** the envelope
budget was recomputed to **240 bytes** (COBS+CRC replacing base64 armor,
see §4's own `cycle_busy`/`cycle_period` note above and
`messages/DESIGN.md` §3), and ticket 004 additively migrated
`cycle_busy`/`cycle_period` (fields 15/16) onto this same frame:
`Telemetry` standalone was **188 B**, wrapped `ReplyEnvelope` total
**194 B** — **46 B** margin under the 240-byte budget, restored from the
pre-123 1-B margin.

**AS OF 124 (current) — the scalar ack slot is DELETED; the ring is the
SOLE ack path; fields are packed fixed-point.** Sprint 124 ticket 008
(issue §B4) deletes `ack_corr`/`ack_err` (the pre-120 scalar pair,
`telemetry.proto` fields 5/6, now `reserved` — not reused) and `flags`
bit 5 (`kFlagAckFresh`, now RESERVED — see the flags bit-string paragraph
below): ring membership already means "really acked," so the separate
"freshest ack" scalar and its freshness bit added nothing a ring scan
didn't already have. `Telemetry::ack(corrId, errCode)` pushes ONLY to the
ring now (`pushAckRing()`, `telemetry.cpp`) — no more dual push. Each ring
element is a single packed `uint32_t` (`corr_id << 4 | err`, `wire.cpp`'s
first real `FieldKind::kRepeatedScalar` use), not `msg::AckEntry`
(deleted) — `corr_id` gets the upper 28 bits, `err` the low 4 (`ErrCode`'s
own span tops out at `ERR_NOT_CONFIGURED`=8). Ticket 008 also switches
`EncoderReading.position`/`velocity`, `OtosReading`'s six fields, and
`Pose2D`/`BodyTwist3`'s fields from `float` to `sint32` + a GENERATED
`(scale)` conversion (zigzag-encoded, options.proto) — a negative value
now costs the varint width of its magnitude, not a sign-extended 10 B —
and renames `EncoderReading.time`/`OtosReading.time` to `age` (an
absolute clock value could never be packed small; `age` is the delta
behind `Telemetry.now`, bounded to 255 ms). `EncoderReading.position_epoch`
(ADDITIVE, field 4) is the position-rebaseline policy's own counter
(sprint 124 architecture Decision 6) — see §2's own "Position-rebaseline
policy" note and `robot_state.h`'s own `Wheel::positionEpoch` doc comment.
Ticket 009 further wires `age` to genuine independent per-sample capture
skew (`Devices::Motor::sampleTime()`/`Devices::Otos::sampleTime()`, ticket
002's enabling change) in place of 008's honest-zero placeholder, and folds
`TelemetrySecondary`'s deletion in (see this section's own paragraph
above). The combined effect: `Telemetry` standalone/wrapped
`ReplyEnvelope` shrinks to **130 B** (`wire.h`'s own regenerated
`kReplyEnvelopeMaxEncodedSize` constant and static_assert — sprint 124's
own ≤130 B gate) despite the ADDITIVE `position_epoch` field, comfortably
under the 240-byte envelope budget. See
[`docs/protocol-v5.md`](../../../docs/protocol-v5.md) §8 for the full
wire-level field table and flags bit reference (supersedes the retired
`docs/protocol-v4.md`).

Host-side matcher (Architecture Step 7's open question, resolved):
`SerialConnection.wait_for_ack()`/`NezhaProtocol.wait_for_ack()`
(`src/host/robot_radio/io/serial_conn.py`,
`src/host/robot_radio/robot/protocol.py`) scan the ring (via
`_match_ack_in_frames()`), not a scalar slot — returning on the FIRST
(frame, ring-entry) match found, scanning frames in arrival order and,
within a frame, ring entries in wire order (oldest first). No freshness
check applies to a ring scan. `TLMFrame.acks` exposes the full decoded
ring to any caller that wants to inspect it directly (bench scripts,
`tlm_log.py`) — since 124-008, this is the ONLY ack-observability field:
`TLMFrame.ack`/`ack_corr`/`ack_err`/`ack_fresh` are DELETED host-side too
(the wire fields they read no longer exist), not merely unused.

**Hardware verification (2026-07-23, robot "tovez",
`/dev/cu.usbmodem2121102`).** The ring itself is proven solid on real
hardware: a dedicated rapid-fire N=5 back-to-back `move_twist()` enqueue
test (`src/tests/bench/ack_ring_rapid_fire_bench.py`) passed all 5/5
ack-observability checks on 3 separate runs (15/15 total), and
`twist_drive.py`'s previously-always-missed `stop()` ack landed cleanly
whenever the command itself reached the firmware. `move_protocol_bench.py`
did NOT reach a clean 43/43 in this session (repeated runs: 38, 34, 33,
30, 35 out of 43) — root-cause isolated via an A/B test against the
UNMODIFIED pre-120 firmware+host code (commit `047555a5`, built in a
throwaway `git worktree`), which showed the IDENTICAL failure signature
(ack=None AND zero encoder movement — the envelope itself never reaching
`RobotLoop::processMessage()`, not an ack-ring miss) at a similar rate.
This is a pre-existing, out-of-scope bench-link reliability gap, filed as
`bench-move-commands-intermittently-never-reach-firmware.md` — NOT a
defect in the ack ring, which the isolated rapid-fire/twist_drive
evidence above shows working exactly as designed whenever the underlying
command actually arrives.

**The `flags` bit-string (115-005 — replaces the old separate
`fault_bits`/`event_bits`/nine-bool frame).** ONE `uint32` carries every
status/fault/event/presence bit: bit 0 `kFlagOtosPresent` (OtosReading
fresh THIS frame — chip detected AND this cycle's burst actually
refreshed the cached pose, NOT the old pre-115 "chip ever detected"
semantic), bit 1 `kFlagOtosConnected` (live bus health), bit 2 `kFlagActive`
(motion in progress), bits 3/4 `kFlagConnLeft`/`kFlagConnRight` (motor bus
connectivity), bit 5 RESERVED (124-008: formerly `kFlagAckFresh` — deleted
along with the scalar ack slot it gated; ring membership already means
"really acked," see this section's own "AS OF 124" paragraph above), bit 6
`kFlagFaultI2CSafetyNet` (`I2CBus::clearanceSafetyNetCount() > 0` —
**120-003, CONFIRMED via pyOCD/DBG trace against real hardware,
2026-07-23** (robot "tovez", `/dev/cu.usbmodem2121102`): this is a
CONTINUOUSLY LIVE, monotonically
growing counter, NOT a boot-time one-shot latch — the prior DRAFT text
here (and 118-001's own acceptance claim) was wrong, falsified by direct
measurement. Raw `clearanceSafetyNetCount_` was sampled via a halted
SWD read at multiple points: ~4.6s post-flash (already 97, not 1), ~8s
post an SWD-triggered reset (167), and across two independent idle-window
brackets (Δ243 over ~14s; Δ148 over ~8.6s) — in BOTH brackets the delta
matches, EXACTLY, half of `Devices::Otos`'s own per-device `txnCount`
delta (Δ486→243 bursts; Δ296→148 bursts), i.e. one trip per Otos burst
read, 1:1, with zero residual attributable to either motor. Root cause:
`Devices::Otos::readPositionVelocity()` (and its sibling register
helpers, `readReg8()`/`readXYH()`) issue a register-select `write()`
immediately followed by a `read()` on the SAME device with NO
intervening loop-scheduled gap (unlike `NezhaMotor`'s split-phase
`requestEncoder()`/`collectEncoder()`, which DOES cross a real
`kSettle`-scheduled gap) — so `waitForClearance()` trips on every single
Otos burst read, unconditionally, at Otos's own ~20ms read cadence,
regardless of `moveQueue_.active()` (`Otos::tick()` runs every cycle
either way). This is entirely unrelated to 118-001's loop-schedule
restore, which is confirmed FULLY EFFECTIVE for its actual target: the
motor's own split-phase path contributes ZERO measured trips in either
an idle or a driving window. No code fix ships this ticket — making the
bit literally clear during driving would require either redesigning
`Otos`'s own I2C register-read pattern (a real hardware-timing change to
a currently-working, bench-proven sensor path, out of this ticket's
authorized file scope) or introducing a caller-intent exemption into the
safety-net counting logic (a stakeholder-level policy decision about
what counts as a "fault," not something to guess after 118-001 already
guessed wrong once) — see
`clasi/issues/i2c-safety-net-bit-conflates-otos-settle-wait-with-loop-schedule-health.md`
for the candidate fix options filed for a future sprint. 118-001's own
acceptance record (`clasi/sprints/done/118-loop-schedule-truth-firmware-loop-reorder-sim-cadence-parity/`)
is corrected to cite this trace.), bit 7
`kFlagFaultWedgeLatch` (`motorL_.wedged() || motorR_.wedged()`), bit 8
`kFlagFaultI2CNak` (declared, not yet wired — no per-transaction NAK
aggregate exists yet), bit 9 `kFlagFaultCommsMalformed`
(`Comms::malformedCount() > 0`), bit 10 `kFlagEventDeadmanExpired` (116:
ORPHANED — its producer, `Deadman::expired()`, was deleted along with
`App::Deadman`; nothing sets this bit any more, see §6), bit 11 RESERVED
(125-002, telemetry-emit-policy-rebuild-spec.md Part 1 item 8: formerly the
boot-ready event bit, `Preamble::done()`'s first-true transition — deleted
outright, not merely orphaned like bit 10: it was a one-shot latch whose
transition was unobservable by construction, and the `READY` cleartext line
already announces boot completion), bit 12
`kFlagEventConfigApplied` (declared, not yet wired), bits 13/14
`kFlagLinePresent`/`kFlagColorPresent` (see §2's line/color polling note),
bit 15 `kFlagFaultMoveTimeout` (116: wired — set on the cycle an active
`Move` ends via `timeout` rather than its kind-specific stop condition),
bit 16 `kFlagFaultShapingDisabled` (119 ticket 001,
kill-the-silent-off-shaping-config-boundary.md: set on every cycle a `Move`
is active AND `MoveQueue::shapingDisabled()` — both linear and angular
`ShaperLimits` axes disabled — mirroring `shapeAndStage()`'s own
early-return gate exactly, so the bit tracks precisely the regime where the
land-at-zero completion path can never fire and the threshold/timeout
backstop is the ONLY completion path; the loud off-state for a
20x-turn-accuracy-delta feature that used to have a silent, invisible off
state), bit 17 `kFlagFaultPositionClamped` (124-008, Decision 6's
"bound-exceeded fallback": a wheel's position was clamped to
`EncoderReading.position`'s own `(abs_max)` at the encode step rather than
allowed to wrap — not the expected path, since `RobotLoop`'s own per-cycle
rebaseline trigger at a 2000mm margin below the bound should prevent this
in normal operation; purely observable evidence the defensive fallback
engaged. Derived from `Types::RobotState::Health::positionClamped` inside
`Telemetry::update()`, 124-009).
Declaring a
bit before it is wired is deliberate — it reserves the bit number for a
future caller without renumbering. As of 124-009, `RobotLoop` never calls
a flag-setting method directly at all (grep-enforceable:
`grep setFlag src/firm/app/robot_loop.cpp` returns nothing) — `Telemetry::
update(state)` derives every bit it can know from `Types::RobotState` in
one place (superseding the former "`RobotLoop` assembles every bit ...
via `Telemetry::setFlag(bit, active)` at the point in the cycle each
condition becomes known" call-site-scattered pattern this paragraph used
to describe); `kFlagFaultMoveTimeout`/`kFlagFaultShapingDisabled` are the
one exception, set via `Telemetry::setLiveFlag(bit, active)` after
`MoveQueue::tick()` runs (their own defining condition isn't known at
`update()` time — see `telemetry.h`'s own doc comment).

**Boot contract.** `Preamble::step()` advances at most one not-yet-resolved
device's own detection entry point per call, never sleeps, and never
touches the bus more than that one call's leaf. `RobotLoop::boot()` owns
the pacing sleep between `step()` calls and emits a boot telemetry frame
each pass, so a host watching the wire can distinguish "still booting" from
"dead" well before the command loop starts. A power-settle wait
(`kPowerSettle`, ported unchanged from the retired `DeviceBus`) blocks the
very first probe from racing the rails on power-up — it exists because the
very first device probed (a motor's `begin()`) has no retry pacing of its
own to lean on. A wall-clock defensive bound (`kMaxPreamble`) forces every
remaining slot terminal if a leaf's own detection never resolves; this is a
safety net against a future leaf regression, not the primary termination
path (every slot already self-bounds its own retry count given step() is
called with real elapsed time between calls).

## 5. Interfaces

### Exposes

- **`RobotLoop::run()` / `boot()` / `cycle()`:** `run()` never returns —
  `boot()` once, then `cycle()` forever. `boot()`/`cycle()` are exposed
  separately so a host harness can step a bounded number of cycles and
  inspect state between them; `cycle()` assumes every device already
  resolved from a prior `boot()` (no readiness checks inside it).
- **`Comms::pump(Cmd& out, uint32_t now)`:** non-blocking, reads at most one
  `\n`-terminated line per call across both transports (124-005 — a plain
  `bool` from `Transport::readLine()`, no more `FrameKind`), parses its
  `<COMMAND>` prefix and dispatches by the generated command registry's
  `binary` flag to either a binary decode (into `out`) or a cleartext
  reply (`HELLO`/`PING`/`ID`/`VER`, answered inline, never populating
  `out`); resets `out.status` to `kNone` at entry so a caller never sees
  stale decode state. `now` ([ms], the caller's own current clock reading)
  stamps `PONG:t=<ms>`'s field — `Comms` holds no `Devices::Clock&` of its
  own.
- **`Comms::sendReply(const msg::ReplyEnvelope&)`:** derives the outbound
  wire verb (`TLM`/`OK`/`ERR`) from `reply.body_kind` (124-005 — no
  separate command parameter; see §1's own "124-005" note), encodes,
  CRC-then-COBS frames with that verb as both the CRC's scope-extension
  input and the wire line's own `<COMMAND>':'` prefix (123-002/124-003 —
  base64-armors before 123), and broadcasts on both transports via the
  async/drop-on-full send path — never blocks the loop on backpressure.
- **`Telemetry::update(state)`/`setLiveFlag`/`ack`/`emit(now)`** (124-009,
  replacing `setFrame`/the public `setFlag`): `update(state)` is the ONE
  assembly-point call — cheap, but called EXACTLY ONCE per cycle (unlike
  the old `setFrame`/scattered-`setFlag` shape, which had no such bound);
  `setLiveFlag`/`ack` remain cheap and callable any number of times per
  cycle; `emit(now)` is the one call that actually sends — always the
  single primary frame now (`TelemetrySecondary` is deleted, so there is
  no "at most one of two frame types" choice left to make), bounded work,
  never sleeps, never touches the I2C bus. See §4's own "RobotState
  assembly + `Telemetry::update()`/`emit()`" paragraph above.
- **`Telemetry::applyAction(Comms::TlmAction)`** (128-012): returns
  whether the caller's next `emit()` call should force a frame; also the
  ONE place `mode_` changes in response to a wire `TLM:` command. See
  §4's own "TLM action application + STATUS projection" paragraph above.
- **`Comms::updateStatus(state, tlm)`** (128-012): refreshes the snapshot
  `sendStatus()` formats from, in one call — replaces the loop's own
  former field-by-field `Comms::Status` assembly. See the same §4
  paragraph.
- **`Drive::setDuty(left, right)`/`stop()`/`tick()`:** (122-002,
  NARROWED — `setTwist()` is GONE, moved to `Motion::MoveQueue`, which now
  calls `BodyKinematics::inverse()` itself and hands `Drive` an
  already-decomposed wheel-target pair through the `Motion::WheelSink`
  boundary `Drive` implements; see
  [`src/motion/DESIGN.md`](../../motion/DESIGN.md).) `setDuty()` only
  STAGES a target; `tick()` stages the last `setDuty()`/`stop()` target
  onto the two motor leaves via their own `setVelocity()` — it never calls
  a motor's own `tick()`, never touches the bus, never sleeps. `stop()`
  stages zero. `Drive` depends on nothing but `Devices::Motor` now — it
  lost its `BodyKinematics` dependency along with `setTwist()` (122-002);
  see `drive.h`'s own doc comment for the exact current contract. (Pre-122
  history: through sprint 121, `Drive::setTwist(v_x, v_y, omega)` was a
  second staging path computing wheel velocities via
  `BodyKinematics::inverse()` internally, and had already lost its
  acceleration-feedforward term at 115-005 — both `setTwist()` and that
  internal `BodyKinematics::inverse()` call moved to `Motion::MoveQueue`
  at 122-002, not merely deleted.) **125-002:** `Motion::WheelSink`'s own
  boundary retooled from a velocity sink to a duty sink — `setWheels()`
  renamed `setDuty()`, `[-1,1]` per the base contract's plausibility bound
  (unenforced here yet — a placeholder, unclamped pass-through; ticket 007
  adds the real `|duty|<=1`/NaN→0 clamp and `App::WheelObserver` wiring).
- **`Motion::Odometry::integrate(leftPosition, rightPosition)`/
  `pathLength()`:** (122-002, MOVED to `src/motion/` — see
  [`src/motion/DESIGN.md`](../../motion/DESIGN.md) for the current,
  exact contract; no longer holds a `Devices::Motor&`, takes the
  caller's current wheel positions as plain float parameters instead.)
  `integrate()` — call once per cycle, after both motors' own `tick()`
  has run that cycle; the caller (`RobotLoop`) reads each leaf's current
  `position()` and passes both floats in; `Odometry` accumulates world
  pose via midpoint-arc integration over `BodyKinematics::forward()`'s
  per-cycle body-frame delta. `pathLength()` (116) is a read-only
  accessor over a running total of `|distance|` that `integrate()`
  already computes internally each cycle — the DISTANCE stop-condition's
  source of truth.
- **`otos_.tick(nowUs)` + inline `state_.otos.*` publish (124-009,
  replacing the deleted `applyOtosSample(otos, now, frame)` free
  function):** safe to call every cycle — a too-soon call given OTOS's own
  internal rate limit is already a documented no-bus-traffic no-op.
  `RobotLoop::cycle()` itself now carries the FULL reading (x, y, heading,
  v_x, v_y, omega, `sampleTime()`) into `state_.otos` inline, only when
  `present` — see §2's own "Otos call site" paragraph. Must not be called
  from inside a motor request→collect window (bus-discipline is the
  loop's job).
- **`RobotLoop::updateLineColor(nowUs)`:** private, called once per cycle
  from the `kPace` block — see §2's own doc comment for the full contract.
- **`Motion::MoveQueue::enqueue(move, now)`/`tick(now, odom)`/`flush()`/
  `active()`:** (116, MOVED to `src/motion/` at 122-002 — see
  [`src/motion/DESIGN.md`](../../motion/DESIGN.md) and `move_queue.h`'s
  own doc comment for the current, exact signatures; `MoveQueue` no
  longer holds a concrete `App::Drive&` or a `Devices::Clock&` — it holds
  a `Motion::WheelSink&` and takes `now` as an explicit parameter
  instead.) `enqueue()` applies `replace`/enqueue semantics (`ERR_FULL`
  past 4 pending) and, for the newly-active slot, computes wheel targets
  (`BodyKinematics::inverse()`, for a twist-kind `Move` — 122-002 moved
  this call here from `Drive::setTwist()`) and stages them onto the
  `WheelSink` boundary, capturing the `Motion::StopCondition` baseline;
  `tick()` advances the active `Move`'s `StopCondition`, hands off to the
  next pending `Move` on stop/timeout (same cycle, no motion gap), and
  calls the sink's `stop()` when the queue drains empty; `flush()` clears
  every pending slot without disturbing the active one (used by `STOP`).
  `active()` reports whether a `Move` is currently in progress (feeds
  `frame_.mode`/`driving_`).
- **`Preamble::step()`/`done()`/per-device status accessors:** `step()`
  never blocks; `done()` is true once every device has reached a terminal
  state (present-and-ready or confirmed-absent).
- **`Motion::StateEstimator::update(input, now)`/`wheelAt(wheel, t)`/
  `bodyAt(t)`/`whereAmI(now)`/`wheelNow(wheel)`/`reset(x, y, heading)`/
  `innovations()`/`setWeights(weights)`** (117, MOVED to `src/motion/` at
  122-002 — see [`src/motion/DESIGN.md`](../../motion/DESIGN.md) and
  `state_estimator.h`'s own doc comment for the current, exact contract;
  `update()` now takes a plain `Motion::StateEstimator::Input` struct
  instead of `App::Telemetry::Frame`, since this tree may not depend on
  `app/` — `RobotLoop` copies the same fields it always read off
  `frame_` into an `Input` and passes that instead, values unchanged):
  `update()` — call once per cycle from the trailing `kPace` block, after
  `frame_.pose` is staged; refreshes both wheel peers' and the body
  peer's basis. `wheelAt`/`bodyAt` — pure ZOH extrapolation from the
  current basis to an explicit query time `t`; no owned clock, hand-fed
  `t` always, mirroring `Motion::StopCondition`'s own testability shape.
  `whereAmI(now)` is exactly `bodyAt(now)`; `wheelNow(wheel)` returns the
  wheel's raw basis with no extrapolation. `reset(x, y, heading)`
  re-anchors the body peer's world pose only (wheel peers are untouched —
  they track per-wheel distance, not world pose, the same reasoning
  `Motion::Odometry::pathLength()` is untouched by `Odometry::reset()`).
  `innovations()` returns the most recent OTOS-vs-predicted heading/omega
  residual — computed for diagnostic/validation purposes even while its
  fusion weight is 0, never fed back into the estimate itself at that
  weight. `setWeights()` is `RobotLoop::handleConfig()`'s own entry point
  for a live `EstimatorConfigPatch` (§3 above) — a plain in-memory update,
  not a bus transaction. All of the above are pure computation: no I2C
  access, no sleep, bounded per call.

### Consumes

- **`Devices::NezhaMotor`, `Devices::Otos`, `Devices::ColorSensorLeaf`,
  `Devices::LineSensorLeaf`, `Devices::I2CBus`, `Devices::Clock`,
  `Devices::Sleeper`:** the device leaves and time/bus seams `app/` drives
  — see [devices/DESIGN.md](../devices/DESIGN.md).
- **`BodyKinematics::inverse()`/`forward()`:** stateless twist↔wheel math
  — moved to `src/motion/body_kinematics.{h,cpp}` at 122-001 (from
  `src/firm/kinematics/`); `Motion::MoveQueue` now calls `inverse()`
  directly (122-002 — the twist-decomposition call moved out of
  `Drive::setTwist()`, see above), while `RobotLoop::cycle()` still calls
  `forward()` directly here in `app/` to fuse the two leaves' measured
  velocities into `state_.pose.v_x/v_y/omega` (124-009 — formerly
  `frame_.twist`/`updateTlm()`; `Drive::trackWidth()`'s own doc comment).
  See [`src/motion/DESIGN.md`](../../motion/DESIGN.md) and
  [`src/firm/kinematics/DESIGN.md`](../kinematics/DESIGN.md) (retired,
  redirects to the current doc) for the full derivation.
- **`msg::CommandEnvelope`/`ReplyEnvelope`/`Telemetry`,
  `msg::wire::encode`/`decode`, `WireRuntime::cobsEncode`/`Decode`,
  `crcCompute`/`Verify` (123 — the current framing primitives;
  `WireRuntime::base64Encode`/`Decode` is retained but no longer called by
  `app/`, see `messages/DESIGN.md` §3):** the wire schema and codec — see
  [messages/DESIGN.md](../messages/DESIGN.md). `msg::TelemetrySecondary`
  is DELETED outright (124-009, robot-state-blackboard-...md) — there is
  no second top-level wire message any more.
- **`SerialPort`, `Radio` (ARM builds only):** the two real transports
  `SerialTransport`/`RadioTransport` adapt into `app::Transport` — see
  [com/DESIGN.md](../com/DESIGN.md).
- **`Motion::StopCondition`** (116, moved to `src/motion/stop_condition.h`
  at 122-001, from `src/firm/motion/`): the bounded-motion stop/timeout
  comparison `Motion::MoveQueue` owns and drives per active `Move` — see
  [`src/motion/DESIGN.md`](../../motion/DESIGN.md) and
  [`src/firm/motion/DESIGN.md`](../motion/DESIGN.md) (retired, redirects
  to the current doc) for the full derivation. This is NOT a revival of
  the deleted `Motion::Executor`/`Motion::Cmd`/`Motion::fromMove()`
  (115-005, still gone) — `src/motion/` contains only the modules listed
  in [`src/motion/DESIGN.md`](../../motion/DESIGN.md) §2, mirroring the
  original tiny `src/firm/motion/`'s own small-pure-comparison pattern at
  a new, sibling-tree location.
- **`Types::RobotState`** (124-007/009, `src/firm/types/robot_state.h`,
  superseding this entry's former `Telemetry::Frame`/hand-copied
  `Motion::StateEstimator::Input` description): the dependency-free
  blackboard `RobotLoop` publishes each cycle and both
  `Motion::StateEstimator::update(state_, now)` and
  `Telemetry::update(state_)` read directly — `Motion::StateEstimator::
  Input` is now a plain alias onto `Types::RobotState` (ticket 007), so
  there is no hand-copied intermediate struct left at all (122-002's own
  "StateEstimator may not depend on `app/`'s `Telemetry::Frame` type"
  constraint is satisfied differently now: `RobotState` lives in
  `src/firm/types/`, a dependency-free header both `src/firm/app` and
  `src/motion` may include, not an `app/`-owned type). `StateEstimator`
  still does not hold its own leaf/bus references and does not read
  `Devices::Motor`/`Devices::Otos` directly either way. Wire-plane
  `msg::EstimatorConfigPatch` stops
  at `RobotLoop::handleConfig()` exactly like `msg::MotorConfigPatch`/
  `msg::OtosConfigPatch` already do (devices/app isolation invariant
  above, extended by analogy) — `StateEstimator`'s own `setWeights()`
  takes a plain, Devices-local-style weights struct, never a `msg::*`
  type.
- **`Config::defaultEstimatorConfig()`** (117, `config/boot_config.h`):
  fail-closed baked fusion-weight defaults (`weight_heading_otos =
  weight_omega_otos = 0.0` this sprint, `staleness_ms`), constructed once
  at boot in `main.cpp` and passed to `StateEstimator`'s constructor —
  see [config/DESIGN.md](../config/DESIGN.md).

## 6. Open Questions / Known Limitations

- **`MotorConfigPatch` and `OtosConfigPatch` (109-004) are live-appliable.**
  Only `DrivetrainConfigPatch`/`WatchdogConfigPatch` still reply
  `ERR_UNIMPLEMENTED` (no on-robot fusion consumer for the former; the
  latter routes to `bb.streamWatchdogWindowIn` directly, not
  `handleConfig`, per config.proto's own `CONFIG_WATCHDOG` comment); see
  §3. `PlannerConfigPatch` is not a third "still unimplemented" case — it
  no longer exists as a type at all (115-005).
- **In-session pose reset has no wire verb yet.** `Odometry::reset()`
  exists and is exercised by the host simulator's teleport-to-origin, but
  no binary command arms it from the wire today.
- **`kFlagFaultI2CNak` (bit 8) and `kFlagEventConfigApplied` (bit 12) are
  declared but unwired** — reserved bit numbers with no live producer yet.
  `kFlagFaultMoveTimeout` (bit 15) is now wired (116) — set on the cycle
  an active `Move` ends via `timeout` rather than its stop condition.
- **`kFlagEventDeadmanExpired` (bit 10) is orphaned by 116, not
  reassigned.** Its sole producer, `Deadman::expired()`, was deleted along
  with `App::Deadman`; the bit constant still exists in `telemetry.h` (no
  wire-shape change) but nothing in the tree calls
  `Telemetry::setFlag(kFlagEventDeadmanExpired, ...)` any more, so it now
  reads permanently 0. Left as declared-dead rather than deleted or
  repurposed — this sprint's scope did not include a `flags` wire-shape
  change, and reassigning a bit number to a new meaning without a version
  signal would be a silent protocol break for any reader still checking
  it. Whether to formally delete or repurpose this bit is open for a
  future sprint.
- **The pre-115 heading-PD/distance-trim/measurement-age-projection design
  history (formerly documented at length in this file's own §2, plus
  `motion/DESIGN.md`) is not carried forward here.** `Pilot`/
  `Motion::Executor`/`HeadingSource` and everything they computed
  (`heading_kp`/`heading_kd` cascade, `distance_kp` trim, `kDeadTime`
  divergence-replan projection, `HeadingSource::headingLead()`) are deleted
  wholesale by 115-005 — the git tag `pre-gut-motion-stack` is the
  authoritative historical record if that design work is ever revisited,
  not a summary re-derived from memory here.
- **`StateEstimator`'s predictions are not exposed on the wire (117).**
  Neither `msg::Telemetry` nor `msg::TelemetrySecondary` gained a field
  for `whereAmI()`/`wheelNow()` output this sprint — validation runs
  host-side against the raw `EncoderReading`/`OtosReading` fields already
  telemetered (sprint 115), replaying the identical ZOH math in Python
  over a captured TLM-log CSV. A future on-robot consumer (the
  remaining-distance trajectory controller) will need `whereAmI()`
  results live, in-process — that consumer calls the estimator directly
  (same process, same cycle), not over the wire, so this gap may never
  need closing; flagged as open only because it was an explicit sizing
  choice, not an oversight.
- **`EstimatorConfigPatch`-set fusion weights are volatile, not
  persisted.** Unlike `MotorConfigPatch`/`OtosConfigPatch` (114-004),
  a live-tuned weight does not survive a reboot — it reverts to the
  baked JSON default. Revisit once fake-OTOS/external-pose fusion
  (future sprints) give these weights real, nonzero, bench-validated
  values worth persisting.
