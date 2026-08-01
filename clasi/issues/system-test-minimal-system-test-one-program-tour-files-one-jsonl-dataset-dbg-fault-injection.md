---
status: pending
extends: square-tour-is-the-one-system-test-sim-bench-playfield.md
---

# Minimal system test: one program, tour files, one JSONL dataset, DBG fault injection

## Description

Stakeholder directive (2026-07-31): the whole system test is **one program**
plus a directory of **tour files** — plain-text scripts of moves, "a text
version of the protocol." A run produces **one dataset file**: every line the
robot/sim emitted (binary telemetry AND debug/cleartext) as JSON records, plus
everything the host sent. Analysis, plotting, and golden-image comparison
operate on that dataset afterward. Additionally, a new family of **`dbg` wire
commands** sent down to the firmware/sim for fault injection (wedge an
encoder, etc.).

This issue **extends** the umbrella charter
[`square-tour-is-the-one-system-test-sim-bench-playfield.md`](square-tour-is-the-one-system-test-sim-bench-playfield.md)
(2026-07-28: one system test, three tiers, square with two planned stops on
opposite legs, closure gate, circle gated behind closure, golden traces
one-signal-per-image). That issue remains the charter — the what and why;
this one adds the concrete mechanism: tour files, the one program, the JSONL
dataset, DBG fault injection, in-tour assertions (`EXPECT`), and camera
position validation (`CAMFIX`).

## Cause

Today the "system test" is a scatter of overlapping standalone scripts
(`src/tests/bench/square_tour.py`, `wheels_square_tour.py`,
`planner_square_tour.py`, the `src/tests/sim/system/` sweep), each hardcoding
its own move sequence, its own capture format (CSV, PNG, stdout), and its own
backend split. Nothing captures the complete wire stream (telemetry + debug +
cleartext + sent commands) in one analyzable file; the golden-trace library
exists but has zero callers; and there is no way to induce a fault (wedge,
frozen encoder, dead sensor) from the wire on either sim or hardware —
firmware can only detect faults, never cause them.

Key exploration findings (2026-07-31) — most of this is assembly, not
construction:

- `src/host/robot_radio/planner/tour.py`: `run_tour()` engine with one-leg
  lookahead, ack-ring completion keyed on `Move.id`, `TourClosure`. Its
  `MoveTransport` protocol is satisfied by both `NezhaProtocol` and `SimLoop`.
  Known bug to fix in passing: its `should_stop` path calls planned `stop()`
  instead of `estop()`.
- `NezhaProtocol(SimConfigConn(SimLoop(...)))` already works
  (square_tour.py:416) — the same protocol client drives sim and hardware.
- `src/tests/tools/golden_trace.py`: a complete, unit-tested, zero-caller
  golden library — pinned-axis `PlotSpec`, widened-band mask, analytic gate
  (`outside_count`/`max_deviation`/`rms`), `suggest_tolerance()` from N runs,
  save/load/render. Needs wiring, not writing.
- `DBG` is already a registered protocol-v5 cleartext verb (robot→host printf
  channel, `App::debugf()`, compiled out unless `ROBOT_DEBUG`; sim auto-defines
  it). Making it bidirectional needs zero registry/codegen churn and is
  typeable from a bare serial monitor.
- Faults induced at the firmware layer (not the SimPlant physics layer) behave
  identically in sim and on hardware, because the sim runs the real firmware —
  aligned with
  [`unify-sim-and-robot-composition-roots.md`](unify-sim-and-robot-composition-roots.md).

## Proposed fix

### File layout

```
src/tests/system/
  systest.py            # THE program: run / plot / compare / bless subcommands
  tourfile.py           # tour-file parser -> typed steps
  recorder.py           # line-tap -> JSONL writer (both directions)
  backends.py           # SimBackend/HardwareBackend, extracted from bench/square_tour.py:339-520
  signals.py            # JSONL -> named series + per-signal PlotSpec
  tours/
    square.tour         # primary: square, two planned stops on opposite legs
    circle.tour         # gated behind square closure
    fault_wedge.tour    # first fault tour
  goldens/<tier>/<tour>/   # <signal>.npz + .png + dataset.jsonl.gz, per tier per tour
out/                    # run outputs: <tour>_<tier>_<timestamp>.jsonl (gitignored)
```

### 1. Tour file grammar

**New grammar mapping ~1:1 onto protocol v5 Move semantics** (not the legacy
`D`/`RT` grammar — that has no stops, no wheels variant, no stop-condition
choice). `run_tour()`'s completion engine is reused via one small
generalization: a new `MoveStep` leg kind carrying explicit Move kwargs,
dispatched straight to `transport.move(**kwargs)`.

Line-oriented; `#` comments; `key=value` args; human units (mm, mm/s, deg, s):

| Directive | Meaning |
|---|---|
| `TWIST vx= [vy=] [omega=] (time=\|dist=\|angle=) [timeout=]` | MoveTwist; exactly one stop condition; timeout defaults 3× expected, floor 2 s |
| `WHEELS left= right= (time=\|dist=\|angle=) [timeout=]` | MoveWheels |
| `STOP [dwell=<s>]` | Planned stop: segment boundary; wait stationary, dwell |
| `DWELL <s>` | Host-side pause, no command sent |
| `DBG <subcmd> [args...]` | Send `DBG:<subcmd> ...` down the wire |
| `MARK <text>` | Sugar for `DBG mark <text>` (in-band annotation) |
| `SEND <verb> [data]` | Send any cleartext wire verb (`SEND STATUS`, `SEND PING`) |
| `EXPECT '<query>' [timeout=<s>]` | Assert a JSONL record matching `<query>` arrives between the previous directive's issue time and the timeout (default 2 s) — see below |
| `CAMFIX x= y= radius= [heading= tol=]` | Take a camera fix (AprilCam) and assert the robot is within `radius` mm of (x, y) — see below |

Non-motion directives are **segment boundaries**: consecutive motion steps
form a segment fed to `run_tour()` as one call, preserving lookahead within a
segment. v1 limitation (documented): `DBG` fires between moves; mid-motion
faults use a duration-ms fault injected just before the next `TWIST`
(`at=<s>` is the future extension point).

`tours/square.tour` (700 mm square, stops mid-leg on legs 1 and 3):

```
MARK leg1a
TWIST vx=150 dist=350 timeout=9
STOP dwell=1.0                      # planned stop #1 (OTOS-at-rest site)
MARK leg1b
TWIST vx=150 dist=350 timeout=9
TWIST omega=45 angle=90 timeout=8
MARK leg2
TWIST vx=150 dist=700 timeout=15
TWIST omega=45 angle=90 timeout=8
MARK leg3a
TWIST vx=150 dist=350 timeout=9
STOP dwell=1.0                      # planned stop #2 (opposite leg)
MARK leg3b
TWIST vx=150 dist=350 timeout=9
TWIST omega=45 angle=90 timeout=8
MARK leg4
TWIST vx=150 dist=700 timeout=15
TWIST omega=45 angle=90 timeout=8
STOP
SEND STATUS
EXPECT '.type=="status" and .payload.ready==true' timeout=1.0
CAMFIX x=0 y=0 radius=50            # closure verified against independent truth
```

`tours/fault_wedge.tour`:

```
MARK baseline
TWIST vx=150 dist=300 timeout=9
STOP dwell=0.5
MARK inject
DBG wedge left 1500                 # latched 1500 ms, spans the next move's start
TWIST vx=150 time=2.0 timeout=8     # time-stopped: completion must not depend on distance
DBG clear
STOP dwell=0.5
MARK recovery
TWIST vx=150 dist=300 timeout=9
STOP
```

Closure tolerances and the square→circle gate live in the runner (per-tier
constants), not the grammar — tour files stay a pure protocol script.

#### `EXPECT` — in-stream assertions (stakeholder, 2026-07-31)

Because **every** response is already a JSONL record — telemetry frames, DBG
echoes, STATUS replies, camera fixes — one assertion primitive covers all of
them. `EXPECT '<query>' [timeout=<s>]` passes iff, between the moment the
**previous directive** was issued and the timeout, at least one record arrives
whose JSON satisfies the query expression. On timeout the tour FAILS at that
line.

- **Query language**: a JSON query expression evaluated per record. Proposed:
  jq syntax (via the `jq` PyPI binding), e.g.
  `.type=="status" and .payload.ready==true and .payload.wedge==false` —
  familiar, expressive over nested payloads and flag-name arrays
  (`.payload.flags | index("FAULT_WEDGE_LATCH")`). Alternative if the
  compiled dependency is unwanted: JMESPath (pure Python, clunkier literals).
  Decide at implementation; the grammar carries an opaque quoted string
  either way.
- **Anchor**: the previous directive's own tx/host record marks the window
  start (`seq_file` ordering, not wall-clock guesswork).
- **Implementation**: the recorder gains an in-process subscription bus — the
  runner's `EXPECT` evaluator consumes the same record stream the JSONL
  writer does, so what you assert on is byte-identical to what lands in the
  dataset. Each evaluation writes its own
  `{"type":"expect","query":…,"ok":…,"matched_seq_file":…}` record.
- **Recorder prerequisite**: cleartext k=v replies (`STATUS:ready=1:…`) are
  parsed into structured `payload` dicts, not left as raw strings, so EXPECT
  can query them.

The canonical smoke pattern:

```
SEND STATUS
EXPECT '.type=="status" and .payload.ready==true and .payload.wedge==false' timeout=1.0
```

and the fault tour's proof becomes declarative:

```
DBG wedge left 1500
TWIST vx=150 time=2.0 timeout=8
EXPECT '.type=="tlm" and (.payload.flags | index("FAULT_WEDGE_LATCH"))' timeout=3.0
DBG clear
EXPECT '.type=="tlm" and (.payload.flags | index("FAULT_WEDGE_LATCH") | not)' timeout=2.0
```

#### `CAMFIX` — camera position validation (stakeholder, 2026-07-31)

`CAMFIX x=<mm> y=<mm> radius=<mm> [heading=<deg> tol=<deg>]` validates the
robot's real position against independent truth:

- **playfield tier**: query AprilCam (tag 100 = robot centre of rotation) per
  the playfield rules — at rest, after the settle dwell, median-of-7 samples
  (`.claude/rules/playfield-testing.md`); check the lights
  (`Switch.GetStatus`) before attributing a missing tag to the camera.
- **sim tier**: the same directive reads the plant's ground-truth pose
  (`SimLoop.get_true_pose()`) — same record shape, same assertion, so square
  tours with CAMFIX lines run unmodified on both tiers.
- **bench tier**: unsupported (the robot cannot translate on the stand); the
  runner refuses to run a CAMFIX-bearing tour at `--tier bench` rather than
  silently skipping.

Every fix writes a `{"type":"camera_fix","payload":{"x":…,"y":…,"heading":…,
"target":{…},"error":…,"ok":…}}` record — so it feeds the golden XY-trace
analysis AND is EXPECT-queryable. Pass/fail: planar distance to (x, y) ≤
radius, and, if `heading=` given, yaw within `tol`. A failed CAMFIX fails the
tour at that line (and the runner `estop()`s).

This is what makes the playfield tier's "camera + odometer validate
independently" requirement (umbrella issue) an executable tour line instead
of a separate script: the square tour ends with `CAMFIX x=0 y=0 radius=50`.

### 2. The program (`systest.py`)

- `run --tier sim|bench|playfield [--port DEV] [--robot-config ...] TOUR...` —
  executes tours in order with gating, one JSONL per run.
- `plot DATASET.jsonl` — per-signal PNGs (human layer + 1-bit comparison layer).
- `compare DATASET.jsonl --goldens goldens/<tier>/<tour>` — analytic + raster
  gates, nonzero exit on fail.
- `bless DATASET.jsonl [--runs ...] --goldens DIR` — manual promote;
  `--runs` feeds `suggest_tolerance()`. Nothing self-blesses.

**Drain architecture: passive `on_recv`/`on_send` line taps, NOT
`row_callback`.** The telemetry queue is single-consumer/destructive;
`run_tour()` stays the one destructive drainer. The recorder observes every
line upstream of the queue:

- rx hardware: `SerialConnection.on_recv` (every line pre-routing) + `on_debug`.
- rx sim: add a symmetric `on_line` observer to `SimLoop`/`SimConfigConn`, and
  fix `drain_debug_lines()` (sim_loop.py:947) dropping non-DBG cleartext
  (banner/READY/STATUS) — required for sim/hardware dataset symmetry.
- tx both: a send-path tap. This is how commanded velocity enters the dataset —
  `cmd_vel` is a permanent wire gap; the tx MOVE record IS the commanded truth.

Runs are bracketed with `tlmOn()`/`tlmOff()` (a parked robot under kAuto emits
nothing); sim no-ops via `backend.enableTelemetry()`.

### 3. JSONL dataset schema

One record per line, both directions. Envelope: `seq_file`, `t_host`, `dir`
(`rx`/`tx`), `transport` (`sim`/`serial`), `plane` (`binary`/`cleartext`),
`verb`, `raw` (base64 for binary, literal for cleartext), `decode_ok`,
`payload` (nested, never flattened), `type`.

- **run_meta** (record 0): tier, tour file + sha256 + full text, robot config +
  sha, firmware `VER`, banner, host git SHA, start time, argv, schema_version.
- **tlm**: flags as raw int AND set-bit-name list (all firmware-defined names
  including bits 17–20, which have no host property today); acks as objects;
  wire-precision floats (not the legacy cdeg/mrad truncation); `cycle_busy`/
  `cycle_period` included; seq wraps at 128 (not 65535 — the host docstring is
  stale).
- **dbg / cleartext**: `{"type":"dbg","verb":"DBG","raw":"DBG:mark leg3a",...}`;
  banner/READY/STATUS get the same shape with their own verb.
- **cmd (tx)**: decoded MOVE/CONFIG/STOP payload with `corr_id` — the join key
  against later ack-ring entries.
- Records with CRC/decode failures still land, `decode_ok: false` with reason —
  a malformed line is data, not a dropped counter.

**Step markers ride the wire**: the runner sends `DBG:mark <label>` down and
firmware echoes it back, so markers appear in the robot's own output stream
with correct ordering relative to telemetry, identically on sim and hardware.
The runner also writes a local tx `step` record at send time; the echo is
authoritative for ordering.

### 4. DBG inbound command set

Wire form `DBG:<subcmd> [args]\n`. Firmware plumbing: widen
`Comms::dispatchCleartext()` to carry dataPtr/dataLen and migrate the existing
TLM special case into it (one contained refactor of `comms.{h,cpp}`); Comms
stages a `DbgAction` drained by `RobotLoop::cycle()` via `takeDbgAction()`,
mirroring `takeTlmAction()`. All handlers behind `ROBOT_DEBUG` (compiled out
of shipped images; hardware test images via `build.py --debug`). Remove the
dead pre-v5 `"DBG OTOS BENCH 1"` senders (testkit/target.py:166,
testgui/__main__.py:3125, 3212) so they cannot hit the new parser as garbage.

| Command | Syntax | Semantics | Layer | Provable in telemetry |
|---|---|---|---|---|
| mark | `DBG:mark <text>` | echo back; no state | firmware (Comms) | echo record in rx stream |
| ping | `DBG:ping` | echo `DBG:pong`; inbound self-test | firmware (Comms) | echo record |
| wedge | `DBG:wedge left\|right\|both [ms]` | `forceWedge` override OR-ed into `Devices::MotorArmor` latch; ms auto-clear, 0 = latched until clear | firmware (identical sim/hw) | bit 7 `FAULT_WEDGE_LATCH` |
| freeze | `DBG:freeze left\|right [ms]` | suppress encoder updates in `NezhaMotor` | firmware | bits 19/20 `FAULT_WHEEL_FROZEN_*`, frozen `enc.position` under nonzero command |
| nak | `DBG:nak otos\|left\|right [ms]` | treat device I2C reads as failed at the driver | firmware where feasible | conn bits 1/3/4 drop, `age` climbs, bit 8 |
| clear | `DBG:clear` | clear every injected override | firmware | injected flags drop within one cycle |

Rule: faults that must behave identically across tiers live at the
**firmware** level (sim runs real firmware). Plant-physics perturbations
(dropout rate, encoder scale error, OTOS drift) stay on the existing SimPlant
ctypes API, sim-tier only.

### 5. Golden wiring

`signals.py` extracts per the umbrella issue's list, one signal per image:
`wheel_speed_left/right`, `cmd_vel_left/right` (from tx records → wheel
kinematics), `meas_vel_left/right`, `xy_trace`, `x_t`/`y_t`/`heading_t`,
`cycle_busy_t`/`cycle_period_t`. Pinned `PlotSpec` per signal per tier —
never autoscale. `compare` reports the two AND-NOT raster terms separately
plus the analytic gate; `bless` uses `suggest_tolerance()` from N no-change
runs and stores the blessing dataset beside the images. Goldens are per-tier,
stakeholder-approved, committed with a why.

### 6. Phasing (each independently landable)

1. **Sim runner end-to-end**: `tourfile.py`, `MoveStep` generalization of
   `run_tour()` (+ the `estop()` bug fix), `recorder.py` with sim taps (incl.
   the cleartext-drop fix) and the in-process subscription bus, `EXPECT` and
   `SEND` directives, sim-tier `CAMFIX` (true-pose backed),
   `systest.py run --tier sim`, square/circle tours, closure gate.
2. **Hardware parity**: `backends.py` extraction, `SerialConnection` taps, tx
   tap, tlm bracketing, `--port` bench runs.
3. **DBG inbound + wedge**: `dispatchCleartext` refactor, `DbgAction`,
   mark/ping/wedge/clear handlers, remove legacy DBG senders,
   `fault_wedge.tour` with EXPECT-based flag assertions.
4. **Golden wiring**: `signals.py`, plot/compare/bless, `goldens/`, first
   stakeholder-blessed sim goldens.
5. **Fault set + convergence**: freeze/nak + tours; retire the three legacy
   square drivers and the `sim/system/` sweep per the umbrella issue (walking
   each deleted test's coverage first; keep `test_sim_wire_loopback.py`,
   relocated); record the no-new-system-tests rule.
6. **Playfield tier**: aprilcam-backed `CAMFIX` (median-of-7 at rest, lights
   check first), geofence + `estop()` on failure, the camera-verified square
   as the acceptance run the umbrella issue requires.

### Critical files

- `src/host/robot_radio/planner/tour.py` — `MoveStep` + `estop()` fix
- `src/host/robot_radio/io/sim_loop.py` — `on_line` observer + cleartext-drop fix
- `src/host/robot_radio/io/serial_conn.py` — rx/tx taps
- `src/tests/bench/square_tour.py` — backend extraction source
- `src/tests/tools/golden_trace.py` — reuse whole, unmodified
- `src/firm/app/comms.{h,cpp}` — dispatchCleartext refactor + DbgAction
- `src/firm/app/robot_loop.{h,cpp}` — takeDbgAction() drain
- `src/firm/devices/motor_armor.h`, `src/firm/devices/nezha_motor.cpp` — fault overrides

## Verification

- Phase 1: `uv run python src/tests/system/systest.py run --tier sim
  tours/square.tour` produces a JSONL with run_meta + tlm + dbg + tx records;
  unit tests for `tourfile.py` and `recorder.py` under `src/tests/unit/`.
- Phase 2: same command with `--tier bench --port <dev>` on the stand; diff the
  record-type inventory against the sim dataset.
- Phase 3: `fault_wedge.tour` on sim AND bench; assert the wedge-latch flag
  transition brackets the `mark inject`/`clear` records in the dataset.
- Phase 4: two no-change sim runs → bless → compare (green); perturb a planner
  gain → compare (red).

## Open questions

1. **dbg set**: wedge/freeze/nak/mark/ping/clear proposed; `mark` and `ping`
   piggyback the mechanism for dataset annotation and self-test. Candidates
   considered and parked: `dbg slow <ms>` (inflate cycle time — proves the
   `cycle_period` golden catches schedule regressions), `dbg drop <n>`
   (discard next N inbound commands — exercises the malformed/ack-timeout
   path). Cheap once `DbgAction` plumbing exists.
2. **EXPECT query language**: jq syntax (compiled `jq` PyPI dep, familiar,
   expressive) vs JMESPath (pure Python, clunkier literals). Grammar carries
   an opaque quoted string either way; decide at implementation.
3. **Mid-motion faults**: v1 injects between moves using durations; an
   `at=<s>` arg is the extension if in-flight injection turns out to matter.
4. Per-tier closure tolerances, `CAMFIX` radii, and golden band widths —
   start from measured run-to-run spread (`suggest_tolerance()`), per the
   umbrella issue.

## Related

- [`square-tour-is-the-one-system-test-sim-bench-playfield.md`](square-tour-is-the-one-system-test-sim-bench-playfield.md)
  — the umbrella charter this issue implements (three tiers, golden-trace
  process rules, deletion list).
- [`unify-sim-and-robot-composition-roots.md`](unify-sim-and-robot-composition-roots.md)
  — firmware-level fault injection depends on "sim runs the real firmware,"
  which that issue strengthens.
