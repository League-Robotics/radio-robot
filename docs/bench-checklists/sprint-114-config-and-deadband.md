# Sprint 114 Bench Checklist — Config-as-Truth Completion + Motor Deadband Compensation

> # STAKEHOLDER-RUN — NOT AGENT-EXECUTED
>
> No agent in this sprint has hardware access. Every check below must be run
> by a human at the bench, on the real `tovez` robot mounted on its stand
> (wheels clear of the surface — see
> [`.claude/rules/hardware-bench-testing.md`](../../.claude/rules/hardware-bench-testing.md)).
> This file is sprint 114 ticket 006's deliverable for that standing gate.

## What sprint 114 changed, and why this checklist exists

Sprint 114 (tickets 001–005) made the firmware **refuse to run** until it has
received a complete configuration (no more silent hardcoded fallbacks),
added a version-stamped **on-flash persisted live-tuning store**
(`Config::PersistedTuning`), and fixed the **motor deadband** so a small
terminal correction creeps to completion instead of holding flat. None of
this is testable by an agent:

- The configuration-completeness *refusal* is real only on hardware that was
  actually left unconfigured — every agent-run test uses a compiled sim
  library that is always deliberately configured.
- The persisted-tuning store is backed by `MicroBitStorage` (real on-chip
  flash) — there is no host-build stand-in anywhere in this tree.
- The deadband fix changes how a *real* motor responds to a *real* small
  duty command near the sub-15 mm/s dead zone — a sim plant models this
  numerically, it does not reproduce the actual mechanical/electrical
  stiction the fix compensates for.

## A note on wire commands used below

This sprint's firmware speaks **protocol v3**: a binary `*B<base64(protobuf)>`
command plane (`CommandEnvelope.cmd` oneof — currently exactly four arms:
`config`, `stop`, `twist`, `move`) plus a tiny hand-typeable text rump
(`HELP`/`HELLO`/`PING`/`ID`/`VER`/`STOP`, no `#`/`*B` prefix). **Do not use
`docs/protocol-v2.md`'s `SET`/`GET`/`TURN`/`RT`/`D`/`G`/`DEV` ASCII command
tables directly against the robot's serial/radio port** — that grammar is
superseded (`docs/protocol-v3.md`'s own banner) and, as of this sprint, the
binary wire has **no live config read-back arm at all** (`GET`'s binary
`ConfigGet` arm was pruned; confirmed by reading
`src/protos/envelope.proto`'s current `oneof cmd` directly — only
`config`/`stop`/`twist`/`move` exist). Every command below instead uses the
same host-side library every current bench script
(`src/tests/bench/tour_bench_run.py`, `profiled_motion_verify.py`,
`dev_exercise.py`) and every sim test in this sprint already uses:
`robot_radio.io.repl` (the `rogo repl` CLI, verbs `twist`/`stop`/`config`/
`enc`/`pose`/`otos`/`vel`/`line`/`color`/`tlm`) and
`robot_radio.planner.tour.run_tour()` for closed-loop moves. These are exact,
copy-pasteable commands, not a re-derivation of the wire format.

## 0. Setup

```bash
mbdeploy probe              # discover/refresh the connected device registry
mbdeploy deploy --build     # build sprint 114's firmware and flash tovez
```

Find your serial port (`mbdeploy probe` prints it, or check
`/dev/cu.usbmodem*`). Set it once for this session:

```bash
export ROBOT_PORT=/dev/cu.usbmodem2121102   # replace with your actual port
```

Everything below assumes the robot is **on the stand, wheels clear of the
surface** — safe to drive freely per
[`hardware-bench-testing.md`](../../.claude/rules/hardware-bench-testing.md).

---

## 1. Standing gate (every firmware sprint touching HAL/motor/sensing/protocol)

Open an interactive session:

```bash
uv run rogo --port "$ROBOT_PORT" repl
```

| Step | Type at the `rogo repl` prompt | Expect | Done |
|---|---|---|---|
| 1a | `enc` | `enc [mm] (L,R)` — some finite reading, not an error | [ ] |
| 1b | `otos` | `otos [mm,mm,cdeg]` — a plausible pose reading (OTOS chip answering) | [ ] |
| 1c | `line` | 4 plausible channel values, not all-zero/all-saturated | [ ] |
| 1d | `color` | plausible R,G,B,C values | [ ] |
| 1e | `vel` | `vel [mm/s]` reading at rest — near zero, both wheels | [ ] |
| 1f | `twist 150 0 800` then wait ~1s, `vel` again | wheels spin forward; `vel` shows both L/R climbing toward ~150 mm/s while commanded | [ ] |
| 1g | `stop` | motors audibly stop; `vel` returns to ~0 | [ ] |
| 1h | `twist -150 0 800` | wheels spin the OTHER direction (confirm both directions) | [ ] |
| 1i | `stop` | motors stop again | [ ] |
| 1j | `enc` (compare to step 1a) | both L/R counts changed in the expected direction/proportion to the two twists above (round-trip over the real link confirmed — command sent, telemetry read back) | [ ] |

**Fail (file an issue):** any sensor read errors out or never changes across
a command that should move it; `vel` never climbs toward a commanded twist;
`enc` doesn't change after driving; motors don't stop on `stop`.

Leave this `rogo repl` session running (or reopen it) for the checks below —
`quit` or Ctrl-D to exit when done with a section.

---

## 2. Config-completeness gate: unconfigured device refuses motion (`ERR_NOT_CONFIGURED`)

**This check is NOT reproducible on a stock build, by design — read this
before trying.** `src/firm/main.cpp`'s boot sequence calls
`Config::default*()` (baked at build time from `data/robots/*.json` by
`gen_boot_config.py`) and `robotLoop.markConfigured()` **unconditionally,
before `run()` starts** — the code comment there calls this out explicitly:
*"the boot-configure sequence above ... is atomic and always complete by
this point on real firmware ... no observable startup delay (Decision 2,
sprint.md)."* `gen_boot_config.py` itself refuses to emit a boot config at
all (`sys.exit(1)`) if `data/robots/active_robot.json` doesn't resolve to a
complete robot JSON — so there is no way to *build* a flashable image that
boots into an unconfigured state through the normal `mbdeploy deploy --build`
path.

What this ticket's own agent work already confirmed the gate does, in sim
(cross-check, not a substitute for hardware — reported here so you know what
"the gate works" evidence already exists):

```bash
uv run python -m pytest src/tests/testgui/test_turn_error_characterization.py -v
```

Before sprint 114 ticket 006 added the required `configure_from_robot()`
call to this file's own test harness, every one of these tests hit the
exact failure this section is about: a `SimLoop` left unconfigured refused
every `Move`, immediately, with `RunOutcome.FAULT` after 3 sim ticks — the
sim-side manifestation of the same `RobotLoop::isConfigured()` gate real
firmware enforces (`handleTwist()`/`handleMove()` both refuse with
`ERR_NOT_CONFIGURED` — `src/firm/app/robot_loop.h`/`.cpp`).

| Step | Action | Expect | Done |
|---|---|---|---|
| 2a | (optional, only if you want to see it on real hardware) Locally, temporarily comment out the `robotLoop.markConfigured();` call in `src/firm/main.cpp` (do NOT commit this), `mbdeploy deploy --build`, then `twist 100 0 500` via `rogo repl` | envelope-level reply carries `Error{code: ERR_NOT_CONFIGURED}` (visible via `rogo repl`'s ack line, e.g. `corr_id=N ERR ERR_NOT_CONFIGURED`) instead of the motors moving | [ ] / [ ] N/A (skipped, not reproducible on stock build) |
| 2b | **Revert** the `main.cpp` edit and reflash the real build before continuing to section 3+ | `mbdeploy deploy --build` succeeds, robot back to normal | [ ] |

**Pass:** either 2a is skipped with the "N/A" box checked (expected — this
is normal, not a failure to investigate), or, if attempted, the wire reply
is exactly `ERR_NOT_CONFIGURED` and no motion occurs.

---

## 3. Persisted live-tuning survives a power cycle, wiped on a schema-version reflash

**No live read-back exists** (the binary `ConfigGet` arm was pruned before
this sprint — `robot_radio/robot/protocol.py`'s own header comment on
`_PLANNER_KEYS`: *"There is no live config READ-back path (the binary `get`
arm was pruned by 103-001)"*), so this check is **behavioral**: push an
extreme `headingKp` and watch a turn's response change, rather than reading
the value back.

`headingKp` boot-bakes to `2.5` (`data/robots/tovez_nocal.json`'s
`control.heading_kp`). We'll push something obviously different (`0.3` —
very sluggish/undershooting) and confirm the sluggish behavior survives a
power cycle, then confirm a **schema-version-bumped** reflash wipes it back
to `2.5`'s normal behavior.

### 3.1 Baseline turn (before any push)

Save this as `/tmp/one_leg.py` (or anywhere convenient) — a small, direct
`run_tour()` single-leg driver mirroring `src/tests/bench/tour_bench_run.py`'s
own connection/leg pattern:

```python
#!/usr/bin/env python3
import sys, time
from types import SimpleNamespace
from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol
from robot_radio.planner.heading import HeadingCorrector
from robot_radio.planner.model import PlannerParams
from robot_radio.planner.tour import TourLeg, run_tour

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem2121102"
angle = float(sys.argv[2]) if len(sys.argv) > 2 else 90.0

conn = SerialConnection(port=port)
conn.connect()
time.sleep(2.0)  # boot/preamble settle
proto = NezhaProtocol(conn)
params = PlannerParams()
heading = HeadingCorrector(params, robot_config=SimpleNamespace(
    geometry=SimpleNamespace(otos_untrusted=True)))
t0 = time.monotonic()
try:
    result = run_tour(proto, params, heading, [TourLeg(kind="turn", value=angle)], v_max=150.0)
finally:
    proto.stop()
print(f"outcome={result.stopped_outcome or 'COMPLETED'} elapsed={time.monotonic()-t0:.2f}s")
```

| Step | Command | Expect | Done |
|---|---|---|---|
| 3.1a | `uv run python /tmp/one_leg.py "$ROBOT_PORT" 90` | Completes; time it — this is the `headingKp=2.5` baseline settle time/feel to compare against | [ ] |

### 3.2 Push a live tune, confirm it changes behavior

```bash
uv run rogo --port "$ROBOT_PORT" repl "config headingKp=0.3; sleep 300"
```

(`rogo repl` joins every positional argument into ONE line and splits on
`;` — pass multiple verbs as ONE `;`-separated quoted string, not as
separate shell arguments, or they get parsed as extra tokens on the FIRST
verb and rejected.)

If the reply shows `ERR_UNIMPLEMENTED` instead of an ack: the flashed
firmware predates this sprint's `RobotLoop::handleConfig()` work — this is
a **real regression** on a fresh `mbdeploy deploy --build`, not an expected
state (`Config::PersistedTuning`/ticket 004 landed specifically so this
applies for real) — file it, don't just note it and move on.

| Step | Command | Expect | Done |
|---|---|---|---|
| 3.2a | `uv run rogo --port "$ROBOT_PORT" repl "config headingKp=0.3"` | ack `OK` (not `ERR`) | [ ] |
| 3.2b | `uv run python /tmp/one_leg.py "$ROBOT_PORT" 90` | Visibly slower to settle / more overshoot-then-crawl than 3.1a's baseline (0.3 is a much weaker heading-PD gain) | [ ] |

### 3.3 Power-cycle: confirm the tune survives

| Step | Action | Expect | Done |
|---|---|---|---|
| 3.3a | Unplug USB (or power-cycle the relay/robot), wait 5s, reconnect | Robot re-boots | [ ] |
| 3.3b | `uv run python /tmp/one_leg.py "$ROBOT_PORT" 90` | STILL the sluggish `headingKp=0.3` behavior from 3.2b, NOT the 3.1a baseline — this is `Config::PersistedTuning` reapplying the stored patch at boot | [ ] |

### 3.4 Schema-version-mismatched reflash: confirm the wipe

A same-version reflash does **not** wipe the store (that's the whole point
of persisting across reflashes) — only a `Config::kConfigSchemaVersion`
mismatch does (`src/firm/config/persisted_tuning.h`, currently `= 1`). To
actually see the wipe, deliberately bump it:

| Step | Action | Expect | Done |
|---|---|---|---|
| 3.4a | Edit `src/firm/config/persisted_tuning.h`: change `kConfigSchemaVersion = 1` to `= 2` (temporary — do NOT commit) | — | [ ] |
| 3.4b | `mbdeploy deploy --build` | Flashes successfully | [ ] |
| 3.4c | `uv run python /tmp/one_leg.py "$ROBOT_PORT" 90` | Back to the 3.1a baseline feel (`headingKp=2.5`, boot-bake value) — the version mismatch wiped the whole store, `0.3` is gone | [ ] |
| 3.4d | **Revert** `persisted_tuning.h` back to `= 1`, `mbdeploy deploy --build` again | Robot back to normal shipped state before continuing | [ ] |

**Pass:** 3.2b and 3.3b show the SAME (tuned, sluggish) behavior; 3.4c shows
the baseline behavior again after the version-mismatched reflash.

---

## 4. Deadband fix: a small terminal correction creeps to completion, doesn't hold flat

Ticket 005 boosts a sub-deadband nonzero commanded duty at write-shaping
time instead of zeroing it, so the historical ~15 mm/s dead zone no longer
makes the robot silently STOP short and hold there — it should keep
creeping (slowly) until the dwell tolerance is actually satisfied.

| Step | Command | Expect | Done |
|---|---|---|---|
| 4a | `uv run python /tmp/one_leg.py "$ROBOT_PORT" 3` (a tiny 3deg turn — the WHOLE move's terminal correction is deep inside the dead zone) | The wheels visibly/audibly creep for the last stretch instead of a hard stop-and-hold that never quite reaches the target; `run_tour()` reports `COMPLETED`, not a timeout | [ ] |
| 4b | While 4a runs, watch `rogo repl`'s `vel` in a second terminal (or add `--record /tmp/deadband.jsonl` to a `tour_bench_run.py`-style capture) | `vel` never sits pinned at a small nonzero value with the position error still open for more than ~1-2 cycles — it keeps decreasing toward the target, however slowly | [ ] |

**Fail (file an issue):** the move times out / never reaches the dwell
tolerance; `vel` reads flat zero (or a flat small nonzero value) for many
consecutive cycles while the reported heading error is still open — that is
the pre-005 "holds flat, never finishes" bug reappearing.

---

## 5. Capture a real wheel-speed trace, eyeball against the shape bar

The stakeholder's shape bar (already verified in sim by ticket 006, see the
ticket's own Completion Notes): a clean trapezoid (smooth ramp-up, hold,
smooth ramp-to-zero), no oscillations, no bumps/discontinuities at the end
of the move, a straight's trace never goes below zero, and a turn's trace
has exactly one wheel entirely below zero (the mirror wheel).

Reuse the SAME `/tmp/one_leg.py` from section 3, extended to record
telemetry (mirrors `tour_bench_run.py`'s own `row_callback` capture of
`frame.vel[0]`/`frame.vel[1]`, the real measured per-wheel velocity —
TLM's `vel=<vl>,<vr>` field):

```bash
uv run rogo --port "$ROBOT_PORT" repl --record /tmp/straight_trace.jsonl "twist 150 0 2000; sleep 2200; stop"
uv run rogo --port "$ROBOT_PORT" repl --record /tmp/turn_trace.jsonl "twist 0 1.57 1500; sleep 1700; stop"
```

(These two `twist` calls are open-loop, for a quick raw-plant shape check
without the heading-PD's own correction tail riding on top; re-run with
`/tmp/one_leg.py 90` recorded via `tour_bench_run.py --tours TOUR_1` for the
CLOSED-LOOP Move-based shape too, since that's what a real tour actually
drives with.)

| Step | Action | Expect | Done |
|---|---|---|---|
| 5a | Open `/tmp/straight_trace.jsonl`, plot/eyeball `vel[0]`/`vel[1]` over `t` | Smooth ramp 0→~150, flat hold near 150, smooth ramp back to 0; never dips below 0; no spike/discontinuity right at the end | [ ] |
| 5b | Open `/tmp/turn_trace.jsonl`, same for `vel[0]`/`vel[1]` | One wheel's trace is entirely ≥0 throughout, the other entirely ≤0 (mirror wheel) — never both positive, never both negative, never a partial/momentary dip across zero on either wheel | [ ] |

**Pass:** both traces visually match every clause of the shape bar above.
**Fail (file an issue with the JSONL attached):** any oscillation, end
bump, sign violation, or non-trapezoidal ramp shape.

---

## Reporting

- All checks pass (or are correctly marked N/A per section 2's own
  explanation): sprint 114 is bench-verified — tell the team-lead so
  `close_sprint` can proceed.
- Any failure: file a GitHub issue with the exact step, the observed vs.
  expected behavior, and (for section 5) the captured `.jsonl` trace.
