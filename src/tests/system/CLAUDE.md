# The system test (`systest`)

This directory is THE system test — one program, tour files, one JSONL
dataset per run. There is deliberately no other system test; new coverage
is a new tour file or a unit test, never a new system-test script
(umbrella issue: `clasi/issues/system-test-square-tour-is-the-one-system-test-sim-bench-playfield.md`).

## Run it

```bash
uv run python src/tests/system/systest.py run --tier sim \
    src/tests/system/tours/square.tour src/tests/system/tours/circle.tour
```

- Tours run in order; a failing tour GATES the rest (circle never runs if
  the square fails). Exit is nonzero on any failure.
- Each run writes `out/<tour>_<tier>_<timestamp>.jsonl` — every wire line
  both directions as one JSON record per line. The dataset is the
  deliverable; the PASS/FAIL lines are just its summary.
- `--tier sim` is the only wired tier today. `bench`/`playfield` backends
  are planned (same backend surface: `move/stop/estop/read_pending_frames/
  send_line/true_pose`). `--robot-config` defaults to
  `data/robots/tovez_nocal.json`; `--speed N` scales sim time (dwells and
  EXPECT timeouts stay wall-clock — fine at 1, approximate above).
- The sim tier needs `src/firm/platform/host/build/libfirmware_host.dylib`:
  `cmake -S src/firm/platform/host -B src/firm/platform/host/build && cmake --build src/firm/platform/host/build -j8`
  (run `uv run python src/scripts/gen_version.py` first in a fresh
  checkout — the version header is generated, gitignored).

## Tour files (`tours/*.tour`)

A tour is a text version of the protocol: one directive per line, `#`
comments, `key=value` args, human units (mm, mm/s, deg, deg/s, s).

| Directive | Meaning |
|---|---|
| `TWIST vx= [vy=] [omega=] (time=\|dist=\|angle=) [timeout=]` | one Move, twist variant; exactly one stop condition |
| `WHEELS left= right= (time=\|dist=\|angle=) [timeout=]` | one Move, per-wheel variant |
| `STOP [dwell=<s>]` | planned stop (queues behind the active Move), wait at rest, dwell |
| `DWELL <s>` | host-side pause, nothing sent |
| `MARK <text>` | sugar for `DBG mark <text>` — the echo lands IN the robot's output stream, so every dataset self-documents its segment boundaries |
| `DBG <subcmd> [args]` | fault injection: `wedge left\|right\|both [ms]`, `clear`, `ping`, `mark <text>` (ROBOT_DEBUG builds only; sim always has it) |
| `SEND <verb> [data]` | any cleartext verb (`SEND STATUS`, `SEND PING`) |
| `EXPECT '<jq>' [timeout=<s>]` | assert a record matching the jq query arrives after the PREVIOUS directive, within timeout |
| `CAMFIX x= y= radius= [heading= tol=]` | independent position check: plant truth on sim, camera on playfield |

Timeout defaults to 3× the expected duration (floor 2 s). Unknown
directives and unknown keys are hard errors — no silent misparsing.

## EXPECT queries

jq syntax over each record. The record envelope: `type` (`tlm`/`dbg`/
`cleartext`/`cmd`/`step`/`expect`/`camera_fix`/`run_meta`/`note`), `dir`,
`verb`, `raw`, `payload`. Useful shapes:

```
.type=="tlm" and (.payload.flags | index("FAULT_WEDGE_LATCH"))
.type=="cleartext" and .verb=="STATUS" and .payload.ready==1
.type=="dbg" and (.payload.text | startswith("pong"))
```

Telemetry payloads are nested (enc_left/enc_right/pose/twist/otos/acks/
cycle_busy/cycle_period) with `flags` as a name list — all 21 firmware
bits, including 17–20 which the host TLMFrame has no properties for.

## Goldens

```bash
uv run python src/tests/system/systest.py plot    out/<run>.jsonl
uv run python src/tests/system/systest.py bless   out/<run>.jsonl [--runs <more>.jsonl ...]
uv run python src/tests/system/systest.py compare out/<run>.jsonl
```

Ten signals, one per image (wheel speeds, commanded wheels, xy trace,
heading, x/y vs t, cycle timing). `bless` pins the axes and tolerance
(from `--runs` spread when given) and stores the gzipped dataset beside
the images under `goldens/<tier>/<tour>/`. **Blessing is Eric's act** —
never bless-and-commit a golden yourself; produce candidates and stop.
`compare` is the analytic gate (max/rms deviation), exits nonzero on any
signal outside tolerance.

## Known state (2026-08-01)

- `square.tour` FAILS its closing CAMFIX on sim (~361 mm) because sim
  turns undershoot ~10° each — that is the test correctly catching
  `clasi/issues/sim-tour-turn-shaping-undershoots-90-degree-turns.md`.
  Do NOT widen the tour's radius to make it pass; fix the undershoot.
- `fault_wedge.tour` is the DBG-injection proof: 13/13, wedge window
  bracketed by mark records in the dataset.
- The firmware's inbound-DBG surface is `#ifdef ROBOT_DEBUG` — shipped
  images compile it out; the sim (HOST_BUILD) always has it. Review
  record: `clasi/issues/systest-firmware-changes-dbg-inbound-wedge-injection-cleartext-plane.md`.

## Files

`systest.py` CLI · `tourfile.py` grammar/parser · `runner.py` backends +
executor (single destructive telemetry drain; ack-ring completion;
estop on failure) · `recorder.py` JSONL writer + record bus · `signals.py`
extractors · `goldens.py` plot/compare/bless · `tours/` the scripts ·
unit tests in `src/tests/unit/test_systest_tourfile.py`.
