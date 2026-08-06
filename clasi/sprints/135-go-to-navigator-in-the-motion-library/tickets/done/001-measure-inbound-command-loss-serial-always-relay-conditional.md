---
id: '001'
title: Measure inbound command loss (serial always, relay conditional)
status: done
use-cases:
- SUC-006
depends-on: []
github-issue: ''
issue: replaceable-go-to-moves-in-the-motion-library.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Measure inbound command loss (serial always, relay conditional)

## Description

Replace an unsourced number with a measured one before it silently sizes
this sprint's EXTERNAL-mode retry policy (ticket 007).

**The folklore, stated precisely (copy this framing into the measurement
report so the provenance travels with the number):** a widely-cited
"~20% inbound command loss" figure appears in `speed_map.py`,
`square_tour.py`, and `planner_square_tour.py`. It traces to a single
unsourced 2026-07-27 docstring — no measurement artifact behind it,
written 45 minutes after the 12-deep command ring landed (so it may
describe pre-ring firmware behavior, not current). One copy of the
comment blames the DAPLink USB bridge; another blames the radio relay —
they disagree on WHERE the loss happens, which is itself evidence nobody
measured it. `wire_truth.py` measures the OTHER direction (telemetry
outbound, not commands inbound) and its own relay budget is explicitly an
unmeasured proposal. The firmware's own dropped-command counter
(`commandsDroppedCount`, telemetry flags bit 18) has **never been read by
any bench script** — this ticket is the first to consult it. The
cloverleaf 19/71 hardware failure
(`clasi/issues/later/path-following-hardware-gaps.md`) is explicitly
UNDIAGNOSED; candidates include the host script's own accounting and
`Move.id` dedup collisions, not established link loss. Do not repeat the
"~20%" figure as fact anywhere in this ticket's output — only as the
folklore being replaced.

**Relay-conditional scoping (this session's hardware constraint):**
`tovez` is on the stand, reachable over **direct serial ONLY** this
session — it is disconnected from the Raspberry Pi and there is no relay
path. This ticket's relay leg is a **conditional bonus**, never a
blocker: check `mbdeploy list` for a relay device before attempting it,
run it only if one is present, and report it as SKIPPED (not failed, not
silently omitted) if none is attached. The direct-serial leg is the one
this sprint's completion depends on.

This ticket has no code dependency on any other ticket in this sprint and
can run at any point — it needs only `tovez` and (optionally) a relay.

## Hardware target

`tovez` **only**, addressed **by UID**, never by port number or a
default/auto-selected target — `mbdeploy list` with no target picks
`vizev` (a different robot) whenever `tovez` happens to be unplugged, and
port numbers move on every re-enumeration. See
`.claude/rules/hardware-bench-testing.md`.

```
tovez UID: 9906360200052820a8fdb5e413abb276000000006e052820
```

Confirm the row says `tovez` in `mbdeploy list` before running anything;
if it's absent, `tovez` is unplugged — stop and report that, do not fall
back to whatever device IS present.

## Acceptance Criteria

- [x] A standalone script (`src/tests/bench/`) streams N id-distinct
      commands at a known rate over direct USB serial to `tovez` (by UID).
- [x] The script counts enqueue acks received (out of N sent) AND reads
      `commandsDroppedCount` / telemetry flags bit 18 after the run, so a
      loss can be attributed to link loss (commands never arrived) vs.
      firmware ring overflow (arrived but the 12-deep queue was full) —
      these are different failure modes with different fixes.
- [x] Script checks `mbdeploy list` for a relay device before attempting
      the relay leg; if present, repeats the same measurement over the
      relay; if absent, reports that leg as SKIPPED and exits with the
      direct-serial result as the sprint-relevant number — never fails or
      blocks on a missing relay.
- [x] Script prints a PASS/summary line reporting: N sent, acks received,
      measured loss %, `commandsDroppedCount` delta, and (if run) the
      relay-leg numbers — all as one self-describing block a future
      reader doesn't need session memory to interpret.
- [x] The unsourced "~20%" docstring claims in `speed_map.py`,
      `square_tour.py`, and `planner_square_tour.py` are each updated to
      cite this measurement's dated result (or explicitly marked
      superseded with a pointer to where the real number now lives) —
      in this same ticket, not deferred.
- [x] Measured result and its provenance (date, transport, rate, N) are
      recorded in this ticket's own Completion Notes so ticket 007 can
      size EXTERNAL-mode retry policy against it without re-deriving it.

## Implementation Plan

**Approach**: a small, standalone bench script (no dependency on the
Navigator/GO_TO work in tickets 002-007 — it exercises the EXISTING
command plane, e.g. repeated `CONFIG`/`SET_FIELD`/`GET_CONFIG` pushes or
another already-live id-carrying command, whichever gives the cleanest
enqueue-ack-per-command signal). Follow the existing bench-script
conventions in `src/tests/bench/` (argparse, `--port`, a `mbdeploy list`
probe rather than a hardcoded port).

**Files to create**:
- `src/tests/bench/command_loss_bench.py` (or similar name following the
  directory's existing naming pattern — check `src/tests/bench/` for the
  closest sibling to model against, e.g. `move_protocol_bench.py`'s
  PASS/FAIL summary style).

**Files to modify**:
- `src/host/robot_radio/pathplan/speed_map.py` — update or mark superseded
  the "~20%" docstring claim.
- `src/tests/bench/square_tour.py` — same.
- `src/host/robot_radio/pathplan/planner_square_tour.py` — same (verify
  this file's exact current path/name first; the issue's reference may be
  stale relative to sprint 127's actual file layout).

## Testing

- **Existing tests to run**: none — this is a new standalone bench
  script with no unit-test surface of its own; verify it imports cleanly
  and its `--help` runs: `uv run python src/tests/bench/command_loss_bench.py --help`.
- **New tests to write**: none required (bench scripts in this repo are
  hardware-exercising tools, not unit-tested per `.claude/rules/hardware-
  bench-testing.md`'s existing convention for this directory).
- **Verification command** (hardware, direct serial):
  `uv run python src/tests/bench/command_loss_bench.py --port <tovez's current port from mbdeploy list>`
  Confirm `tovez`'s row in `uv run mbdeploy list` immediately before
  running, and pass that session's port — never a remembered one.

## Completion Notes

**Path correction (found during execution):** the "Files to modify" list
above names `src/host/robot_radio/pathplan/speed_map.py` and
`src/host/robot_radio/pathplan/planner_square_tour.py` — both paths are
stale/wrong. The actual files, confirmed by `find`, are
`src/tests/bench/speed_map.py` and `src/tests/bench/planner_square_tour.py`
(`src/tests/bench/square_tour.py`, as already listed, was correct). All
three "~20%" docstrings in fact live in `src/tests/bench/`. Edits below
were made at the real paths.

**Script**: `src/tests/bench/command_loss_bench.py` (new). Streams
id-distinct `move_twist(replace=True)` enqueues over one open connection in
two legs — a **steady** leg (default 20 Hz, N=200) and a **burst** leg
(back-to-back, no inter-send delay, default N=40) — then scans every
captured telemetry frame's ack ring for each enqueue's own `corr_id`. An
enqueue counts as lost only if its `corr_id` never appears in any frame's
ring (an ack that arrived but reported a rejection is not "lost"). Reads
`flags` bit 18 (`kFlagFaultCommandsDropped`, ring-full backpressure) and
bit 9 (`kFlagFaultMalformedFrame`, corrupt inbound line) directly off
`TLMFrame.flags` before/after each leg — neither bit is decoded as a
`TLMFrame` property yet (`protocol.py`'s own comment already flags this as
an open gap for bits 17/18), so this script is the first bench script to
read them at all, exactly as this ticket required. Relay leg is
conditional on an `mbdeploy list` ROLE=RADIOBRIDGE row and is reported
SKIPPED, not failed, when absent.

**Measured result** (2026-08-05, hardware `tovez`, UID
`9906360200052820a8fdb5e413abb276000000006e052820`):

| leg | transport | rate | N | acks observed | loss % | flags bit18 (ring-full) | flags bit9 (malformed) |
|---|---|---|---|---|---|---|---|
| steady | direct serial (`/dev/cu.usbmodem2121102`) | 20 Hz | 200 | 200/200 | **0.00%** | False→False | (already set at session start — pre-existing, not attributable to this leg) |
| burst | direct serial | back-to-back (~500-650 cmd/s attempted) | 40 | 27-31/40 (two runs) | **22.5%-32.5%** | False→False | (already set) |
| steady | radio relay (`getez` @ `/dev/cu.usbmodem214102`) | 20 Hz | 200 | 190-193/200 (two runs) | **3.5%-5.0%** | False→False | False→False (one run — clean; genuine link loss, neither firmware fault fired) |
| burst | radio relay | back-to-back | 40 | 0/40 | **100%** | False→False | already set (one run) / clean (other run) |

Two full runs were made (the second added the bit-9 check); numbers above
span both. The steady-rate legs are the sprint-relevant numbers (20 Hz is
a realistic operating rate; the burst legs intentionally exceed any real
bench-script send rate to probe the ring's own overflow boundary).

**Headline finding for ticket 007's retry-policy sizing:** at a realistic
20 Hz send rate, direct-serial loss is ~0% and radio-relay loss is
~3.5-5%, and in every steady-rate leg the firmware's own command-ring-full
fault bit (flags bit 18) stayed **False throughout** — i.e. the ring never
actually overflowed at this rate. The measured relay-leg loss is therefore
genuine link loss (bytes/frames never assembled into a decodable line),
matching the "radio relay" copy of the old folklore comment, now measured
rather than assumed, and roughly 4-6x smaller than the unsourced "~20%"
figure it replaces. The burst legs' much higher loss (22.5-100%) is a
**different, unpaced-host-send-rate artifact**, not representative of any
real caller in this codebase (every existing sender — `speed_map.py`,
`square_tour.py`, `planner_square_tour.py` — already paces its own sends,
e.g. `planner_square_tour.py`'s `ENQUEUE_SPACING_S = 0.15`) and should not
be used to size a retry policy meant for realistic traffic.

**Documented gap surfaced by this measurement:** the wire carries no raw
`commandsDroppedCount` integer at all — only the one latching boolean fault
bit (flags bit 18). A future ticket wanting a live per-run drop *count*
(not just "has this happened since boot") would need a new wire field;
this measurement worked around that by reading the bit's before/after
transition instead, which is sufficient to attribute loss to link-vs-ring
but not to quantify ring-overflow volume precisely.

**Provenance summary**: date 2026-08-05; hardware `tovez` (UID above);
transports direct-serial (`/dev/cu.usbmodem2121102`) and radio relay
(`getez`, `/dev/cu.usbmodem214102`); steady-rate leg 20 Hz x N=200;
burst leg back-to-back x N=40; script
`src/tests/bench/command_loss_bench.py` (run with default arguments,
`--port` explicit, no other flags).
