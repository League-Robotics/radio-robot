---
id: '012'
title: 'Physical-layer measurement: USB and relay wire quality, stated loss budget
  (measurement only, no debugging expedition)'
status: done
use-cases:
- SUC-007
depends-on:
- '005'
- '010'
github-issue: ''
issue: telemetry-physical-layer-corruption-and-move-ack-observability.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Physical-layer measurement: USB and relay wire quality, stated loss budget (measurement only, no debugging expedition)

## Description

**Measurement and budget only — explicit non-goal below.** Characterize
BOTH the direct-USB and radio-relay physical wire paths under the final
v5 framing (ticket 005) and a clean relay connect (ticket 010), and
state a loss budget for each. This ticket promotes the scratchpad
`wire_truth.py`-style probe (single-threaded raw pyserial + demux +
decode, no queue/threads — the authoritative wire-quality measurement
from 123's overnight bench work) into a committed script under
`src/tests/bench/`.

**Explicit non-goal**: this ticket does NOT open a hardware
investigation. The residual ~5-11% USB corruption is ALREADY PROVEN
100% CRC-caught (`unparseable=0` on every prior run) — the link is
safe, just lossy, and that is an ACCEPTED condition, not a bug to chase.
Do not attempt to root-cause the physical-layer corruption itself here;
measure it, and the relay path, and state numbers.

## Acceptance Criteria

- [x] A `wire_truth`-equivalent script is committed to
      `src/tests/bench/` (not left in scratchpad) — single-threaded raw
      capture + demux + decode, matching the 123-era probe's
      methodology. — `src/tests/bench/wire_truth.py`. Reuses
      `SerialConnection.connect()` for the proven HELLO/relay-`!GO`
      handshake only, then stops the background reader thread
      (`_stop_reader()`) and takes over the raw port for a
      single-threaded, no-queue capture loop — see the script's own
      "Methodology" docstring section.
- [ ] Both USB and relay are measured under comparable conditions
      (same robot, same firmware build, similar session length);
      results recorded (not just asserted in code) in a form ticket
      013 can reference for its own bench-gate pass/fail. — **Not
      measured this session: no hardware connected** (`pyocd list`
      found no probe, no `/dev/cu.usbmodem*`). The instrument is built,
      dry-run verified (see Completion Notes), and takes `--json-out`
      specifically so ticket 013 can consume its result directly. The
      stakeholder's bench run closes this criterion — see Completion
      Notes for the exact commands.
- [x] A stated loss budget (a percentage, or equivalent) is recorded
      for each path. — `kUsbBudget`/`kRelayBudget` in
      `wire_truth.py`, with the full numeric reasoning in the script's
      own "The stated loss budget" docstring section (also summarized
      in Completion Notes below). The relay budget is an explicit
      first-pass proposal, not a measured fact — no relay baseline
      exists yet.
- [ ] `unparseable=0` is reconfirmed on both paths (CRC catches
      everything; nothing mis-parses). — **Hardware-only; not
      reconfirmed this session** (no hardware connected). The
      instrument enforces this as a hard budget gate
      (`kUsbUnparseableBudget`/`kRelayUnparseableBudget = 0`, scored
      against the `cobs_malformed` bucket) and was dry-run verified to
      correctly detect a nonzero count (see Completion Notes). The
      stakeholder's bench run produces the real number.
- [x] The non-goal above is stated explicitly in this ticket's closing
      notes: no root-cause investigation was opened for the residual
      USB corruption. — See Completion Notes' own "Non-goal" section,
      and the identical statement in `wire_truth.py`'s module
      docstring.

## Testing

- **Existing tests to run**: N/A — this ticket is itself a measurement
  script, run on the bench, not a unit-test change.
- **New tests to write**: the promoted `wire_truth`-equivalent bench
  script itself. — Done: `src/tests/bench/wire_truth.py`. Per
  `src/tests/CLAUDE.md`, `src/tests/bench/` scripts are HITL Python CLI
  tools, not pytest-collected — no pytest test file was added for this
  ticket; the script's classification/episode/budget logic was instead
  dry-run exercised directly (offline, no hardware) — see Completion
  Notes for the exact checks and their output.
- **Verification command**: run the promoted script over both USB and
  relay on the bench; record results per
  `.claude/rules/hardware-bench-testing.md`. — Not run this session
  (no hardware connected); see Completion Notes for the exact commands
  the stakeholder should run.

## Completion Notes

**Non-goal (restated per the Definition of Done — this must not be
misread later as an open investigation):** this ticket did NOT open a
hardware or root-cause investigation into the residual physical-layer
corruption. The ~5-11% USB steady-state corruption (post-9b4ea538) is
an ACCEPTED condition, already proven 100% CRC-caught by sprint 123's
overnight bench work. Nothing in this ticket's work — the script, its
docstring, this note — forms or records a theory about WHY bytes get
corrupted on either path. If a future bench run surfaces such a theory,
it belongs in a fresh `clasi/issues/` entry, not a reopening of this
ticket. `wire_truth.py`'s own module docstring states this identically,
so the non-goal travels with the instrument, not just the ticket.

**No hardware was connected to this development environment** (`pyocd
list`: "No available debug probes are connected"; `ls /dev/cu.usbmodem*`:
no matches) — every hardware-observed acceptance criterion above is
left unchecked, with the stakeholder's own bench run as the closing
step. This agent built the instrument and stated the budget; it could
not take the measurement.

**The instrument**: `src/tests/bench/wire_truth.py`.
```
uv run python src/tests/bench/wire_truth.py \
    --port /dev/cu.usbmodem2121102 --label direct-usb --duration 120 \
    --json-out /tmp/wire_truth_usb.json

uv run python src/tests/bench/wire_truth.py \
    --port /dev/cu.usbmodem2121302 --label relay --relay --duration 120 \
    --json-out /tmp/wire_truth_relay.json
```
Prints a per-bucket breakdown (`ok`/`cleartext`/`empty`/
`cobs_malformed`/`crc_mismatch`/`unrecognized_verb`/
`crc_ok_protobuf_invalid`), byte/line rate, episode detection (>=80%
corrupted within a 1s window, contiguous windows merged), and a
BUDGET/RESULT PASS-FAIL line scored against the constants below. Ports
are bench-specific — confirm with `mbdeploy list`'s ROLE column.

**The stated loss budget** (full reasoning in the script's own "The
stated loss budget" docstring section):

| path | corrupted-rate budget | longest-episode budget | unparseable budget |
|---|---|---|---|
| USB direct (`kUsbBudget`) | <=15% | <=5s | ==0 (hard gate) |
| radio relay (`kRelayBudget`, first-pass proposal) | <=25% | <=10s | ==0 (hard gate) |

USB's numbers are grounded in 123's own established ~5-11% steady-state
baseline (1.4-3x margin) and `NezhaProtocol.wait_for_ack()`'s 500ms
default ack-wait timeout (an episode at the 5s budget ceiling already
guarantees a default-timeout ack wait sees silence — expected/budgeted,
not a bug — while staying well under the worst historical ~16s episode
so a real regression is still caught). Relay has NO measured baseline —
its numbers are an explicit first-pass proposal (one added physical hop
over USB, proportionally, not unconditionally, wider tolerance) for the
stakeholder's bench run to confirm or replace; if the real number lands
meaningfully outside this proposal, the fix is editing the `kRelay*`
constants in the script, not re-running this ticket.

**The `#`/`!`/`?` counting bias (124-010)** — handled by NOT correcting
for it silently: `wire_truth.py` classifies every demuxed line on its
own merits regardless of first byte (a sigil-corrupted verb byte lands
in `unrecognized_verb` here, same as any other unrecognized verb), then
separately tags and reports how many corrupted-bucket hits had a
`#`/`!`/`?` first byte (`sigil_biased` in the JSON/report). The printed
report shows BOTH this script's own true, uncorrected rate AND the rate
production's own `malformedCount_`/`malformed_frame_count` counters
would show (this script's rate minus the sigil-biased fraction) side by
side, so a reader comparing the two numbers knows the gap is this known
~1.2%-of-corrupted-population bias, not a second corruption mechanism.
Dry-run verified: a synthetic line with its verb's first byte corrupted
to `#` classifies as `unrecognized_verb` with `sigil_first_byte=True`
(see below).

**Dry-run verification performed** (no hardware; all offline, direct
Python calls against the script's own functions):
- `--help` — argument parsing renders correctly, full docstring shown.
- No-device path: `--port /dev/cu.doesnotexist999 --duration 1` failed
  in 0.12s (not a hang) with a clean, specific error
  (`connect() failed for '/dev/cu.doesnotexist999': {'error': "[Errno 2]
  could not open port..."`) and exit code 2.
- `classify_line()` exercised against: (1) a genuine, uncorrupted `TLM`
  frame built via the real `wire_codec.encode_frame()` — classifies
  `ok`, `seq`/`cycle_period` extracted correctly; (2) the same frame
  with one payload byte flipped — classifies `crc_mismatch`; (3) the
  same frame with its verb's first byte corrupted to `#` — classifies
  `unrecognized_verb`, `sigil_first_byte=True` (proves the 124-010 bias
  handling above); (4) totally random garbage bytes — classifies
  `unrecognized_verb`, no crash; (5) an empty line — classifies
  `empty`.
- `find_episodes()` exercised against a synthetic window series with
  four contiguous >=80%-corrupted 1s windows followed by a clean one —
  correctly collapses them into one `(0.0, 4.0)` episode.
- `report()`/budget pass-fail exercised against two synthetic
  `CaptureStats` (6% corrupted -> PASS against `kUsbBudget`'s 15%
  ceiling; 20% corrupted -> FAIL) and JSON-serialized via `asdict()` to
  confirm the `--json-out` path produces valid, ticket-013-consumable
  JSON (including `Counter` fields, which serialize as plain dicts).

**Full verification, this session:**
- `uv run python -m pytest -q` (foreground, no background-poll) —
  **1458 passed, 2 skipped, 10 xfailed, 1 xpassed in 459.57s** —
  matches the sprint baseline exactly, zero regressions. (Known flake
  `test_managed_seg_0_cdeg_turn[-270]` did not fire this run.)
- `just build` — clean, `Built target MICROBIT`/`MICROBIT_hex`, plus
  `firmware_host`.
- `just build-sim` — clean, `Built target firmware_host`.
- No firmware or production host source touched — `git status`/`git
  diff` show only `src/tests/bench/wire_truth.py` (new file) and this
  ticket.

**What the stakeholder's bench run should watch for**: run both
commands above back-to-back (same robot, same firmware build, same
duration — the ticket's own "comparable conditions" criterion), record
the printed BUDGET/RESULT line and the `--json-out` files for ticket
013. If USB's own steady-state rate has drifted materially from the
5-11% sprint-123 baseline (either direction), that is itself a signal
worth a note (possibly a fresh cable/port, possibly a real regression)
— but per this ticket's non-goal, do not chase WHY here; record the
number and let ticket 013's own bench gate and/or a fresh
`clasi/issues/` entry take it from there. If `unparseable`
(`cobs_malformed`) is nonzero on EITHER path, treat that as the one
result this ticket would consider a genuine surprise (123 established
framing itself never breaks) — flag it, do not investigate it inline.
