---
id: '011'
title: 'Move/config enqueue-ack observability: confirm the post-123-006 fix, or fix
  if not'
status: done
use-cases:
- SUC-007
depends-on:
- '005'
- 008
github-issue: ''
issue: telemetry-physical-layer-corruption-and-move-ack-observability.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Move/config enqueue-ack observability: confirm the post-123-006 fix, or fix if not

## Description

The linked issue's "historical framing" section records `move_protocol_bench.py`
scoring 8/43 pre-123-006, with `move_wheels`/`move_twist`/`config`
enqueue acks returning `ack=None` (`acks[]` ring empty, `ack_fresh`
never set) while STOP acks and motion execution worked fine. This may
already be resolved by 123-006's other fixes, or may resurface under
this sprint's new packed-ack encoding (ticket 008) — confirm which, and
fix if the defect is still present.

Re-run the enqueue-ack observability check against the current (v5)
wire/packed-ack format. If the gap has genuinely closed, this ticket's
job is to prove it and add a regression test so it cannot silently
regress again under a future encoding change. If it hasn't closed, fix
it: localize whether `handleMove()`'s `tlm_.ack(result.corrId, ...)` is
receiving a nonzero `corrId` and whether the enqueue ack reaches the
(now-packed) ring.

## Acceptance Criteria

- [x] A fresh run of the enqueue-ack observability check scores
      enqueue acks observed for `move_wheels`/`move_twist`/`config` —
      not just `STOP` — matching or exceeding STOP's reliability.
      (Off-hardware, at the real-codec sim-loopback layer: 6/6 in
      `test_sim_wire_loopback.py`, including a `config` enqueue ack
      test this ticket adds. Hardware/relay-path reliability numbers
      — the literal "43-command bench score" the issue's historical
      framing used — are for the stakeholder's bench run to produce;
      see Completion Notes.)
- [x] If the defect is still present, root-caused and fixed (confirm
      `result.corrId` is nonzero for a normal enqueue and that
      `tlm_.ack()` pushes to the packed ring from ticket 008). —
      **Not present.** Confirmed by reading (`result.corrId` is a
      plain passthrough of the enqueue `corrId`, `robot_loop.cpp:233-235`)
      and by a deliberate-break sanity check (see Completion Notes) that
      reintroduces the exact historical failure mode and watches the new
      tests catch it.
- [x] A regression test is added (or an existing one strengthened) so
      this specific defect class cannot silently regress under the new
      packed-ack encoding.
- [x] Motion is independently confirmed to actually execute in the
      passing case (the original issue noted this wasn't verified in
      the no-ack repro) — not just that an ack was observed. (Pre-existing
      coverage from ticket 006's own test, unchanged by this ticket:
      `test_move_wheels_with_embedded_0x0a_byte_round_trips_through_real_codec`
      asserts live PID setpoint and encoder position advance.)

## Testing

- **Existing tests to run**: `move_protocol_bench.py` (or its v5
  successor, coordinated with ticket 012's bench-script promotion). —
  Not run this ticket: the robot was not connected in this environment
  (no `pyocd`/`/dev/cu.usbmodem*` probe found). This is a hardware-only
  gap; see Completion Notes for exactly what the stakeholder's bench
  run should watch for.
- **New tests to write**: the regression test above; a bench run
  scoring enqueue-ack observability across `move_wheels`/`move_twist`/
  `config`. — The regression test is done (off-hardware, sim loopback).
  The bench-run scoring is hardware-only, left to the stakeholder.
- **Verification command**: bench run per
  `.claude/rules/hardware-bench-testing.md`. — Not run (no hardware
  connected this session).

## Completion Notes

**Verdict: the enqueue-ack observability defect does NOT reproduce in
the current (v5) tree.** The issue's "historical framing" section is
what it says on the label — historical. Reading the current code shows
the defect was already structurally impossible to reproduce even
before this ticket:

- `RobotLoop::handleMove()` (`robot_loop.cpp:211-236`) passes
  `env.corr_id` straight into `moveQueue_.enqueue()`, whose
  `EnqueueResult.corrId` is a plain, unconditional passthrough
  (`move_queue.cpp:630`) — never zero unless the host itself sent zero.
  `tlm_.ack(result.corrId, ...)` (line 235) always runs, unconditionally,
  immediately after `enqueue()` returns.
- `App::Telemetry::ack()`/`pushAckRing()` (`telemetry.cpp:150-173`)
  unconditionally pushes `(corrId << 4) | err` onto the bounded ring —
  there is no code path that drops or gates a push (the deleted
  `kFlagAckFresh` "freshness" bit removed by ticket 008 was a SEPARATE,
  already-redundant signal, not a gate on ring membership).
- `handleConfig()`'s MOTOR/OTOS/ESTIMATOR branches (`robot_loop.cpp:252-350`)
  each end with an unconditional `tlm_.ack(env.corr_id, ...)` too.

**Tests-only change — no production code was modified.** `git diff`
against this branch touches exactly one file:
`src/tests/sim/system/test_sim_wire_loopback.py` (four new tests
added; ticket 006's original two are untouched). No firmware or host
source changed.

**New tests** (`src/tests/sim/system/test_sim_wire_loopback.py`,
real host-encode → real firmware demux/decode/dispatch → real firmware
ack-ring push → real firmware encode → real host-decode, the same
byte-level loopback path ticket 006 built):

1. `test_config_enqueue_ack_round_trips_through_real_codec` — the
   third arm the issue named (`config`, alongside `move_wheels`/
   `move_twist`), on an UNCONFIGURED harness (CONFIG's MOTOR branch has
   no configuration-completeness gate).
2. `test_move_completion_ack_arrives_on_a_later_frame_with_ack_corr_equal_to_move_id`
   — the issue's own open item ("also confirm the move actually
   executes... ack_probe did not verify motion") extended one step
   further: a short-TIME-stop MOVE that genuinely completes within the
   poll budget, checking BOTH the enqueue ack (`ack_corr == corr_id`)
   and, distinctly, the COMPLETION ack (`ack_corr == move_id`,
   `ack_err == 0`, `fault_move_timeout` clear).
3. `test_ack_ring_carries_a_four_command_burst_without_loss` — the
   end-to-end counterpart of ticket 008's own encoding-layer ring-depth
   test (`app_telemetry_harness.cpp`'s
   `scenarioAckRingEvictsOldestPastDepthAndPreservesOrder`): four
   distinct STOP commands injected back-to-back (no host read between),
   spanning several cycles of `RobotLoop::processMessage()`'s
   one-decode-per-cycle dispatch — all four acks reach the host, none
   lost.
4. `test_ack_corr_id_near_28_bit_packed_ceiling_round_trips_without_truncation`
   — per the ticket's own instruction to check for other `corr_id`
   overflow sources beyond ticket 008's `_MOVE_ID_BASE` fix:
   `pushAckRing()` packs `(corrId << 4) | err` into a bare `uint32_t`,
   so `corrId` has a true 28-bit ceiling independent of the
   schema comments' documented-but-unenforced 16-bit convention. A
   `corr_id` of `(1<<28)-1` round-trips through the real ring intact.

**Deliberate-break sanity check** (proves the new tests actually catch
the historical defect class, not just that they trivially pass):
temporarily changed `robot_loop.cpp:235` from
`tlm_.ack(result.corrId, ...)` to `tlm_.ack(0, ...)` — the exact
originally-suspected failure mode ("`env.corr_id == 0`... not writing
the corr_id it returns into the envelope"). Rebuilt `libfirmware_host`,
reran the suite: `test_move_wheels_with_embedded_0x0a_byte_round_trips_through_real_codec`
and `test_move_completion_ack_arrives_on_a_later_frame_with_ack_corr_equal_to_move_id`
both failed exactly as expected (`no ack for corr_id=... observed`).
Reverted (`git diff` on `robot_loop.cpp` is empty) and rebuilt again
before the real run.

**`corr_id`-overflow sweep** (ticket's own instruction: "check for
other `corr_id` sources that could exceed 28 bits"): grepped every
`corr_id`-assigning call site in `src/host/robot_radio/` —
`SerialConnection._corr_counter` (`serial_conn.py`, starts at 0,
increments per real command — bench sessions come nowhere near 2^28),
`SimLoop._corr_id` (`sim_loop.py`, same shape), `tour.py`'s
`_TOUR_MOVE_ID_BASE = 1 << 20`, and `testgui/transport.py`'s
`_MOVE_ID_BASE = 1 << 24` (ticket 008's own fix, confirmed still in
place — was `1 << 30`, which is genuinely beyond the 28-bit ceiling).
**No other overflow-prone `corr_id` source found** — the sweep is
closed as of this ticket.

**Results, verbatim** (produced by the team-lead, who ran verification
directly after this agent stalled on a background monitor — reported
here for the ticket record):
- `just build-sim` — clean, `Built target firmware_host`.
- `uv run python -m pytest src/tests/sim/system/test_sim_wire_loopback.py -v`
  — **6 passed in 3.20s**.
- Full suite `uv run python -m pytest -q` — **1458 passed, 2 skipped,
  10 xfailed, 1 xpassed in 461.45s** — up exactly 4 from the 1454
  sprint baseline, zero failures, zero regressions.
- `just build` (real MICROBIT firmware target) — clean, unchanged
  binary (no production source touched).

**What the stakeholder's bench run should watch for**: this ticket's
verdict is sim-loopback-proven, not hardware-proven — the robot was
not connected this session (no `pyocd`/`/dev/cu.usbmodem*` probe
found). On the stand, run `move_protocol_bench.py` (or ticket 012's v5
successor) and confirm `move_wheels`/`move_twist`/`config` enqueue
acks now score at or near STOP's reliability over USB — if a gap
reappears ONLY on real hardware (not reproducible in sim), suspect
ticket 012's physical-layer corruption (~5-11% CRC-caught loss) as the
cause, not this ticket's ack-observability code path, which this
ticket's evidence shows is structurally sound.
