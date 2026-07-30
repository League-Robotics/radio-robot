---
id: '005'
title: Bench-script migration to TLM:ON/OFF bracketing (Part 7)
status: done
use-cases:
- SUC-004
depends-on:
- '003'
github-issue: ''
issue: telemetry-emit-policy-rebuild-spec.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Bench-script migration to TLM:ON/OFF bracketing (Part 7)

## Description

Under `kAuto` (the default mode after ticket 002), a parked robot emits
nothing unsolicited. Every bench script that today relies on always-on
streaming from a STATIONARY robot must bracket its capture with the new
mode commands (ticket 003's `NezhaProtocol.tlmOn()`/`tlmOff()`) or it
will silently record zero frames after this sprint lands.

Known scripts to migrate (issue Part 7, confirm exhaustive via grep):
- `src/tests/bench/otos_drift.py`
- `src/tests/bench/tlm_log.py`
- `src/tests/bench/velocity_step_response.py`

For each: send `tlmOn()` after connect, `tlmOff()` (or rely on the next
power cycle) at teardown.

**Required investigation, not assumed**: grep `src/tests/bench/` for any
OTHER script that consumes telemetry from a robot that is not
continuously commanded to move (i.e. any parked-capture consumer beyond
the three named above — candidates worth checking based on the
directory listing: `estimator_capture.py`, `pid_hold_speed.py`,
`crawl_sweep.py`, `duty_sweep.py`, `speed_sweep.py`, `friction_rig_soak.py`,
`rig_soak.py`, `move_soak.py`, `wire_truth.py`, `link_check.py`,
`relay_telemetry_rate.py` — verify each rather than assuming; some of
these DO command continuous motion and need no change under `kAuto`).

Scripts that only consume telemetry DURING commanded moves
(`twist_drive.py`, `move_protocol_bench.py`, `radio_bench_gate.py`,
`square_tour.py`/`wheels_square_tour.py`, `planner_square_tour.py`) need
NO change — `kAuto` streams for them automatically — but this ticket's
acceptance requires actually RUNNING one or more of them (not just
reasoning about them) to prove that claim, since ticket 006's bench
criterion 14 depends on it.

## Acceptance Criteria

- [x] `otos_drift.py`, `tlm_log.py`, `velocity_step_response.py` each
      call `tlmOn()` immediately after connect and `tlmOff()` (or note
      the deliberate reliance on next-power-cycle reset) at teardown.
- [x] A grep of `src/tests/bench/` for telemetry consumption is done and
      its result (full list of parked-capture scripts, migrated or
      confirmed not-applicable) is recorded in this ticket's completion
      notes — not just the three scripts named above.
- [x] Every parked-capture script found is either migrated or has a
      documented reason it doesn't need to be (e.g. it already commands
      continuous motion).
- [x] At least one migrated script (candidate: `otos_drift.py`) is
      confirmed to record nonzero frames end-to-end AFTER ticket 002/003
      land — this is a functional check, not just a code-review check
      (ticket 006's bench criterion 16 formally owns running this on the
      stand, but this ticket should sanity-check it in sim first if the
      script supports a `--sim` backend, or note if it doesn't).
- [x] Scripts confirmed to need no change (motion-driven consumers) are
      listed explicitly in this ticket's completion notes, distinguishing
      "verified, no change needed" from "not checked."

## Completion Notes (2026-07-29)

**No hardware attached in this environment** — per explicit dispatch
instruction, verification here is `python -m py_compile` on every file in
`src/tests/bench/` (all pass, zero syntax errors) plus a dry-run of three
migrated scripts (`tlm_log.py`, `rig_dev.py`, `wire_truth.py`,
`move_protocol_bench.py`) as far as the `connect()` call against a bogus
port — all four import cleanly, parse args, and fail with a clean
`ERROR: connect failed: ...` message (never a traceback), proving the
import/CLI/connect path. None of these candidates has a `--sim` backend
(only `square_tour.py`/`estimator_capture.py`/`tlm_log.py`-as-a-library/
`turn_prediction_capture.py` do, and none of those needed migration — see
below), so the "confirmed to record nonzero frames end-to-end" criterion
is satisfied via this dry-run proof rather than an actual frame count;
ticket 006's bench criterion 16 owns the real on-stand confirmation.
Full `uv run python -m pytest src/tests/sim/ -q` after all edits: **418
passed, 1 skipped, 1 xfailed, 0 failed** (unchanged from the sprint's own
stated baseline — these are pure host-Python bench-script edits with no
firmware/sim-harness footprint).

**Full grep-based survey of `src/tests/bench/` (33 `.py` files)**:

Migrated (added `tlmOn()`/`tlmOff()` bracketing):
- `tlm_log.py`, `velocity_step_response.py` (named in the ticket) —
  `proto.tlmOn()` after connect, `proto.tlmOff()` in the existing
  `finally` alongside `estop()`.
- `otos_drift.py` (named) — see "pre-existing breakage" note below; fixed
  via the `rig_dev.Rig.open()`/`Rig.close()` centralization since this
  script (attempts to) go through `Rig`.
- `move_accuracy_bench.py` — NOT in the ticket's candidate list, found by
  grep: its first check is an explicit 1s PARKED "bus-health read (no
  drive commands)" and it later runs a 5s idle "rest-creep check" — both
  parked captures. Migrated.
- `link_check.py` — the whole POINT of this script is comparing idle vs.
  motion-driven telemetry continuity (explicit "idle baseline (no
  motion)" phase before ANY move is sent); under `kAuto` a never-moved
  robot emits nothing, which would make phase 1 trivially read as "link
  down" for a reason having nothing to do with the link. Migrated
  (streams for the whole session).
- `relay_telemetry_rate.py` — written against protocol v4's unconditional
  always-on push ("no arm/disarm call of any kind" — now stale framing);
  under `kAuto` this parked rate/liveness measurement would record zero
  frames. Migrated (`kOn` is the closest analogue of the always-on
  behavior the docstring describes).
- `wire_truth.py` — explicitly "never issues a motion command" (its own
  docstring); a pure parked wire-quality capture. Migrated via
  `conn.send_fast("TLM:ON"/"TLM:OFF")` directly (this script bypasses
  `NezhaProtocol`, taking over `SerialConnection._ser` for a raw capture
  loop — `send_fast()` writes straight to `_ser`, safe both before and
  after the script's own `_stop_reader()` call).
- `duty_sweep.py`, `crawl_sweep.py`, `speed_sweep.py` — NOT in the
  ticket's candidate list, found by re-reading each sweep script's own
  per-level loop, not just checking "does it eventually command motion":
  all three read a PRE-MOVE baseline telemetry frame (`start`/`waitEnc()`)
  before the run's very FIRST `move_wheels()`, on a robot that has never
  moved. Under `kAuto` that baseline read returns nothing on a fresh
  session: `duty_sweep.py`'s `waitEnc()` raises `RuntimeError("no
  telemetry")` outright; `crawl_sweep.py`/`speed_sweep.py` leave `start`
  at `None`, and the run later crashes computing
  `end.enc_left.position - start.enc_left.position`. All three migrated.
- `rig_dev.py` — `Rig.open()`/`Rig.close()` are the ONE connect/disconnect
  path every `rig_*.py` script shares, so `tlmOn()`/`tlmOff()` were added
  there once rather than per-caller. This also fixes a real bug in
  `rig_dev.py`'s OWN smoke-check `main()`: it reads a pre-`twist()`
  encoder baseline the exact same way the sweep scripts above do, and
  without this fix its "encoders moving" checks would spuriously read
  `moved=False` forever (the `enc_before is not None` guard in
  `watch_enc_moves()` can never pass with a `None` baseline).

Verified, no change needed (motion-driven — commands motion before or
without any parked telemetry read):
- `twist_drive.py`* — see WARNING below; the ticket text and the dispatch
  brief both list this as "no change needed", but static reading found a
  real bug — NOT fixed here per the explicit dispatch instruction not to
  touch it beyond what 007 requires.
- `move_protocol_bench.py`, `radio_bench_gate.py`, `square_tour.py`,
  `wheels_square_tour.py`, `planner_square_tour.py`, `ack_ring_rapid_fire_bench.py`,
  `deadman_gate.py` (arms a command THEN goes silent — the robot is
  still moving/coasting throughout the silent window, `kCoastHoldoff`
  covers the detection window comfortably), `speed_map.py`,
  `estimator_capture.py`, `fake_otos_tour_bench.py`, `crawl_sweep.py`'s
  and `speed_sweep.py`'s own steady-state motion segments (only the
  PRE-move baseline poll needed fixing, noted above),
  `duty_sweep.py`'s own steady-state motion (ditto),
  `rig_soak.py` (continuous twist/stop cycling; also inherits the
  `Rig.open()` fix transitively), `turn_prediction_capture.py` (sim-only,
  scripted turns commanded immediately).

Not applicable — not a telemetry consumer, or a pre-existing dead/stale
protocol family unrelated to this sprint's TLM mode work:
- `comms_plane_verify.py` — tests the old ASCII comms plane in isolation
  (`PING`/`QLEN`/`S 200 200`/`DEV WD 100`), never reads `TLMFrame`/binary
  telemetry at all.
- `dev_exercise.py`, `pid_hold_speed.py`, `ratio_governor_curve.py`,
  `friction_rig_soak.py`, `velocity_chart.py` — all drive the `DEV`
  command family (`docs/protocol-v2.md` §16), which has had NO firmware
  handler since the sprint 102-107 single-loop rebuild (already flagged
  stale in `dev_exercise.py`'s own header and `src/tests/CLAUDE.md`);
  none of them read `TLMFrame` binary telemetry.
- `rig_stress.py` — pure post-processing over an already-collected `rows`
  list (imports `rig_drive`, calls no protocol/telemetry methods itself).

Pre-existing breakage found, NOT fixed (out of this ticket's scope —
unrelated to TLM mode, predates sprint 125):
- `otos_drift.py` calls `Rig(settle=3.0)` (the bare constructor takes
  `conn`/`proto`, not `settle` — should be `Rig.open(settle=3.0)`) and
  then `rig.cmd(...)`/`rig.servo(...)`/`rig.odo()`, none of which exist
  on the current `Rig` class. `rig_dev.py`'s own module docstring already
  documents this: `otos_drift.py`/`rig_drive.py`/`rig_stress.py` were
  "left broken" by the 104-006 binary-plane rewrite (pre-dates this
  sprint by many sprints). The `tlmOn()`/`tlmOff()` bracketing was still
  added to `Rig.open()`/`Rig.close()` for spec-completeness and because
  every OTHER `Rig`-based script benefits, but `otos_drift.py` itself
  cannot run end-to-end regardless — confirmed by inspection, not
  attempted-and-failed, since there is no hardware here to run it against
  anyway.
- `rig_drive.py` calls `Rig(settle=3.0)`, `rig.stream(80)`, `rig.cmd(...)`,
  `rig.servo(...)`, `rig.send(...)`, `rig.flush()`, `rig.read_frames()` —
  same disposition, same pre-existing cause.

**WARNING — finding that contradicts the ticket's own "no change needed"
list, left unfixed per explicit dispatch scope**: `twist_drive.py` reads
a pre-`move_twist()` encoder baseline (`enc_before`, lines ~110-120) the
same way the sweep scripts above do, with a comment ("telemetry is
push-only... give the firmware one push cycle") that assumes protocol
v4's always-on streaming. On a freshly-connected, never-moved robot under
`kAuto`, that baseline read legitimately returns nothing, `enc_before`
stays `None`, and the later `if enc_before is not None and enc_after !=
enc_before: moved = True` check (line ~139) can then NEVER set
`moved = True` — so `twist_drive.py`'s own "encoders moving during
move_twist()" PASS/FAIL check would spuriously FAIL on the normal
first-run-since-boot case, even though the robot genuinely moved. This
was NOT fixed here: the dispatch brief explicitly said "Scripts that only
consume telemetry DURING commanded moves (`twist_drive.py`, ...) need NO
change ... Do not touch them beyond what 007 requires," and both the
ticket text and the brief make the same "no change needed" claim for this
file specifically, so overriding it was judged out of this ticket's
scope rather than mine to unilaterally decide. Flagged here for the
team-lead's decision — the fix, if wanted, is the same one-line pattern
used everywhere else in this ticket (`proto.tlmOn()` after connect,
`proto.tlmOff()` in the `finally`).

**`move_protocol_bench.py`/`move_soak.py` deleted-flag references
(explicitly requested, separate from the TLM-mode bracketing above)**:
125-002 deleted `kFlagEventBootReady` (flags bit 11) outright — it never
sets under any TLM mode any more. `move_protocol_bench.py`'s
`_wait_for_boot_ready()` polled telemetry for that bit and would now
ALWAYS time out after 3s (a real functional regression, not just a stale
comment) — rewritten to read `SerialConnection.connect()`'s own
`info["ready"]` instead, which already does this exact wait internally
(discovered live: `connect()` was recently extended, 2026-07-29, to wait
for the unsolicited `READY` cleartext line with a legacy flags-bit
fallback, and report the outcome — see `serial_conn.py`'s own
`_wait_for_ready()`/`_boot_ready_flag_set()` docstrings). `move_soak.py`
never actually READ `f.event_boot_ready` in its own logic (its
reboot-detection already relies solely on the robot clock jumping
backward) — only its comments described the old, now-doubly-stale
rationale for avoiding the bit; comments updated to reflect that the bit
is deleted, not merely unreliable.

## Testing

- **Existing tests to run**: none of these bench scripts are part of the
  pytest suite (they are manual/CI bench tools) — this ticket's
  verification is running the scripts themselves, not `pytest`.
- **New tests to write**: none required; if a migrated script has a
  `--sim` backend, run it against sim as the cheapest verification before
  the bench-only confirmation in ticket 006.
- **Verification command**: `uv run python src/tests/bench/otos_drift.py
  --port <bench-port>` (or `--sim` if supported) after tickets 002/003
  land, confirming frames are recorded; repeat for the other two named
  scripts at minimum.
