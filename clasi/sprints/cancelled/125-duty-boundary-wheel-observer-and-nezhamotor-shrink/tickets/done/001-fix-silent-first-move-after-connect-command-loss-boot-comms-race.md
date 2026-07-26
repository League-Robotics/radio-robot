---
id: '001'
title: Fix silent first-MOVE-after-connect command loss (boot/comms race)
status: done
use-cases:
- SUC-009
depends-on: []
github-issue: ''
issue:
- firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md
- bench-move-commands-intermittently-never-reach-firmware.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fix silent first-MOVE-after-connect command loss (boot/comms race)

## Description

**Team-lead note**: this ticket ALSO addresses
`clasi/issues/later/bench-move-commands-intermittently-never-reach-firmware.md`
(filed 2026-07-23, confirmed reproducible against the v5 cutover on
2026-07-26). `issue:` above was set by `create_ticket`'s single-issue
auto-link at the moment this ticket was created (the sprint had exactly
one linked issue then) — it is NOT that specific defect issue. Once the
team-lead promotes it out of `later/` and links it to sprint 125, attach
it here via `add_issue_ref()`; `completes_issue` is set `false` above so
this ticket alone does not prematurely archive the base-hardening umbrella
issue it auto-linked to.

Root-cause and fix the 100%-reproducible silent loss of the first `MOVE`
sent after a fresh connect (`move_protocol_bench.py`'s
`scenario_distance_stop`: `ack=None`, encoders read `(0,0)` before AND
after — the command itself never reached `RobotLoop::processMessage()`,
not merely its ack). Confirmed NOT physical-layer corruption (`wire_truth.py`
measured 0% corruption the same session). Leading hypothesis (session
analysis, not yet confirmed): a startup race between `boot()`'s own
`comms_.pump()` (123-006) and the configuration-completeness gate /
queue-readiness `handleMove()` checks — a `MOVE` arriving in that window
may be silently consumed and discarded with no ack of any kind, not even
`ERR_NOT_CONFIGURED`. Also investigate the additional intermittent
enqueue-ack losses seen later in the same bench runs (34/43-39/43 across
five full runs) — confirm whether they share this root cause or are a
separate, still-open gap.

Scoped narrowly and landed FIRST, independent of and before the duty-
boundary/observer work (sprint Architecture Design Rationale Decision
10): this is a `Comms`/`boot()`/`processMessage()` fix, not a duty-
primitive change, so it is verifiable against the CURRENT tree before any
other ticket in this sprint touches `robot_loop.cpp`.

## Acceptance Criteria

- [x] Root cause confirmed with evidence (verbose `on_send`/`on_recv`
      logging and/or `Comms::malformedCount()`/`kFlagFaultCommsMalformed`
      inspection around the drop) — state definitively whether the bytes
      never arrived, arrived but failed to decode, or decoded but were
      discarded by the configuration-completeness/queue-readiness gate.
      **Confirmed: decoded cleanly, then discarded.** `RobotLoop::boot()`
      called `comms_.pump(bootCmd, ...)` every preamble-probe pass, but
      `bootCmd` was a throwaway loop-local never handed to
      `processMessage()`/any handler — a fully-decoded command was simply
      dropped, no ack of any kind. A hardware timing probe
      (`boot_race_probe.py`, sends the SAME first `MOVE`
      `move_protocol_bench.py` sends, immediately after `connect()`
      returns) proved the race is real and reliable: 4/5 timed runs showed
      `connect()` returning in 25-120ms while `kFlagEventBootReady` did not
      appear until ~1.2-1.3s later (device-probe retries), so the MOVE
      landed squarely inside `boot()`'s own pump window. `TLMFrame.
      fault_malformed_frame`/`Comms::malformedCount()` never fired in any
      run, proving the bytes were NOT corrupt and DID decode — this is a
      discard-after-decode, not a link/decode failure.
- [x] Fix lands such that a `MOVE` arriving before the robot is fully
      ready is EITHER accepted and executed OR explicitly rejected with
      `ERR_NOT_CONFIGURED` (or an equivalent explicit error) — never
      silently dropped with no ack of any kind.
      **Landed: explicit rejection.** `RobotLoop::rejectDuringBoot()`
      (`src/firm/app/robot_loop.{h,cpp}`) acks `ERR_NOT_CONFIGURED` for any
      command `comms_.pump()` decodes during `boot()`'s own while loop,
      mirroring `handleMove()`'s existing `configured_` gate but applied
      uniformly to MOVE/CONFIG/STOP alike (boot() never dispatches into
      `handleMove()`/`handleConfig()`/`handleStop()` — none of
      `moveQueue_`/`motorL_`/`motorR_`/`otos_` are safe to act on before
      `Preamble::done()`, so executing early was rejected as the chosen
      branch, not just rejected-with-error as a fallback).
- [x] **[off-hardware]** A sim/`motion_tests` regression test constructs
      the same race window (a `MOVE` arriving during `boot()`'s pump
      window, before `configured_` flips true) and asserts one of the two
      outcomes above, never a silent drop.
      `scenarioMoveArrivingDuringBootIsExplicitlyRejectedNeverSilentlyDropped()`
      in `src/tests/sim/unit/app_robot_loop_harness.cpp` (registered in
      `main()`, run by `test_app_robot_loop.py` /
      `uv run python -m pytest`): injects a MOVE via `FakeTransport`
      BEFORE calling `robotLoop.boot()` at all, on a virtual clock (no
      real-time flakiness), so `boot()`'s own FIRST `comms_.pump()` call
      decodes it while `preamble_.done()` is provably still false (5
      productive `step()` calls still needed). Asserts the ack is
      observed (never absent) with `err == ERR_NOT_CONFIGURED`, and that
      `MoveQueue` stays untouched. `markConfigured()` is called on the
      fixture too, matching real firmware's own `main.cpp` ordering
      (`markConfigured()` runs BEFORE `run()`), so the scenario proves the
      fix against the same state real firmware boots into, not a fixture
      where the `configured_` gate would coincidentally also refuse the
      command for an unrelated reason.
- [x] **[stand-required, USB]** `move_protocol_bench.py`'s
      `scenario_distance_stop` acks and executes on the first `MOVE`
      after a fresh connect, 5/5 runs — positive evidence: 5 observed
      acks + 5 observed nonzero encoder deltas.
      **5/5, all executed** (traveled 204.9mm / 206.2mm / 206.2mm /
      206.2mm / 209.1mm — commanded 200mm, tolerance ±20%). This required
      ONE additional change beyond the firmware fix: `move_protocol_bench.py`
      itself sent its first `MOVE` immediately after `connect()` returned,
      which (per the root-cause evidence above) can be well before the
      robot is actually ready — an explicit `ERR_NOT_CONFIGURED` ack is
      correct-and-observable but still does not EXECUTE the move. Added
      `_wait_for_boot_ready()` (polls `Telemetry.flags` bit 11,
      `kFlagEventBootReady`, before the first scenario runs) — the bench
      script's own pre-existing premature-send gap, now closed at its
      true origin (the host waiting for actual readiness) rather than
      papered over by having firmware execute early against un-probed
      devices.
- [x] **[stand-required, USB]** A full `move_protocol_bench.py` run shows
      zero unexplained enqueue-ack losses across all scenarios (this
      session's own baseline: 34/43-39/43 across five runs) — state
      whether the fix closed the additional intermittent losses too, or
      whether they are a separate, still-open defect (do not claim closure
      of a mechanism not actually confirmed fixed).
      **Not closed — confirmed separate and still open.** Post-fix tallies
      across 5 runs: 41/43, 41/43, 38/43, 42/43, 38/43 (better than the
      34-39/43 baseline, likely just noise/incidental — the boot race this
      ticket targets is gone, corr_id=1's scenario_distance_stop is now
      clean 5/5). Every remaining `[FAIL]` in these 5 runs is an
      `ack=None` on a scenario AFTER `scenario_distance_stop`
      (`scenario_angle_stop`, `scenario_chaining_seamless`,
      `scenario_err_full`, `scenario_stop_mid_motion`) — i.e. mid-session
      losses unrelated to the boot window this ticket fixes, matching
      `bench-move-commands-intermittently-never-reach-firmware.md`'s own
      prior finding that this is a pre-existing, separate bench-link gap
      (its root-cause-isolation section already proved it predates and is
      independent of the ack-ring). Left open for a future ticket/issue —
      not claimed fixed here.

## Testing

- **Existing tests to run**: full sim/unit suite (`uv run pytest`,
  `ctest` under `src/sim/build`); `src/tests/bench/move_protocol_bench.py`
  against real hardware.
- **New tests to write**: a sim/`motion_tests` regression test
  constructing the boot-window race deterministically (virtual clock —
  do not rely on real timing flakiness to reproduce it in CI).
- **Verification command**: `uv run pytest` (off-hardware); `uv run
  python src/tests/bench/move_protocol_bench.py --port
  /dev/cu.usbmodem2121102` (stand, 5 repeated runs).
