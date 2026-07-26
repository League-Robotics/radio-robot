---
id: '001'
title: Fix silent first-MOVE-after-connect command loss (boot/comms race)
status: open
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

- [ ] Root cause confirmed with evidence (verbose `on_send`/`on_recv`
      logging and/or `Comms::malformedCount()`/`kFlagFaultCommsMalformed`
      inspection around the drop) — state definitively whether the bytes
      never arrived, arrived but failed to decode, or decoded but were
      discarded by the configuration-completeness/queue-readiness gate.
- [ ] Fix lands such that a `MOVE` arriving before the robot is fully
      ready is EITHER accepted and executed OR explicitly rejected with
      `ERR_NOT_CONFIGURED` (or an equivalent explicit error) — never
      silently dropped with no ack of any kind.
- [ ] **[off-hardware]** A sim/`motion_tests` regression test constructs
      the same race window (a `MOVE` arriving during `boot()`'s pump
      window, before `configured_` flips true) and asserts one of the two
      outcomes above, never a silent drop.
- [ ] **[stand-required, USB]** `move_protocol_bench.py`'s
      `scenario_distance_stop` acks and executes on the first `MOVE`
      after a fresh connect, 5/5 runs — positive evidence: 5 observed
      acks + 5 observed nonzero encoder deltas.
- [ ] **[stand-required, USB]** A full `move_protocol_bench.py` run shows
      zero unexplained enqueue-ack losses across all scenarios (this
      session's own baseline: 34/43-39/43 across five runs) — state
      whether the fix closed the additional intermittent losses too, or
      whether they are a separate, still-open defect (do not claim closure
      of a mechanism not actually confirmed fixed).

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
