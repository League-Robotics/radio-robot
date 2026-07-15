---
id: '006'
title: Bench script family rewritten to the binary twist/config/stop plane
status: done
use-cases:
- SUC-016
depends-on:
- '001'
- '003'
github-issue: ''
issue: single-loop-firmware-p3-p7-continuation.md
completes_issue:
  single-loop-firmware-p3-p7-continuation.md: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Bench script family rewritten to the binary twist/config/stop plane

## Description

`tests/bench/rig_dev.py`/`rig_soak.py` (and
`tests/bench/device_bus_bringup.py`,
`tests/unit/test_device_bus_bringup_bench.py`) assume the pre-103
segment/drive wire surface and `Devices::DeviceBus`-era bringup image —
both retired by sprint 103 (103's own Impact on Existing Components
section flags this explicitly as deferred to sprint 104). This ticket
rewrites the interactive and soak bench scripts onto
`twist`/`config`/`stop` + the hardened ack-ring matcher, and resolves the
`DeviceBus`-era bringup script (rewrite against `Preamble`, 103's
replacement, or retire with a documented reason).

Depends on ticket 001 (complete command surface — `config()` needed for
any bench script that wants to exercise config application) and ticket
003 (hardened, shared ack-ring matcher + `TelemetrySecondary` — bench
scripts read telemetry directly, not just through `NezhaProtocol`'s
convenience methods).

## Acceptance Criteria

- [x] `rig_dev.py` drives the rig interactively over the binary plane:
      connect, send twist/config/stop, observe ack + telemetry, matching
      its pre-103 interactive-session purpose but against the new wire
      surface.
- [x] `rig_soak.py` runs a sustained twist/stop loop over the binary
      plane, logging: TLM drop rate, fault/event bits over time, encoder
      motion per commanded twist (sanity-checking the plant responds, not
      just that acks arrive). Must run against both direct USB and the
      radio relay (a `--relay`/`--serial` style flag, matching the
      project's existing bench-script convention per
      `.claude/knowledge/relay-transport-and-stand-vs-floor.md` and
      similar).
- [x] `device_bus_bringup.py`/`test_device_bus_bringup_bench.py` are
      either rewritten against `Preamble` (103's boot-sequencing
      replacement for `DeviceBus`) with equivalent bringup diagnostics, OR
      retired with a documented reason in this ticket's completion notes
      (e.g. "no equivalent diagnostic need exists post-103 because X") —
      a ticket-time call per architecture-update.md's own flagged
      uncertainty, not pre-decided in the plan.
- [x] Both rewritten scripts are exercised at least once against real
      hardware as part of this ticket's own verification (not deferred
      entirely to ticket 007) — a smoke-level run, distinct from ticket
      007's sustained soak run.

## Testing

- **Existing tests to run**: none directly (these are bench scripts, not
  library code with their own pytest suite) — but any shared helper code
  extracted for both scripts should get unit coverage.
- **New tests to write**: if `rig_dev.py`/`rig_soak.py` share a helper
  module (e.g. a common connect/arm/send-loop function), add unit
  coverage for that helper in isolation (mocked transport), matching the
  project's existing bench-script testing conventions.
- **Verification command**: a real bench session per Acceptance Criteria
  (`.claude/rules/hardware-bench-testing.md` — robot on the stand, wheels
  off the ground); no `pytest` gate substitutes for this.

## Implementation Plan

**Approach**: Read the existing `rig_dev.py`/`rig_soak.py`/
`device_bus_bringup.py` first to understand what operator-facing behavior
must be preserved (connection flow, logging format, CLI flags) versus
what is purely wire-surface plumbing that must change. Reuse
`NezhaProtocol`/`serial_conn.py` from tickets 001/003 rather than
reimplementing envelope construction or ack matching in the scripts
themselves.

**Files to create/modify**:
- `tests/bench/rig_dev.py` — rewritten.
- `tests/bench/rig_soak.py` — rewritten.
- `tests/bench/device_bus_bringup.py`,
  `tests/unit/test_device_bus_bringup_bench.py` — rewritten or retired.

**Testing plan**: covered above.

**Documentation updates**: update any `tests/bench/README.md` (or
equivalent) describing script usage/flags to match the new invocation;
record the bringup-script disposition decision (rewritten vs. retired) in
this ticket's completion notes.

## SUC-016: Bench script family rewritten to the binary twist/stop plane

Parent: `single-loop-firmware-p3-p7-continuation.md` (P6).

- **Actor**: Bench operator running `rig_dev.py`/`rig_soak.py`.
- **Preconditions**: Scripts target the retired pre-103 wire surface and
  `DeviceBus` bringup image.
- **Main Flow**: Rewrite onto the binary plane; retire/rewrite the
  bringup script.
- **Postconditions**: No bench tool targets deleted wire arms or the
  deleted `DeviceBus`.
- **Acceptance Criteria**: see above.

## Completion Notes

### Disposition table

| Script | Disposition | Notes |
|---|---|---|
| `tests/bench/rig_dev.py` | Rewritten | `Rig` class now wraps `SerialConnection`/`NezhaProtocol` (`Rig.open()`/`twist()`/`stop()`/`config()`/`wait_for_ack()`/`read_tlm()`/`read_secondary_tlm()`/`close()`), plus pure helpers `waveform()` (kept, unchanged shape) and `secondary_to_dict()` (new — `TelemetrySecondary` adapter). `main()` is a scripted smoke sequence (connect → twist(v_x) → twist(omega) → stop → config → secondary telemetry), Result-checklist style matching `twist_drive.py`/`dev_exercise.py`. |
| `tests/bench/rig_soak.py` | Rewritten | Sustained twist/stop loop (smooth sine `v_x`/`omega` via `rig_dev.waveform()`, explicit `stop()` segments every 6s, `--relay`/direct-USB flag matching `gamepad_teleop.py`'s convention). Logs TLM drop rate, new fault/event bits, encoder-responsiveness-per-commanded-interval, ack coverage (informational only — see below). |
| `tests/bench/device_bus_bringup.py` + `tests/unit/test_device_bus_bringup_bench.py` | **Retired** (deleted, not rewritten) | The bring-up firmware image this script targeted (`source/devices/bringup_main.cpp`, `codal.devicebus.json`) was **already deleted** by commit `72d8be7e` ("feat(102-005): delete Elite plumbing + banner-only stub main") — confirmed via `git log --all -- '**/bringup_main*'`; only build-cache artifacts remain, no source. There is no firmware image left to rewrite this script against. `Preamble` (103's `DeviceBus` replacement — `source/app/preamble.{h,cpp}`) is a boot-time device-*detection sequencer* only (`present()`/`connected()` accessors), with no `DEV` command surface at all — not an equivalent bring-up test harness, so "rewrite against Preamble" is not a coherent option. The diagnostic need this script's 5 gates served (dual-motor pipelining, cycle-latency proxy, reversal-stress armor, serial burst-loss, flash/RAM) is superseded by `rig_soak.py`'s own dual-transport sustained loop (this ticket) plus ticket 007's soak gate — no equivalent standalone bring-up-image diagnostic is needed post-103. |

### Downstream breakage (out of this ticket's explicit scope, flagged not fixed)

`tests/bench/rig_drive.py`, `rig_stress.py`, and `otos_drift.py` all `import Rig` (and, for `rig_drive.py`/`otos_drift.py`, `SERVO_PIN`) from the pre-rewrite `rig_dev.py` and drive it via the retired DEV grammar (`rig.cmd("M <port> ...")`, `rig.servo(...)`, `rig.stream(...)`, `ODO SETPOSE`) — none of which exist on the new `Rig` class or the P4 wire at all (no per-port addressing, no `SERVO` verb, no `ODO SETPOSE` arm). This ticket's own Files-to-modify list names only `rig_dev.py`/`rig_soak.py`/`device_bus_bringup.py` (+ test) — neither the ticket text nor `architecture-update.md` mentions these three scripts — so they are left as-is, now broken at import. Not rewritten here per "keep changes focused on ticket scope." Filed nowhere yet as a fresh issue — recommend a follow-up ticket/issue to retire or reimagine them (their underlying capability — independent per-motor setpoints + a 360° OTOS servo sweep — has no equivalent on the binary plane at all, so "rewrite" may not be possible; likely retirement, mirroring `device_bus_bringup.py`'s own disposition).

### Surprise: intermittent ack-ring delivery gap (hardware finding, not a script bug)

Hardware verification (`/dev/cu.usbmodem2121102`, direct USB, robot on the stand)
surfaced a real, pre-existing, non-deterministic gap: `wait_for_ack()` intermittently
returns `None` for an individual `corr_id` even when well-separated from other
commands and given generous timeouts (tested up to 2000ms) — confirmed, via ad-hoc
diagnostics using bare `NezhaProtocol.twist()`/`wait_for_ack()` (no `rig_dev.py`/
`rig_soak.py` code involved), to be independent of anything this ticket wrote. Full
characterization, evidence, and follow-up direction filed as
`clasi/issues/ack-ring-intermittent-delivery-gap.md`. Key finding: the miss rate is
much worse for *discrete, well-separated* single commands (`rig_dev.py`'s own smoke-
test style: typically 5-7/8 checks pass, run to run) than for `rig_soak.py`'s
*continuous* reissue loop (measured 0.65% ack loss, 0.00% TLM drop, 97% encoder-
responsiveness over a clean 25s run) — so `rig_soak.py` deliberately does NOT gate
pass/fail on ack loss (see its own module docstring/`ACK_LOSS` handling), only on TLM
drop rate, new fault bits, and encoder responsiveness, all of which are unaffected by
this gap. `rig_dev.py`'s smoke script still records each ack check individually (with
one bounded retry) so the gap is visible in its own output, but a smoke run's overall
exit code is expected to occasionally show 1-3 ack-check misses — this reflects real,
already-understood hardware behavior, not a defect in the rewritten scripts.

### Hardware verification evidence

- `rig_dev.py` (`--v-x 100 --omega 0.6 --duration 700 --watch 0.6`): representative
  clean run — 7/8 checks passed (`connect()`, both `twist()` acks + confirmed encoder
  movement, `stop()` ack, secondary telemetry all PASS; `config()`'s ack missed that
  run — see finding above, `config()` independently confirmed to ack correctly,
  `ok=False, err_code=ERR_UNIMPLEMENTED` matching `source/main.cpp`'s documented
  `CmdKind::CONFIG` behavior, in an isolated diagnostic).
- `rig_soak.py` (`--duration 25`, direct USB): PASS — 155 twist/stop commands sent (3
  explicit stop segments), 347 primary frames, 0.00% TLM drop rate, 0.65% ack loss
  (informational), zero new fault bits, 97% encoder-responsiveness (140/144 commanded
  intervals), 127 secondary-telemetry samples. `--relay` dual-transport sustained
  verification is ticket 007's own job (this ticket's AC4 only requires "at least once
  against real hardware," not both transports).

### Tests

`uv run python -m pytest`: 561 passed (down from the pre-ticket baseline of 574: -28
for the deleted `test_device_bus_bringup_bench.py`, +15 new in
`tests/unit/test_rig_dev.py` covering `waveform()`, `secondary_to_dict()`, and every
`Rig` method's forwarding contract against fake `conn`/`proto` objects).
