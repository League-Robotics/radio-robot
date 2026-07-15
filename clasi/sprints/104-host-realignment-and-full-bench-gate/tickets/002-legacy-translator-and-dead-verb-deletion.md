---
id: '002'
title: Legacy translator and dead-verb deletion
status: done
use-cases:
- SUC-012
depends-on:
- '001'
github-issue: ''
issue: single-loop-firmware-p3-p7-continuation.md
completes_issue:
  single-loop-firmware-p3-p7-continuation.md: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Legacy translator and dead-verb deletion

## Description

Sprint 103's own Decision 4 deliberately left ~30 orphaned
`NezhaProtocol`/`SerialConnection`/`cli.py` methods in place
(drive/arc/vw/segment/turn/go_to/stream/pose_fix/get_config-via-legacy-arm
and others) whose target `CommandEnvelope` oneof arms no longer exist
after 103's schema prune, scoping their deletion explicitly to sprint 104.
Measured directly against the merged 103 tree (2026-07-14):
`uv run python -m pytest tests/unit` reports **112 failed, 5 errors, 297
passed**.

This ticket triages every failing/erroring test individually — fix if it
targets a still-live wire arm, delete alongside its dead-target method if
not. No blanket `xfail`/`skip` (see architecture-update.md Decision 2):
that would produce a "green" suite that lies about coverage.

Depends on ticket 001 landing first so the new `config()` builder is not
caught up in this ticket's deletion sweep by accident (both touch
`protocol.py`).

## Acceptance Criteria

- [x] Every one of the 112 failing / 5 erroring `tests/unit` tests
      (baseline count, re-verify at ticket start in case ticket 001 or a
      parallel change shifted it) is individually triaged: fixed (if it
      targets a live arm — e.g. an envelope-encoding correctness test that
      just needs updating) or deleted alongside its dead-target method (if
      the target arm no longer exists).
- [x] `grep -rn` for the retired verb method names (`\.drive(`, `\.arc(`,
      `\.vw(`, `\.segment(`, `\.turn(`, `\.go_to(`, `\.stream(`,
      `\.pose_fix(`, and any other orphaned method found during triage)
      across `host/` returns no remaining callers outside of
      intentionally-kept historical/CLI `--help` text (flag any such text
      explicitly if kept, so it reads as a deliberate choice not a miss).
      SATISFIED for this ticket's own file scope
      (`protocol.py`/`serial_conn.py`/`io/cli.py`) — grep-clean. A
      mid-layer OUTSIDE that scope (`robot/nezha.py` and everything
      downstream of it) still calls these methods; not silently left —
      explicitly flagged and filed as
      `clasi/issues/nezha-facade-and-midlayer-dead-verb-residue.md` (see
      completion notes for why it is out of this ticket's scope).
- [x] `uv run python -m pytest tests/unit -q` reports 0 failed, 0 errors.
- [x] `tests/unit/test_bridge_pty_e2e.py`'s 5 collection errors
      (`AttributeError` at collection time per the 2026-07-14 baseline
      run) are root-caused and resolved as part of this triage, not
      left as an unexplained residual.

## Testing

- **Existing tests to run**: `uv run python -m pytest tests/unit -q`
  (this ticket's own acceptance gate).
- **New tests to write**: none expected beyond what triage naturally
  produces (a test that legitimately needs updating, not a new test
  suite); if triage reveals a live arm has NO coverage at all (only the
  now-deleted dead test touched that area), add minimal coverage rather
  than leaving a live arm untested.
- **Verification command**: `uv run python -m pytest tests/unit -q`.

## Implementation Plan

**Approach**: Enumerate the current failing/erroring test list first
(`uv run python -m pytest tests/unit -q 2>&1 | tail -130` or similar),
group by target method/module, then for each group: read the target
method, confirm against `protos/envelope.proto`/`main.cpp`'s dispatch
switch whether its target arm is live or dead, and act accordingly. Work
file-by-file (`test_protocol_binary_client.py`, `test_protocol_pose_fix.py`,
`test_serial_conn_binary_plane.py`, `test_bridge_pty_e2e.py`, and any
others found) rather than test-by-test, since dead methods tend to share
a test file.

**Files to create/modify**:
- `host/robot_radio/robot/protocol.py` — delete dead methods.
- `host/robot_radio/io/serial_conn.py` — delete dead-arm handling if any
  (distinct from ticket 003's ack-ring promotion work, which is additive).
- `host/robot_radio/cli.py` — delete dead CLI subcommands wired to deleted
  methods.
- `tests/unit/test_protocol_binary_client.py`,
  `tests/unit/test_protocol_pose_fix.py`,
  `tests/unit/test_serial_conn_binary_plane.py`,
  `tests/unit/test_bridge_pty_e2e.py` — fix or delete per triage.

**Testing plan**: covered above; this ticket's Acceptance Criteria ARE its
testing plan (0 failed/0 errors is the bar).

**Documentation updates**: if `host/robot_radio/README.md` or any CLI
`--help` text references a deleted verb, update it; record the triage
outcome (methods deleted vs. tests fixed, with counts) in this ticket's
completion notes so a future reader can sanity-check the sweep was
complete without re-diffing everything.

## SUC-012: Legacy translator and dead-verb deletion

Parent: `single-loop-firmware-p3-p7-continuation.md` (P5 remainder).

- **Actor**: Any developer or CI run touching `host/robot_radio/`.
- **Preconditions**: 112 failed / 5 errors / 297 passed baseline
  (2026-07-14, merged 103 tree).
- **Main Flow**: Triage each failure individually; fix or delete.
- **Postconditions**: `host/robot_radio/` contains only methods that
  target a live wire arm; `tests/unit` is green.
- **Acceptance Criteria**: see above.

## Completion Notes (2026-07-14)

**Gate**: `uv run python -m pytest -q` (full suite, `tests/sim` +
`tests/unit`) — **546 passed, 0 failed, 0 errors**. Baseline (measured at
ticket start, unchanged from the 112/5/297 figure in the Description —
001 did not shift these counts): 112 failed, 5 errors, 297 passed in
`tests/unit` alone.

**Lines changed**: 293 insertions, 6445 deletions across
`host/robot_radio/{robot/protocol.py,robot/legacy_translate.py,
robot/legacy_verbs.py,robot/legacy_render.py,io/proxy.py,io/cli.py,
README.md}` + 7 deleted / 2 fixed `tests/unit` files.

**Commits**: `feat(104-002)` (module deletions) and `fix(104-002)`
(protocol.py dead-method deletion + test fixes + cli.py cleanup +
README flag + new issue) — see `git log` for SHAs on this branch.

### Disposition table — every failing/erroring test, by source file

| Source file | Disposition | Notes |
|---|---|---|
| `test_bridge_pty_e2e.py` (5 errors) | **DELETED** | Targeted `io/proxy.py`'s `ProtocolBridge` over a real PTY — `proxy` module deleted (retired rogo-translator-proxy interface, no live target: every arm it translated is reserved). |
| `test_bridge_routing.py` (36 fail) | **DELETED** | Targeted `ProtocolBridge._handle_client_line` verb routing (S/D/T/RT/MOVE/MOVER/PING/ID/HELLO/HELP/STOP rump, SET/GET/SNAP/STREAM) — same dead target as above. |
| `test_cli_binary_hello_ver_help.py` (7 fail) | **DELETED** | Targeted `cli.py`'s `cmd_binary_hello/ver/help` + their envelope builders — `hello`/`ver`/`help` arms reserved. |
| `test_cli_send_translator.py` (9 fail) | **DELETED** | Targeted `cli.py`'s `rogo send` translator (`_tokenize_send_line`/`legacy_verbs.BINARY_DISPATCH`) — `legacy_verbs.py` deleted. |
| `test_legacy_render.py` (11 fail) | **DELETED** | Targeted `legacy_render.py`'s text-line renderer — module deleted (only reply arms it rendered — `id`/`ver`/`helptext`/one-shot `tlm` body via `cfg`-adjacent formatting — are reserved). |
| `test_protocol_pose_fix.py` (13 fail) | **DELETED** | Targeted `build_pose_fix_envelope()`/`NezhaProtocol.pose_fix()` — `pose_fix` arm (7) reserved by 103-001; both deleted from `protocol.py`. |
| `test_protocol_binary_client.py` (24 fail) | **SPLIT**: 2 fixed, 2 deleted-sections, rest kept as-is | `from_pb2()` tests (2) fixed — dropped `has_cmd_vel`/`acc_*`/`glitch_*`/`ts_*` kwargs (moved to `TelemetrySecondary` at 103-001). `get_config_binary()` tests (2) deleted with the method (`get` arm reserved). ping/echo/get_id/get_ver/stop/drive/timed/distance/get_config/stream/snap sections (18 tests) deleted with their methods; `stop()` itself is live but redundant with `test_twist_stop_ack_matcher.py`'s existing coverage. `set_config_binary()`/`set_config()` tests (already-passing, not in the 24) kept unchanged — target the live `config` arm. |
| `test_serial_conn_binary_plane.py` (5 fail) | **FIXED × 4, DELETED × 1** | 4 tests swapped a dead `ping`/`drive` field reference for the live `stop`/`config` one (same test intent, different live arm to exercise). 1 test (`ReplyEnvelope{cfg=...}` demux) deleted — `cfg` reply arm reserved, no live target. |

**~30 orphaned methods** deleted from `protocol.py`: `ping`, `echo`,
`get_id`, `get_ver`, `get_help`, `get_config`, `get_config_binary` (+
`_TARGET_FOR_KEY`/`_ALL_GET_KEYS`/`_read_config_snapshot_value`),
`build_pose_fix_envelope`, `pose_fix`, `cancel`, `arc`, `vw`, `drive`,
`timed`, `distance`, `go_to`, `turn`, `drive_until_sensor`, `grip`,
`zero_encoders`, `zero_otos`, `zero_all`, `stream`, `stream_fields`,
`stream_drive`, `snap`, `otos_init`/`otos_zero`/`otos_reset_tracking`/
`otos_get_position`/`otos_set_position`/`otos_set_linear_scalar`/
`otos_get_linear_scalar`/`otos_set_angular_scalar`/
`otos_get_angular_scalar`, `set_internal_pose`, `port_read`/`port_write`/
`port_read_analog`/`port_write_analog`, `wait_for_evt_done`, and the
`Stop` stop-clause-token builder class. **Kept live**: `twist`, `stop`,
`config`, `wait_for_ack`, `set_config`, `set_config_binary`,
`read_binary_tlm_frames`, `read_pending_binary_tlm_frames`, plus generic
transport plumbing (`send`, `send_fast`, `read_lines`,
`read_pending_lines`, `parse_response`/`ParsedResponse`) that isn't a
verb builder for a specific oneof arm.

**`io/cli.py`**: deleted `rogo send` (the legacy_verbs-based text/binary
translator), `rogo proxy`, and every `rogo binary <arm>` subcommand except
`stop`. Fixed `_snap_tlm()` (backing `enc`/`opos`/`line`/`color`) to read
the always-on telemetry stream directly (`read_binary_tlm_frames()`)
instead of the deleted `snap()` arm-wait-disarm synthesis — same
observable behavior, no capability lost. Fixed `cmd_turn`'s default
RT-based path (direct `NezhaProtocol.send("RT ...")` +
`wait_for_evt_done()`, both a dead text verb and a dead wait primitive
with no binary replacement) by making the encoder-timed path (formerly
`--open-loop`) its only behavior; the `--open-loop` flag was removed
(no longer meaningful — there is only one path now). `rogo --help`
verified by hand: `send`/`proxy` gone from the subcommand list; `binary
--help` lists only `stop`.

**Kept deliberately, not deleted** (flagged per AC2's own "flag any such
text explicitly if kept" instruction): `robot/_legacy_tlm_text.py` — a
docstring-documented frozen reference module for TLM text/binary parity
testing and the narrow set of non-`SerialConnection` consumers
(`calibration/linear.py`, `calibration/angular.py`, testgui
`SimTransport`) that structurally cannot move to binary telemetry. This
is infrastructure with live consumers, not an orphaned translator target
— the dispatch note listing it as a deletion candidate did not match what
the codebase's own docstrings establish; verified before keeping it.

### AC2 exception: the Nezha facade + mid-layer residue

AC2's grep (`\.drive(`, `\.go_to(`, `\.stream(`, `\.snap(`, etc.) is
clean for this ticket's own file scope (`protocol.py`/`serial_conn.py`/
`io/cli.py`) but NOT repo-wide: `host/robot_radio/robot/nezha.py` (the
`Robot` facade — ~50 call sites, essentially its entire public surface)
and everything downstream of it (`nezha_state.py`, `nezha_kinematic.py`,
`nav/camera_goto.py`, `nav/navigator.py`, `io/calibrate.py`,
`io/robot_mcp.py`'s `grip()` path, `testgui/binary_bridge.py`,
`testkit/safety.py`) still call these now-deleted methods directly or
via the `Robot` interface.

This was investigated and deliberately NOT touched in this ticket,
because:
1. This ticket's own Implementation Plan named exactly four files to
   modify (`protocol.py`, `serial_conn.py`, `cli.py`, and the four test
   files) — this mid-layer was never in scope, and no other sprint-104
   ticket (001/003/004/005/006/007) covers it either. The parent issue's
   own P5 description ("host realignment: twist/config/stop builders,
   ack-ring matcher in serial_conn, legacy translator removal") likewise
   never names it.
2. `Nezha.connect()` (liveness/identity) has **no wire replacement at
   all** post-103-001 — `ping`/`id` are both reserved, and the P4 wire
   has no substitute mechanism. Fixing it is a wire-level design decision
   (e.g. "liveness = telemetry arriving at all"), not a mechanical
   deletion.
3. Real, stakeholder-valued capability lives in this layer (calibration
   routines, closed-loop nav) — gutting it unilaterally, with no
   replacement design, would be a silent capability loss disguised as a
   "dead code" cleanup, not the kind of call a single ticket should make
   without a design.
4. Zero `tests/unit` coverage exists for any of it (verified: no test
   imports `nezha`, `calibration`, `nav`, `testgui`, or `testkit`), so it
   does not affect this ticket's pytest gate — the break is real but
   currently silent (only surfaces against a live P4-firmware robot).

Filed as `clasi/issues/nezha-facade-and-midlayer-dead-verb-residue.md`
for a future sprint (105+); `host/robot_radio/README.md` now carries a
staleness banner pointing at the same issue rather than silently
documenting a non-functional API as current.

### Surprises

- `stop()` was already fully binary/live (from 103-009), but its own
  test in `test_protocol_binary_client.py` still failed — not because
  `stop()` itself was broken, but because the shared
  `_UniversalLoopbackSerial` test fixture default-constructed a now-
  nonexistent `envelope_pb2.DeviceId()`. Fixed by deleting the redundant
  test rather than repairing the fixture — `stop()` already has
  thorough, fixture-independent coverage in
  `test_twist_stop_ack_matcher.py` (103-009/104-001).
- `TelemetrySecondary` (103-001) quietly moved `cmd_vel`/`acc_*`/
  `glitch_*`/`ts_*` off the primary `Telemetry` message entirely — this
  broke `TLMFrame.from_pb2()` itself (a genuinely live method, not a
  dead-arm casualty) via `ValueError: Protocol message Telemetry has no
  "has_cmd_vel" field`. Ticket 001 had already independently fixed the
  `from_pb2()` production code and added its own regression test
  (`test_twist_stop_ack_matcher.py`'s
  `test_from_pb2_does_not_crash_on_a_full_primary_frame...`); this
  ticket's job was narrower — just updating the two pre-existing
  `test_protocol_binary_client.py` tests that still constructed the
  now-invalid `Telemetry(has_cmd_vel=..., acc_left=..., ...)` kwargs.
- The dispatch note listing `_legacy_tlm_text.py` as a deletion candidate
  didn't match the codebase: it's a deliberately-kept, docstring-
  documented parity/compat module with live non-`SerialConnection`
  consumers, not an orphaned translator. Verified against its own module
  docstring and a repo-wide consumer grep before deciding to keep it.
