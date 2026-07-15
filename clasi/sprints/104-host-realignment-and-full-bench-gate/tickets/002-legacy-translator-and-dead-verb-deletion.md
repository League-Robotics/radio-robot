---
id: '002'
title: Legacy translator and dead-verb deletion
status: open
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

- [ ] Every one of the 112 failing / 5 erroring `tests/unit` tests
      (baseline count, re-verify at ticket start in case ticket 001 or a
      parallel change shifted it) is individually triaged: fixed (if it
      targets a live arm — e.g. an envelope-encoding correctness test that
      just needs updating) or deleted alongside its dead-target method (if
      the target arm no longer exists).
- [ ] `grep -rn` for the retired verb method names (`\.drive(`, `\.arc(`,
      `\.vw(`, `\.segment(`, `\.turn(`, `\.go_to(`, `\.stream(`,
      `\.pose_fix(`, and any other orphaned method found during triage)
      across `host/` returns no remaining callers outside of
      intentionally-kept historical/CLI `--help` text (flag any such text
      explicitly if kept, so it reads as a deliberate choice not a miss).
- [ ] `uv run python -m pytest tests/unit -q` reports 0 failed, 0 errors.
- [ ] `tests/unit/test_bridge_pty_e2e.py`'s 5 collection errors
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
