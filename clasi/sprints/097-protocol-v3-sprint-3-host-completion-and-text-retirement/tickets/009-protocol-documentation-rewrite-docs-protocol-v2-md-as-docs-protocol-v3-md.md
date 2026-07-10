---
id: 009
title: 'Protocol documentation: pure-binary wire + rump + rogo proxy'
status: done
use-cases:
- SUC-010
depends-on:
- '004'
- '006'
- '007'
- 008
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Protocol documentation: pure-binary wire + rump + rogo proxy

## Description

**REWRITTEN — see `architecture-update-r2.md` Decision 9.** Rewrite
`docs/protocol-v2.md` (2252 lines) as `docs/protocol-v3.md`, describing
the actual, final wire surface once tickets 004/006/007/008 have landed
(now depends on 004 too, since the proxy is a first-class part of the
documented story, not an optional convenience):

- Envelope framing: `*B<base64(envelope_bytes)>\n`, the armor/dearmor
  rule, the 186-byte per-arm payload cap.
- Every implemented `CommandEnvelope`/`ReplyEnvelope` oneof arm: drive,
  segment, replace, stop, ping, echo, id, config, get, stream — field
  shapes, validation bounds (cite `protos/*.proto`, not restate every
  number by hand), and which Blackboard queue/Configurator path each
  reaches.
- **The text safety rump — final size determined by ticket 006's
  resolution of the flagged open question** (3-verb default STOP/PING/
  HELLO, or Eric's confirmed override) — hand-typeable, with `STOP`'s
  explicit safety-affordance framing (a human with a raw serial terminal
  and no host program can always halt the robot).
- **The `rogo` translator proxy (ticket 004) as the PRIMARY
  text-compatibility story** for everything the rump doesn't cover: how
  to start it, what socket it opens, which verbs it translates, and its
  explicit limitation (no binary arm exists for `R`/`TURN`/`G`, so the
  proxy returns a typed error for them, it cannot manufacture capability
  the firmware doesn't have).
- An explicit "off the wire entirely, not proxied" section naming
  `otos_commands.cpp`/`pose_commands.cpp` (preserved as sprint 098's
  transcription reference; 098 owns their binary `pose`/`otos` arms) and
  `dev_commands.cpp` (never had or needs a binary counterpart) — these are
  DIFFERENT from the gutted-but-proxied families (S/D/T/RT/MOVE/MOVER/
  ECHO/VER/QLEN/handleTlm/SET/GET/STREAM/SNAP): the gutted families have a
  binary replacement AND proxy coverage; OTOS/pose/DEV have neither and
  were never touched by 097 at all.
- **An explicit note on the accepted breakage window**: TestGUI's manual
  command panel, `robot_mcp.py`'s calibration push, `calibration/
  linear.py`/`angular.py`, `gamepad_teleop.py`, and the bench demo
  scripts all currently point at the real robot connection directly, not
  the proxy — they are BROKEN against this firmware until individually
  rewired to the proxy socket (tracked by
  `realign-host-tooling-to-gutted-four-verb-wire-surface.md`). State this
  plainly; do not imply continuity that doesn't exist.

Mark `docs/protocol-v2.md` as superseded (a clear pointer at the top of
the file to `docs/protocol-v3.md`), do not delete it — history stays
reachable.

## Acceptance Criteria

- [x] `docs/protocol-v3.md` exists and documents every implemented binary
      arm (drive/segment/replace/stop/ping/echo/id/config/get/stream).
- [x] `docs/protocol-v3.md` documents the FINAL rump size exactly as
      ticket 006 shipped it (not the stale 5-verb or the stated 2-verb
      default — the actual outcome), and the `rogo` proxy as the primary
      text-compatibility path.
- [x] `docs/protocol-v3.md` explicitly names and explains
      `otos_commands.cpp`/`pose_commands.cpp`/`dev_commands.cpp` as
      off-the-wire-entirely (never gutted, never proxied, different
      category from the rump/proxy-covered families).
- [x] `docs/protocol-v3.md` explicitly states which currently-live host
      tools break against this firmware and are not yet rewired to the
      proxy (per ticket 006/007/008's own completion notes).
- [x] `docs/protocol-v2.md` carries a clear, prominent superseded-by
      pointer to `docs/protocol-v3.md` at the top of the file; the file is
      NOT deleted.
- [x] No section of `docs/protocol-v3.md` describes a verb tickets 006/
      007/008 deleted as if it were still live on the firmware TEXT
      plane (it may of course describe it as available via the proxy).
- [x] `tests/sim` stays green (documentation-only ticket; sanity check).

## Implementation Plan

### Approach

1. Read the final state of `source/commands/binary_channel.cpp`,
   `protos/envelope.proto`, `protos/motion.proto`, `protos/telemetry.proto`,
   `protos/config.proto`, and `motion_commands.cpp`/`system_commands.cpp`
   (post-006/007/008) as the source of truth for the FIRMWARE wire
   surface; read `legacy_translate.py`/the proxy implementation (ticket
   004) as the source of truth for the PROXY's translation coverage.
2. Draft `docs/protocol-v3.md`: envelope framing, every implemented arm,
   the final rump, the proxy as the primary compatibility story, the
   off-the-wire-entirely section, the accepted-breakage note.
3. Add the superseded-by banner to the top of `docs/protocol-v2.md`.

### Files to modify

- `docs/protocol-v3.md` (new)
- `docs/protocol-v2.md` (superseded banner only)

### Testing plan

- No code changes; `tests/sim` run as a sanity check.
- Manual cross-check: every verb named in `docs/protocol-v3.md` is
  grep-confirmed against the actual post-006/007/008/004 source tree (no
  stale claims about what's live on the wire vs. proxied vs. off-the-wire
  entirely).

### Documentation updates

- This ticket IS the documentation update.

## Completion Notes

- `docs/protocol-v3.md` written (11 sections + appendix): overview,
  envelope framing (§2), every `CommandEnvelope`/`ReplyEnvelope` arm
  (§3/§4, cites `protos/*.proto` rather than restating bounds), error
  taxonomy (§5), the confirmed 3-verb rump `STOP`/`PING`/`HELLO` (§6,
  grep-verified against `buildTable()`/`systemCommands()`/
  `motionCommands()`/`telemetryCommands()`), the `rogo proxy` PTY bridge
  as primary compatibility path incl. verb routing table and EVT
  synthesis (§7), the off-the-wire-entirely OTOS/pose/DEV families (§8),
  the accepted host-tooling breakage window (§9), and a v2-section →
  v3-status cross-reference table (§10).
- `docs/protocol-v2.md` got a superseded-by banner at the top; file not
  deleted.
- **Correction found during grep-cross-check** (this ticket's own
  acceptance criterion 6): the ticket's own description text (line ~50)
  groups `QLEN` with the "gutted-but-proxied" families
  (S/D/T/RT/MOVE/MOVER/ECHO/VER/QLEN/handleTlm/SET/GET/STREAM/SNAP).
  Reading `host/robot_radio/io/proxy.py` directly shows this is wrong:
  `QLEN` is in `_ALWAYS_UNSUPPORTED_VERBS` alongside `G`/`R`/`TURN`/`GRIP`
  — no binary `qlen` arm exists in `envelope.proto`, so the proxy answers
  it with a typed `ERR unsupported`, not a translation. `docs/protocol-v3.md`
  documents the verified behavior (QLEN alongside G/R/TURN/GRIP, §7.4/§7.6),
  not the ticket description's original grouping — flagged here per this
  ticket's own "grep-cross-check every verb you claim is live" instruction,
  not silently reconciled.
- Also verified and corrected while drafting: `docs/protocol-v2.md` §8's
  historical `TLM Frame Format` table (`line=`/`color=`/`wedge=`/
  `otos_health=`/`ekf_rej=`) describes an older `source_old/`-era wire
  format. This rebuilt tree's `source/telemetry/tlm_frame.cpp` never
  emitted `line=`/`color=` at all (line/color sensor HAL leaves are
  declared but never ticked — see
  `clasi/issues/restore-line-and-color-sensors-as-ticked-blackboard-devices.md`)
  and never had a `wedge=` token either — `docs/protocol-v3.md` §4/§7.6
  state this precisely rather than implying those fields were ever live
  on this tree's text wire before 097 gutted it.
- Verification: `uv run python -m pytest tests/sim -q` → **597 passed**
  in 83.52s (sanity, unaffected by a docs-only ticket). Every verb named
  in `docs/protocol-v3.md` was grep-confirmed against
  `source/runtime/command_router.cpp` (`buildTable()`),
  `source/commands/{system_commands,motion_commands,telemetry_commands}.{h,cpp}`,
  `source/commands/binary_channel.cpp`, `protos/envelope.proto`, and
  `host/robot_radio/io/proxy.py`/`robot/{legacy_verbs,legacy_render}.py`.
