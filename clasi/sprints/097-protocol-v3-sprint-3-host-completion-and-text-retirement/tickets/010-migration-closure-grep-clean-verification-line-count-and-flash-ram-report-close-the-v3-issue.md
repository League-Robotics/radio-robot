---
id: '010'
title: 'Migration closure: grep-clean verification, line-count and flash/RAM report,
  close the v3 issue'
status: open
use-cases: [SUC-011]
depends-on: ['009']
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Migration closure: grep-clean verification, line-count and flash/RAM report, close the v3 issue

## Description

**REVISED EXPECTATION — see `architecture-update-r1.md` Decision 8.**
Tickets 006/007/008 discovered that essentially every motion/config/
telemetry text family still has at least one live, unmigrated production
consumer (TestGUI's manual command panel and connect-time `STREAM`, the
MCP server's calibration push, `rogo turn`/`sync-cal`, two calibration
scripts, and several bench/demo scripts). Per the issue's own
consumer-migration rule, those tickets deleted NOTHING beyond
`ParsedCommand` (a zero-reference struct). **This ticket's flash-reduction
expectation drops accordingly: the issue's original "15-30 KB reclaimed"
estimate is NOT met this sprint** — the actual number will be whatever a
single dead struct is worth, effectively negligible. This is not a
shortfall to explain away; it is the correct, conservative outcome of
following the issue's own deletion rule once its real precondition
(consumers migrated) turned out not to hold yet. The remaining flash win
is real and still achievable, gated on
`realign-host-tooling-to-gutted-four-verb-wire-surface.md` landing first,
in a future sprint.

This is the closing ticket of the 3-sprint protocol-v3 program
(`protocol-v3-schema-driven-binary-command-plane-protobuf.md`, sprints
095/096/097) **as it stands after this revision**: host completion
(NezhaProtocol's SerialConnection-reachable surface) is done; firmware
text retirement of the motion/config/telemetry families is DEFERRED, not
completed. It does not modify production code — it verifies what was
actually finished and records the numbers the issue itself asks for,
honestly reflecting the partial outcome.

1. **Grep-clean verification** — confirm `ParsedCommand` (the ONE
   deletion target actually achieved) is gone, with NO dangling
   references. Confirm every OTHER originally-listed deletion target
   (S/D/T/RT/MOVE/MOVER, ECHO/VER, `config_commands.{h,cpp}`, text
   STREAM/SNAP + `buildTlmFrame()`) is STILL PRESENT, unregistered status
   unchanged from pre-097, per tickets 006/007/008's revised, conservative
   outcome — this is now an expected-preserved list, not a
   deletion-target list. Confirm host `parse_tlm`/`parse_cfg` are gone
   (ticket 003's achieved `SerialConnection`-reachable scope) and that
   `host/robot_radio/robot/_legacy_tlm_text.py` exists as the documented
   bridge for the four unswept consumers. Confirm the ORIGINALLY-preserved
   families remain: the five-verb rump (PING/ID/HELLO/HELP/STOP), R/TURN/G
   + stop-clause grammar, `otos_commands.cpp`/`pose_commands.cpp`,
   `dev_commands.cpp`, `handleTlm`/`QLEN`.
2. **`source/commands/` line count** — measure the final total. It will be
   close to the pre-095 ~4,900-line baseline, NOT the issue's
   ~1,000-1,300-line estimate (that estimate assumed full text-family
   deletion, which did not happen this sprint) — report the actual figure
   and state plainly that the gap versus the estimate is because firmware
   text retirement is deferred, not a measurement error.
3. **Final flash/RAM report** — `.map` diff comparing the current build
   against the pre-095 baseline (095's own recorded starting point:
   image at 0x684B8 of 0x80000). Report the actual net change — expect it
   to be roughly flat (095/096's dual-stack additions still present, only
   `ParsedCommand` removed), NOT the issue's original 15-30 KB reclaimed
   estimate. State this gap explicitly rather than implying the estimate
   was met. RAM: report the `.bss` delta; do not flag high RAM % on its
   own as a regression signal (this target runs at ~98% RAM by design).
4. Mark the issue resolved for the scope actually completed — this ticket
   carries `completes_issue: true` for the protocol-v3 issue's HOST
   COMPLETION goal; the closing notes must explicitly record that
   firmware text retirement of the motion/config/telemetry families
   remains open, owned by `realign-host-tooling-to-gutted-four-verb-wire-
   surface.md`'s now-updated scope, and that the protocol-v3 issue itself
   carries a forward note to that effect (see the issue's own updated
   Context section).

## Acceptance Criteria

- [ ] Grep-clean report produced and attached to this ticket's completion
      notes: `ParsedCommand` gone (the one achieved deletion); host
      `parse_tlm`/`parse_cfg` gone; zero dangling references for either.
- [ ] Every preserved family confirmed present and unregistered-status
      unchanged: the five-verb rump (PING/ID/HELLO/HELP/STOP), R/TURN/G +
      stop-clause grammar, `otos_commands.cpp`/`pose_commands.cpp`,
      `dev_commands.cpp`, `handleTlm`/`QLEN` (original preservation list,
      Decision 5/6/7) AND S/D/T/RT/MOVE/MOVER/ECHO/VER,
      `config_commands.{h,cpp}`, text STREAM/SNAP + `buildTlmFrame()`
      (newly-preserved this revision, Decision 8) — all present,
      byte-for-byte unchanged from pre-097.
- [ ] Final `source/commands/` line count recorded, compared against BOTH
      the issue's original ~1,000-1,300-line estimate (not met — explain
      why, citing Decision 8) and the pre-095 ~4,900-line baseline (should
      be close to unchanged, minus `ParsedCommand`).
- [ ] Final flash report recorded (`.map` diff vs. the pre-095 baseline),
      stating the actual net KB change — expected roughly flat (only
      `ParsedCommand` removed this sprint; the family-scale reduction is
      deferred), NOT the issue's original "15-30 KB reclaimed" estimate.
      State the gap explicitly; do not assert the estimate was met.
- [ ] Final RAM `.bss` delta recorded (informational; not treated as a
      pass/fail signal on its own).
- [ ] `tests/sim` is green at closing.
- [ ] `just build` (ARM) succeeds at closing.
- [ ] The bench gate this ticket does NOT cover is stated explicitly in
      the completion notes (full binary regression over serial + relay;
      typed `STOP` from a bare terminal halting a moving robot; a
      combined TestGUI + gamepad-teleop session over radio) — that is the
      team-lead's consolidated post-sprint session, per `.claude/rules/
      hardware-bench-testing.md` and this sprint's own Definition of Done,
      not this ticket's job.

## Implementation Plan

### Approach

1. Run the grep-clean sweep for every deletion target and every
   preservation target; compile the report.
2. `wc -l` across `source/commands/*.{h,cpp}`; compare to the issue's
   estimate; write the comparison.
3. Build (`just build-clean` or equivalent full rebuild, matching the
   project's established flash-measurement convention — see
   `.clasi/knowledge/` for any bench-gate gotchas already documented) and
   diff `MICROBIT.map` against the pre-095 baseline recorded in 095's own
   architecture document.
4. Write the closing summary, explicitly scoping out the hardware bench
   gate as the team-lead's next step.

### Files to modify

- None (verification/reporting ticket; no production source changes
  expected).

### Testing plan

- `tests/sim` full run — must be green.
- `just build` (or `just build-clean`) — ARM build must succeed.
- The grep-clean sweep and line-count/flash measurements themselves ARE
  this ticket's testing/verification activity.

### Documentation updates

- This ticket's own completion notes carry the grep-clean report, the
  line-count comparison, and the flash/RAM report — these are the
  artifacts the issue itself asks for at closure.
