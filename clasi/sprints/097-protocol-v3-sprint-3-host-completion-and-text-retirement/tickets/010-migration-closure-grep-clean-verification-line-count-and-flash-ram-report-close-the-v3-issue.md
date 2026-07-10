---
id: '010'
title: 'Migration closure: grep-clean verification, line-count and flash/RAM report,
  close the v3 issue'
status: open
use-cases: [SUC-011]
depends-on: ['009']
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Migration closure: grep-clean verification, line-count and flash/RAM report, close the v3 issue

## Description

This is the closing ticket of the 3-sprint protocol-v3 program
(`protocol-v3-schema-driven-binary-command-plane-protobuf.md`, sprints
095/096/097). It does not modify production code — it verifies the
migration is actually finished and records the numbers the issue itself
asks for.

1. **Grep-clean verification** — confirm every deletion target listed in
   the issue's "What gets deleted" section and `sprint.md`'s Success
   Criteria is gone, with NO dangling references (no stray `#include`, no
   forward declaration, no comment claiming a deleted symbol is still
   live): the six motion parse/handle pairs (S/D/T/RT/MOVE/MOVER), `ECHO`/
   `VER`, `ParsedCommand`, `config_commands.{h,cpp}` in full, text
   STREAM/SNAP + `buildTlmFrame()`, host `parse_tlm`/`parse_cfg`. Confirm
   the explicitly-preserved families are STILL present and functioning:
   the five-verb rump (PING/ID/HELLO/HELP/STOP), R/TURN/G + stop-clause
   grammar, `otos_commands.cpp`/`pose_commands.cpp`, `dev_commands.cpp`,
   `handleTlm`/`QLEN`.
2. **`source/commands/` line count** — measure the final total and compare
   against the issue's own ~1,000-1,300-line estimate (down from the
   pre-095 ~4,900-line baseline); report the actual figure and explain any
   material deviation from the estimate (e.g. the preserved R/TURN/G/
   otos/pose/dev families account for lines the issue's estimate may not
   have anticipated being retained).
3. **Final flash/RAM report** — `.map` diff comparing the current build
   against the pre-095 baseline (095's own recorded starting point:
   image at 0x684B8 of 0x80000). Report whether the net change is
   negative (reclaimed), per the issue's own expectation (095's dual-stack
   peak was +12-15 KB; this sprint's own estimate was 15-30 KB reclaimed —
   report the ACTUAL number, do not assert the estimate). RAM: report the
   `.bss` delta; do not flag high RAM % on its own as a regression signal
   (this target runs at ~98% RAM by design).
4. Mark the issue resolved — this ticket carries `completes_issue: true`.

## Acceptance Criteria

- [ ] Grep-clean report produced and attached to this ticket's completion
      notes, covering every deletion target listed above with zero
      dangling references found (or each found reference fixed before
      closing).
- [ ] Every explicitly-preserved family confirmed present: five-verb rump,
      R/TURN/G + stop-clause grammar, `otos_commands.cpp`/
      `pose_commands.cpp`, `dev_commands.cpp`, `handleTlm`/`QLEN`.
- [ ] Final `source/commands/` line count recorded, compared against the
      issue's ~1,000-1,300-line estimate, with deviation explained.
- [ ] Final flash report recorded (`.map` diff vs. the pre-095 baseline),
      stating the actual net KB change (expect negative/reclaimed) — not
      an assertion of the issue's own estimate.
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
