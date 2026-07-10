---
id: '010'
title: 'Migration closure: grep-clean verification, line-count and flash/RAM report'
status: open
use-cases: [SUC-011]
depends-on: ['009']
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Migration closure: grep-clean verification, line-count and flash/RAM report

## Description

**REWRITTEN — see `architecture-update-r2.md` Decision 9 (supersedes r1's
"partial retirement" framing).** Under Decision 9, tickets 006/007/008
gut the firmware text plane unconditionally — this is now the FULL
firmware-side closure the original issue asked for, not the deferred,
near-zero outcome r1 produced. `completes_issue` stays `false` on this
ticket (team-lead's own explicit call, unchanged by this revision — the
team-lead decides the issue's final resolution at sprint close, not this
ticket).

1. **Grep-clean verification** — confirm every deletion target from
   tickets 006/007/008 is gone, with NO dangling references (no stray
   `#include`, no forward declaration, no comment claiming a deleted
   symbol is still live): the six motion parse/handle pairs (S/D/T/RT/
   MOVE/MOVER), `QLEN`, `R`/`TURN`/`G` + the stop-clause text grammar,
   `StreamingDriveWatchdog`, `ECHO`/`VER`/`HELP` (and `ID` per the rump
   outcome), `ParsedCommand`, `config_commands.{h,cpp}` in full, text
   `STREAM`/`SNAP` + `handleTlm` + `buildTlmFrame()`. Confirm the
   text safety rump matches ticket 006's ACTUAL shipped outcome (3-verb
   default or Eric's confirmed override — check ticket 006's own
   completion notes, do not assume). Confirm the untouched-for-different-
   reasons families remain present: `otos_commands.cpp`/
   `pose_commands.cpp`, `dev_commands.cpp`. Confirm the `rogo` translator
   proxy (ticket 004) exists and its extended `legacy_translate.py`
   covers every gutted verb.
2. **`source/commands/` line count** — measure the final total and compare
   against the issue's own ~1,000-1,300-line estimate (down from the
   pre-095 ~4,900-line baseline); report the actual figure. Given
   Decision 9's fuller gut (including R/TURN/G/QLEN/handleTlm/the stop-
   clause grammar, which r1's own estimate assumed would stay), the
   actual figure may come in BELOW the issue's original estimate — report
   whichever direction it goes, do not force-fit the number to the
   estimate.
3. **Final flash/RAM report** — `.map` diff comparing the current build
   against the pre-095 baseline (095's own recorded starting point: image
   at 0x684B8 of 0x80000). **Expect a BIG reduction this time** — the
   full text motion/config/telemetry/liveness surface is gone, not just
   `ParsedCommand`. Report the actual number against both the pre-095
   baseline and the issue's own "15-30 KB reclaimed" estimate; a bigger
   reduction than the estimate is plausible (Decision 9's gut is broader
   than the original issue's own dual-stack-only framing assumed) and
   should be reported as such, not treated as suspicious. RAM: report the
   `.bss` delta; do not flag high RAM % on its own as a regression signal
   (this target runs at ~98% RAM by design).
4. **Record the accepted breakage window** — list every host tool
   confirmed broken by tickets 006/007/008 (TestGUI's command panel,
   `robot_mcp.py`, `calibration/linear.py`/`angular.py`,
   `gamepad_teleop.py`, bench demo scripts) and confirm each is either
   (a) still broken and tracked by
   `realign-host-tooling-to-gutted-four-verb-wire-surface.md`, or (b) has
   since been rewired to the `rogo` proxy — do not silently assume (a)
   without checking whether any rewiring happened between tickets landing
   and this closure ticket running.

## Acceptance Criteria

- [ ] Grep-clean report produced and attached to completion notes,
      covering every deletion target from 006/007/008 with zero dangling
      references found (or each found reference fixed before closing).
- [ ] The text rump's actual shipped size (per ticket 006's own
      resolution of the flagged open question) is confirmed present and
      correctly documented in `docs/protocol-v3.md` (ticket 009).
- [ ] `otos_commands.cpp`/`pose_commands.cpp`/`dev_commands.cpp` confirmed
      present, untouched (different-reason preservation, unaffected by
      Decision 9).
- [ ] `rogo` proxy (ticket 004) confirmed present and functional
      (re-run ticket 004's own acceptance tests as part of this closure
      pass, don't just check the file exists).
- [ ] Final `source/commands/` line count recorded, compared against the
      issue's ~1,000-1,300-line estimate, with the direction of any
      deviation explained (Decision 9's fuller gut vs. the estimate's
      original dual-stack-only assumption).
- [ ] Final flash report recorded (`.map` diff vs. the pre-095 baseline),
      stating the actual net KB change — expected to be a substantial
      reduction; report the actual number against the issue's own
      estimate without assuming either under- or over-shoot.
- [ ] Final RAM `.bss` delta recorded (informational; not treated as a
      pass/fail signal on its own).
- [ ] The accepted-breakage list (Description item 4) is recorded with
      each tool's current status (still broken / rewired).
- [ ] `tests/sim` is green at closing.
- [ ] `just build` (ARM) succeeds at closing.
- [ ] The bench gate this ticket does NOT cover is stated explicitly:
      full binary regression over serial + relay; typed `STOP` from a
      bare terminal halting a moving robot (or whatever the final rump
      verb is, per ticket 006's outcome); a combined TestGUI +
      gamepad-teleop session over radio (both currently broken pending
      proxy rewiring — note this explicitly, it is not achievable at this
      closure point without that follow-up work) — that consolidated
      session is the team-lead's, per `.claude/rules/hardware-bench-
      testing.md`, not this ticket's.
- [ ] `completes_issue` remains `false` on this ticket, unchanged by this
      revision — the team-lead decides the issue's final resolution.

## Implementation Plan

### Approach

1. Run the grep-clean sweep for every deletion target and every
   preservation target; compile the report.
2. `wc -l` across `source/commands/*.{h,cpp}`; compare to the issue's
   estimate; write the comparison.
3. Build (`just build-clean` or equivalent full rebuild) and diff
   `MICROBIT.map` against the pre-095 baseline recorded in 095's own
   architecture document.
4. Check each accepted-breakage tool's current status.
5. Write the closing summary, explicitly scoping out the hardware bench
   gate (now larger than before — it also needs the proxy exercised, not
   just the binary arms) as the team-lead's next step.

### Files to modify

- None (verification/reporting ticket; no production source changes
  expected).

### Testing plan

- `tests/sim` full run — must be green.
- `just build` (or `just build-clean`) — ARM build must succeed.
- Ticket 004's own proxy acceptance tests re-run as part of this
  closure's verification.
- The grep-clean sweep and line-count/flash measurements themselves ARE
  this ticket's testing/verification activity.

### Documentation updates

- This ticket's own completion notes carry the grep-clean report, the
  line-count comparison, the flash/RAM report, and the accepted-breakage
  status list — these are the artifacts the issue itself asks for at
  closure.
