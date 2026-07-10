---
id: '010'
title: 'Migration closure: grep-clean verification, line-count and flash/RAM report'
status: done
use-cases:
- SUC-011
depends-on:
- 009
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

- [x] Grep-clean report produced and attached to completion notes,
      covering every deletion target from 006/007/008 with zero dangling
      references found (or each found reference fixed before closing).
- [x] The text rump's actual shipped size (per ticket 006's own
      resolution of the flagged open question) is confirmed present and
      correctly documented in `docs/protocol-v3.md` (ticket 009).
- [x] `otos_commands.cpp`/`pose_commands.cpp`/`dev_commands.cpp` confirmed
      present, untouched (different-reason preservation, unaffected by
      Decision 9).
- [x] `rogo` proxy (ticket 004) confirmed present and functional
      (re-run ticket 004's own acceptance tests as part of this closure
      pass, don't just check the file exists).
- [x] Final `source/commands/` line count recorded, compared against the
      issue's ~1,000-1,300-line estimate, with the direction of any
      deviation explained (Decision 9's fuller gut vs. the estimate's
      original dual-stack-only assumption).
- [x] Final flash report recorded (`.map` diff vs. the pre-095 baseline),
      stating the actual net KB change — expected to be a substantial
      reduction; report the actual number against the issue's own
      estimate without assuming either under- or over-shoot.
- [x] Final RAM `.bss` delta recorded (informational; not treated as a
      pass/fail signal on its own).
- [x] The accepted-breakage list (Description item 4) is recorded with
      each tool's current status (still broken / rewired).
- [x] `tests/sim` is green at closing.
- [x] `just build` (ARM) succeeds at closing.
- [x] The bench gate this ticket does NOT cover is stated explicitly:
      full binary regression over serial + relay; typed `STOP` from a
      bare terminal halting a moving robot (or whatever the final rump
      verb is, per ticket 006's outcome); a combined TestGUI +
      gamepad-teleop session over radio (both currently broken pending
      proxy rewiring — note this explicitly, it is not achievable at this
      closure point without that follow-up work) — that consolidated
      session is the team-lead's, per `.claude/rules/hardware-bench-
      testing.md`, not this ticket's.
- [x] `completes_issue` remains `false` on this ticket, unchanged by this
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

## Completion Notes

**1. Grep-clean sweep — clean, with 3 dangling prose-comment references found
and fixed (comment-only, zero functional/behavioral change).**

Re-ran the full sweep for every deletion target from 006/007/008 across
`source/` (not just the three files ticket 006 originally flagged): the six
motion parse/handle pairs (S/D/T/RT/MOVE/MOVER), `QLEN`, R/TURN/G, the
stop-clause grammar (`parseStopClauseValue`/`collectStopClauses`/
`packStopKVs`/`kMaxStopConds`/`replyStopBadarg`), `ECHO`/`VER`/`HELP`/text
`ID`/`kEchoSchema`, `ParsedCommand`, `config_commands.{h,cpp}`, and
`handleStream`/`handleSnap`/`kStreamSchema`/`telemetryEmit`/`buildTlmFrame`/
`handleTlm`. **Zero remaining references anywhere** except legitimate
past-tense "DELETED (097-00X)" explanatory doc comments in the files that
used to host them, `binary_channel.cpp`'s own CURRENT binary
`handleEcho`/`handleId` functions (correctly reusing the same names for
their binary-plane counterparts), and `command_types.h`'s own deletion-note
comment (which necessarily names `ParsedCommand` to explain it was
deleted). `config_commands.h`/`.cpp` confirmed deleted
(`ls source/commands/config_commands.*` — no matches); every remaining
`config_commands` mention in `source/` is a "removed 097-007" past-tense
note, except the one deliberate documented exception at `dev_commands.h:93`
(ticket 007's own accepted AC-conflict resolution: `dev_commands.{h,cpp}`
byte-for-byte-untouched wins over that one grep-clean line).

**Found and fixed 3 genuinely dangling references** — all three claimed,
present tense, that the `StreamingDriveWatchdog` class (deleted outright by
097-006 as already-dead code, "fed by nothing... it fed the S-only
streaming-drive-silence timeout, and S no longer exists") is a live,
loop-owned instance:
- `source/runtime/blackboard.h:144-148`/`221` — "published every pass by
  the loop from its own SerialSilenceWatchdog/StreamingDriveWatchdog
  instances" plus `streamWatchdogWindow(In)`'s trailing comments naming the
  deleted text verb `SET sTimeout=` as the field's setter. Reworded:
  `devWatchdogWindow`'s `SerialSilenceWatchdog` half is left as-is (a
  `dev_commands.h`-preserved, untouched class, see caveat below);
  `streamWatchdogWindow(In)`'s own comments now state its
  `StreamingDriveWatchdog` consumer was deleted 097-006, the field is
  written only by the binary config `WATCHDOG` patch
  (`handleConfigWatchdog`, `binary_channel.cpp`), and it has no live
  consumer.
- `source/runtime/commands.h:61-68` (`MotionCommand::feedStreamWatchdog`'s
  doc comment) — claimed "Set ONLY by handleS()" (deleted 097-006) feeding
  "the loop's own loop-owned StreamingDriveWatchdog" (also deleted).
  Reworded to past tense, explaining both deletions and noting — matching
  `motion_commands.h`'s own precedent — that `Rt::MotionCommand`/
  `bb.motionIn` are now fully unreferenced plumbing, left in place as a
  documented future-cleanup vestige (architecture-update-r2.md Open
  Question 1) rather than removed by this ticket.
- `source/commands/binary_channel.cpp:352-358` (`handleConfigWatchdog`'s
  doc comment) — same `StreamingDriveWatchdog` present-tense claim;
  reworded, leaving the already-correct "config_commands.cpp/.h, removed
  097-007" framing on the same lines untouched.

**Caveat, flagged rather than silently expanded**: `dev_commands.h`'s own
`SerialSilenceWatchdog` documentation (lines 97-116) independently claims a
"loop-owned instance (main.cpp/sim_api.cpp)" that does not actually exist
in the CURRENT `main.cpp` (181 lines, zero watchdog code, confirmed by
grep) or in `tests/_infra/sim/sim_api.cpp` (whose own comments already say
"there is no watchdog left to feed here" / "no live StreamingDriveWatchdog
instance here to feed"). This drift predates 097 entirely (ticket 093's
"bare wheel driving" simplification removed the loop's watchdog
instantiation without updating `dev_commands.h`'s doc comment) and is
unrelated to `StreamingDriveWatchdog` or any 097 deletion target —
`dev_commands.{h,cpp}` is this ticket's own "confirmed present, untouched"
preserved family (AC 3), correctly left alone. Noted for visibility, not
fixed (out of this ticket's scope).

**2. Text safety rump**: confirmed present. `STOP` (`motion_commands.cpp:84`),
`PING`/`HELLO` (`system_commands.cpp:126-127`) are registered.
`docs/protocol-v3.md` §6 (ticket 009) documents this exact 3-verb rump,
grep-verified per that ticket's own completion notes.

**3. Preserved families**: `otos_commands.cpp`/`pose_commands.cpp`/
`dev_commands.cpp` all confirmed present and functionally untouched by
Decision 9. Precision on "untouched": `dev_commands.{h,cpp}` is
byte-for-byte identical to its pre-097 (`352d634b`) state (`git diff`
against that commit is empty). `pose_commands.{h,cpp}` has zero commits
touching it since pre-097. `otos_commands.{h,cpp}` received ONE
comment-only dangling-reference-cleanup commit (`a61ffb6e`, ticket 097-007's
own permitted grep-clean scope, rewording stale `config_commands.h`
mentions) — functionally identical, not literally byte-identical; the same
category of edit ticket 007's own AC explicitly allowed and this ticket's
own Description explicitly invites more of.

**4. `rogo` proxy**: confirmed present (`host/robot_radio/io/proxy.py`,
`robot/legacy_verbs.py`, `robot/legacy_render.py`) and re-verified
functional, not just file-existence-checked — re-ran ticket 004's own test
suite as this closure's own verification step:
`uv run python -m pytest tests/unit/test_cli_send_translator.py
tests/unit/test_legacy_render.py tests/unit/test_bridge_routing.py
tests/unit/test_bridge_pty_e2e.py -v` → **133 passed** (27+52+48+6,
matching ticket 004's own completion-notes breakdown exactly).

**5. `source/commands/` line count**: `wc -l source/commands/*.h
source/commands/*.cpp` → **3,700 lines total** (this ticket's own +3-line
comment expansion in `binary_channel.cpp` included; 3,697 immediately after
ticket 009 landed). Compared:
- Pre-095 baseline, re-verified fresh (not assumed from the issue's prose
  alone): `git show 1ed89ed5:source/commands/*.{h,cpp} | wc -l` at commit
  `1ed89ed5` (the commit immediately preceding sprint 095's own planning
  commit, `3249c94c`) → **4,927 lines**, matching the issue's own "~4,900"
  figure almost exactly.
- Issue's own estimate: "~4,900 → roughly 1,000-1,300 (rump + BinaryChannel
  + dispatch core)".
- **Actual final total, 3,700 lines, lands far ABOVE the estimate's
  1,000-1,300 range** (roughly 2.8x-3.7x higher) — the opposite direction
  from what this ticket's own Description flagged as plausible ("the actual
  figure may come in BELOW the issue's original estimate"). Root cause,
  explained rather than force-fit: the estimate's own parenthetical ("rump +
  BinaryChannel + dispatch core") does not appear to have budgeted for
  `otos_commands.cpp`/`pose_commands.cpp`/`dev_commands.cpp` (1,639 lines
  combined) remaining permanently in `source/commands/` — these were NEVER
  migration targets (different-reason preservation: `dev_commands.cpp` has
  no binary arm planned; `otos`/`pose` are sprint 098's own transcription
  reference) but they physically live in the same directory the estimate
  measured. **Excluding those three preserved files** (the estimate's own
  apparent intent), the "migrated + BinaryChannel + dispatch core" surface
  totals **2,061 lines** (`motion_commands` 156 + `system_commands` 202 +
  `telemetry_commands` 167 + `binary_channel` 625 + `command_processor` 660
  + `arg_parse` 251) — still roughly 1.6x-2x the estimate, likely because
  `command_processor.cpp` (660 lines, entirely untouched by 097 — shared
  dispatch machinery, never itself reduced) and `binary_channel.cpp` (625
  lines, ALL of it new code added by 095/096, never part of the pre-095
  baseline at all) both came in larger than the estimate's rough budgeting
  anticipated, on top of this codebase's own documentation convention
  (extensive per-decision rationale comments, visible throughout this very
  sweep) adding real line count a bare LOC estimate would not anticipate.
  The reduction itself is real and substantial: 4,927 → 3,700 is **1,227
  lines removed (24.9%)** — reported honestly in both directions per this
  ticket's own instruction, not force-fit to either the "below the
  estimate" prediction or the estimate's absolute range.

**6. Flash/RAM report**: `just build-clean` (ARM + host sim) succeeded.
Measured via the SAME methodology 095's own architecture doc used
("verified against `build/MICROBIT.map`'s `.data`/`.bss` load addresses")
— the true end-of-flash-image address (`.data`'s load address + size,
confirmed by `.tm_clone_table`/`.igot.plt`'s own zero-length entries
landing at that identical address):
- **Pre-095 baseline, re-verified by an actual clean build** (not assumed
  from the recorded hex number alone) in an isolated git worktree at commit
  `1ed89ed5`: image ends **0x684C0** (427,200 bytes) — matches the recorded
  `0x684B8` (427,192) within 8 bytes (build-environment/toolchain-patch
  noise, negligible). `arm-none-eabi-size`: `text=312512`.
- **Current (this ticket, full 097 gut landed)**: image ends **0x69794**
  (432,020 bytes). `arm-none-eabi-size`: `text=317332 data=140823
  bss=119824 dec=577979`.
- **Net delta vs. pre-095 baseline: +4,820 to +4,828 bytes (~+4.7 KB), an
  INCREASE, not the expected reduction.** Reported plainly per this
  ticket's own instruction ("report the actual number... without assuming
  either under- or over-shoot"; "report the actual figure whichever
  direction it goes") rather than force-fitting a reduction narrative. Root
  cause: sprints 095/096 added the ENTIRE binary/protobuf command plane
  (envelope codec, `BinaryChannel` oneof dispatch, base64 armor/dearmor —
  none of which existed in the pre-095, text-only baseline) as a dual-stack
  addition alongside the pre-existing text plane; 097's own three gutting
  tickets (006/007/008) reclaimed only **9,292 bytes** of that growth
  (-7,008 + 0 + -2,284 on the `text` column, matching those tickets' own
  recorded before/after numbers exactly) — not enough to fully offset what
  095/096 added (326,624 − 312,512 = 14,112 bytes grown pre-097). The new
  binary command plane, net of everything the old text motion/config/
  telemetry families used to cost, is simply larger than what it replaced.
- **Caveat on `arm-none-eabi-size`'s own "data" column**: per this
  project's own recorded finding, the `data` column is misleading on this
  linker script (it reports 140,823 bytes for a `.data` section whose own
  map entry is only 0x284 = 644 bytes) — this ticket did NOT use
  "text+data" as the flash-usage figure for that reason; the `text` column
  alone (which the map cross-check above confirms equals the true
  FLASH-region byte count almost exactly: 317,332 vs. a directly-computed
  318,868) is the trustworthy number, and is what the delta above is
  computed from.
- **RAM (`.bss`) delta: 0 bytes** (119,824 both before and after) — no
  runtime state added or removed, only code. RAM sits at 98.33% either way;
  per project convention this is normal for this target and not a
  regression signal on its own.

**7. Accepted-breakage list**: re-checked (not assumed) whether any
consumer has been rewired to the proxy since tickets 006/007/008/004
landed — **none has**. Grepped every named tool for `robot-pty`/
`ProtocolBridge`/proxy references:
- **TestGUI's manual command panel** (`testgui/commands.py`'s `COMMANDS`
  table) and its connect-time `"STREAM 50"` push (`testgui/transport.py:985`,
  `testgui/__main__.py:2189`) — still raw text, still broken.
- **`robot_mcp.py`** — still imports and calls
  `calibration.push.push_calibration` (raw text `SET`), still broken.
- **`calibration/linear.py`/`angular.py`** — no proxy references found;
  still on the raw-pyserial `RelaySerial`/`DirectSerial` path, still
  broken.
- **`gamepad_teleop.py`**, bench demo scripts (`dtr_drive_demo.py`/
  `random_segment_demo.py`) — no proxy references found; still broken.
- **`cli.py`'s `cmd_turn`** (non-`--open-loop` default) still sends raw
  text `RT` directly (`cli.py:785-800`); `_push_calibration`/
  `rogo sync-cal` still sends raw text `SET`/`OI`/`OL`/`OA`
  (`cli.py:110-`).
- The only `cli.py` hits for proxy-related strings are the `rogo proxy`
  subcommand's OWN implementation (`cmd_proxy`, ~lines 1599-1942) — the
  proxy's launcher itself, not a rewired consumer.
- All of the above are explicitly tracked by `clasi/issues/
  realign-host-tooling-to-gutted-four-verb-wire-surface.md`'s 2026-07-10
  update section, which names every one of these tools and already
  narrows its own scope (per ticket 004's completion notes) from "migrate
  to `NezhaProtocol` directly" to "point at the proxy's PTY path" —
  confirmed current, not stale.

**8. Test summaries**:
- `just build-clean` (ARM + host sim): both succeed, v0.20260710.5.
- `uv run python -m pytest tests/sim -q` → **597 passed** in 92.2s.
- `uv run python -m pytest tests/unit -q` → **239 passed** in 12.7s
  (includes the 133 proxy-specific tests re-run individually above).

**9. Bench-gate scope-out** (stated explicitly per this ticket's own AC):
this ticket does NOT cover the hardware bench gate. Per
`.claude/rules/hardware-bench-testing.md`, the full binary-plus-rump-
plus-proxy exercise on the physical robot on the stand — typed `STOP` from
a bare terminal halting a moving robot, a full binary regression over
serial + relay, and a combined TestGUI+gamepad-teleop session over radio
through the proxy (both currently broken pending the accepted-breakage
rewiring above) — is the team-lead's own next step, not this ticket's.
Ticket 004's own flagship bench test
(`calibration/linear.py --port ~/.rogo/robot-pty --direct` +
`gamepad_teleop.py` at 20 Hz) remains unexecuted for the same reason
(ticket 004's own completion notes already flagged this).

**10. Unrelated concurrent-session note** (observed, not acted on): during
this ticket's work, `clasi/issues/restore-line-and-color-sensors-as-ticked-
blackboard-devices.md` was modified and two new untracked issue files
appeared (`plan-write-up-the-rogo-testgui-issues-protocol-v3-issue-already-
exists.md`, `sprint-095-restore-line-color-sensors-as-ticked-blackboard-
devices.md`), timestamped during this session's work window. These are
unrelated to protocol-v3/sprint 097 and were NOT created by this ticket —
left untouched, not staged, not committed. Flagged for the team-lead's
awareness (possible concurrent session on the same working tree).
