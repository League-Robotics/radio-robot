---
status: pending
---

# Plan: Write up the rogo + TestGUI issues (protocol v3 issue already exists)

## Context

Eric wants three pieces of work captured in the CLASI issue pool, ordered: (1) streamline the wire protocol and delete the parser layers, (2) rework rogo to absorb the text-command mode leaving the robot — typed or `cat`-piped text commands over a **persistent** serial session, (3) restore TestGUI to full function in all three modes (sim / bench / playfield), **after** the protocol lands, **including** GOTO/tours.

Exploration established:
- **The protocol issue already exists and is complete**: `clasi/issues/protocol-v3-schema-driven-binary-command-plane-protobuf.md` (stakeholder decisions 2026-07-09: protobuf envelopes over ASCII-armored `*B<base64>` lines, one oneof arm per blackboard queue, generated validation, text rump = PING/ID/HELLO/HELP/STOP, rogo becomes the human REPL, 3 sprints dual-stack, ~4,900 → ~1,100 lines in `source/commands/`, host `protocol.py` keeps the NezhaProtocol API as a shim). **Nothing to write here** — only back-links to the two new issues.
- **rogo** (`host/robot_radio/io/cli.py`, 1783 lines, console script `rogo`): one-shot argparse CLI. Every subcommand opens/closes the port (`_make_robot()` … `disconnect()`); **no REPL/stdin mode exists**; drives verbs the post-093/094 firmware no longer registers (SET/GET, STREAM/SNAP, OTOS, G). The `>`-prefix/relay story is stale — SerialConnection already does the `!GO` handshake (036-007; cite the CORRECTION banner in `.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md`).
- **TestGUI** (`host/robot_radio/testgui/`, 13 modules): intact, not deleted — stranded by the 093/094 gut (last GUI commit 2026-07-06). All three transports already exist in `testgui/transport.py` (`SimTransport`=SIM, `SerialTransport`=BENCH, `RelayTransport`=PLAYFIELD). Dead call sites enumerated (G at `__main__.py:1508`, SNAP/STREAM in commands.py/transport.py/operations.py, SI/ZERO, OZ, DEV/DBG, calibration-SET push). `tests/testgui`: 364 green → 16 failures / 7 files.
- **Existing issue `realign-host-tooling-to-gutted-four-verb-wire-surface.md`** covers TestGUI realignment to the *current* surface — with "GUI after the protocol" (Eric's call today) its scope is subsumed by protocol-v3 Sprint 3 + the new TestGUI issue → supersede it.

Stakeholder decisions captured today (AskUserQuestion):
- **Three linked issues** (protocol one already on file).
- Wire format: already decided in the existing v3 issue.
- TestGUI restored **after** the protocol streamline (one pass, rides the v3 NezhaProtocol shim).
- TestGUI scope: **full 2026-07-06 feature set incl. GOTO/tours** — hard dependency on `restore-goto-pursuit-with-pose-estimator.md`.

## Deliverables (all doc-only; no code changes)

### 1. New issue: `clasi/issues/rogo-persistent-text-shell.md` (via the `/issue` skill)
"rogo: persistent-session text shell — the human text plane moves from firmware to host". Content:
- **Ordering**: follows [protocol-v3 issue] — rogo is where the text mode the robot sheds lands (v3 issue already seeds `rogo send`-as-REPL in its Sprint 3; this issue is the full tool).
- **Context**: current one-shot CLI, per-command port cycling, no stdin mode, dead-verb subcommands; stale `>`-prefix premise corrected (knowledge-doc banner).
- **Ask**:
  - Shell/pipe mode: interactive prompt AND `cat commands.txt | rogo …` (non-TTY stdin); each line = v2-style text command translated to a v3 envelope, replies pretty-printed.
  - **One port open per session** — persistent `SerialConnection`, id-correlated pipelined sends, keepalive/deadman handling for motion verbs; no per-command reconnect → fast command streams.
  - Realign/reduce the existing one-shot subcommands to the live surface (planning decides which survive vs. fold into the shell).
  - Absorbed firmware UX: HELP, verbose error explanation, `--decode` pretty-printer.
- **Acceptance sketch**: pipe a file of N motion/telemetry commands through one session over serial AND relay; interactive session round-trips; throughput measurably >> one-shot mode.

### 2. New issue: `clasi/issues/testgui-full-three-mode-restore.md` (via the `/issue` skill)
"TestGUI: full restoration in sim / bench / playfield modes on protocol v3". Content:
- **Ordering**: after protocol v3 (stakeholder 2026-07-09); rides the unchanged NezhaProtocol shim; config/pose/otos capability returns as v3 binary arms (v3 Sprint 2) — the GUI's features need those planes, not just the API.
- **Scope**: full 2026-07-06 feature set — driving, telemetry panel, traces (camera/encoder/OTOS/fused), canvas, sim-error panel, calibration push, set-origin/ZERO, OTOS ops, GOTO, TOUR_1/TOUR_2, live camera view (playfield), recorder — in all three transport modes.
- **Dependencies**: hard on [protocol-v3] and [restore-goto-pursuit-with-pose-estimator] (GOTO/tours); soft on [restore-line-and-color-sensors…] (line/color telemetry fields) and [relay-round-trip-bench-verification] (playfield transport proof).
- **Supersedes** `realign-host-tooling-to-gutted-four-verb-wire-surface.md` — absorb its scope items (capability gating, MOVE/MOVER + pull-TLM adoption, un-park/update `tests/testgui` [16 failures/7 files listed], `tests/CLAUDE.md` gate-doc reconciliation).
- **Acceptance sketch**: `just testgui` works in all three modes on real hardware for bench/playfield; tours run end-to-end; `tests/testgui` green (or a consciously parked, documented subset).

### 3. Reconcile the issue pool
- Add a short "Follow-on issues" note to `protocol-v3-schema-driven-binary-command-plane-protobuf.md` linking the two new issues with the ordering (protocol → rogo → TestGUI).
- Mark `realign-host-tooling-to-gutted-four-verb-wire-surface.md` superseded (pointer to the new TestGUI issue) and move it via `mcp__clasi__move_issue_to_done` (never a manual move; commit the move promptly — fs-only rename gotcha).

### 4. Commit
Single docs-only commit of exactly these files (`git add` the specific paths — the tree is dirty with unrelated WIP: devices.json, main.cpp, notebooks, .clasi.db — leave untouched). No test run needed for a docs-only issue-pool commit (matches prior `chore(issues):` commits, e.g. `abfa58e0`). No version bump per rule ("substantive changes" = code).

## Verification
- `mcp__clasi__list_issues` shows both new issues (status: pending) and no longer lists the realign issue as open.
- All `[...]` cross-links between the four issues resolve to real filenames.
- `git status` clean for clasi/issues/; unrelated WIP untouched.

## Explicitly NOT in scope
No implementation, no sprint creation (sprint-planner does that later, per mcp-guard role split), no edits to code, docs/protocol-v2.md, or the knowledge base.
