---
status: pending
---

# Finish ticket 097-011 — consolidate text commands into text_channel.{h,cpp}

## Context

Eric directed: collect all remaining text-command source into one file pair,
`source/commands/text_channel.{h,cpp}` (mirroring `binary_channel.{h,cpp}`),
shrinking `source/commands/` to a handful of files. This is ticket work only —
no implementation in this plan; a programmer agent executes the ticket later.

**Already done** (before plan mode activated): ticket **011** was created via
`mcp__clasi__create_ticket` in sprint 097 at
`clasi/sprints/097-protocol-v3-sprint-3-host-completion-and-text-retirement/tickets/011-consolidate-all-text-command-source-into-text-channel-h-cpp.md`.
It currently holds only the auto-generated template — the body still needs to
be written.

## One correction to the original directive (verified in code)

`telemetry_commands.{h,cpp}` **cannot be deleted outright**: besides the dead,
empty `telemetryCommands()` registrar, it holds the **live binary-plane
periodic-emission machinery** — `tickTelemetry()` + `telemetryEmitBinary()` —
called every loop pass from `source/main.cpp:157` and
`tests/_infra/sim/sim_api.cpp:357`. The ticket therefore relocates those two
functions **verbatim into `binary_channel.{h,cpp}`** (binary-plane code, not
text), keeping them at global namespace scope so both call sites need only an
`#include` swap. Then `telemetry_commands.{h,cpp}` deletes cleanly.

## Step: write ticket 011's body

Replace the template `## Description` / `## Acceptance Criteria` / `## Testing`
sections with the fully-drafted body already prepared at
`/Users/eric/.claude/plans/wobbly-growing-matsumoto-agent-a317aab2bf7cccb18.md`
(§ "Full ticket body to write"), keeping the tool-generated frontmatter.
Summary of what that body specifies:

- **End state**: `source/commands/` = `arg_parse`, `binary_channel`,
  `command_processor`, `text_channel` (4 pairs, down from 9). Registered text
  verb table byte-for-byte unchanged: STOP, PING, HELLO.
- **Moves into `text_channel.{h,cpp}`**: STOP (motion_commands), PING/HELLO +
  external-linkage `formatDeviceAnnouncement()`/`deviceIdentity()`
  (system_commands), and the unregistered dead families preserved verbatim
  with all doc comments — SI/ZERO (pose_commands), OI/OZ/OR/OP/OV/OL/OA
  (otos_commands), DEV M/DT/STATE/STOP/WD (dev_commands) — the pose/otos
  comments are sprint 098's transcription reference for the binary
  `pose`/`otos` envelope arms.
- **Moves into `binary_channel.{h,cpp}`**: `tickTelemetry()` +
  `telemetryEmitBinary()` (per the correction above).
- **Deletes**: the six old file pairs (motion, system, telemetry, pose, otos,
  dev `_commands.{h,cpp}`).
- **Fixups**: `command_router.cpp` `buildTable()` calls one live-rump builder
  (`textCommands(router)`); include swaps in `communicator.cpp`,
  `binary_channel.cpp`, `main.cpp`, `sim_api.cpp`;
  `tests/_infra/sim/CMakeLists.txt` explicit source list + its two file-naming
  comment blocks (ARM root CMakeLists globs — no edit); ~7 stale file-path
  citations in `docs/protocol-v3.md`.
- **Acceptance**: `just build-clean` both targets; `uv run python -m pytest`;
  grep-clean for the six deleted basenames; registered verb set provably
  unchanged; hardware bench smoke (PING/HELLO/STOP over serial, per
  `.claude/rules/hardware-bench-testing.md`).
- **Sequencing**: must land before in-progress ticket 010 (migration closure)
  finalizes its grep-clean/line-count/flash-RAM report — 010 re-runs its
  counts after this lands.

## Verification

- Ticket 011 file contains the full body; frontmatter untouched.
- `mcp__clasi__list_tickets(sprint_id="097")` shows 011 open alongside
  in-progress 010.
- Report back to Eric, flagging the telemetry_commands correction explicitly.

Not in scope: implementing the ticket, committing (tree already carries other
097 work-in-progress), or dispatching a programmer — those follow the normal
sprint execution flow when Eric says go.
