---
status: in-progress
sprint: 088
tickets:
- 088-003
---

# `HELP` returns a hardcoded verb list instead of enumerating the registered command table

## Context

The stakeholder observed that motion verbs (`D`, `T`, `R`, `RT`, `TURN`, `G`, …)
"don't show up with `HELP`," and suspected they were not wired up. Investigation
shows they **are** wired and functional in the real build — the problem is `HELP`
itself.

In `ROBOT_DEV_BUILD`, the `CommandRouter` assembles the **full** command table —
`systemCommands()` + `devCommands()` + `telemetryCommands()` + `motionCommands()` +
`configCommands()` + `poseCommands()` + `otosCommands()`
([command_router.cpp:24-41](source/runtime/command_router.cpp#L24-L41)) — so
`S/T/D/R/TURN/RT/G/STOP` ([motion_commands.cpp:734-742](source/commands/motion_commands.cpp#L734-L742)),
`SET/GET`, `STREAM/SNAP`, the `O*` OTOS verbs, `SI/ZERO`, and the `DEV` family are
all registered and routable. (The stakeholder's separate report that `D/T/S`
actually drive the wheels confirms these execute.)

But `HELP` is a `systemCommands()` handler that returns a **hardcoded string**:

```cpp
// system_commands.cpp:62-67
void handleHelp(...) {
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "help", "PING VER HELP ECHO ID", ...);
}
```

So `HELP` always reports only the five liveness verbs, regardless of what is
actually registered. It lies about the command surface.

Note: the `ROBOT_DEV_BUILD == 0` fallback genuinely installs only
`systemCommands()` (a bare `CommandProcessor` — [main.cpp:217-236](source/main.cpp#L217-L236)),
but that is the deliberate "liveness-only fallback (no HAL, no DEV, no watchdog)"
described in the file header — not the bug here.

## Desired behavior

`HELP` enumerates the **actual registered command table** of the active
`CommandProcessor` (every verb prefix in the descriptor list `buildTable()`
produced), rather than a static literal. This matches the documented contract at
[docs/protocol-v2.md:237](docs/protocol-v2.md#L237), which shows `HELP` returning
the full verb list (`PING ECHO ID VER HELP SET GET STREAM SNAP S T D G STOP GRIP
ZERO OI OZ OR OP OV OL OA P PA`).

Notes for whoever picks this up:
- `HELP` currently has no handle to the full table (it lives in
  `systemCommands()`, built before the router concatenates the other families).
  Give the `HELP` handler access to the assembled `CommandProcessor` descriptor
  list — e.g. iterate the processor's registered descriptors and join their
  prefixes — so it stays correct as families are added/removed and across build
  variants.
- Keep the output shape `OK help <space-separated verbs> [#id]`.

## Acceptance

- In the dev build, `HELP` lists every registered verb (system + dev + telemetry +
  motion + config + pose + otos), not just the five liveness verbs.
- The list is derived from the live registered table, so adding/removing a command
  family changes `HELP` output with no edit to the handler.
- **HITL bench (on the stand):** `HELP` over the real link returns the full,
  accurate verb set.
