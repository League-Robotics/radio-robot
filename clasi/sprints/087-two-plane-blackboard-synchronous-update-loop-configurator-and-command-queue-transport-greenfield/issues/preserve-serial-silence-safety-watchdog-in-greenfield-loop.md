---
status: in-progress
sprint: 087
tickets:
- 087-007
- 087-009
---

# Preserve the serial-silence safety watchdog in the greenfield loop rewrite

**Part of sprint 087** (two-plane blackboard / greenfield loop). Sprint 087 deletes
`source/dev_loop.*` and rebuilds `main.cpp`'s loop from scratch. Today's loop carries a
**serial-silence safety watchdog** that exists *because of the full-speed runaway
history* — it must NOT be silently dropped in the rewrite. This issue tracks
preserving it.

## Current behavior (grounded)

- `SerialSilenceWatchdog` (`source/commands/dev_commands.h`): fed by **any** inbound
  statement on **either** channel (serial or radio), regardless of content, via
  `feed(now)`. Default window ~1 s (`kDefaultWindow = 1000`); settable at runtime via
  `DEV WD <window>`.
- In `devLoopTick()` (`source/dev_loop.cpp`), `watchdog->check(now)` fires **once per
  silence episode**. On fire it **immediately** applies `buildBroadcastNeutral(BRAKE)`
  to the hardware and `buildDrivetrainStop(BRAKE)` to the drivetrain, and emits
  `EVT dev_watchdog`. Applied **immediately, not via the outbox** — it is an emergency
  stop, so it deliberately bypasses the normal staged/one-tick command path.

## Requirements in the new architecture

1. **The watchdog lives in the new loop's mandatory portion.** Its check runs every
   pass (same-pass, deterministic).
2. **On expiry it neutralizes motors IMMEDIATELY, same-pass** — using the
   architecture's already-sanctioned emergency-stop exception to synchronous update
   (the design already states the watchdog/emergency-stop acts same-pass, never
   deferred to the next clock edge). It must NOT route through the `driveIn`/`motorIn`
   one-tick queues.
3. **Fed on statement arrival** during slack ingest/routing (any statement, any
   channel, any content — same contract as today).
4. **Settable window preserved** (`DEV WD`), routed like any other command.
5. **`EVT dev_watchdog` still emitted** on fire.

## Acceptance

- **Sim:** with no statement for longer than the window, the motors are neutralized and
  `EVT dev_watchdog` is emitted; feeding a statement re-arms it.
- **Bench/HITL:** on the stand, comms silence neutralizes the wheels — exercised over
  the **radio** path specifically (ties to the sprint's radio-yield acceptance: a loop
  that starves radio would also starve the feed and mis-fire the watchdog).
- **No regression** of the runaway protection — this is the safety-critical acceptance
  bar for the loop-rewrite ticket.

## Scope

Belongs to the ticket that rewrites the loop, or a dedicated safety ticket sequenced
alongside it. Not a separate sprint.
