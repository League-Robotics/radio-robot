---
status: pending
---

# Bench finding — `DBG OTOS BENCH` / `DBG OTOS` emit no reply on hardware

## Context

Found during sprint 032 hardware bench validation (firmware v0.20260612.17, robot `tovez`).
The Bench OTOS DBG commands added in sprint 031-003 do NOT reply on the real robot:

- `DBG` alone → `ERR unknown` (the DBG subsystem parses fine).
- `DBG OTOS BENCH 1 20 10 0` → **no reply at all** (not even ERR), even with a 1.2s read window.
- `DBG OTOS` → **no reply at all**.

Because they're not `ERR unknown`, the `DBG OTOS [BENCH ...]` prefixes ARE registered (031-003 shipped),
but the handlers emit nothing on hardware. The sim tests (`host_tests/test_dbg_otos_commands.py`) pass —
this is a hardware-only reply-path failure, the exact class the bench harness exists to catch.

Consequence: the Bench OTOS could not be enabled or verified on the robot, so the 032 bench test ran on
encoder odometry instead of the synthetic OTOS. Fixing this is the prerequisite to re-running the bench
validation under the Bench OTOS.

## Likely cause (to investigate)

- The `#ifndef HOST_BUILD` reply path in `handleDbgOtosBench` / `handleDbgOtos`
  (`source/app/DebugCommandable.cpp`, 031-003) may not call the reply function in the firmware build,
  or replies into a channel/ctx that isn't forwarded.
- Check the longest-prefix-first registration ordering of `DBG OTOS BENCH` vs `DBG OTOS` on the real
  command table, and that the handler's `replyFn`/ctx are wired the same way working DBG handlers
  (e.g. `DBG WEDGE`, which presumably does reply) are.

## Acceptance

- On hardware, `DBG OTOS BENCH 1 ...` replies `OK dbg otos bench=1`, `DBG OTOS BENCH 0` replies
  `OK dbg otos bench=0`, and `DBG OTOS` replies the `ideal=.. otos=.. fused=.. err=..` line.
- Add a hardware smoke step (or note in the smoke ritual) that exercises these so it can't regress silently.
