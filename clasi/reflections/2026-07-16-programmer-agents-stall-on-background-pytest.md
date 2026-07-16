# Reflection: programmer sub-agents stall on background pytest

Date: 2026-07-16 (sprint 108 + the OOP sim-command-surface follow-up)

## The failure mode

~6 times this session, a dispatched **programmer sub-agent** finished its code
edits, launched the full `uv run python -m pytest` suite as a **background /
detached task**, and then **ended its turn** with the work **uncommitted and
the ticket not marked done** — reporting "standing by for the background pytest
to complete" or similar. It never resumed to see the result and finish.

Affected dispatches: ticket 010 (x2), ticket 011 (x2), the sim command-surface
agent, and its finisher. Every time, the team-lead had to take over: run the
suite in the team-lead's own context and commit the sub-agent's work.

## Root cause

A sub-agent that spawns a long background task and then has **no remaining
foreground tool call** has its turn ended by the harness. The main loop gets
re-invoked when a background bash task completes; a **sub-agent apparently does
not reliably get resumed** the same way — the completion fires but the agent has
already "reported and stopped," so its terminal work (commit, mark ticket done,
final report) never happens.

## Contributing factors

1. **The suite is slow (~4-5 min).** Each C++ harness test under `tests/sim/`
   `subprocess`-**recompiles its binary from scratch** on every run (no shared
   build/cache). That slowness makes backgrounding the run tempting.
2. **`run_in_background` is an available affordance** and the agents reach for it
   for anything long — not realizing it's a trap for a sub-agent (ends the turn).
3. **Every programmer runs the FULL suite**, redundantly (N tickets x ~5 min),
   instead of a scoped subset.
4. Adding "run FOREGROUND, do not background" to dispatch prompts helped but did
   not fully stop it — habit/affordance won.

## What to fix (ranked)

1. **Programmer agent definition** (`.claude/agents/programmer/*`): add a hard
   rule — "NEVER background the test run. Run it synchronously so you stay alive
   to see the result and finish. A ticket is not done until committed." And:
   "Run only the tests relevant to your ticket; the team-lead runs the full gate."
2. **Split the test gate:** programmers run a scoped subset; the **team-lead /
   execute-sprint runs the full suite once** before close. Removes both the
   redundancy and the per-agent stall temptation.
3. **Speed up `tests/sim/`:** cache/reuse the compiled C++ harness binaries
   (one shared build step) instead of `subprocess`-recompiling per test. The
   4-5 min is almost entirely recompilation; a shared build would make it
   seconds and remove the incentive to background.
4. **Harness-level (deepest):** either block `run_in_background` inside a
   sub-agent, OR guarantee a sub-agent is re-invoked when its background task
   completes so it can finish. Today a backgrounded task in a sub-agent silently
   orphans the sub-agent's remaining work.

## Interim mitigation the team-lead used

Run the full suite in the team-lead's own context (where background bash DOES
re-invoke on completion), then dispatch a **commit-only** finisher, or commit
directly. Works, but defeats the point of delegating.
