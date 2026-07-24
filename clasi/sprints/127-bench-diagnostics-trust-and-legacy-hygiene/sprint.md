---
id: '127'
title: Bench diagnostics trust & legacy hygiene
status: roadmap
branch: sprint/127-bench-diagnostics-trust-and-legacy-hygiene
worktree: false
use-cases: []
issues:
- i2c-safety-net-bit-conflates-otos-settle-wait-with-loop-schedule-health.md
- stale-ruckig-cmake-comment-and-dead-dev-family-docs.md
- testgui-dbg-otos-bench-verb-dead-on-serial-connect.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 127: Bench diagnostics trust & legacy hygiene

> Re-planned per `clasi/issues/replan-sprints-122-plus-to-close-goal-exact-tours.md`:
> merges the old 126 (dead-legacy cleanup) and old 127 (I2C bit semantics)
> into one hygiene sprint, plus the 2026-07-23 exactness review's §6–7
> residue. Scheduled BEFORE the bench accuracy campaign so S3 debugging runs
> with truthful fault flags and truthful docs.

## Goals

1. **flags bit 6 means what it says** (existing issue, scope unchanged):
   stakeholder picks candidate b/c at sprint start; after the fix the bit is
   clear during normal idle AND driving on real hardware and provably
   re-asserts on a genuine schedule violation. Update the protocol-v4 §8.2
   bit-6 row (currently claims "boot-time one-shot" — falsified by 120-003).
2. **Dead legacy retired** (old-126 scope): no Ruckig claims or build
   machinery; no removed-`DEV`-family references in live docs/scripts; the
   dead `DBG OTOS BENCH` connect push gone (no `ERR ... legacy verb` line on
   any Serial connect).
3. **Exactness-review §6–7 residue:** the four comment/UI lies fixed
   (`io/sim_loop.py:995` 50ms->40ms; `telemetry.h` "single ack slot" header
   vs its own ring; `testgui/__main__.py:803/809` tooltips citing deleted
   Motion::Executor/Pilot; `telemetry_panel.py:15` HeadingSource);
   `src/sim/DESIGN.md` walkthrough rewritten to MOVE/ack-ring reality;
   naming residue renamed per coding-standards (`Otos::writePoseMm` ->
   `writePose` + unit tag; `nowMs`/`ageMs`/`shortfallMs` locals; the host
   `_cm` parameter cluster in path/, calibration/linear.py, playfield.py,
   cli.py); remaining narrative-comment blocks relocated per the 119-004
   recipe (protocol.py, __main__.py, transport.py, serial_conn.py,
   robot_loop.cpp kCycle history, sim_loop.py, sim_prefs.py, push.py,
   boot_config.h/persisted_tuning.h eulogies); a monolith growth-freeze rule
   recorded in `.claude/rules/` (new TestGUI features land as modules;
   `__main__.py` only wires).

## Success Criteria

- Bit 6: hardware-verified idle AND driving clear; injected violation
  asserts it; protocol-v4 row rewritten.
- Zero behavior change to any live motion/telemetry/wire path (suite green;
  no gate numbers move).
- Greps clean: Ruckig/DEV/DBG-push/single-ack-slot claims; units-in-
  identifier residue list empty; `docs/` "Last reviewed" stamps refreshed on
  touched DESIGN.mds.

## Scope

In: the three issue scopes + review §6–7 items above. Out: any control-law,
estimator, or wire change; anything the S3 campaign owns.

## Dependencies / Sequencing

- Bit-6 fix wants a short stand session (hardware verify) — can share 125's
  bench time. Everything else is host/doc-only. Before 128 (S3 needs
  truthful flags and docs).

## Architecture / Use Cases / Tickets

Deferred to detail planning. Expected tickets: (1) bit-6 semantics + doc row
+ hardware verify; (2) legacy retirement (Ruckig/DEV/DBG); (3) review-residue
sweep (comment lies, naming, sim DESIGN rewrite, relocation, growth-freeze
rule).
