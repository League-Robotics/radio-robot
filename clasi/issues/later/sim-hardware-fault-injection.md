---
status: pending
---

# Sim hardware fault injection — disconnect, wedge, encoder dropout

## Problem

The host-side simulation harness (see
`clasi/issues/host-side-simulation-environment-for-the-new-tree-design-write-up.md`)
ships v1 with healthy hardware only: every `SimMotor` reports `connected() == true`
and `wedged() == false`, and the OTOS always returns a fresh pose. The firmware's
*fault-handling* paths — how it reacts to a motor dropping off the bus, an encoder
that sticks at a stale value (the wedge/latch family), a sensor that stops updating —
therefore have no deterministic, off-hardware way to be exercised. Today those paths
can only be provoked on the bench, unreliably (you cannot make real hardware wedge on
command), which is exactly why the wedge saga took as long as it did to root-cause.

## Why this is worth capturing

A simulator's highest-leverage capability is injecting faults that real hardware
won't produce on demand. The encoder wedge specifically
(`docs/knowledge/2026-07-04-encoder-wedge.md`, and the `later/`
`encoder-wedge-corrupts-tour-legs.md` issue) is a stale-readback failure the firmware
must detect and recover from; a deterministic in-sim wedge would turn "reproduce it on
the bench and hope it triggers" into a fast, repeatable regression test.

## Sketch (not a v1 commitment)

Follow-on ctypes-backdoor knobs on the sim devices (no wire surface — same rule as the
rest of the sim's error knobs):

- **Motor disconnect** — force `SimMotor::connected()` to false for a named port; verify
  the firmware's connected-gating and any DEV/telemetry reporting.
- **Encoder wedge / stuck value** — freeze a `SimMotor`'s reported encoder at its
  current value (or an injected one) while the plant keeps moving, reproducing the
  boundary-latch flavor; verify the wedge detector fires and recovery unfreezes it.
- **Encoder dropout** — drop a fraction of encoder samples (read returns "no new data")
  to exercise the read-failure / outlier-filter recovery paths (cf. sprint 064).
- **OTOS staleness / warn bits** — hold the OTOS pose stale or assert a warn flag to
  exercise the fusion gate and health reporting (cf. sprints 065/074), once firmware
  fusion consumes the OTOS at all.

## Preconditions / when to pick this up

- The v1 sim harness (motors + OTOS + plant + C ABI + Python fixtures) must exist first.
- Most valuable once there are firmware consumers whose fault-reactions are worth
  regression-testing — the wedge detector already exists; OTOS-health reactions arrive
  with fusion. Revisit when either becomes a priority.

Deferred by stakeholder decision 2026-07-04 (v1 sim scope excludes fault injection);
filed so the capability is not lost.
