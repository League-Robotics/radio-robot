---
id: '125'
title: Bench-link transport reliability
status: roadmap
branch: sprint/125-bench-link-transport-reliability
worktree: false
use-cases: []
issues:
- bench-move-commands-intermittently-never-reach-firmware.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 125: Bench-link transport reliability

> Roadmap-level plan (Phase 1). Architecture, use cases, and tickets are
> filled in at detail-planning time.

## Goals

Diagnose and (where the evidence points to a fixable cause) fix the
pre-existing bench-link reliability gap: over the direct USB serial link, some
fraction of outbound `CommandEnvelope` writes are silently lost or corrupted
before reaching `RobotLoop::processMessage()` — the command gets NO ack at all
and the commanded motion never happens (encoders bit-for-bit identical
before/after).

## Problem

Observed on robot "tovez" over `/dev/cu.usbmodem2121102`
(`src/tests/bench/move_protocol_bench.py`: 30-38/43 across runs). Proven
**pre-existing** (A/B against commit `047555a5`, pre-120: same ~38/43, same
signature) and **distinct** from the 120-001 ack-ring issue (which is proven
solid: rapid-fire N-enqueue passed 15/15). This is an envelope-DROP upstream of
the ack ring / `Telemetry` entirely — matching `docs/protocol-v4.md` sec 7.4's
"a malformed/undecodable frame gets no reply at all."

## Solution (diagnosis-first — confirm at detail time)

This is a diagnostic sprint; a fix is a conditional outcome, not a guarantee.
Suggested investigation (from the issue):

1. `on_send`/`on_recv` verbose logging (`SerialConnection`'s callback hooks) to
   confirm whether the envelope bytes are actually written to the OS serial
   port on a failing call (rule out a host-side write bug) vs. never
   arriving/decoding on the firmware.
2. Check `Comms::malformedCount()`/`kFlagFaultCommsMalformed` on frames after a
   suspected drop — if NOT incrementing, the bytes never arrived (a
   link/transport problem, not a decode problem).
3. Different USB cable/port to rule out a marginal physical connection.
4. Correlate with recent motor activity/EMI (motor back-EMF coupling into the
   USB link is a classic cause of exactly this symptom shape).

Deliverable: a root-cause diagnosis with hardware evidence, plus EITHER a fix
(host retransmit-on-missing-ack, firmware decode hardening, or a physical/EMI
remedy the evidence supports) OR a documented characterization with a
recommended operational workaround.

## Success Criteria

- The drop mechanism is identified with hardware evidence (host-write vs.
  never-arrived vs. decode-fail, and whether motor-EMI-correlated).
- `move_protocol_bench.py` reaches a materially higher, stated pass rate on
  hardware after the fix, OR the gap is characterized as physical/environmental
  with a documented workaround (e.g. the 120-001 note that a tour runner can
  retry-on-missing-ack now that a dropped enqueue is OBSERVABLE).
- Hardware bench verify on the stand.
- `.claude/rules/hardware-bench-testing.md` corrected (replan addition): the
  stale banner shape, "ack slot" wording, and deploy commands match current
  firmware/tooling — it is the live bench checklist and must not lie
  (exactness review §7).

## Scope

### In Scope

- Diagnosis over the real serial link (`SerialConnection` hooks,
  `Comms::malformedCount()`), and a fix IF the evidence supports one
  (host and/or firmware, `src/host/robot_radio/io/` and/or `src/firm/com/`).

### Out of Scope

- The host-side TLM read-RATE issue (`tlm-rate-15-19hz-vs-50hz-nominal-serial.md`)
  — related, adjacent, but a distinct inbound-throughput concern with its own
  scope.
- The 120-001 ack ring (proven solid; not the cause here).

## Dependencies / Sequencing

- **Independent** of 121/122/123/124/126/127. Can run in parallel. Benefits
  from 120-001's ack ring already landed (a dropped enqueue is now observable
  as a missing ack in the expected window).

## Architecture

Deferred to detail planning. Expected tier: diagnosis-first; module footprint
(host transport and/or firmware comms) depends on what the trace finds.

## Use Cases

Deferred to detail planning.

## Tickets

Deferred to detail planning. Replan addition: one small ticket to fix
`.claude/rules/hardware-bench-testing.md` (banner, ack-ring wording, deploy
commands) alongside the diagnosis work — same bench session, zero code risk.
