---
id: '112'
title: 'Firmware comms and device robustness: ack-ring delivery, relay handshake fault,
  absent-device re-probe'
status: roadmap
branch: sprint/112-firmware-comms-and-device-robustness-ack-ring-delivery-relay-handshake-fault-absent-device-re-probe
worktree: false
use-cases: []
issues:
- ack-ring-intermittent-delivery-gap.md
- relay-handshake-trips-comms-malformed.md
- absent-device-reprobe-after-boot.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 112: Firmware comms and device robustness: ack-ring delivery, relay handshake fault, absent-device re-probe

## Goals

Close the wire-reliability gaps found in the 104-007 bench soak, plus add
device re-detection.

(a) **Ack-ring delivery gap.** The practically-observed failure mode is
HOST-side polling, not a firmware/ring problem: a bounded
`wait_for_ack()`-then-give-up caller misses a late-but-not-lost ack
(104-007 finding 1 — a delay, not a loss, not ring-depth eviction). True
ring-depth eviction is real only under an unrealistic zero-paced burst no
realistic caller produces (finding 3). Primary fix is a continuous-draining
discipline — don't gate closed-loop control decisions on a bounded
per-command `wait_for_ack()`; prefer continuously-flowing telemetry fields
or a continuously-draining background ack matcher. Instrument the firmware
(pyOCD/gdb per `.claude/rules/debugging.md`) to confirm write-vs-transmit-
vs-wire-loss where root cause is still open.

(b) **Relay handshake fault.** The relay `!GO`/RAW250 transition trips
`kFaultCommsMalformed` (bit 3) once on every relay connect, before any
application command — root-cause the leaked control-plane bytes / transition
race and fix, rather than permanently living with a spurious latched bit on
every relay session.

(c) **Absent-device re-probe.** Add a slow (seconds-scale) re-probe slot to
the perception round-robin so a device absent (or transiently failing) at
boot can be detected later, respecting the bus schedule — re-probe runs only
in a perception slot, never a motor window. Present/connected semantics stay
as they are; absence remains a first-class state.

## Scope

### In Scope

- Ack-ring / `wait_for_ack()` polling-discipline fix (and/or firmware
  instrumentation to confirm root cause) for the discrete-command delivery
  gap.
- Root cause and fix for the relay-connect `kFaultCommsMalformed` one-shot
  trip.
- Slow background re-probe slot for absent devices in the perception
  round-robin.

### Out of Scope

- Heading/turn-accuracy co-tuning and wedge-latch-during-motion — sprint 111.
- Host P4 mid-layer rewrite (Nezha facade, nav, calibrate) — sprint 113. Note:
  this sprint's ack-ring "liveness = telemetry arriving" finding is expected
  to feed 113's `connect()` redesign; 113 is sequenced after this sprint for
  that reason.
- Repo hygiene / naming sweep / comment audit / vendor symlink — sprint 114.

## Acceptance Sketch (at-a-glance)

- No closed-loop control path (current or planned) gates on a bounded
  `wait_for_ack()`; either telemetry-based feedback or a continuously-
  draining matcher is used instead.
- `kFaultCommsMalformed` stays clear across a fresh relay connect (no
  application traffic sent) — not just "already-latched-at-baseline,
  never re-trips."
- A device absent at boot (e.g. OTOS or color/line unplugged) is detected
  and reported present once plugged in later, within the re-probe period,
  without a reboot.
- Direct-USB and relay soak gates (per 104-007's methodology) stay green.

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning document is complete (sprint.md, including its
      Architecture and Use Cases sections)
- [ ] Architecture review passed (or skipped, for changes with no
      architectural impact)
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|

Tickets execute serially in the order listed.
