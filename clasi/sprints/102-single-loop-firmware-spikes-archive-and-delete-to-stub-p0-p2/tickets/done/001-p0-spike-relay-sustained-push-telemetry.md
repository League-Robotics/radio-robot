---
id: '001'
title: 'P0 spike: relay sustained-push telemetry'
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: single-loop-firmware-de-fiber-delete-the-elite-plumbing-telemetry-only-return-path.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# P0 spike: relay sustained-push telemetry

## Description

Measure, on CURRENT firmware (no code changes), whether the radio relay's
`!GO` data plane sustains a pushed binary telemetry stream at the target
~30 Hz rate, or silently drops frames the way the standing knowledge note
(`.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md`) claims async
`STREAM` frames are dropped by the bridge. This is the first of three P0
spikes that must be answered before the P2 delete (ticket 005), because P4
(sprint 103/104) needs to know whether the new ack-ring telemetry design can
rely on a pushed stream through the relay, or must fall back to host-paced
polling of the same frame.

Do not modify firmware or host code for this ticket — it is measurement and
documentation only. Use the existing binary `STREAM` command against the
current (pre-delete) firmware build.

**Scope expansion (stakeholder decision 2026-07-14, replacing dropped ticket
002):** the serial baud-ceiling spike (former ticket 002 / SUC-002) is
dropped — the radio relay is the robot's production interface and its
throughput is fixed, so raising the USB baud would only let the bench
diverge from the field. This ticket's measurements now do double duty: the
same direct-USB and relay captures already required below (at the current,
fixed 115200 serial baud — no baud switching) also determine the RATE-SETTING
number for telemetry cadence on BOTH transports. Concretely, this ticket's
verdict must report and record three numbers: (1) the sustainable frame rate
through the relay, (2) the sustainable frame rate over direct USB at 115200,
and (3) a recommended common cadence for both transports — the minimum of
(1) and (2), with explicit headroom — that P4 (wire protocol) and P5 (host)
must design to. If the radio sustains less than the ~30 Hz target, the
recommended cadence (or frame size) comes down to match; it never goes up
past what serial's fixed 115200 baud can carry.

## Acceptance Criteria

- [x] Binary `STREAM` armed at ~30 Hz against current firmware. (33 ms
      period, ~30.3 Hz target, robot v0.20260714.2.)
- [x] Direct-USB frame delivery captured for at least several minutes; frame
      count, gap pattern, and any corrupted/malformed frames recorded with
      concrete numbers, and a sustainable frames/sec figure computed at the
      fixed 115200 baud (no baud switching in this ticket). (240 s,
      6430/6430 delivered, 0.00% drop, 0 malformed, 26.79 fps.)
- [x] Same capture repeated through the radio relay's `!GO` data plane
      (opened with DTR asserted, `!GO` sent to enter the data plane, per
      `.clasi/knowledge/` protocol notes) for a comparable duration, with its
      own sustainable frames/sec figure computed. (240 s, 6428/6430
      delivered, 0.031% drop, 26.78 fps, `entered_data_plane: true`
      confirmed in connect info.)
- [x] Delivered-frame rate and drop pattern compared numerically between
      direct-USB and relay paths (e.g. frames/sec, % dropped, longest gap).
      (26.79 vs 26.78 fps; 0.00% vs 0.031% drop; longest gap 0 vs 1;
      relay loss classified uniform/sparse — two isolated single-frame
      gaps, not a burst. See spike-001-relay-telemetry.md.)
- [x] `.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md` updated
      with an explicit confirm-or-retract verdict on "async STREAM frames
      dropped by the bridge" against the CURRENT relay firmware, dated and
      backed by the measurement numbers above. (2026-07-14 correction
      block added: claim RETRACTED.)
- [x] A push-vs-host-paced-poll recommendation is written down for the
      P4/P5 designers (sprint 103/104), stating which return-path strategy
      the ack-ring telemetry design should assume. (PUSH/STREAM recommended;
      see spike-001-relay-telemetry.md "Push vs. host-paced-poll
      recommendation".)
- [x] The ticket's verdict explicitly records all three rate-setting numbers
      — relay-sustained rate, direct-USB-at-115200-sustained rate, and the
      recommended common cadence (their minimum, with headroom) — as the
      telemetry rate budget both transports must honor. This replaces the
      dropped serial baud-ceiling spike (former ticket 002); no baud change
      is proposed or implied anywhere in the stack. (Relay 26.78 fps,
      direct-USB 26.79 fps, recommended common cadence 25 Hz / 40 ms
      period, no baud change.)
- [x] No production firmware or host source file is modified by this
      ticket — only the knowledge-note doc and this ticket's own
      documentation. (Confirmed: only the knowledge note, this ticket, the
      new spike-001-relay-telemetry.md results note, and the new
      tests/bench/relay_telemetry_rate.py diagnostic script changed.)

## Completion Notes (2026-07-14)

Measured on the bench rig against current firmware (robot v0.20260714.2,
tovez/2314287040; relay "zavaz"/4076631795) — no firmware or production host
code changed. Two sequential 240 s sustained-push captures at a 33 ms
(~30.3 Hz) armed `STREAM` period:

- **Direct USB (115200 baud)**: 6430/6430 delivered, 0.00% drop, 0
  malformed, sustained 26.79 fps.
- **Relay (`!GO` data plane)**: 6428/6430 delivered, 0.031% drop (2 isolated
  single-frame gaps, uniform/sparse — traced to `source/com/radio.cpp:62-71`'s
  single-slot RX mailbox, not a bridge-wide async-drop behavior), sustained
  26.78 fps.
- **Verdict**: the 2026-06-12 knowledge note's "async STREAM frames are
  dropped by the bridge" claim is RETRACTED for current relay firmware.
  PUSH telemetry is recommended for the P4/P5 ack-ring design over
  host-paced polling.
- **Three rate-setting numbers**: relay-sustained 26.78 fps,
  direct-USB-sustained 26.79 fps, recommended common cadence **25 Hz
  (40 ms period)** for both transports, no baud change.

Full writeup:
`clasi/sprints/102-single-loop-firmware-spikes-archive-and-delete-to-stub-p0-p2/spike-001-relay-telemetry.md`.
Knowledge note updated:
`.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md` (2026-07-14
correction block). Diagnostic script committed:
`tests/bench/relay_telemetry_rate.py`.

## Implementation Plan

**Approach**: This is a measurement/spike ticket, not a code-change ticket.
Use the existing bench tooling (`tests/bench/` scripts, or a throwaway
scratch script if none currently exercises sustained `STREAM` capture) to
arm telemetry and log received frames with timestamps, once over direct USB
serial (at the fixed 115200 baud — no baud switching) and once through the
relay dongle. Compute delivery-rate and gap/drop statistics from the two
logs, compare them, and derive the recommended common cadence (the minimum
of the two sustainable rates, with headroom) that becomes the rate budget
for P4/P5 — this is the deliverable that replaces the dropped baud-ceiling
spike (former ticket 002).

**Files to create/modify**:
- `.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md` — update with
  the verdict, including the three rate-setting numbers (relay-sustained,
  direct-USB-sustained, recommended common cadence) (this is the one
  persistent artifact this ticket produces).
- A throwaway measurement script under `tests/bench/` or a scratch location
  is acceptable and does not need to be committed if it adds no lasting
  value; if it is generally useful for future relay-telemetry debugging,
  keep it under `tests/bench/`.
- No production `source/` or `host/` files change.

**Testing plan**: The "test" is the measurement run itself — direct-USB and
relay captures, each covering multiple minutes at the target rate, with
frame counts and gaps logged. No new pytest is required for a
measurement-only ticket; if a reusable capture script is kept, no automated
assertions are needed (it's a bench diagnostic tool, not a regression test).

**Documentation updates**: `.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md`.

## Verification (hardware bench gate)

Per `.claude/rules/hardware-bench-testing.md`: this ticket runs entirely
against the bench-mounted robot (wheels off the ground, safe to power). No
motor commands are involved — this exercises the serial/radio telemetry
path only. Confirm the relay dongle is verified (not the robot) before
using it, per `mbdeploy list`'s ROLE column.

## Testing

- **Existing tests to run**: `uv run python -m pytest` (surviving suite,
  unaffected by this ticket — sanity check only).
- **New tests to write**: none required; measurement script under
  `tests/bench/` if kept is a diagnostic tool, not an asserting test.
- **Verification command**: `uv run pytest`
