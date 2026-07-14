---
status: obsolete
---

> **OBSOLETE (2026-07-14 stakeholder triage).** Superseded by the single-loop
> firmware rebuild (`clasi/issues/single-loop-firmware-de-fiber-delete-the-elite-plumbing-telemetry-only-return-path.md`;
> review: `docs/code_review/2026-07-13-devices-drive-review.md`). Superseded by the rebuild's P0 relay push-stream spike and P6 bench gate, which verify the relay round-trip against the NEW surface; the surface this issue targets is deleted.

# Bench-verify the current command surface over the radio-relay round-trip (not just direct serial)

> Refreshed 2026-07-09 (stakeholder triage). Originally filed after sprint
> 088's serial-only bench pass; the command surface has since been gutted
> (093) and rebuilt around the segment-executing Drivetrain (094 + teleop).
> The relay round-trip has still never been proven against ANY of the new
> surfaces — the ask below targets the current one.

## Context

Sprint 088's on-stand verification (`tests/bench/motion_command_verify.py`)
proved the then-current surface over **direct USB serial only**; the relay
round-trip (host → relay `!GO` data plane → radio → robot) was deferred and
has stayed deferred through the 093/094 rebuilds. The current live surface
is `PING`/`VER`/`HELP`/`ECHO`/`ID`/`HELLO` + `S`/`STOP`/`D`/`T`/`RT`/
`MOVE`/`MOVER`/`TLM`/`QLEN` (see `source/runtime/command_router.cpp`
`buildTable()`); the 088 script also drives verbs that are no longer
registered (`SET`/`GET`, `STREAM`), so it cannot be re-run as-is.

## Ask

On the stand, over the radio relay (`SerialConnection` auto-detects
`mode=relay` and does the `!GO` handshake; relay dongle
`/dev/cu.usbmodem2121402`):

- **Liveness + announcement:** `PING` round-trips; `HELLO` re-announce and
  the boot `DEVICE:` banner reach the host over the relay path (relay
  rediscovery is the design case for the announcement).
- **Segment motion:** `MOVE` and at least `D`/`RT` execute with encoders
  responding (`TLM enc=`/`vel=` polled over the relay — `TLM` is pull-based,
  which suits the relay's dropped-async limitation; there is no `STREAM`).
  Confirm graceful queue-drain stops.
- **MOVER deadman under relay throughput:** the interesting new case. The
  relay caps throughput ~12 msg/s and teleop refreshes `MOVER` every
  ~150 ms with a deadman of 3× the period — verify a sustained `MOVER`
  stream over the relay does NOT starve the deadman into spurious stops,
  and that ceasing the stream DOES stop the wheels within the deadman
  window (observed on the wheels, not just telemetry).
- **Direct path sanity:** `S`/`STOP` round-trip and drive the wheels.

Expect radio-specific flakiness — use a pipelined, id-correlated reader
(`robot_radio` NezhaProtocol), never hand-rolled lock-step send/read (see
`.claude/` memory: apparent "loss" over this link is always a host harness
bug; the link and CDC chip are reliable).

## Acceptance

- The verbs above verified functional over the relay with encoder response;
  `HELLO`/`DEVICE:` banner received over the relay; the MOVER deadman
  behaves correctly (no spurious stops while streaming, prompt stop when
  the stream ceases). Result appended to a bench-verification log
  (sprint 088's `bench-verification-log.md` or a fresh relay log).
- Any relay-specific discrepancy filed as its own follow-up issue rather
  than silently patched.
