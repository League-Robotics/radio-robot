---
status: pending
priority: high
---

# Inbound (host→robot) command loss over the relay needs an ARQ, not a slower telemetry stream

## Description

Measured on `tovez` via the `getez` relay (channel 3, 2026-08-07) while
raising `App::RobotLoop::kCycle` 50→32ms. The nRF link is **half duplex**,
so the robot's outbound telemetry airtime directly eats the window in which
it can HEAR the host:

| `kPrimaryPeriod` | telemetry | `radio_bench_gate` | `move_wheels` | 0x0A repro |
|---|---|---|---|---|
| 25 (shipped) | 31.4 fps | 30/35 | **command LOST** | 8/10 |
| 40 | 15.8 fps | 31/35 | ok | 9/10 |

**The loss is inbound, and it is the COMMAND, not the ack.** Evidence:

- Outbound wire quality is 99.2% ok at BOTH rates — **zero** unparseable,
  **zero** CRC mismatches, over 251 frames. Telemetry is not being lost.
- The lost `move_wheels` left the encoders at `(338, 358)` before AND after,
  `d_left=0.0mm`. The robot never moved, so it never received the command. A
  lost ack would have left it driving.
- Acks are already redundant against frame loss: `App::kAckRepeats = 3` in a
  depth-12 ring (`app/telemetry.cpp:152`), so every ack rides three
  consecutive frames. A single dropped telemetry frame cannot lose one.

## Why halving the telemetry rate is the wrong fix

It was tried (the 40 row above) and it does buy the reliability back, but:

1. **It aims at the wrong direction.** Nothing is wrong with the outbound
   path; slowing it is a side-effect remedy for an inbound defect.
2. **It costs every telemetry consumer** — host-side closed-loop pursuit,
   bench captures, the TestGUI traces — for a problem none of them cause.
3. **It breaks perception reporting outright.** `robot_loop.cpp`'s pace
   block alternates line/colour by cycle parity. An emit floor longer than
   one cycle ALIASES with that parity: measured `line_present 93,
   color_present 0` over 93 idle frames at `kPrimaryPeriod=40 / kCycle=32`.
   The colour sensor disappeared from the wire entirely.

## Proposed fix — a real ARQ on the host→robot direction

Stakeholder sketch, 2026-08-07, and it is the right shape:

- Sequence every inbound command.
- Robot reports, in telemetry it already sends, a **cumulative ACK** (the
  highest sequence received in order) plus a **NACK** (the first gap).
- Host keeps a bounded window of unacked commands and **retransmits from
  the NACKed sequence onward**.
- Out-of-order arrivals are dropped and re-NACKed until the gap fills, so
  ordering is restored without per-packet state.

The machinery this needs already exists: protocol v5 frames are COBS-framed
with a CRC, so corruption is already detected rather than silently accepted;
what is missing is only the sequencing and the retransmit window. The same
substrate would also permit a streaming binary `DATA` command later
(explicitly noted as possible, NOT proposed here).

## Notes / prior art in-tree

- This compounds an ALREADY-TRACKED inbound problem, it does not create it:
  `clasi/issues/later/radio-bench-gate-fault-latch-check-contradicts-inbound-loss-budget.md`
  records sprint 128's own 31/35 FAIL on this same gate, from a latching
  `kFaultCommsMalformed` plus a lost 0x0A iteration — with no telemetry-rate
  change involved. That issue asks for exactly the measurement this one
  supplies ("characterize inbound loss directly (host→robot direction)").
- The often-quoted "~20% inbound loss budget" remains unsourced folklore —
  do not cite it as a measurement (see the project memory of the same name).
- USB is unaffected: 31 fps is ~2400 B/s, ~21% of the link, and every
  `move_protocol_bench.py` scenario passes over it at `kCycle=32`.

## Acceptance

- Inbound loss over the relay is characterized with a number (sent vs
  received, host→robot), not inferred from a gate check failing.
- With the ARQ in place, `radio_bench_gate.py` reaches its command-path
  checks at the SHIPPED telemetry rate — no command silently dropped.
- The telemetry rate is chosen on its own merits (consumers, airtime
  budget), not as a workaround for inbound reliability.
