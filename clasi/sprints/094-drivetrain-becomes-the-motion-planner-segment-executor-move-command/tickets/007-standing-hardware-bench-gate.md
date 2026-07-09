---
id: "094-007"
title: "Standing hardware bench gate [HITL]"
status: open
use-cases: ["SUC-004", "SUC-005"]
depends-on: ["094-006"]
issue: communicator-drivetrain-motion-command-segment.md
---

# 094-007: Standing hardware bench gate [HITL]

## ⚠️ HITL — stakeholder/bench-operator ticket, not a code-execution ticket

This ticket is **human-in-the-loop**. It requires physical access to the
robot mounted on its stand (`.claude/rules/hardware-bench-testing.md`). Do
**not** attempt to execute this ticket by driving hardware autonomously —
an execution agent without physical presence and a human safety observer
must not flash and drive the real robot unsupervised. The stakeholder or a
bench operator runs this ticket's steps directly; a programmer agent's role
here is limited to preparing the exact command sequence and recording the
stakeholder's/operator's reported results.

## Description

Deploy the sprint's firmware to the robot and exercise the full new/
preserved command surface on the stand, per this sprint's success criteria
and the standing hardware-bench-testing gate every sprint touching HAL/
motor/command-surface must pass. This is the sprint's final gate — it
confirms in sim-unreachable ways that (a) the flash-budget measurement from
094-005 holds on the actual flashed image, (b) the I2C flip-flop's timing
is genuinely unchanged (sim cannot fully prove real-bus timing), and (c)
the presolved graceful decel-to-zero has no reverse-creep on the real,
lagging plant — the regression check sim's idealized plant cannot fully
stand in for (per the project's own encoder-wedge/actuation-latency
knowledge base).

## Acceptance Criteria (run and reported by the stakeholder/bench operator)

- [ ] `just build-clean` (non-incremental — avoids the documented stale-
      incremental-build risk) succeeds.
- [ ] `arm-none-eabi-size build/MICROBIT` is run against the final flashed
      image and its `text`/`data`/`bss` figures are recorded, compared
      against 094-005's earlier measurement (should match closely — this
      re-confirms the flash gate on the actual artifact that gets flashed,
      not just an intermediate build).
- [ ] `mbdeploy deploy <full-UID> --hex MICROBIT.hex` flashes successfully
      (use `mbdeploy probe` first to confirm/dedupe the target device
      registry, per project knowledge on `mbdeploy`/registry gotchas).
- [ ] On the stand, over the real link (serial at the bench; radio relay if
      testing that path too):
  - [ ] Encoders are alive and increment in the expected direction,
        roughly proportional to commanded speed.
  - [ ] `S <l> <r>` spins both wheels directly (both directions tested).
  - [ ] `STOP` decelerates smoothly to rest — **no terminal reverse-creep**
        (the sprint's named regression gate vs. 093's presolved fix).
  - [ ] A straight `MOVE` (e.g. `MOVE 300 0 0`) completes and drains to a
        graceful stop.
  - [ ] A translate-then-terminal-pivot `MOVE` (e.g. `MOVE 300 0 9000`)
        completes, translates, then pivots at the end, and drains to a
        graceful stop.
  - [ ] A pure in-place `MOVE` (e.g. `MOVE 0 0 9000`) completes and drains
        to a graceful stop.
  - [ ] `MOVE ... j=<higher-than-boot-default> wj=<higher>` produces
        visibly smoother (less abrupt) accel/decel edges than the boot
        default, confirming the jerk knob has a real, observable effect.
  - [ ] `TLM` reports measured `enc=`/`vel=` throughout — values track real
        wheel motion, not a commanded target.
  - [ ] The RX watchdog is fed throughout the session (`send_fast PING`
        every ~200ms, per the project's own dev-serial-passive-pump-sampling
        knowledge) so the session itself doesn't stall on transport
        starvation unrelated to this sprint's own changes.
- [ ] Any failure at any step is reported explicitly (which step, what was
      observed) — this ticket is not satisfied by a single "it worked"
      summary. A failure here blocks sprint close.

## Implementation Plan

**Approach**: This is not a code-writing ticket. Its "implementation" is a
prepared bench script/checklist (a programmer agent may prepare this part)
plus the stakeholder's/operator's own hands-on session (which no agent
performs autonomously).

**Preparation (may be done by a programmer agent)**:
- Confirm/refresh a bench command-sequence script or a short manual
  checklist mirroring the AC list above, using `robot_radio`'s
  `NezhaProtocol` client (not raw `pyserial`, per the project's own
  bench-verification-gotchas knowledge — direct-USB CDC lags replies by
  about one command against raw pyserial) or the radio-relay path with the
  `!GO` data-plane handshake per `.clasi/knowledge/`.
- Confirm `docs/protocol-v2.md`'s currency question (094-006's Open
  Question 1) is resolved one way or the other before this ticket runs, so
  the bench operator has an accurate verb reference.

**Files to modify**: none required; a new `tests/bench/` script may be
added if useful (following the project's own `tests/bench/` conventions —
HITL CLI tools, not pytest-collected) but is not itself the acceptance
gate — the stand session is.

**Testing plan**: the AC list above, executed live, on the stand, by a
human.

**Documentation updates**: none required by this ticket itself, beyond
recording the stakeholder's/operator's session results in this ticket's
own completion notes (pass/fail per AC item) for the sprint-close review.
