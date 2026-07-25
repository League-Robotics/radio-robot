---
id: '006'
title: Bench hardware verification gate
status: open
use-cases: [SUC-002, SUC-003, SUC-004]
depends-on: ['003', '004']
github-issue: ''
issue: cobs-crc-binary-framing-replace-base64-armor.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Bench hardware verification gate

## Description

Deploy to the robot on its stand and exercise the new
wire framing on the real link, per
`.claude/rules/hardware-bench-testing.md`'s standing bench gate (this
sprint touches the transport layer directly). Includes the sensors-
alive/wheels-and-encoders/round-trip checklist AND the COBS+CRC-specific
fault-injection check on real hardware (not just sim/unit fault
injection).

## Acceptance Criteria

- [ ] Sensors alive: encoders, OTOS, line sensor, color sensor, digital/
      analog ports all respond with plausible, changing values over the
      new framing.
- [ ] Wheels drive and encoders increment correctly (both directions) —
      confirms the new framing carries MOVE commands correctly.
- [ ] Round-trip confirmed over BOTH transports: USB serial at the bench
      AND the radio relay (SUC-004).
- [ ] Envelope sizes above the old 252-byte armored ceiling transmit
      without truncation on both transports.
- [ ] A real, fault-injected corrupted frame (e.g. a deliberately
      bit-flipped test frame, or induced via a lossy-link stress test)
      is dropped and counted on real hardware, not just in the unit/sim
      fault-injection tests from tickets 001-003 (SUC-002).
- [ ] HELLO/PING text rump confirmed working interleaved with binary
      frames on real hardware, both transports (SUC-003).

## Implementation Plan

- **Approach:** Follow the Quick Smoke Sequence
  (`.claude/rules/hardware-bench-testing.md`) using `mbdeploy deploy
  --build`, then bench scripts under `src/tests/bench/` (extend or add a
  COBS+CRC-specific fault-injection bench script alongside the existing
  `move_protocol_bench.py`/`tlm_log.py` catalog).
- **Files:** possibly a new `src/tests/bench/cobs_crc_bench.py`; no
  production firmware/host changes expected in this ticket (verification
  only) unless the bench run surfaces a real defect, in which case it is
  fixed here before the gate can pass.
- **Testing:** the bench run itself IS the test; document results (pass/
  fail per checklist item) in the ticket on completion.
- **Documentation:** none beyond recording the bench results.
