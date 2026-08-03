---
status: done
priority: high
sprint: '131'
tickets:
- 131-002
---

# "Stop is stop" has a Stage-B-sized hole, and Stage B has no freshness gate

## Description

2026-08-02 post-130 review, **F8 (MAJOR, latent)**. Both halves are latent
**only** because Stage B gains ship at zero on every robot — i.e. exactly until
the next bench session tunes them, which is the recommended next step in
[[A-next-physical-bench-session-checklist]]. Fix before that session, not after.

## Half 1 — commanded zero is only enforced in Stage A

The zero-command guard lives in `correctedCommand()` (`src/firm/app/drive.cpp:114`).
`tick()` then adds the PID term **unconditionally** (`drive.cpp:299-313`).

With gains on, a retained integrator after a normal WHEELS expiry keeps duty ≠ 0
at rest. Everything downstream then conspires:

- `commandedStop`/`alreadyQuiet` require duty `== 0.0f` **exactly**, so a tiny
  nonzero duty defeats the quiet path;
- `writeShapedDuty()` **boosts** any nonzero sub-deadband duty up to the 3%
  floor (`src/firm/devices/nezha_motor.cpp:274-280`).

Net: a parked robot can creep or buzz on encoder-quantization noise. This is the
same silent-stall class the deadband boost was built to kill, running in reverse.

**Fix:** give commanded-zero the same explicit treatment the intercept got —
skip the PID entirely at command == 0, or actively decay the integrator there.
Do not rely on the integrator happening to be small.

## Half 2 — Stage B consumes velocity with none of Stage C's gates

`drive.cpp:299-306` reads `state.wheelLeft.velocity` with no freshness, no
`connected`, and no frozen check. `drive.cpp:323-328` (Stage C) has all three.

Worse, a failed encoder collect **manufactures velocity = 0**
(`nezha_motor.cpp:468-487`). So Stage B will wind up against a wheel it cannot
see, and any gain sweep run on a glitching bus is silently contaminated — on a
bench that has wedged four times in two days.

Related freshness dishonesty (post-130 review, Part 1 MINOR):
`NezhaMotor::tick()` stamps `lastTickUs_` even when the collect failed, so
`EncoderReading.age` reads *fresh* on a disconnected bus, and `motor.h:103-109`
promises a `lastFreshUs_` that does not exist. Stage C survives only via its
separate `connected` conjunct. Fix this alongside, or Stage B's new gates
inherit the same lie.

**Fix:** Stage B adopts Stage C's exact measurement gates; `age` tells the
truth about the last *successful* read.

## Why this is A and not B

Both halves are dormant at shipped gains, which is precisely the trap: the next
thing anyone does to this controller is turn Stage B on. A tuning sweep that
runs against manufactured zeros produces gains that are wrong in a way no plot
will show.

## Verification

- Firmware test: with nonzero Stage B gains and a wound integrator, a
  commanded-zero wheel writes exactly 0 duty and stays there.
- Firmware test: with the encoder collect failing, Stage B does not integrate;
  `age` reflects the last successful read, not the last tick attempt.
- Bench (stand): park the robot with gains on — no creep, no buzz, over 60 s.
- Re-run `estop_unlosable_bench.py` with gains on: still 10/10.

## Related

- `docs/code_review/2026-08-02-post-130-wheels-solid-review.md` F8, Part 1
  freshness MINOR.
- `docs/code_review/2026-08-02-sprint-130-midpoint.md` finding 10 (Stage B
  silently dead unless Stage C's `aSteady` is nonzero — a cross-stage gate
  documented nowhere; settle that while in here).
- [[A-move-takeover-wipes-the-controllers-learned-state]] — the other reason
  integrator/bias lifetime is currently accidental.
