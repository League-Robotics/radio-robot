---
status: pending
priority: medium
---

# `Preamble::probeSlot` has no timeout: the most common bench failure produces the least informative symptom the firmware can produce

## Description

Post-mortem **recommendation #3**; post-130 review Part 1 NOTE.

When the motor brick is unpowered — the single most common bench condition —
`Preamble::probeSlot`'s I2C probe never returns. The core spins in
`codal::system_timer_wait_cycles` with IRQs masked, so even serial DMA stops.
The observable is:

**no banner, no error, nothing on the wire at all.**

That is indistinguishable from a dead board, a bad cable, a wrong port, or a
bricked chip. It cost a debugger session **four times in two days** during
sprint 130 — each time ending in the same diagnosis, reached the same slow way
(`pyocd halt` + `reg pc lr sp`, then a `gdb backtrace`).

## Why this is worth its own issue

It does not fix the wedge — the wedge is a physical power problem
([[A-next-physical-bench-session-checklist]] item 1). It fixes the *diagnosis
time*, which is where the hours actually went, and it converts the project's
most-hit failure from "attach a debugger" to "read the line."

It is also the observability face of the same contract
[[B-observability-contract-is-inert-as-shipped]] covers: the firmware's stated
posture is loud failure, and its most frequent real-world failure is silent.

## What to do

Bound the probe and report the outcome:

- A per-slot timeout on the probe transaction, sized well under the boot budget.
- On timeout, continue boot and emit a cleartext line naming the slot —
  something a human or a script reads without a debugger, e.g.
  `probe: slot 1 no answer (motor power?)`.
- The result belongs in the banner or `STATUS` so it survives past boot, and
  ideally in a telemetry bit so a host can see it mid-session.

Note the constraint that makes this non-trivial: the wedge happens with IRQs
masked inside the CODAL/NRF52 I2C driver's own wait, so a timeout has to be
enforced at or above the `Devices::MicroBitI2CBus` layer rather than by
expecting the vendor call to return. Do **not** edit CODAL —
`.claude/rules/` and prior sprints are explicit about that.

## Verification

- With the motor brick unpowered, the robot boots, emits a banner, and names the
  unanswered slot on the wire within a bounded time.
- With the brick powered, boot behavior and timing are unchanged (measure
  `cycleBusy`/`cyclePeriod` before and after).
- `STATUS` reports the probe outcome after boot.
- The recipe in `.clasi/knowledge/`'s silent-robot note gets an update saying
  the loud path now exists.

## Related

- `docs/post-mortem/2026-08-02-sprint-130-wheels-solid.md` recommendation 3,
  process observations.
- `docs/knowledge/2026-08-02-sprint-130-residuals-and-what-went-wrong.md` —
  "the bench is unreliable and that is its own problem."
- [[A-next-physical-bench-session-checklist]] — the physical half.
