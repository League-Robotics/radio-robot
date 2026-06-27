---
date: 2026-06-12
sprint: 032
category: ignored-instruction
---

# Consult the documentation site before reverse-engineering

## What Happened

During the sprint 032 hardware bench validation I could not get the host to talk
to the robot through the radio relay. Instead of consulting the project
documentation — which the stakeholder had previously told me lives at
**https://robots.jointheleague.org/** — I spent roughly two hours
reverse-engineering the relay's behavior from first principles: chasing a
macOS "Resource busy" port conflict (VS Code serial monitor), DTR/HUPCL
reset red herrings, raw-serial probes, and the reader-thread/corr-id reply
path. The actual answer was a documented protocol: open the relay with DTR
asserted, send `!GO` to enter the data plane, then send plain commands (no `>`
prefix). The stakeholder ultimately handed me the working sequence from their
serial monitor and corrected me: "you didn't read the documentation… Please
ensure you always remember where the documentation is."

## What Should Have Happened

At the first sign of unknown protocol/hardware behavior (the robot not
answering over the relay), I should have **WebFetched
https://robots.jointheleague.org/** and its relay/protocol pages, found the
`!GO` data-plane handshake, and implemented it directly — saving ~2 hours and
several rounds of the stakeholder's time replugging hardware.

## Root Cause

Ignored/unrecorded instruction. The stakeholder had told me where the docs were
in an earlier session, but I never recorded it to memory, so it wasn't in front
of me, and I defaulted to reverse-engineering instead of reading. There was also
no standing guardrail ("docs-first for hardware/protocol questions") to catch the
omission.

## Proposed Fix

1. **Done — durable memory:** wrote `project-documentation-site.md` (and a
   top-of-index line in MEMORY.md) recording that ALL docs are at
   https://robots.jointheleague.org/ and that it must be WebFetched FIRST for any
   protocol/hardware/relay/radio/firmware/tooling question before
   reverse-engineering. Cross-linked from `relay-go-data-plane-protocol.md`.
2. **Behavioral guardrail:** treat "I don't know how this hardware/protocol
   behaves" as a trigger to fetch the docs site before writing diagnostic code.
   Applies especially to comms, calibration, and firmware-command questions.
3. **Note for the host tooling:** the docs describe the `!GO` relay protocol, but
   `robot_radio`/`rogo` still use the old `>`-prefix protocol and can't reach the
   robot through the current relay firmware — a real gap to reconcile against the
   documented protocol (separate from this reflection).
