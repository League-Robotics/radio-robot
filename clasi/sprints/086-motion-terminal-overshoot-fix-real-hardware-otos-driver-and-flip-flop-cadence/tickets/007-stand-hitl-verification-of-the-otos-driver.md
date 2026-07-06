---
id: "007"
title: "Stand HITL verification of the OTOS driver"
status: open
use-cases: [SUC-005, SUC-006, SUC-007]
depends-on: ["006"]
github-issue: ""
issue: nezha-hardware-otos-driver-for-new-source-tree.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Stand HITL verification of the OTOS driver

## Description

Final ticket in the OTOS driver set. Depends on ticket 006 (the leaf +
`NezhaHardware` wiring). Closes the parent issue
(`nezha-hardware-otos-driver-for-new-source-tree.md`).

**Hardware dependency (flagged explicitly, not discovered mid-execution):
this ticket requires the physical SparkFun OTOS sensor connected to the
robot under test.** If unavailable at execution time, tickets 005/006 can
still be complete and reviewed on their own merits (host-testable against a
scripted `I2CBus` fake) — only this ticket's HITL gate blocks on hardware
availability. If the sensor is unavailable, report that explicitly rather
than skipping the gate silently; do not claim this ticket is done without
it.

## Acceptance Criteria

- [ ] `NezhaHardware::odometer()` confirmed returning a non-null,
      OTOS-backed leaf on the deployed build.
- [ ] On the stand: OTOS position and velocity reads change plausibly
      (correct sign/magnitude) as the robot or a wheel is moved by hand.
- [ ] `TLM`'s `pose=`/`otos=` fields are live (not `ERR nodev`/absent) on
      real hardware.
- [ ] All seven OTOS wire verbs (`OI`/`OZ`/`OR`/`OP`/`OV`/`OL`/`OA`) ack
      `OK` against the real robot, matching `docs/protocol-v2.md` §11's
      documented reply shapes.
- [ ] A pure spin-in-place on the stand produces bounded residual
      translation (the lever-arm compensation is working correctly with a
      same-instant heading) — not a large phantom offset (the historical
      ~433 mm `db11b7c` failure mode).
- [ ] `OL`/`OA` read back the values the leaf was configured with, matching
      `otos_commands.cpp`'s existing shadow-read contract.
- [ ] The hardware-bench gate's "OTOS alive" check
      (`.claude/rules/hardware-bench-testing.md`) passes.
- [ ] The parent issue is closeable.

## Implementation Plan

**Approach**: Deploy the ticket-006 build to the robot (`mbdeploy
deploy --build` or the project's current deploy path per
`.claude/rules/hardware-bench-testing.md`), open a serial/relay session,
and work through the acceptance criteria above in order: confirm liveness
first (verbs ack), then plausibility (values change correctly), then the
lever-arm-specific spin-in-place check last (the most sensitive check for a
regression of the `db11b7c` failure mode).

**Files to create/modify**: None expected — this is a verification-only
ticket. If the stand pass surfaces a genuine driver bug ticket 006's
sim-only testing couldn't catch (e.g., a real-hardware-only I2C timing
issue), fix it here and document the deviation explicitly, per the
architecture doc's own "verification may find something sim couldn't"
allowance.

**Testing plan**: The stand HITL pass itself, per the acceptance criteria
above, following `.claude/rules/hardware-bench-testing.md`'s standing
verification gate (sensors alive, this being an "OTOS alive" check
specifically).

**Documentation updates**: Close the parent issue file. Record the stand
session's observations (plausible values, spin-in-place residual, verb
acks) in this ticket's completion notes for traceability.
