---
status: done
tickets:
- 106-003
---

# TestSim::SimApi cannot safely observe a closed-loop decay to exactly zero

From ticket 105-006 (the scripted-twist demo, `tests/sim/system/
scripted_twist_demo_harness.cpp`). Applies to `tests/sim/support/sim_api.{h,
cpp}` (105-004) as currently built.

## Problem

`SimApi::scriptCycleBusResponses()` pre-provisions exactly ONE extra
("mode-activation" or "fresh command") duty write per injected command, at a
single hand-derived cycle index (`pendingEventCycle_`). This is correct for
every scenario built so far (105-004's ramp/stop/deadman scenarios, 105-005's
fault-knob scenarios) because they all keep the commanded `|v_x|` far enough
above the plant's achievable ceiling (`TestSim::kDefaultDutyVelMax`) that the
velocity-PID output stays saturated at +-1.0 duty forever once set — "one
write, then never again" holds for all of them.

A scenario that commands a velocity TARGET the plant can actually reach (most
notably `injectStop()`, target 0, while the plant is mid-ramp) eventually
drives the PID output back out of saturation as the error shrinks. Once
unsaturated, `NezhaMotor`'s write-on-change gate (`nezha_motor.cpp`) issues
several MORE duty writes as the quantized percent counts down toward 0 — none
of which `SimApi`'s single-transition script provisions for. Verified
empirically (105-006): stepping the real `RobotLoop` past
`pendingEventCycle_+4` after an `injectStop()` desyncs the shared `I2CBus`
script FIFO — an unscripted write consumes an entry meant for a different
device's encoder request — producing directly observable corruption in
decoded telemetry (`connRight` flipping false, `velLeft` freezing at a wrong
value, a false `kFaultWedgeLatch` trip a few cycles later).

Practical consequence: no scenario built on `SimApi` today can safely observe
a full closed-loop settle to exactly zero after a target change — only the
first ~4 cycles of the transition. `tests/sim/system/
scripted_twist_demo_harness.cpp`'s own STOP phase documents and works within
this exact bound (asserts a >50% velocity drop within the safe window, not
arrival at zero) rather than exceeding it.

## Direction

Extend `SimApi::scriptCycleBusResponses()` to script writes based on whether
`appliedDuty()` (or the freshly-computed PID output) actually differs from the
last cycle's, rather than a single hand-derived `pendingEventCycle_` index —
i.e. detect "did this leaf's duty change" dynamically each cycle instead of
assuming exactly one transition per injected command. This generalizes the
harness to any scenario that lets a PID target settle rather than stay
permanently saturated, at the cost of the current design's "exact counts
tractable by hand" simplicity (`sim_api.h`'s own "Plant/PID tuning" section).

Low priority relative to what it unblocks; no current ticket needs a
literal-zero-convergence assertion. Fold into a future sim_api ticket
(sprint 106's own profile-validation work is the most likely first consumer)
rather than a standalone ticket.
