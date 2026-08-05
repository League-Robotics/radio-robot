---
status: pending
priority: medium
---

# One physical quantity, many constants: the speed floor and the duty map each have three owners

## Description

2026-08-02 post-130 review, **F7 (MAJOR)**, minus the period half (which is
[[A-nominal-50ms-vs-delivered-54ms]]). Sprint-130 midpoint finding #5 covers the
duty map independently.

This is the exact `duty_per_speed`/`wheel_gain` disease sprint 130 was chartered
to kill — alive at two other layers.

## The speed floor: two constants

| Where | Value |
|---|---|
| firmware `wheel_v_min` | **99.7** mm/s (`drive.cpp:150-156`) |
| host taper `_UNMANAGED_FLOOR` | **90.0** mm/s (`testgui/transport.py:200-201`) |

The host tapers to a floor the firmware does not have, so the last segment of
every unmanaged drive is negotiated between two different ideas of "slowest".

## The duty map: three or four

| Where | Value |
|---|---|
| firmware baked `Drive::kDutyPerSpeed` | **0.001182** (`drive.h:138`, "MEASURED, NOT CONFIGURED") |
| robot JSON `duty_per_speed_*` | **0.00187325** — still generated, deliberately ignored (`boot_calibration.cpp:83-93`) |
| `duty_sweep.py`'s `KNOWN_DUTY_PER_SPEED` | hand-mirrored copy of the firmware constant |
| the sim | `SimLoop.configure_from_robot()` pushes the **JSON** value (`sim_loop.py:715-719`) |

So a TestGUI sim session runs a **third** feedforward, ~58% off the one hardware
uses, while `sim_harness.h:117-121` comments that it uses "the same values as
hardware" — false.

There is a fifth: `SimHarness` defaults 0.002.

Note also that the JSON's own numbers are superseded: `tovez.json`'s 08-01 note
refits left fwd at **1101.3** mm/s per duty (offset −14.2) and rev at 1023.1,
explicitly superseding the 853.6 the baked constant descends from — while
`drive.h:120-137`'s provenance comment still tells the 853.6 story and cites a
superseded issue (midpoint finding #8).

## What to do

Per layer, pick the one owner and make everything else read it.

- **Floor**: one constant, firmware-side, surfaced to the host. The host taper
  reads it rather than carrying 90.0. Note the floor's *semantics* are separately
  broken — see [[A-speed-floor-snaps-the-planner-differential]] — and the real
  loaded value is unmeasured ([[A-next-physical-bench-session-checklist]] item 4).
  Sequence: measure, then set one owner, then fix the differential semantics.
- **Duty map**: delete the dead JSON keys and their generator path (the cleanup
  lost its tracking issue when `04-continuous-...` closed), point the sim and
  `duty_sweep.py` at the firmware's value, and correct `sim_harness.h`'s comment
  and `drive.h`'s provenance block.

**What actually makes this stick is config read-back**
([[A-no-firmware-to-host-config-readback]]) — without it, the host copies are
re-derived by hand every time and drift again. Consider sequencing that first.

## Verification

- `grep -rn "duty_per_speed\|dutyPerSpeed\|DUTY_PER_SPEED" src/ data/` shows one
  authoritative definition and readers, no independent literals.
- A TestGUI sim session and hardware run the same feedforward — assert it in the
  composition-root parity test, which today checks only `PlannerLimits` and is
  blind to `WheelControllerBootConfig` and the sim's `setDutyPerSpeed()`
  override (midpoint finding #9).
- The host taper and the firmware agree on the floor.
- No comment claims parity that the parity test does not check.

## Related

- `docs/code_review/2026-08-02-post-130-wheels-solid-review.md` F7, S3, Part 2 C5.
- `docs/code_review/2026-08-02-sprint-130-midpoint.md` findings 5, 8, 9.
- `docs/post-mortem/2026-08-02-sprint-130-wheels-solid.md` root cause 2
  (circular calibration) — this is how it recurs.
