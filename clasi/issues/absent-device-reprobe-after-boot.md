---
status: pending
sprint: '112'
---

# Re-probe absent devices after boot (slow background re-detection)

From the 2026-07-13 code review (docs/code_review/2026-07-13-devices-drive-review.md,
Part 1 finding **m1**). Applies to the single-loop rebuild's preamble/perception design
(the current fiber preamble has the same limitation).

## Problem

Device detection runs once at boot: a device absent (or transiently failing) during the
preamble latches `present()==false` forever — OTOS after ~2 s of retries, color/line
after ~1 s. A sensor plugged in after power-up, or one that missed its probe window due
to a transient bus error, is invisible until reboot. There is also no bus-wedge
detection + re-init path.

## Direction

Add a slow (seconds-scale) re-probe slot for absent devices to the perception
round-robin — absent devices get one probe attempt every N seconds instead of being
skipped forever. Present/connected semantics stay as they are (absence remains a
first-class state); re-probe must respect the bus schedule (runs in a perception slot,
never in a motor window). Boot telemetry already reports per-device status under the
rebuild, so a late detection becomes visible to the host the moment it happens.

Low priority relative to the rebuild itself; fold into the rebuild's perception design
if convenient, otherwise a follow-up.
