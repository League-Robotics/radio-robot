---
status: done
tickets:
- NONE
---

# TestGUI should warn when the robot's firmware version is stale

## Problem

Today's bench-tour failures were partly caused by the robot running
`fw=0.20260702.20` (built the previous day) while the host tree was at
`0.20260703.19`. That firmware predated the entire sprint-074 OTOS
fusion-recovery work, so `DBG OTOS BENCH 1` swapped a pointer the fusion
path never read (fixed by 074-002) and the warn gate latched fusion off
permanently (fixed by 074's re-admission). Hours of bench behaviour were
un-diagnosable until the version mismatch was noticed via `VER`.

The host already knows both sides: the robot answers
`VER` (`OK ver fw=<version> proto=<n>`) and the host has its expected
version (`source/types/Protocol.h` / `dotconfig` version, importable or
readable at run time).

## Proposed fix

On TestGUI connect, send `VER` and compare against the host's expected
firmware version:

- Show the firmware version in the status bar next to the transport
  label.
- If it differs from the host version (or protocol version mismatches),
  show a prominent warning banner ("robot firmware 0.20260702.20 ≠ host
  0.20260703.19 — reflash before bench testing").

Optionally the same check in `robot_radio.testkit.make_target` for bench
scripts (print a warning, `--strict` to abort).
