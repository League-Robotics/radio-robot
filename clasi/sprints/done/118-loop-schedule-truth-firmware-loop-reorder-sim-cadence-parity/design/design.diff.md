---
source_file: design.md
source_hash: 1781125289a12a2b86112407016a7036daba91bd5e1a8b2b006370408c99051c
---
# Diff: design.md

Comparison of the sprint overlay copy of `design.md` against its pristine (seed-commit) canonical version.

```diff
--- design.md (pristine)
+++ design.md (current)
@@ -17,14 +17,15 @@
 commands and streams telemetry, and a Python host package that talks to
 it over USB serial or a radio relay, with a parallel host-build
 simulator for development without hardware. The current architecture
-(post sprint 117, "predict-to-now estimator v1", on `master`) is
+(post sprint 118, "loop schedule truth", on `master`) is
 deliberately minimal — the firmware speaks exactly three inbound
 commands (**MOVE / CONFIG / STOP**): `Move` carries its own velocity
 (twist or wheels variant), a stop condition (time/distance/angle), and a
 required `timeout` backstop, queued 1-active + 4-pending — and emits one
 telemetry frame (**frame v2**: per-wheel `EncoderReading`/`OtosReading`
 with their own sample times, a single `flags` bit-string, a single ack
-slot, packed line/color words) every 20 ms cycle. There is still **no**
+slot, packed line/color words) every 40 ms cycle (118 — restored from a
+fictional 20 ms; see §5's "Pace" step below). There is still **no**
 jerk-limited trajectory solver and no heading-source policy on the
 firmware side — sprint 115 ("gut-to-minimal-firmware S1") deleted the
 old motion stack, and sprint 116 ("MOVE protocol cutover", S2) replaced
@@ -317,9 +318,13 @@
    same cycle's staged `Frame` and refreshes its wheel/body ZOH
    predict-to-now estimates; `App::Telemetry` emits the primary TLM frame
    (or the slower secondary diagnostic frame) through Comms.
-5. **Pace** — a final `runAndWait` paces the cycle to `kCycle` = 20 ms
-   (~50 Hz), matching `Telemetry::kPrimaryPeriod` so every cycle emits a
-   primary frame.
+5. **Pace** — a final `runAndWait` paces the cycle to `kCycle` = 40 ms
+   (~25 Hz), matching `Telemetry::kPrimaryPeriod` so every cycle emits a
+   primary frame. (118 — restores the schedule's genuine 4ms/4ms
+   settle/clearance budget, regressed to a fictional 20ms/~50Hz by commit
+   `5f5a2ba7`; the sim's own `SimHarness::kCycleDtUs` now matches this
+   value exactly, closing the sim/firmware cadence gap — see
+   [`src/sim/DESIGN.md`](../../src/sim/DESIGN.md).)
 
 Boot is a separate loop: `App::Preamble` steps per-device detection (one
 bounded probe per pass) while telemetry frames report detection status;
```
