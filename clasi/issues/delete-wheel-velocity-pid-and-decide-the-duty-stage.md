---
status: pending
---

# Delete WheelVelocityPid (zero call sites); decide the fate of the discarded duty stage

**Source:** code review 2026-07-30, `02-motion.md` MAJOR §4.
**Priority:** P1 — three generations of the wheel velocity-control law
coexist; only one reaches the wheels.
**Goal served:** when the wheels misbehave, an engineer must know which
controller to suspect. Today the tree offers three answers, two wrong.

## What is wrong

- `Motion::WheelVelocityPid` (`wheel_velocity_pid.{h,cpp}`): **zero
  instantiations** anywhere in `src/firm` — `main.cpp:70-75`'s own comment
  confirms the pair that used it is gone. Dead weight, still compiled into
  the `motion` static library, absent from DESIGN.md.
- `Motion::WheelPid` / `Planner::stageDuty()` (`planner.cpp:248-337`): runs a
  full per-wheel PID (rest-clamp, actuation-lead, stiction kick) every one of
  ~21 cycles/s — and `main.cpp:408-412` says its output "is computed every
  tick and DISCARDED". Only `Motion::WheelTrim` (velocity-domain) actually
  reaches the wheels.

## What to do

1. **Delete `wheel_velocity_pid.{h,cpp}`** and its entry in
   `src/motion/CMakeLists.txt` (remember all four build source lists). No
   caller, no transition plan references it — this is unambiguous.
2. **Decide the duty stage** (stakeholder call, two honest options):
   - **Park it:** stop calling `stageDuty()` from the live tick (keep the
     class + its ctest tiers as the validated basis for a future duty-sink
     cutover, with a DESIGN.md note naming that intent and owner). The loop
     stops spending ~21 PID evaluations/s on a discarded value, and a
     profiler/reader stops finding a live-looking controller that does
     nothing.
   - **Adopt it:** if the duty-sink cutover is actually near, say so in
     DESIGN.md and leave it computing — but then the "DISCARDED" comment
     becomes the plan-of-record with a date, not an indefinite shrug.
3. Either way, `src/motion/DESIGN.md` gains a one-paragraph "wheel control
   generations" note: WheelVelocityPid (deleted), WheelPid duty stage
   (parked/adopted, per the decision), WheelTrim (live, the loop that
   reaches the wheels).

## Acceptance

- `grep -rn "WheelVelocityPid" src/` returns nothing (including the two
  historical doc-comment mentions in `device_config.h`/`nezha_motor.h` —
  update those comments).
- The decision from step 2 is visible in code (call removed or DESIGN.md
  intent note) — not left implicit.
- Clean build, motion_tests + planner ctest suite pass, bench smoke
  unchanged.
