---
status: in-progress
sprint: 091
tickets:
- 091-003
---

# Serial-silence watchdog: arm only while motors are running (+ bench verification)

## Context

The serial-silence safety watchdog (comms-silence → immediate motor neutralize +
`EVT dev_watchdog`) is the runaway-prevention feature. It was preserved through the
sprint 087 greenfield loop rewrite (shipped in `v0.20260707.1`): it now lives in the
cyclic-executive `source/runtime/main_loop.{cpp,h}` using `SerialSilenceWatchdog`
(`source/commands/dev_commands.h`) — `check()` in the mandatory portion, immediate
`emergencyNeutralize()` bypass on fire, `DEV WD`-settable window, fire-once `EVT`.

This issue supersedes the buried `preserve-serial-silence-safety-watchdog-in-
greenfield-loop.md` issue (archived inside `sprints/done/087-…/issues/`, no longer a
live/trackable item — the reason this fresh issue exists).

## Problem — fires regardless of whether motors are running

The watchdog is fed by **any** inbound statement (any channel, any content) and fires
whenever no statement arrives within the window — **even when the robot is idle with
the motors stopped**. So sitting still with the host quiet produces a spurious
neutralize + `EVT dev_watchdog`, when there is no runaway to prevent. The watchdog's
whole purpose is to catch a *live drive command* going unmonitored; when nothing is
driving, it should stay quiet.

## Desired behavior

**The watchdog only fires while the motors are actually running.** When the robot is
stopped / neutral / idle, comms silence must NOT fire it (no neutralize, no `EVT`).
When a live, non-neutral drive command is active and comms go silent past the window,
it fires exactly as today (immediate neutralize + `EVT dev_watchdog`, fire-once).

Notes for whoever picks this up:
- "Motors running" is now readable from the blackboard — `DrivetrainState.active` was
  added in 087 (ticket 003), plus the commanded drive/motor state. Define the precise
  predicate at implementation time (e.g. an active non-neutral drive command / non-zero
  commanded output); prefer the commanded state over measured encoder motion so the
  gate is deterministic.
- The change is to the **fire gate**, not necessarily the feed source — silence
  detection (feed on any statement) can stay; only firing gets gated on motors-running.
  Keep the same-pass `emergencyNeutralize()` bypass, the fire-once `EVT`, and the
  `DEV WD` settable window unchanged for the running case.

## Acceptance

- **Idle:** motors stopped/neutral + comms silence past the window → **no** neutralize,
  **no** `EVT dev_watchdog`.
- **Driving:** a live drive command + comms silence past the window → immediate
  same-pass neutralize + `EVT dev_watchdog` (unchanged runaway protection).
- Sim tests for both cases (extend `tests/sim/unit/test_watchdog_policy.py`).
- **HITL bench (on the stand, over the RADIO path):** a long drive then host silence
  neutralizes the wheels + emits `EVT dev_watchdog`; an idle-then-silence case does
  **not** fire. This also closes out sprint 087's still-unverified watchdog bench
  acceptance (radio-path proves the loop's `uBit.sleep(1)` yield isn't starving radio,
  which would also starve the watchdog's feed over radio).
