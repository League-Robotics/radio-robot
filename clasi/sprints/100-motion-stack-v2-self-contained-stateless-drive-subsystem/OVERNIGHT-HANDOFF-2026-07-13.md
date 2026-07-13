# Overnight handoff — 2026-07-13

Mission (stakeholder, before bed): get encoders working reliably on hardware,
do a complete cutover to the DeviceBus hardware layer, validate it, then
continue the sprint. **All three headline goals done. Sprint 100 at 10/13.**

## 1. Encoders reliable on hardware — DONE

Two real hardware bugs found via the DeviceBus bring-up image and fixed:
- **Velocity glitch-gate**: the fiber cycles ~16ms but the Nezha brick refreshes
  encoders ~80ms, so a fresh multi-cycle position step got divided by one
  cycle's dt, tripped the >2000mm/s glitch filter, and velocity stuck at 0
  (which also false-latched the wedge). Fixed: compute velocity on genuinely
  fresh samples using dt-since-last-fresh (mirrors sprint 099-005).
- **Motor-2 request/collect starvation** (alternating per-motor encoder
  request/collect).

Validated on hardware: both motors, both directions, steady velocity tracks
the commanded 200 mm/s to ~1 mm/s, repeatable across runs.

## 2. Complete DeviceBus cutover — DONE + VALIDATED ON ROBOT

DeviceBus (fiber-owned I2C/motor/sensor subsystem) is now the LIVE device layer
of the real firmware, replacing NezhaHardware + its flip-flop.

- **Adapter approach** (`source/subsystems/device_bus_hardware.{h,cpp}`):
  `Subsystems::DeviceBusHardware : Hardware` + thin `DeviceBusMotor`/
  `DeviceBusOdometer` forwarding leaves over the DeviceBus handles. `tick()` is
  a no-op (the fiber owns collect+PID+armored-write), so NO double-PID. The
  entire motion-v2 stack (Drivetrain, PoseEstimator, MainLoop) and the sim path
  are UNCHANGED — `main.cpp` just swaps `NezhaHardware` → `DeviceBusHardware`.
- **Hardware validation** (see `device-bus-cutover-hardware-validation.md`):
  sensors live (pose/otos/enc via DeviceBus), wheels drive both dirs + spin with
  encoders incrementing, a distance MOVE through `source/drive/` executes
  end-to-end (pose advanced ~196mm on a 200mm command). Standing bench gate met.
- Old stack (NezhaHardware/NezhaMotor/OtosOdometer, and segment_executor/
  stop_condition from the motion cutover) is PARKED on disk, not deleted —
  ticket 013 removes it after the field sign-off.

## 3. Sprint 100 — 10/13 tickets done (all host/sim + hardware-validated cutover)

Done: 001 schema · 002 drive core · 003 facade · 004 tracker · 005 policy/
terminal machine · 006 tier-0 Python suite · 007 motion-v2 cutover (validated)
· DBX DeviceBus cutover (validated) · 008 MOVER · 009 trace/plan-dump · 010
fault-knob matrix. **Full suite 1486 passed, 0 failed. Clean build passes.**

Notable finds fixed along the way: the tracker sign-convention bug (would have
made the loop unconditionally unstable — caught in host tests before hardware),
the 093 duty-write hazard structurally excluded by the fiber's call order, and
`StreamControl.trace`/`bb.motionTrace` never actually wired (fixed in 009).
Bit-exact cross-tier replay proven (a MotionTrace from sim replays through the
tier-0 step() with every field diffing 0.0).

## What's LEFT for you (needs the robot OFF the stand)

These three can't be done on the stand — turn/distance accuracy needs the body
free to move:
- **011 bench**: arc/turn-sweep accuracy grids, plateau measurement to pin
  `v_wheel_max` in tovez.json, gain/envelope tuning, the 098 pivot-accuracy grid
  re-run. Robot on the FLOOR.
- **012 field**: camera-verified multi-segment chains + live PoseFix, on the
  PLAYFIELD (aprilcam + a camera — none was connected overnight).
- **013 cleanup**: delete the parked old stack (NezhaHardware/NezhaMotor/
  OtosOdometer/segment_executor/stop_condition), retire heading_kp/kd +
  governRatio segment path. Correctly waits for the 012 field sign-off so we can
  revert if the field test surfaces anything.

## State of the robot / repo

- Robot currently holds the **sprint-100 cutover firmware** (validated, working).
- Branch `sprint/100-...` has everything; NOT merged to master (awaits 011-013 +
  your close call). `wt-devicebus` worktree still exists (its work is merged in;
  safe to `git worktree remove` when convenient).
- Known telemetry gaps documented in the cutover ticket: wedged/wedgeSuspect/
  hardResetCount/acceleration read inert through the adapter (non-virtual base
  accessors — DeviceBus's own armor is live, only the msg::/TLM surfacing is a
  gap); live CFG/OI/OR/OL/OA reconfigure is accepted-inert; color/line sensors
  not yet bridged to the Hardware interface (not in the motion path).
- `docs/protocol-v3.md` is stale after the 099/100 wire changes (flagged by
  several tickets) — a doc-maintenance pass to schedule.
