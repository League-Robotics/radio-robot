---
status: done
sprint: '074'
tickets:
- 074-001
- 074-002
- 074-003
- 074-004
---

# OTOS not being used — pose frozen all session, ekf_rej climbing every tick, EKF running encoder-only

## Description

Observed 2026-07-02 across every recording of the day, on the stand AND
while driving 500 mm on the playfield in Playfield mode:

- The TLM `otos=` pose is **frozen for the entire session** — e.g.
  `otos=46,-5,-198` identical across all 40 frames of
  `recordings/latest.jsonl` while the robot physically drove ~500 mm and
  spun ~40°. Earlier session: pinned at `47,-3,2` for all 73 frames of
  `recording_20260702_194340.jsonl`. The values differ *between* sessions,
  so something updates at/near boot, then never again.
- `ekf_rej` increments essentially every EKF tick, all run long (e.g.
  10 → 187 over one 6.6 s recording; ~+6 per 210 ms TLM frame) — the EKF
  gate is rejecting every OTOS measurement, consistent with a frozen input
  that immediately disagrees with encoder prediction.
- Net effect: the EKF runs **encoder-only**. `pose` tracks `encpose` and
  inherits all encoder pathologies (during the D-drive thrash the fused pose
  briefly jumped to x=135→176 on phantom wheel motion before snapping back).

Additionally, per the stakeholder: **bench mode is supposed to simulate the
OTOS**, and that is not happening either — a bench-mode run shows the same
frozen-OTOS signature instead of a simulated moving OTOS.

### Why it matters

- Position/heading corrections are silently absent; drives complete on raw
  odometry with no cross-check. The `ekf_rej` counter is currently the only
  wire-visible symptom, and nothing alerts on it.
- Not the cause of the D-drive terminal thrash (that loop is encoder-only by
  design — see `d-drive-terminal-instability-reversal-thrash.md`), but it
  removes the sensor that would have flagged the resulting gross pose error.

### Investigation pointers

- Was the OTOS read failing at the HAL level (I2C), returning stale cached
  pose, or reading fine and being dropped before TLM? Determine what the
  TLM `otos=` field actually reflects (raw read vs last-accepted).
- `Robot::otosCorrect()` runs at the slow cadence in LoopScheduler — check
  its unreadable/warn paths and the CR-06 warn-persistence gate
  (`Drive::_updateOtosFusionGate`, `_otosFusionBlocked`) — a latched
  fusion-block with no re-admission would look exactly like this.
- Known related memory: Tovez OTOS ignores REG_OFFSET writes (lever-arm must
  be host-side); OTOS heading distinguishes stand from floor — a frozen OTOS
  breaks that diagnostic too.
- Bench mode: verify the DBG OTOS BENCH / bench-OTOS-sim path is actually
  engaged by the TestGUI bench transport, and that it moves with the plant.

### Acceptance sketch

- On the playfield, TLM `otos=` visibly tracks a 500 mm drive; `ekf_rej`
  stays near-flat during nominal motion.
- A persistent OTOS read failure or fusion block is surfaced on the wire
  (EVT or TLM health field), not silent.
- Bench mode shows a simulated OTOS pose that tracks commanded motion.
