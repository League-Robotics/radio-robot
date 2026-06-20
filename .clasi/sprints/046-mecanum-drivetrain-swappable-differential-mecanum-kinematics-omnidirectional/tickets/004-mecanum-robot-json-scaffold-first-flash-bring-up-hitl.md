---
id: "004"
title: "Mecanum robot JSON scaffold + first-flash bring-up (HITL)"
status: open
use-cases:
  - SUC-002
  - SUC-003
depends-on:
  - "046-003"
github-issue: ""
issue: ""
completes_issue: false
hardware-in-the-loop: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 046-004: Mecanum robot JSON scaffold + first-flash bring-up (HITL)

## Description

Create the mecanum robot JSON configuration file with known values and
calibration placeholders, flash the mecanum robot for the first time, read
the 5-char micro:bit announcement name, and confirm each of the 4 motors spins
in the correct direction.

**This ticket is hardware-in-the-loop. The programmer writes the JSON and
prepares the build; the team-lead flashes the robot and runs the bench checks.**

## Approach

### Programmer deliverables (before HITL run)

#### 1. Stub robot JSON: data/robots/mecanum-proto.json

Create a placeholder JSON (name `mecanum-proto` until the real 5-char name is
read from the hardware):

```json
{
  "schema_version": 2,
  "identity": {
    "robot_name": "PLACEHOLDER_5CHAR",
    "uid": "mecanum-proto",
    "hardware_model": "DFRobot Nezha / mecanum chassis",
    "common_name": "mecanum-bot",
    "drivetrain_type": "mecanum"
  },
  "connection": {
    "device_announcement_name": "PLACEHOLDER_5CHAR"
  },
  "vision": {
    "robot_tag_id": 101,
    "tag_offset_mm": { "x": 0.0, "y": 0.0, "yaw_rad": 0.0 }
  },
  "geometry": {
    "odometry_offset_mm": { "x": -51.5, "y": 0.0, "yaw_rad": 0.0 },
    "odometry_chip_upside_down": false,
    "trackwidth": null,
    "wheelbase_mm": null
  },
  "mecanum_geometry": {
    "half_track_mm": null,
    "half_wheelbase_mm": null
  },
  "wheels": {
    "wheel_diameter_mm": null,
    "ticks_per_rev": 360,
    "ticks_per_mm": null
  },
  "encoders": {
    "has_encoders": true,
    "encoder_count": 4
  },
  "drive": {},
  "gripper": { "has_gripper": false, "gripper_offset_mm": null },
  "peripherals": { "laser_port": null },
  "calibration": {
    "otos_linear_scale": 1.05,
    "otos_angular_scale": 0.987
  },
  "mecanum_calibration": {
    "fwd_sign_fr": -1,
    "fwd_sign_fl": 1,
    "fwd_sign_br": -1,
    "fwd_sign_bl": 1,
    "mm_per_wheel_deg_fr": null,
    "mm_per_wheel_deg_fl": null,
    "mm_per_wheel_deg_br": null,
    "mm_per_wheel_deg_bl": null
  },
  "control": {
    "vel_kp": 0.3,
    "vel_ki": 0.5,
    "vel_kff": 0.15,
    "vel_imax": 20.0,
    "vel_kaw": 3.0,
    "vel_filt": 0.15,
    "sync": 0.0,
    "turn_gate": 35,
    "yaw_rate_max": 180,
    "arrive_tol_mm": 25
  }
}
```

Note `sync: 0.0` (sync-coupling disabled for mecanum — independent per-wheel PI).

#### 2. data/robots/active_robot.json — point at the stub

```json
{ "path": "data/robots/mecanum-proto.json" }
```

#### 3. Build verification (programmer runs before handing to team-lead)

Run `python build.py` with the mecanum stub active; confirm it exits 0 and
produces `MICROBIT.hex`. Run `uv run --with pytest python -m pytest tests/simulation -q`
to confirm 2093 passed with the stub config active (sim build uses differential
by default so this is the differential sim — should still be green).

### Team-lead HITL deliverables

#### 4. First flash

- Copy `MICROBIT.hex` to the mecanum micro:bit (USB drag-and-drop or `pyocd`).
- Open the radio relay; wait for the robot announcement.
- Record the **5-char announcement name** (e.g. `"WUBAX"`).

#### 5. Rename and finalize JSON

After reading the name:
- Rename `data/robots/mecanum-proto.json` → `data/robots/<5char>.json`.
- Update `identity.robot_name`, `identity.uid`, and
  `connection.device_announcement_name` to `<5char>`.
- Update `data/robots/active_robot.json` pointer accordingly.

#### 6. Motor direction verification

Send one-wheel commands using `VW`/raw motor commands to spin each motor
individually. Confirm:

| Port | Motor | Expected direction for +speed | fwd_sign |
|------|-------|-------------------------------|----------|
| 1    | FR    | CW looking from above         | -1       |
| 2    | FL    | CCW looking from above        | +1       |
| 3    | BR    | CW looking from above         | -1       |
| 4    | BL    | CCW looking from above        | +1       |

If FL rolls BACKWARD on a +150 mm/s forward command (all four wheels
simultaneously via `VW 150 0`), flip all four signs together (left side and
right side must maintain opposite polarity: FL/BL same sign, FR/BR same sign,
but the pair polarity inverts). Update the JSON `fwd_sign_*` values and
rebuild.

#### 7. Commit

Commit the renamed JSON and the updated `active_robot.json` pointer. The
`mecanum-proto.json` stub is deleted.

## Files to Create

- `data/robots/mecanum-proto.json` (stub; renamed to `<5char>.json` after HITL)

## Files to Modify

- `data/robots/active_robot.json` (point at stub; updated to `<5char>.json` after HITL)

## Acceptance Criteria

- [ ] Mecanum robot JSON exists at `data/robots/<5char>.json` with the correct
      `device_announcement_name` matching the actual micro:bit name.
- [ ] `python build.py` with the mecanum JSON active exits 0 and produces
      `MICROBIT.hex` with `ROBOT_DRIVETRAIN_MECANUM` defined.
- [ ] Robot powers on, connects over radio relay (team-lead confirms
      announcement name appears).
- [ ] All 4 motors spin when commanded; team-lead confirms each spins in the
      expected forward direction (FL/BL forward = CCW, FR/BR forward = CW or
      equivalent per the physical chassis orientation).
- [ ] `fwd_sign_*` values in the JSON match observed motor directions.
- [ ] Switching `active_robot.json` back to `tovez.json` and rebuilding
      restores the differential build (SUC-001 gate: DefaultConfig.cpp diff
      additive-constant only, 2093 sim passed).
- [ ] `uv run --with pytest python -m pytest tests/simulation -q` still reports
      `2093 passed` with mecanum JSON active (sim uses differential by default).

## Testing

- **Regression gate**: `uv run --with pytest python -m pytest tests/simulation -q`
- **HITL verification**: team-lead manually confirms announcement name + motor directions.
- **No new automated tests**: motor direction is bench-verified by observation.

## Implementation Notes

- The `fwd_sign` values given in this ticket (`FL=+1, FR=-1, BL=+1, BR=-1`)
  are the bench-confirmed values from the issue design doc. If the physical
  bench run contradicts these, update the JSON and note the discrepancy in the
  commit message.
- `sync: 0.0` in the JSON is critical — the sync-coupling formula in
  `MotorController` is undefined for mecanum and is gated out in T5. Until T5
  lands, the firmware already disables sync for mecanum via the `#ifdef` in
  the control code. The JSON value of 0 is belt-and-suspenders.
- Color sensor: present. Line sensor: absent (begin() fails gracefully).
- The OTOS offset `-51.5 mm` in `odometry_offset_mm.x` is the bench-measured
  value from the issue. Do not change without re-measurement.
