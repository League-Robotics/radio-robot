---
status: done
tickets:
- NONE
---

# TestGUI: live-view avatar yaw uses velocity course (`heading_rad`) instead of tag orientation

## Symptom

In PLAYFIELD live view the avatar's yaw is wrong/unstable. Stakeholder: "the
yaw is still wrong. You're supposed to be getting the yaw for the avatar from
the camera [tag]."

## Root cause (verified numerically, 2026-07-01)

`live_view.py::_capture_and_emit` picks the avatar yaw as:

```python
h_rad = getattr(rec, "heading_rad", None)
tyaw = float(h_rad) if h_rad is not None else float(getattr(rec, "yaw", tyaw))
```

But `heading_rad` from the aprilcam daemon is the **velocity
course-over-ground**, not the tag orientation. Two live samples taken hours
apart both show `heading_rad == atan2(vel_world.y, vel_world.x)` exactly:

- sample 1: heading_rad = −98.07°, atan2(−0.051, −0.007) = −98.1°
- sample 2: heading_rad = +76.1°, atan2(1.034, 0.256) = +76.1°

Meanwhile the tag's true orientation (`yaw` on the client TagRecord /
`orientation_yaw` via MCP) read ≈0.9° with the robot physically facing that
way. For a stationary robot the velocity is sensor noise, so `heading_rad`
wanders arbitrarily → the avatar's rotation is wrong and jittery; when
reversing it points backwards.

The proven Sync Pose path already does it right:
`robot_radio/robot/sync_pose.py::daemon_read_pose` uses **`t.yaw`** with the
documented rationale "daemon reports tag orientation in world frame (0 = east,
CCW+). That orientation IS the robot's forward heading — no offset."

## Fix

In `live_view.py::_capture_and_emit`, take the avatar yaw from the tag
orientation exactly as `sync_pose.daemon_read_pose` does:

```python
y = getattr(rec, "yaw", None)
if y is not None:
    tyaw = float(y)
# else hold last-known yaw (existing _last_tag semantics)
```

Do NOT use `heading_rad` at all for the avatar. Keep position (`world_xy`) and
last-known-pose semantics unchanged.

## Affected code

- `host/robot_radio/testgui/live_view.py` — `_capture_and_emit` (yaw pick) +
  docstring.
- `tests/testgui/test_live_view.py` — assert the emitted `tyaw` equals the
  fake TagRecord's `yaw` even when a (different) `heading_rad` attribute is
  present; assert `heading_rad` is ignored.

## Relation

Same live-view session as [[testgui-set-background-yanks-avatar]] (position
lock). This closes the yaw half of the stakeholder's requirement: avatar
locked to the tag in position AND yaw.
