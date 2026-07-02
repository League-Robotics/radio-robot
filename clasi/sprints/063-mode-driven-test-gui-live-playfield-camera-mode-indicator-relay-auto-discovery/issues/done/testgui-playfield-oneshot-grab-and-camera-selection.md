---
status: done
sprint: '063'
tickets:
- 063-008
---

# TestGUI: One-shot playfield grab reliability + camera-selection mismatch

## Symptom (reported, uncertain)

The ability to update the playfield image even once may be broken.

## Findings

The one-shot grab path (`OpsController.trigger_live_grab` →
`_capture_playfield_frame_and_calib`) delivers its result via its own,
**correctly bridged** `_GrabBridge` (a main-thread `QObject` with a
`QueuedConnection` slot), and AprilCam returns a good frame — so the one-shot
grab most likely still works. This should be confirmed live.

### Camera-selection inconsistency (real, worth fixing)

Different code paths pick the camera differently:

- `_capture_playfield_frame_and_calib` (one-shot grab) selects the camera whose
  name/index contains the playfield index (`_PLAYFIELD_CAMERA_INDEX` = 3 →
  "Arducam OV9782").
- `live_view.py::_capture_and_emit` (live worker) and
  `operations.py::_read_daemon_pose` use **`cams[0]`** — the *first* enumerated
  camera. In the MCP enumeration the first camera was "Brio 501" (index 1), not
  the Arducam playfield camera.

If the daemon enumerates more than one camera, the live-view and pose-read paths
may read the **wrong (possibly uncalibrated) camera** and silently produce no
usable frame/pose. This needs confirmation against the **daemon's own**
`list_cameras()` (the MCP session enumeration may differ from what the testgui's
`DaemonControl` sees).

## Fix direction

- Make all three paths select the playfield camera consistently (by the
  configured playfield index/name), instead of `cams[0]`.
- Verify the one-shot grab live in Relay mode.

## Affected code

- `host/robot_radio/testgui/operations.py` — `_capture_playfield_frame_and_calib`,
  `_read_daemon_pose`.
- `host/robot_radio/testgui/live_view.py` — `_capture_and_emit`.

## Relation to other issues

Closely related to the live-update bug
([[testgui-playfield-not-live-updating]]); both concern the Relay/Playfield
camera path. Fixing camera selection here won't help until the live-frame
delivery bug is also fixed.
