# Bug report for AprilCam: live camera frame is vertically mirrored (Y / yaw)

**Updated:** 2026-06-16 · **Reporter:** radio-robot-c (client of the camera/daemon)
**Camera:** `arducam-ov9782-usb-camera` · **Playfield:** `main-playfield` (134.3 × 89.3 cm, A1-centred)

## TL;DR
The live camera frame is **vertically mirrored** relative to the physical field: world
**+Y currently maps to physical SOUTH**. Position Y is inverted, and after an earlier
(mis-guided, now retracted) yaw change, the yaw is flipped *with* it — so the frame is
**internally consistent but still physically mirrored**, which makes internal/relative
checks falsely pass while a robot that drives to map coordinates goes to the Y-mirror
(and into the boards).

## Unambiguous physical evidence (known, physically-placed colored squares)
The operator placed the colored squares at fixed physical spots; the map matches that
placement. Live `get_objects` detects the **physically-SOUTH** squares at **+Y (north)**:

| Square (physical placement) | Map / true position | Camera detects at |
|---|---|---|
| **green** = south-east | (+35, **−24**) | (+35.4, **+24.8**) |
| **blue**  = south-west | (−35, **−24**) | (−36.1, **+25.0**) |

(The physically-NORTH squares — black/orange/purple — are correspondingly detected at −Y.)
So **world +Y = physical SOUTH**. X is correct. This is a pure vertical mirror.

## History (so you can see what changed)
1. First report: position Y inverted (squares Y-mirrored). 
2. You fixed something; we then measured `travel_direction = −reported_yaw`. **That measurement
   is ambiguous** — it is equally "position true + yaw flipped" OR "position flipped + yaw true."
   We guessed the former and asked you to **negate the yaw**. **Please disregard that ask.**
3. Negating the yaw produced the current state: position **flipped** + yaw flipped =
   a consistent mirror that passes relative checks but is physically wrong (above).

## Correct target / acceptance (single right-handed ENU frame)
Make BOTH position and orientation match the **physical** field — not merely agree with
each other:
- `get_objects`/`get_tags`: **green at (+35, −24)** and **blue at (−35, −24)** (south = −Y).
- A tag physically facing **north** reads `orientation_yaw ≈ +90°` (ENU: +X east, +Y north, CCW+).
- A robot driving **physically forward** has `travel ≈ reported_yaw` (DIR err ≈ 0), **and**
  moving physically north increases world Y.

## Notes
Robot side is verified correct (drive, turn, heading, forward all check out); it's being
fed a mirrored frame. We made no changes to the AprilCam repo and are not patching this
client-side.
