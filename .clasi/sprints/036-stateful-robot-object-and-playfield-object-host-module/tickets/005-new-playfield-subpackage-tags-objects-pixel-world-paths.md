---
id: '005'
title: 'New Playfield subpackage: tags, objects, pixel-world, paths'
status: done
use-cases:
- SUC-008
- SUC-009
- SUC-010
- SUC-011
depends-on: []
github-issue: ''
issue: plan-stateful-robot-object-playfield-object-for-the-host-module.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# New Playfield subpackage: tags, objects, pixel-world, paths

## Description

Create the new `host/robot_radio/field/` subpackage containing `Playfield`,
`Tag`, and `Feature`. This ticket is independent of T001–T004 (no `Nezha`
dependency) and can be developed in parallel, though it executes after in the
serial plan.

**Files to create:**
- `host/robot_radio/field/__init__.py` — exports `Playfield`, `Tag`, `Feature`.
- `host/robot_radio/field/playfield.py` — the full implementation.

**`Tag` and `Feature` dataclasses:**
```python
@dataclass
class Tag:
    id: int
    x: float   # cm, world frame A1-centred
    y: float   # cm, world frame A1-centred
    yaw: float # rad, CCW-positive

@dataclass
class Feature:
    slug: str
    type: str   # "rectangle", "dot", "april_tag", etc.
    color: str | None
    x: float    # cm
    y: float    # cm
```

**`Playfield` class:**

```python
class Playfield:
    @classmethod
    def open(cls, cam_name=None) -> "Playfield"
        # DaemonControl.connect_default(Config.load()); list_cameras()[0] if cam_name None

    def get_tag(self, tag_id: int) -> Tag | None
        # dc.get_tags(cam) → find tag by id → Tag(id, world_xy[0], world_xy[1], yaw)
        # Returns None if tag absent or world_xy is None

    def tags(self) -> dict[int, Tag]
        # All visible tags from get_tags()

    def get_object(self, slug_or_query: str) -> Feature | None
        # Try dc.where_is(slug_or_query, cam) first (returns matches with x,y)
        # Fall back to scanning playfield.json categories directly
        # Return Feature or None

    def world_to_pixel(self, x_cm: float, y_cm: float) -> tuple[float, float] | None
        # Requires cached H and origin. Returns None if not available.
        # raw = [x_cm + origin_x, origin_y - y_cm, 1]   (y-flip + origin offset)
        # px_h = inv(H) @ raw; normalize: (px_h[0]/px_h[2], px_h[1]/px_h[2])

    def pixel_to_world(self, u: float, v: float) -> tuple[float, float] | None
        # raw_h = H @ [u, v, 1]; normalize to (rx, ry, 1)
        # x = rx - origin_x; y = origin_y - ry

    @property
    def homography(self) -> np.ndarray | None

    def add_path(self, path_id: str, points: list[tuple[float,float]], *,
                 symbol: str = "filled_circle",
                 color: tuple[int,int,int] = (0, 200, 255),
                 size_cm: float = 1.2) -> None
        # Builds or replaces the path_id entry in the in-memory path list
        # Calls _write_paths() atomically

    def add_symbol(self, x_cm: float, y_cm: float, *,
                   symbol: str = "x",
                   color: tuple[int,int,int] = (255, 235, 130),
                   size_cm: float = 1.0) -> None
        # Appends a single-point path or symbol to a special "symbols" path

    def clear_paths(self) -> None
        # Writes [] to paths.json atomically; clears in-memory path list

    def close(self) -> None
        # dc.close()
```

**paths.json schema** (from playfield_random_tour.py:119-139):
```json
[{"path_id": "...", "playfield_id": "...", "waypoints": [
    {"x": 1.0, "y": 2.0, "size_cm": 1.2,
     "symbol": "filled_circle",
     "symbol_color": [0, 200, 255],
     "line_color": [0, 200, 255]}
]}]
```
Atomic write: `write to <path>.tmp`, then `os.replace(tmp, path)`.

**Homography and origin:**
- `H` is sourced from the latest `TagFrame.homography` returned by `dc.get_tags()`.
  It is a flat 9-float list; reshape to `np.array(h_flat).reshape(3,3)`.
- Cache `H` and its inverse; invalidate when a new tag frame is received.
- `origin_x`, `origin_y` come from `playfield.json` `playfield` block (`width_cm`,
  `height_cm`, and the `origin` key if present, else `width_cm/2`, `height_cm/2`
  — inspect `playfield.json` for the exact key names used by the demo at line 49).
- **CRITICAL**: The transform is NOT a naive `H @ [x,y,1]`. It is the A1-centred
  y-up convention from `aprilcam/ui/display.py:411-447`. Replicate it exactly.

**paths.json location:**
`<daemon_data_dir>/cameras/<cam_name>/paths.json`. The daemon data dir is
available from `Config.load()` (same path the demo uses at line 43).

## Acceptance Criteria

- [x] `host/robot_radio/field/__init__.py` exists; `from robot_radio.field import
      Playfield, Tag, Feature` works.
- [x] `Playfield.open()` constructs a `Playfield` without error (no live daemon
      needed in unit tests — `DaemonControl` is mocked).
- [x] `get_tag(tag_id)` returns a `Tag` with correct `x`, `y`, `yaw` from a
      mocked `TagRecord` with `world_xy` and `orientation_yaw`.
- [x] `get_tag(tag_id)` returns `None` when the tag is absent or `world_xy` is
      None.
- [x] `get_object(slug)` returns a `Feature` with correct fields from a fixture
      `playfield.json`.
- [x] `world_to_pixel` and `pixel_to_world` round-trip within 0.01 tolerance for
      a known H matrix and origin. The y-flip and origin offset are asserted
      explicitly (not just the round-trip).
- [x] `add_path("track", [(1.0, 2.0), (3.0, 4.0)])` writes a `paths.json` with
      one path entry matching the schema; `clear_paths()` writes `[]`.
- [x] File write is atomic: uses a `.tmp` file and `os.replace`.
- [x] `uv run --with pytest python -m pytest host/tests/test_playfield.py -q` passes.

## Implementation Plan

### Approach

Create `host/robot_radio/field/` from scratch. No imports from `robot/nezha.py`
or any other `robot/` module — this subpackage is fully independent.

For `playfield.json` loading: open the file path from `Config.load()` (or accept
it as an optional constructor arg for testability). Parse categories `rectangles`,
`dots`, `april_tags`, `aruco_tags`.

For `world_to_pixel`/`pixel_to_world`: import `numpy` (already a transitive dep
via `aprilcam`). Keep `H_inv = np.linalg.inv(H)` cached. Update cache on each
`get_tags()` call that returns a non-None `homography`.

### Files to Create

- `host/robot_radio/field/__init__.py`
- `host/robot_radio/field/playfield.py`
- `host/tests/test_playfield.py`

### Files to Modify

- `host/robot_radio/__init__.py` — add `from robot_radio.field import Playfield,
  Tag, Feature` to the package-level exports.

### Testing Plan

New file `host/tests/test_playfield.py`. Mock `DaemonControl` — no live daemon.

1. `test_get_tag_found` — mock `get_tags()` returning a `TagRecord` with
   `id=100`, `world_xy=(12.5, -3.0)`, `orientation_yaw=0.785`; assert
   `field.get_tag(100)` returns `Tag(100, 12.5, -3.0, 0.785)`.
2. `test_get_tag_absent` — mock `get_tags()` with no matching id; assert `None`.
3. `test_get_tag_no_world_xy` — mock tag with `world_xy=None`; assert `None`.
4. `test_get_object_from_fixture` — write a fixture `playfield.json` with one
   rectangle entry; assert `get_object("rect_slug")` returns correct `Feature`.
5. `test_world_to_pixel_round_trip` — use a known 3×3 H and `origin_x=50,
   origin_y=40`; assert `pixel_to_world(*world_to_pixel(10.0, 5.0)) == (10.0, 5.0)`
   within 1e-6. Also assert that `world_to_pixel` uses `raw = [x+origin_x,
   origin_y-y, 1]` (not `[x, y, 1]`).
6. `test_add_path_writes_paths_json` — call `add_path("cam", [(1.0,2.0)])` with a
   temp dir for paths.json; assert file exists with correct schema.
7. `test_clear_paths_writes_empty_list` — after `add_path`, call `clear_paths()`;
   assert file contains `[]`.
8. `test_add_path_atomic` — assert a `.tmp` file is created and renamed (mock
   `os.replace` and verify it was called).

Verification: `uv run --with pytest python -m pytest host/tests/test_playfield.py -q`

### Documentation Updates

- Add module docstring to `playfield.py` explaining the coordinate convention
  (A1-centred, y-up, world frame cm) and the `display.py:411-447` reference.
- Add docstring to `world_to_pixel` explicitly stating the transform steps.
