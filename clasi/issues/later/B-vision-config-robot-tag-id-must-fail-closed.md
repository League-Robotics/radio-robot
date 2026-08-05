---
status: pending
priority: medium
---

# VisionConfig.robot_tag_id must fail closed, not default to the field-origin tag

**Source:** code review 2026-07-30, `03-host-core.md` MAJOR §9.
**Priority:** P1 — small fix, nasty failure mode: a misconfigured robot
tracks the *always-visible, stationary* origin tag as itself, so everything
looks confident and healthy while being wrong.
**Goal served:** fail-closed config is how mis-setup becomes a loud boot
error instead of a subtle "why does the robot think it never moves" hunt.

## What is wrong

`config/robot_config.py:77`: `robot_tag_id: int = 1`. Tag 1 is the fixed
field-origin AprilTag. Every real robot config already overrides it (tovez:
100, togov: 101), and three call sites (`io/cli.py:164,640,738`,
`io/calibrate.py:314,648`) already hardcode a fallback of 100 — the codebase
has learned 1 is wrong everywhere except the default itself. A config that
omits the field silently tracks the origin: `Odometry.is_valid` stays True,
pose is confident, static, and wrong.

## What to do

```python
# config/robot_config.py
robot_tag_id: Optional[int] = None
"""AprilTag id registered as this robot's centre of rotation. None =
unconfigured -- FAIL CLOSED: consumers must raise, never guess. Tag 1 is
the field origin and is never a robot."""
```

Add one accessor that every consumer goes through, so the failure is loud
and singular:

```python
def require_robot_tag_id(cfg: VisionConfig) -> int:
    if cfg.robot_tag_id is None:
        raise ConfigError(
            "vision.robot_tag_id is not configured -- refusing to guess. "
            "Set it in the robot's data/robots/*.json (tovez=100, togov=101).")
    if cfg.robot_tag_id == 1:
        raise ConfigError("robot_tag_id=1 is the FIELD ORIGIN tag, not a robot.")
    return cfg.robot_tag_id
```

Then delete the five hardcoded `100` fallbacks in `cli.py`/`calibrate.py` —
they exist only because the default couldn't be trusted; with a fail-closed
config they are a second source of truth to rot.

## Acceptance

- `VisionConfig()` with no tag id → any vision-consuming path raises
  `ConfigError` before touching the camera; unit test asserts it, plus the
  `== 1` rejection.
- `grep -rn "robot_tag_id.*100\|tag_id=100\|, 100)" src/host/robot_radio/io/`
  shows no hardcoded fallback.
- `rogo goto`-family and calibrate commands still resolve tag 100 on tovez
  via config alone (playfield check).
