---
id: "005"
title: "defaultRobotConfig() field-pin canary with golden snapshot"
status: open
use-cases: [SUC-004]
depends-on: ["003"]
github-issue: ""
issue: ""
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# defaultRobotConfig() field-pin canary with golden snapshot

## Description

Add `tests/simulation/unit/test_default_config_pin.py` — a pytest test that loads the
host sim lib, reads every field of `defaultRobotConfig()`, serializes them to a dict, and
compares against a committed golden JSON file `tests/_infra/default_config_golden.json`.
Any field that changes causes the test to fail with a human-readable diff.

This gate catches calibration drift — any change to `data/robots/tovez.json` →
`scripts/gen_default_config.py` → `DefaultConfig.cpp` that was not intentional.

## How `defaultRobotConfig()` is accessible from Python

`DefaultConfig.cpp` is already compiled into `libfirmware_host` (it is in
`source/robot/DefaultConfig.cpp`, which is included via `ROBOT_SOURCES` in CMakeLists.txt).
The function `defaultRobotConfig()` is a C++ function returning `RobotConfig` by value.

Two approaches — programmer chooses the simpler one:

**Option A (preferred): use the existing `Sim` wrapper**
The `Sim` ctypes wrapper already calls firmware functions. Check whether `firmware.py`
or `sim_api.cpp` exposes a way to read config fields (e.g., the `GET` command with
the `sim` fixture: `s.send_command("GET <key>")` returns a value). If all `RobotConfig`
fields are exposed via GET commands registered in `ConfigRegistry`, use the `sim` fixture
to issue GET commands for every known key and build the snapshot dict from the responses.

**Option B: add a ctypes export to sim_api.cpp**
Add a thin C export function to `tests/_infra/sim/sim_api.cpp`:
```cpp
extern "C" RobotConfig sim_get_default_config() {
    return defaultRobotConfig();
}
```
Then define the `RobotConfig` ctypes struct in `firmware.py` and call it directly.

Option A is preferred if it can reach all fields without adding new sim_api.cpp exports.
Option B is a fallback if GET commands don't cover all fields.

## Golden snapshot format (`tests/_infra/default_config_golden.json`)

A flat JSON object with field names as keys and values as numbers (float or int):
```json
{
  "wheelDiamMm": 50.0,
  "trackWidthMm": 108.5,
  "fwdSignL": 1,
  "fwdSignR": -1,
  ...
}
```

All `RobotConfig` fields that are meaningful for calibration should appear.
Float fields: use enough decimal places to detect any change (e.g., 6 decimal places).

## Test structure (`test_default_config_pin.py`)

```python
# Pseudocode — programmer implements:
import json, pathlib, pytest

GOLDEN = pathlib.Path(__file__).parents[3] / "tests" / "_infra" / "default_config_golden.json"

def read_config_via_sim(sim):
    """Read all config fields from the sim via GET commands."""
    config = {}
    for key in KNOWN_CONFIG_KEYS:  # enumerate from ConfigRegistry or RobotConfig struct
        resp = sim.send_command(f"GET {key}")
        # parse OK GET <key> <value> response
        config[key] = parse_value(resp)
    return config

def test_default_robot_config_unchanged(sim):
    golden = json.loads(GOLDEN.read_text())
    actual = read_config_via_sim(sim)
    diffs = {k: (golden.get(k), actual.get(k))
             for k in set(golden) | set(actual)
             if golden.get(k) != actual.get(k)}
    assert not diffs, (
        "defaultRobotConfig() fields differ from golden snapshot:\n"
        + "\n".join(f"  {k}: golden={v[0]!r} actual={v[1]!r}" for k, v in sorted(diffs.items()))
        + "\nIf intentional, regenerate: scripts/gen_default_config.py + update golden."
    )
```

## Golden snapshot generation procedure

1. Run the generation script or a one-off Python snippet to dump all config fields.
2. Save to `tests/_infra/default_config_golden.json`.
3. Run `test_default_config_pin.py` — must pass green immediately.
4. Commit both `test_default_config_pin.py` and `default_config_golden.json` together.

## Files to Create

- `tests/simulation/unit/test_default_config_pin.py`
- `tests/_infra/default_config_golden.json` (generated, then committed)

## Acceptance Criteria

- [ ] `test_default_config_pin.py` exists in `tests/simulation/unit/`.
- [ ] `tests/_infra/default_config_golden.json` is committed with all `RobotConfig` fields.
- [ ] `uv run --with pytest python -m pytest tests/simulation/unit/test_default_config_pin.py -v` passes.
- [ ] The test uses the `sim` fixture (session build, fresh Sim per test — no rebuild).
- [ ] Manually changing a calibration value in `data/robots/tovez.json`, regenerating
      `DefaultConfig.cpp`, rebuilding the lib, and re-running the test causes it to FAIL.
- [ ] The overall simulation suite still passes ≥ 1954 tests with the new canary added.
- [ ] `git diff source/` is empty. If Option B (sim_api.cpp export) is used, the only
      change in `tests/_infra/sim/sim_api.cpp` is the new `sim_get_default_config()` export.

## Testing Plan

```bash
# Full suite including new canary:
uv run --with pytest python -m pytest -q

# Canary alone:
uv run --with pytest python -m pytest tests/simulation/unit/test_default_config_pin.py -v

# Synthetic regression (manual — do not commit):
# Edit data/robots/tovez.json, change one value, run gen_default_config.py, rebuild
# python3 build.py && uv run --with pytest python -m pytest tests/simulation/unit/test_default_config_pin.py -v
# Must FAIL; then revert the change.
```
