"""test_default_config_pin.py — defaultRobotConfig() field-pin canary (038-005).

Reads all RobotConfig fields from a freshly-created Sim (no fixture
overrides) via GET commands and compares against a committed golden JSON
snapshot. Any field change causes this test to fail with a human-readable
diff.

This gate catches calibration drift: any change to data/robots/tovez.json
→ scripts/gen_default_config.py → DefaultConfig.cpp that was not intentional.

NOTE: A raw Sim() is used (not the ``sim`` fixture) so that the ``sim``
fixture's ``SET sTimeout=60000`` watchdog override does NOT mask changes to
the DefaultConfig sTimeout field.  build_lib is requested via the fixture
parameter to ensure the shared lib is built before this test runs.

To regenerate the golden snapshot after an intentional change:
    python3 -c "
    import sys, json
    sys.path.insert(0, 'tests/_infra/sim')
    from firmware import Sim
    s = Sim()
    resp = s.send_command('GET')
    config = {}
    for line in resp.splitlines():
        if not line.startswith('CFG'): continue
        for kv in line[3:].strip().split():
            if '=' not in kv: continue
            k, v = kv.split('=', 1)
            try: config[k] = int(v)
            except ValueError:
                try: config[k] = float(v)
                except ValueError: config[k] = v
    print(json.dumps(config, indent=2, sort_keys=True))
    " > tests/_infra/default_config_golden.json
"""
from __future__ import annotations

import json
import pathlib

import pytest

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

# __file__ is tests/simulation/unit/test_default_config_pin.py
# parents[0] = tests/simulation/unit/
# parents[1] = tests/simulation/
# parents[2] = tests/
# parents[3] = repo root
GOLDEN = pathlib.Path(__file__).parents[3] / "tests" / "_infra" / "default_config_golden.json"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _read_raw_config() -> dict:
    """Create a fresh Sim and read all RobotConfig fields via GET.

    A raw Sim() is used (not the ``sim`` fixture) so no ``SET`` overrides
    are applied before the GET — the values returned are exactly
    defaultRobotConfig() as compiled into the library.

    Integer-valued keys (int32 or CFG_FLOAT_AS_INT displayed as int) are
    stored as int; float-valued keys are stored as float.
    """
    from firmware import Sim  # noqa: PLC0415

    with Sim() as s:
        resp = s.send_command("GET")

    config: dict = {}
    for line in resp.splitlines():
        if not line.startswith("CFG"):
            continue
        # Strip "CFG" prefix and optional correlation-id suffix
        content = line[3:].strip()
        # Remove trailing #<corrId> if present
        if " #" in content:
            content = content[: content.rfind(" #")]
        for kv in content.split():
            if "=" not in kv:
                continue
            k, v = kv.split("=", 1)
            # Type-coerce: try int first, then float, else keep as str.
            try:
                config[k] = int(v)
            except ValueError:
                try:
                    config[k] = float(v)
                except ValueError:
                    config[k] = v
    return config


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_default_robot_config_unchanged(build_lib):
    """All defaultRobotConfig() fields match the committed golden snapshot.

    Uses a raw Sim() (no fixture overrides) so that the fixture's watchdog
    extension (SET sTimeout=60000) does not mask changes to the default
    sTimeout field.  build_lib is requested to ensure the shared library is
    built before this test runs.
    """
    assert GOLDEN.exists(), (
        f"Golden snapshot missing: {GOLDEN}\n"
        "Regenerate with: see docstring at top of this file."
    )
    golden = json.loads(GOLDEN.read_text())
    actual = _read_raw_config()

    # Check for field-level mismatches (including missing / extra keys).
    all_keys = sorted(set(golden) | set(actual))
    diffs = {}
    for k in all_keys:
        g_val = golden.get(k)
        a_val = actual.get(k)
        if g_val != a_val:
            diffs[k] = (g_val, a_val)

    assert not diffs, (
        "defaultRobotConfig() fields differ from golden snapshot:\n"
        + "\n".join(
            f"  {k}: golden={v[0]!r}  actual={v[1]!r}"
            for k, v in sorted(diffs.items())
        )
        + "\nIf intentional, regenerate: update data/robots/tovez.json, run "
        "scripts/gen_default_config.py, rebuild, then update "
        "tests/_infra/default_config_golden.json."
    )
