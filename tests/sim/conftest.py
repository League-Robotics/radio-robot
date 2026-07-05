"""tests/sim/conftest.py — SIM domain fixtures (ticket 081-005).

Replaces the 077-006 placeholder (no fixtures, no harness) now that the
new-tree simulator exists: ticket 004's compiled ``libfirmware_host`` C ABI
(``tests/_infra/sim/sim_api.cpp``) plus this ticket's Python wrapper
(``tests/_infra/sim/firmware.py``'s ``Sim`` class).

Provides:
  build_lib — session-scoped: builds libfirmware_host once per session
              (``just build-sim``). NOT autouse — only tests that actually
              need the compiled sim library depend on it (directly, or
              transitively via the ``sim`` fixture below); the existing
              ``tests/sim/unit/*_harness.cpp``-backed tests compile their
              own throwaway binary ad hoc and never touch this fixture.
  sim       — function-scoped: a fresh ``Sim()`` per test, watchdog widened
              immediately so a long ``tick_for()`` isn't neutralized
              mid-test by the 1 s default ``SerialSilenceWatchdog`` window.

sys.path setup ensures ``from firmware import Sim`` resolves from any test
under ``tests/sim/`` (mirrors tests_old/conftest.py's precedent).
"""
from __future__ import annotations

import pathlib
import subprocess
import sys

import pytest

# ---------------------------------------------------------------------------
# Path constants
# ---------------------------------------------------------------------------
_TESTS_SIM_DIR = pathlib.Path(__file__).resolve().parent      # tests/sim/
_TESTS_DIR = _TESTS_SIM_DIR.parent                             # tests/
_REPO_ROOT = _TESTS_DIR.parent                                 # repo root
_SIM_INFRA_DIR = _TESTS_DIR / "_infra" / "sim"                 # tests/_infra/sim/

if str(_SIM_INFRA_DIR) not in sys.path:
    sys.path.insert(0, str(_SIM_INFRA_DIR))

# The DEV WD ceiling (docs/protocol-v2.md §16, dev_commands.cpp's
# kDevWdArgs ArgDef): [50, 60000] ms. 60 s is the widest window the
# firmware's own range validation accepts -- ample headroom for any single
# test's tick_for() duration, so the serial-silence watchdog never fires
# except in the dedicated watchdog-policy test that deliberately narrows it.
_WATCHDOG_WIDE_WINDOW = 60000   # [ms]


@pytest.fixture(scope="session")
def build_lib() -> None:
    """Build libfirmware_host once per test session (`just build-sim`)."""
    subprocess.run(["just", "build-sim"], cwd=_REPO_ROOT, check=True)


@pytest.fixture
def sim(build_lib: None):
    """Fresh Sim instance per test; destroyed in a finally so a failing
    test still frees its SimHandle.

    Widens the serial-silence watchdog to the firmware's own maximum
    (60 s) immediately after sim_create() -- a long tick_for() would
    otherwise trip the 1 s default window and neutralize every motor
    mid-test. Tests that want to exercise the watchdog itself should
    re-narrow it explicitly (e.g. ``sim.command("DEV WD 100")``).
    """
    from firmware import Sim  # noqa: PLC0415 -- import after build_lib runs

    s = Sim()
    try:
        s.command(f"DEV WD {_WATCHDOG_WIDE_WINDOW}")
        yield s
    finally:
        s.close()
