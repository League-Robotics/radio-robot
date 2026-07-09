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
  sim       — function-scoped: a fresh ``Sim()`` per test. No longer widens
              a watchdog on setup (093-003) -- ticket 093 removed the
              serial-silence watchdog, ``estop()``, and the ``DEV`` command
              family entirely from ``Rt::CommandRouter::buildTable()``
              (architecture-update.md Decision 2), so there is nothing left
              to widen; see the fixture's own docstring below.

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

@pytest.fixture(scope="session")
def build_lib() -> None:
    """Build libfirmware_host once per test session (`just build-sim`)."""
    subprocess.run(["just", "build-sim"], cwd=_REPO_ROOT, check=True)


@pytest.fixture
def sim(build_lib: None):
    """Fresh Sim instance per test; destroyed in a finally so a failing
    test still frees its SimHandle.

    093-003: this used to widen the serial-silence watchdog to the
    firmware's own maximum (``DEV WD 60000``) immediately after
    ``sim_create()``, because a long ``tick_for()`` would otherwise trip the
    1 s default window and neutralize every motor mid-test. Ticket 093
    (architecture-update.md Decision 2) removed that watchdog -- and the
    entire ``DEV`` command family -- from ``Rt::CommandRouter::
    buildTable()``, so the widen call became a silent ``ERR unknown`` at the
    top of every single test using this fixture. Removed rather than left in
    place: there is no watchdog left to widen, and the bench-posture
    (stand-mounted, wheels-off-the-ground) justification for its removal is
    documented at ``.claude/rules/hardware-bench-testing.md``.
    """
    from firmware import Sim  # noqa: PLC0415 -- import after build_lib runs

    s = Sim()
    try:
        yield s
    finally:
        s.close()
