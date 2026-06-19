"""
tests/conftest.py — root pytest fixtures for the merged test tree (ticket 037-004).

Provides:
  build_lib        — session-scoped, autouse: cmake build of libfirmware_host.
  sim              — function-scoped: one fresh Sim per test.
  sim_field_profile — function-scoped: Sim with field profile (turn slip + OTOS fusion).

sys.path setup ensures:
  - tests/_infra/sim/ is on sys.path → `from firmware import Sim` works from tests/unit/.
  - host/ is on sys.path → `from robot_radio.testkit import ...` works everywhere.
"""
import pathlib
import subprocess
import sys

import pytest

# ---------------------------------------------------------------------------
# Path constants
# ---------------------------------------------------------------------------

_TESTS_DIR = pathlib.Path(__file__).parent          # tests/
_SIM_DIR   = _TESTS_DIR / "_infra" / "sim"         # tests/_infra/sim/
_BUILD_DIR = _SIM_DIR / "build"                    # tests/_infra/sim/build/
_REPO_ROOT = _TESTS_DIR.parent                     # repo root
_HOST_DIR  = _REPO_ROOT / "host"                   # host/ (robot_radio package)

# ---------------------------------------------------------------------------
# sys.path setup
# ---------------------------------------------------------------------------
# Add tests/sim/ so that `from firmware import Sim` works in tests/unit/ tests.
if str(_SIM_DIR) not in sys.path:
    sys.path.insert(0, str(_SIM_DIR))

# Add host/ so that `from robot_radio.testkit import ...` works.
if str(_HOST_DIR) not in sys.path:
    sys.path.insert(0, str(_HOST_DIR))


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(scope="session", autouse=True)
def build_lib():
    """Configure and build libfirmware_host if needed."""
    _BUILD_DIR.mkdir(exist_ok=True)
    subprocess.run(
        ["cmake", "-S", str(_SIM_DIR), "-B", str(_BUILD_DIR)],
        check=True,
    )
    subprocess.run(
        ["cmake", "--build", str(_BUILD_DIR)],
        check=True,
    )


@pytest.fixture
def sim(build_lib):
    """Create a fresh Sim instance for the test; destroy it afterwards.

    The keepalive watchdog (sTimeoutMs, default 500 ms) is extended to 60 s
    so that tests which don't re-send keepalives don't hit the safety-stop.
    Tests that explicitly want to test watchdog behaviour should override via
    ``sim.send_command("SET sTimeout=<value>")``.
    """
    # Import here so the module is only loaded after build_lib runs.
    from firmware import Sim  # noqa: PLC0415

    with Sim() as s:
        # Extend watchdog timeout so single-shot S/VW commands don't trigger
        # the safety-stop during tick_for loops that don't send keepalives.
        s.send_command("SET sTimeout=60000")
        yield s


@pytest.fixture
def sim_field_profile(build_lib):
    """Create a Sim pre-configured with the field profile (turn slip + OTOS fusion).

    Mirrors the ``sim`` fixture but additionally applies the field profile:
    turn-slip over-report (slipTurnExtra = 0.26) and OTOS EKF fusion enabled.
    Use this fixture for tests that must pass under field conditions.

    The watchdog is extended to 60 s identically to the ``sim`` fixture.
    """
    from firmware import Sim  # noqa: PLC0415

    with Sim() as s:
        s.send_command("SET sTimeout=60000")
        s.set_field_profile(slip_turn_extra=0.26, fuse_otos=True)
        yield s
