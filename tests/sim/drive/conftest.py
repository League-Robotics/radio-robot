"""tests/sim/drive/conftest.py -- tier-0 DRIVE domain fixtures (ticket
100-006). Mirrors tests/sim/conftest.py's own SIM-domain fixtures
(``build_lib``/``sim``) for the new tier-0 Drive:: instrument:

  build_drive_lib -- session-scoped: builds libdrive_host once per session
                     (``just build-drive``, tests/_infra/drive/
                     CMakeLists.txt). NOT autouse -- only tests that
                     actually construct a ``Drive`` need it.

sys.path is extended so ``from drive import ...`` / ``from plant import
...`` / ``from replay import ...`` resolve from any test under
tests/sim/drive/ (mirrors tests/sim/conftest.py's own precedent for
tests/_infra/sim/).
"""
from __future__ import annotations

import pathlib
import subprocess
import sys

import pytest

_TESTS_SIM_DRIVE_DIR = pathlib.Path(__file__).resolve().parent  # tests/sim/drive/
_TESTS_DIR = _TESTS_SIM_DRIVE_DIR.parent.parent  # tests/
_REPO_ROOT = _TESTS_DIR.parent  # repo root
_DRIVE_INFRA_DIR = _TESTS_DIR / "_infra" / "drive"  # tests/_infra/drive/

if str(_DRIVE_INFRA_DIR) not in sys.path:
    sys.path.insert(0, str(_DRIVE_INFRA_DIR))
if str(_TESTS_SIM_DRIVE_DIR) not in sys.path:
    # `from _common import make_limits` -- pytest's own rootdir-relative
    # import machinery does not add a test file's own directory to sys.path
    # when there is no __init__.py (mirrors tests/sim/unit/'s own
    # package-less layout), so this mirrors conftest.py's sys.path.insert
    # precedent above for this directory's own shared helper module.
    sys.path.insert(0, str(_TESTS_SIM_DRIVE_DIR))


@pytest.fixture(scope="session")
def build_drive_lib() -> None:
    """Build libdrive_host once per test session (`just build-drive`)."""
    subprocess.run(["just", "build-drive"], cwd=_REPO_ROOT, check=True)
