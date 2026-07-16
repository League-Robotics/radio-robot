"""tests/conftest.py — root pytest configuration for the new three-domain tree.

Sprint 077 rebuilt `tests/` from scratch (`tests_old/` is the parked
pre-rebuild tree; see `tests/CLAUDE.md`). This root conftest only sets up
``sys.path`` so any domain can ``import robot_radio`` — no fixtures are
defined at this scope. Domain-specific fixtures live in each domain's own
conftest.py (e.g. ``src/tests/sim/conftest.py``); ``src/tests/bench/`` and
``src/tests/playfield/`` are HITL CLI tools, not pytest-collected, and have no
conftest of their own.
"""

import pathlib
import sys

_TESTS_DIR = pathlib.Path(__file__).parent    # tests/
_REPO_ROOT = _TESTS_DIR.parent                # repo root
_HOST_DIR = _REPO_ROOT / "src" / "host"               # host/ (robot_radio package)

if str(_HOST_DIR) not in sys.path:
    sys.path.insert(0, str(_HOST_DIR))
