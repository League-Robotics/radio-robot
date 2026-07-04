"""Placeholder collectible test for the tests/sim/unit/ skeleton (ticket 077-006).

``tests/sim/`` has no real content yet — the new-tree simulator harness is
later-ticket work (see tests/sim/conftest.py and the sprint-077 issue). This
placeholder exists solely so ``uv run python -m pytest`` (which collects
``tests/sim/`` per ``pyproject.toml``'s ``testpaths``) has at least one
passing test to run, proving collection of the new tree is clean, rather than
reporting "0 tests collected" (which pytest treats as a passing but easily
overlooked no-op). Delete this once ``tests/sim/unit/`` gains real tests.
"""


def test_sim_domain_collects():
    """Trivial tautology — exists only to keep collection non-empty."""
    assert True
