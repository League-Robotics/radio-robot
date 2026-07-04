"""tests/sim/conftest.py — SIM domain fixtures (skeleton only, ticket 077-006).

The SIM domain merges the old ``tests_old/sim/`` + ``tests_old/simulation/``
pair (see ``tests/CLAUDE.md``). A fresh simulator harness for the new
``source/`` tree — the equivalent of the old ``tests_old/_infra/sim/``
``libfirmware_host`` ctypes wrapper (``from firmware import Sim``) — has not
been built yet; that is explicit later-ticket work (see the sprint-077 issue,
"Later tickets (not this one)"). This conftest is a placeholder: no
``build_lib``/``sim`` fixtures are defined here because there is nothing yet
for them to wrap.

When a later ticket adds the new-tree sim harness, its build/session
fixtures land here, following the same shape the old
``tests_old/conftest.py`` used (``build_lib`` session-scoped autouse fixture
that builds the sim library; a function-scoped ``sim`` fixture that yields a
fresh instance per test).
"""
