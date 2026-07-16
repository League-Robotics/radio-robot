"""src/tests/sim/conftest.py — SIM domain fixtures.

105-006: this file used to carry `build_lib`/`sim` fixtures (081-005)
wired to `src/sim/sim_api.cpp` (a compiled `libfirmware_host` C
ABI) and `src/sim/firmware.py`'s ctypes `Sim` wrapper, built via
a `just build-sim` recipe. All three -- the compiled lib, the Python
wrapper, and the justfile recipe -- were deleted before this ticket
(confirmed by reading both `tests/_infra/` and the `justfile` directly),
so any test depending on `sim`/`build_lib` failed immediately on
collection-adjacent use; `git grep` confirmed zero live callers anywhere
under `src/tests/sim/` before this file was cut down.

Ticket-time call (105-006, architecture-update.md Step 7 Open Question 2:
"a shared `sim_api`-backed fixture is worth adding" — a ticket-time call
given the established ad hoc per-file compile convention). DEFAULT bias
per the ticket's own plan: delete rather than invent a replacement,
unless writing 2+ new scenario files without one proves genuinely
repetitive. It didn't -- this ticket added exactly one new scenario file
(`system/test_scripted_twist_demo.py`), and it follows the SAME
already-established pattern every sibling file in this tier already uses
(`plant/test_plant.py`, `system/test_sim_api.py`,
`system/faults/test_fault_knobs.py`): a pytest file compiles its own
throwaway C++ harness binary + the shared `src/tests/sim/support/sim_api.{h,
cpp}`/plant sources via `subprocess` into a per-test `tmp_path`, runs it,
and asserts exit 0. `TestSim::SimApi` is a C++ class linked directly into
each harness binary -- there is no ctypes/dlopen boundary left for a
Python fixture to wrap, so a `sim` fixture would have nothing to return.

No fixtures live here. The file is kept (rather than deleted outright) as
the documented landing spot for this decision, and as the natural place a
future *Python-level* SIM domain fixture would go if one is ever actually
needed.
"""
from __future__ import annotations
