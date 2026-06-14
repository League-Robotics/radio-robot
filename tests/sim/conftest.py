"""
tests/sim/conftest.py — sim-subtree conftest (ticket 037-004).

Fixtures build_lib / sim / sim_field_profile are defined in tests/conftest.py
(the tree-root conftest) so they are available to tests/unit/ AND tests/sim/.
This file documents the sim infrastructure paths:

  SIM_DIR  = tests/sim/          (CMakeLists.txt, sim_api.cpp, firmware.py)
  BUILD_DIR = tests/sim/build/   (libfirmware_host.{dylib,so})

No fixtures are re-defined here; all are inherited from the parent conftest.
"""
