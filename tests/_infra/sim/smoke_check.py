"""smoke_check.py — minimal ctypes proof that libfirmware_host loads and its
C ABI responds (ticket 081-004).

Not pytest-collected (tests/_infra/ is outside pyproject.toml's testpaths) --
ticket 081-005 owns the real pytest fixtures and the Sim wrapper class
(tests/_infra/sim/firmware.py). This script is the minimal, ahead-of-005
proof: build the library (`just build-sim`), then run this directly.

    just build-sim
    python3 tests/_infra/sim/smoke_check.py

Exercises exactly the sequence this ticket's acceptance criteria describes:
sim_create -> sim_tick -> sim_command("PING") -> assert a sane reply ->
sim_destroy.
"""

from __future__ import annotations

import ctypes
import pathlib
import sys

_HERE = pathlib.Path(__file__).resolve().parent
_LIB_NAME = "libfirmware_host.dylib" if sys.platform == "darwin" else "libfirmware_host.so"
_LIB_PATH = _HERE / "build" / _LIB_NAME


def main() -> int:
    if not _LIB_PATH.exists():
        print(f"smoke_check: library not found at {_LIB_PATH}")
        print("Build it first:  just build-sim")
        return 1

    lib = ctypes.CDLL(str(_LIB_PATH))

    lib.sim_create.argtypes = []
    lib.sim_create.restype = ctypes.c_void_p

    lib.sim_destroy.argtypes = [ctypes.c_void_p]
    lib.sim_destroy.restype = None

    lib.sim_tick.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    lib.sim_tick.restype = None

    lib.sim_command.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int]
    lib.sim_command.restype = ctypes.c_int

    h = lib.sim_create()
    if not h:
        print("smoke_check: sim_create() returned NULL")
        return 1

    try:
        # Advance the sim once before issuing a command, exactly like a real
        # caller's first pass (tick, then command) -- not strictly required
        # (sim_command works fine before any sim_tick(), replaying at
        # lastTickNow_'s default 0), but this exercises both entry points.
        lib.sim_tick(h, ctypes.c_uint32(0))

        buf = ctypes.create_string_buffer(2048)
        n = lib.sim_command(h, b"PING", buf, 2048)
        reply = buf.raw[:n].decode(errors="replace") if n > 0 else ""

        print(f"sim_command(PING) -> {reply!r}")

        if "pong" not in reply:
            print("smoke_check: FAILED -- expected 'pong' in PING reply")
            return 1

        if not reply.startswith("OK"):
            print("smoke_check: FAILED -- expected an OK-prefixed reply")
            return 1

        print("smoke_check: PASSED")
        return 0
    finally:
        lib.sim_destroy(h)


if __name__ == "__main__":
    sys.exit(main())
