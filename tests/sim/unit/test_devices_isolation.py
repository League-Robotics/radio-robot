"""Isolation grep test for ticket DB-001 (device-bus-tickets.md).

Enforces the standing isolation invariant (device-bus-tickets.md, "Standing
isolation invariant" -- mirrors sprint 100's ``source/drive/`` discipline,
see clasi/sprints/100-.../tickets/002-.../ for the precedent):

    source/devices/ may #include ONLY: its own headers, the C/C++ standard
    library, and CODAL/micro:bit (MicroBit.h and friends). NO include may
    reach messages/, hal/, com/, subsystems/, config/, or any other project
    path.

This is a pure-Python grep test, not a compile-time check: it scans every
``#include`` line under ``source/devices/*.{h,cpp}`` (recursively, so later
tickets are covered even if they nest files under a subdirectory) and fails
loudly -- naming the offending file and line -- if a *quoted*
(``#include "..."``) include path contains a ``/`` and does not start with
``devices/``, unless the bare included filename is on the small vendor
whitelist below (CODAL/micro:bit headers this project does not control and
cannot rename -- they are included bare, e.g. ``#include "MicroBit.h"``, so
in practice they never trip the ``/``-containment check at all; the
whitelist exists as an explicit, auditable belt-and-suspenders record of
which non-``devices/`` quoted includes are sanctioned, not a loophole).
Angle-bracket (``#include <...>``) includes are always allowed -- they are
the standard-library form every allowed libc/libm header
(``<cstdint>``, ``<math.h>``, ``<cstdio>``, ``<cstring>``, ...) uses.

Landed in DB-001 (device_types.h/device_config.h, which include nothing but
``<cstdint>``) specifically so it stands guard over every later ticket
(DB-002 onward) that adds real ``.cpp``/``.h`` files under ``source/
devices/``.
"""

import pathlib
import re
import sys

import pytest

# tests/sim/unit/test_devices_isolation.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
_DEVICES_DIR = _REPO_ROOT / "source" / "devices"

# #include "..."  or  #include <...>  (leading whitespace tolerated; the
# preprocessor allows arbitrary whitespace between '#' and 'include').
_INCLUDE_RE = re.compile(r'^\s*#\s*include\s*(<[^>]+>|"[^"]+")')

# Bare vendor filenames sanctioned as a quoted include even though they
# carry no "devices/" prefix -- CODAL/micro:bit vendor headers, not project
# paths. Kept small and explicit (not a wildcard) so a real project-path
# leak (e.g. "hal/velocity_pid.h") can never hide behind it.
_VENDOR_WHITELIST = frozenset({
    "MicroBit.h",
    "MicroBitConfig.h",
    "codal_target_hal.h",
})


def _devices_source_files():
    assert _DEVICES_DIR.is_dir(), f"source/devices/ missing: {_DEVICES_DIR}"
    files = sorted(
        p for p in _DEVICES_DIR.rglob("*")
        if p.is_file() and p.suffix in (".h", ".cpp")
    )
    assert files, f"no .h/.cpp files found under {_DEVICES_DIR}"
    return files


def _find_violations(files):
    """Return a list of human-readable "path:line: include" violation strings."""
    violations = []
    for path in files:
        rel = path.relative_to(_REPO_ROOT)
        for lineno, line in enumerate(
            path.read_text(encoding="utf-8").splitlines(), start=1
        ):
            match = _INCLUDE_RE.match(line)
            if not match:
                continue
            token = match.group(1)

            if token.startswith("<"):
                # Angle-bracket includes are always the standard-library
                # form -- always allowed.
                continue

            # Quoted include: strip the surrounding double quotes.
            included_path = token[1:-1]

            if included_path.startswith("devices/"):
                continue
            if included_path in _VENDOR_WHITELIST:
                continue
            if "/" not in included_path:
                # A bare, non-whitelisted quoted filename -- e.g. a
                # same-directory sibling included without its "devices/"
                # prefix. Not itself a cross-subsystem leak (there is no
                # "/" to indicate one), but also not the documented
                # "devices/xxx.h" self-include convention this subsystem's
                # own files use -- flag it rather than silently allow it,
                # since it cannot be told apart from an accidental
                # non-project vendor header by this rule alone.
                violations.append(
                    f"{rel}:{lineno}: bare quoted include {token} is "
                    "neither devices/-prefixed nor on the vendor whitelist"
                )
                continue

            violations.append(
                f"{rel}:{lineno}: {token} reaches outside source/devices/ "
                "(isolation invariant violation)"
            )
    return violations


def test_devices_isolation_no_foreign_includes():
    """No source/devices/*.{h,cpp} file may #include outside devices/ (isolation invariant)."""
    files = _devices_source_files()
    violations = _find_violations(files)
    assert not violations, (
        "source/devices/ isolation invariant violated -- see "
        "device-bus-tickets.md's \"Standing isolation invariant\":\n"
        + "\n".join(violations)
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
