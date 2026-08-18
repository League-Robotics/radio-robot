"""Isolation grep test for the platform / hardware / hal layering.

Successor to ``test_devices_isolation.py``, which guarded the standing
isolation invariant over the single ``src/firm/devices/`` tree
(device-bus-tickets.md, "Standing isolation invariant"; mirrors sprint
100's ``src/firm/drive/`` discipline). The platform/hardware/hal
reorganization split that one tree into three layers, so the one rule
becomes three -- same shape, same grep, one allowed-prefix set per layer:

    platform/  may include: platform/, hal/     (+ vendor, + stdlib)
    hal/       may include: hal/                          (+ stdlib)
    hardware/  may include: hardware/, hal/, platform/  (+ vendor, + stdlib)

Dependencies run strictly downward: `hal/` names no bus and no clock, and
`hardware/` reaches DOWN to `platform/` and UP only as far as the `hal/`
interface it implements. `platform/` reaches UP only as far as the `hal/`
interfaces its per-target implementations bind to (136-004 moved
`Hal::Clock`/`Hal::Sleeper`/`Hal::I2CBus` into `hal/`, so
`Platform::MicroBitClock`/`MicroBitSleeper`/`MicroBitI2CBus` and
`TestSim::SimClock`/`SimSleeper`/`SimPlant` now implement HAL interfaces
from outside `hal/` -- the same "implementor reaches up to the interface
it implements" shape `hardware/` already had). None of the three may reach
`messages/`, `config/`, `com/`, `kinematics/`, `control/`, or `core/` --
reaching the wire schema or generated boot config from a driver is what
kills its reuse under ``-DHOST_BUILD``/sim, and reaching upward past `hal/`
inverts the layering outright.

``src/firm/motion/`` -- the planner/navigator/odometry tree this list used
to also name -- was deleted outright by the DifferentialDrive-kernel
exploratory rewrite (2026-08-15, differentialdrive-one-class-one-fiber-
exploratory-worktree.md): motion queueing folded into
``Control::DifferentialDrive`` (the wheel kernel, ``src/firm/control/``)
and the application layer. It was never one of this test's own checked
``_LAYERS`` entries (only named in this prose as a forbidden reach target
for hal/platform/hardware) -- there is nothing left to update in the
grep itself, only this comment.

``platform/host/`` is excluded: it is the host PLATFORM (the sim), whose
whole job is to compose the firmware it substitutes primitives for, so it
legitimately includes ``app/`` and ``hardware/``.

This is a pure-Python grep test, not a compile-time check: it scans every
``#include`` line under each layer (recursively, so files nested in family
subdirectories are covered) and fails loudly -- naming the offending file
and line -- if a *quoted* (``#include "..."``) include path is not under one
of that layer's allowed prefixes, unless the bare included filename is on
the small vendor whitelist below (CODAL/micro:bit headers this project does
not control and cannot rename -- they are included bare, e.g.
``#include "MicroBit.h"``). Angle-bracket (``#include <...>``) includes are
always allowed -- they are the standard-library form every allowed libc/libm
header (``<cstdint>``, ``<math.h>``, ``<cstdio>``, ``<cstring>``, ...) uses.
"""

import pathlib
import re
import sys

import pytest

# src/tests/sim/unit/test_layer_isolation.py -> unit -> sim -> tests -> repo root
_REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
_FIRM_DIR = _REPO_ROOT / "src" / "firm"

# One entry per layer: (directory, allowed quoted-include prefixes).
_LAYERS = (
    ("hal", ("hal/",)),
    ("platform", ("platform/", "hal/")),
    ("hardware", ("hardware/", "hal/", "platform/")),
)

# platform/host is the host PLATFORM (the sim) -- a composition root, not a
# primitive. It includes app/ and hardware/ by design.
_EXCLUDED_SUBTREES = ("platform/host/",)

# #include "..."  or  #include <...>  (leading whitespace tolerated; the
# preprocessor allows arbitrary whitespace between '#' and 'include').
_INCLUDE_RE = re.compile(r'^\s*#\s*include\s*(<[^>]+>|"[^"]+")')

# Bare vendor filenames sanctioned as a quoted include even though they
# carry no layer prefix -- CODAL/micro:bit vendor headers, not project
# paths. Kept small and explicit (not a wildcard) so a real project-path
# leak can never hide behind it.
_VENDOR_WHITELIST = frozenset({
    "MicroBit.h",
    "MicroBitConfig.h",
    "codal_target_hal.h",
})


def _layer_source_files(layer):
    layer_dir = _FIRM_DIR / layer
    assert layer_dir.is_dir(), f"src/firm/{layer}/ missing: {layer_dir}"
    files = []
    for path in sorted(layer_dir.rglob("*")):
        if not path.is_file() or path.suffix not in (".h", ".cpp"):
            continue
        rel = path.relative_to(_FIRM_DIR).as_posix()
        if any(rel.startswith(skip) for skip in _EXCLUDED_SUBTREES):
            continue
        files.append(path)
    assert files, f"no .h/.cpp files found under {layer_dir}"
    return files


def _find_violations(layer, allowed_prefixes, files):
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

            if included_path.startswith(allowed_prefixes):
                continue
            if included_path in _VENDOR_WHITELIST:
                continue
            if "/" not in included_path:
                # A bare, non-whitelisted quoted filename -- e.g. a
                # same-directory sibling included without its layer prefix.
                # Not itself a cross-layer leak (there is no "/" to indicate
                # one), but also not the documented "<layer>/xxx.h"
                # self-include convention these files use -- flag it rather
                # than silently allow it, since it cannot be told apart from
                # an accidental non-project vendor header by this rule alone.
                violations.append(
                    f"{rel}:{lineno}: bare quoted include {token} is neither "
                    f"prefixed with one of {allowed_prefixes} nor on the "
                    "vendor whitelist"
                )
                continue

            violations.append(
                f"{rel}:{lineno}: {token} is not one of {allowed_prefixes} "
                f"-- src/firm/{layer}/ isolation invariant violation"
            )
    return violations


@pytest.mark.parametrize("layer,allowed_prefixes", _LAYERS, ids=[e[0] for e in _LAYERS])
def test_layer_isolation_no_foreign_includes(layer, allowed_prefixes):
    """No src/firm/<layer>/*.{h,cpp} file may #include outside its allowed prefixes."""
    files = _layer_source_files(layer)
    violations = _find_violations(layer, allowed_prefixes, files)
    assert not violations, (
        f"src/firm/{layer}/ isolation invariant violated -- see "
        "src/firm/hardware/DESIGN.md §3 and src/firm/hal/DESIGN.md §3:\n"
        + "\n".join(violations)
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
