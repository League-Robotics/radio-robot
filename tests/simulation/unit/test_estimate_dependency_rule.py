"""test_estimate_dependency_rule.py — PhysicalStateEstimate dependency-rule fence
(Phase C, Sprint 041-003).

Phase C names the estimate seam (`PhysicalStateEstimate`, the estimate dual of
`PhysicsWorld`) and establishes its structural invariant BEFORE Phase F repoints
readers: the `source/state/` estimator layer must stay dependency-clean.

This test walks the ACTUAL `#include` graph reachable from
`source/state/PhysicalStateEstimate.h` (resolving relative includes across the
firmware `source/` tree the same way the build's include dirs do) and asserts:

  1. No forbidden token appears anywhere in that transitive header set:
       - CODAL:                 `MicroBit.h`, `microbit_random`, `CODAL_`
       - command-dispatch:      `CommandTypes.h`, `CommandProcessor.h`,
                                 `class Commandable` / `public Commandable`,
                                 `CommandDescriptor`
       - device handles:        `I2CBus`, `NezhaHAL`
     i.e. the estimator includes neither the command surface nor a device handle.

  2. The reachable header set is a SUBSET of the documented allowed set
     (architecture-update.md §Module Definitions → PhysicalStateEstimate).
     The allowed transitive set is intentionally narrow:
       <stdint.h>, <math.h>           — scalar types / EKF math
       PhysicalStateEstimate.h        — the seam itself
       Odometry.h                     — wrapped estimator (composition)
       EKF.h                          — 5-state CTRV filter
       Inputs.h                       — HardwareState POD (types header;
                                        was control/RobotState.h pre-044-002)
       Config.h                       — RobotConfig POD (types header)
       Protocol.h                     — reply tags + ReplyFn + KVPair (types
                                        header; NOT a command-dispatch surface:
                                        no Commandable / CommandDescriptor)
       MotionEventSink.h              — EVT sink interface (types header)
       IOtosSensor.h / IOdometer.h    — odometry capability interface (the io
       Sensor.h / Pose2D.h              capability seam; pure interface, no device)

Per architecture-update.md, `Protocol.h` is reachable ONLY transitively via
`Inputs.h` (the types header, formerly control/RobotState.h). The rule the seam
enforces is the absence of the
COMMAND-DISPATCH surface (`Commandable`/`CommandTypes.h`/`CommandProcessor.h`) and
of CODAL/device handles — which the forbidden-token check above asserts directly.

If Phase F (or any later change) ever drags the command surface, a CODAL header,
or a device handle back into the estimator's include graph, this fence fails.
"""
from __future__ import annotations

import pathlib
import re

import pytest

REPO_ROOT = pathlib.Path(__file__).parents[3]
SOURCE_DIR = REPO_ROOT / "source"
ROOT_HEADER = SOURCE_DIR / "state" / "PhysicalStateEstimate.h"

# Firmware include dirs that bare `#include "Foo.h"` may resolve against. Mirrors
# the sim CMakeLists target_include_directories list (source/{state,control,
# types,robot,io,io/capability}) plus the directory of each including file.
INCLUDE_DIRS = [
    SOURCE_DIR / "state",
    SOURCE_DIR / "control",
    SOURCE_DIR / "types",
    SOURCE_DIR / "robot",
    SOURCE_DIR / "io",
    SOURCE_DIR / "io" / "capability",
    SOURCE_DIR,
]

# Allowed reachable headers (file basenames). The estimator's transitive include
# graph must be a subset of this set — see module docstring for the rationale.
ALLOWED_HEADERS = {
    "PhysicalStateEstimate.h",
    "Odometry.h",
    "EKF.h",
    "Inputs.h",
    "Config.h",
    "Protocol.h",
    "MotionEventSink.h",
    "IOtosSensor.h",
    "IOdometer.h",
    "Sensor.h",
    "Pose2D.h",
}

# Forbidden tokens — command-dispatch surface, CODAL, and device handles. None of
# these may appear in any header reachable from PhysicalStateEstimate.h.
FORBIDDEN_PATTERNS = [
    # CODAL / micro:bit vendor surface
    (r"#include\s*[<\"].*MicroBit\.h", "CODAL MicroBit.h include"),
    (r"\bmicrobit_random\b", "CODAL microbit_random"),
    # command-dispatch surface
    (r"#include\s*[<\"].*CommandTypes\.h", "CommandTypes.h include"),
    (r"#include\s*[<\"].*CommandProcessor\.h", "CommandProcessor.h include"),
    (r"\bpublic\s+Commandable\b", "inherits Commandable"),
    (r"\bclass\s+Commandable\b", "Commandable class decl"),
    (r"\bCommandDescriptor\b", "CommandDescriptor type"),
    # device handles
    (r"\bI2CBus\b", "I2CBus device handle"),
    (r"\bNezhaHAL\b", "NezhaHAL device handle"),
]

_INCLUDE_RE = re.compile(r'^\s*#\s*include\s*"([^"]+)"')


def _resolve(inc: str, including_file: pathlib.Path) -> pathlib.Path | None:
    """Resolve a quoted include against the including file's dir, then the
    firmware include dirs. Returns the resolved path, or None if unresolved
    (e.g. a header outside the firmware source tree)."""
    candidates = [including_file.parent / inc]
    candidates += [d / inc for d in INCLUDE_DIRS]
    # Also try just the basename against each include dir (handles "../x/Foo.h"
    # forms and shim re-exports).
    base = pathlib.PurePath(inc).name
    candidates += [d / base for d in INCLUDE_DIRS]
    for c in candidates:
        rc = c.resolve()
        if rc.exists() and rc.is_file():
            return rc
    return None


def _reachable_headers(root: pathlib.Path) -> dict[pathlib.Path, str]:
    """BFS the quoted-#include graph from root. Returns {path: text} for every
    firmware header reachable from root (root included). Angle-bracket includes
    (<stdint.h>, <math.h>, ...) are standard-library and not traversed."""
    seen: dict[pathlib.Path, str] = {}
    queue = [root.resolve()]
    while queue:
        f = queue.pop()
        if f in seen:
            continue
        text = f.read_text(errors="replace")
        seen[f] = text
        for line in text.splitlines():
            m = _INCLUDE_RE.match(line)
            if not m:
                continue
            resolved = _resolve(m.group(1), f)
            if resolved is not None and resolved not in seen:
                queue.append(resolved)
    return seen


def test_estimate_root_header_exists():
    assert ROOT_HEADER.exists(), (
        f"{ROOT_HEADER} not found — the PhysicalStateEstimate seam must exist."
    )


def test_estimate_include_graph_has_no_forbidden_tokens():
    """No CODAL, command-dispatch surface, or device handle in the estimator's
    transitive include graph."""
    graph = _reachable_headers(ROOT_HEADER)
    violations: list[str] = []
    for path, text in graph.items():
        rel = path.relative_to(REPO_ROOT)
        for lineno, line in enumerate(text.splitlines(), 1):
            # Skip pure comment lines — the rule is about real includes/decls,
            # not prose that happens to name a forbidden token.
            stripped = line.lstrip()
            if stripped.startswith("//") or stripped.startswith("*"):
                continue
            for pat, why in FORBIDDEN_PATTERNS:
                if re.search(pat, line):
                    violations.append(f"{rel}:{lineno}: {why} — {line.strip()[:80]}")
    assert not violations, (
        "PhysicalStateEstimate include graph dragged in a forbidden dependency "
        f"({len(violations)}):\n" + "\n".join(sorted(violations))
        + "\nThe source/state/ estimator must stay free of the command-dispatch "
          "surface (Commandable/CommandTypes/CommandProcessor), CODAL (MicroBit.h), "
          "and device handles (I2CBus/NezhaHAL)."
    )


def test_estimate_include_graph_is_subset_of_allowed():
    """The reachable firmware-header set is a subset of the documented allowed
    set (architecture-update.md §Module Definitions)."""
    graph = _reachable_headers(ROOT_HEADER)
    reached = {p.name for p in graph}
    unexpected = reached - ALLOWED_HEADERS
    assert not unexpected, (
        "PhysicalStateEstimate include graph reached unexpected headers "
        f"{sorted(unexpected)} — not in the documented allowed set "
        f"{sorted(ALLOWED_HEADERS)}.\nIf this is an intentional, dependency-clean "
        "addition, extend ALLOWED_HEADERS and update architecture-update.md "
        "§Module Definitions. If it dragged in a command/device dependency, "
        "the seam is broken."
    )


def test_protocol_h_is_a_types_header_not_a_command_surface():
    """Document the one subtle allowed include: Protocol.h is reachable (via
    Inputs.h) but carries ONLY reply tags + ReplyFn + KVPair POD types — it
    is NOT a command-dispatch surface (no Commandable / CommandDescriptor)."""
    proto = SOURCE_DIR / "types" / "Protocol.h"
    assert proto.exists()
    text = proto.read_text(errors="replace")
    assert "Commandable" not in text, (
        "Protocol.h now references Commandable — it has become a command-dispatch "
        "surface and is no longer an allowed transitive of the estimator."
    )
    assert "CommandDescriptor" not in text, (
        "Protocol.h now references CommandDescriptor — it has become a command "
        "surface and is no longer an allowed transitive of the estimator."
    )
