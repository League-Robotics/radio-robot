"""tests/unit/test_gen_messages_no_getters.py — getter regression guard (080-001).

`scripts/gen_messages.py` used to emit a `get_*`-prefixed accessor for every
field shape on a generated `msg::` struct (oneof-kind discriminator, `Opt<T>`,
message, enum, string, plain scalar). Every one was a trivial pass-through to
an already-public struct field — `naming-and-style.md` / `coding-standards.md`
ban `get_`-prefixed function names project-wide, so ticket 080-001 removed all
six getter-emitting branches from `_emit_message`; callers read the field
directly instead (`x.foo`, `x.foo_kind`, `x.foo.has` / `x.foo.val` for an
`Opt<T>` field).

This test is the guard against that getter reappearing. Per
architecture-update.md Decision 3, it invokes the generator itself
in-process (`gen_messages.generate_headers()`) and scans the emitted header
text for a `get_[a-z_]*(` method-defining pattern — NOT a grep restricted to
the checked-in `source/messages/*.h` files, so a regression is caught the
instant the generator template changes, before anyone forgets to regenerate
and commit.

Collected under `tests/unit/` (a generator/tooling-level check, not
sim/bench/playfield-scoped — see `tests/CLAUDE.md`); `pyproject.toml`'s
`testpaths` includes `tests/unit` so `uv run python -m pytest` collects it.
"""

import re
import sys
from pathlib import Path

# tests/unit/test_gen_messages_no_getters.py -> unit -> tests -> repo root
_REPO_ROOT = Path(__file__).resolve().parents[2]
_SCRIPTS_DIR = _REPO_ROOT / "scripts"

if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))

import gen_messages  # noqa: E402  (path must be set up before this import)

# Matches a get_-prefixed C++ method DEFINITION or CALL (a `(` immediately
# follows the name, modulo whitespace) -- never a field declaration, so a
# hypothetical field literally named `get_something` would not false-positive
# here (no proto field in protos/*.proto is named that way today).
_GET_METHOD_RE = re.compile(r"\bget_[a-z_]*\s*\(")


def test_generated_headers_define_no_get_prefixed_methods():
    """No header gen_messages.py emits may define a get_*-prefixed method."""
    outputs = gen_messages.generate_headers()

    assert outputs, "generator produced no headers -- check protos/ discovery"

    offenders = []
    for header_name, content in sorted(outputs.items()):
        for lineno, line in enumerate(content.splitlines(), 1):
            if _GET_METHOD_RE.search(line):
                offenders.append(f"{header_name}:{lineno}: {line.strip()}")

    assert not offenders, (
        "generated message headers must never define a get_*-prefixed "
        "method -- fields are plain public members; direct field access "
        "(x.foo, x.foo_kind, x.foo.has/.val) replaces get_foo() everywhere "
        "(ticket 080-001). Offending line(s):\n" + "\n".join(offenders)
    )


if __name__ == "__main__":
    import pytest

    sys.exit(pytest.main([__file__, "-v"]))
