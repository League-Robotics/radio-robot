"""
test_logging_contract.py — verify that source/subsystems/ contains no
print statements or telemetry emits.

§6 logging contract: every subsystem writes its inputs slice in updateInputs(),
and NO subsystem prints. Phase E moved subsystem bodies verbatim so no prints
were introduced, but there was no gate. This grep-based test makes the contract
permanent: a subsystem that starts printing (printf), emitting telemetry
(telemetryEmit), or replying to the command channel (replyFn / snprintf-into-reply)
fails CI.
"""
import pathlib
import re

REPO_ROOT = pathlib.Path(__file__).parent.parent.parent.parent
SUBSYSTEMS_DIR = REPO_ROOT / "source" / "subsystems"

FORBIDDEN_PATTERNS = [
    re.compile(r"\bprintf\s*\("),
    re.compile(r"\btelemetryEmit\s*\("),
    re.compile(r"\bsnprintf\b.*replyFn"),
    re.compile(r"replyFn\s*\("),
]


def _find_violations():
    violations = []
    for path in SUBSYSTEMS_DIR.rglob("*.cpp"):
        text = path.read_text()
        for lineno, line in enumerate(text.splitlines(), 1):
            for pat in FORBIDDEN_PATTERNS:
                if pat.search(line):
                    violations.append(
                        f"{path.relative_to(REPO_ROOT)}:{lineno}: {line.strip()}"
                    )
    return violations


def test_subsystems_dir_exists():
    """The subsystems tree must exist for this contract to be meaningful."""
    assert SUBSYSTEMS_DIR.is_dir(), f"subsystems dir missing: {SUBSYSTEMS_DIR}"


def test_no_subsystem_prints():
    """No subsystem in source/subsystems/ calls printf, telemetryEmit, or replyFn."""
    v = _find_violations()
    assert not v, (
        "Logging contract violation(s) in source/subsystems/:\n" + "\n".join(v)
    )
