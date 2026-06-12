#!/usr/bin/env python3
"""check_config_sync.py — Config registry sync lint.

Checks that the three sets that define "config" stay consistent:

  Set A — struct fields : every field declared inside `struct RobotConfig {}`
           in source/types/Config.h.

  Set B — registered keys : every key that appears in a CFG_F(, CFG_I(, or
           CFG_FI( macro call in source/robot/ConfigRegistry.cpp. The macro's
           second argument (the struct field name) is compared against Set A.
           The macro's first argument (the wire key name) drives Set C.

  Set C — firmware usage : every *registered* key (Set B wire key OR field
           name) that appears in a source file outside ConfigRegistry.cpp and
           DefaultConfig.cpp.  A key is "used" when the firmware actually reads
           or writes it — i.e. it has observable effect.

Three mismatches are reported:

  in-struct-not-registered
      Fields in Set A that have no corresponding entry in Set B.
      These fields are invisible to the host — SET/GET can never reach them.

  registered-not-in-struct
      Set B field names that do not exist in Set A.
      These entries are broken (offsetof of a nonexistent field would fail
      at compile time, but the check guards against copy-paste drift).

  registered-not-used
      Set B entries whose wire key or field name is not referenced in any
      source file other than ConfigRegistry.cpp and DefaultConfig.cpp.
      These entries accept SET commands but change nothing.

Usage
-----
Run from the repository root::

    python scripts/check_config_sync.py

Exit 0 when all three categories are empty (or all non-empty entries are
covered by the allowlist).  Exit 1 otherwise, printing the offending names.

Allowlist
---------
Create ``scripts/config_sync_allowlist.json`` to exempt known-intentional
exceptions.  Format::

    {
      "in-struct-not-registered": {
        "fieldName": "human-readable justification"
      },
      "registered-not-in-struct": {
        "fieldName": "human-readable justification"
      },
      "registered-not-used": {
        "keyName": "human-readable justification"
      }
    }

Any category key may be omitted; missing means no exemptions for that
category.  Each entry's value is the required justification string (must
be a non-empty string — the script refuses blank justifications so the
allowlist stays meaningful).
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

# ---------------------------------------------------------------------------
# Repo root resolution
# ---------------------------------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parent.parent
CONFIG_H   = REPO_ROOT / "source" / "types" / "Config.h"
REGISTRY_CPP = REPO_ROOT / "source" / "robot" / "ConfigRegistry.cpp"
SOURCE_DIR   = REPO_ROOT / "source"
ALLOWLIST_FILE = REPO_ROOT / "scripts" / "config_sync_allowlist.json"

# Files excluded from the "firmware usage" grep (these are the definition
# sites, not use sites).
EXCLUDED_FROM_USAGE = {
    REGISTRY_CPP.resolve(),
    (SOURCE_DIR / "robot" / "DefaultConfig.cpp").resolve(),
}


# ---------------------------------------------------------------------------
# Set A: parse struct RobotConfig fields from Config.h
# ---------------------------------------------------------------------------

def parse_struct_fields(path: Path) -> list[str]:
    """Return field names declared inside `struct RobotConfig { ... }`."""
    text = path.read_text()

    # Extract the struct body.
    m = re.search(r'\bstruct\s+RobotConfig\s*\{(.*?)\}', text, re.DOTALL)
    if not m:
        raise RuntimeError(f"Could not find 'struct RobotConfig' in {path}")

    body = m.group(1)
    fields: list[str] = []

    for line in body.splitlines():
        # Strip line comments.
        line = re.sub(r'//.*', '', line).strip()
        if not line or line.startswith('/*') or line.startswith('*'):
            continue
        # Match a field declaration: <type> <name> [= init] ;
        # Type may be: int8_t, uint8_t, int32_t, uint32_t, float, bool, ...
        # We capture the last identifier before the optional initialiser and ';'.
        m2 = re.match(
            r'^(?:const\s+)?(?:unsigned\s+)?'
            r'(?:int8_t|uint8_t|int16_t|uint16_t|int32_t|uint32_t|'
            r'int64_t|uint64_t|float|double|bool|int|unsigned|char)\s+'
            r'(\w+)'           # field name
            r'(?:\s*=\s*[^;]+)?'  # optional initialiser
            r'\s*;',
            line,
        )
        if m2:
            fields.append(m2.group(1))

    return fields


# ---------------------------------------------------------------------------
# Set B: parse CFG macro calls from ConfigRegistry.cpp
# ---------------------------------------------------------------------------

# Each entry is (wire_key, field_name).
def parse_registry_entries(path: Path) -> list[tuple[str, str]]:
    """Return (wire_key, field_name) pairs from CFG_F/CFG_I/CFG_FI macros."""
    text = path.read_text()
    # Match: CFG_F("key", fieldName)  or CFG_I(...) or CFG_FI(...)
    pattern = re.compile(
        r'CFG_(?:F|I|FI|B)\s*\(\s*"([^"]+)"\s*,\s*(\w+)\s*\)'
    )
    return pattern.findall(text)


# ---------------------------------------------------------------------------
# Set C: firmware usage grep
# ---------------------------------------------------------------------------

def build_usage_set(source_dir: Path, entries: list[tuple[str, str]]) -> set[str]:
    """Return the set of (wire_key or field_name) that appear in firmware source."""
    # Collect all .cpp and .h files, excluding definition sites.
    source_files = [
        f for f in source_dir.rglob('*')
        if f.suffix in ('.cpp', '.h') and f.resolve() not in EXCLUDED_FROM_USAGE
    ]

    # Build a combined corpus of all source text (annotated by file for
    # diagnostics, but here we just check presence).
    corpus = '\n'.join(f.read_text(errors='replace') for f in source_files)

    used: set[str] = set()
    for wire_key, field_name in entries:
        # A field name is "used" if it appears as a whole word anywhere in the
        # filtered source (cfg.fieldName, cfg->fieldName, etc.).
        if re.search(r'\b' + re.escape(field_name) + r'\b', corpus):
            used.add(wire_key)
        # Also accept if the wire key itself appears (unlikely outside registry,
        # but possible in comments or string literals we want to catch).
        elif re.search(r'\b' + re.escape(wire_key) + r'\b', corpus):
            used.add(wire_key)

    return used


# ---------------------------------------------------------------------------
# Allowlist loader
# ---------------------------------------------------------------------------

def load_allowlist(path: Path) -> dict[str, dict[str, str]]:
    """Load the allowlist JSON, returning an empty structure if absent."""
    if not path.exists():
        return {}
    data = json.loads(path.read_text())
    # Validate: each value must be a non-empty string justification.
    for category, entries in data.items():
        for name, justification in entries.items():
            if not isinstance(justification, str) or not justification.strip():
                raise ValueError(
                    f"Allowlist entry '{name}' in '{category}' has a blank "
                    f"justification — every exemption requires a non-empty reason."
                )
    return data


# ---------------------------------------------------------------------------
# Main lint logic
# ---------------------------------------------------------------------------

def run_lint() -> int:
    """Perform the lint.  Returns 0 (clean) or 1 (drift detected)."""

    # Parse.
    struct_fields: list[str] = parse_struct_fields(CONFIG_H)
    registry_entries: list[tuple[str, str]] = parse_registry_entries(REGISTRY_CPP)

    struct_set = set(struct_fields)
    reg_field_set = {field for _, field in registry_entries}
    reg_key_set   = {key   for key, _ in registry_entries}
    # Map wire_key -> field_name for usage reporting.
    key_to_field  = {key: field for key, field in registry_entries}

    used_keys = build_usage_set(SOURCE_DIR, registry_entries)

    # Compute the three mismatch categories.
    in_struct_not_registered  = struct_set - reg_field_set
    registered_not_in_struct  = reg_field_set - struct_set
    registered_not_used       = reg_key_set - used_keys

    # Load allowlist.
    allowlist = load_allowlist(ALLOWLIST_FILE)

    al_not_registered = allowlist.get("in-struct-not-registered", {})
    al_not_in_struct  = allowlist.get("registered-not-in-struct", {})
    al_not_used       = allowlist.get("registered-not-used", {})

    # Apply allowlist.
    offenders_a = in_struct_not_registered - set(al_not_registered)
    offenders_b = registered_not_in_struct - set(al_not_in_struct)
    # registered-not-used is keyed by wire_key in the allowlist.
    offenders_c = registered_not_used - set(al_not_used)

    # Report.
    clean = True

    def _report(category: str, items: set[str]) -> None:
        nonlocal clean
        if items:
            clean = False
            print(f"\n[FAIL] {category}:")
            for name in sorted(items):
                print(f"  - {name}")

    def _report_ok(category: str, items: set[str], allowlisted: dict) -> None:
        exempt = {k: v for k, v in allowlisted.items() if k in items}
        if exempt:
            print(f"\n[OK]   {category} (allowlisted):")
            for name, reason in sorted(exempt.items()):
                print(f"  - {name}: {reason}")

    # Show allowlisted entries so the output is fully transparent.
    _report_ok("in-struct-not-registered",
               in_struct_not_registered, al_not_registered)
    _report_ok("registered-not-in-struct",
               registered_not_in_struct, al_not_in_struct)
    # For registered-not-used, al_not_used is keyed by wire_key.
    _report_ok("registered-not-used", registered_not_used, al_not_used)

    _report("in-struct-not-registered",  offenders_a)
    _report("registered-not-in-struct",  offenders_b)
    _report("registered-not-used",       offenders_c)

    if clean:
        print("check_config_sync: OK — no drift detected.")
        return 0
    else:
        print("\ncheck_config_sync: FAIL — see above.")
        return 1


if __name__ == "__main__":
    sys.exit(run_lint())
