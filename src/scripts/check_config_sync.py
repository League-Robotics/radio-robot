#!/usr/bin/env python3
"""check_config_sync.py — binary config-plane sync lint (096-008 rewrite).

Full rewrite. The PREVIOUS implementation compared `struct RobotConfig` in
`src/firm/types/Config.h` against `CFG_F`/`CFG_I`/`CFG_FI` macro calls in
`source/robot/ConfigRegistry.cpp` — both `source_old`-era files that do not
exist anywhere in the post-077-rebuild `source/` tree. That comparison had
been silently non-functional (crashing with a bare ``RuntimeError`` the
moment anyone ran it) since the greenfield rebuild; see
`clasi/sprints/096-.../architecture-update.md` Decision 6.

New comparison model
---------------------
Protocol v3's binary config plane (096) exposes exactly three curated
"Patch" messages on the wire — `DrivetrainConfigPatch`, `MotorConfigPatch`,
`PlannerConfigPatch` (`protos/config.proto`, generated to
`src/host/robot_radio/robot/pb2/config_pb2.py` by ticket 096-001). Each `Patch`
field is a `proto3 optional` scalar; presence signals "this field is being
set/was populated". These three messages are a CURATED SUBSET of a much
larger config surface — they mirror only the ~15 keys
`src/firm/commands/config_commands.cpp`'s `kAllKeys` registers for the
text SET/GET verb (config.proto's own file header explains why: the full
generated `DrivetrainConfig`/`MotorConfig`/`PlannerConfig` messages don't
fit the envelope budget and most of their fields have no wire-config verb
at all).

`src/host/robot_radio/config/robot_config.py`'s `RobotConfig` pydantic model is
the host's per-robot JSON config — identity, connection, vision, geometry,
wheels, encoders, drive, gripper, peripherals, calibration, and PID/control
tuning. It is intentionally NOT a 1:1 mirror of the wire Patch surface: most
of it (robot identity, camera/vision geometry, host-only crawl-mode tuning,
...) has no wire config verb and never will. Only a SMALL slice of it is
the host-side representation of a value that can also be pushed over the
binary config plane.

This script therefore:

  1. Flattens `RobotConfig`'s nested pydantic fields to dotted paths
     (e.g. ``geometry.trackwidth``, ``control.vel_kp``).
  2. Reads the three curated `Patch` messages' fields straight from the
     generated `pb2` descriptors (never re-derives them from `.proto`
     source text — the descriptor IS the wire truth).
  3. Diffs the two via `PATCH_TO_PYDANTIC`, a hand-curated map below from
     every `(PatchMessage, field)` pair to the dotted pydantic path(s) that
     represent it host-side (or `[]` when no host-side field exists yet).
     This map is itself part of what this script checks: a `Patch` field
     the map has no entry for AT ALL (not even a documented `[]`) is a
     forced failure, not an allowlist candidate — it means ticket 096-001
     (or a future ticket) added/renamed a wire field and nobody updated
     this script, which is exactly the "real skew" this lint exists to
     catch.

Reported categories
--------------------
  patch-field-no-pydantic
      A `Patch` descriptor field has no pydantic counterpart (`[]` in
      `PATCH_TO_PYDANTIC`). The binary config plane can set/get a value the
      host has nowhere to keep. Allowlistable — this is expected for a
      handful of fields today (the EKF covariances, the `min_speed` planner
      floor, and `MotorConfigPatch.side`, which is a selector enum, not a
      settable value).

  pydantic-field-no-patch
      A pydantic leaf field is not any `PATCH_TO_PYDANTIC` mapping target.
      By the curated-subset design (architecture-update.md (096) Decision
      2) this is true for MOST of `RobotConfig` — identity, vision,
      geometry offsets, drive/crawl tuning, and several stale pre-096
      `control.*` fields that reference wire keys
      (`vel.kP`/`sync`/`minWheelMms`/`turnGate`/`yawRateMax`) that no longer
      exist in `config_commands.cpp`'s current key table at all. Every one
      of these is enumerated and requires an explicit
      `config_sync_allowlist.json` entry (wildcard prefixes like
      `"identity.*"` are supported) — nothing is silently skipped.

  type-mismatch
      A mapped pair exists on both sides, but the pydantic leaf's Python
      type is not what the `Patch` field's protobuf `cpp_type` implies
      (e.g. a `CPPTYPE_FLOAT` field mapped to a pydantic `str` field).
      Allowlistable, though none exist on the current tree.

  unmapped-patch-field / stale-map-target
      Internal-consistency failures in `PATCH_TO_PYDANTIC` itself (a wire
      field this script has never seen, or a mapping target that no longer
      exists in the pydantic model). NEVER allowlistable — these mean the
      map above must be edited (a code change), not the JSON escape hatch.

Bound/option checking: `protos/config.proto`'s own header note records that
no `Patch` field carries a `(min)`/`(max)`/`(abs_max)` option today (096-001
Decision 6's validation-note) — `validateCandidate()`'s business rules
(`trackwidth > 0`, `rotational_slip`'s non-contiguous domain, `sTimeout >
0`) live in hand-written C++, not the wire schema. `CPP_TYPE_TO_PY` below
still reads `field.GetOptions()` so this check activates automatically the
day any `Patch` field gains one, without needing another rewrite.

Mapping from the OLD Set-A/B/C language, for a reader who remembers it
------------------------------------------------------------------------
  Set A (struct fields)     → pydantic model, flattened to dotted paths.
  Set B (registered keys)   → `Patch` descriptor fields (the wire surface).
  Set C (firmware usage)    → dropped entirely. There is no host-side
                               equivalent of "grep the firmware source for
                               this key" left to check — `Patch` fields ARE
                               the registered/usable wire surface by
                               construction (unlike the old free-text
                               `CFG_F(...)` table, a field that exists in
                               the generated descriptor is, by definition,
                               reachable by `BinaryChannel`, ticket 004).
  in-struct-not-registered  → `pydantic-field-no-patch`.
  registered-not-in-struct  → `patch-field-no-pydantic`.
  registered-not-used       → dropped (no Set-C equivalent — see above).

Usage
-----
Run from the repository root::

    python scripts/check_config_sync.py

Exit 0 when every category is empty or every non-empty entry is covered by
the allowlist (`unmapped-patch-field`/`stale-map-target` excepted — see
above). Exit 1 otherwise, printing the offending names. Never raises an
uncaught exception — a missing/unimportable input (e.g. `robot_radio` not
on `sys.path`) is reported as a clean failure, not a traceback.

Allowlist
---------
Create ``scripts/config_sync_allowlist.json`` to exempt known-intentional
exceptions. Format::

    {
      "pydantic-field-no-patch": {
        "identity.*": "human-readable justification (wildcard prefix ok)"
      },
      "patch-field-no-pydantic": {
        "DrivetrainConfigPatch.ekf_q_xy": "human-readable justification"
      },
      "type-mismatch": {
        "DrivetrainConfigPatch.trackwidth->geometry.trackwidth": "..."
      }
    }

Any category key may be omitted; missing means no exemptions for that
category. Each entry's value must be a non-empty justification string. A
key ending in ``".*"`` matches any item sharing that dotted-path prefix
(``"identity.*"`` matches ``"identity.robot_name"``, etc.) — only
`pydantic-field-no-patch` entries use dotted paths, so only that category
honors the wildcard form.
"""

from __future__ import annotations

import json
import sys
import typing
from pathlib import Path
from typing import Optional

from google.protobuf.descriptor import FieldDescriptor

# ---------------------------------------------------------------------------
# Repo root / import setup
# ---------------------------------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
HOST_DIR = REPO_ROOT / "src" / "host"
ALLOWLIST_FILE = REPO_ROOT / "src" / "scripts" / "config_sync_allowlist.json"

if str(HOST_DIR) not in sys.path:
    sys.path.insert(0, str(HOST_DIR))


# ---------------------------------------------------------------------------
# The curated wire<->host mapping. See the module docstring for what each
# category means and why unmapped/stale entries are never allowlistable.
# ---------------------------------------------------------------------------

PatchKey = tuple[str, str]  # (message_name, field_name)

PATCH_TO_PYDANTIC: dict[PatchKey, list[str]] = {
    # -- DrivetrainConfigPatch ------------------------------------------------
    ("DrivetrainConfigPatch", "trackwidth"): ["geometry.trackwidth"],
    ("DrivetrainConfigPatch", "rotational_slip"): ["calibration.rotational_slip"],
    # EKF covariances: no host-side pydantic field yet. Carried over from the
    # pre-096 Config.h-era allowlist's ekfQxy/ekfQtheta/ekfROtosXy entries
    # ("register as tunable in a future sprint") — still true today.
    ("DrivetrainConfigPatch", "ekf_q_xy"): [],
    ("DrivetrainConfigPatch", "ekf_q_theta"): [],
    ("DrivetrainConfigPatch", "ekf_r_otos_xy"): [],
    ("DrivetrainConfigPatch", "ekf_r_otos_theta"): [],
    # ekf_r_fix_xy/ekf_r_fix_theta (099-008): the delayed camera-fix's own
    # UNGATED measurement-noise pair. No host-side pydantic field yet --
    # mirrors the ekf_r_otos_xy/ekf_r_otos_theta entries immediately above
    # exactly (same "register as tunable in a future sprint" posture).
    ("DrivetrainConfigPatch", "ekf_r_fix_xy"): [],
    ("DrivetrainConfigPatch", "ekf_r_fix_theta"): [],
    # -- MotorConfigPatch -------------------------------------------------------
    # `side` is a selector enum (which bound motor `travel_calib` targets),
    # not a settable config value — it has no pydantic counterpart by design.
    ("MotorConfigPatch", "side"): [],
    # travel_calib is side-selected: LEFT -> mm_per_wheel_deg_left, RIGHT ->
    # mm_per_wheel_deg_right. Both targets are listed; either existing in the
    # pydantic model is sufficient (see compute_findings()).
    ("MotorConfigPatch", "travel_calib"): [
        "calibration.mm_per_wheel_deg_left",
        "calibration.mm_per_wheel_deg_right",
    ],
    ("MotorConfigPatch", "kp"): ["control.vel_kp"],
    ("MotorConfigPatch", "ki"): ["control.vel_ki"],
    ("MotorConfigPatch", "kff"): ["control.vel_kff"],
    ("MotorConfigPatch", "i_max"): ["control.vel_imax"],
    ("MotorConfigPatch", "kaw"): ["control.vel_kaw"],
    # -- PlannerConfigPatch -----------------------------------------------------
    # No host-side pydantic field. DriveConfig.min_drive_mm_s is a DIFFERENT
    # quantity (rogo's host-only crawl-mode fallback floor), not the
    # firmware planner's trapezoidal-profile min_speed — do not conflate them.
    ("PlannerConfigPatch", "min_speed"): [],
    # heading_kp/heading_kd (098-005; pydantic fields added 2026-07-18): the
    # two outer heading-loop PD gains. ControlConfig.heading_kp/heading_kd
    # now exist and ride the connect-time calibration push
    # (calibration_commands() -> `SET headingKp/headingKd`), alongside the
    # gen_boot_config.py build-time bake that was always there.
    ("PlannerConfigPatch", "heading_kp"): ["control.heading_kp"],
    ("PlannerConfigPatch", "heading_kd"): ["control.heading_kd"],
    # arrive_dwell (100-001): the one Drive::Limits/tracker/policy field
    # (of the original PlannerConfig fields 15-31, architecture-update.md
    # M1) that turned out to be live. No host-side pydantic field yet --
    # bench tuning lives in data/robots/tovez.json's control.arrive_dwell
    # key (baked into the firmware boot config by scripts/
    # gen_boot_config.py), not in RobotConfig; same "no pydantic round trip
    # yet" posture as heading_kp/heading_kd immediately above. Its 16 dead
    # siblings (v_wheel_max..arrive_vel_tol) were removed as dead wire
    # fields in 111-004 -- see config.proto's own PlannerConfigPatch header
    # comment -- and their PATCH_TO_PYDANTIC entries removed alongside them
    # (a removed wire field is not a `patch-field-no-pydantic` finding; it
    # is simply absent from `patch_fields`, so this map's own consistency
    # would tolerate a stale entry silently -- removed anyway for hygiene).
    ("PlannerConfigPatch", "arrive_dwell"): [],
}

# proto3 scalar cpp_type -> the Python type a pydantic leaf representing it
# should have. CPPTYPE_ENUM/CPPTYPE_MESSAGE are deliberately absent: an enum
# selector (MotorConfigPatch.side) and any nested message have no single
# scalar Python type to compare against.
CPP_TYPE_TO_PY: dict[int, type] = {
    FieldDescriptor.CPPTYPE_INT32: int,
    FieldDescriptor.CPPTYPE_INT64: int,
    FieldDescriptor.CPPTYPE_UINT32: int,
    FieldDescriptor.CPPTYPE_UINT64: int,
    FieldDescriptor.CPPTYPE_DOUBLE: float,
    FieldDescriptor.CPPTYPE_FLOAT: float,
    FieldDescriptor.CPPTYPE_BOOL: bool,
    FieldDescriptor.CPPTYPE_STRING: str,
}

PATCH_MESSAGE_NAMES = ("DrivetrainConfigPatch", "MotorConfigPatch", "PlannerConfigPatch")

ALLOWLIST_CATEGORIES = ("pydantic-field-no-patch", "patch-field-no-pydantic", "type-mismatch")
FORCED_FAIL_CATEGORIES = ("unmapped-patch-field", "stale-map-target")
ALL_CATEGORIES = ALLOWLIST_CATEGORIES + FORCED_FAIL_CATEGORIES


# ---------------------------------------------------------------------------
# Pydantic model flattening
# ---------------------------------------------------------------------------

def _unwrap_optional(annotation: object) -> object:
    """Return the non-None arm of `Optional[T]`, or `annotation` unchanged."""
    if typing.get_origin(annotation) is typing.Union:
        args = [a for a in typing.get_args(annotation) if a is not type(None)]
        if len(args) == 1:
            return args[0]
    return annotation


def _is_basemodel(annotation: object) -> bool:
    try:
        from pydantic import BaseModel
    except ImportError:
        return False
    return isinstance(annotation, type) and issubclass(annotation, BaseModel)


def flatten_pydantic_fields(model_cls: type, prefix: str = "") -> dict[str, type]:
    """Recursively flatten a pydantic BaseModel's fields to dotted-path -> leaf type.

    Nested BaseModel fields (e.g. `geometry: GeometryConfig`) are expanded
    in place (`geometry.trackwidth`, ...) rather than kept as one opaque
    entry, so every scalar leaf is individually checkable.
    """
    fields: dict[str, type] = {}
    for name, field_info in model_cls.model_fields.items():
        path = f"{prefix}{name}"
        leaf = _unwrap_optional(field_info.annotation)
        if _is_basemodel(leaf):
            fields.update(flatten_pydantic_fields(leaf, prefix=f"{path}."))
        else:
            fields[path] = leaf
    return fields


# ---------------------------------------------------------------------------
# pb2 descriptor collection
# ---------------------------------------------------------------------------

def collect_patch_fields(config_pb2_module: object) -> dict[PatchKey, int]:
    """Return {(message_name, field_name): cpp_type} for the three curated Patch messages."""
    fields: dict[PatchKey, int] = {}
    for msg_name in PATCH_MESSAGE_NAMES:
        msg_cls = getattr(config_pb2_module, msg_name)
        for field in msg_cls.DESCRIPTOR.fields:
            fields[(msg_name, field.name)] = field.cpp_type
    return fields


# ---------------------------------------------------------------------------
# Allowlist loader (same shape/contract as the pre-rewrite script)
# ---------------------------------------------------------------------------

def load_allowlist(path: Path) -> dict[str, dict[str, str]]:
    """Load the allowlist JSON, returning an empty structure if absent."""
    if not path.exists():
        return {}
    data = json.loads(path.read_text())
    for category, entries in data.items():
        for name, justification in entries.items():
            if not isinstance(justification, str) or not justification.strip():
                raise ValueError(
                    f"Allowlist entry '{name}' in '{category}' has a blank "
                    f"justification — every exemption requires a non-empty reason."
                )
    return data


def _allowlist_match(item: str, allowed: dict[str, str]) -> Optional[str]:
    """Return the justification for `item`, honoring `"prefix.*"` wildcard keys."""
    if item in allowed:
        return allowed[item]
    for key, justification in allowed.items():
        if key.endswith(".*") and item.startswith(key[:-1]):
            return justification
    return None


# ---------------------------------------------------------------------------
# Core diff — pure function, no I/O, directly unit-testable with synthetic
# pydantic_fields/patch_fields/mapping.
# ---------------------------------------------------------------------------

def compute_findings(
    pydantic_fields: dict[str, type],
    patch_fields: dict[PatchKey, int],
    mapping: dict[PatchKey, list[str]],
) -> dict[str, set[str]]:
    """Diff pydantic leaf fields against Patch descriptor fields via `mapping`.

    Returns one `set[str]` per category in `ALL_CATEGORIES` (see module
    docstring for what each category means and its item-string format).
    """
    findings: dict[str, set[str]] = {c: set() for c in ALL_CATEGORIES}

    mapped_pydantic_targets: set[str] = set()

    for key, cpp_type in patch_fields.items():
        msg_name, field_name = key
        label = f"{msg_name}.{field_name}"

        if key not in mapping:
            findings["unmapped-patch-field"].add(label)
            continue

        targets = mapping[key]
        if not targets:
            findings["patch-field-no-pydantic"].add(label)
            continue

        expected_py_type = CPP_TYPE_TO_PY.get(cpp_type)
        for target in targets:
            mapped_pydantic_targets.add(target)
            if target not in pydantic_fields:
                findings["stale-map-target"].add(f"{label}->{target}")
                continue
            actual_type = pydantic_fields[target]
            if expected_py_type is not None and actual_type is not expected_py_type:
                findings["type-mismatch"].add(f"{label}->{target}")

    for pydantic_path in pydantic_fields:
        if pydantic_path not in mapped_pydantic_targets:
            findings["pydantic-field-no-patch"].add(pydantic_path)

    return findings


# ---------------------------------------------------------------------------
# Reporting + allowlist application
# ---------------------------------------------------------------------------

def apply_allowlist_and_report(findings: dict[str, set[str]], allowlist: dict[str, dict[str, str]]) -> bool:
    """Print the report. Returns True (clean) or False (offenders remain)."""
    clean = True

    for category in ALLOWLIST_CATEGORIES:
        allowed = allowlist.get(category, {})
        items = findings[category]
        exempt = {item: _allowlist_match(item, allowed) for item in items if _allowlist_match(item, allowed)}
        offenders = items - set(exempt)

        if exempt:
            print(f"\n[OK]   {category} (allowlisted):")
            for name in sorted(exempt):
                print(f"  - {name}: {exempt[name]}")
        if offenders:
            clean = False
            print(f"\n[FAIL] {category}:")
            for name in sorted(offenders):
                print(f"  - {name}")

    for category in FORCED_FAIL_CATEGORIES:
        items = findings[category]
        if items:
            clean = False
            print(f"\n[FAIL] {category} (not allowlistable — edit PATCH_TO_PYDANTIC in "
                  f"scripts/check_config_sync.py instead):")
            for name in sorted(items):
                print(f"  - {name}")

    return clean


# ---------------------------------------------------------------------------
# Main lint logic
# ---------------------------------------------------------------------------

def run_lint() -> int:
    """Perform the lint. Returns 0 (clean) or 1 (drift detected / inputs unavailable)."""
    try:
        from robot_radio.config.robot_config import RobotConfig
        from robot_radio.robot.pb2 import config_pb2
    except ImportError as exc:
        print(f"check_config_sync: SKIP-FAIL — could not import host inputs: {exc}")
        print("  Run from the repository root with `robot_radio` importable, "
              "e.g. `uv run python scripts/check_config_sync.py`.")
        return 1

    try:
        pydantic_fields = flatten_pydantic_fields(RobotConfig)
        patch_fields = collect_patch_fields(config_pb2)
        allowlist = load_allowlist(ALLOWLIST_FILE)
        findings = compute_findings(pydantic_fields, patch_fields, PATCH_TO_PYDANTIC)
        clean = apply_allowlist_and_report(findings, allowlist)
    except Exception as exc:  # noqa: BLE001 — never let this script crash the caller
        print(f"check_config_sync: SKIP-FAIL — unexpected error: {exc!r}")
        return 1

    if clean:
        print("check_config_sync: OK — no drift detected.")
        return 0
    else:
        print("\ncheck_config_sync: FAIL — see above.")
        return 1


if __name__ == "__main__":
    sys.exit(run_lint())
