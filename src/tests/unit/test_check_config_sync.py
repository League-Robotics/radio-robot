"""src/tests/unit/test_check_config_sync.py — 096-008 (check_config_sync.py rewrite).

Covers `scripts/check_config_sync.py`'s new comparison model: pydantic
`RobotConfig` fields (flattened to dotted paths) vs. the generated `pb2`
descriptors for the two curated Patch messages this script tracks
(`DrivetrainConfigPatch`/`MotorConfigPatch` -- `PlannerConfigPatch` was
tracked here through sprint 114, DELETED 115-003, gut-to-minimal-firmware
S1 motion-stack excision).

Two kinds of coverage:

1. An end-to-end subprocess run of the real script against the real repo
   tree — proves the rewrite actually runs clean today (acceptance
   criterion 1: exits 0, never crashes).
2. Direct unit tests of `compute_findings()` — the pure diff function — with
   synthetic pydantic-field/patch-field/mapping inputs, proving each
   reported category (`pydantic-field-no-patch`, `patch-field-no-pydantic`,
   `type-mismatch`, `unmapped-patch-field`, `stale-map-target`) actually
   fires on an intentionally introduced mismatch, and that a fully-mapped,
   fully-consistent scratch scenario reports nothing at all.

Collected under src/tests/unit/ (a host-tooling check, not sim/bench/playfield-
scoped — see tests/CLAUDE.md); pyproject.toml's testpaths includes
tests/unit.
"""

import subprocess
import sys
import importlib.util
from pathlib import Path

# src/tests/unit/test_check_config_sync.py -> unit -> tests -> repo root
_REPO_ROOT = Path(__file__).resolve().parents[3]
_SCRIPT = _REPO_ROOT / "src" / "scripts" / "check_config_sync.py"


def _load_module():
    """Import scripts/check_config_sync.py in-process (it isn't a package)."""
    spec = importlib.util.spec_from_file_location("check_config_sync", _SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# ---------------------------------------------------------------------------
# End-to-end: the real script against the real tree.
# ---------------------------------------------------------------------------

def test_script_exits_zero_on_current_tree():
    """python scripts/check_config_sync.py exits 0 against the checked-in
    RobotConfig pydantic model + config_pb2 Patch descriptors + allowlist."""
    result = subprocess.run(
        [sys.executable, str(_SCRIPT)],
        cwd=_REPO_ROOT,
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, (
        f"check_config_sync.py exited {result.returncode}, expected 0.\n"
        f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}"
    )
    assert "OK — no drift detected" in result.stdout


# ---------------------------------------------------------------------------
# compute_findings() — pure-function diff, synthetic scenarios.
# ---------------------------------------------------------------------------

def test_compute_findings_clean_scenario_reports_nothing():
    """A fully-mapped, fully-consistent scratch scenario yields empty sets
    in every category — the diff doesn't invent phantom findings."""
    mod = _load_module()
    from google.protobuf.descriptor import FieldDescriptor

    pydantic_fields = {"geometry.trackwidth": float}
    patch_fields = {("DrivetrainConfigPatch", "trackwidth"): FieldDescriptor.CPPTYPE_FLOAT}
    mapping = {("DrivetrainConfigPatch", "trackwidth"): ["geometry.trackwidth"]}

    findings = mod.compute_findings(pydantic_fields, patch_fields, mapping)

    assert all(len(items) == 0 for items in findings.values()), findings


def test_compute_findings_flags_pydantic_field_with_no_patch_field():
    """A pydantic leaf that isn't any mapping target is reported —
    'invisible to the binary config plane' in the other direction."""
    mod = _load_module()
    from google.protobuf.descriptor import FieldDescriptor

    pydantic_fields = {"geometry.trackwidth": float, "identity.robot_name": str}
    patch_fields = {("DrivetrainConfigPatch", "trackwidth"): FieldDescriptor.CPPTYPE_FLOAT}
    mapping = {("DrivetrainConfigPatch", "trackwidth"): ["geometry.trackwidth"]}

    findings = mod.compute_findings(pydantic_fields, patch_fields, mapping)

    assert findings["pydantic-field-no-patch"] == {"identity.robot_name"}


def test_compute_findings_flags_patch_field_with_no_pydantic_field():
    """A Patch descriptor field explicitly mapped to `[]` (no host field) is
    reported — the wire allows something the host can't represent."""
    mod = _load_module()
    from google.protobuf.descriptor import FieldDescriptor

    pydantic_fields = {}
    patch_fields = {("DrivetrainConfigPatch", "ekf_q_xy"): FieldDescriptor.CPPTYPE_FLOAT}
    mapping = {("DrivetrainConfigPatch", "ekf_q_xy"): []}

    findings = mod.compute_findings(pydantic_fields, patch_fields, mapping)

    assert findings["patch-field-no-pydantic"] == {"DrivetrainConfigPatch.ekf_q_xy"}


def test_compute_findings_flags_type_mismatch():
    """A mapped pair exists on both sides, but the pydantic leaf's type
    doesn't match what the Patch field's cpp_type implies."""
    mod = _load_module()
    from google.protobuf.descriptor import FieldDescriptor

    pydantic_fields = {"geometry.trackwidth": str}  # wrong on purpose (should be float)
    patch_fields = {("DrivetrainConfigPatch", "trackwidth"): FieldDescriptor.CPPTYPE_FLOAT}
    mapping = {("DrivetrainConfigPatch", "trackwidth"): ["geometry.trackwidth"]}

    findings = mod.compute_findings(pydantic_fields, patch_fields, mapping)

    assert findings["type-mismatch"] == {"DrivetrainConfigPatch.trackwidth->geometry.trackwidth"}


def test_compute_findings_flags_unmapped_patch_field():
    """A Patch descriptor field the mapping table has never heard of at all
    (not even a documented `[]`) — the 'real skew' canary for a future
    ticket adding a new wire field without updating this script."""
    mod = _load_module()
    from google.protobuf.descriptor import FieldDescriptor

    pydantic_fields = {}
    patch_fields = {("DrivetrainConfigPatch", "brand_new_field"): FieldDescriptor.CPPTYPE_FLOAT}
    mapping = {}

    findings = mod.compute_findings(pydantic_fields, patch_fields, mapping)

    assert findings["unmapped-patch-field"] == {"DrivetrainConfigPatch.brand_new_field"}


def test_compute_findings_flags_stale_map_target():
    """A mapping target that no longer exists in the pydantic model —
    the model was refactored and the script's own map wasn't updated."""
    mod = _load_module()
    from google.protobuf.descriptor import FieldDescriptor

    pydantic_fields = {}  # geometry.trackwidth renamed/removed
    patch_fields = {("DrivetrainConfigPatch", "trackwidth"): FieldDescriptor.CPPTYPE_FLOAT}
    mapping = {("DrivetrainConfigPatch", "trackwidth"): ["geometry.trackwidth"]}

    findings = mod.compute_findings(pydantic_fields, patch_fields, mapping)

    assert findings["stale-map-target"] == {"DrivetrainConfigPatch.trackwidth->geometry.trackwidth"}


# ---------------------------------------------------------------------------
# Allowlist application — forced-fail categories are never exemptible.
# ---------------------------------------------------------------------------

def test_forced_fail_categories_ignore_the_allowlist():
    """unmapped-patch-field / stale-map-target can NEVER be silenced by the
    JSON allowlist — only by editing PATCH_TO_PYDANTIC itself."""
    mod = _load_module()

    findings = {c: set() for c in mod.ALL_CATEGORIES}
    findings["unmapped-patch-field"] = {"DrivetrainConfigPatch.brand_new_field"}
    allowlist = {"unmapped-patch-field": {"DrivetrainConfigPatch.brand_new_field": "please ignore"}}

    clean = mod.apply_allowlist_and_report(findings, allowlist)

    assert clean is False


def test_pydantic_field_no_patch_allowlist_supports_wildcard_prefix():
    """A `"section.*"` allowlist key exempts every dotted path under it."""
    mod = _load_module()

    findings = {c: set() for c in mod.ALL_CATEGORIES}
    findings["pydantic-field-no-patch"] = {"identity.robot_name", "identity.uid"}
    allowlist = {"pydantic-field-no-patch": {"identity.*": "host-only identity metadata"}}

    clean = mod.apply_allowlist_and_report(findings, allowlist)

    assert clean is True


# ---------------------------------------------------------------------------
# The real map's own internal consistency (belt-and-suspenders on top of the
# end-to-end run above — pins the exact field count so a future ticket 001-
# style change to config.proto is caught even if the allowlist happened to
# already cover the new field by accident).
# ---------------------------------------------------------------------------

def test_real_patch_to_pydantic_map_has_no_forced_failures():
    mod = _load_module()

    from robot_radio.config.robot_config import RobotConfig
    from robot_radio.robot.pb2 import config_pb2

    pydantic_fields = mod.flatten_pydantic_fields(RobotConfig)
    patch_fields = mod.collect_patch_fields(config_pb2)
    findings = mod.compute_findings(pydantic_fields, patch_fields, mod.PATCH_TO_PYDANTIC)

    assert findings["unmapped-patch-field"] == set()
    assert findings["stale-map-target"] == set()
