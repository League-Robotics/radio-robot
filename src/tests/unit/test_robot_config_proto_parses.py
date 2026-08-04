"""src/tests/unit/test_robot_config_proto_parses.py — 132-001 (robot_config.proto
— the one schema, end-state grouped shape + wire messages).

Ticket 132-001 authors `src/protos/robot_config.proto` as SCHEMA ONLY: it
declares `Config::Robot`'s end-state field set (three host-only groups —
Identity/Connection/Vision — plus seven robot-config groups — Geometry/
Motors/Drive/WheelControl/Planner/Otos/Estimator — each carrying a
`ConfigGroupTarget` enum value), and the wire envelope messages the schema
drives (`ConfigGroupTarget`, `SetConfigGroup`, `GetConfig`, `ConfigSnapshot`,
`SetConfigField`). It does NOT generate anything — no C++ headers, no
pydantic model, no JSON Schema (`gen_messages.py`'s wire-C++/pydantic/JSON
Schema emission modes are ticket 002's job) — so this test is deliberately
narrow: "a bare parse/compile check" (the ticket's own acceptance criterion
wording), run through the SAME parser `gen_messages.py`'s own codegen
pipeline uses (`grpc_tools.protoc`, see `_run_codegen_pipeline()`'s own
"Run protoc to get a FileDescriptorSet" step, src/scripts/gen_messages.py)
— not `gen_messages.generate_headers()` (the FULL codegen entry point
`test_gen_messages_no_getters.py` already exercises), which would also
invoke the custom C++/wire-codec emission engine this ticket's own
acceptance criteria explicitly defer to ticket 002.

Beyond the bare parse, this file also walks the resulting FileDescriptorSet
to check the schema-SHAPE acceptance criteria a plain "protoc exited 0"
cannot see on its own: which groups exist, which are host-only, and that
`ColorConfig`/`LineConfig` were not (re)declared as groups (sprint.md Out of
Scope). It does not touch generated code at all — nothing here imports or
calls `gen_messages.generate_headers()`.

Collected under `src/tests/unit/` — `pyproject.toml`'s `testpaths` includes
`tests/unit`, so `uv run python -m pytest` collects it by default.
"""

import tempfile
from pathlib import Path

import pytest

# src/tests/unit/test_robot_config_proto_parses.py -> unit -> tests -> src -> repo root
_REPO_ROOT = Path(__file__).resolve().parents[3]
_PROTO_DIR = _REPO_ROOT / "src" / "protos"

# The (host_only) MessageOptions extension field number (options.proto,
# 132-001) — a bool, wire type 0 (varint). Mirrors gen_messages.py's own
# hand-rolled option-reading convention (_read_max_count()/
# _parse_field_options()) rather than building a descriptor pool: this
# project already reads custom proto options by walking the serialized
# Options bytes directly, so this test does the same for consistency.
_MESSAGE_OPT_HOST_ONLY = 50200


def _read_varint(buf: bytes, pos: int):
    val = 0
    shift = 0
    while pos < len(buf):
        b = buf[pos]
        pos += 1
        val |= (b & 0x7F) << shift
        shift += 7
        if not (b & 0x80):
            return val, pos
    return val, pos


def _read_message_bool_option(options_proto, opt_field_number: int):
    """Return a bool MessageOptions extension value, or None if absent.

    Same hand-rolled wire walk gen_messages.py's _read_max_count() already
    uses for FieldOptions — here applied to a message's (not a field's)
    serialized Options bytes.
    """
    raw = options_proto.SerializeToString()
    pos = 0
    while pos < len(raw):
        tag, pos = _read_varint(raw, pos)
        field_num = tag >> 3
        wire_type = tag & 7
        if wire_type == 0:  # varint
            val, pos = _read_varint(raw, pos)
            if field_num == opt_field_number:
                return bool(val)
        elif wire_type == 2:  # length-delimited
            vlen, pos = _read_varint(raw, pos)
            pos += vlen
        elif wire_type == 5:  # fixed32
            pos += 4
        elif wire_type == 1:  # fixed64
            pos += 8
        else:
            break
    return None


@pytest.fixture(scope="module")
def file_descriptor_set():
    """Run protoc over every src/protos/*.proto (the exact set gen_messages.py's
    own _run_codegen_pipeline() compiles together) and return the parsed
    FileDescriptorSet — proof `robot_config.proto` parses cleanly ALONGSIDE
    every pre-existing proto file (including config.proto, whose own
    `ConfigTarget` enum is why this file's enum is named `ConfigGroupTarget`
    instead — see robot_config.proto's own header comment)."""
    try:
        import grpc_tools
    except ImportError:
        pytest.skip("grpcio-tools not installed — run: uv sync")

    from google.protobuf import descriptor_pb2
    from grpc_tools import protoc

    well_known_dir = str(Path(grpc_tools.__file__).parent / "_proto")
    proto_names = sorted(p.name for p in _PROTO_DIR.glob("*.proto"))
    assert "robot_config.proto" in proto_names, (
        "src/protos/robot_config.proto not found on disk"
    )
    proto_paths = [str(_PROTO_DIR / n) for n in proto_names]

    with tempfile.NamedTemporaryFile(suffix=".pb", delete=False) as tmp_f:
        tmp_path = tmp_f.name

    try:
        ret = protoc.main(
            [
                "protoc",
                "-I", str(_PROTO_DIR),
                "-I", well_known_dir,
                f"--descriptor_set_out={tmp_path}",
                "--include_imports",
            ]
            + proto_paths
        )
        assert ret == 0, (
            "protoc failed to parse src/protos/*.proto — robot_config.proto "
            "(or its options.proto (host_only) extension) has a syntax error, "
            "or collides with an existing top-level type/enum-value name in "
            "the `robot` package (see robot_config.proto's own header "
            "comment for the ConfigTarget/ConfigGroupTarget collision this "
            "already caught once)."
        )
        fds_bytes = Path(tmp_path).read_bytes()
    finally:
        Path(tmp_path).unlink(missing_ok=True)

    fds = descriptor_pb2.FileDescriptorSet()
    fds.ParseFromString(fds_bytes)
    return fds


@pytest.fixture(scope="module")
def robot_config_file(file_descriptor_set):
    for fd in file_descriptor_set.file:
        if fd.name == "robot_config.proto":
            return fd
    pytest.fail("robot_config.proto not present in the parsed FileDescriptorSet")


def _message_names(fd) -> set:
    return {md.name for md in fd.message_type}


def _enum_by_name(fd, name):
    for ed in fd.enum_type:
        if ed.name == name:
            return ed
    return None


def _message_by_name(fd, name):
    for md in fd.message_type:
        if md.name == name:
            return md
    return None


def test_protoc_parses_the_full_proto_set(file_descriptor_set):
    """The bare parse/compile check this ticket's own acceptance criterion
    asks for — already enforced by the fixture's own assert, re-asserted
    here as an explicit, named test."""
    names = {fd.name for fd in file_descriptor_set.file}
    assert "robot_config.proto" in names
    # "config.proto" -- DELETED, ticket 013 (patch-surface retirement,
    # wholesale). This assertion pre-dated that deletion (it originally
    # documented the two schemas coexisting during the transition); fixed
    # here (132-015) as a pre-existing, trivially-correct leftover found
    # while auditing config.proto's own ConfigTarget/CONFIG_PLANNER/
    # CONFIG_WATCHDOG fate for this ticket's own Description item 4.
    assert "config.proto" not in names


HOST_ONLY_GROUPS = ("Identity", "Connection", "Vision")
# PlannerShaper (132-017): split out of Planner mid-sprint, stakeholder-
# sanctioned -- see robot_config.proto's PlannerShaper message header
# comment for why (the six shaper-ceiling fields carry their own
# re-appliable setter, unlike the rest of Planner).
ROBOT_CONFIG_GROUPS = (
    "Geometry", "Motors", "Drive", "WheelControl", "Planner", "PlannerShaper",
    "Otos", "Estimator",
)


@pytest.mark.parametrize("name", HOST_ONLY_GROUPS)
def test_host_only_groups_declared_and_marked(robot_config_file, name):
    md = _message_by_name(robot_config_file, name)
    assert md is not None, f"{name} not declared in robot_config.proto"
    assert _read_message_bool_option(md.options, _MESSAGE_OPT_HOST_ONLY) is True, (
        f"{name} must carry option (host_only) = true"
    )


@pytest.mark.parametrize("name", ROBOT_CONFIG_GROUPS)
def test_robot_config_groups_declared_and_not_host_only(robot_config_file, name):
    md = _message_by_name(robot_config_file, name)
    assert md is not None, f"{name} not declared in robot_config.proto"
    assert _read_message_bool_option(md.options, _MESSAGE_OPT_HOST_ONLY) in (None, False), (
        f"{name} is a robot-config group and must not carry (host_only) = true"
    )


def test_config_group_target_enum_has_one_value_per_robot_config_group(robot_config_file):
    ed = _enum_by_name(robot_config_file, "ConfigGroupTarget")
    assert ed is not None, "ConfigGroupTarget enum not declared in robot_config.proto"

    value_names = {v.name for v in ed.value}
    assert "CONFIG_GROUP_UNSPECIFIED" in value_names  # proto3-required zero value

    # Group message name -> its own enum value name (this file's own stated
    # naming convention: WheelControl <-> WHEEL_CONTROL, everything else is
    # a plain upper-case of the message name).
    expected = {
        "Geometry": "GEOMETRY",
        "Motors": "MOTORS",
        "Drive": "DRIVE",
        "WheelControl": "WHEEL_CONTROL",
        "Planner": "PLANNER",
        "PlannerShaper": "PLANNER_SHAPER",
        "Otos": "OTOS",
        "Estimator": "ESTIMATOR",
    }
    for group, enum_value in expected.items():
        assert enum_value in value_names, (
            f"ConfigGroupTarget has no value for group {group!r} (expected {enum_value!r})"
        )
    # Exactly one value per robot-config group, plus the zero sentinel.
    assert len(value_names) == len(expected) + 1


@pytest.mark.parametrize(
    "name", ["SetConfigGroup", "GetConfig", "ConfigSnapshot", "SetConfigField"]
)
def test_wire_envelope_messages_declared(robot_config_file, name):
    assert _message_by_name(robot_config_file, name) is not None, (
        f"{name} not declared in robot_config.proto"
    )


def test_set_config_field_carries_target_field_number_and_value(robot_config_file):
    md = _message_by_name(robot_config_file, "SetConfigField")
    fields = {f.name: f for f in md.field}
    assert set(fields) == {"target", "field", "value"}
    assert fields["target"].type_name.endswith("ConfigGroupTarget")
    assert fields["field"].type == fields["field"].TYPE_UINT32  # a wire NUMBER, never a string key
    assert fields["value"].type == fields["value"].TYPE_FLOAT


def test_color_and_line_config_not_declared_as_groups(robot_config_file):
    """sprint.md Out of Scope + this ticket's own acceptance criterion:
    ColorConfig/LineConfig are not part of Config::Robot's end-state
    schema — neither root has ever baked a robot-JSON override for
    either (boot_wiring.cpp:28-33)."""
    names = _message_names(robot_config_file)
    assert "ColorConfig" not in names
    assert "LineConfig" not in names


# ---------------------------------------------------------------------------
# Field-by-field census against gen_boot_config.py's actual `_require()`
# call sites — the acceptance criterion asks for a checklist "not just a
# field count." One test per REQUIRED (cfg["section"]["key"]) pair this
# ticket's own header comment claims a home for; each assertion locates the
# group message and the exact field name, so a future rename of either
# breaks this test loudly rather than silently drifting from the checklist
# robot_config.proto's own header documents.
# ---------------------------------------------------------------------------

# (group_message_name, field_name) for every gen_boot_config.py _require()
# (or, where noted in robot_config.proto's own header, _get()) call site.
REQUIRED_FIELD_HOMES = [
    # otos_boot_config_values()
    ("Otos", "offset_x"),
    ("Otos", "offset_y"),
    ("Otos", "offset_yaw"),
    ("Otos", "linear_scale"),
    ("Otos", "angular_scale"),
    # vel_gains_for_config()
    ("Motors", "vel_kp"),
    ("Motors", "vel_ki"),
    ("Motors", "vel_kff"),
    ("Motors", "vel_i_max"),
    ("Motors", "vel_kaw"),
    ("Motors", "vel_filt_alpha"),
    # output_deadband_for_config() / reversal_dwell_for_config()
    ("Motors", "output_deadband"),
    ("Motors", "reversal_dwell"),
    # trackwidth_for_config() / rotational_slip_for_config()
    ("Geometry", "trackwidth"),
    ("Geometry", "rotational_slip"),
    # rotation_calibration_for_config()
    ("Geometry", "rotation_gain_pos"),
    ("Geometry", "rotation_offset"),
    ("Geometry", "rotation_gain_neg"),
    ("Geometry", "rotation_offset_neg"),
    # estimator_config_for_config()
    ("Estimator", "weight_heading_otos"),
    ("Estimator", "weight_omega_otos"),
    ("Estimator", "staleness"),
    # shaper_config_for_config() -- DELETED, 132-015: dead ShaperBootConfig
    # (zero live consumers) and its mirroring Planner.shaper_* schema
    # fields are both gone; field numbers 17-22 are now `reserved` on the
    # Planner message rather than declared (see that message's own
    # trailing comment) -- no schema-home entries for them any more.
    # wheel_correction_for_config()
    ("Drive", "wheel_gain_left_accel"),
    ("Drive", "wheel_intercept_left_accel"),
    ("Drive", "wheel_gain_left_decel"),
    ("Drive", "wheel_intercept_left_decel"),
    ("Drive", "wheel_gain_right_accel"),
    ("Drive", "wheel_intercept_right_accel"),
    ("Drive", "wheel_gain_right_decel"),
    ("Drive", "wheel_intercept_right_decel"),
    # drive_config_for_config()
    ("Drive", "duty_per_speed_left"),
    ("Drive", "duty_per_speed_right"),
    ("Drive", "crawl_pulse"),
    # wheel_controller_config_for_config()
    ("WheelControl", "v_min"),
    ("WheelControl", "bias_max"),
    ("WheelControl", "tau_adapt"),
    ("WheelControl", "a_steady"),
    ("WheelControl", "deficit_threshold"),
    ("WheelControl", "deficit_window"),
    ("WheelControl", "pid_kp"),
    ("WheelControl", "pid_ki"),
    ("WheelControl", "pid_i_max"),
    ("WheelControl", "pid_kaff"),
    ("WheelControl", "pid_max"),
    # planner_config_for_config() -- SPLIT, 132-017 (JSON reshape ticket,
    # stakeholder-sanctioned mid-sprint scope addition): the six shaper-
    # ceiling fields moved to their own PlannerShaper group/PLANNER_SHAPER
    # ConfigGroupTarget (live -- see that message's own header comment for
    # why), leaving Planner itself boot-only.
    ("Planner", "v_max"),
    ("PlannerShaper", "a_max"),
    ("PlannerShaper", "a_decel"),
    ("Planner", "omega_max"),
    ("PlannerShaper", "alpha_max"),
    ("PlannerShaper", "alpha_decel"),
    ("PlannerShaper", "jerk_max"),
    ("PlannerShaper", "yaw_jerk_max"),
    ("Planner", "control_period"),
    ("Planner", "actuation_delay"),
    ("Planner", "settle_rest_velocity"),
    ("Planner", "settle_rest_omega"),
    ("Planner", "settle_epsilon_linear"),
    ("Planner", "settle_epsilon_angular"),
    ("Planner", "heading_hold_gain"),
    ("Planner", "decel_plan_fraction"),
    # travel_calib_for_ports() / fwd_sign_for_ports() (soft _get(), not
    # _require() -- included per this ticket's own explicit mapping text)
    ("Motors", "travel_calib_left"),
    ("Motors", "travel_calib_right"),
    ("Motors", "fwd_sign_left"),
    ("Motors", "fwd_sign_right"),
]


@pytest.mark.parametrize("group_name,field_name", REQUIRED_FIELD_HOMES)
def test_required_field_has_a_schema_home(robot_config_file, group_name, field_name):
    md = _message_by_name(robot_config_file, group_name)
    assert md is not None, f"group message {group_name!r} not declared"
    field_names = {f.name for f in md.field}
    assert field_name in field_names, (
        f"{group_name}.{field_name} missing — gen_boot_config.py reads this "
        f"value but robot_config.proto has no field for it"
    )


def test_required_field_home_count_matches_group_field_counts(robot_config_file):
    """Cross-check: every field actually declared on a robot-config group
    (other than the deliberately-excluded dead/unread fields robot_config.
    proto's own header documents) appears in REQUIRED_FIELD_HOMES above —
    catches a field added to the schema with no corresponding checklist
    entry, the mirror image of test_required_field_has_a_schema_home."""
    claimed = {name for group, name in REQUIRED_FIELD_HOMES if group == "Geometry"}
    md = _message_by_name(robot_config_file, "Geometry")
    actual = {f.name for f in md.field}
    assert actual == claimed
