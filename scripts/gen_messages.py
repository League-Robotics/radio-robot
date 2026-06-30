#!/usr/bin/env python3
"""Generate source/messages/*.h — C++11 POD headers from proto3 message definitions.

Run:  python3 scripts/gen_messages.py [--dry-run] [--emit-inventory]

Reads protos/*.proto via grpcio-tools (host-only; the device never sees protobuf)
and emits one header per proto file to source/messages/.

Generated code targets CODAL/C++11 with -fno-rtti -fno-exceptions.  No STL
containers, no heap, no exceptions, no RTTI.

Flags
-----
--dry-run        Print what would be written without touching the filesystem.
--emit-inventory Write docs/design/message-inventory.md (traceability table).
"""

import argparse
import os
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
PROTO_DIR  = REPO_ROOT / "protos"
OUT_DIR    = REPO_ROOT / "source" / "messages"
INVENTORY_OUT = REPO_ROOT / "docs" / "design" / "message-inventory.md"

# ---------------------------------------------------------------------------
# Extension field numbers defined in options.proto
# ---------------------------------------------------------------------------
_FIELD_OPT_UNITS     = 50000
_FIELD_OPT_MAX_COUNT = 50001

# Message types that get chainable setters (Command and Config).
_SETTER_TYPES = frozenset([
    "DrivetrainCommand", "MotorCommand", "PlannerCommand",
    "DrivetrainConfig",  "MotorConfig",  "PlannerConfig",
    "LineSensorConfig",  "ColorSensorConfig",
    "GripperConfig",     "PortConfig",
    "GripperCommand",
])

# ---------------------------------------------------------------------------
# Hand-authored field-to-existing-symbol mapping (used by --emit-inventory).
# Format: {(MessageName, field_name): "ExistingSymbol::path"}
# ---------------------------------------------------------------------------
_INVENTORY_MAP: dict = {
    ("DrivetrainState", "fused"):    "ActualState::fused",
    ("DrivetrainState", "encoder"):  "ActualState::encoder",
    ("DrivetrainState", "optical"):  "ActualState::otos",
    ("DrivetrainState", "enc_mm"):   "ActualState::encMm[]",
    ("DrivetrainState", "vel_mms"):  "ActualState::velMms[]",
    ("DrivetrainState", "connected"):"ActualState::connected",
    ("PlannerState",    "mode"):     "DesiredState::mode",
    ("PlannerState",    "body_twist"): "DesiredState::bodyTwist",
    ("PlannerState",    "active"):   "DesiredState::active",
    ("DrivetrainConfig","trackwidth_mm"): "RobotConfig::trackwidthMm",
    ("DrivetrainConfig","vel_gains"): "RobotConfig::{velKp,velKi,velKff,...}",
}

# ---------------------------------------------------------------------------
# protobuf type constants
# ---------------------------------------------------------------------------
try:
    from google.protobuf import descriptor_pb2 as _dpb2
    _TYPE_FLOAT   = _dpb2.FieldDescriptorProto.TYPE_FLOAT
    _TYPE_DOUBLE  = _dpb2.FieldDescriptorProto.TYPE_DOUBLE
    _TYPE_INT32   = _dpb2.FieldDescriptorProto.TYPE_INT32
    _TYPE_INT64   = _dpb2.FieldDescriptorProto.TYPE_INT64
    _TYPE_UINT32  = _dpb2.FieldDescriptorProto.TYPE_UINT32
    _TYPE_UINT64  = _dpb2.FieldDescriptorProto.TYPE_UINT64
    _TYPE_BOOL    = _dpb2.FieldDescriptorProto.TYPE_BOOL
    _TYPE_STRING  = _dpb2.FieldDescriptorProto.TYPE_STRING
    _TYPE_BYTES   = _dpb2.FieldDescriptorProto.TYPE_BYTES
    _TYPE_MESSAGE = _dpb2.FieldDescriptorProto.TYPE_MESSAGE
    _TYPE_ENUM    = _dpb2.FieldDescriptorProto.TYPE_ENUM
    _LABEL_REPEATED = _dpb2.FieldDescriptorProto.LABEL_REPEATED
except ImportError as exc:
    print(f"gen_messages: google.protobuf not found — install grpcio-tools: {exc}",
          file=sys.stderr)
    sys.exit(1)


# ---------------------------------------------------------------------------
# Helper: read a varint from a bytes buffer
# ---------------------------------------------------------------------------
def _read_varint(buf: bytes, pos: int):
    val = 0
    shift = 0
    while pos < len(buf):
        b = buf[pos]; pos += 1
        val |= (b & 0x7f) << shift
        shift += 7
        if not (b & 0x80):
            return val, pos
    return val, pos


def _read_max_count(field) -> int | None:
    """Return the (max_count) option value from a FieldDescriptorProto, or None."""
    raw = field.options.SerializeToString()
    pos = 0
    while pos < len(raw):
        tag, pos = _read_varint(raw, pos)
        field_num = tag >> 3
        wire_type = tag & 7
        if wire_type == 0:   # varint
            val, pos = _read_varint(raw, pos)
            if field_num == _FIELD_OPT_MAX_COUNT:
                return val
        elif wire_type == 2:  # length-delimited
            vlen, pos = _read_varint(raw, pos)
            pos += vlen
        elif wire_type == 5:  # 32-bit fixed
            pos += 4
        elif wire_type == 1:  # 64-bit fixed
            pos += 8
        else:
            break
    return None


# ---------------------------------------------------------------------------
# Proto scalar → C++ type mapping
# ---------------------------------------------------------------------------
def _scalar_cpp_type(field) -> str:
    """Map a proto scalar field type to a C++ type name."""
    t = field.type
    if t == _TYPE_FLOAT:   return "float"
    if t == _TYPE_DOUBLE:  return "double"
    if t == _TYPE_INT32:   return "int32_t"
    if t == _TYPE_INT64:   return "int64_t"
    if t == _TYPE_UINT32:  return "uint32_t"
    if t == _TYPE_UINT64:  return "uint64_t"
    if t == _TYPE_BOOL:    return "bool"
    if t == _TYPE_STRING:  return "char"   # char arrays, size N=64 by default
    if t == _TYPE_BYTES:   return "uint8_t"
    return "uint8_t"  # fallback


def _scalar_default(field) -> str:
    """Return a sensible zero-initialiser literal for a scalar field."""
    t = field.type
    if t == _TYPE_FLOAT:   return "0.0f"
    if t == _TYPE_DOUBLE:  return "0.0"
    if t in (_TYPE_INT32, _TYPE_INT64, _TYPE_UINT32, _TYPE_UINT64): return "0"
    if t == _TYPE_BOOL:    return "false"
    return "0"


def _short_type_name(type_name: str) -> str:
    """Strip .robot. prefix from a message/enum type_name."""
    if type_name.startswith(".robot."):
        return type_name[len(".robot."):]
    if type_name.startswith("."):
        return type_name[1:].replace(".", "::")
    return type_name


def _cpp_field_type(field) -> str:
    """Return the C++ type for a field (scalar, enum, or message)."""
    t = field.type
    if t in (_TYPE_MESSAGE, _TYPE_ENUM):
        return _short_type_name(field.type_name)
    return _scalar_cpp_type(field)


# ---------------------------------------------------------------------------
# Identify oneof classification
# ---------------------------------------------------------------------------
def _classify_oneofs(md):
    """
    Return (real_oneofs, proto3_optional_oneof_indices).

    Proto3 `optional T` fields get a synthetic oneof whose name starts with
    '_'.  These are *not* code-gen'd as union-based oneofs — they map to
    Opt<T> instead.

    Returns:
        real_oneofs: list of (oneof_index, oneof_name, [field_descriptor])
            for real (non-synthetic) oneofs.
        opt_field_indices: set of field oneof_index values that are
            synthetic proto3-optional wrappers.
    """
    oneof_is_synthetic = {}
    for idx, od in enumerate(md.oneof_decl):
        oneof_is_synthetic[idx] = od.name.startswith("_")

    # Group fields by oneof index (real oneofs only)
    real_oneof_fields: dict[int, list] = {}
    for field in md.field:
        if field.HasField("oneof_index"):
            oi = field.oneof_index
            if not oneof_is_synthetic[oi]:
                real_oneof_fields.setdefault(oi, []).append(field)

    real_oneofs = []
    for idx, od in enumerate(md.oneof_decl):
        if not oneof_is_synthetic[idx] and idx in real_oneof_fields:
            real_oneofs.append((idx, od.name, real_oneof_fields[idx]))

    opt_indices = {idx for idx, synth in oneof_is_synthetic.items() if synth}
    return real_oneofs, opt_indices


# ---------------------------------------------------------------------------
# Enum code generation
# ---------------------------------------------------------------------------
def _emit_enum(ed, lines: list[str]) -> None:
    """Emit a C++11 enum class for a proto enum descriptor."""
    lines.append(f"enum class {ed.name} : uint8_t {{")
    for val in ed.value:
        lines.append(f"    {val.name} = {val.number},")
    lines.append("};")
    lines.append("")


# ---------------------------------------------------------------------------
# Message code generation
# ---------------------------------------------------------------------------
def _emit_message(md, want_setters: bool, lines: list[str],
                  all_enums: set[str]) -> None:
    """Emit a C++11 POD struct for a proto message descriptor.

    Rules (from ticket 002 specification):
      - scalar/message fields         → plain members
      - proto3_optional fields        → Opt<T> members
      - repeated with (max_count)=N  → T name[N]; uint8_t name_count = 0;
      - real oneof                    → KindName enum + union
      - setters                       → only if want_setters (Command/Config)
    """
    real_oneofs, opt_indices = _classify_oneofs(md)
    real_oneof_field_indices: set[int] = set()
    for oi, _name, fields in real_oneofs:
        for f in fields:
            real_oneof_field_indices.add(f.number)

    struct_name = md.name

    lines.append(f"// {struct_name}")
    lines.append(f"struct {struct_name} {{")

    # --- Emit real oneof kinds first (before the fields that reference them) ---
    for oi, oneof_name, oneof_fields in real_oneofs:
        kind_name = f"{_cap_camel(oneof_name)}Kind"
        lines.append(f"    enum class {kind_name} : uint8_t {{")
        lines.append(f"        NONE = 0,")
        for idx_f, f in enumerate(oneof_fields, 1):
            lines.append(f"        {f.name.upper()} = {idx_f},")
        lines.append("    };")
        lines.append(f"    {kind_name} {oneof_name}_kind = {kind_name}::NONE;")
        lines.append(f"    union {{")
        for f in oneof_fields:
            ft = _cpp_field_type(f)
            if f.type == _TYPE_BOOL:
                # bools in unions need padding to avoid UB on some compilers;
                # use uint8_t to be safe in C++11
                lines.append(f"        uint8_t {f.name}_v;  // bool")
            elif f.type == _TYPE_STRING:
                lines.append(f"        char {f.name}[64];")
            else:
                lines.append(f"        {ft} {f.name};")
        lines.append(f"    }} {oneof_name} = {{}};")
        lines.append("")

    # --- Emit regular fields ---
    for field in md.field:
        fname = field.name
        is_repeated = field.label == _LABEL_REPEATED
        is_opt      = field.HasField("oneof_index") and field.oneof_index in opt_indices
        in_real_oneof = field.HasField("oneof_index") and (
            field.oneof_index not in opt_indices
        )

        # Real oneof fields were emitted above as part of the union
        if in_real_oneof:
            continue

        if is_repeated:
            max_n = _read_max_count(field)
            if max_n is None:
                print(f"  WARNING: repeated field {struct_name}.{fname} has no "
                      f"(max_count) option; defaulting to 8", file=sys.stderr)
                max_n = 8
            ft = _cpp_field_type(field)
            if ft == "bool":
                # bool arrays in C++11 embedded: use uint8_t for ABI clarity
                ft_arr = "uint8_t"
                comment = "  // bool[]"
            else:
                ft_arr = ft
                comment = ""
            # Use trailing _ on data member to avoid collision with getter method.
            if field.type == _TYPE_STRING:
                # repeated string → not expected in our protos; treat as char[][64]
                lines.append(f"    char {fname}_[{max_n}][64] = {{}};")
            else:
                lines.append(f"    {ft_arr} {fname}_[{max_n}] = {{}};{comment}")
            lines.append(f"    uint8_t {fname}_count = 0;")

        elif is_opt:
            ft = _cpp_field_type(field)
            if field.type == _TYPE_STRING:
                # optional string → Opt<char[64]> is awkward; use Opt<char*> stub
                # but we have no heap; use a fixed array with a has flag
                lines.append(f"    bool {fname}_has = false;")
                lines.append(f"    char {fname}[64] = {{}};")
            else:
                lines.append(f"    Opt<{ft}> {fname} = {{}};")

        elif field.type == _TYPE_MESSAGE:
            ft = _short_type_name(field.type_name)
            lines.append(f"    {ft} {fname} = {{}};")

        elif field.type == _TYPE_ENUM:
            ft = _short_type_name(field.type_name)
            # emit with default value of 0 cast to the enum type
            lines.append(f"    {ft} {fname} = static_cast<{ft}>(0);")

        elif field.type == _TYPE_STRING:
            lines.append(f"    char {fname}[64] = {{}};")

        else:
            # plain scalar
            default = _scalar_default(field)
            ft = _scalar_cpp_type(field)
            lines.append(f"    {ft} {fname} = {default};")

    # --- Getters ---
    lines.append("")
    lines.append("    // --- getters ---")
    for oi, oneof_name, oneof_fields in real_oneofs:
        kind_name = f"{_cap_camel(oneof_name)}Kind"
        lines.append(f"    {kind_name} get_{oneof_name}_kind() const"
                     f" {{ return {oneof_name}_kind; }}")

    for field in md.field:
        fname = field.name
        is_repeated = field.label == _LABEL_REPEATED
        is_opt      = field.HasField("oneof_index") and field.oneof_index in opt_indices
        in_real_oneof = field.HasField("oneof_index") and (
            field.oneof_index not in opt_indices
        )
        if in_real_oneof:
            continue  # access via union directly

        if is_repeated:
            max_n = _read_max_count(field) or 8
            ft = _cpp_field_type(field)
            if ft == "bool":
                ft_arr = "uint8_t"
            else:
                ft_arr = ft
            if field.type == _TYPE_STRING:
                pass  # skip getter for char[][] — too messy
            else:
                # Data member is {fname}_ to avoid collision with getter method.
                lines.append(f"    const {ft_arr}* {fname}() const {{ return {fname}_; }}")
                lines.append(f"    uint8_t {fname}_count_val() const"
                             f" {{ return {fname}_count; }}")
        elif is_opt:
            if field.type == _TYPE_STRING:
                lines.append(f"    bool has_{fname}() const {{ return {fname}_has; }}")
                lines.append(f"    const char* get_{fname}() const {{ return {fname}; }}")
            else:
                ft = _cpp_field_type(field)
                lines.append(f"    const Opt<{ft}>& get_{fname}() const"
                             f" {{ return {fname}; }}")
        elif field.type == _TYPE_MESSAGE:
            ft = _short_type_name(field.type_name)
            lines.append(f"    const {ft}& get_{fname}() const {{ return {fname}; }}")
        elif field.type == _TYPE_ENUM:
            ft = _short_type_name(field.type_name)
            lines.append(f"    {ft} get_{fname}() const {{ return {fname}; }}")
        elif field.type == _TYPE_STRING:
            lines.append(f"    const char* get_{fname}() const {{ return {fname}; }}")
        else:
            ft = _scalar_cpp_type(field)
            lines.append(f"    {ft} get_{fname}() const {{ return {fname}; }}")

    # --- Setters (Command/Config types only) ---
    if want_setters:
        lines.append("")
        lines.append("    // --- chainable setters (Command/Config only) ---")
        for oi, oneof_name, oneof_fields in real_oneofs:
            kind_name = f"{_cap_camel(oneof_name)}Kind"
            for f in oneof_fields:
                ft = _cpp_field_type(f)
                cap = _cap_camel(f.name)
                if f.type == _TYPE_BOOL:
                    lines.append(f"    {struct_name}& set{cap}(bool v) {{")
                    lines.append(f"        {oneof_name}_kind = {kind_name}::{f.name.upper()};")
                    lines.append(f"        {oneof_name}.{f.name}_v = v ? 1 : 0;")
                    lines.append(f"        return *this;")
                    lines.append("    }")
                elif f.type == _TYPE_STRING:
                    lines.append(f"    // set{cap}: string oneof arm — use"
                                 f" {oneof_name}.{f.name} directly")
                else:
                    if ft in ("float", "double") or "int" in ft or ft == "uint8_t":
                        lines.append(f"    {struct_name}& set{cap}({ft} v) {{")
                        lines.append(f"        {oneof_name}_kind = {kind_name}::{f.name.upper()};")
                        lines.append(f"        {oneof_name}.{f.name} = v;")
                        lines.append(f"        return *this;")
                        lines.append("    }")
                    else:
                        # message type
                        lines.append(f"    {struct_name}& set{cap}(const {ft}& v) {{")
                        lines.append(f"        {oneof_name}_kind = {kind_name}::{f.name.upper()};")
                        lines.append(f"        {oneof_name}.{f.name} = v;")
                        lines.append(f"        return *this;")
                        lines.append("    }")

        for field in md.field:
            fname = field.name
            is_repeated = field.label == _LABEL_REPEATED
            is_opt      = field.HasField("oneof_index") and field.oneof_index in opt_indices
            in_real_oneof = field.HasField("oneof_index") and (
                field.oneof_index not in opt_indices
            )
            if in_real_oneof:
                continue

            cap = _cap_camel(fname)

            if is_repeated:
                # Setters for repeated are tricky — just expose a clear helper
                max_n = _read_max_count(field) or 8
                ft = _cpp_field_type(field)
                if ft == "bool":
                    ft_s = "uint8_t"
                else:
                    ft_s = ft
                if field.type == _TYPE_STRING:
                    pass
                else:
                    lines.append(f"    {struct_name}& clear{cap}()"
                                 f" {{ {fname}_count = 0; return *this; }}")
            elif is_opt:
                if field.type == _TYPE_STRING:
                    lines.append(f"    // set{cap}: optional string — set"
                                 f" {fname}_has and {fname} directly")
                else:
                    ft = _cpp_field_type(field)
                    lines.append(f"    {struct_name}& set{cap}({ft} v) {{")
                    lines.append(f"        {fname}.has = true; {fname}.val = v;")
                    lines.append(f"        return *this;")
                    lines.append("    }")
            elif field.type == _TYPE_MESSAGE:
                ft = _short_type_name(field.type_name)
                lines.append(f"    {struct_name}& set{cap}(const {ft}& v)"
                             f" {{ {fname} = v; return *this; }}")
            elif field.type == _TYPE_ENUM:
                ft = _short_type_name(field.type_name)
                lines.append(f"    {struct_name}& set{cap}({ft} v)"
                             f" {{ {fname} = v; return *this; }}")
            elif field.type == _TYPE_STRING:
                lines.append(f"    // set{cap}: set {fname}[] directly (char array)")
            elif field.type == _TYPE_BOOL:
                lines.append(f"    {struct_name}& set{cap}(bool v)"
                             f" {{ {fname} = v; return *this; }}")
            else:
                ft = _scalar_cpp_type(field)
                lines.append(f"    {struct_name}& set{cap}({ft} v)"
                             f" {{ {fname} = v; return *this; }}")

    lines.append("};")
    lines.append("")


def _cap_camel(name: str) -> str:
    """Convert snake_case to CapCamelCase for setter names."""
    return "".join(w.capitalize() for w in name.split("_"))


# ---------------------------------------------------------------------------
# File-level code generation
# ---------------------------------------------------------------------------
_BANNER = """\
// AUTO-GENERATED — do not edit by hand.
// Regenerated by scripts/gen_messages.py before each firmware build.
// Source: protos/{proto_name}
#pragma once
"""

_COMMON_PREAMBLE = """\
#include <stdint.h>

// Opt<T> — nullable wrapper for proto3 optional fields.
// Replaces std::optional<T> (which requires RTTI / exceptions).
// Target: CODAL C++11, -fno-rtti -fno-exceptions, no heap.
template<class T>
struct Opt { bool has = false; T val{}; };
"""

_OTHER_INCLUDE = '#include "messages/common.h"\n'


def _emit_file(fd, file_messages: dict, file_enums: dict,
               all_enums: set[str]) -> str:
    """Emit the full content of one generated header."""
    lines: list[str] = []
    proto_name = fd.name

    # Banner + pragma once
    lines.append("// AUTO-GENERATED — do not edit by hand.")
    lines.append("// Regenerated by scripts/gen_messages.py before each firmware build.")
    lines.append(f"// Source: protos/{proto_name}")
    lines.append("#pragma once")
    lines.append("")

    is_common = (proto_name == "common.proto")

    if is_common:
        lines.append(_COMMON_PREAMBLE)
    else:
        lines.append(_OTHER_INCLUDE)
        lines.append("")

    # Emit top-level enums in this file
    for ed in fd.enum_type:
        _emit_enum(ed, lines)

    # Emit messages in this file
    for md in fd.message_type:
        want_setters = md.name in _SETTER_TYPES
        _emit_message(md, want_setters, lines, all_enums)

    return "\n".join(lines) + "\n"


def _emit_bridges_header() -> str:
    """Emit source/messages/bridges.h — static_assert compatibility checks."""
    return """\
// AUTO-GENERATED — do not edit by hand.
// Regenerated by scripts/gen_messages.py before each firmware build.
//
// bridges.h — compile-time compatibility checks between the generated
// message headers (source/messages/*.h) and the existing HAL types.
//
// Usage in firmware TUs:
//   Include this file AFTER the subsystem message headers. It assumes the
//   generated message types are already visible (via messages/common.h etc.)
//   and checks size/layout compatibility with hal/capability/Pose2D.h types.
//
// At ticket 002 the checks are trivially true; they bind tighter in ticket 003
// once the naming is resolved and full static_assert coverage is added.
#pragma once
#include "hal/capability/Pose2D.h"

// The HAL Pose2D struct must remain 3 floats (x, y, h).
static_assert(sizeof(::Pose2D) == sizeof(float) * 3,
              "HAL Pose2D must be 3 floats {x,y,h} — check hal/capability/Pose2D.h");

// The HAL BodyTwist3 struct must remain 3 floats (vx, vy, omega).
static_assert(sizeof(::BodyTwist3) == sizeof(float) * 3,
              "HAL BodyTwist3 must be 3 floats {vx,vy,omega} — check hal/capability/Pose2D.h");

// TODO (ticket 003): add cross-asserts between generated messages/common.h
// Pose2D and the HAL Pose2D once the naming conflict is resolved.
"""


# ---------------------------------------------------------------------------
# Inventory emission
# ---------------------------------------------------------------------------
def _emit_inventory(file_descriptors) -> str:
    """Generate docs/design/message-inventory.md."""
    rows = []
    for fd in file_descriptors:
        if fd.name.startswith("google"):
            continue
        for md in fd.message_type:
            for field in md.field:
                cpp_type = _cpp_field_type(field)
                if field.type in (_TYPE_MESSAGE, _TYPE_ENUM):
                    cpp_type = _short_type_name(field.type_name)
                if field.proto3_optional:
                    cpp_type = f"Opt<{cpp_type}>"
                elif field.label == _LABEL_REPEATED:
                    max_n = _read_max_count(field) or "?"
                    cpp_type = f"{cpp_type}[{max_n}]"
                existing = _INVENTORY_MAP.get((md.name, field.name), "")
                rows.append((fd.name, md.name, field.name, cpp_type, existing))

    lines = [
        "<!-- AUTO-GENERATED by scripts/gen_messages.py --emit-inventory -->",
        "<!-- Do not edit by hand. -->",
        "# Message Inventory",
        "",
        "| Proto file | Message | Field | C++ type | Maps to existing |",
        "|---|---|---|---|---|",
    ]
    for proto_file, msg, field, cpp_type, existing in rows:
        lines.append(f"| {proto_file} | {msg} | {field} | {cpp_type} | {existing} |")
    lines.append("")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------
def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Generate C++11 POD headers from proto3 message definitions."
    )
    parser.add_argument("--dry-run", action="store_true",
                        help="Print what would be written; do not touch files.")
    parser.add_argument("--emit-inventory", action="store_true",
                        help="Also write docs/design/message-inventory.md.")
    args = parser.parse_args(argv)

    # ------------------------------------------------------------------
    # Locate grpcio-tools _proto directory for well-known imports
    # ------------------------------------------------------------------
    try:
        import grpc_tools
    except ImportError:
        print("gen_messages: grpcio-tools is not installed.\n"
              "  Run: uv sync  (grpcio-tools is in the 'codegen' dependency group)",
              file=sys.stderr)
        return 1

    well_known_dir = str(Path(grpc_tools.__file__).parent / "_proto")

    # ------------------------------------------------------------------
    # Collect all proto files (deterministic order)
    # ------------------------------------------------------------------
    proto_names = sorted(p.name for p in PROTO_DIR.glob("*.proto"))
    proto_paths = [str(PROTO_DIR / n) for n in proto_names]

    # ------------------------------------------------------------------
    # Run protoc to get a FileDescriptorSet
    # ------------------------------------------------------------------
    from grpc_tools import protoc
    from google.protobuf import descriptor_pb2

    with tempfile.NamedTemporaryFile(suffix=".pb", delete=False) as tmp_f:
        tmp_path = tmp_f.name

    try:
        ret = protoc.main([
            "protoc",
            "-I", str(PROTO_DIR),
            "-I", well_known_dir,
            f"--descriptor_set_out={tmp_path}",
            "--include_imports",
        ] + proto_paths)

        if ret != 0:
            print("gen_messages: protoc failed — check proto syntax.", file=sys.stderr)
            return 1

        fds_bytes = Path(tmp_path).read_bytes()
    finally:
        try:
            os.unlink(tmp_path)
        except OSError:
            pass

    fds = descriptor_pb2.FileDescriptorSet()
    fds.ParseFromString(fds_bytes)

    # ------------------------------------------------------------------
    # Build index: file_name → FileDescriptorProto
    # Collect all enum names for type resolution
    # ------------------------------------------------------------------
    all_enums: set[str] = set()
    file_map: dict[str, object] = {}
    for fd in fds.file:
        file_map[fd.name] = fd
        for ed in fd.enum_type:
            all_enums.add(ed.name)
        for md in fd.message_type:
            for ed in md.enum_type:
                all_enums.add(ed.name)

    # ------------------------------------------------------------------
    # Emit one header per proto file (skip options.proto — no messages)
    # ------------------------------------------------------------------
    emit_order = [n for n in proto_names if n != "options.proto"]

    outputs: dict[str, str] = {}
    for proto_name in emit_order:
        fd = file_map.get(proto_name)
        if fd is None:
            print(f"gen_messages: WARNING — {proto_name} not found in descriptor set",
                  file=sys.stderr)
            continue
        header_name = proto_name.replace(".proto", ".h")
        content = _emit_file(fd, {}, {}, all_enums)
        outputs[header_name] = content

    # bridges.h is hand-authored in this generator (not from a proto file)
    outputs["bridges.h"] = _emit_bridges_header()

    # ------------------------------------------------------------------
    # Write or dry-run
    # ------------------------------------------------------------------
    if not args.dry_run:
        OUT_DIR.mkdir(parents=True, exist_ok=True)

    for header_name, content in sorted(outputs.items()):
        out_path = OUT_DIR / header_name
        if args.dry_run:
            print(f"[dry-run] would write {out_path.relative_to(REPO_ROOT)}")
            print("  first 5 lines:")
            for line in content.splitlines()[:5]:
                print(f"    {line}")
        else:
            out_path.write_text(content)
            print(f"gen_messages: wrote {out_path.relative_to(REPO_ROOT)}",
                  file=sys.stderr)

    # ------------------------------------------------------------------
    # Optional inventory
    # ------------------------------------------------------------------
    if args.emit_inventory:
        inv_content = _emit_inventory(list(fds.file))
        if args.dry_run:
            print(f"[dry-run] would write {INVENTORY_OUT.relative_to(REPO_ROOT)}")
        else:
            INVENTORY_OUT.parent.mkdir(parents=True, exist_ok=True)
            INVENTORY_OUT.write_text(inv_content)
            print(f"gen_messages: wrote {INVENTORY_OUT.relative_to(REPO_ROOT)}",
                  file=sys.stderr)

    return 0


if __name__ == "__main__":
    sys.exit(main())
