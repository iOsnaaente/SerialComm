#!/usr/bin/env python3

"""
SerialComm IDL Generator

Features:
    - Struct/Event/Request/Mission/Enum parsing
    - Automatic dependency resolution
    - Automatic C++ serializer generation
    - Automatic Python pack/unpack generation
    - DynamicArray support
    - Little/Big endian metadata
    - Duplicate ID detection
    - Max serialized size generation
    - Reflection metadata generation
    - Runtime serialization helpers generation
    - Enum class generation (C++ enum class + Python IntEnum)

Author:
    Bruno Gabriel Flores Sampaio
"""

import re
import json
import shutil
import argparse

from pathlib import Path


# =============================================================================
# BUILTIN TYPES
# =============================================================================

CPP_TYPE_MAP = {
    "bool": "bool",

    "int8": "int8_t",
    "uint8": "uint8_t",

    "int16": "int16_t",
    "uint16": "uint16_t",

    "int32": "int32_t",
    "uint32": "uint32_t",

    "float32": "float",
    "float64": "double",

    "string": "SerialCommDynamicString"
}


TYPE_SIZE_MAP = {
    "bool": 1,

    "int8": 1,
    "uint8": 1,

    "int16": 2,
    "uint16": 2,

    "int32": 4,
    "uint32": 4,

    "float32": 4,
    "float64": 8,
}


# Python struct format character per IDL primitive type
PYTHON_FMT_MAP = {
    "bool":    "?",
    "int8":    "b",
    "uint8":   "B",
    "int16":   "h",
    "uint16":  "H",
    "int32":   "i",
    "uint32":  "I",
    "float32": "f",
    "float64": "d",
}

# Wire byte size per IDL primitive type
PYTHON_SIZE_MAP = {
    "bool":    1,
    "int8":    1,
    "uint8":   1,
    "int16":   2,
    "uint16":  2,
    "int32":   4,
    "uint32":  4,
    "float32": 4,
    "float64": 8,
}


USED_IDS = {}
MANIFEST = []

# Enum types registered during .enum processing; keyed by PascalCase name.
# Value: {"base": "uint8", "values": {"IDLE": 0, ...}}
KNOWN_ENUMS = {}


def make_manifest_entry(name: str, msg_type: str, metadata: dict) -> dict:
    """Build a rich MANIFEST entry from a parsed metadata dict."""
    entry = {
        "name":          name,
        "type":          msg_type,
        "id":            metadata.get("id", None),
        "delivery_mode": metadata.get("delivery_mode", "BEST_EFFORT"),
        "deprecated":    bool(metadata.get("deprecated", False)),
        "retain":        bool(metadata.get("retain", False)),
    }
    if "max_rate_hz" in metadata:
        entry["max_rate_hz"] = float(metadata["max_rate_hz"])
    if "timeout_ms" in metadata:
        entry["timeout_ms"] = int(metadata["timeout_ms"])
    if "version" in metadata:
        entry["version"] = int(metadata["version"])
    return entry


# ============================================================================
# GENERATED SERIALIZER REGISTRY
# ============================================================================
generated_serializer_includes = []

# Python modules to import in __init__.py: list of (snake_name, symbols)
generated_python_modules = []


# =============================================================================
# HELPERS
# =============================================================================
def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def snake_case(name: str):

    s1 = re.sub(
        r"(.)([A-Z][a-z]+)",
        r"\1_\2",
        name
    )

    s2 = re.sub(
        r"([a-z0-9])([A-Z])",
        r"\1_\2",
        s1
    ).lower()

    # Collapse runs of underscores that arise from mixed PascalCase+underscore
    # inputs such as "SetMotor_Request" → "set_motor__request" → "set_motor_request"
    return re.sub(r"_+", "_", s2)


def pascal_case(name: str):

    # IDL identifiers (file stems and field type references) are written
    # in PascalCase already (RobotState, SetPose, DefaultReturn, ...), not
    # snake_case. `str.capitalize()` would lowercase the rest of each
    # segment and mangle them ("RobotState" -> "Robotstate"), breaking the
    # link between the IDL name, the generated struct name and any
    # hand-written code that references it. Upper-casing only the first
    # character of each underscore-separated segment keeps snake_case
    # inputs working ("robot_state" -> "RobotState") while leaving
    # already-PascalCase input untouched and the function idempotent.
    return "".join(
        x[:1].upper() + x[1:]
        for x in name.split("_")
        if x
    )


def read_lines(path: Path):

    with open(path, "r", encoding="utf-8") as f:

        return [
            line.strip()
            for line in f.readlines()
            if line.strip() and
            not line.strip().startswith("#")
        ]


# =============================================================================
# TYPE HELPERS
# =============================================================================

def extract_base_type(type_name: str):

    base = type_name

    base = re.sub(r"\[\]", "", base)
    base = re.sub(r"\[\d+\]", "", base)
    base = re.sub(r"<=\d+", "", base)

    return base


def is_custom_type(type_name: str):

    base = extract_base_type(type_name)

    return base not in CPP_TYPE_MAP


def is_dynamic_array(type_name: str):

    # Matches both plain dynamic arrays ("T[]") and bounded arrays
    # ("T<=N[]"): both are stored and (de)serialized as
    # SerialCommDynamicArray<T> at runtime, so callers that branch on
    # "does this field use dynamic-array storage?" should treat them
    # identically. Use is_bounded_array() when the declared bound
    # itself matters (e.g. tighter MAX_SERIALIZED_SIZE estimates).
    return re.match(r".+\[\]", type_name) is not None


def is_bounded_array(type_name: str):

    return re.match(r".+<=\d+\[\]", type_name) is not None


def bounded_array_max_count(type_name: str):

    match = re.search(r"<=(\d+)\[\]", type_name)

    if not match:
        return None

    return int(match.group(1))


def is_fixed_array(type_name: str):

    return re.match(r".+\[\d+\]", type_name) is not None


def fixed_array_size(type_name: str):

    match = re.match(r".+\[(\d+)\]", type_name)

    if not match:
        return None

    return int(match.group(1))


def convert_type(type_name: str):

    # FIXED ARRAY: "T[N]"
    fixed = re.match(r"(.+)\[(\d+)\]", type_name)

    if fixed:

        base = convert_type(fixed.group(1))
        size = fixed.group(2)

        return f"std::array<{base}, {size}>"

    # BOUNDED ARRAY: "T<=N[]" — checked before the plain dynamic-array
    # pattern below, since "(.+)\[\]" would otherwise greedily match the
    # "<=N" prefix as part of the base type. Bounded arrays still carry
    # their elements through SerialCommDynamicArray<T> at runtime (there
    # is no separate fixed-capacity container); the declared bound is
    # only used to tighten MAX_SERIALIZED_SIZE estimates (see
    # field_max_size()).
    bounded = re.match(r"(.+)<=\d+\[\]", type_name)

    if bounded:

        base = convert_type(bounded.group(1))

        return f"SerialCommDynamicArray<{base}>"

    # DYNAMIC ARRAY: "T[]"
    dynamic = re.match(r"(.+)\[\]", type_name)

    if dynamic:

        base = convert_type(dynamic.group(1))

        return f"SerialCommDynamicArray<{base}>"

    return CPP_TYPE_MAP.get(
        type_name,
        pascal_case(type_name)
    )


# =============================================================================
# FIELD PARSER
# =============================================================================

FIELD_REGEX = re.compile(
    r"([a-zA-Z0-9_<>\[\]=]+)\s+([a-zA-Z0-9_]+)"
)


def parse_field(line: str):

    match = FIELD_REGEX.match(line)

    if not match:
        return None

    return {
        "type": match.group(1),
        "name": match.group(2)
    }


# =============================================================================
# METADATA
# =============================================================================

def parse_metadata(lines):

    metadata = {}
    filtered = []

    for line in lines:

        if line.startswith("@"):

            tokens = line.split()

            key = tokens[0][1:]

            value = (
                " ".join(tokens[1:])
                if len(tokens) > 1
                else True
            )

            metadata[key] = value

        else:
            filtered.append(line)

    if "big" in metadata:
        metadata["endian"] = "big"
    else:
        metadata["endian"] = "little"

    if "reliable" in metadata:
        metadata["delivery_mode"] = "RELIABLE"
    else:
        metadata["delivery_mode"] = "BEST_EFFORT"

    # Normalise flag-only decorators to booleans
    metadata.setdefault("retain",     False)
    metadata.setdefault("deprecated", False)
    if metadata.get("retain")     is True: metadata["retain"]     = True
    if metadata.get("deprecated") is True: metadata["deprecated"] = True

    return metadata, filtered


# =============================================================================
# ID VALIDATION
# =============================================================================

def validate_metadata_ids(name, metadata):

    if "id" not in metadata:
        return

    packet_id = metadata["id"]

    if packet_id in USED_IDS:

        raise RuntimeError(
            f"[ERROR] duplicated ID {packet_id} "
            f"between {name} and {USED_IDS[packet_id]}"
        )

    USED_IDS[packet_id] = name


# =============================================================================
# DEPENDENCIES
# =============================================================================

def collect_dependencies(fields):

    deps = set()

    for field in fields:

        field_type = field["type"]

        if is_custom_type(field_type):

            deps.add(
                extract_base_type(field_type)
            )

    return deps


# =============================================================================
# SERIALIZED SIZE
# =============================================================================

def field_max_size(field):

    field_type = field["type"]

    base = extract_base_type(field_type)

    # strings are dynamic
    if base == "string":
        return "0 /* dynamic */"

    # ENUM TYPE: fixed size based on underlying base type
    if base in KNOWN_ENUMS:
        enum_base = KNOWN_ENUMS[base]["base"]
        elem_size = TYPE_SIZE_MAP.get(enum_base, 1)
        if is_fixed_array(field_type):
            count = fixed_array_size(field_type)
            return elem_size * count
        if is_dynamic_array(field_type):
            return "0 /* dynamic */"
        return elem_size

    # FIXED ARRAY
    if is_fixed_array(field_type):

        count = fixed_array_size(field_type)

        if base in TYPE_SIZE_MAP:
            return TYPE_SIZE_MAP[base] * count

        base_pascal = pascal_case(base)
        return f"({base_pascal}::MAX_SERIALIZED_SIZE * {count})"

    # BOUNDED ARRAY: stored as SerialCommDynamicArray<T> (so its size is
    # technically variable), but the declared "<=N" cap lets us emit a
    # real worst-case estimate instead of giving up with "0 /* dynamic */":
    # 2-byte length prefix (see SerialCommBufferWriter::write_dynamic_array)
    # plus N elements at their max element size.
    if is_bounded_array(field_type):

        count = bounded_array_max_count(field_type)
        element_size = (
            str(TYPE_SIZE_MAP[base]) if base in TYPE_SIZE_MAP
            else f"{pascal_case(base)}::MAX_SERIALIZED_SIZE"
        )
        return f"(sizeof(uint16_t) + ({element_size}) * {count})"

    # DYNAMIC ARRAY (unbounded): true worst case is whatever fits in the
    # remaining payload, so it cannot contribute a compile-time constant.
    if is_dynamic_array(field_type):

        return "0 /* dynamic */"

    # PRIMITIVE
    if base in TYPE_SIZE_MAP:

        return TYPE_SIZE_MAP[base]

    # CUSTOM STRUCT
    base_pascal = pascal_case(base)
    return f"{base_pascal}::MAX_SERIALIZED_SIZE"


# =============================================================================
# HEADER GENERATOR
# =============================================================================

def generate_header(dependencies):

    out = []

    out.append("#pragma once")
    out.append("")

    out.append("#include <stdint.h>")
    out.append("#include <stddef.h>")
    out.append("#include <array>")
    out.append("")

    out.append('#include "serial_comm/common/serial_comm_types.hpp"')
    out.append("")

    for dep in sorted(dependencies):
        # Enum types get their own include directory
        if dep in KNOWN_ENUMS:
            out.append(
                f'#include "serial_comm/generated/enum/{snake_case(dep)}.hpp"'
            )
        else:
            out.append(
                f'#include "serial_comm/generated/struct/{snake_case(dep)}.hpp"'
            )

    out.append("")

    return "\n".join(out)


# =============================================================================
# STRUCT GENERATOR
# =============================================================================

def generate_struct(name, fields, metadata, delivery_mode=None):
    out = []

    # @deprecated: warn on every use of the generated type
    if metadata.get("deprecated"):
        reason = f"SerialComm IDL: {name} is @deprecated — migrate to a newer version"
        out.append(f'[[deprecated("{reason}")]]')

    out.append(f"struct {name} {{")
    for field in fields:
        cpp_type = convert_type(field["type"])
        out.append(
            f"    {cpp_type} {field['name']};"
        )
    out.append("")

    # MAX SIZE
    out.append("    static constexpr size_t MAX_SERIALIZED_SIZE =")
    total = []
    for field in fields:
        total.append(str(field_max_size(field)))
    if total:
        out.append(
            "        " + " + ".join(total) + ";"
        )
    else:
        out.append("        0;")

    # DELIVERY MODE (only for top-level message types, not embedded structs)
    if delivery_mode is not None:
        out.append(
            f"    static constexpr DeliveryMode DELIVERY_MODE = "
            f"DeliveryMode::{delivery_mode};"
        )

    # @timeout_ms N — per-type call_service timeout (read by serial_comm_timeout_ms<T>())
    if "timeout_ms" in metadata:
        out.append(
            f"    static constexpr uint32_t TIMEOUT_MS = "
            f"{metadata['timeout_ms']};"
        )

    # @version N — IDL schema version (informational; read via T::SCHEMA_VERSION)
    if "version" in metadata:
        out.append(
            f"    static constexpr uint8_t SCHEMA_VERSION = "
            f"{metadata['version']};"
        )

    # @retain — last-value cache flag (read by serial_comm_retain<T>())
    if metadata.get("retain"):
        out.append(
            "    static constexpr bool RETAIN = true;"
        )

    # @max_rate_hz N — enforce publish rate in Python client and runtime guards
    if "max_rate_hz" in metadata:
        rate_val = float(metadata["max_rate_hz"])
        rate_str = f"{rate_val:.6g}"          # e.g. "33" → "33"
        if "." not in rate_str and "e" not in rate_str:
            rate_str += ".0"                  # ensure valid C++ float literal
        out.append(
            f"    static constexpr float MAX_RATE_HZ = {rate_str}f;"
        )

    out.append("};")

    # ID
    if "id" in metadata:
        out.append("")
        out.append(
            f"static constexpr uint16_t "
            f"{name.upper()}_ID = {metadata['id']};"
        )
    return "\n".join(out)


# =============================================================================
# SERIALIZER GENERATOR
# =============================================================================
def generate_serializer(name, fields, metadata, include_path=None):
    endian = metadata["endian"]
    out = []
    out.append("#pragma once")
    out.append("")
    if include_path:
        out.append(f'#include "{include_path}"')
    else:
        out.append(
            f'#include "serial_comm/generated/struct/{snake_case(name)}.hpp"'
        )
    out.append('#include "serial_comm/common/serial_comm_serializer_base.hpp"')
    out.append('#include "serial_comm/common/serial_comm_buffer_writer.hpp"')
    out.append('#include "serial_comm/common/serial_comm_buffer_reader.hpp"')
    out.append("")
    out.append("template<>")
    out.append(f"struct SerialCommSerializer<{name}> {{")
    # SERIALIZE
    out.append("")
    out.append("    static bool serialize(")
    out.append(f"        const {name}& msg,")
    out.append("        uint8_t* buffer,")
    out.append("        size_t buffer_size,")
    out.append("        size_t& serialized_size")
    out.append("    ) {")
    out.append("")
    out.append(
        f"        SerialCommBufferWriter writer(buffer, buffer_size, "
        f'SerialCommBufferEndian::{endian.upper()});'
    )
    for field in fields:
        field_name = field["name"]
        out.append("")
        if is_dynamic_array(field["type"]):
            out.append(
                f"        if(!writer.write_dynamic_array(msg.{field_name})) "
                f"return false;"
            )
        else:
            out.append(
                f"        if(!writer.write(msg.{field_name})) "
                f"return false;"
            )
    out.append("")
    out.append("        serialized_size = writer.offset();")
    out.append("        return true;")
    out.append("    }")
    # DESERIALIZE
    out.append("")
    out.append("    static bool deserialize(")
    out.append("        const uint8_t* buffer,")
    out.append("        size_t buffer_size,")
    out.append(f"        {name}& msg,")
    out.append("        size_t* consumed_size = nullptr")
    out.append("    ) {")
    out.append("")
    out.append(
        f"        SerialCommBufferReader reader(buffer, buffer_size, "
        f'SerialCommBufferEndian::{endian.upper()});'
    )

    for field in fields:
        field_name = field["name"]
        out.append("")
        if is_dynamic_array(field["type"]):
            out.append(
                f"        if(!reader.read_dynamic_array(msg.{field_name})) "
                f"return false;"
            )
        else:
            out.append(
                f"        if(!reader.read(msg.{field_name})) "
                f"return false;"
            )
    out.append("")
    out.append("        if(consumed_size) {")
    out.append("            *consumed_size = reader.offset();")
    out.append("        }")
    out.append("        return true;")
    out.append("    }")
    out.append("};")
    return "\n".join(out)


# =============================================================================
# PYTHON SERIALIZER GENERATOR
# =============================================================================

def generate_python_serializer(name, fields, metadata):
    """Generate a Python pack/unpack module for a single struct/event/request type."""
    snake = snake_case(name)
    endian = "<" if metadata.get("endian", "little") == "little" else ">"

    out = []
    out.append('"""')
    out.append(f"Auto-generated Python serializer for {name}")
    out.append("Do not edit — regenerate with generate.py")
    out.append('"""')
    out.append("import struct")
    out.append("")

    # Imports for embedded struct dependencies (not primitives, not enums)
    struct_deps_seen = []
    for field in fields:
        base = extract_base_type(field["type"])
        if is_custom_type(field["type"]) and base not in KNOWN_ENUMS:
            dep_snake = snake_case(base)
            if dep_snake not in struct_deps_seen:
                struct_deps_seen.append(dep_snake)

    for dep_snake in struct_deps_seen:
        out.append(f"from .{dep_snake} import pack_{dep_snake}, unpack_{dep_snake}")
    if struct_deps_seen:
        out.append("")

    # ---- pack function --------------------------------------------------
    out.append(f"def pack_{snake}(msg: dict) -> bytes:")
    out.append(f'    """Serialize {name} to bytes (little-endian wire format)."""')
    out.append("    buf = b''")

    for field in fields:
        fn = field["name"]
        ft = field["type"]
        base = extract_base_type(ft)

        if base == "string":
            out.append(f"    _s = msg['{fn}']")
            out.append(f"    if isinstance(_s, str): _s = _s.encode('utf-8')")
            out.append(f"    buf += struct.pack('{endian}H', len(_s)) + _s")

        elif is_fixed_array(ft):
            count = fixed_array_size(ft)
            if base in PYTHON_FMT_MAP:
                fmt = PYTHON_FMT_MAP[base]
                out.append(f"    for _v in msg['{fn}'][:{count}]:")
                out.append(f"        buf += struct.pack('{endian}{fmt}', _v)")
            elif base in KNOWN_ENUMS:
                efmt = PYTHON_FMT_MAP.get(KNOWN_ENUMS[base]["base"], "B")
                out.append(f"    for _v in msg['{fn}'][:{count}]:")
                out.append(f"        buf += struct.pack('{endian}{efmt}', int(_v))")
            else:
                ds = snake_case(base)
                out.append(f"    for _v in msg['{fn}'][:{count}]:")
                out.append(f"        buf += pack_{ds}(_v)")

        elif is_dynamic_array(ft):
            if base in PYTHON_FMT_MAP:
                fmt = PYTHON_FMT_MAP[base]
                out.append(f"    _arr = msg['{fn}']")
                out.append(f"    buf += struct.pack('{endian}H', len(_arr))")
                out.append(f"    for _v in _arr:")
                out.append(f"        buf += struct.pack('{endian}{fmt}', _v)")
            elif base in KNOWN_ENUMS:
                efmt = PYTHON_FMT_MAP.get(KNOWN_ENUMS[base]["base"], "B")
                out.append(f"    _arr = msg['{fn}']")
                out.append(f"    buf += struct.pack('{endian}H', len(_arr))")
                out.append(f"    for _v in _arr:")
                out.append(f"        buf += struct.pack('{endian}{efmt}', int(_v))")
            else:
                ds = snake_case(base)
                out.append(f"    _arr = msg['{fn}']")
                out.append(f"    buf += struct.pack('{endian}H', len(_arr))")
                out.append(f"    for _v in _arr:")
                out.append(f"        buf += pack_{ds}(_v)")

        elif base in PYTHON_FMT_MAP:
            fmt = PYTHON_FMT_MAP[base]
            out.append(f"    buf += struct.pack('{endian}{fmt}', msg['{fn}'])")
        elif base in KNOWN_ENUMS:
            efmt = PYTHON_FMT_MAP.get(KNOWN_ENUMS[base]["base"], "B")
            out.append(f"    buf += struct.pack('{endian}{efmt}', int(msg['{fn}']))")
        else:
            ds = snake_case(base)
            out.append(f"    buf += pack_{ds}(msg['{fn}'])")

    out.append("    return buf")
    out.append("")

    # ---- unpack function ------------------------------------------------
    out.append(f"def unpack_{snake}(data: bytes, offset: int = 0):")
    out.append(f'    """Deserialize {name} from bytes. Returns (dict, bytes_consumed)."""')
    out.append("    msg = {}")
    out.append("    _start = offset")

    for field in fields:
        fn = field["name"]
        ft = field["type"]
        base = extract_base_type(ft)

        if base == "string":
            out.append(f"    _len, = struct.unpack_from('{endian}H', data, offset)")
            out.append(f"    offset += 2")
            out.append(f"    msg['{fn}'] = data[offset:offset + _len].decode('utf-8')")
            out.append(f"    offset += _len")

        elif is_fixed_array(ft):
            count = fixed_array_size(ft)
            if base in PYTHON_FMT_MAP:
                fmt = PYTHON_FMT_MAP[base]
                sz = PYTHON_SIZE_MAP[base]
                out.append(f"    msg['{fn}'] = []")
                out.append(f"    for _i in range({count}):")
                out.append(f"        _v, = struct.unpack_from('{endian}{fmt}', data, offset)")
                out.append(f"        offset += {sz}")
                out.append(f"        msg['{fn}'].append(_v)")
            elif base in KNOWN_ENUMS:
                efmt = PYTHON_FMT_MAP.get(KNOWN_ENUMS[base]["base"], "B")
                esz = PYTHON_SIZE_MAP.get(KNOWN_ENUMS[base]["base"], 1)
                out.append(f"    msg['{fn}'] = []")
                out.append(f"    for _i in range({count}):")
                out.append(f"        _v, = struct.unpack_from('{endian}{efmt}', data, offset)")
                out.append(f"        offset += {esz}")
                out.append(f"        msg['{fn}'].append(_v)")
            else:
                ds = snake_case(base)
                out.append(f"    msg['{fn}'] = []")
                out.append(f"    for _i in range({count}):")
                out.append(f"        _item, _sz = unpack_{ds}(data, offset)")
                out.append(f"        offset += _sz")
                out.append(f"        msg['{fn}'].append(_item)")

        elif is_dynamic_array(ft):
            if base in PYTHON_FMT_MAP:
                fmt = PYTHON_FMT_MAP[base]
                sz = PYTHON_SIZE_MAP[base]
                out.append(f"    _count, = struct.unpack_from('{endian}H', data, offset)")
                out.append(f"    offset += 2")
                out.append(f"    msg['{fn}'] = []")
                out.append(f"    for _i in range(_count):")
                out.append(f"        _v, = struct.unpack_from('{endian}{fmt}', data, offset)")
                out.append(f"        offset += {sz}")
                out.append(f"        msg['{fn}'].append(_v)")
            elif base in KNOWN_ENUMS:
                efmt = PYTHON_FMT_MAP.get(KNOWN_ENUMS[base]["base"], "B")
                esz = PYTHON_SIZE_MAP.get(KNOWN_ENUMS[base]["base"], 1)
                out.append(f"    _count, = struct.unpack_from('{endian}H', data, offset)")
                out.append(f"    offset += 2")
                out.append(f"    msg['{fn}'] = []")
                out.append(f"    for _i in range(_count):")
                out.append(f"        _v, = struct.unpack_from('{endian}{efmt}', data, offset)")
                out.append(f"        offset += {esz}")
                out.append(f"        msg['{fn}'].append(_v)")
            else:
                ds = snake_case(base)
                out.append(f"    _count, = struct.unpack_from('{endian}H', data, offset)")
                out.append(f"    offset += 2")
                out.append(f"    msg['{fn}'] = []")
                out.append(f"    for _i in range(_count):")
                out.append(f"        _item, _sz = unpack_{ds}(data, offset)")
                out.append(f"        offset += _sz")
                out.append(f"        msg['{fn}'].append(_item)")

        elif base in PYTHON_FMT_MAP:
            fmt = PYTHON_FMT_MAP[base]
            sz = PYTHON_SIZE_MAP[base]
            out.append(f"    msg['{fn}'], = struct.unpack_from('{endian}{fmt}', data, offset)")
            out.append(f"    offset += {sz}")
        elif base in KNOWN_ENUMS:
            efmt = PYTHON_FMT_MAP.get(KNOWN_ENUMS[base]["base"], "B")
            esz = PYTHON_SIZE_MAP.get(KNOWN_ENUMS[base]["base"], 1)
            out.append(f"    msg['{fn}'], = struct.unpack_from('{endian}{efmt}', data, offset)")
            out.append(f"    offset += {esz}")
        else:
            ds = snake_case(base)
            out.append(f"    _val, _sz = unpack_{ds}(data, offset)")
            out.append(f"    offset += _sz")
            out.append(f"    msg['{fn}'] = _val")

    out.append("    return msg, offset - _start")
    out.append("")

    return "\n".join(out)


# =============================================================================
# FILE PROCESSORS
# =============================================================================

# =============================================================================
# ENUM PROCESSOR
# =============================================================================

def process_enum(path: Path, output_dir: Path):
    """Process a .enum IDL file → C++ enum class + Python IntEnum."""
    raw_name = path.stem
    name = pascal_case(raw_name)
    lines = read_lines(path)
    metadata, lines = parse_metadata(lines)
    validate_metadata_ids(name, metadata)

    # @base uint8 (default) controls the underlying C++ integer type
    base_type = metadata.get("base", "uint8")
    cpp_base = CPP_TYPE_MAP.get(base_type, "uint8_t")

    # Parse "NAME = VALUE" lines; skip the optional type-name line
    values = []
    for line in lines:
        m = re.match(r"([A-Z_][A-Z0-9_]*)\s*=\s*(-?\d+)", line)
        if m:
            values.append((m.group(1), int(m.group(2))))

    # Register for use by Python/C++ generators that reference this enum as a field type
    KNOWN_ENUMS[name] = {
        "base": base_type,
        "values": {k: v for k, v in values}
    }

    # ── C++ output ────────────────────────────────────────────────────────
    enum_dir = output_dir / "enum"
    ensure_dir(enum_dir)

    cpp_lines = []
    cpp_lines.append("#pragma once")
    cpp_lines.append("")
    cpp_lines.append("#include <stdint.h>")
    cpp_lines.append("")

    if metadata.get("deprecated"):
        reason = f"SerialComm IDL: {name} is @deprecated"
        cpp_lines.append(f'[[deprecated("{reason}")]]')

    cpp_lines.append(f"enum class {name} : {cpp_base} {{")
    for enum_name, enum_val in values:
        cpp_lines.append(f"    {enum_name} = {enum_val},")
    cpp_lines.append("};")

    if "id" in metadata:
        cpp_lines.append("")
        cpp_lines.append(
            f"static constexpr uint16_t {name.upper()}_ID = {metadata['id']};"
        )

    with open(enum_dir / f"{snake_case(name)}.hpp", "w") as f:
        f.write("\n".join(cpp_lines))

    # ── Python output ─────────────────────────────────────────────────────
    py_dir = output_dir / "python"
    ensure_dir(py_dir)

    py_lines = []
    py_lines.append('"""')
    py_lines.append(f"Auto-generated Python enum for {name}")
    py_lines.append("Do not edit — regenerate with generate.py")
    py_lines.append('"""')
    py_lines.append("from enum import IntEnum")
    py_lines.append("")
    py_lines.append(f"class {name}(IntEnum):")
    for enum_name, enum_val in values:
        py_lines.append(f"    {enum_name} = {enum_val}")
    py_lines.append("")

    py_file = snake_case(name)
    with open(py_dir / f"{py_file}.py", "w") as f:
        f.write("\n".join(py_lines))

    generated_python_modules.append({
        "file": py_file,
        "symbols": [name],
    })

    entry = make_manifest_entry(name, "enum", metadata)
    entry["base"] = base_type
    MANIFEST.append(entry)

    print(f"[GEN] Enum: {name}")


def process_struct(path: Path, output_dir: Path):
    raw_name = path.stem
    name = pascal_case(raw_name)
    lines = read_lines(path)
    metadata, lines = parse_metadata(lines)
    validate_metadata_ids(name, metadata)
    fields = []
    for line in lines:
        field = parse_field(line)
        if field:
            fields.append(field)
    deps = collect_dependencies(fields)
    header = generate_header(deps)
    struct_code = generate_struct( name, fields, metadata )
    serializer_code = generate_serializer( name, fields, metadata )
    struct_dir = output_dir / "struct"
    serializer_dir = output_dir / "serializers"
    ensure_dir(struct_dir)
    ensure_dir(serializer_dir)
    with open(
        struct_dir / f"{snake_case(name)}.hpp",
        "w"
    ) as f:
        f.write(header)
        f.write("\n\n")
        f.write(struct_code)
    with open(
        serializer_dir /
        f"{snake_case(name)}_serializer.hpp",
        "w"
    ) as f:
        f.write(serializer_code)
        generated_serializer_includes.append({
            "name": name,
            "deps": deps,
            "include": f'#include "serial_comm/generated/serializers/{snake_case(name)}_serializer.hpp"'
        })

    # Python serializer
    py_dir = output_dir / "python"
    ensure_dir(py_dir)
    py_code = generate_python_serializer(name, fields, metadata)
    py_file = snake_case(name)
    with open(py_dir / f"{py_file}.py", "w") as f:
        f.write(py_code)
    generated_python_modules.append({
        "file": py_file,
        "symbols": [f"pack_{py_file}", f"unpack_{py_file}"],
    })

    MANIFEST.append(make_manifest_entry(name, "struct", metadata))
    print(f"[GEN] Struct: {name}")


# =============================================================================
# REQUEST PROCESSOR
# =============================================================================
def process_request(path: Path, output_dir: Path):

    raw_name = path.stem
    name = pascal_case(raw_name)

    lines = read_lines(path)

    metadata, lines = parse_metadata(lines)

    validate_metadata_ids(name, metadata)

    sections = []
    current = []

    for line in lines:

        if line == "===":

            sections.append(current)
            current = []
            continue

        current.append(line)

    sections.append(current)

    if len(sections) != 2:

        raise RuntimeError(
            f"{path.name} requires 2 sections (request + response)"
        )

    req_fields = [
        parse_field(x)
        for x in sections[0]
    ]

    res_fields = [
        parse_field(x)
        for x in sections[1]
    ]

    req_fields = [x for x in req_fields if x]
    res_fields = [x for x in res_fields if x]

    deps = collect_dependencies(
        req_fields + res_fields
    )

    header = generate_header(deps)

    delivery_mode = metadata["delivery_mode"]

    req_code = generate_struct(
        f"{name}_Request",
        req_fields,
        metadata,
        delivery_mode
    )

    res_code = generate_struct(
        f"{name}_Response",
        res_fields,
        metadata,
        delivery_mode
    )

    out_dir = output_dir / "request"

    ensure_dir(out_dir)

    with open(
        out_dir / f"{snake_case(name)}.hpp",
        "w"
    ) as f:

        f.write(header)
        f.write("\n\n")
        f.write(req_code)
        f.write("\n\n")
        f.write(res_code)

    # Generate serializers for request/response
    serializer_dir = output_dir / "serializers"
    ensure_dir(serializer_dir)

    req_serializer = generate_serializer(
        f"{name}_Request",
        req_fields,
        metadata,
        f"serial_comm/generated/request/{snake_case(name)}.hpp"
    )

    res_serializer = generate_serializer(
        f"{name}_Response",
        res_fields,
        metadata,
        f"serial_comm/generated/request/{snake_case(name)}.hpp"
    )

    with open(
        serializer_dir / f"{snake_case(name)}_request_serializer.hpp",
        "w"
    ) as f:

        f.write(req_serializer)

    with open(
        serializer_dir / f"{snake_case(name)}_response_serializer.hpp",
        "w"
    ) as f:

        f.write(res_serializer)
        generated_serializer_includes.append({
            "name": f"{name}_Request",
            "deps": deps,
            "include": f'#include "serial_comm/generated/serializers/{snake_case(name)}_request_serializer.hpp"'
        })
        generated_serializer_includes.append({
            "name": f"{name}_Response",
            "deps": deps,
            "include": f'#include "serial_comm/generated/serializers/{snake_case(name)}_response_serializer.hpp"'
        })

    # Python serializers for request/response
    py_dir = output_dir / "python"
    ensure_dir(py_dir)

    req_py = generate_python_serializer(f"{name}_Request", req_fields, metadata)
    res_py = generate_python_serializer(f"{name}_Response", res_fields, metadata)

    req_py_file = f"{snake_case(name)}_request"
    res_py_file = f"{snake_case(name)}_response"

    with open(py_dir / f"{req_py_file}.py", "w") as f:
        f.write(req_py)
    with open(py_dir / f"{res_py_file}.py", "w") as f:
        f.write(res_py)

    generated_python_modules.append({
        "file": req_py_file,
        "symbols": [f"pack_{req_py_file}", f"unpack_{req_py_file}"],
    })
    generated_python_modules.append({
        "file": res_py_file,
        "symbols": [f"pack_{res_py_file}", f"unpack_{res_py_file}"],
    })

    MANIFEST.append(make_manifest_entry(name, "request", metadata))

    print(f"[GEN] Request: {name}")


# =============================================================================
# EVENT PROCESSOR
# =============================================================================
def process_event(path: Path, output_dir: Path):
    raw_name = path.stem
    name = pascal_case(raw_name)
    lines = read_lines(path)
    metadata, lines = parse_metadata(lines)
    validate_metadata_ids(name, metadata)
    fields = []
    for line in lines:
        field = parse_field(line)
        if field:
            fields.append(field)
    deps = collect_dependencies(fields)
    header = generate_header(deps)
    event_code = generate_struct( name, fields, metadata, metadata["delivery_mode"] )
    event_dir = output_dir / "event"
    serializer_dir = output_dir / "serializers"
    ensure_dir(event_dir)
    ensure_dir(serializer_dir)
    with open( event_dir / f"{snake_case(name)}.hpp", "w" ) as f:
        f.write(header)
        f.write("\n\n")
        f.write(event_code)
    serializer_code = generate_serializer(
        name,
        fields,
        metadata,
        f"serial_comm/generated/event/{snake_case(name)}.hpp"
    )
    with open( serializer_dir / f"{snake_case(name)}_serializer.hpp", "w" ) as f:
        f.write(serializer_code)
        generated_serializer_includes.append({
            "name": name,
            "deps": deps,
            "include": f'#include "serial_comm/generated/serializers/{snake_case(name)}_serializer.hpp"'
        })

    # Python serializer
    py_dir = output_dir / "python"
    ensure_dir(py_dir)
    py_code = generate_python_serializer(name, fields, metadata)
    py_file = snake_case(name)
    with open(py_dir / f"{py_file}.py", "w") as f:
        f.write(py_code)
    generated_python_modules.append({
        "file": py_file,
        "symbols": [f"pack_{py_file}", f"unpack_{py_file}"],
    })

    MANIFEST.append(make_manifest_entry(name, "event", metadata))
    print(f"[GEN] Event: {name}")


# =============================================================================
# MISSION PROCESSOR
# =============================================================================
def process_mission(path: Path, output_dir: Path):
    raw_name = path.stem
    name = pascal_case(raw_name)
    lines = read_lines(path)
    metadata, lines = parse_metadata(lines)
    validate_metadata_ids(name, metadata)
    sections = []
    current = []
    for line in lines:
        if line == "===":
            sections.append(current)
            current = []
            continue
        current.append(line)
    sections.append(current)
    if len(sections) != 3:
        raise RuntimeError(
            f"{path.name} requires 3 sections (goal + result + feedback)"
        )
    goal_fields = [ parse_field(x) for x in sections[0] ]
    result_fields = [ parse_field(x) for x in sections[1] ]
    feedback_fields = [ parse_field(x) for x in sections[2] ]
    goal_fields = [x for x in goal_fields if x]
    result_fields = [x for x in result_fields if x]
    feedback_fields = [x for x in feedback_fields if x]
    deps = collect_dependencies(
        goal_fields + result_fields + feedback_fields
    )
    header = generate_header(deps)
    delivery_mode = metadata["delivery_mode"]
    goal_code = generate_struct( f"{name}_Goal", goal_fields, metadata, delivery_mode )
    result_code = generate_struct( f"{name}_Result", result_fields, metadata, delivery_mode )
    feedback_code = generate_struct( f"{name}_Feedback", feedback_fields, metadata, delivery_mode )
    out_dir = output_dir / "mission"
    ensure_dir(out_dir)
    with open( out_dir / f"{snake_case(name)}.hpp", "w" ) as f:
        f.write(header)
        f.write("\n\n")
        f.write(goal_code)
        f.write("\n\n")
        f.write(result_code)
        f.write("\n\n")
        f.write(feedback_code)
    # Generate serializers for goal/result/feedback
    serializer_dir = output_dir / "serializers"
    ensure_dir(serializer_dir)
    goal_serializer = generate_serializer(
        f"{name}_Goal",
        goal_fields,
        metadata,
        f"serial_comm/generated/mission/{snake_case(name)}.hpp"
    )
    result_serializer = generate_serializer(
        f"{name}_Result",
        result_fields,
        metadata,
        f"serial_comm/generated/mission/{snake_case(name)}.hpp"
    )
    feedback_serializer = generate_serializer(
        f"{name}_Feedback",
        feedback_fields,
        metadata,
        f"serial_comm/generated/mission/{snake_case(name)}.hpp"
    )
    with open(
        serializer_dir / f"{snake_case(name)}_goal_serializer.hpp",
        "w"
    ) as f:
        f.write(goal_serializer)
    with open(
        serializer_dir / f"{snake_case(name)}_result_serializer.hpp",
        "w"
    ) as f:
        f.write(result_serializer)
    with open(
        serializer_dir / f"{snake_case(name)}_feedback_serializer.hpp",
        "w"
    ) as f:
        f.write(feedback_serializer)
        generated_serializer_includes.append({
            "name": f"{name}_Goal",
            "deps": deps,
            "include": f'#include "serial_comm/generated/serializers/{snake_case(name)}_goal_serializer.hpp"'
        })
        generated_serializer_includes.append({
            "name": f"{name}_Result",
            "deps": deps,
            "include": f'#include "serial_comm/generated/serializers/{snake_case(name)}_result_serializer.hpp"'
        })
        generated_serializer_includes.append({
            "name": f"{name}_Feedback",
            "deps": deps,
            "include": f'#include "serial_comm/generated/serializers/{snake_case(name)}_feedback_serializer.hpp"'
        })

    # Python serializers
    py_dir = output_dir / "python"
    ensure_dir(py_dir)
    for sub_name, sub_fields in [
        (f"{name}_Goal",     goal_fields),
        (f"{name}_Result",   result_fields),
        (f"{name}_Feedback", feedback_fields),
    ]:
        py_code = generate_python_serializer(sub_name, sub_fields, metadata)
        py_file = snake_case(sub_name)
        with open(py_dir / f"{py_file}.py", "w") as f:
            f.write(py_code)
        generated_python_modules.append({
            "file": py_file,
            "symbols": [f"pack_{py_file}", f"unpack_{py_file}"],
        })

    MANIFEST.append(make_manifest_entry(name, "mission", metadata))

    print(f"[GEN] Mission: {name}")


# =============================================================================
# SERIALIZER INCLUDE ORDERING
# =============================================================================
def order_serializer_includes(entries):
    """
    Topologically sort generated serializer includes so that every type's
    serializer is emitted only after the serializers of the custom types
    it embeds as fields.

    This ordering is not cosmetic: every generated serializer specializes
    the primary `SerialCommSerializer<T>` template (see
    serial_comm_serializer_base.hpp), which carries a generic fallback
    body. If `generated_serializers.hpp` `#include`s e.g.
    "robot_state_serializer.hpp" (RobotState embeds a Pose field) before
    "pose_serializer.hpp", the compiler implicitly instantiates
    `SerialCommSerializer<Pose>` from that generic primary template the
    moment it type-checks `SerialCommSerializer<RobotState>::serialize` --
    and the explicit specialization for `Pose` encountered afterwards then
    fails with the cryptic "explicit specialization after instantiation".
    Plain alphabetical sorting of include paths cannot guarantee this never
    happens (it depends entirely on how a dependency's name happens to
    compare to its user's), so the include order has to follow the
    dependency graph instead. Entries with no ordering constraint between
    them keep a stable alphabetical order, matching the previous behavior.
    """

    by_name = { entry["name"]: entry for entry in entries }
    state = {}
    ordered = []

    def visit(entry):
        name = entry["name"]
        if state.get(name) == "done":
            return
        if state.get(name) == "visiting":
            raise RuntimeError(
                f"[ERROR] circular type dependency detected involving "
                f"'{name}' (a fixed-layout C++ struct cannot embed "
                f"itself, directly or indirectly)"
            )
        state[name] = "visiting"
        for dep in entry["deps"]:
            dep_entry = by_name.get(dep)
            if dep_entry is not None:
                visit(dep_entry)
        state[name] = "done"
        ordered.append(entry)

    for entry in sorted(entries, key=lambda e: e["include"]):
        visit(entry)

    return ordered


# =============================================================================
# MANIFEST
# =============================================================================
def generate_manifest(output_dir):

    # ============================================================================
    # GENERATED SERIALIZER AGGREGATOR
    # ============================================================================
    aggregator_path = output_dir / "generated_serializers.hpp"
    with open(aggregator_path, "w") as f:
        f.write(
            "/**\n"
            " * @file generated_serializers.hpp\n"
            " * @brief Auto generated serializer aggregation file\n"
            " */\n\n"
        )
        f.write("#pragma once\n\n")
        for entry in order_serializer_includes(generated_serializer_includes):
            f.write(entry["include"] + "\n")
    print("[GEN] Serializer aggregator generated")

    # ============================================================================
    # PYTHON __init__.py AGGREGATOR
    # ============================================================================
    py_dir = output_dir / "python"
    ensure_dir(py_dir)

    init_lines = []
    init_lines.append('"""')
    init_lines.append("Auto-generated SerialComm Python bindings.")
    init_lines.append("Import pack_*/unpack_* functions or enum classes directly.")
    init_lines.append('"""')
    init_lines.append("")
    for mod in generated_python_modules:
        symbols = ", ".join(mod["symbols"])
        init_lines.append(f"from .{mod['file']} import {symbols}")
    init_lines.append("")

    with open(py_dir / "__init__.py", "w") as f:
        f.write("\n".join(init_lines))

    print("[GEN] Python __init__.py generated")

    with open( output_dir / "manifest.json", "w" ) as f:
        json.dump( MANIFEST, f, indent=4 )


# =============================================================================
# PYTHON CONSTANTS GENERATOR
# =============================================================================

def generate_python_constants(manifest: list, output_path: Path) -> None:
    """
    Write serial_comm_py/generated_constants.py from the current MANIFEST.
    Only entries with an @id are emitted; types without an ID are skipped.
    """
    # Collect entries that have an ID
    typed_entries = [e for e in manifest if e.get("id") is not None]

    out = []
    out.append('"""')
    out.append("generated_constants.py — Auto-generated by user_app/generate.py")
    out.append("")
    out.append("Contains command IDs and IDL decorator metadata for all message types,")
    out.append("so the Python side can mirror the C++ constexpr values without re-parsing")
    out.append("the IDL files.")
    out.append("")
    out.append("DO NOT EDIT MANUALLY — regenerate with:")
    out.append("    python3 user_app/generate.py --input user_app --output include/serial_comm/generated")
    out.append('"""')
    out.append("")
    out.append("from ._types import DeliveryMode")
    out.append("")

    # --- IDs ---
    out.append("# ---------------------------------------------------------------------------")
    out.append("# Command IDs")
    out.append("# ---------------------------------------------------------------------------")
    out.append("")
    for e in typed_entries:
        name_upper = snake_case(e["name"]).upper()
        # For requests, the ID covers both _Request and _Response; emit it once
        raw_id = e["id"]
        deprecated_note = "   # (@deprecated)" if e.get("deprecated") else ""
        out.append(f"{name_upper}_ID = {raw_id}{deprecated_note}")
    out.append("")

    # --- Delivery modes ---
    out.append("# ---------------------------------------------------------------------------")
    out.append("# Delivery modes  (mirrors DELIVERY_MODE constexpr in generated headers)")
    out.append("# ---------------------------------------------------------------------------")
    out.append("")
    out.append("DELIVERY_MODE: dict[int, DeliveryMode] = {")
    for e in typed_entries:
        name_upper = snake_case(e["name"]).upper()
        dm = "DeliveryMode.RELIABLE" if e.get("delivery_mode") == "RELIABLE" else "DeliveryMode.BEST_EFFORT"
        out.append(f"    {name_upper}_ID: {dm},")
    out.append("}")
    out.append("")

    # --- max_rate_hz ---
    rate_entries = [e for e in typed_entries if "max_rate_hz" in e]
    out.append("# ---------------------------------------------------------------------------")
    out.append("# @max_rate_hz  (mirrors MAX_RATE_HZ constexpr)")
    out.append("# ---------------------------------------------------------------------------")
    out.append("")
    out.append("MAX_RATE_HZ: dict[int, float] = {")
    for e in rate_entries:
        name_upper = snake_case(e["name"]).upper()
        out.append(f"    {name_upper}_ID: {e['max_rate_hz']},")
    out.append("}")
    out.append("")

    # --- retain ---
    retain_entries = [e for e in typed_entries if e.get("retain")]
    out.append("# ---------------------------------------------------------------------------")
    out.append("# @retain  (mirrors RETAIN constexpr)")
    out.append("# ---------------------------------------------------------------------------")
    out.append("")
    retain_ids = ", ".join(
        f"{e['name'].upper().replace(' ', '_')}_ID" for e in retain_entries
    )
    out.append(f"RETAIN_COMMANDS: frozenset[int] = frozenset({{")
    for e in retain_entries:
        name_upper = snake_case(e["name"]).upper()
        out.append(f"    {name_upper}_ID,")
    out.append("})")
    out.append("")

    # --- timeout_ms ---
    timeout_entries = [e for e in typed_entries if "timeout_ms" in e]
    out.append("# ---------------------------------------------------------------------------")
    out.append("# @timeout_ms  (mirrors TIMEOUT_MS constexpr; used by call_service)")
    out.append("# ---------------------------------------------------------------------------")
    out.append("")
    out.append("TIMEOUT_MS: dict[int, int] = {")
    for e in timeout_entries:
        name_upper = snake_case(e["name"]).upper()
        out.append(f"    {name_upper}_ID: {e['timeout_ms']},")
    out.append("}")
    out.append("")

    # --- schema version ---
    version_entries = [e for e in typed_entries if "version" in e]
    out.append("# ---------------------------------------------------------------------------")
    out.append("# @version  (mirrors SCHEMA_VERSION constexpr)")
    out.append("# ---------------------------------------------------------------------------")
    out.append("")
    out.append("SCHEMA_VERSION: dict[int, int] = {")
    for e in version_entries:
        name_upper = snake_case(e["name"]).upper()
        out.append(f"    {name_upper}_ID: {e['version']},")
    out.append("}")
    out.append("")

    # --- deprecated ---
    deprecated_entries = [e for e in typed_entries if e.get("deprecated")]
    out.append("# ---------------------------------------------------------------------------")
    out.append("# @deprecated  (informational)")
    out.append("# ---------------------------------------------------------------------------")
    out.append("")
    out.append("DEPRECATED_COMMANDS: frozenset[int] = frozenset({")
    for e in deprecated_entries:
        name_upper = snake_case(e["name"]).upper()
        out.append(f"    {name_upper}_ID,")
    out.append("})")
    out.append("")

    with open(output_path, "w") as f:
        f.write("\n".join(out))

    print(f"[GEN] Python constants → {output_path}")


# =============================================================================
# MAIN
# =============================================================================
def main():

    parser = argparse.ArgumentParser()
    parser.add_argument( "--input" , required = True )
    parser.add_argument( "--output", required = True )
    parser.add_argument(
        "--constants-out",
        default = None,
        help    = "Path to write generated_constants.py (default: auto-detect serial_comm_py/)"
    )

    args = parser.parse_args()

    input_dir = Path(args.input)
    output_dir = Path(args.output)

    if output_dir.exists():
        shutil.rmtree(output_dir)
    ensure_dir(output_dir)

    # ENUMS first — must be registered in KNOWN_ENUMS before any struct/event
    # that references them as field types
    for path in sorted(input_dir.rglob("*.enum")):
        process_enum(path, output_dir)
    # STRUCTS
    for path in input_dir.rglob("*.struct"):
        process_struct( path, output_dir)
    # EVENTS
    for path in input_dir.rglob("*.event"):
        process_event( path, output_dir )
    # REQUESTS
    for path in input_dir.rglob("*.request"):
        process_request( path, output_dir )
    # MISSIONS
    for path in input_dir.rglob("*.mission"):
        process_mission( path, output_dir )

    # MANIFEST + Python __init__
    generate_manifest(output_dir)

    # PYTHON CONSTANTS (generated_constants.py for serial_comm_py/)
    if args.constants_out:
        constants_path = Path(args.constants_out)
    else:
        # Auto-detect: look for serial_comm_py/ relative to input_dir's parent
        candidate = input_dir.parent / "serial_comm_py" / "generated_constants.py"
        constants_path = candidate if candidate.parent.exists() else None

    if constants_path:
        generate_python_constants(MANIFEST, constants_path)

    print("")
    print("[SerialComm] Generation completed")


if __name__ == "__main__":
    main()
