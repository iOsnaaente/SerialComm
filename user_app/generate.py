#!/usr/bin/env python3

"""
SerialComm IDL Generator

Features:
    - Struct/Event/Request/Mission parsing
    - Automatic dependency resolution
    - Automatic serializer generation
    - DynamicArray support
    - Little/Big endian metadata
    - Duplicate ID detection
    - Max serialized size generation
    - Reflection metadata generation
    - Runtime serialization helpers generation

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


USED_IDS = {}
MANIFEST = []


# ============================================================================
# GENERATED SERIALIZER REGISTRY
# ============================================================================
generated_serializer_includes = []


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

    return re.sub(
        r"([a-z0-9])([A-Z])",
        r"\1_\2",
        s1
    ).lower()


def pascal_case(name: str):

    return "".join(
        x.capitalize()
        for x in name.split("_")
    )


def read_lines(path: Path):

    with open(path, "r", encoding="utf-8") as f:

        return [
            line.strip()
            for line in f.readlines()
            if line.strip() and
            not line.startswith("#")
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

    return re.match(r".+\[\]", type_name) is not None


def is_fixed_array(type_name: str):

    return re.match(r".+\[\d+\]", type_name) is not None


def fixed_array_size(type_name: str):

    match = re.match(r".+\[(\d+)\]", type_name)

    if not match:
        return None

    return int(match.group(1))


def convert_type(type_name: str):

    # FIXED ARRAY
    fixed = re.match(r"(.+)\[(\d+)\]", type_name)

    if fixed:

        base = convert_type(fixed.group(1))
        size = fixed.group(2)

        return f"std::array<{base}, {size}>"

    # DYNAMIC ARRAY
    dynamic = re.match(r"(.+)\[\]", type_name)

    if dynamic:

        base = convert_type(dynamic.group(1))

        return f"SerialCommDynamicArray<{base}>"

    # BOUNDED ARRAY
    bounded = re.match(r"(.+)<=\d+\[\]", type_name)

    if bounded:

        base = convert_type(bounded.group(1))

        return f"DynamicArray<{base}>"

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

    # FIXED ARRAY
    if is_fixed_array(field_type):

        count = fixed_array_size(field_type)

        if base in TYPE_SIZE_MAP:
            return TYPE_SIZE_MAP[base] * count

        base_pascal = pascal_case(base)
        return f"({base_pascal}::MAX_SERIALIZED_SIZE * {count})"

    # DYNAMIC ARRAY
    if is_dynamic_array(field_type):

        return "0 /* dynamic */"

    # PRIMITIVE
    if base in TYPE_SIZE_MAP:

        return TYPE_SIZE_MAP[base]

    # CUSTOM
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

        out.append(
            f'#include "serial_comm/generated/struct/{snake_case(dep)}.hpp"'
        )

    out.append("")

    return "\n".join(out)


# =============================================================================
# STRUCT GENERATOR
# =============================================================================

def generate_struct(name, fields, metadata):
    out = []
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
# FILE PROCESSORS
# =============================================================================
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
        generated_serializer_includes.append(
            f'#include "serial_comm/generated/serializers/{snake_case(name)}_serializer.hpp"'
        )
    MANIFEST.append({
        "name": name,
        "type": "struct",
        "id": metadata.get("id", None)
    })
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

    req_code = generate_struct(
        f"{name}_Request",
        req_fields,
        metadata
    )

    res_code = generate_struct(
        f"{name}_Response",
        res_fields,
        metadata
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
        generated_serializer_includes.append(
            f'#include "serial_comm/generated/serializers/{snake_case(name)}_request_serializer.hpp"'
        )
        generated_serializer_includes.append(
            f'#include "serial_comm/generated/serializers/{snake_case(name)}_response_serializer.hpp"'
        )

    MANIFEST.append({
        "name": name,
        "type": "request",
        "id": metadata.get("id", None)
    })

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
    event_code = generate_struct( name, fields, metadata )
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
        generated_serializer_includes.append(
            f'#include "serial_comm/generated/serializers/{snake_case(name)}_serializer.hpp"'
        )
    MANIFEST.append({
        "name": name,
        "type": "event",
        "id": metadata.get("id", None)
    })
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
    goal_code = generate_struct( f"{name}_Goal", goal_fields, metadata )
    result_code = generate_struct( f"{name}_Result", result_fields, metadata )
    feedback_code = generate_struct( f"{name}_Feedback", feedback_fields, metadata )
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
        generated_serializer_includes.append(
            f'#include "serial_comm/generated/serializers/{snake_case(name)}_goal_serializer.hpp"'
        )
        generated_serializer_includes.append(
            f'#include "serial_comm/generated/serializers/{snake_case(name)}_result_serializer.hpp"'
        )
        generated_serializer_includes.append(
            f'#include "serial_comm/generated/serializers/{snake_case(name)}_feedback_serializer.hpp"'
        )

    MANIFEST.append({
        "name": name,
        "type": "mission",
        "id": metadata.get("id", None)
    })

    print(f"[GEN] Mission: {name}")


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
        for include in sorted(generated_serializer_includes):
            f.write(include + "\n")
    print("[GEN] Serializer aggregator generated")

    with open( output_dir / "manifest.json", "w" ) as f:
        json.dump( MANIFEST, f, indent=4 )


# =============================================================================
# MAIN
# =============================================================================
def main():

    parser = argparse.ArgumentParser()
    parser.add_argument( "--input" , required = True )
    parser.add_argument( "--output", required = True )

    args = parser.parse_args()

    input_dir = Path(args.input)
    output_dir = Path(args.output)

    if output_dir.exists():
        shutil.rmtree(output_dir)
    ensure_dir(output_dir)

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

    # MANIFEST
    generate_manifest(output_dir)

    print("")
    print("[SerialComm] Generation completed")


if __name__ == "__main__":
    main()