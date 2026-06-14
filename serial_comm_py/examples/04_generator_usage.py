"""
04_generator_usage.py — Using the IDL generator in a Python project

The SerialComm code generator (user_app/generate.py) processes IDL files
and outputs C++ headers.  Python projects consume the same IDL through
two complementary mechanisms:

  1. generated_constants.py  — auto-maintained file with command IDs,
       delivery modes, retain flags, schema versions, and rate ceilings.
       Import it instead of hard-coding magic numbers in your scripts.

  2. struct.pack / struct.unpack  — serialize/deserialize fields manually
       following the same byte order declared in the IDL (@little / @big).

This example shows the complete workflow end-to-end:

  Step 1 — Write your IDL files in user_app/
  Step 2 — Run the generator
  Step 3 — Use the generated constants and pack/unpack in Python

──────────────────────────────────────────────────────────────────────────
Step 1: IDL files (user_app/event/MyStatus.event)
──────────────────────────────────────────────────────────────────────────
    @id 0x50
    @best_effort
    @version 2
    @retain
    @max_rate_hz 10

    uint8  device_id
    float32 voltage
    float32 current
    bool    fault

──────────────────────────────────────────────────────────────────────────
Step 2: Run the generator from the project root
──────────────────────────────────────────────────────────────────────────
    python3 user_app/generate.py \\
        --input  user_app \\
        --output include/serial_comm/generated

    Generated files:
        include/serial_comm/generated/event/my_status.hpp
        include/serial_comm/generated/serializers/my_status_serializer.hpp
        include/serial_comm/generated/generated_serializers.hpp
        include/serial_comm/generated/manifest.json

──────────────────────────────────────────────────────────────────────────
Step 3: Use the constants in Python
──────────────────────────────────────────────────────────────────────────
    (see code below)

Each section below is annotated with what to look for in the generated
C++ header to understand the mapping.
"""

from __future__ import annotations

import json
import os
import struct
import sys
import time
import argparse
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from serial_comm_py import SerialCommClient, Command, Packet, DeliveryMode
from serial_comm_py.generated_constants import (
    # Command IDs  (C++: static constexpr uint16_t ROBOTVELOCITYCMD_ID = 0x10)
    ROBOT_VELOCITY_CMD_ID,
    ROBOT_TELEMETRY_ID,

    # Delivery modes  (C++: static constexpr DeliveryMode DELIVERY_MODE = ...)
    DELIVERY_MODE,

    # Rate ceilings  (C++: static constexpr float MAX_RATE_HZ = 33.0f)
    MAX_RATE_HZ,

    # Retain flags  (C++: static constexpr bool RETAIN = true)
    RETAIN_COMMANDS,

    # Schema versions  (C++: static constexpr uint8_t SCHEMA_VERSION = 1)
    SCHEMA_VERSION,
)

# ---------------------------------------------------------------------------
# How the generated C++ constexprs map to Python
# ---------------------------------------------------------------------------
#
#  C++ (robot_velocity_cmd.hpp):
#    static constexpr uint16_t ROBOTVELOCITYCMD_ID   = 0x10;
#    static constexpr DeliveryMode DELIVERY_MODE      = DeliveryMode::BEST_EFFORT;
#    static constexpr float MAX_RATE_HZ               = 33.0f;
#
#  Python (generated_constants.py):
#    ROBOT_VELOCITY_CMD_ID = 0x10
#    DELIVERY_MODE[0x10]   = DeliveryMode.BEST_EFFORT
#    MAX_RATE_HZ[0x10]     = 33.0
#
#  C++ (robot_telemetry.hpp):
#    static constexpr uint16_t ROBOTTELEMETRY_ID = 0x11;
#    static constexpr bool RETAIN                = true;
#    static constexpr uint8_t SCHEMA_VERSION     = 1;
#
#  Python (generated_constants.py):
#    ROBOT_TELEMETRY_ID    = 0x11
#    RETAIN_COMMANDS       = frozenset({0x11})
#    SCHEMA_VERSION[0x11]  = 1

# ---------------------------------------------------------------------------
# Serialization format strings derived from the IDL field list
# ---------------------------------------------------------------------------
#
# IDL type → struct.pack format character (always little-endian '<'):
#   uint8   → 'B'    int8    → 'b'
#   uint16  → 'H'    int16   → 'h'
#   uint32  → 'I'    int32   → 'i'
#   float32 → 'f'    float64 → 'd'
#   bool    → '?'    string  → (not supported by struct — use bytes manually)
#
# RobotVelocityCmd: uint8 robot_id, float32 vl, float32 vr
VEL_CMD_FMT  = '<Bff'   # 9 bytes
VEL_CMD_SIZE = struct.calcsize(VEL_CMD_FMT)

# RobotTelemetry: uint8 robot_id, float32 ax, ay, gz, vl, vr, bool ball_detected
TEL_FMT  = '<Bfffff?'   # 22 bytes
TEL_SIZE = struct.calcsize(TEL_FMT)


def pack_vel_cmd(robot_id: int, vl: float, vr: float) -> bytes:
    return struct.pack(VEL_CMD_FMT, robot_id & 0xFF, vl, vr)


def unpack_telemetry(payload: bytes) -> dict | None:
    if len(payload) < TEL_SIZE:
        return None
    robot_id, ax, ay, gz, vl, vr, ball = struct.unpack_from(TEL_FMT, payload)
    return {
        "robot_id": robot_id,
        "ax": ax, "ay": ay, "gz": gz,
        "vl_measured": vl, "vr_measured": vr,
        "ball_detected": bool(ball),
    }


# ---------------------------------------------------------------------------
# Reading the manifest.json produced by the generator
# ---------------------------------------------------------------------------

def print_manifest(repo_root: str) -> None:
    manifest_path = Path(repo_root) / "include" / "serial_comm" / "generated" / "manifest.json"
    if not manifest_path.exists():
        print(f"  manifest.json not found at {manifest_path}")
        print("  Run:  python3 user_app/generate.py --input user_app --output include/serial_comm/generated")
        return

    with open(manifest_path) as f:
        manifest = json.load(f)

    print("Generated types (from manifest.json):")
    for entry in manifest:
        print(f"  {entry['type']:8s}  {entry['name']:30s}  id={entry.get('id', 'n/a')}")


# ---------------------------------------------------------------------------
# Demo — wires everything together
# ---------------------------------------------------------------------------

def run_demo(client: SerialCommClient) -> None:
    vel_cmd = Command(ROBOT_VELOCITY_CMD_ID)
    tel_cmd = Command(ROBOT_TELEMETRY_ID)

    # Apply the rate limit declared in the IDL (@max_rate_hz 33)
    if vel_cmd in MAX_RATE_HZ:
        client.set_rate_limit(vel_cmd, MAX_RATE_HZ[int(vel_cmd)])
        print(f"Rate limit set: cmd=0x{int(vel_cmd):02X}  max={MAX_RATE_HZ[int(vel_cmd)]} Hz")

    # Enable retain cache for RobotTelemetry (@retain)
    if int(tel_cmd) in RETAIN_COMMANDS:
        client.set_retain(tel_cmd, enabled=True)
        print(f"Retain enabled: cmd=0x{int(tel_cmd):02X}  schema_version={SCHEMA_VERSION.get(int(tel_cmd), 'n/a')}")

    def on_telemetry(pkt: Packet) -> None:
        tel = unpack_telemetry(pkt.payload)
        if tel:
            print(
                f"  TEL  robot={tel['robot_id']}  "
                f"ax={tel['ax']:.2f} ay={tel['ay']:.2f} gz={tel['gz']:.2f}  "
                f"ball={'YES' if tel['ball_detected'] else 'no'}"
            )

    client.subscribe(tel_cmd, on_telemetry)

    # Send velocity commands at the IDL-declared rate
    tick = 0
    try:
        while True:
            payload = pack_vel_cmd(robot_id=0, vl=1.0, vr=1.0)
            rc = client.publish(
                vel_cmd,
                payload,
                delivery_mode=DELIVERY_MODE.get(int(vel_cmd), DeliveryMode.BEST_EFFORT),
            )
            print(f"  VEL tick={tick}  rc={rc.name}")

            # Read last telemetry from retain cache (no blocking, no callback needed)
            last = client.get_last(tel_cmd)
            if last:
                tel = unpack_telemetry(last)
                if tel:
                    print(f"  LAST TEL (cached)  robot={tel['robot_id']}  ball={'YES' if tel['ball_detected'] else 'no'}")

            tick += 1
            time.sleep(1.0 / 33.0)    # 33 Hz
    except KeyboardInterrupt:
        pass


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="IDL generator usage demo")
    p.add_argument("--port",      default="/dev/ttyUSB0")
    p.add_argument("--baud",      type=int, default=921600)
    p.add_argument("--manifest",  action="store_true", help="Print generated manifest and exit")
    p.add_argument("--repo-root", default=os.path.join(os.path.dirname(__file__), '..', '..'),
                   help="Path to repo root (for manifest reading)")
    return p.parse_args()


def main() -> None:
    args = _parse_args()

    if args.manifest:
        print_manifest(args.repo_root)
        return

    with SerialCommClient(port=args.port, baudrate=args.baud) as client:
        run_demo(client)


if __name__ == "__main__":
    main()
