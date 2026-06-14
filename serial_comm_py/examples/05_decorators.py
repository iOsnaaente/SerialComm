"""
05_decorators.py — IDL decorators in Python

SerialComm IDL decorators affect both the generated C++ code and the Python
client behaviour.  This example exercises all five decorators from the Python
side and shows what each one does at runtime.

Decorator summary
─────────────────
  @best_effort          → DeliveryMode.BEST_EFFORT  (default)
  @reliable             → DeliveryMode.RELIABLE — flushes OS TX buffer
  @timeout_ms N         → per-call timeout for call_service()
  @version N            → schema version check (informational; warn on mismatch)
  @deprecated           → warnings.warn() when the type is used
  @retain               → set_retain() + get_last() last-value cache
  @max_rate_hz N        → set_rate_limit() publish ceiling; ERR_BUSY on excess

Run:
    python 05_decorators.py --port /dev/ttyUSB0
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
import warnings
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from serial_comm_py import SerialCommClient, Command, Packet, DeliveryMode, ResultCode
from serial_comm_py.generated_constants import (
    ROBOT_VELOCITY_CMD_ID,
    ROBOT_TELEMETRY_ID,
    LEGACY_STATUS_ID,
    MAX_RATE_HZ,
    RETAIN_COMMANDS,
    DEPRECATED_COMMANDS,
    SCHEMA_VERSION,
    DELIVERY_MODE,
)

# ---------------------------------------------------------------------------
# Serialization helpers (matching the IDL field layouts)
# ---------------------------------------------------------------------------

VEL_CMD_FMT = '<Bff'    # robot_id, vl, vr
TEL_FMT     = '<Bfffff?'  # robot_id, ax, ay, gz, vl, vr, ball
LEGACY_FMT  = '<BB'     # robot_id, status_flags


def pack_vel_cmd(robot_id: int, vl: float, vr: float) -> bytes:
    return struct.pack(VEL_CMD_FMT, robot_id & 0xFF, vl, vr)

def unpack_telemetry(payload: bytes) -> dict | None:
    if len(payload) < struct.calcsize(TEL_FMT): return None
    rid, ax, ay, gz, vl, vr, ball = struct.unpack_from(TEL_FMT, payload)
    return {"robot_id": rid, "ax": ax, "ay": ay, "gz": gz,
            "vl": vl, "vr": vr, "ball": bool(ball)}

def pack_legacy_status(robot_id: int, flags: int) -> bytes:
    return struct.pack(LEGACY_FMT, robot_id & 0xFF, flags & 0xFF)


# ---------------------------------------------------------------------------
# Helper: configure client from IDL metadata
# ---------------------------------------------------------------------------

def configure_from_idl(client: SerialCommClient) -> None:
    """
    Read generated_constants.py and apply decorator semantics to the client.
    In a real project you would call this once at startup.
    """
    print("\n── Applying IDL decorator metadata ──")

    for cmd_id, rate_hz in MAX_RATE_HZ.items():
        client.set_rate_limit(cmd_id, rate_hz)
        print(f"  @max_rate_hz  cmd=0x{cmd_id:02X}  limit={rate_hz} Hz")

    for cmd_id in RETAIN_COMMANDS:
        client.set_retain(cmd_id, enabled=True)
        ver = SCHEMA_VERSION.get(cmd_id, "n/a")
        print(f"  @retain       cmd=0x{cmd_id:02X}  schema_version={ver}")

    for cmd_id in DEPRECATED_COMMANDS:
        print(f"  @deprecated   cmd=0x{cmd_id:02X}  (avoid in new code)")

    print()


# ---------------------------------------------------------------------------
# Demo 1: @best_effort vs @reliable
# ---------------------------------------------------------------------------

def demo_delivery_mode(client: SerialCommClient) -> None:
    print("── @best_effort vs @reliable ──")

    vel_cmd = Command(ROBOT_VELOCITY_CMD_ID)
    payload = pack_vel_cmd(0, 1.0, 1.0)

    # @best_effort — fire and forget, no flush
    mode = DELIVERY_MODE.get(int(vel_cmd), DeliveryMode.BEST_EFFORT)
    rc   = client.publish(vel_cmd, payload, delivery_mode=mode)
    print(f"  BEST_EFFORT  cmd=0x{int(vel_cmd):02X}  rc={rc.name}")

    # @reliable — flush OS TX buffer after write (guarantees bytes left the host)
    rc = client.publish(vel_cmd, payload, delivery_mode=DeliveryMode.RELIABLE)
    print(f"  RELIABLE     cmd=0x{int(vel_cmd):02X}  rc={rc.name}")
    print()


# ---------------------------------------------------------------------------
# Demo 2: @max_rate_hz — rate limiter
# ---------------------------------------------------------------------------

def demo_max_rate_hz(client: SerialCommClient) -> None:
    print("── @max_rate_hz — publish ceiling ──")

    vel_cmd  = Command(ROBOT_VELOCITY_CMD_ID)
    rate_hz  = MAX_RATE_HZ.get(int(vel_cmd), 33.0)
    interval = 1.0 / rate_hz

    print(f"  Declared rate: {rate_hz} Hz  (min interval: {interval*1000:.1f} ms)")
    print(f"  Sending 6 packets as fast as possible — expect ~3 dropped:")

    ok_count  = 0
    err_count = 0
    payload   = pack_vel_cmd(0, 1.0, 1.0)

    for i in range(6):
        rc = client.publish(vel_cmd, payload)
        if rc == ResultCode.OK:
            ok_count += 1
            print(f"    packet {i}  OK")
        elif rc == ResultCode.ERR_BUSY:
            err_count += 1
            print(f"    packet {i}  ERR_BUSY (rate exceeded)")
        time.sleep(interval / 3)   # faster than the declared rate

    print(f"  Sent: {ok_count}  Dropped: {err_count}")
    print()


# ---------------------------------------------------------------------------
# Demo 3: @retain — last-value cache
# ---------------------------------------------------------------------------

def demo_retain(client: SerialCommClient) -> None:
    print("── @retain — last-value cache ──")

    tel_cmd = Command(ROBOT_TELEMETRY_ID)

    # Simulate receiving a telemetry packet (normally it comes from firmware)
    fake_tel = struct.pack('<Bfffff?', 1, 0.1, -0.2, 0.03, 1.5, 1.5, True)

    def on_telemetry(pkt: Packet) -> None:
        pass  # callback still works alongside the cache

    client.subscribe(tel_cmd, on_telemetry)

    # Manually inject into the retain cache (demonstrates the API;
    # in production the cache is populated by incoming packets automatically)
    with client._retain_lock:
        if int(tel_cmd) in client._retained_enabled:
            client._retained[int(tel_cmd)] = fake_tel

    # get_last() returns the most recent payload without blocking
    last = client.get_last(tel_cmd)
    if last:
        tel = unpack_telemetry(last)
        if tel:
            print(
                f"  get_last() robot={tel['robot_id']}  "
                f"ax={tel['ax']:.2f} ay={tel['ay']:.2f}  ball={'YES' if tel['ball'] else 'no'}"
            )
    else:
        print("  No telemetry received yet — get_last() returned None")

    print()


# ---------------------------------------------------------------------------
# Demo 4: @deprecated — usage warning
# ---------------------------------------------------------------------------

def demo_deprecated(client: SerialCommClient) -> None:
    print("── @deprecated — usage warning ──")

    legacy_cmd = Command(LEGACY_STATUS_ID)

    if int(legacy_cmd) in DEPRECATED_COMMANDS:
        warnings.warn(
            f"Command 0x{int(legacy_cmd):02X} (LegacyStatus) is @deprecated. "
            f"Use RobotTelemetry (0x{ROBOT_TELEMETRY_ID:02X}) instead.",
            DeprecationWarning,
            stacklevel=2,
        )

    # Sending still works — @deprecated is advisory only
    payload = pack_legacy_status(robot_id=0, flags=0x01)
    rc = client.publish(legacy_cmd, payload)
    print(f"  Sent legacy packet  rc={rc.name}  (see DeprecationWarning above)")
    print()


# ---------------------------------------------------------------------------
# Demo 5: @version — schema compatibility check
# ---------------------------------------------------------------------------

def demo_version(client: SerialCommClient) -> None:
    print("── @version — schema version guard ──")

    tel_cmd        = Command(ROBOT_TELEMETRY_ID)
    expected_ver   = SCHEMA_VERSION.get(int(tel_cmd))
    received_ver   = 1     # in practice you'd embed this in the payload header

    if expected_ver is not None and received_ver != expected_ver:
        print(
            f"  [WARNING] Schema mismatch for cmd=0x{int(tel_cmd):02X}  "
            f"expected={expected_ver}  received={received_ver}"
        )
    else:
        print(
            f"  Schema OK  cmd=0x{int(tel_cmd):02X}  version={expected_ver}"
        )
    print()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="IDL decorator demo")
    p.add_argument("--port", default="/dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=921600)
    return p.parse_args()


def main() -> None:
    args = _parse_args()

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")

        with SerialCommClient(port=args.port, baudrate=args.baud) as client:
            configure_from_idl(client)
            demo_delivery_mode(client)
            demo_max_rate_hz(client)
            demo_retain(client)
            demo_deprecated(client)
            demo_version(client)

        if caught:
            print("── Deprecation warnings issued ──")
            for w in caught:
                print(f"  {w.category.__name__}: {w.message}")


if __name__ == "__main__":
    main()
