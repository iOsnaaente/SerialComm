"""
02_request.py — Service calls (Request / Response)

A Request is a blocking call-and-wait: the caller sends a packet and blocks
until the matching reply arrives (or a timeout fires).  On the server side,
``serve()`` registers a handler that receives the request payload and returns
the response payload.

IDL equivalent (user_app/request/SetMotor.request):
    @reliable
    @timeout_ms 500
    ---  Request  ---
    uint8   motor_id
    float32 speed
    bool    enable
    ===
    --- Response ---
    bool    success
    float32 actual_speed

Wire layout (little-endian):
    Request : '<Bf?'  →  motor_id(1) + speed(4) + enable(1) = 6 bytes
    Response: '<Bf'   →  success(1)  + actual_speed(4)       = 5 bytes

Run two terminals:
    terminal A (server):  python 02_request.py --port /dev/ttyUSB0 --role server
    terminal B (client):  python 02_request.py --port /dev/ttyUSB1 --role client
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from serial_comm_py import SerialCommClient, Command, Packet

# ---------------------------------------------------------------------------
# Command ID
# ---------------------------------------------------------------------------
CMD_SET_MOTOR = Command(0x30)

# ---------------------------------------------------------------------------
# Serialization helpers
# ---------------------------------------------------------------------------
_REQ_FMT  = '<Bf?'   # motor_id, speed, enable
_RES_FMT  = '<Bf'    # success, actual_speed

_REQ_SIZE = struct.calcsize(_REQ_FMT)   # 6 bytes
_RES_SIZE = struct.calcsize(_RES_FMT)   # 5 bytes


def pack_set_motor_request(motor_id: int, speed: float, enable: bool) -> bytes:
    return struct.pack(_REQ_FMT, motor_id & 0xFF, speed, enable)


def unpack_set_motor_request(payload: bytes) -> dict:
    if len(payload) < _REQ_SIZE:
        return {}
    motor_id, speed, enable = struct.unpack_from(_REQ_FMT, payload)
    return {"motor_id": motor_id, "speed": speed, "enable": bool(enable)}


def pack_set_motor_response(success: bool, actual_speed: float) -> bytes:
    return struct.pack(_RES_FMT, success, actual_speed)


def unpack_set_motor_response(payload: bytes) -> dict:
    if len(payload) < _RES_SIZE:
        return {}
    success, actual_speed = struct.unpack_from(_RES_FMT, payload)
    return {"success": bool(success), "actual_speed": actual_speed}


# ---------------------------------------------------------------------------
# Client side — calls the service
# ---------------------------------------------------------------------------

def run_client(client: SerialCommClient, timeout: float = 0.5) -> None:
    print(f"Client: calling SetMotor service on cmd=0x{int(CMD_SET_MOTOR):02X}")

    test_cases = [
        (0, 50.0,  True),
        (1, 75.5,  True),
        (0, 0.0,   False),
        (2, 100.0, True),
    ]

    for motor_id, speed, enable in test_cases:
        payload = pack_set_motor_request(motor_id, speed, enable)

        t0 = time.monotonic()
        ok, resp = client.call_service(CMD_SET_MOTOR, payload, timeout=timeout)
        rtt_ms = (time.monotonic() - t0) * 1000

        if ok:
            r = unpack_set_motor_response(resp)
            print(
                f"  motor={motor_id}  speed={speed:.1f}  enable={enable}  →  "
                f"success={r['success']}  actual={r['actual_speed']:.1f}  "
                f"RTT={rtt_ms:.1f}ms"
            )
        else:
            print(f"  motor={motor_id}  TIMEOUT after {timeout*1000:.0f}ms")

        time.sleep(0.5)


# ---------------------------------------------------------------------------
# Server side — handles the service
# ---------------------------------------------------------------------------

# Simulated motor state
_motor_speeds = [0.0, 0.0, 0.0]

def _handle_set_motor(request_payload: bytes) -> bytes:
    req = unpack_set_motor_request(request_payload)
    if not req:
        return pack_set_motor_response(False, 0.0)

    motor_id = req["motor_id"]
    speed    = req["speed"]
    enable   = req["enable"]

    print(f"  [SERVICE] SetMotor motor={motor_id}  speed={speed:.1f}  enable={enable}")

    if motor_id >= len(_motor_speeds):
        return pack_set_motor_response(False, 0.0)

    if enable:
        # Clamp to [0, 100]
        actual = max(0.0, min(100.0, speed))
        _motor_speeds[motor_id] = actual
    else:
        actual = 0.0
        _motor_speeds[motor_id] = 0.0

    return pack_set_motor_response(True, actual)


def run_server(client: SerialCommClient) -> None:
    print(f"Server: listening for SetMotor on cmd=0x{int(CMD_SET_MOTOR):02X}")
    client.serve(CMD_SET_MOTOR, _handle_set_motor)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="SerialComm request/response example")
    p.add_argument("--port",    default="/dev/ttyUSB0")
    p.add_argument("--baud",    type=int, default=921600)
    p.add_argument("--role",    choices=["client", "server"], default="client")
    p.add_argument("--timeout", type=float, default=0.5, help="Service call timeout (s)")
    return p.parse_args()


def main() -> None:
    args = _parse_args()

    with SerialCommClient(port=args.port, baudrate=args.baud) as client:
        if args.role == "client":
            run_client(client, timeout=args.timeout)
        else:
            run_server(client)


if __name__ == "__main__":
    main()
