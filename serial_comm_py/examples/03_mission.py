"""
03_mission.py — Mission pattern (Goal → Feedback → Result)

A Mission is a long-running action with three phases:
  1. Goal    — caller sends the target; server returns an acceptance ACK.
  2. Feedback — server streams progress events while executing.
  3. Result   — server sends a final event when the mission finishes.

SerialCommClient does not have a dedicated mission API (missions live in
the firmware's SerialCommAction).  From Python you compose the same three
primitives you already know:

  call_service(GOAL_CMD, ...)         ← start the mission, get ACK
  subscribe(FEEDBACK_CMD, callback)   ← stream progress updates
  subscribe(RESULT_CMD, callback)     ← one-shot completion notification

IDL equivalent (user_app/mission/Navigate.mission):

    @reliable
    --- Goal ---
    float32 target_x
    float32 target_y
    float32 speed
    ===
    --- Result ---
    bool    reached
    float32 final_x
    float32 final_y
    ===
    --- Feedback ---
    float32 current_x
    float32 current_y
    float32 distance_remaining

Wire layout (little-endian):
    Goal     : '<fff'  →  x(4) + y(4) + speed(4)          = 12 bytes
    Result   : '<Bff'  →  reached(1) + fx(4) + fy(4)      =  9 bytes
    Feedback : '<fff'  →  cx(4) + cy(4) + dist(4)         = 12 bytes

Run two terminals:
    terminal A (executor): python 03_mission.py --port /dev/ttyUSB0 --role executor
    terminal B (caller):   python 03_mission.py --port /dev/ttyUSB1 --role caller
"""

from __future__ import annotations

import argparse
import struct
import sys
import threading
import time
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from serial_comm_py import SerialCommClient, Command, Packet

# ---------------------------------------------------------------------------
# Command IDs — three commands per mission type
# ---------------------------------------------------------------------------
CMD_NAV_GOAL     = Command(0x40)   # Request: caller → executor
CMD_NAV_FEEDBACK = Command(0x41)   # Event:   executor → caller (streaming)
CMD_NAV_RESULT   = Command(0x42)   # Event:   executor → caller (final)

# ---------------------------------------------------------------------------
# Serialization
# ---------------------------------------------------------------------------

def pack_goal(target_x: float, target_y: float, speed: float) -> bytes:
    return struct.pack('<fff', target_x, target_y, speed)

def unpack_goal(payload: bytes) -> dict:
    if len(payload) < 12: return {}
    x, y, s = struct.unpack_from('<fff', payload)
    return {"target_x": x, "target_y": y, "speed": s}

def pack_goal_ack(accepted: bool) -> bytes:
    return struct.pack('<B', accepted)

def pack_result(reached: bool, final_x: float, final_y: float) -> bytes:
    return struct.pack('<Bff', reached, final_x, final_y)

def unpack_result(payload: bytes) -> dict:
    if len(payload) < 9: return {}
    r, fx, fy = struct.unpack_from('<Bff', payload)
    return {"reached": bool(r), "final_x": fx, "final_y": fy}

def pack_feedback(cx: float, cy: float, dist: float) -> bytes:
    return struct.pack('<fff', cx, cy, dist)

def unpack_feedback(payload: bytes) -> dict:
    if len(payload) < 12: return {}
    cx, cy, d = struct.unpack_from('<fff', payload)
    return {"current_x": cx, "current_y": cy, "distance_remaining": d}

# ---------------------------------------------------------------------------
# Caller side — sends the goal and waits for feedback + result
# ---------------------------------------------------------------------------

def run_caller(client: SerialCommClient) -> None:
    result_event = threading.Event()
    result_data  = {}

    def on_feedback(pkt: Packet) -> None:
        fb = unpack_feedback(pkt.payload)
        if fb:
            print(
                f"  [FEEDBACK] pos=({fb['current_x']:.2f}, {fb['current_y']:.2f})  "
                f"remaining={fb['distance_remaining']:.2f}m"
            )

    def on_result(pkt: Packet) -> None:
        res = unpack_result(pkt.payload)
        result_data.update(res)
        result_event.set()

    # Subscribe BEFORE sending the goal so no packets are missed
    client.subscribe(CMD_NAV_FEEDBACK, on_feedback)
    client.subscribe(CMD_NAV_RESULT,   on_result)

    goal_payload = pack_goal(target_x=5.0, target_y=3.0, speed=1.0)

    print("Caller: sending Navigate goal  target=(5.0, 3.0)  speed=1.0 m/s")
    ok, ack = client.call_service(CMD_NAV_GOAL, goal_payload, timeout=2.0)

    if not ok:
        print("Caller: goal not accepted (timeout)")
        return
    accepted = bool(ack[0]) if ack else False
    print(f"Caller: goal {'ACCEPTED' if accepted else 'REJECTED'} by executor")
    if not accepted:
        return

    print("Caller: waiting for mission result...")
    if result_event.wait(timeout=30.0):
        r = result_data
        print(
            f"Caller: RESULT  reached={r['reached']}  "
            f"final=({r['final_x']:.2f}, {r['final_y']:.2f})"
        )
    else:
        print("Caller: result timeout — mission still running or executor died")


# ---------------------------------------------------------------------------
# Executor side — receives the goal and streams feedback + result
# ---------------------------------------------------------------------------

def run_executor(client: SerialCommClient) -> None:
    print("Executor: ready to receive Navigate goals")

    def _handle_goal(request_payload: bytes) -> bytes:
        goal = unpack_goal(request_payload)
        if not goal:
            return pack_goal_ack(False)

        print(
            f"Executor: goal received  target=({goal['target_x']:.2f}, {goal['target_y']:.2f})"
            f"  speed={goal['speed']:.2f}"
        )

        # Accept immediately — execute in a background thread
        threading.Thread(
            target=_execute_mission,
            args=(client, goal),
            daemon=True,
        ).start()
        return pack_goal_ack(True)

    client.serve(CMD_NAV_GOAL, _handle_goal)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


def _execute_mission(client: SerialCommClient, goal: dict) -> None:
    tx, ty = goal["target_x"], goal["target_y"]
    speed  = max(0.1, goal["speed"])

    cx, cy = 0.0, 0.0
    step   = speed * 0.5       # advance per 500ms tick

    import math

    while True:
        dx   = tx - cx
        dy   = ty - cy
        dist = math.sqrt(dx*dx + dy*dy)

        if dist <= step:
            cx, cy = tx, ty
            # Send final result
            client.publish(CMD_NAV_RESULT, pack_result(True, cx, cy))
            print(f"Executor: REACHED target  final=({cx:.2f}, {cy:.2f})")
            return

        # Move towards target
        cx += step * dx / dist
        cy += step * dy / dist
        dist -= step

        client.publish(CMD_NAV_FEEDBACK, pack_feedback(cx, cy, dist))
        print(f"  [FEEDBACK sent]  pos=({cx:.2f}, {cy:.2f})  rem={dist:.2f}m")
        time.sleep(0.5)

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="SerialComm mission example")
    p.add_argument("--port", default="/dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--role", choices=["caller", "executor"], default="caller")
    return p.parse_args()


def main() -> None:
    args = _parse_args()
    with SerialCommClient(port=args.port, baudrate=args.baud) as client:
        if args.role == "caller":
            run_caller(client)
        else:
            run_executor(client)


if __name__ == "__main__":
    main()
