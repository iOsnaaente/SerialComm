"""
05_vsss_udp_controller.py — VSSS robot control via UDP transport

Demonstrates ``UDPSerialCommClient`` sending ``RobotVelocityCmd`` packets
to an ESP32 bridge over WiFi/UDP and receiving ``RobotTelemetry`` back.

Hardware setup
--------------

  ESP32 bridge (examples/vsss_udp_bridge, NODE_TYPE=0)
    • connects to your WiFi network
    • listens on UDP port 4210 for SerialComm-framed packets from this script
    • sends telemetry back to this machine on UDP port 4211

  ESP32 robots (examples/vsss_udp_bridge, NODE_TYPE=1/2/3)
    • communicate with the bridge via ESP-NOW

Usage
-----

  python serial_comm_py/examples/05_vsss_udp_controller.py \
      --bridge-ip 192.168.1.42 \
      --robot-id 0

  # Spin robot 0 in place
  python .../05_vsss_udp_controller.py --bridge-ip 192.168.1.42 --robot-id 0 \
      --vl 1.5 --vr -1.5 --duration 3

Protocol
--------

  PC (this script) ──UDP:4210──▶ Bridge ESP32 ──ESP-NOW──▶ Robot 0/1/2
  PC               ◀──UDP:4211── Bridge ESP32 ◀─────────── Robot telemetry
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
import time

_REPO_ROOT = os.path.join(os.path.dirname(__file__), '..', '..')
sys.path.insert(0, _REPO_ROOT)
sys.path.insert(0, os.path.join(_REPO_ROOT, "include", "serial_comm", "generated"))

from serial_comm_py import UDPSerialCommClient, Packet, DeliveryMode
from serial_comm_py.generated_constants import (
    ROBOT_VELOCITY_CMD_ID,
    ROBOT_TELEMETRY_ID,
    DELIVERY_MODE,
    MAX_RATE_HZ,
)
from python import pack_robot_velocity_cmd, unpack_robot_telemetry

log = logging.getLogger(__name__)

VEL_CMD = ROBOT_VELOCITY_CMD_ID
TEL_CMD = ROBOT_TELEMETRY_ID


def on_telemetry(pkt: Packet) -> None:
    tel, _ = unpack_robot_telemetry(pkt.payload)
    print(
        f"  TEL  robot={tel['robot_id']}  "
        f"ax={tel['ax']:+.2f}  ay={tel['ay']:+.2f}  gz={tel['gz']:+.2f}  "
        f"vl={tel['vl_measured']:+.2f}  vr={tel['vr_measured']:+.2f}  "
        f"ball={'YES' if tel['ball_detected'] else 'no'}"
    )


def run(args: argparse.Namespace) -> None:
    client = UDPSerialCommClient(
        bridge_ip=args.bridge_ip,
        bridge_port=args.bridge_port,
        local_port=args.local_port,
    )

    with client:
        # Apply rate limit from IDL @max_rate_hz
        rate = MAX_RATE_HZ.get(int(VEL_CMD), 33.0)
        client.set_rate_limit(VEL_CMD, rate)

        # Subscribe to telemetry from any robot
        client.subscribe(TEL_CMD, on_telemetry)

        print(
            f"Connected to bridge at {args.bridge_ip}:{args.bridge_port}  "
            f"(recv on :{args.local_port})"
        )
        print(f"Sending to robot_id={args.robot_id}  "
              f"vl={args.vl:.2f}  vr={args.vr:.2f}  "
              f"duration={args.duration}s  rate={rate}Hz")

        interval  = 1.0 / rate
        deadline  = time.monotonic() + args.duration
        tick      = 0

        while time.monotonic() < deadline:
            payload = pack_robot_velocity_cmd({
                "robot_id": args.robot_id & 0xFF,
                "vl": args.vl,
                "vr": args.vr,
            })
            mode = DELIVERY_MODE.get(int(VEL_CMD), DeliveryMode.BEST_EFFORT)
            rc   = client.publish(VEL_CMD, payload, delivery_mode=mode)

            if tick % int(rate) == 0:  # log once per second
                print(f"  VEL  tick={tick:4d}  rc={rc.name}")

            tick += 1
            time.sleep(interval)

        # Stop motors
        stop_payload = pack_robot_velocity_cmd({
            "robot_id": args.robot_id & 0xFF,
            "vl": 0.0,
            "vr": 0.0,
        })
        client.publish(VEL_CMD, stop_payload)
        print("Motors stopped.")

        # Keep listening to telemetry for a moment after stopping
        time.sleep(1.0)

        stats = client.stats()
        print(
            f"\nStats: tx={stats['tx_packets']}  rx={stats['rx_packets']}  "
            f"crc_errors={stats['crc_errors']}"
        )


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="VSSS UDP controller demo")
    p.add_argument("--bridge-ip",   required=True,     help="Bridge ESP32 IP address")
    p.add_argument("--bridge-port", type=int, default=4210, help="Bridge UDP listen port")
    p.add_argument("--local-port",  type=int, default=4211, help="Local port for telemetry RX")
    p.add_argument("--robot-id",    type=int, default=0,    help="Target robot (0/1/2 or 255=all)")
    p.add_argument("--vl",          type=float, default=1.0, help="Left wheel velocity (m/s)")
    p.add_argument("--vr",          type=float, default=1.0, help="Right wheel velocity (m/s)")
    p.add_argument("--duration",    type=float, default=5.0, help="Duration in seconds")
    p.add_argument("--verbose",     action="store_true")
    return p.parse_args()


def main() -> None:
    args = _parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s  %(levelname)s  %(name)s  %(message)s",
    )
    run(args)


if __name__ == "__main__":
    main()
