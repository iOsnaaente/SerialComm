"""
vsss_controller.py — VSSS bridge controller example

This script connects to the VSSS ESP-NOW bridge via UART and:
  • Sends RobotVelocityCmd events to individual robots or all at once.
  • Receives and prints RobotTelemetry events from all robots.
  • Optionally calls the SetMotor service.

Run with:
    python vsss_controller.py --port /dev/ttyUSB0 --baud 115200

Message formats are auto-generated from the IDL in user_app/ — see:
    include/serial_comm/generated/python/
"""

import argparse
import logging
import sys
import time

# Add parent directory to path so the script can be run from any location.
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# Auto-generated serializers live alongside the C++ headers
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "include", "serial_comm", "generated"
))

from serial_comm_py import SerialCommClient, Command, DeliveryMode, Packet
from python import (
    pack_robot_velocity_cmd,
    unpack_robot_telemetry,
    pack_set_motor_request,
    unpack_set_motor_response,
)

logging.basicConfig(
    level  = logging.INFO,
    format = "%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
log = logging.getLogger("vsss")

# ---------------------------------------------------------------------------
# Command IDs (must match firmware @id values in the IDL)
# ---------------------------------------------------------------------------

VEL_CMD = Command(0x10)   # RobotVelocityCmd
TEL_CMD = Command(0x11)   # RobotTelemetry
MOT_CMD = Command(0x14)   # SetMotor request (@id 0x14 from SetMotor.request IDL)

ROBOT_ID_ALL = 0xFF       # broadcast to all robots

NUM_ROBOTS = 3


# ---------------------------------------------------------------------------
# Controller class
# ---------------------------------------------------------------------------

class VSSSController:
    """
    High-level controller for the VSSS bridge.

    All calls to ``send_velocity`` are thread-safe.
    Telemetry callbacks run in the SerialCommClient RX thread — keep them fast.
    """

    def __init__(self, port: str, baudrate: int = 921600) -> None:
        self._client = SerialCommClient(
            port     = port,
            baudrate = baudrate,
            inter_byte_timeout_chars = 3.5,
            read_timeout = 0.001,
        )
        self._telemetry: dict[int, dict] = {}
        self._telem_lock = __import__("threading").Lock()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        self._client.start()
        self._client.subscribe(TEL_CMD, self._on_telemetry)
        log.info("VSSS Controller ready — subscribing to RobotTelemetry (cmd=0x11)")

    def stop(self) -> None:
        self._client.stop()

    def __enter__(self) -> "VSSSController":
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.stop()

    # ------------------------------------------------------------------
    # Velocity control
    # ------------------------------------------------------------------

    def send_velocity(
        self,
        robot_id: int,
        vl:       float,
        vr:       float,
    ) -> None:
        """
        Send a wheel velocity command to one robot or all robots.

        Parameters
        ----------
        robot_id : int
            0, 1, or 2 for a specific robot; ``ROBOT_ID_ALL`` (0xFF) to
            broadcast to all three.
        vl : float
            Left-wheel velocity (units depend on firmware motor driver).
        vr : float
            Right-wheel velocity.
        """
        payload = pack_robot_velocity_cmd({"robot_id": robot_id & 0xFF, "vl": vl, "vr": vr})
        rc = self._client.publish(VEL_CMD, payload, DeliveryMode.BEST_EFFORT)
        if robot_id == ROBOT_ID_ALL:
            log.debug("VelCmd → ALL  vl=%.2f vr=%.2f  rc=%s", vl, vr, rc.name)
        else:
            log.debug("VelCmd → Robot%d  vl=%.2f vr=%.2f  rc=%s",
                      robot_id, vl, vr, rc.name)

    def stop_all(self) -> None:
        """Send zero velocity to all robots."""
        self.send_velocity(ROBOT_ID_ALL, 0.0, 0.0)

    # ------------------------------------------------------------------
    # Service calls
    # ------------------------------------------------------------------

    def set_motor(
        self,
        motor_id: int,
        speed:    float,
        enable:   bool,
        timeout:  float = 2.0,
    ) -> tuple[bool, float]:
        """
        Call the SetMotor service on the bridge.

        Returns (success, actual_speed).
        """
        payload = pack_set_motor_request({"motor_id": motor_id & 0xFF, "speed": speed, "enable": enable})
        # call_service is always reliable: we're waiting for a reply
        ok, resp = self._client.call_service(MOT_CMD, payload, timeout=timeout)
        if ok:
            r, _ = unpack_set_motor_response(resp)
            return r.get("success", False), r.get("actual_speed", 0.0)
        return False, 0.0

    def ping(self, timeout: float = 1.0) -> float | None:
        """Measure bridge round-trip latency in ms."""
        return self._client.ping(timeout=timeout)

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    def get_telemetry(self, robot_id: int) -> dict:
        """Return the most recent telemetry received from robot N."""
        with self._telem_lock:
            return dict(self._telemetry.get(robot_id, {}))

    def _on_telemetry(self, pkt: Packet) -> None:
        if len(pkt.payload) < 22:
            log.warning("Malformed telemetry payload (%d bytes)", len(pkt.payload))
            return
        tel, _ = unpack_robot_telemetry(pkt.payload)
        rid = tel["robot_id"]
        with self._telem_lock:
            self._telemetry[rid] = tel
        log.debug(
            "Telemetry Robot%d  ax=%.2f ay=%.2f gz=%.2f  vl=%.2f vr=%.2f  ball=%s",
            rid, tel["ax"], tel["ay"], tel["gz"],
            tel["vl_measured"], tel["vr_measured"], tel["ball_detected"],
        )

    # ------------------------------------------------------------------
    # Stats
    # ------------------------------------------------------------------

    def stats(self) -> dict:
        return self._client.stats()


# ---------------------------------------------------------------------------
# Demo main — sends alternating speed commands and prints telemetry
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="VSSS bridge controller demo")
    p.add_argument("--port",  default="/dev/ttyUSB0", help="Serial port")
    p.add_argument("--baud",  type=int, default=921600, help="Baud rate")
    p.add_argument("--hz",    type=float, default=10.0, help="Command rate (Hz)")
    p.add_argument("--debug", action="store_true", help="Enable debug logging")
    return p.parse_args()


def main() -> None:
    args = _parse_args()
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    with VSSSController(port=args.port, baudrate=args.baud) as ctrl:
        rtt = ctrl.ping(timeout=2.0)
        if rtt is not None:
            log.info("Bridge ping OK  RTT=%.1f ms", rtt)
        else:
            log.warning("Bridge did not reply to PING — continuing anyway")

        period = 1.0 / args.hz
        tick   = 0

        log.info("Sending velocity commands at %.0f Hz — Ctrl+C to quit", args.hz)

        try:
            while True:
                t0 = time.monotonic()

                # Simple alternating pattern: forward / rotate / stop
                phase = tick % 30
                if phase < 10:
                    vl, vr = 1.0, 1.0        # forward
                elif phase < 20:
                    vl, vr = 0.5, -0.5       # rotate right
                else:
                    vl, vr = 0.0, 0.0        # stop

                # Send to each robot individually
                for rid in range(NUM_ROBOTS):
                    ctrl.send_velocity(rid, vl, vr)

                # Print latest telemetry from every robot
                for rid in range(NUM_ROBOTS):
                    tel = ctrl.get_telemetry(rid)
                    if tel:
                        print(
                            f"Robot{rid}  "
                            f"ax={tel['ax']:+.2f}  ay={tel['ay']:+.2f}  "
                            f"gz={tel['gz']:+.2f}  "
                            f"vl={tel['vl_measured']:.2f}  "
                            f"vr={tel['vr_measured']:.2f}  "
                            f"ball={'YES' if tel['ball_detected'] else ' no'}",
                            flush=True,
                        )

                tick += 1
                elapsed = time.monotonic() - t0
                wait    = period - elapsed
                if wait > 0:
                    time.sleep(wait)

        except KeyboardInterrupt:
            log.info("Stopping...")
            ctrl.stop_all()


if __name__ == "__main__":
    main()
