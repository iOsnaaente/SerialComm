"""
01_event.py — Publishing and subscribing to Events

An Event is a fire-and-forget message with no reply (topic / pub-sub).
This maps to ``publish()`` on the sender and ``subscribe()`` on the receiver.

IDL equivalent (user_app/event/SensorReading.event):
    @best_effort
    float32 temperature
    float32 humidity
    uint32  timestamp_ms
    GpsCoord location   ← embedded struct: float32 lat, lon, alt

Wire layout (little-endian):
    offset  0  float32  temperature     4 bytes
    offset  4  float32  humidity        4 bytes
    offset  8  uint32   timestamp_ms    4 bytes
    offset 12  float32  lat             4 bytes
    offset 16  float32  lon             4 bytes
    offset 20  float32  alt             4 bytes
    total = 24 bytes

Run two terminals on the same machine (TX→RX loopback or two boards):
    terminal A (publisher):   python 01_event.py --port /dev/ttyUSB0 --role pub
    terminal B (subscriber):  python 01_event.py --port /dev/ttyUSB1 --role sub
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
# Command ID — must match the firmware @id (or any agreed-upon byte)
# ---------------------------------------------------------------------------
CMD_SENSOR_READING = Command(0x20)

# ---------------------------------------------------------------------------
# Serialization — mirrors the generated C++ SerialCommSerializer<SensorReading>
# ---------------------------------------------------------------------------
_FMT = '<ffIffff'   # temperature, humidity, timestamp_ms, lat, lon, alt
_SIZE = struct.calcsize(_FMT)  # 24 bytes


def pack_sensor_reading(
    temperature: float,
    humidity: float,
    timestamp_ms: int,
    lat: float = 0.0,
    lon: float = 0.0,
    alt: float = 0.0,
) -> bytes:
    return struct.pack(_FMT, temperature, humidity, timestamp_ms, lat, lon, alt)


def unpack_sensor_reading(payload: bytes) -> dict:
    if len(payload) < _SIZE:
        return {}
    t, h, ts, lat, lon, alt = struct.unpack_from(_FMT, payload)
    return {
        "temperature": t,
        "humidity":    h,
        "timestamp_ms": ts,
        "lat": lat,
        "lon": lon,
        "alt": alt,
    }


# ---------------------------------------------------------------------------
# Publisher side
# ---------------------------------------------------------------------------

def run_publisher(client: SerialCommClient, hz: float = 1.0) -> None:
    period = 1.0 / hz
    tick   = 0
    print(f"Publisher: sending SensorReading @ {hz:.0f} Hz on cmd=0x{int(CMD_SENSOR_READING):02X}")

    while True:
        now_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
        payload = pack_sensor_reading(
            temperature  = 20.0 + (tick % 20) * 0.5,
            humidity     = 60.0 + (tick % 10) * 1.0,
            timestamp_ms = now_ms,
            lat          = -23.5505,
            lon          = -46.6333,
            alt          = 760.0,
        )

        rc = client.publish(CMD_SENSOR_READING, payload)
        print(
            f"  TX tick={tick:04d}  T={20.0 + (tick%20)*0.5:.1f}°C  "
            f"H={60.0 + (tick%10)*1.0:.1f}%  rc={rc.name}"
        )
        tick += 1
        time.sleep(period)


# ---------------------------------------------------------------------------
# Subscriber side
# ---------------------------------------------------------------------------

def run_subscriber(client: SerialCommClient) -> None:
    print(f"Subscriber: waiting for SensorReading on cmd=0x{int(CMD_SENSOR_READING):02X}")

    def on_reading(pkt: Packet) -> None:
        r = unpack_sensor_reading(pkt.payload)
        if not r:
            print(f"  RX malformed payload ({len(pkt.payload)} bytes)")
            return
        print(
            f"  RX seq={pkt.seq_id:#06x}  "
            f"T={r['temperature']:.1f}°C  H={r['humidity']:.1f}%  "
            f"ts={r['timestamp_ms']}ms  "
            f"GPS=({r['lat']:.4f}, {r['lon']:.4f}, {r['alt']:.0f}m)"
        )

    client.subscribe(CMD_SENSOR_READING, on_reading)

    # Block forever — callbacks run in the background RX thread
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="SerialComm event example")
    p.add_argument("--port", default="/dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--role", choices=["pub", "sub"], default="pub")
    p.add_argument("--hz",   type=float, default=1.0, help="Publish rate (pub role)")
    return p.parse_args()


def main() -> None:
    args = _parse_args()

    with SerialCommClient(port=args.port, baudrate=args.baud) as client:
        if args.role == "pub":
            run_publisher(client, hz=args.hz)
        else:
            run_subscriber(client)


if __name__ == "__main__":
    main()
