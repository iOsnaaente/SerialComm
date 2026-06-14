"""
serial_comm_py — Python implementation of the SerialComm middleware protocol.

Quick start
-----------
::

    from serial_comm_py import SerialCommClient, Command
    import struct

    client = SerialCommClient(port='/dev/ttyUSB0', baudrate=115200)
    client.start()

    # Fire-and-forget event (no reply expected)
    payload = struct.pack('<Bff', robot_id, vl, vr)
    client.publish(Command(0x10), payload)   # RobotVelocityCmd

    # Subscribe to incoming events
    def on_telemetry(packet):
        robot_id, ax, ay, gz, vl, vr, ball = struct.unpack('<BfffffB', packet.payload)
        print(f"Robot{robot_id}: ax={ax:.2f}  vl={vl:.2f}  vr={vr:.2f}")

    client.subscribe(Command(0x11), on_telemetry)   # RobotTelemetry

    # Blocking request / reply (service call)
    ok, resp = client.call_service(Command.WRITE, req_payload, timeout=2.0)

    # PING round-trip
    rtt_ms = client.ping()

    client.stop()
"""

from .client   import SerialCommClient, RxCallback, ServiceHandler
from ._types   import Command, Packet, ResultCode, DeliveryMode
from ._crc     import compute_crc16, has_numba
from ._protocol import encode as encode_packet, decode as decode_packet
from ._parser  import StreamParser

__all__ = [
    # Main class
    "SerialCommClient",
    # Types
    "Command",
    "Packet",
    "ResultCode",
    "DeliveryMode",
    # Protocol utilities
    "compute_crc16",
    "encode_packet",
    "decode_packet",
    "StreamParser",
    "has_numba",
    # Callback type aliases
    "RxCallback",
    "ServiceHandler",
]

__version__ = "0.1.0"
__author__  = "Bruno Gabriel Flores Sampaio"
