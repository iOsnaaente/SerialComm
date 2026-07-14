"""
UDPSerialCommClient — SerialComm middleware over UDP (WiFi transport).

Extends ``SerialCommClient`` with a UDP socket transport layer.  All
publish/subscribe/call_service/serve/ping APIs work identically to the
UART-based client — only the physical transport changes.

Typical use with the vsss_udp_bridge example
--------------------------------------------

  PC ──UDP──▶ Bridge ESP32 (NODE_TYPE=0) ──ESP-NOW──▶ Robots
  PC ◀────── Bridge ESP32               ◀─────────── Robots (telemetry)

The bridge listens on ``BRIDGE_UDP_PORT`` (default 4210).
The bridge sends telemetry back to ``<pc_ip>:PC_LISTEN_PORT`` (default 4211).

Usage
-----

  from serial_comm_py.udp_client import UDPSerialCommClient
  from serial_comm_py.generated_constants import ROBOT_VELOCITY_CMD_ID

  client = UDPSerialCommClient(
      bridge_ip="192.168.1.42",
      bridge_port=4210,
      local_port=4211,
  )
  with client:
      payload = pack_robot_velocity_cmd({"robot_id": 0, "vl": 1.0, "vr": -1.0})
      client.publish(ROBOT_VELOCITY_CMD_ID, payload)

Notes
-----

• UDP is message-oriented (no byte-stream framing concerns), but SerialComm's
  wire format uses the same ``[0xAA][0x55][0xAA] + header + CRC`` framing on
  top of UDP datagrams — so the ``StreamParser`` is still used to handle the
  case where multiple SerialComm frames arrive in a single UDP packet.

• Inter-byte timeout is disabled by default (``inter_byte_timeout_chars=0``)
  because UDP datagrams arrive atomically; partial frames across separate
  datagrams are not possible with a well-behaved sender.

• ``call_service()`` and ``serve()`` work over UDP, but UDP's unreliable
  delivery means you should set a reasonable timeout (default 2 s is fine
  for LAN usage).
"""

from __future__ import annotations

import logging
import socket
import threading
import time
from typing import Optional

from .client import SerialCommClient

log = logging.getLogger(__name__)

_DEFAULT_BRIDGE_PORT = 4210
_DEFAULT_LOCAL_PORT  = 4211
_DEFAULT_TIMEOUT_S   = 0.05   # 50 ms socket recv timeout — keeps RX loop responsive


class UDPSerialCommClient(SerialCommClient):
    """
    SerialComm middleware client using a UDP socket instead of a serial port.

    Parameters
    ----------
    bridge_ip : str
        IP address of the bridge ESP32 node.
    bridge_port : int
        UDP port the bridge listens on (default 4210).
    local_port : int
        Local UDP port for receiving telemetry from the bridge (default 4211).
    recv_timeout : float
        Socket receive timeout in seconds (default 0.05).  Controls how quickly
        ``stop()`` is honoured — does not affect normal packet latency.
    """

    def __init__(
        self,
        bridge_ip:    str,
        bridge_port:  int   = _DEFAULT_BRIDGE_PORT,
        local_port:   int   = _DEFAULT_LOCAL_PORT,
        recv_timeout: float = _DEFAULT_TIMEOUT_S,
    ) -> None:
        # Pass a dummy port/baud; super().__init__ only stores them —
        # _write_frame and _rx_loop are fully overridden here.
        super().__init__(
            port="udp",
            baudrate=1,                  # unused; non-zero to avoid division
            inter_byte_timeout_chars=0,  # UDP datagrams arrive atomically
        )
        self._bridge_ip   = bridge_ip
        self._bridge_port = bridge_port
        self._local_port  = local_port
        self._recv_timeout = recv_timeout

        self._sock: Optional[socket.socket] = None

    # ------------------------------------------------------------------
    # Lifecycle overrides
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Open a UDP socket and start the background RX thread."""
        if self._running:
            return

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("", self._local_port))
        self._sock.settimeout(self._recv_timeout)

        self._stop_event.clear()
        self._running   = True
        self._rx_thread = threading.Thread(
            target=self._rx_loop,
            name="SerialComm-UDP-RX",
            daemon=True,
        )
        self._rx_thread.start()
        log.info(
            "UDPSerialCommClient started  bridge=%s:%d  local_port=%d",
            self._bridge_ip, self._bridge_port, self._local_port,
        )

    def stop(self) -> None:
        """Stop the background thread and close the socket."""
        if not self._running:
            return
        self._stop_event.set()
        if self._rx_thread:
            self._rx_thread.join(timeout=2.0)
        if self._sock:
            self._sock.close()
            self._sock = None
        self._running = False
        log.info("UDPSerialCommClient stopped")

    # ------------------------------------------------------------------
    # TX overrides
    # ------------------------------------------------------------------

    def _flush_tx(self) -> None:
        pass  # UDP datagrams are sent atomically — no flush needed

    def _write_frame(self, frame: bytes) -> None:
        assert self._sock is not None
        with self._tx_lock:
            self._sock.sendto(frame, (self._bridge_ip, self._bridge_port))

    # ------------------------------------------------------------------
    # RX loop override
    # ------------------------------------------------------------------

    def _rx_loop(self) -> None:
        """
        Background thread: receives UDP datagrams, feeds each through the
        StreamParser, dispatches completed packets.

        Each UDP datagram may contain one or more SerialComm frames — the
        parser handles reassembly just as it does for the byte-stream case.
        """
        parser = self._parser
        assert self._sock is not None

        while not self._stop_event.is_set():
            try:
                data, _addr = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError as exc:
                if not self._stop_event.is_set():
                    log.error("UDP RX error: %s", exc)
                break

            if data:
                packets = parser.feed(data)
                for pkt in packets:
                    with self._stats_lock:
                        self._stats["rx_packets"] += 1
                    self._dispatch(pkt)
