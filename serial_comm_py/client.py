"""
SerialCommClient — Python-side implementation of the SerialComm middleware.

Provides the same primitives as ``SerialCommManager`` on the firmware side:

  • ``publish(cmd, payload)``         — fire-and-forget (topic / event)
  • ``subscribe(cmd, callback)``      — incoming packet handler
  • ``call_service(cmd, payload)``    — blocking request / reply
  • ``serve(cmd, handler)``           — register a Python-side service server
  • ``ping()``                        — round-trip latency measurement

Threading model
---------------
A single background daemon thread reads bytes from the serial port, feeds
them to ``StreamParser``, applies the inter-byte timeout, and dispatches
completed packets.  All public methods are thread-safe.

Timing
------
Inter-byte timeout mirrors the firmware's hardware watchdog:
  timeout_us = 10_000_000 / baudrate * chars   (default chars = 3.5)

If no byte arrives within that window while a partial frame is in progress,
the parser is reset — exactly as the firmware does.
"""

from __future__ import annotations

import logging
import struct
import threading
import time
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional

import serial

from ._types import (
    Command,
    Packet,
    ResultCode,
    REPLY_MASK,
    make_reply,
    make_request,
    is_reply,
    PROTOCOL_VERSION_V1,
)
from ._protocol import encode as _encode_packet
from ._parser import StreamParser

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Type aliases
# ---------------------------------------------------------------------------

RxCallback   = Callable[[Packet], None]
ServiceHandler = Callable[[bytes], bytes]   # (request_payload) -> response_payload


# ---------------------------------------------------------------------------
# Internal: pending service call
# ---------------------------------------------------------------------------

@dataclass
class _PendingCall:
    seq_id:    int
    reply_cmd: int                 # expected reply command byte
    event:     threading.Event = field(default_factory=threading.Event)
    response:  Optional[bytes] = None
    timed_out: bool = False


# ---------------------------------------------------------------------------
# SerialCommClient
# ---------------------------------------------------------------------------

class SerialCommClient:
    """
    Python-side SerialComm middleware client.

    Parameters
    ----------
    port : str
        Serial port path (e.g. ``"/dev/ttyUSB0"`` or ``"COM3"``).
    baudrate : int
        Must match the firmware's ``CONFIG_SERIAL_COMM_UART_BAUDRATE``.
    inter_byte_timeout_chars : float
        Number of character times before an incomplete frame is discarded.
        The firmware default is 3.5 characters.  Set to 0 to disable.
    read_chunk : int
        Bytes to request per ``serial.read()`` call (performance tuning only).
    """

    def __init__(
        self,
        port:                       str,
        baudrate:                   int   = 115200,
        inter_byte_timeout_chars:   float = 3.5,
        read_chunk:                 int   = 256,
    ) -> None:
        self._port     = port
        self._baudrate = baudrate
        self._read_chunk = read_chunk

        # Inter-byte timeout in seconds
        byte_time_s = 10.0 / baudrate
        self._interbyte_timeout_s: float = (
            byte_time_s * inter_byte_timeout_chars
            if inter_byte_timeout_chars > 0
            else 0.0
        )

        self._serial:  Optional[serial.Serial] = None
        self._parser   = StreamParser()
        self._seq_id   = 0
        self._seq_lock = threading.Lock()
        self._tx_lock  = threading.Lock()

        # Subscriptions: cmd byte → list of callbacks
        self._subscriptions: Dict[int, List[RxCallback]] = defaultdict(list)
        self._sub_lock = threading.Lock()

        # Service servers: cmd byte → handler
        self._services: Dict[int, ServiceHandler] = {}
        self._svc_lock = threading.Lock()

        # Pending call_service futures: seq_id → _PendingCall
        self._pending: Dict[int, _PendingCall] = {}
        self._pending_lock = threading.Lock()

        # Statistics
        self._stats = {
            "tx_packets":   0,
            "rx_packets":   0,
            "crc_errors":   0,
            "timeouts":     0,
            "parser_resets": 0,
        }
        self._stats_lock = threading.Lock()

        self._rx_thread:  Optional[threading.Thread] = None
        self._stop_event  = threading.Event()
        self._running     = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        """
        Open the serial port and start the background RX thread.

        Raises ``serial.SerialException`` on port open failure.
        """
        if self._running:
            return

        self._serial = serial.Serial(
            port     = self._port,
            baudrate = self._baudrate,
            bytesize = serial.EIGHTBITS,
            parity   = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE,
            timeout  = 0.005,      # 5 ms read timeout → responsive inter-byte check
        )

        self._stop_event.clear()
        self._running    = True
        self._rx_thread  = threading.Thread(
            target = self._rx_loop,
            name   = "SerialComm-RX",
            daemon = True,
        )
        self._rx_thread.start()
        log.info(
            "SerialCommClient started on %s @ %d baud  "
            "(inter-byte timeout %.0f µs)",
            self._port, self._baudrate,
            self._interbyte_timeout_s * 1e6,
        )

    def stop(self) -> None:
        """Stop the background thread and close the serial port."""
        if not self._running:
            return
        self._stop_event.set()
        if self._rx_thread:
            self._rx_thread.join(timeout=2.0)
        if self._serial and self._serial.is_open:
            self._serial.close()
        self._running = False
        log.info("SerialCommClient stopped")

    def __enter__(self) -> "SerialCommClient":
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.stop()

    # ------------------------------------------------------------------
    # Topic — publish (fire and forget)
    # ------------------------------------------------------------------

    def publish(
        self,
        command: int | Command,
        payload: bytes | bytearray = b"",
    ) -> ResultCode:
        """
        Send a packet without waiting for a reply.

        Equivalent to ``SerialCommManager::publish()`` on the firmware.

        Parameters
        ----------
        command : Command or int
            Command byte.  Use ``Command`` enum for standard commands or
            a plain int for user-defined ``@id`` values (e.g. ``0x10``).
        payload : bytes
            Serialized message payload.
        """
        if not self._running:
            return ResultCode.ERR_NOT_INITIALIZED
        try:
            seq = self._next_seq_id()
            frame = _encode_packet(seq, int(command), payload)
            self._write_frame(frame)
            with self._stats_lock:
                self._stats["tx_packets"] += 1
            log.debug("TX  seq=%04x  cmd=%02x  len=%d", seq, int(command), len(payload))
            return ResultCode.OK
        except serial.SerialException as exc:
            log.error("publish() serial error: %s", exc)
            return ResultCode.ERR_IO

    # ------------------------------------------------------------------
    # Service — call (blocking request / reply)
    # ------------------------------------------------------------------

    def call_service(
        self,
        command:  int | Command,
        payload:  bytes | bytearray = b"",
        timeout:  float = 2.0,
    ) -> tuple[bool, bytes]:
        """
        Send a request packet and block until the matching reply arrives.

        Equivalent to ``SerialCommManager::call_service()`` on the firmware.

        Returns
        -------
        (success, response_payload) :
            ``success`` is ``True`` if a reply arrived within *timeout* seconds.
            ``response_payload`` is the payload of the reply packet (empty on timeout).

        Example
        -------
        ::

            ok, resp = client.call_service(Command.WRITE, req_bytes, timeout=1.0)
            if ok:
                success, actual_speed = struct.unpack('<Bf', resp[:5])
        """
        if not self._running:
            return False, b""

        seq       = self._next_seq_id()
        cmd_int   = int(command)
        reply_cmd = make_reply(cmd_int)

        future = _PendingCall(seq_id=seq, reply_cmd=reply_cmd)
        with self._pending_lock:
            self._pending[seq] = future

        try:
            frame = _encode_packet(seq, cmd_int, payload)
            self._write_frame(frame)
            with self._stats_lock:
                self._stats["tx_packets"] += 1
            log.debug(
                "CALL  seq=%04x  cmd=%02x → reply=%02x  timeout=%.1fs",
                seq, cmd_int, reply_cmd, timeout,
            )
        except serial.SerialException as exc:
            log.error("call_service() send error: %s", exc)
            with self._pending_lock:
                self._pending.pop(seq, None)
            return False, b""

        got_reply = future.event.wait(timeout)
        with self._pending_lock:
            self._pending.pop(seq, None)

        if got_reply and future.response is not None:
            log.debug(
                "REPLY  seq=%04x  cmd=%02x  len=%d",
                seq, reply_cmd, len(future.response),
            )
            return True, future.response

        with self._stats_lock:
            self._stats["timeouts"] += 1
        log.warning("call_service() timeout  seq=%04x  cmd=%02x", seq, cmd_int)
        return False, b""

    # ------------------------------------------------------------------
    # Topic — subscribe
    # ------------------------------------------------------------------

    def subscribe(
        self,
        command:  int | Command,
        callback: RxCallback,
    ) -> None:
        """
        Register a callback for incoming packets with the given command byte.

        Multiple callbacks can be registered for the same command.
        Callbacks are called synchronously in the RX thread — keep them fast.

        Parameters
        ----------
        command : Command or int
        callback : Callable[[Packet], None]
        """
        with self._sub_lock:
            self._subscriptions[int(command)].append(callback)
        log.debug("subscribe cmd=%02x  callback=%s", int(command), callback.__name__)

    def unsubscribe(self, command: int | Command, callback: RxCallback) -> None:
        """Remove a previously registered subscription callback."""
        with self._sub_lock:
            lst = self._subscriptions.get(int(command), [])
            try:
                lst.remove(callback)
            except ValueError:
                pass

    # ------------------------------------------------------------------
    # Service — serve (Python acts as a server)
    # ------------------------------------------------------------------

    def serve(
        self,
        command: int | Command,
        handler: ServiceHandler,
    ) -> None:
        """
        Register a service handler.  When the firmware sends a request with
        *command*, *handler* is called with the request payload and its
        return value is sent back as the reply.

        Equivalent to ``SerialCommManager::create_service()`` on the firmware.

        Parameters
        ----------
        command : Command or int
        handler : Callable[[bytes], bytes]
            Receives the request payload; returns the response payload.
        """
        with self._svc_lock:
            self._services[int(command)] = handler
        log.debug("serve cmd=%02x  handler=%s", int(command), handler.__name__)

    def unserve(self, command: int | Command) -> None:
        """Remove a registered service handler."""
        with self._svc_lock:
            self._services.pop(int(command), None)

    # ------------------------------------------------------------------
    # Ping
    # ------------------------------------------------------------------

    def ping(self, timeout: float = 1.0) -> Optional[float]:
        """
        Send a PING and measure round-trip time.

        Returns the round-trip time in **milliseconds**, or ``None`` on timeout.
        """
        t0 = time.monotonic()
        ok, _ = self.call_service(Command.PING, b"", timeout=timeout)
        if ok:
            return (time.monotonic() - t0) * 1000.0
        return None

    # ------------------------------------------------------------------
    # Statistics / diagnostics
    # ------------------------------------------------------------------

    def stats(self) -> dict:
        """Return a snapshot of RX/TX statistics."""
        with self._stats_lock:
            s = dict(self._stats)
        s["parser_rx"]        = self._parser.rx_count
        s["parser_crc_errors"] = self._parser.crc_error_count
        s["parser_ok"]        = self._parser.ok_count
        return s

    def is_running(self) -> bool:
        return self._running

    # ------------------------------------------------------------------
    # Private — sequence ID
    # ------------------------------------------------------------------

    def _next_seq_id(self) -> int:
        with self._seq_lock:
            sid = self._seq_id
            self._seq_id = (self._seq_id + 1) & 0xFFFF
        return sid

    # ------------------------------------------------------------------
    # Private — TX
    # ------------------------------------------------------------------

    def _write_frame(self, frame: bytes) -> None:
        assert self._serial is not None
        with self._tx_lock:
            self._serial.write(frame)

    # ------------------------------------------------------------------
    # Private — RX loop (background thread)
    # ------------------------------------------------------------------

    def _rx_loop(self) -> None:
        """
        Background thread: reads bytes, feeds parser, applies inter-byte
        timeout, dispatches completed packets.
        """
        parser  = self._parser
        timeout = self._interbyte_timeout_s
        ser     = self._serial
        assert ser is not None

        while not self._stop_event.is_set():
            try:
                data = ser.read(self._read_chunk)
            except serial.SerialException as exc:
                log.error("RX serial error: %s", exc)
                self._stop_event.set()
                break

            now = time.monotonic()

            if data:
                # Apply inter-byte timeout: if too much time passed since the
                # last byte while a partial frame is in progress, reset parser.
                if (
                    timeout > 0
                    and parser.last_byte_time > 0
                    and (now - parser.last_byte_time) > timeout
                    and parser.state_name not in ("WAIT_HEADER_0",)
                ):
                    log.debug(
                        "Inter-byte timeout (%.0f µs) — parser reset",
                        timeout * 1e6,
                    )
                    parser.reset()
                    with self._stats_lock:
                        self._stats["parser_resets"] += 1

                packets = parser.feed(data)
                for pkt in packets:
                    with self._stats_lock:
                        self._stats["rx_packets"] += 1
                    self._dispatch(pkt)

            else:
                # No data — check if a partial frame has been waiting too long.
                if (
                    timeout > 0
                    and parser.last_byte_time > 0
                    and (now - parser.last_byte_time) > timeout
                    and parser.state_name not in ("WAIT_HEADER_0",)
                ):
                    log.debug("Inter-byte timeout — parser reset (idle)")
                    parser.reset()
                    with self._stats_lock:
                        self._stats["parser_resets"] += 1

    # ------------------------------------------------------------------
    # Private — packet dispatcher
    # ------------------------------------------------------------------

    def _dispatch(self, pkt: Packet) -> None:
        """
        Route a completed packet to the right handler:

        1. If it is a *reply* (bit 7 set) → look up a pending call_service.
        2. If we have a *service handler* registered for this command → call
           it and send the reply automatically.
        3. Dispatch to all registered subscription callbacks.
        """
        cmd = pkt.command
        log.debug("RX  seq=%04x  cmd=%02x  len=%d", pkt.seq_id, cmd, len(pkt.payload))

        if is_reply(cmd):
            # --- Match to a pending call_service ---
            with self._pending_lock:
                future = self._pending.get(pkt.seq_id)
            if future and future.reply_cmd == cmd:
                future.response = pkt.payload
                future.event.set()
                return   # reply consumed — don't forward to subscriptions

        else:
            # --- Service server auto-reply ---
            with self._svc_lock:
                handler = self._services.get(cmd)
            if handler:
                try:
                    resp_payload = handler(pkt.payload)
                except Exception as exc:
                    log.error("Service handler cmd=%02x raised: %s", cmd, exc)
                    resp_payload = b""
                try:
                    frame = _encode_packet(
                        pkt.seq_id,
                        make_reply(cmd),
                        resp_payload,
                    )
                    self._write_frame(frame)
                    with self._stats_lock:
                        self._stats["tx_packets"] += 1
                except serial.SerialException as exc:
                    log.error("Service reply serial error: %s", exc)

        # --- Subscription callbacks ---
        with self._sub_lock:
            callbacks = list(self._subscriptions.get(cmd, []))
        for cb in callbacks:
            try:
                cb(pkt)
            except Exception as exc:
                log.error("Subscription callback cmd=%02x raised: %s", cmd, exc)
