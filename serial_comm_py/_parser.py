"""
Incremental streaming parser for the SerialComm V1 wire format.

Mirrors SerialCommParser from serial_comm_parser.cpp.

The state machine processes one byte at a time and emits a ``Packet``
when a complete, CRC-valid frame has been accumulated.  Partial frames
are kept in internal state across calls so the caller can feed data in
arbitrarily-sized chunks (single bytes, large DMA bursts, etc.).

Additionally tracks the wall-clock time of the last received byte so the
``SerialCommClient`` can implement the inter-byte timeout that the firmware
applies via its hardware watchdog timer.
"""

from __future__ import annotations

import time
from enum import IntEnum, auto

from ._types import (
    Packet,
    HEADER_BYTES,
    MAX_PAYLOAD_SIZE,
    PROTOCOL_VERSION_V1,
)
from ._crc import compute_crc16


class _State(IntEnum):
    WAIT_HEADER_0 = auto()
    WAIT_HEADER_1 = auto()
    WAIT_HEADER_2 = auto()
    GET_SEQ_ID_L  = auto()
    GET_SEQ_ID_H  = auto()
    READ_VERSION  = auto()
    READ_COMMAND  = auto()
    READ_LEN_L    = auto()
    READ_LEN_H    = auto()
    READ_PAYLOAD  = auto()
    READ_CRC_L    = auto()
    READ_CRC_H    = auto()


class StreamParser:
    """
    Byte-at-a-time streaming parser.

    Usage::

        parser = StreamParser()

        for chunk in serial_port:
            packets = parser.feed(chunk)
            for pkt in packets:
                handle(pkt)

    The parser is *not* thread-safe — protect with a lock when feeding
    from multiple threads.
    """

    __slots__ = (
        "_state",
        "_seq_id",
        "_version",
        "_command",
        "_payload_len",
        "_payload",
        "_payload_idx",
        "_crc_low",
        "_last_byte_time",
        "_rx_count",
        "_crc_err_count",
        "_ok_count",
    )

    def __init__(self) -> None:
        self._reset()
        self._rx_count:      int   = 0
        self._crc_err_count: int   = 0
        self._ok_count:      int   = 0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def feed(self, data: bytes | bytearray) -> list[Packet]:
        """
        Feed a chunk of raw bytes.

        Returns a list (possibly empty) of fully decoded ``Packet`` objects.
        """
        packets: list[Packet] = []
        for b in data:
            self._rx_count += 1
            self._last_byte_time = time.monotonic()
            pkt = self._feed_byte(b)
            if pkt is not None:
                packets.append(pkt)
        return packets

    def feed_byte(self, b: int) -> Packet | None:
        """
        Feed a single byte.

        Returns a ``Packet`` if this byte completed a valid frame, else ``None``.
        """
        self._rx_count += 1
        self._last_byte_time = time.monotonic()
        return self._feed_byte(b)

    def reset(self) -> None:
        """Reset the parser state machine without clearing statistics."""
        self._reset()

    @property
    def last_byte_time(self) -> float:
        """``time.monotonic()`` timestamp of the most recently received byte."""
        return self._last_byte_time

    @property
    def state_name(self) -> str:
        return self._state.name

    @property
    def rx_count(self) -> int:
        return self._rx_count

    @property
    def crc_error_count(self) -> int:
        return self._crc_err_count

    @property
    def ok_count(self) -> int:
        return self._ok_count

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _reset(self) -> None:
        self._state:       _State = _State.WAIT_HEADER_0
        self._seq_id:      int    = 0
        self._version:     int    = 0
        self._command:     int    = 0
        self._payload_len: int    = 0
        self._payload:     bytearray = bytearray()
        self._payload_idx: int    = 0
        self._crc_low:     int    = 0
        self._last_byte_time: float = 0.0

    def _feed_byte(self, b: int) -> Packet | None:  # noqa: C901 (complex but mirrors firmware)
        st = self._state

        if st is _State.WAIT_HEADER_0:
            if b == HEADER_BYTES[0]:
                self._state = _State.WAIT_HEADER_1

        elif st is _State.WAIT_HEADER_1:
            if b == HEADER_BYTES[1]:
                self._state = _State.WAIT_HEADER_2
            else:
                self._reset()

        elif st is _State.WAIT_HEADER_2:
            if b == HEADER_BYTES[2]:
                self._state = _State.GET_SEQ_ID_L
            else:
                self._reset()

        elif st is _State.GET_SEQ_ID_L:
            self._seq_id = b
            self._state  = _State.GET_SEQ_ID_H

        elif st is _State.GET_SEQ_ID_H:
            self._seq_id |= b << 8
            self._state   = _State.READ_VERSION

        elif st is _State.READ_VERSION:
            self._version = b
            if b != PROTOCOL_VERSION_V1:
                self._reset()
            else:
                self._state = _State.READ_COMMAND

        elif st is _State.READ_COMMAND:
            self._command = b
            self._state   = _State.READ_LEN_L

        elif st is _State.READ_LEN_L:
            self._payload_len = b
            self._state       = _State.READ_LEN_H

        elif st is _State.READ_LEN_H:
            self._payload_len |= b << 8
            if self._payload_len > MAX_PAYLOAD_SIZE:
                self._reset()
            elif self._payload_len == 0:
                self._payload = bytearray()
                self._state   = _State.READ_CRC_L
            else:
                self._payload     = bytearray(self._payload_len)
                self._payload_idx = 0
                self._state       = _State.READ_PAYLOAD

        elif st is _State.READ_PAYLOAD:
            self._payload[self._payload_idx] = b
            self._payload_idx += 1
            if self._payload_idx >= self._payload_len:
                self._state = _State.READ_CRC_L

        elif st is _State.READ_CRC_L:
            self._crc_low = b
            self._state   = _State.READ_CRC_H

        elif st is _State.READ_CRC_H:
            crc_rx = self._crc_low | (b << 8)
            pkt = self._try_complete(crc_rx)
            self._reset()
            return pkt

        return None

    def _try_complete(self, crc_rx: int) -> Packet | None:
        """Validate CRC and return a Packet on success."""
        import struct
        hdr_fields = struct.pack(
            "<HBBH",
            self._seq_id,
            self._version,
            self._command,
            self._payload_len,
        )
        crc_calc = compute_crc16(hdr_fields + bytes(self._payload))

        if crc_calc != crc_rx:
            self._crc_err_count += 1
            return None

        self._ok_count += 1
        return Packet(
            seq_id  = self._seq_id,
            version = self._version,
            command = self._command,
            payload = bytes(self._payload),
            crc     = crc_rx,
        )
