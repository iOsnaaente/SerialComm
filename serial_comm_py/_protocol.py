"""
Packet encode / decode for the SerialComm V1 wire format.

Wire layout (little-endian):
  [0xAA][0x55][0xAA]           — 3-byte header (not covered by CRC)
  seq_id      uint16 LE        ─┐
  version     uint8             │
  command     uint8             │  covered by CRC16-CCITT
  payload_len uint16 LE         │
  payload     bytes[N]         ─┘
  crc16       uint16 LE        — CRC of the fields above

Mirrors SerialCommProtocol::encode() / decode() from serial_comm_protocol.cpp.
"""

from __future__ import annotations

import struct
from ._types import (
    Packet,
    HEADER_BYTES,
    MAX_PAYLOAD_SIZE,
    PROTOCOL_VERSION_V1,
)
from ._crc import compute_crc16


# Pre-computed minimum frame size (no payload):
#   header(3) + seq_id(2) + version(1) + command(1) + len(2) + crc(2) = 11
_MIN_FRAME_SIZE: int = 11


def encode(
    seq_id:  int,
    command: int,
    payload: bytes | bytearray = b"",
    version: int = PROTOCOL_VERSION_V1,
) -> bytes:
    """
    Encode a packet into its wire representation.

    Returns a ``bytes`` object ready to be written to the serial port.
    Raises ``ValueError`` if ``payload`` exceeds ``MAX_PAYLOAD_SIZE``.
    """
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise ValueError(
            f"payload length {len(payload)} exceeds MAX_PAYLOAD_SIZE={MAX_PAYLOAD_SIZE}"
        )

    seq_id  &= 0xFFFF
    command &= 0xFF
    version &= 0xFF

    # Fields after the header — these are what the CRC covers.
    # Pack as little-endian: seq_id(H) version(B) command(B) payload_len(H)
    header_fields = struct.pack(
        "<HBBH",
        seq_id,
        version,
        command,
        len(payload),
    )

    crc_input = header_fields + bytes(payload)
    crc       = compute_crc16(crc_input)
    crc_bytes = struct.pack("<H", crc)

    return HEADER_BYTES + crc_input + crc_bytes


def decode(data: bytes | bytearray | memoryview) -> Packet | None:
    """
    Decode a complete wire frame into a ``Packet``.

    Returns ``None`` if the data is too short, the header is wrong,
    the payload length field is invalid, or the CRC does not match.

    This function expects a *single* complete frame starting at byte 0
    (i.e. already framed by the stream parser).  For incremental parsing
    of a live byte stream use ``StreamParser`` from ``_parser.py``.
    """
    data = memoryview(data) if not isinstance(data, memoryview) else data

    if len(data) < _MIN_FRAME_SIZE:
        return None

    # Header check
    if bytes(data[:3]) != HEADER_BYTES:
        return None

    # Decode fixed fields
    seq_id, version, command, payload_len = struct.unpack_from("<HBBH", data, 3)

    if payload_len > MAX_PAYLOAD_SIZE:
        return None

    expected_total = _MIN_FRAME_SIZE + payload_len
    if len(data) < expected_total:
        return None

    payload = bytes(data[9 : 9 + payload_len])
    crc_rx  = struct.unpack_from("<H", data, 9 + payload_len)[0]

    # CRC validation — covers fields 3..end-2
    crc_calc = compute_crc16(data[3 : 9 + payload_len])
    if crc_calc != crc_rx:
        return None

    return Packet(
        seq_id  = seq_id,
        version = version,
        command = command,
        payload = payload,
        crc     = crc_rx,
    )
