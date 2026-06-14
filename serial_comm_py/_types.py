"""
Protocol types, enums and data containers for SerialComm.
Mirrors serial_comm_messages.h and serial_comm_utils.h from the firmware.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from enum import IntEnum


# ---------------------------------------------------------------------------
# Command byte
# ---------------------------------------------------------------------------

class DeliveryMode(IntEnum):
    """
    Transport-level delivery guarantee — mirrors DeliveryMode in serial_comm_types.hpp.

    BEST_EFFORT — fire-and-forget (no ACK expected).
        For UART: bytes enter the OS TX queue and the call returns immediately.
        For ESP-NOW: fragments are sent without checking the send-done status.
        This is the default for @best_effort IDL types.

    RELIABLE — wait for link-layer confirmation before returning.
        For UART: blocks until the hardware TX FIFO drains.
        For ESP-NOW: waits for the per-fragment send-done callback and aborts
        if any fragment fails.
        This is used for @reliable IDL types (e.g. service requests).

    Note: from Python the transport is always UART.  DeliveryMode affects how
    the *firmware* sends packets over its configured transport (ESP-NOW, etc.).
    Publishing from Python always routes through the OS serial driver and the
    mode value is not enforced on the host side — it is carried informatively.
    """
    BEST_EFFORT = 0
    RELIABLE    = 1


class Command(IntEnum):
    UNDEFINED     = 0x00
    READ          = 0x01
    WRITE         = 0x02
    PING          = 0x03
    READ_UTILITY  = 0x04
    WRITE_UTILITY = 0x05
    STATUS_TOPIC  = 0x06

    @classmethod
    def _missing_(cls, value: object) -> Command:
        """Allow arbitrary uint8 values (e.g. user-defined @id commands)."""
        obj = int.__new__(cls, value)  # type: ignore[arg-type]
        obj._name_  = f"CMD_0x{value:02X}"
        obj._value_ = value
        return obj


REPLY_MASK: int = 0x80


def is_reply(cmd: int) -> bool:
    return bool(cmd & REPLY_MASK)


def is_request(cmd: int) -> bool:
    return not is_reply(cmd)


def make_reply(cmd: int) -> int:
    return (cmd | REPLY_MASK) & 0xFF


def make_request(cmd: int) -> int:
    return cmd & (~REPLY_MASK & 0xFF)


# ---------------------------------------------------------------------------
# Protocol version
# ---------------------------------------------------------------------------

PROTOCOL_VERSION_V1: int = 0x01
PROTOCOL_VERSION_V2: int = 0x02   # reserved

# ---------------------------------------------------------------------------
# Frame constants  (mirrors serial_comm_protocol.h)
# ---------------------------------------------------------------------------

HEADER_BYTES: bytes   = bytes([0xAA, 0x55, 0xAA])
MAX_PAYLOAD_SIZE: int = 1024


# ---------------------------------------------------------------------------
# Packet dataclass
# ---------------------------------------------------------------------------

@dataclass
class Packet:
    """Decoded SerialComm protocol packet."""
    seq_id:  int
    version: int
    command: int          # raw byte — use Command(packet.command) if needed
    payload: bytes
    crc:     int = 0

    def command_enum(self) -> Command:
        return Command(self.command)

    def is_reply(self) -> bool:
        return is_reply(self.command)

    def is_request(self) -> bool:
        return is_request(self.command)

    def reply_command(self) -> int:
        return make_reply(self.command)

    def request_command(self) -> int:
        return make_request(self.command)

    def __repr__(self) -> str:
        cmd_name = Command(self.command)._name_
        return (
            f"Packet(seq={self.seq_id:#06x} ver={self.version} "
            f"cmd={cmd_name}({self.command:#04x}) "
            f"payload[{len(self.payload)}]={self.payload.hex(' ')})"
        )


# ---------------------------------------------------------------------------
# Result codes
# ---------------------------------------------------------------------------

class ResultCode(IntEnum):
    OK                   =  0
    ERR_FAIL             = -1
    ERR_INVALID_ARG      = -2
    ERR_INVALID_STATE    = -3
    ERR_TIMEOUT          = -4
    ERR_NOT_SUPPORTED    = -5
    ERR_NO_MEMORY        = -6
    ERR_BUFFER_FULL      = -7
    ERR_NOT_INITIALIZED  = -8
    ERR_ALREADY_INIT     = -9
    ERR_IO               = -10
    ERR_CRC              = -11
    ERR_PARSER           = -12
    ERR_DISCONNECTED     = -13
    ERR_OVERFLOW         = -14
    ERR_BUSY             = -15
    ERR_EMPTY            = -16
    ERR_NULL_POINTER     = -17
