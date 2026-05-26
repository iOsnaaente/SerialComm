"""
    SerialComm Python Client - Callback-driven, no blocking
    Compatible with:
        - SerialComm Protocol V1

    Dependencies:
        pip install pyserial
"""

import threading
import struct
import serial
import time
import datetime

from dataclasses import dataclass
from typing import Callable, Optional


# CONSTANTS
HEADER = bytes([0xAA, 0x55, 0xAA])
VERSION_V1 = 0x01
MAX_PAYLOAD_SIZE = 1024

# COMMANDS
class Commands:
    UNKNOWN    = 0x00
    PING       = 0x01
    READ       = 0x02
    WRITE      = 0x03
    REPLY      = 0x04
    ERROR      = 0x05
    SYNC_WRITE = 0x06
    SYNC_READ  = 0x07
    EVENT      = 0x08
    ACK        = 0x09
    NACK       = 0x0A


# PACKET
@dataclass
class SerialCommPacket:
    command: int
    payload: bytes = b''


# CRC16
class CRC16:
    @staticmethod
    def compute(data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc ^= (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return crc


# SERIAL COMM CLIENT
class SerialCommClient:
    def __init__( self, port: str, baudrate: int = 921600, timeout: float = 1 ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial: Optional[serial.Serial] = None
        self.running = False
        self.rx_thread = None
        self.callbacks = {}
        self.rx_buffer = bytearray()
        self._last_tx = None
        self.DEBUG = False

    def _now(self):
        return datetime.datetime.now().isoformat(timespec='milliseconds')

    def _log(self, *args, **kwargs):
        if not self.DEBUG:
            return
        print(f"[PYDBG {self._now()}]", *args, **kwargs)


    # SERIAL
    def connect(self):
        self.serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout
        )
        self.running = True
        self.rx_thread = threading.Thread(
            target=self._rx_loop,
            daemon=True
        )
        self.rx_thread.start()
        print(f"[SerialComm] Connected to {self.port}")


    def disconnect(self):
        self.running = False
        if self.rx_thread:
            self.rx_thread.join(timeout=1.0)
        if self.serial:
            self.serial.close()
        print("[SerialComm] Disconnected")


    # CALLBACKS
    def register_callback(
        self,
        command: int,
        callback: Callable[[SerialCommPacket], None]
    ):
        self.callbacks[command] = callback


    # TX
    def send_packet( self, packet: SerialCommPacket ):
        encoded = self.encode_packet(packet)
        self.serial.write(encoded)
        self._last_tx = encoded


    # ENCODER
    def encode_packet( self, packet: SerialCommPacket ) -> bytes:
        payload_len = len(packet.payload)
        frame = bytearray()
        frame.extend(HEADER)
        frame.append(VERSION_V1)
        frame.append(packet.command)
        frame.extend(struct.pack('<H', payload_len))
        frame.extend(packet.payload)
        crc = CRC16.compute(frame[len(HEADER):])
        frame.extend(struct.pack('<H', crc))
        return bytes(frame)


    # RX LOOP
    def _rx_loop(self):
        while self.running:
            try:
                available = self.serial.in_waiting
                if available > 0:
                    data = self.serial.read(min(available, 256))
                else:
                    data = self.serial.read(1)
                
                if data:
                    self.rx_buffer.extend(data)
                    self._process_rx_buffer()
            except Exception as e:
                self._log(f"rx error: {e}")


    # PARSER
    def _process_rx_buffer(self):
        while True:
            minimum_packet_size = 3 + 1 + 1 + 2 + 2

            if len(self.rx_buffer) < minimum_packet_size:
                return
            
            header_index = self.rx_buffer.find(HEADER)
            if header_index == -1:
                self.rx_buffer.clear()
                return
            
            if header_index > 0:
                del self.rx_buffer[:header_index]

            if len(self.rx_buffer) < minimum_packet_size:
                return

            version = self.rx_buffer[3]
            if version != VERSION_V1:
                del self.rx_buffer[0]
                continue

            payload_len = struct.unpack( '<H', self.rx_buffer[5:7] )[0]
            if payload_len > MAX_PAYLOAD_SIZE:
                del self.rx_buffer[0]
                continue

            packet_size = ( 3 + 1 + 1 + 2 + payload_len + 2 )
            if len(self.rx_buffer) < packet_size:
                return

            packet_bytes = bytes( self.rx_buffer[:packet_size] )
            del self.rx_buffer[:packet_size]

            # ignore local echo
            if self._last_tx is not None and packet_bytes == self._last_tx:
                self._last_tx = None
                continue

            # CRC VALIDATION
            received_crc = struct.unpack( '<H', packet_bytes[-2:] )[0]
            computed_crc = CRC16.compute( packet_bytes[len(HEADER):-2] )
            if received_crc != computed_crc:
                self._log(f"CRC ERROR: received=0x{received_crc:04X} computed=0x{computed_crc:04X}")
                continue

            # DECODE
            command = packet_bytes[4]
            payload = packet_bytes[7:-2]
            packet = SerialCommPacket( 
                command = command, 
                payload = payload 
            )

            self._dispatch_packet(packet)


    # DISPATCH
    def _dispatch_packet( self, packet: SerialCommPacket ):
        callback = self.callbacks.get(packet.command)
        if callback:
            callback(packet)


    # HELPERS
    def ping(self):
        packet = SerialCommPacket( command = Commands.PING, payload = b'' )
        self.send_packet(packet)


    def read_position( self, motor_id: int ):
        payload = struct.pack( '<B', motor_id )
        packet = SerialCommPacket(
            command = Commands.READ,
            payload = payload
        )
        self.send_packet(packet)


    def write_position( self, motor_id: int, position: int ):
        payload = struct.pack( '<BH', motor_id, position )
        packet = SerialCommPacket(
            command = Commands.WRITE,
            payload = payload
        )
        self.send_packet(packet)


# EXAMPLE

if __name__ == '__main__':
    client = SerialCommClient( port = '/dev/ttyUSB1', baudrate = 921600 )
    
    last_ticks = 0
    read_count = 0
    ping_count = 0
    reply_count = 0

    def on_ping( packet: SerialCommPacket ):
        global ping_count

        # Respond to PING with a REPLY (empty payload)
        reply = SerialCommPacket( 
            command = Commands.REPLY, 
            payload = b''
            )
        client.send_packet(reply)
        print(f'[PING] Received & Replied')


    def on_reply( packet: SerialCommPacket ):
        global last_ticks, read_count
        try:
            # REPLY payload format: [ticks (4 bytes)]
            if len(packet.payload) == 4:
                ticks = struct.unpack('<I', packet.payload)[0]
                ticks_diff = ticks - last_ticks
                last_ticks = ticks
                print(f'[REPLY] ticks={ticks} diff={ticks_diff})')
            else:
                print(f'[REPLY] payload_len={len(packet.payload)} (expected 4)')
        except Exception as exc:
            print(f'[REPLY] parse failed: {exc}')


    client.register_callback( Commands.PING, on_ping )
    client.register_callback( Commands.REPLY, on_reply )

    print("[SerialComm] Starting client (callback-driven)...")
    client.connect()
    time.sleep(0.5)

    try:
        while True:
            # Send PING every 50ms
            client.ping()
            time.sleep( 0.05 )
            
            # Send READ every 100ms
            if int(time.time() * 10) % 2 == 0:
                client.read_position(0)
            time.sleep( 0.1 )

    except KeyboardInterrupt:
        print("\n[SerialComm] Stopping...")
        client.disconnect()
