# serial_comm_py

Python implementation of the **SerialComm** middleware protocol — drop this
folder into any project and communicate with an ESP32 running SerialComm
firmware over UART.

## Installation

```bash
pip install pyserial          # required
pip install numba numpy       # optional — enables JIT-accelerated CRC16
```

Or install all at once:

```bash
pip install -r requirements.txt
```

## Quick start

```python
from serial_comm_py import SerialCommClient, Command
import struct

client = SerialCommClient(port='/dev/ttyUSB0', baudrate=921600)
client.start()

# --- Publish an event (fire and forget) ---------------------------------
# Payload matches the IDL struct byte-for-byte.
# RobotVelocityCmd @id 0x10: uint8 robot_id, float32 vl, float32 vr
payload = struct.pack('<Bff', 0, 1.0, 1.0)
client.publish(Command(0x10), payload)

# --- Subscribe to incoming events ---------------------------------------
# RobotTelemetry @id 0x11: uint8 robot_id, float32 ax, ay, gz, vl, vr, bool ball
def on_telemetry(packet):
    rid, ax, ay, gz, vl, vr, ball = struct.unpack('<BfffffB', packet.payload)
    print(f"Robot{rid}: ax={ax:.2f}  vl={vl:.2f}  ball={bool(ball)}")

client.subscribe(Command(0x11), on_telemetry)

# --- Blocking service call (request / reply) ----------------------------
# SetMotor_Request: uint8 motor_id, float32 speed, bool enable  (6 bytes)
req = struct.pack('<Bf?', 0, 1.5, True)
ok, resp = client.call_service(Command.WRITE, req, timeout=2.0)
if ok:
    success, actual_speed = struct.unpack('<Bf', resp[:5])

# --- Ping ---------------------------------------------------------------
rtt_ms = client.ping()   # returns None on timeout

client.stop()
```

## Protocol reference

| Field         | Size     | Endian | Notes                                  |
|---------------|----------|--------|----------------------------------------|
| Header        | 3 bytes  | —      | `0xAA 0x55 0xAA` (not covered by CRC) |
| seq_id        | 2 bytes  | LE     | Request/reply matching                 |
| version       | 1 byte   | —      | Always `0x01`                          |
| command       | 1 byte   | —      | See table below; bit 7 = reply flag    |
| payload_len   | 2 bytes  | LE     | Max 1024 bytes                         |
| payload       | N bytes  | —      | Message-specific                       |
| CRC16         | 2 bytes  | LE     | CCITT poly=0x1021 init=0xFFFF; covers seq_id → payload |

### Standard commands

| Command        | Value  | Description              |
|----------------|--------|--------------------------|
| `UNDEFINED`    | `0x00` | —                        |
| `READ`         | `0x01` | Read register            |
| `WRITE`        | `0x02` | Write register / service |
| `PING`         | `0x03` | Keepalive                |
| `READ_UTILITY` | `0x04` | Utility read             |
| `WRITE_UTILITY`| `0x05` | Utility write            |
| `STATUS_TOPIC` | `0x06` | Pub/sub event            |

Reply bit: `0x80` ORed with the request command byte.
User-defined `@id` values (e.g. `0x10`, `0x11`) are cast to `Command(value)`.

### CRC16-CCITT

```
poly  = 0x1021
init  = 0xFFFF
input = seq_id(2 LE) + version(1) + command(1) + payload_len(2 LE) + payload(N)
```

## VSSS bridge controller

`vsss_controller.py` provides a ready-to-use `VSSSController` class:

```python
from serial_comm_py.vsss_controller import VSSSController, ROBOT_ID_ALL

with VSSSController(port='/dev/ttyUSB0', baudrate=921600) as ctrl:
    rtt = ctrl.ping()
    ctrl.send_velocity(0,  1.0, 1.0)  # Robot 0 forward
    ctrl.send_velocity(ROBOT_ID_ALL, 0.5, -0.5)  # all robots rotate
    tel = ctrl.get_telemetry(0)        # latest sensor data from Robot 0
```

Run the demo:

```bash
python serial_comm_py/vsss_controller.py --port /dev/ttyUSB0 --baud 921600 --hz 33
```

## API reference

### `SerialCommClient`

```python
client = SerialCommClient(
    port='...',
    baudrate=921600,
    inter_byte_timeout_chars=3.5,   # mirrors firmware watchdog
    read_chunk=256,
    read_timeout=0.001,             # caps RX latency at 1 ms (reduce to 0.0002 for ≥2 Mbaud)
)
```

| Method | Description |
|--------|-------------|
| `start()` / `stop()` | Open/close serial port, start/stop RX thread |
| `publish(cmd, payload)` | Send packet, no reply expected → `ResultCode` |
| `subscribe(cmd, callback)` | Register `callback(Packet)` for incoming cmd |
| `unsubscribe(cmd, callback)` | Remove subscription |
| `call_service(cmd, payload, timeout)` | Blocking request/reply → `(bool, bytes)` |
| `serve(cmd, handler)` | Register `handler(bytes) → bytes` service server |
| `ping(timeout)` | RTT in ms or `None` |
| `stats()` | `dict` with TX/RX counts, CRC errors, etc. |

### `StreamParser`

Low-level streaming parser — used internally by `SerialCommClient` but also
useful standalone for offline log analysis:

```python
from serial_comm_py import StreamParser

parser = StreamParser()
with open('capture.bin', 'rb') as f:
    while chunk := f.read(1024):
        for packet in parser.feed(chunk):
            print(packet)
```

### Utility functions

```python
from serial_comm_py import compute_crc16, encode_packet, decode_packet, has_numba

crc   = compute_crc16(b'\x00\x01\x02')      # CRC16-CCITT
frame = encode_packet(seq_id=1, command=0x06, payload=b'...')
pkt   = decode_packet(frame)                 # Packet | None
print(has_numba())                           # True if Numba JIT is active
```

## Inter-byte timeout

The firmware resets the parser when no byte arrives within
`3.5 × byte_time` of the previous byte (hardware watchdog timer).
`SerialCommClient` mirrors this in software: the RX thread checks
`time.monotonic()` between reads and resets `StreamParser` accordingly.

You can tune the threshold:

```python
client = SerialCommClient(port=..., inter_byte_timeout_chars=3.5)  # default
client = SerialCommClient(port=..., inter_byte_timeout_chars=0)     # disable
```

## Performance

| Path                 | Throughput (approx.)          |
|----------------------|-------------------------------|
| CRC16 — pure Python  | ~5 MB/s on modern hardware    |
| CRC16 — Numba JIT    | ~200 MB/s (first call: JIT compile ~1 s) |
| Parser — pure Python | >100 MB/s (state machine, Python 3.12) |

At 921600 baud (~92 KB/s) pure Python CRC and parser are more than sufficient.
Numba only matters when replaying large binary captures offline.

### Protocol limits

| Layer | Hard limit | How to detect | Fix |
|-------|-----------|---------------|-----|
| Kconfig baud range | ~~1 Mbaud~~ → raised to **5 Mbaud** | menuconfig rejects value | `range` already updated |
| ESP32 UART hardware | ~3.5 Mbaud (UART2, short cable) | framing errors, CRC errors | lower baud |
| UART RX buffer (firmware) | stalls if processing lag > buffer drain time | `UART_BUFFER_FULL_INT` fires | raise `CONFIG_SERIAL_COMM_UART_RX_BUFFER_SIZE` |
| Python `read_timeout` | max RX delivery latency = `read_timeout` | data correct but delayed | lower `read_timeout` (default 1 ms) |
| Python thread / GIL | sustained >~500 KB/s may lag | `parser_resets` grows | `read_chunk=4096` |
| Python inter-byte timeout | false reset if `inter_byte_timeout_chars` window < `read_timeout` | `parser_resets` > 0 with valid data | set `inter_byte_timeout_chars=0` to disable |

**VSSS bridge at 921600 baud, 30 ms cycle — bandwidth utilization:**

- 3 × `RobotVelocityCmd` (20 B each) + 3 × `RobotTelemetry` (33 B each) = 159 B / cycle
- Required: 159 / 0.030 s = 5 300 B/s — **5.75% of 92 160 B/s available (17× headroom)**
- Actual bottleneck: ESP-NOW unicast + ACK (~2–5 ms per packet × 6 packets = 12–30 ms)

**Diagnosing saturation at runtime:**

```python
s = ctrl.stats()
crc_rate = s["crc_errors"] / max(1, s["rx_packets"])
print(f"CRC error rate : {crc_rate:.1%}")   # >1% → UART noise or buffer overrun
print(f"Parser resets  : {s['parser_resets']}")  # >0 → inter-byte timeout false trigger
print(f"Timeouts       : {s['timeouts']}")        # >0 → service reply lost, queue backup
```

Round-trip latency (`ctrl.ping()`) growing over successive calls → firmware queue saturated.
