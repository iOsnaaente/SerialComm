# serial_comm_py

Python client library for the **SerialComm** embedded middleware.  
Communicates with an ESP32 running SerialComm firmware over a standard UART / USB-serial port.

> **Firmware repo:** [github.com/iOsnaaente/SerialComm](https://github.com/iOsnaaente/SerialComm)  
> **Library version:** 0.3.2 · **Protocol:** V1 (header `0xAA 0x55 0xAA`) · **Python:** ≥ 3.10

---

## Overview

`serial_comm_py` mirrors the firmware's `SerialCommManager` API on the host side.  
The same three communication patterns available in the C++ middleware are available in Python:

| Pattern | Python method | Firmware equivalent | Direction |
|---------|--------------|---------------------|-----------|
| **Event** (topic) | `publish()` / `subscribe()` | `SerialCommManager::publish<T>()` | unidirectional, no reply |
| **Request** (service) | `call_service()` / `serve()` | `SerialCommManager::call_service<Req,Res>()` | request → reply, blocking |
| **Mission** (action) | `call_service()` + `subscribe()` | `SerialCommAction` | goal → feedback stream → result |

All public methods are thread-safe. A single background daemon thread handles all RX parsing and packet dispatch.

---

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│  Your application                                       │
│                                                         │
│  ctrl.publish()  ctrl.call_service()  ctrl.get_last()   │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────┐
│  SerialCommClient                                       │
│                                                         │
│  ┌──────────┐  ┌──────────────┐  ┌───────────────────┐ │
│  │ TX path  │  │ RX thread    │  │ Rate limiter      │ │
│  │ _tx_lock │  │ StreamParser │  │ Retain cache      │ │
│  │ flush()  │  │ _dispatch()  │  │ Pending futures   │ │
│  └────┬─────┘  └──────┬───────┘  └───────────────────┘ │
└───────┼───────────────┼─────────────────────────────────┘
        │               │
┌───────▼───────────────▼─────────────────────────────────┐
│  pyserial · Serial(port, baudrate, timeout=0.001)       │
└────────────────────────────┬────────────────────────────┘
                             │ UART / USB-serial
┌────────────────────────────▼────────────────────────────┐
│  ESP32 firmware — SerialCommManager                     │
│  UART transport  ·  ESP-NOW transport  ·  MultiTransport│
└─────────────────────────────────────────────────────────┘
```

---

## Installation

```bash
# Minimum — UART communication only
pip install pyserial

# Optional — Numba JIT-compiled CRC16 (~40× faster; useful for offline log replay)
pip install numba numpy

# All at once
pip install -r requirements.txt
```

**Requirements:** Python ≥ 3.10, pyserial ≥ 3.5.

---

## Quick start

```python
import struct
from serial_comm_py import SerialCommClient, Command, DeliveryMode

client = SerialCommClient(port="/dev/ttyUSB0", baudrate=921600)
client.start()

# ── Publish an event (fire-and-forget) ───────────────────────────────
# IDL: RobotVelocityCmd @id 0x10  →  uint8 robot_id, float32 vl, float32 vr
payload = struct.pack("<Bff", 0, 1.0, 1.0)          # 9 bytes, little-endian
client.publish(Command(0x10), payload)

# ── Subscribe to incoming events ──────────────────────────────────────
# IDL: RobotTelemetry @id 0x11  →  uint8 robot_id, 5× float32, bool ball
def on_telemetry(pkt):
    rid, ax, ay, gz, vl, vr, ball = struct.unpack_from("<Bfffff?", pkt.payload)
    print(f"Robot{rid}: ax={ax:.2f}  vl={vl:.2f}  ball={bool(ball)}")

client.subscribe(Command(0x11), on_telemetry)

# ── Blocking service call (request → reply) ───────────────────────────
# IDL: SetMotor_Request  →  uint8 motor_id, float32 speed, bool enable
req = struct.pack("<Bf?", 0, 1.5, True)
ok, resp = client.call_service(Command(0x30), req, timeout=0.5)
if ok:
    success, actual_speed = struct.unpack_from("<Bf", resp)

# ── Round-trip latency ────────────────────────────────────────────────
rtt_ms = client.ping()          # None on timeout

client.stop()
```

Use as a context manager to guarantee cleanup:

```python
with SerialCommClient(port="/dev/ttyUSB0", baudrate=921600) as client:
    client.publish(Command(0x10), struct.pack("<Bff", 0, 0.0, 0.0))
```

---

## Communication patterns

### Event — publish / subscribe

An event carries data in one direction with no reply.
The payload byte layout must match the IDL field order exactly.

```python
# Publisher
payload = struct.pack("<ffI", temperature, humidity, timestamp_ms)
client.publish(Command(0x20), payload)

# Subscriber (callback runs in the background RX thread — keep it fast)
def on_reading(pkt):
    temp, hum, ts = struct.unpack_from("<ffI", pkt.payload)

client.subscribe(Command(0x20), on_reading)
client.unsubscribe(Command(0x20), on_reading)   # remove when done
```

### Request — call service / serve

A service call blocks until the matching reply arrives or a timeout fires.
Python can also act as the *server* and reply to requests from the firmware.

```python
# Client side — send request, wait for reply
ok, resp = client.call_service(Command(0x30), request_payload, timeout=0.5)
if ok:
    process(resp)

# Server side — handle firmware requests and reply automatically
def handle_set_motor(request_payload: bytes) -> bytes:
    motor_id, speed, enable = struct.unpack_from("<Bf?", request_payload)
    actual = run_motor(motor_id, speed, enable)
    return struct.pack("<Bf", True, actual)

client.serve(Command(0x30), handle_set_motor)
client.unserve(Command(0x30))    # remove handler
```

### Mission — goal / feedback / result

A mission is a long-running action with three phases.  
There is no separate mission API; compose the primitives you already know:

```python
# 1. Subscribe to feedback and result BEFORE sending the goal
client.subscribe(CMD_FEEDBACK, on_feedback)   # stream of progress events
client.subscribe(CMD_RESULT,   on_result)     # one-shot completion

# 2. Send the goal and wait for acceptance
ok, ack = client.call_service(CMD_GOAL, goal_payload, timeout=1.0)

# 3. Feedback and result arrive asynchronously via callbacks
```

See [`examples/03_mission.py`](examples/03_mission.py) for a full working example.

---

## IDL generator

`serial_comm_py` is designed to work alongside the SerialComm **IDL generator**
(`user_app/generate.py`).  The generator reads `.event`, `.request`, `.mission`,
and `.struct` files and outputs C++ headers.  On the Python side you use the
generated `generated_constants.py` for command IDs and decorator metadata,
and `struct.pack` / `struct.unpack` for serialization.

### Workflow

```
1. Write your IDL files in user_app/
   ├── event/MyStatus.event
   ├── request/SetMotor.request
   └── struct/GpsCoord.struct

2. Run the generator
   python3 user_app/generate.py \
       --input  user_app \
       --output include/serial_comm/generated

3. The generator produces:
   ├── include/serial_comm/generated/event/my_status.hpp
   ├── include/serial_comm/generated/serializers/...
   ├── include/serial_comm/generated/generated_serializers.hpp
   └── include/serial_comm/generated/manifest.json

4. For Python: update generated_constants.py with the new command IDs
   and use struct.pack / struct.unpack following the IDL field order.
```

### IDL type → struct format mapping

| IDL type  | `struct` format | Size |
|-----------|-----------------|------|
| `uint8`   | `B`             | 1 B  |
| `int8`    | `b`             | 1 B  |
| `uint16`  | `H`             | 2 B  |
| `int16`   | `h`             | 2 B  |
| `uint32`  | `I`             | 4 B  |
| `int32`   | `i`             | 4 B  |
| `float32` | `f`             | 4 B  |
| `float64` | `d`             | 8 B  |
| `bool`    | `?`             | 1 B  |

All fields are **little-endian** unless the IDL file declares `@big`.  
Format strings must be prefixed with `<` (little) or `>` (big).

---

## Decorators

IDL decorators control runtime behaviour on both the firmware and the Python side.

| Decorator | IDL syntax | C++ constexpr | Python effect |
|-----------|-----------|---------------|---------------|
| `@best_effort` | default | `DELIVERY_MODE = BEST_EFFORT` | `publish(..., delivery_mode=DeliveryMode.BEST_EFFORT)` |
| `@reliable` | explicit | `DELIVERY_MODE = RELIABLE` | `publish(..., delivery_mode=DeliveryMode.RELIABLE)` — flushes OS TX buffer |
| `@timeout_ms N` | `@timeout_ms 500` | `TIMEOUT_MS = 500` | per-call timeout override for `call_service()` |
| `@version N` | `@version 1` | `SCHEMA_VERSION = 1` | version guard; `SCHEMA_VERSION[cmd]` in `generated_constants.py` |
| `@deprecated` | `@deprecated` | `[[deprecated(...)]]` | `DEPRECATED_COMMANDS` frozenset; raise `DeprecationWarning` on use |
| `@retain` | `@retain` | `RETAIN = true` | `set_retain(cmd)` → `get_last(cmd)` last-value cache |
| `@max_rate_hz N` | `@max_rate_hz 33` | `MAX_RATE_HZ = 33.0f` | `set_rate_limit(cmd, 33)` → `ERR_BUSY` on excess |

### Using decorators

```python
from serial_comm_py import SerialCommClient, DeliveryMode
from serial_comm_py.generated_constants import (
    ROBOT_VELOCITY_CMD_ID, ROBOT_TELEMETRY_ID,
    DELIVERY_MODE, MAX_RATE_HZ, RETAIN_COMMANDS,
)

with SerialCommClient("/dev/ttyUSB0", 921600) as client:

    # Apply all IDL-declared constraints at once
    for cmd, hz in MAX_RATE_HZ.items():
        client.set_rate_limit(cmd, hz)           # @max_rate_hz

    for cmd in RETAIN_COMMANDS:
        client.set_retain(cmd, enabled=True)     # @retain

    # Publish with declared delivery mode
    payload = struct.pack("<Bff", 0, 1.0, 1.0)
    mode = DELIVERY_MODE.get(ROBOT_VELOCITY_CMD_ID, DeliveryMode.BEST_EFFORT)
    rc = client.publish(ROBOT_VELOCITY_CMD_ID, payload, delivery_mode=mode)

    # Read cached telemetry without waiting for a new packet
    last = client.get_last(ROBOT_TELEMETRY_ID)   # bytes or None
```

### `DeliveryMode`

```python
from serial_comm_py import DeliveryMode

DeliveryMode.BEST_EFFORT   # 0 — fire-and-forget (default)
DeliveryMode.RELIABLE      # 1 — flush OS TX buffer after write
```

On the firmware side, `RELIABLE` routes to `transport_->write()` which blocks
until the UART TX FIFO drains or the ESP-NOW send-done callback confirms delivery.

---

## API reference

### `SerialCommClient`

```python
client = SerialCommClient(
    port                     = "/dev/ttyUSB0",
    baudrate                 = 921600,
    inter_byte_timeout_chars = 3.5,    # mirrors firmware hardware watchdog
    read_chunk               = 256,    # bytes per serial.read() call
    read_timeout             = 0.001,  # max RX delivery latency in seconds
)
```

#### Lifecycle

| Method | Description |
|--------|-------------|
| `start()` | Open serial port and start background RX thread. Raises `serial.SerialException` on failure. |
| `stop()` | Stop RX thread and close port. |
| `__enter__` / `__exit__` | Context manager — calls `start()` / `stop()` automatically. |

#### Event (topic)

| Method | Signature | Description |
|--------|-----------|-------------|
| `publish` | `(cmd, payload=b"", delivery_mode=BEST_EFFORT) → ResultCode` | Send packet with no reply. Returns `ERR_BUSY` if rate limit exceeded. |
| `subscribe` | `(cmd, callback: Packet → None) → None` | Register incoming packet handler. Multiple callbacks per command are supported. |
| `unsubscribe` | `(cmd, callback) → None` | Remove a registered callback. |

#### Service (request / reply)

| Method | Signature | Description |
|--------|-----------|-------------|
| `call_service` | `(cmd, payload=b"", timeout=2.0) → (bool, bytes)` | Send request and block until reply or timeout. |
| `serve` | `(cmd, handler: bytes → bytes) → None` | Register a service handler (Python acts as server). |
| `unserve` | `(cmd) → None` | Remove service handler. |

#### Rate limiting (`@max_rate_hz`)

| Method | Signature | Description |
|--------|-----------|-------------|
| `set_rate_limit` | `(cmd, max_rate_hz: float) → None` | Enforce a publish ceiling. `publish()` returns `ERR_BUSY` if called faster than `1/max_rate_hz` seconds since the last accepted call. Pass `0` to remove the limit. |

#### Retain cache (`@retain`)

| Method | Signature | Description |
|--------|-----------|-------------|
| `set_retain` | `(cmd, enabled=True) → None` | Enable or disable the last-value cache for `cmd`. |
| `get_last` | `(cmd) → bytes \| None` | Return the most recently received payload for `cmd`, or `None` if no packet has arrived yet. |

#### Diagnostics

| Method | Signature | Description |
|--------|-----------|-------------|
| `ping` | `(timeout=1.0) → float \| None` | Measure round-trip latency in milliseconds. Returns `None` on timeout. |
| `stats` | `() → dict` | Snapshot of runtime counters (see below). |
| `is_running` | `() → bool` | `True` if the RX thread is active. |

```python
s = client.stats()
# {
#   "tx_packets":    int,   # total packets sent
#   "rx_packets":    int,   # total valid packets received
#   "crc_errors":    int,   # packets with CRC mismatch
#   "timeouts":      int,   # call_service() timeouts
#   "parser_resets": int,   # inter-byte timeout resets
#   "parser_rx":     int,   # StreamParser total bytes fed
#   "parser_ok":     int,   # StreamParser successfully decoded packets
# }
```

---

### `StreamParser`

Low-level streaming byte parser — the same state machine as the firmware's `SerialCommParser`.
Used internally by `SerialCommClient`; also useful for offline log analysis.

```python
from serial_comm_py import StreamParser

parser = StreamParser()

# Feed arbitrary byte chunks
packets = parser.feed(b"\xaa\x55\xaa...")    # returns list[Packet]

# Offline replay
with open("capture.bin", "rb") as f:
    while chunk := f.read(4096):
        for pkt in parser.feed(chunk):
            print(pkt)

parser.reset()                  # clear partial state
parser.state_name               # current state string
parser.last_byte_time           # time.monotonic() of last byte received
parser.rx_count                 # total bytes fed
parser.ok_count                 # successfully decoded packets
parser.crc_error_count          # CRC failures
```

---

### `generated_constants.py`

Auto-maintained file that mirrors all IDL decorator values on the Python side.
Update it whenever you regenerate the C++ headers.

```python
from serial_comm_py.generated_constants import (
    ROBOT_VELOCITY_CMD_ID,    # int  — @id 0x10
    ROBOT_TELEMETRY_ID,       # int  — @id 0x11
    LEGACY_STATUS_ID,         # int  — @id 0x09 (@deprecated)

    DELIVERY_MODE,            # dict[int, DeliveryMode]  — @best_effort / @reliable
    MAX_RATE_HZ,              # dict[int, float]         — @max_rate_hz N
    RETAIN_COMMANDS,          # frozenset[int]           — @retain
    SCHEMA_VERSION,           # dict[int, int]           — @version N
    DEPRECATED_COMMANDS,      # frozenset[int]           — @deprecated
    TIMEOUT_MS,               # dict[int, int]           — @timeout_ms N
)
```

---

### Utility functions

```python
from serial_comm_py import compute_crc16, encode_packet, decode_packet, has_numba

frame = encode_packet(seq_id=1, command=0x10, payload=b"\x00\x00\x80\x3f\x00\x00\x80\x3f\x00")
pkt   = decode_packet(frame)    # Packet dataclass or None
crc   = compute_crc16(b"\x00\x01\x00\x10\x09\x00...")   # CRC16-CCITT
print(has_numba())              # True if Numba JIT is active
```

---

## Protocol wire format

```
 Byte  0  1  2    3  4    5    6    7  8    9 … 9+N    9+N  9+N+1
┌─────────────┬───────┬─────┬─────┬───────┬──────────┬────────────┐
│ 0xAA 55 AA  │ SEQ   │ VER │ CMD │ LEN   │ PAYLOAD  │  CRC16     │
│  (3 bytes)  │ (2 LE)│(1 B)│(1 B)│ (2 LE)│ (N bytes)│   (2 LE)  │
└─────────────┴───────┴─────┴─────┴───────┴──────────┴────────────┘
```

| Field | Size | Endian | Notes |
|-------|------|--------|-------|
| Header | 3 B | — | `0xAA 0x55 0xAA` — not covered by CRC |
| `seq_id` | 2 B | LE | Request/reply correlation — wraps at 0xFFFF → 1 |
| `version` | 1 B | — | Always `0x01` |
| `command` | 1 B | — | Bit 7 = reply flag (`0x80`); bits 0–6 = command index |
| `payload_len` | 2 B | LE | 0 – 1024 bytes |
| `payload` | N B | — | Message-specific; byte order declared in IDL |
| `CRC16` | 2 B | LE | CRC16-CCITT (`poly=0x1021 init=0xFFFF`) over `seq_id → payload` |

**Minimum frame size:** 11 bytes (empty payload).

### Command byte conventions

| Value range | Meaning |
|-------------|---------|
| `0x00 – 0x06` | Standard commands (`PING`, `READ`, `WRITE`, …) |
| `0x07 – 0x7F` | User-defined commands (assigned via `@id` in IDL) |
| `0x80 – 0xFF` | Reply commands (request command OR-ed with `0x80`) |

### CRC16-CCITT

```
poly  = 0x1021
init  = 0xFFFF
input = seq_id(2 LE) ‖ version(1) ‖ command(1) ‖ payload_len(2 LE) ‖ payload(N)
```

---

## VSSS bridge controller

`vsss_controller.py` provides a domain-specific high-level controller for the VSSS
(Very Small Size Soccer) ESP-NOW bridge. It abstracts the raw byte packing behind
typed methods and maintains a thread-safe per-robot telemetry cache.

```python
from serial_comm_py.vsss_controller import VSSSController, ROBOT_ID_ALL

with VSSSController(port="/dev/ttyUSB0", baudrate=921600) as ctrl:
    rtt = ctrl.ping()
    print(f"Bridge RTT: {rtt:.1f} ms")

    ctrl.send_velocity(0,  1.0,  1.0)           # Robot 0 forward
    ctrl.send_velocity(1,  0.5, -0.5)           # Robot 1 rotate right
    ctrl.send_velocity(ROBOT_ID_ALL, 0.0, 0.0)  # all robots stop

    tel = ctrl.get_telemetry(0)                 # dict or {}
    # {"robot_id": 0, "ax": ..., "ay": ..., "gz": ...,
    #  "vl_measured": ..., "vr_measured": ..., "ball": True}

    ok, speed = ctrl.set_motor(motor_id=0, speed=50.0, enable=True)
```

Run the live demo:

```bash
python serial_comm_py/vsss_controller.py \
    --port /dev/ttyUSB0 --baud 921600 --hz 33 --debug
```

---

## Examples

All examples live in [`examples/`](examples/) and can be run standalone:

| File | Demonstrates |
|------|-------------|
| [`01_event.py`](examples/01_event.py) | `publish()` and `subscribe()` with a SensorReading event |
| [`02_request.py`](examples/02_request.py) | `call_service()` and `serve()` with a SetMotor request |
| [`03_mission.py`](examples/03_mission.py) | Goal → feedback stream → result using service + subscribe |
| [`04_generator_usage.py`](examples/04_generator_usage.py) | IDL generator workflow, `generated_constants.py`, `manifest.json` |
| [`05_decorators.py`](examples/05_decorators.py) | All 7 decorators — delivery mode, rate limit, retain cache, version guard |

Each script accepts `--port`, `--baud`, and `--role` arguments.

---

## Performance

| Path | Throughput | Notes |
|------|-----------|-------|
| CRC16 — pure Python | ~5 MB/s | Sufficient at any UART baud rate |
| CRC16 — Numba JIT | ~200 MB/s | First call incurs ~1 s JIT compile |
| Parser — pure Python | >100 MB/s | State machine, no regex |

At 921600 baud (~92 KB/s) the pure-Python paths are entirely sufficient.
Enable Numba only for replaying large binary captures offline.

### Protocol limits

| Layer | Limit | Symptom | Fix |
|-------|-------|---------|-----|
| Kconfig baud range | **5 Mbaud** (raised from 1 Mbaud) | menuconfig rejects value | already updated |
| ESP32 UART hardware | ~3.5 Mbaud (UART2, short cable) | framing errors, CRC errors | lower baud |
| Firmware RX buffer | stalls if processing lag exceeds drain time | `UART_BUFFER_FULL_INT` | raise `CONFIG_SERIAL_COMM_UART_RX_BUFFER_SIZE` |
| `read_timeout` | max RX delivery latency per read() call | correct data but delayed | lower `read_timeout` (default 1 ms) |
| Python GIL | sustained >~500 KB/s may lag | `parser_resets` grows | `read_chunk=4096` |
| Inter-byte timeout | false reset if threshold < OS scheduler tick | `parser_resets` > 0 with valid data | `inter_byte_timeout_chars=0` |

### VSSS bridge — bandwidth analysis (921600 baud, 30 ms cycle, 3 robots)

```
Frames per cycle:  3 × RobotVelocityCmd (20 B) + 3 × RobotTelemetry (33 B) = 159 B
Required:          159 B / 0.030 s = 5 300 B/s
Available:         921600 / 10 = 92 160 B/s
Utilisation:       5.75%  (17× headroom on UART)
Real bottleneck:   ESP-NOW unicast + ACK ≈ 2–5 ms × 6 packets = 12–30 ms/cycle
```

---

## Diagnostics

```python
s = client.stats()

# UART noise or firmware RX buffer overflow
crc_rate = s["crc_errors"] / max(1, s["rx_packets"])
if crc_rate > 0.01:
    print(f"WARNING: {crc_rate:.1%} CRC error rate — check cable or lower baud")

# Inter-byte timeout firing on valid data
if s["parser_resets"] > 0:
    print("Hint: set inter_byte_timeout_chars=0 to disable inter-byte timeout")

# Firmware queue saturated
if s["timeouts"] > 0:
    rtt = client.ping()
    print(f"Service timeouts: {s['timeouts']}  RTT: {rtt} ms")
```

---

## Module structure

```
serial_comm_py/
├── __init__.py             ← Public API (all exports)
├── _types.py               ← Command, Packet, ResultCode, DeliveryMode
├── _crc.py                 ← CRC16-CCITT (Numba JIT + pure-Python fallback)
├── _protocol.py            ← encode_packet() / decode_packet()
├── _parser.py              ← StreamParser — byte-by-byte state machine
├── client.py               ← SerialCommClient — threading, RX dispatch, TX mutex
├── generated_constants.py  ← Command IDs and decorator metadata (maintain manually)
├── vsss_controller.py      ← Domain-specific high-level controller (VSSS bridge)
├── requirements.txt
└── examples/
    ├── 01_event.py
    ├── 02_request.py
    ├── 03_mission.py
    ├── 04_generator_usage.py
    └── 05_decorators.py
```

---

## License

This project is part of the SerialComm firmware component.  
See the repository root for license information.
