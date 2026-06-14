# SerialComm

A lightweight, ROS2-inspired communication middleware for ESP32 and other microcontrollers.
Define message interfaces in IDL files — SerialComm generates all serialization code and
wires it into a typed publish/subscribe and service-call API with no manual packet handling.

> **ESP-IDF:** ≥ 5.0 · **C++ standard:** 17 · **Python client:** ≥ 3.10  
> **Version:** 0.3.2-beta · [idf_component.yml](idf_component.yml) contains the full changelog

---

## Features

- **Three communication patterns** — Event (topic), Request (service), Mission (action)
- **IDL code generator** — write `.event` / `.request` / `.mission` / `.struct` files; get typed C++ structs and serializers
- **IDL decorators** — `@best_effort`, `@reliable`, `@retain`, `@max_rate_hz`, `@timeout_ms`, `@version`, `@deprecated`
- **Transport abstraction** — swap UART, ESP-NOW, or a composite MultiTransport without changing application code
- **Python client** — full mirror of the firmware API for host-side integration and testing
- **CRC16-CCITT** packet integrity on every frame
- **FreeRTOS-native** — mutex-protected TX, queue-based dispatcher, configurable stack sizes via `menuconfig`

---

## Architecture

```
┌────────────────────────────────────────────────────────────────┐
│  Application (user_app/)                                       │
│  RobotVelocityCmd.event  SetMotor.request  Navigate.mission    │
└────────────────────┬───────────────────────────────────────────┘
                     │ typed publish / subscribe / call_service
┌────────────────────▼───────────────────────────────────────────┐
│  Middleware  (SerialCommManager)                                │
│  Topics · Services · Actions · Transaction manager             │
└────────────────────┬───────────────────────────────────────────┘
                     │ SerialCommProtoPacket
┌────────────────────▼───────────────────────────────────────────┐
│  Serialization  (SerialCommSerializer<T>)                      │
│  BufferWriter · BufferReader · Dynamic arrays / strings        │
└────────────────────┬───────────────────────────────────────────┘
                     │ byte stream
┌────────────────────▼───────────────────────────────────────────┐
│  Protocol  (SerialCommProtocol / SerialCommParser)             │
│  Framing · CRC16-CCITT · SEQ_ID · reply-bit convention        │
└────────────────────┬───────────────────────────────────────────┘
                     │ hardware bytes
┌────────────────────▼───────────────────────────────────────────┐
│  Transport  (ISerialCommTransport)                             │
│  UARTTransport · ESPNowTransport · MultiTransport              │
└────────────────────────────────────────────────────────────────┘
```

Each layer is independently replaceable. Adding a new transport does not touch the
middleware or serialization layers; changing a message schema only requires regenerating
the headers.

---

## Quick start

### 1. Add the component

```bash
# In your IDF project root
mkdir -p components && cd components
git clone https://github.com/iOsnaaente/SerialComm SerialComm
```

Or add to `idf_component.yml` as a managed component.

### 2. Define your messages

```
user_app/
├── struct/
│   └── GpsCoord.struct
├── event/
│   └── SensorReading.event
└── request/
    └── SetMotor.request
```

**`GpsCoord.struct`**
```
float32 latitude
float32 longitude
float32 altitude
```

**`SensorReading.event`**
```
@id 0x20
@best_effort
@max_rate_hz 10

float32    temperature
float32    humidity
uint32     timestamp_ms
GpsCoord   location
```

**`SetMotor.request`**
```
@reliable
@timeout_ms 500

uint8   motor_id
float32 speed
bool    enable
===
bool    success
float32 actual_speed
```

### 3. Generate C++ headers

```bash
python3 user_app/generate.py \
    --input  user_app \
    --output include/serial_comm/generated
```

Output:
```
include/serial_comm/generated/
├── struct/gps_coord.hpp
├── event/sensor_reading.hpp
├── request/set_motor.hpp
├── serializers/
├── generated_serializers.hpp
└── manifest.json
```

### 4. Use in firmware

```cpp
#include "serial_comm/transport/uart_serial_comm.h"
#include "serial_comm/serial_comm.h"
#include "serial_comm/middleware/serial_comm_manager.h"
#include "serial_comm/generated/generated_serializers.hpp"
#include "serial_comm/generated/event/sensor_reading.hpp"
#include "serial_comm/generated/request/set_motor.hpp"

// Transport
static UARTTransport transport({ .uart_port = UART_NUM_2, .tx_pin = 17, .rx_pin = 16 });
transport.init({});
transport.start();

// Stack
static SerialComm     comm(&transport);
static SerialCommManager mgr(&comm);
comm.init({});
comm.start();
mgr.init({});

// Publish a SensorReading event
SensorReading msg{};
msg.temperature  = 25.0f;
msg.humidity     = 60.0f;
msg.timestamp_ms = pdTICKS_TO_MS(xTaskGetTickCount());
msg.location     = { -23.55f, -46.63f, 760.0f };
mgr.publish<SensorReading>(static_cast<Command>(SENSORREADING_ID), msg);

// Call the SetMotor service (blocks until reply or timeout)
SetMotor_Request  req{ .motor_id = 0, .speed = 1.5f, .enable = true };
SetMotor_Response res{};
errCode err = mgr.call_service<SetMotor_Request, SetMotor_Response>(
    static_cast<Command>(0x30), req, res
);

mgr.start();
```

---

## IDL reference

### Field types

| IDL type | C++ type | Size |
|----------|----------|------|
| `bool` | `bool` | 1 B |
| `int8` / `uint8` | `int8_t` / `uint8_t` | 1 B |
| `int16` / `uint16` | `int16_t` / `uint16_t` | 2 B |
| `int32` / `uint32` | `int32_t` / `uint32_t` | 4 B |
| `float32` / `float64` | `float` / `double` | 4 / 8 B |
| `string` | `SerialCommDynamicString` | dynamic |
| `T[]` | `SerialCommDynamicArray<T>` | dynamic |
| `T<=N[]` | `SerialCommDynamicArray<T>` | dynamic (≤ N elements) |
| `T[N]` | `std::array<T, N>` | N × sizeof(T) |
| `MyStruct` | `MyStruct` (embedded) | struct size |

Default byte order is **little-endian**. Declare `@big` at the top of a file to override.

### File types

| Extension | Pattern | Sections |
|-----------|---------|----------|
| `.struct` | Reusable embedded data | One section, no `===` |
| `.event` | Fire-and-forget topic | One section, no `===` |
| `.request` | Blocking service call | Two sections: request `===` response |
| `.mission` | Long-running action | Three sections: goal `===` result `===` feedback |

### Generated names

| IDL file | Generated types |
|----------|----------------|
| `Foo.struct` | `Foo` |
| `FooBar.event` | `FooBar` |
| `SetPose.request` | `SetPose_Request`, `SetPose_Response` |
| `Navigate.mission` | `Navigate_Goal`, `Navigate_Result`, `Navigate_Feedback` |

---

## Decorators

Decorators go at the top of any IDL file. Each maps to a `static constexpr` member in
the generated struct, consumed automatically by the middleware at zero call-site cost.

| Decorator | Syntax | C++ constexpr | Effect |
|-----------|--------|---------------|--------|
| `@id` | `@id 0x10` | `FOO_ID = 0x10` | Command byte for this message. Valid range: `0x07`–`0x7F`. |
| `@best_effort` | `@best_effort` | `DELIVERY_MODE = BEST_EFFORT` | Fire-and-forget. Uses `write_async()`. Default. |
| `@reliable` | `@reliable` | `DELIVERY_MODE = RELIABLE` | Waits for TX-FIFO drain (UART) or send-done ACK (ESP-NOW). |
| `@timeout_ms` | `@timeout_ms 500` | `TIMEOUT_MS = 500` | Per-type `call_service()` timeout via `serial_comm_timeout_ms<T>()` trait. |
| `@version` | `@version 1` | `SCHEMA_VERSION = 1` | Schema version for compatibility checks. |
| `@deprecated` | `@deprecated` | `[[deprecated(...)]]` | C++17 compiler warning on every use site. |
| `@retain` | `@retain` | `RETAIN = true` | Last-value cache in `SerialCommManager`; retrieve with `get_last<Msg>()`. |
| `@max_rate_hz` | `@max_rate_hz 33` | `MAX_RATE_HZ = 33.0f` | Python client enforces publish rate ceiling. |
| `@little` / `@big` | `@big` | — | Byte order for all fields in this file. |

### Delivery mode in detail

`@reliable` routes through `transport_->write()`:
- **UART** — blocks until the hardware TX FIFO drains (`uart_wait_tx_done`).
- **ESP-NOW** — sends each fragment and waits for the per-fragment send-done callback,
  returning an error if any fragment fails.

`@best_effort` routes through `transport_->write_async()` and returns immediately on all transports.

### Retain cache

When a subscription is created for a `@retain` type, every received packet is stored in
a fixed-size last-value table (8 slots, mutex-protected). Retrieve without blocking:

```cpp
RobotTelemetry tel{};
if (mgr.get_last<RobotTelemetry>(static_cast<Command>(ROBOTTELEMETRY_ID), tel)) {
    ESP_LOGI(TAG, "Last known: robot=%d ball=%d", tel.robot_id, tel.ball_detected);
}
```

---

## Transports

| Transport | Header | Use case |
|-----------|--------|----------|
| `UARTTransport` | `transport/uart_serial_comm.h` | Wired UART link (default, most examples) |
| `ESPNowTransport` | `transport/espnow_serial_comm.h` | Wireless, infrastructure-free, up to 250 B/frame |
| `MultiTransport` | `transport/multi_transport.h` | Composite — route RX/TX across up to 4 sub-transports |

### MultiTransport

`MultiTransport` implements `ISerialCommTransport` and aggregates sub-transports behind a
single interface. `SerialComm` and `SerialCommManager` require no changes.

```cpp
MultiTransport multi;
MultiTransport::SlotConfig rx_only{ .rx = true,  .tx = false };
MultiTransport::SlotConfig tx_only{ .rx = false, .tx = true  };

multi.add_transport(&uart_transport,   rx_only);   // receive from UART
multi.add_transport(&espnow_transport, tx_only);   // transmit via ESP-NOW
multi.init({});
multi.start();

SerialComm comm(&multi);   // only change vs. a single transport
```

### ESPNowTransport

```cpp
ESPNowTransport::HardwareConfig hw{};
// Broadcast by default — no add_peer() needed
memset(hw.tx_peer_mac, 0xFF, 6);

ESPNowTransport transport(hw);
transport.init({});

// Unicast: register a peer, then switch TX target at runtime
const uint8_t robot_mac[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
transport.add_peer(robot_mac, /*node_id=*/0x01);
transport.set_tx_peer(robot_mac);
```

---

## Menuconfig

All tunable parameters are exposed via `menuconfig` under **Component config → Serial Communication**:

| Key | Default | Description |
|-----|---------|-------------|
| `CONFIG_SERIAL_COMM_UART_PORT` | 2 | UART peripheral number |
| `CONFIG_SERIAL_COMM_UART_TX_PIN` | 17 | TX GPIO |
| `CONFIG_SERIAL_COMM_UART_RX_PIN` | 16 | RX GPIO |
| `CONFIG_SERIAL_COMM_UART_BAUDRATE` | 921600 | Baud rate (9600–5000000) |
| `CONFIG_SERIAL_COMM_UART_RX_BUFFER_SIZE` | 4096 | RX ring buffer size (bytes) |
| `CONFIG_SERIAL_COMM_UART_TX_BUFFER_SIZE` | 4096 | TX ring buffer size (bytes) |
| `CONFIG_SERIAL_COMM_MAX_PAYLOAD` | 512 | Maximum payload size (bytes) |
| `CONFIG_SERIAL_COMM_TRANSACTION_TIMEOUT_MS` | 2000 | Default service call timeout |
| `CONFIG_SERIAL_COMM_DISPATCHER_QUEUE_SIZE` | 16 | RX packet queue depth |
| `CONFIG_SERIAL_COMM_DISPATCHER_STACK_SIZE` | 4096 | Dispatcher task stack (bytes) |

---

## Protocol wire format

```
Byte:  0    1    2    3    4    5    6    7    8    9 … 9+N    9+N  9+N+1
      ┌─────────────┬─────────┬─────┬─────┬─────────┬────────┬───────────┐
      │ 0xAA 55 AA  │  SEQ_ID │ VER │ CMD │   LEN   │PAYLOAD │  CRC16   │
      │  (3 bytes)  │  (2 LE) │(1 B)│(1 B)│  (2 LE) │(N bytes│  (2 LE) │
      └─────────────┴─────────┴─────┴─────┴─────────┴────────┴───────────┘
```

| Field | Notes |
|-------|-------|
| Header | `0xAA 0x55 0xAA` — not covered by CRC; used for frame sync |
| `seq_id` | 16-bit rolling counter; wraps at `0xFFFF → 1`; matches requests to replies |
| `version` | Always `0x01` |
| `command` | Bit 7 = reply flag (`0x80`); bits 0–6 = command index (`0x07`–`0x7F` for user types) |
| `payload_len` | 0 – 1024 bytes |
| `payload` | Serialized struct; byte order as declared in IDL |
| `CRC16` | CRC16-CCITT (`poly=0x1021 init=0xFFFF`) over `seq_id → payload` |

**Minimum frame:** 11 bytes (empty payload).

---

## Python client

`serial_comm_py/` provides a full Python mirror of `SerialCommManager` over a standard
UART / USB-serial port. The same three patterns (Event, Request, Mission) are available
from Python with the same API names.

```bash
pip install pyserial          # required
pip install numba numpy       # optional — JIT-compiled CRC16 (~40× faster for log replay)
```

```python
from serial_comm_py import SerialCommClient, Command, DeliveryMode
import struct

with SerialCommClient("/dev/ttyUSB0", baudrate=921600) as client:
    # Publish event
    client.publish(Command(0x20), struct.pack("<ffI", 25.0, 60.0, 12345))

    # Subscribe
    client.subscribe(Command(0x20), lambda pkt: print(pkt))

    # Service call
    ok, resp = client.call_service(Command(0x30), struct.pack("<Bf?", 0, 1.5, True))

    # Rate limit + retain cache (from IDL decorators)
    client.set_rate_limit(0x20, max_rate_hz=10.0)
    client.set_retain(0x20, enabled=True)
    last = client.get_last(0x20)   # bytes or None
```

See [`serial_comm_py/README.md`](serial_comm_py/README.md) for the full API reference,
and [`serial_comm_py/examples/`](serial_comm_py/examples/) for runnable demos.

---

## Examples

All firmware examples are in [`examples/`](examples/). Each is a self-contained IDF
project with a `components/SerialComm` symlink and its own `user_app/` IDL files.

| Example | Demonstrates |
|---------|-------------|
| [`uart_example/`](examples/uart_example/) | UART transport — PING service + SensorData topic, two-board server/client |
| [`espnow_example/`](examples/espnow_example/) | ESP-NOW transport — same API, wireless |
| [`multi_transport_example/`](examples/multi_transport_example/) | MultiTransport bridge — receive UART, transmit ESP-NOW |
| [`uart_to_espnow_bridge/`](examples/uart_to_espnow_bridge/) | Production bridge — SensorReading relay + local SetMotor service |
| [`vsss_espnow_bridge/`](examples/vsss_espnow_bridge/) | VSSS robot system — bridge + 3 robots, all roles in one file via `NODE_TYPE` |

### VSSS bridge quick guide

The `vsss_espnow_bridge` example implements a 4-node wireless robot system:

```
PC ──UART──▶ Bridge (NODE_TYPE 0) ──ESP-NOW──▶ Robot 0 (NODE_TYPE 1)
              ◀── RobotTelemetry ──────────────  Robot 1 (NODE_TYPE 2)
                                                 Robot 2 (NODE_TYPE 3)
```

1. Flash `NODE_TYPE 0` on the bridge → read the MAC from boot log → fill `BRIDGE_MAC`
2. Flash `NODE_TYPE 1/2/3` on each robot → read MACs → fill `ROBOT_MACS[]`
3. Re-flash all with the correct addresses

Control from Python:

```python
from serial_comm_py.vsss_controller import VSSSController, ROBOT_ID_ALL

with VSSSController("/dev/ttyUSB0", baudrate=921600) as ctrl:
    ctrl.ping()
    ctrl.send_velocity(0, 1.0, 1.0)              # Robot 0 forward
    ctrl.send_velocity(ROBOT_ID_ALL, 0.0, 0.0)   # stop all
    tel = ctrl.get_telemetry(0)                  # {"ax": ..., "ball": True, ...}
```

---

## Repository structure

```
SerialComm/
├── include/serial_comm/
│   ├── common/             ← types, serializer base, buffer r/w, CRC
│   ├── core/               ← protocol, parser, SerialComm
│   ├── middleware/         ← SerialCommManager, services, topics, actions
│   ├── transport/          ← UART, ESP-NOW, MultiTransport interfaces
│   └── generated/          ← auto-generated by generate.py (gitignored)
├── src/
│   ├── core/
│   ├── middleware/
│   └── transport/
├── user_app/               ← IDL files (gitignored — project-specific)
│   ├── generate.py         ← IDL code generator
│   ├── event/
│   ├── request/
│   ├── mission/
│   └── struct/
├── examples/
│   ├── uart_example/
│   ├── espnow_example/
│   ├── multi_transport_example/
│   ├── uart_to_espnow_bridge/
│   └── vsss_espnow_bridge/
├── serial_comm_py/         ← Python client library
├── idf_component.yml       ← Component manifest + full engineering changelog
├── CMakeLists.txt
└── Kconfig.projbuild
```

> **Note:** `user_app/event/*`, `user_app/request/*`, etc. are gitignored because they
> are project-specific. Each example carries its own `user_app/` with the IDL files it
> needs. The root `user_app/` is the working directory for the active project.

---

## Version history

The [`idf_component.yml`](idf_component.yml) file serves as the primary engineering
changelog. Every release entry documents what changed, why the change was made, key
architectural decisions, and known troubleshooting cases — not just a list of commits.

Current version: **0.3.2-beta**

| Version | Summary |
|---------|---------|
| 0.3.2 | 5 new IDL decorators: `@timeout_ms`, `@version`, `@deprecated`, `@retain`, `@max_rate_hz` |
| 0.3.1 | `@best_effort` / `@reliable` delivery mode decorators; `DeliveryMode` trait system |
| 0.3.0 | Baudrate range raised to 5 Mbaud; Python `read_timeout` fix; VSSS capacity analysis |
| 0.2.9 | VSSS bridge example; `RobotVelocityCmd` + `RobotTelemetry` IDL types |
| 0.2.8 | `serial_comm_py` Python client library |
| 0.2.7 | `ESPNowTransport` + `MultiTransport`; stack overflow fix |
| 0.2.6 | IDL code generator (`generate.py`); auto-serializer generation |
| 0.2.5 | ROS-like event/request/mission middleware API |
| 0.2.4 | `menuconfig` integration; directory restructure |
| 0.2.3 | Transaction manager; blocking service calls |
| 0.2.2 | `SerialCommManager`; services, topics, actions |
| 0.2.1 | Protocol V2 fields: `seq_id`, request/reply command bit |
| 0.1.x | UART transport; parser; dispatcher; inter-byte timeout watchdog |

---

## License

See [LICENSE](LICENSE) for details.  
Author: Bruno Gabriel Flores Sampaio — [github.com/iOsnaaente](https://github.com/iOsnaaente)
