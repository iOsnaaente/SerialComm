# SerialComm Project Context

## Overview

SerialComm is a lightweight communication middleware designed for embedded systems.

The project provides a complete communication stack including:

* Transport abstraction
* Packet framing
* Serialization
* Middleware services
* Event distribution
* Mission execution
* Request/response transactions
* Automatic code generation from IDL files

The architecture is heavily inspired by concepts from:

* ROS2
* micro-ROS
* DDS
* Modbus
* Custom binary protocols

while remaining lightweight enough to run on small microcontrollers such as:

* ESP32
* STM32
* RP2040
* AVR (with reduced features)

The primary goal is to provide a ROS-like communication framework without requiring ROS.

---

# Project Goals

The project was created to solve several problems commonly found in embedded firmware:

* Manual serialization
* Manual packet parsing
* Manual request/reply handling
* Manual command registration
* Protocol maintenance complexity
* Code duplication

Instead of manually defining every packet structure and serializer, users define interfaces using IDL files and the framework generates all serialization code automatically.

---

# Architecture

The project follows a layered architecture similar to the OSI model.

## Application Layer

User application.

Responsible for:

* Business logic
* Event publication
* Service implementation
* Mission execution

Examples:

```text
RobotState.event
SetPose.request
PlayMovement.mission
```

---

## Middleware Layer

Provides communication abstractions.

Responsible for:

* Service server/client
* Event publisher/subscriber
* Mission server/client
* Transaction tracking
* Timeout handling
* Command dispatching

Main modules:

```text
SerialCommManager
SerialCommService
SerialCommTransactionManager
```

---

## Serialization Layer

Responsible for:

* Object serialization
* Object deserialization
* Endianness handling
* Dynamic arrays
* Dynamic strings

Main classes:

```text
SerialCommSerializer<T>
SerialCommBufferWriter
SerialCommBufferReader
```

---

## Protocol Layer

Responsible for:

* Packet framing
* CRC validation
* Sequence IDs
* Payload extraction

Main modules:

```text
SerialCommProtocol
SerialCommParser
```

---

## Transport Layer

Responsible for:

* UART
* USB CDC
* TCP
* UDP
* BLE
* CAN

Current implementation:

```text
UARTTransport
```

Future implementations can reuse the entire stack.

---

# IDL System

The framework contains an IDL generator inspired by ROS2.

User files are stored in:

```text
user_app/
```

Supported file types:

```text
.struct
.event
.request
.mission
```

---

# Struct

Reusable data structures.

Example:

```text
Pose.struct

int16[9] joints_positions
```

Generated:

```cpp
struct Pose
{
    std::array<int16_t, 9> joints_positions;
};
```

---

# Event

Equivalent to ROS Topics.

One-way communication.

Example:

```text
RobotState.event

Pose pose
float32 battery
bool enabled
```

Generated:

```cpp
struct RobotState
{
    ...
};
```

And:

```cpp
SerialCommSerializer<RobotState>
```

---

# Request

Equivalent to ROS Services.

Request/Response communication.

Example:

```text
SetPose.request

Pose target
float32 duration
bool wait_finish

===

bool success
float32 actual_position
```

Generated:

```cpp
SetPose_Request
SetPose_Response
```

and corresponding serializers.

---

# Mission

Equivalent to ROS Actions.

Long-running operations.

Example:

```text
PlayMovement.mission

Pose[] poses
float32[] durations

===

bool success

===

float32 progress
uint32 current_pose
```

Generated:

```cpp
PlayMovement_Goal
PlayMovement_Result
PlayMovement_Feedback
```

and serializers.

---

# Metadata System

IDL files support metadata directives.

Example:

```text
@id 0x03
@qos reliable
@timeout 1000
@version 1
```

---

## Supported Metadata

### Command ID

```text
@id 0x03
```

Defines packet command identifier.

---

### Version

```text
@version 1
```

Defines protocol version.

---

### Endianness

Default:

```text
@little
```

Optional:

```text
@big
```

---

### QoS

```text
@qos best_effort
```

or

```text
@qos reliable
```

---

### Timeout

```text
@timeout 1000
```

Milliseconds.

---

# Automatic Code Generation

Generator:

```text
user_app/generate.py
```

Input:

```text
user_app/
```

Output:

```text
include/serial_comm/generated/
```

Generated structure:

```text
generated/

├── struct/
├── event/
├── request/
├── mission/
├── serializers/
├── manifest.json
└── generated_serializers.hpp
```

---

# Serializer System

Every generated type automatically receives:

```cpp
template<>
struct SerialCommSerializer<T>
```

Responsibilities:

* Serialize object to bytes
* Deserialize object from bytes

No manual serializer implementation is required.

---

# Dynamic Strings

Strings are transmitted as:

```text
[length][data]
```

Example:

```text
"OK"
```

Serialized:

```text
02 00 4F 4B
```

Where:

```text
02 00 = length
4F     = O
4B     = K
```

Runtime type:

```cpp
SerialCommDynamicString
```

---

# Dynamic Arrays

Arrays are transmitted as:

```text
[length][elements]
```

Example:

```text
3 elements

10
20
30
```

Serialized:

```text
03 00
0A
14
1E
```

Runtime type:

```cpp
SerialCommDynamicArray<T>
```

---

# Packet Structure

Current protocol packet:

```text
+----------+
| Header   |
+----------+
| Payload  |
+----------+
| CRC16    |
+----------+
```

Header:

```text
sync
seq_id
version
command
payload_len
```

Payload:

```text
Serialized structure
```

CRC:

```text
CRC16
```

---

# Transaction Manager

Responsible for synchronous requests.

Features:

* Request tracking
* Reply matching
* Timeout detection
* Semaphore synchronization

Key field:

```cpp
seq_id
```

Used to match responses.

---

# Service Flow

Client:

```text
serialize request
send packet
wait response
deserialize response
```

Server:

```text
receive packet
deserialize request
execute callback
serialize response
send response
```

---

# Event Flow

Publisher:

```text
serialize event
send packet
```

Subscriber:

```text
receive packet
deserialize event
invoke callback
```

---

# Mission Flow

Client:

```text
send goal
receive feedback
receive result
```

Server:

```text
receive goal
execute operation
publish feedback
publish result
```

---

# Current Status

Implemented:

* UART transport
* Packet parser
* Packet builder
* CRC validation
* Serializer generation
* Struct generation
* Event generation
* Request generation
* Mission generation
* Dynamic arrays
* Dynamic strings
* Service manager
* Transaction manager
* Auto-generated serializers

In progress:

* Event manager
* Mission manager
* Discovery system
* Automatic command negotiation
* QoS enforcement

Planned:

* TCP transport
* UDP transport
* CAN transport
* BLE transport
* Encryption
* Compression
* Runtime reflection
* Wireshark dissector

---

## Relação da Arquitetura SerialComm com o Modelo ISO/OSI

Embora o SerialComm seja um middleware embarcado e não implemente diretamente todas as camadas do modelo ISO/OSI, sua arquitetura foi organizada de forma semelhante, separando responsabilidades em diferentes níveis de abstração. Essa divisão facilita a manutenção, extensibilidade e portabilidade do sistema para diferentes meios de comunicação.

A tabela abaixo apresenta uma correspondência conceitual entre os módulos do SerialComm e as camadas do modelo ISO/OSI:

| Camada OSI       | Responsabilidade                                          | Implementação no SerialComm                                                                              |
| ---------------- | --------------------------------------------------------- | -------------------------------------------------------------------------------------------------------- |
| Aplicação (7)    | Lógica da aplicação e troca de informações entre sistemas | `.event`, `.request`, `.mission`, `.struct`, callbacks do usuário                                        |
| Apresentação (6) | Representação e serialização dos dados                    | `SerialCommSerializer<T>`, `SerialCommBufferWriter`, `SerialCommBufferReader`                            |
| Sessão (5)       | Controle de sessões e transações                          | `SerialCommTransactionManager`, gerenciamento de `seq_id`, timeouts e sincronização                      |
| Transporte (4)   | Comunicação confiável entre entidades                     | Serviços, respostas automáticas, controle de timeout e associação de requisições e respostas             |
| Rede (3)         | Endereçamento e roteamento                                | Não implementado atualmente. Pode ser fornecido pela camada de transporte subjacente (TCP/IP, CAN, etc.) |
| Enlace (2)       | Estruturação dos quadros, detecção de erros               | `SerialCommProtocol`, cabeçalhos, CRC16, tamanho de payload e sincronização                              |
| Física (1)       | Transmissão dos bits no meio físico                       | UART, USB CDC, TCP, UDP, BLE, CAN ou qualquer outro transporte implementado                              |

### Camada de Aplicação

A camada mais alta é composta pelos arquivos IDL definidos pelo usuário:

```text
RobotState.event
SetPose.request
PlayMovement.mission
Pose.struct
```

Esses arquivos descrevem apenas os dados e os contratos de comunicação, sem detalhes sobre serialização ou transporte.

### Camada de Apresentação

Responsável por transformar estruturas C++ em sequências de bytes e vice-versa.

Exemplos:

```cpp
SerialCommSerializer<Pose>
SerialCommSerializer<SetPose_Request>
SerialCommSerializer<SetPose_Response>
```

Também é nessa camada que são tratados:

* Endianness (`@little` e `@big`)
* Arrays estáticos
* Arrays dinâmicos
* Strings dinâmicas
* Estruturas aninhadas

### Camada de Sessão

O SerialComm mantém o conceito de sessão através dos identificadores de sequência (`seq_id`).

Cada requisição enviada recebe um identificador único que permite:

* Associar respostas à requisição original;
* Detectar timeouts;
* Gerenciar múltiplas transações simultâneas.

Essa funcionalidade é implementada principalmente pelo:

```cpp
SerialCommTransactionManager
```

### Camada de Transporte

A camada de transporte do SerialComm fornece mecanismos semelhantes aos encontrados em protocolos como TCP ou DDS, porém focados em sistemas embarcados.

Ela é responsável por:

* Serviços síncronos;
* Controle de timeout;
* Associação entre requisição e resposta;
* Entrega de eventos;
* Gerenciamento de missões.

Essas funcionalidades são implementadas pelos componentes:

```cpp
SerialCommManager
SerialCommService
SerialCommMission
```

### Camada de Enlace

Responsável por transformar mensagens em pacotes transmissíveis.

Um pacote SerialComm possui a seguinte estrutura:

```text
+--------+--------+---------+---------+-------------+---------+-------+
| SYNC   | SEQ_ID | VERSION | COMMAND | PAYLOAD_LEN | PAYLOAD | CRC16 |
+--------+--------+---------+---------+-------------+---------+-------+
```

Essa camada fornece:

* Delimitação de pacotes;
* Verificação de integridade (CRC16);
* Identificação de comandos;
* Controle de versão;
* Recuperação de sincronismo.

Os principais módulos são:

```cpp
SerialCommProtocol
SerialCommParser
```

### Camada Física

A camada física é abstraída pela interface de transporte.

Atualmente o projeto possui implementação para:

```cpp
UARTTransport
```

Entretanto, a arquitetura foi projetada para permitir futuras implementações utilizando:

* USB CDC
* TCP/IP
* UDP
* BLE
* CAN
* ESP-NOW
* SPI
* RS-485

sem necessidade de alterar as camadas superiores.

### Benefícios da Separação em Camadas

A adoção de uma arquitetura inspirada no modelo ISO/OSI traz diversas vantagens:

* Baixo acoplamento entre módulos;
* Facilidade de manutenção;
* Possibilidade de reutilizar o protocolo em diferentes meios físicos;
* Facilidade de testes unitários;
* Extensão futura do sistema sem impactar aplicações existentes;
* Portabilidade entre diferentes microcontroladores e sistemas operacionais.

Dessa forma, o SerialComm pode ser visto como um middleware de comunicação embarcada que implementa, de forma simplificada, conceitos equivalentes às camadas superiores do modelo ISO/OSI, oferecendo ao desenvolvedor uma interface de alto nível baseada em tipos fortemente definidos e geração automática de código.

--- 

## Version History and Engineering Documentation

The `idf_component.yml` file is used not only as a component manifest but also as the primary historical record of the project's evolution.

Instead of documenting only released versions, the file captures the technical motivations, architectural decisions, major refactorings and future development plans that shaped the middleware.

This approach allows developers to understand not only **what changed**, but also **why the change was necessary**.

### Documentation Philosophy

Every release should answer four questions:

1. What was added?
2. What was modified?
3. Why was the modification necessary?
4. What impact does it have on compatibility and future development?

For this reason, each version entry may contain:

```yaml
version:
description:
features:
breakpoints:
design decisions:
troubleshooting:
```

### Description

Provides a high-level summary of the release goals.

Example:

```yaml
description: >
  Added transaction manager and synchronous
  request/reply service support.
```

The description should explain the main objective of the release instead of listing individual changes.

### Features

Lists the user-visible or developer-visible capabilities introduced by the release.

Example:

```yaml
features:
  - Transaction manager
  - Reply synchronization
  - Timeout handling
```

Features should describe functionality rather than implementation details.

### Breakpoints

Documents significant architectural milestones.

A breakpoint represents a major change in how the system operates internally.

Example:

```yaml
breakpoints:
  - Added 16-bit sequence identifier
  - Introduced transaction matching architecture
  - Refactored protocol structures
```

These entries are especially useful when reviewing historical changes and understanding how the architecture evolved over time.

### Design Decisions

Used to explain the engineering reasoning behind a modification.

Example:

```yaml
design decisions:
  - Added SEQ_ID field to support
    transaction correlation and future
    middleware abstractions.
```

This section should answer:

> Why was this implemented this way instead of another way?

This is particularly valuable months or years later when revisiting the design.

### Troubleshooting

Documents known issues and their solutions.

Example:

```yaml
troubleshooting:

  - issue: "Parser timeout reset spam"

    solution: >
      Reset parser only when packet
      assembly is in progress.
```

This section becomes a knowledge base of previously solved problems and helps future maintainers avoid repeating the same debugging process.

### Planned Versions

The `planned` section describes the expected future evolution of the project.

Unlike the roadmap, planned versions represent concrete development targets.

Example:

```yaml
planned:

  - version: "0.3.x"

    features:
      - Wi-Fi transport support
      - Bluetooth transport support
      - Service discovery mechanism
```

Only features that are expected to be implemented in the next major development cycle should be included.

### Roadmap

The roadmap represents the historical progression of the project.

Entries should never be removed.

Even deprecated features and unsuccessful architectural experiments should remain documented whenever they influenced the evolution of the project.

The roadmap serves as:

* Historical archive
* Engineering journal
* Architectural decision history
* Release notes
* Developer onboarding reference

### Writing Guidelines

When adding a new version:

1. Start with the release objective in `description`.
2. List user-visible functionality in `features`.
3. Document major architectural changes in `breakpoints`.
4. Explain important engineering reasoning in `design decisions`.
5. Register solved problems in `troubleshooting`.
6. Keep entries chronological.
7. Never rewrite previous history unless correcting factual errors.

The goal is to preserve the reasoning process that led to the current architecture, making the project easier to maintain, extend and understand over time.


---

# Design Philosophy

SerialComm follows four fundamental principles:

1. Embedded-first design

Everything must run efficiently on microcontrollers.

2. Code generation over manual coding

Developers should describe interfaces instead of implementing serializers manually.

3. Strong type safety

Communication is strongly typed.

4. Middleware abstraction

Application code should never depend on packet layouts.

The application should interact only with typed structures while the framework handles serialization, transport and protocol details automatically.

---

# Project TODO

This is a concrete, code-level punch list produced by a full-project review (generator,
runtime, and docs). It is deliberately different from "Roadmap" / "Planned Versions"
above: those describe the *historical and future shape* of the project for the
`idf_component.yml` changelog, while this section tracks *specific, actionable findings*
— what was found, why it matters, and a suggested direction — so both humans and AI
assistants can act on them without re-deriving the same investigation. When an item is
resolved, move its story into a version entry (with the *why*) rather than just deleting
the bullet.

Items are ordered by impact, not by ease.

## 1. `@id` values are not validated against the built-in command range or the reply bit

`validate_metadata_ids()` in [`generate.py`](user_app/generate.py) only rejects
*duplicate* `@id` values across IDL files — it never checks whether an ID is safe to use
in the first place.

`SerialCommCommand` (see
[`serial_comm_messages.h`](include/serial_comm/messages/serial_comm_messages.h)) reserves
two regions of the 8-bit command space:

* `0x00`–`0x06` for built-in commands (`UNDEFINED`, `READ`, `WRITE`, `PING`,
  `READ_UTILITY`, `WRITE_UTILITY`, `STATUS_TOPIC`);
* bit `0x80` (`SERIAL_COMM_CMD_REPLY_MASK`), tested by `is_reply()` / toggled by
  `make_reply()` / `make_request()`.

A custom `@id` that lands in either region compiles and generates fine, then misbehaves
*at runtime*: an ID in `0x00`–`0x06` collides with a built-in command and gets routed to
the wrong handler; any ID `>= 0x80` makes `is_reply()` true for a plain request/event/
mission packet and breaks transaction matching in the transaction manager. The only safe
range for custom IDs is **`0x07`–`0x7F`** — every example IDL file added in this review
(`RobotState` `0x10`, `SetPose` `0x20`, `PlayMovement` `0x30`) deliberately stays inside it.

**Suggested fix:** extend `validate_metadata_ids()` to `raise RuntimeError` (matching the
existing `[ERROR]`-style message for duplicate IDs) when `id < 0x07 or id > 0x7F`,
naming the valid range. Small, localized change; turns a silent runtime misrouting bug
into an immediate, understandable generation-time error.

## 2. `SerialCommDynamicArray<T>` / `SerialCommDynamicString` need manual pre-allocation before every deserialize

Both types (see
[`serial_comm_types.hpp`](include/serial_comm/common/serial_comm_types.hpp)) are bare PODs:

```cpp
struct SerialCommDynamicString  { char* data = nullptr; uint16_t size = 0; uint16_t capacity = 0; };
template <typename T>
struct SerialCommDynamicArray   { T*    data = nullptr; uint16_t size = 0; uint16_t capacity = 0; };
```

`SerialCommBufferReader::read_dynamic_array()` / `read(SerialCommDynamicString&)` (see
[`serial_comm_buffer_reader.hpp`](include/serial_comm/common/serial_comm_buffer_reader.hpp))
correctly *refuse* to read a payload larger than `capacity` rather than overflowing the
buffer — that part is safe. But because a freshly-constructed instance has
`data == nullptr` and `capacity == 0`, **every** attempt to deserialize a non-empty
dynamic array or string into a default-constructed generated type (any `_Request`/
`_Goal`/event/struct containing a `string` or `T[]` field — e.g. `DefaultReturn.message`,
`PlayMovement_Goal.poses`) returns `false` immediately, with nothing pointing the caller
at "you forgot to allocate `data` and set `capacity` first". This requirement only
becomes visible by reading the buffer reader's source.

**Suggested fix:** the project intentionally avoids STL containers for embedded
portability (see "Design Philosophy"), so full RAII isn't the goal — but a thin, explicit
helper would remove the trap without adding hidden allocation: e.g. member functions
`allocate(uint16_t capacity)` / `release()`, or free functions
`serial_comm_make_array<T>(capacity)` / `serial_comm_make_string(capacity)`, with
matching cleanup. This gives generated `_Request`/`_Goal` constructors (or factories)
something natural and discoverable to call.

## 3. `@qos`, `@timeout` and `@version` are accepted by the parser but have zero effect

`parse_metadata()` in [`generate.py`](user_app/generate.py) stores *any* `@key value`
line into a generic `metadata` dict, so `@qos reliable`, `@timeout 1000` and
`@version 1` all parse without complaint — but `metadata["qos"]`, `metadata["timeout"]`
and `metadata["version"]` are never read again anywhere in the generator. None of the
three:

* changes one byte of the generated structures, serializers, or `_ID` constants;
* is emitted as something the application could use (e.g. a `TIMEOUT_MS` constant on the
  generated request type);
* is enforced anywhere at runtime — `SerialCommManager::call_service()` takes its own
  independent `timeout_ms` argument (defaulting to `SerialCommConfig::TRANSACTION_TIMEOUT_MS`),
  completely decoupled from any `@timeout` written in the `.request` file, and no
  QoS-aware delivery path exists anywhere in the middleware.

This is consistent with "Current Status" listing QoS enforcement as still outstanding —
but it's worth being explicit that, today, **the directive syntax exists and parses, while
the implementation behind it is empty**: `@qos best_effort` vs. `@qos reliable` (or
`@timeout 1000` vs. `@timeout 60000`) currently produce byte-identical generated code and
identical runtime behavior. `SetPose.request` and `RobotState.event` (added in this
review) already use these directives as the IDL spec intends — so the fields are sitting
in `metadata` ready to be consumed the moment this is implemented.

**Suggested first step:** wire up `@timeout` alone before tackling QoS — emit a
`static constexpr uint32_t <Name>::TIMEOUT_MS` on generated `_Request`/`_Goal` types so
`manager.call_service<SetPose_Request>(req, SetPose_Request::TIMEOUT_MS)` becomes
possible. That gives the directive a real, visible effect and a template for handling
`@qos` once "what QoS means over a single point-to-point serial link" is settled.

## 4. No example application exercises `SerialCommManager` end-to-end

There is no `examples/`, `demo/`, `sample/`, or test application anywhere in the
component that wires up a `SerialCommManager`, registers handlers, publishes an event,
calls a service, and runs a mission using generated types. `user_app/` holds only IDL
sources and the generator — the headers it produces are never consumed by any in-repo
code, so nothing in the repository demonstrates (or regression-tests) the middleware layer.

The standalone round-trip check written during this review (serialize → deserialize →
byte-compare, zero ESP-IDF dependencies, every generated type from `Pose` through
`PlayMovement_Feedback`) proved the *generated code* is correct end-to-end — but it only
exercises the serializer layer, not `SerialCommManager`'s service/event/mission flows,
which currently have no test coverage at all.

**Suggested fix:** add one minimal example — it doesn't need real hardware, a host-side
mock `ISerialCommTransport` is enough — that drives `RobotState` / `SetPose` /
`PlayMovement` (the example types added in this review, chosen so the example and the IDL
docs reference the exact same files) through publish/subscribe, request/response, and
mission goal/feedback/result. It would double as living documentation for newcomers
(human or AI) and as the project's first integration-level smoke test.

## Investigated and intentionally left as-is

**Registering the same callback 256 times in `SerialCommManager::init()`**
([`serial_comm_manager.cpp`](src/middleware/serial_comm_manager.cpp)) looks at first
glance like redundant work:

```cpp
for ( uint16_t cmd = 0; cmd < 256; cmd++ ) {
    this->serial_->register_callback( static_cast<Command>(cmd), serial_packet_callback, this );
}
```

It isn't a bug: it pre-fills every slot of the transport's `uint8_t`-indexed dispatch
table with the central router *up front*, so the hot path (`route_packet`) never has to
branch on "is anything registered for this command?" on a per-packet basis — an
intentional, one-time-cost-for-O(1)-dispatch tradeoff appropriate for an embedded hot
path. Recorded here so a future pass doesn't "clean it up" into something slower.

---

# Protocol Evolution Roadmap: Network Layer, Addressing and New Transports

This section is the product of a deliberate design review of the project's next phase.
It answers the question: *what does it take to evolve SerialComm from a point-to-point
UART link into a real multi-node network — with meaningful `@id` semantics and Wi-Fi /
ESP-NOW / Bluetooth Classic / BLE transport support?*

It is not a new idea grafted onto the project: three pieces of evidence already sitting
in the codebase prove this direction was always intended.

1. [`include/serial_comm/core/serial_comm_protocol.h`](include/serial_comm/core/serial_comm_protocol.h)
   already defines `SERIAL_COMM_SRC_SIZE`, `SERIAL_COMM_DST_SIZE`, `SERIAL_COMM_FLAG_SIZE`,
   `SERIAL_COMM_PROTOCOL_VER2` and `SERIAL_COMM_MAX_PACKET_SIZE_V2`.  The byte budget
   for an addressed packet format is already there; only `SerialCommProtoHeader`,
   `encode()` and `decode()` were never extended to use it (they still speak V1 only —
   `validate_packet()` even has an explicit comment: *"only accepting V1 for now"*).

2. `idf_component.yml` v0.2.1 states in the author's own words: *"The Seq_id and the
   request/reply bit in the command field was planned to be present in protocol version
   2, **with the source and destination fields**"* — addressing was always what V2 was
   meant to be, not an afterthought.

3. The v0.1.2 entry already justifies multi-packet-per-buffer parsing by citing *"a non
   stream-based transport like Wi-Fi and other kinds of networks"* — the parser was built
   anticipating exactly this.

The `idf_component.yml` `planned: 0.3.x` entry also lists "Wi-Fi transport support",
"Bluetooth transport support", and "Service discovery mechanism" as goals.  What follows
is a concrete shape for those goals — what to build, in what order, and why each piece
unlocks the next.

---

## A. Foundation — Complete Protocol V2 (this is also what gives `@id` real meaning)

### The gap

`SerialCommProtoHeader` only carries `header[3]`, `seq_id`, `version`, `command`, and
`payload_len`.  The three V2 size constants (`SERIAL_COMM_SRC_SIZE`,
`SERIAL_COMM_DST_SIZE`, `SERIAL_COMM_FLAG_SIZE`) are defined but nothing reads or writes
them — `encode()` always emits V1 byte order, `decode()` uses
`SERIAL_COMM_MAX_PACKET_SIZE_V2` only as an upper-bound sanity check, and
`validate_packet()` rejects every non-V1 packet outright.

### Implementation plan

Extend the header struct to carry V2 fields, then make every function that touches raw
bytes branch on `header.version`:

```cpp
struct SerialCommProtoHeader {
    uint8_t  header[3];
    uint16_t seq_id;
    uint8_t  version;
    // V2 fields — present on wire only when version >= SERIAL_COMM_PROTOCOL_VER2
    uint8_t  src;    // originating node logical ID
    uint8_t  dst;    // destination node logical ID (0xFF = broadcast)
    uint8_t  flags;  // control bits — see below
    SerialCommCommand command;
    uint16_t payload_len;
};
```

Functions that need version-aware branches:

| Function | Change |
|---|---|
| `encode()` | write `src`/`dst`/`flags` after `version` if `packet.header.version >= VER2` |
| `decode()` | read `src`/`dst`/`flags` from the same position if `raw_data[version_offset] >= VER2` |
| `packet_size()` | add `SRC+DST+FLAG` sizes when `packet.header.version >= VER2` |
| `validate_packet()` | remove the hard "V1 only" rejection; validate V2 on same logic |
| `validate_crc()` | fold `src`/`dst`/`flags` bytes into the CRC chain between `version` and `command` |

`validate_crc()` already uses incremental CRC folding
([`src/core/serial_comm_protocol.cpp`](src/core/serial_comm_protocol.cpp)) — just add
a third chunk between the existing `meta_bytes` and `len_bytes` folds:

```cpp
// V2 only: include src/dst/flags in CRC coverage
if (packet.header.version >= SERIAL_COMM_PROTOCOL_VER2) {
    const uint8_t addr_bytes[3] = { packet.header.src, packet.header.dst, packet.header.flags };
    crc = compute_crc16(addr_bytes, sizeof(addr_bytes), crc);
}
```

V1 packets remain exactly as they are — old UART peers keep working without recompile.

### `@id` — what it becomes once SRC/DST exist

Today `command`/`@id` is a flat, global namespace: `0x10` means the same thing to
everyone on the link, because there is only ever one peer.  Once `src` and `dst` exist,
the *same* `@id` becomes meaningful **per sender**: packets
`(src=Robot_A, cmd=0x10/RobotState)` and `(src=Robot_B, cmd=0x10/RobotState)` are
distinguishable streams of the same message type from different origins — exactly the
ROS2 "topic" concept the README maps `Event` onto (`/robot_a/state` vs.
`/robot_b/state`).  Concretely:

* Add `SerialCommConfig::LOCAL_NODE_ID` as a `Kconfig` integer (following the existing
  pattern in `Kconfig.projbuild`) — populated into `header.src` on every send.
* Let `SerialCommManager` / `SerialCommDispatcher` route and filter on `(src, command)`
  pairs rather than `command` alone.  This turns a flat command ID into a per-origin
  topic — without changing the IDL format or the `@id` assignment rules.
* `dst` gives unicast (`dst == LOCAL_NODE_ID`), broadcast (`dst == 0xFF`), and the
  ability to silently drop packets not addressed to this node — essential the moment more
  than two nodes share a medium.

### Spend the `flags` byte deliberately

One byte, eight bits of future capability, cheapest possible place to give wire-format
meaning to metadata the review found to be completely inert (see "Project TODO", item 3):

| Bit | Suggested name | Purpose |
|---|---|---|
| 0 | `IS_BROADCAST` | Packet is for all nodes; suppress unicast delivery checks |
| 1 | `MORE_FRAGMENTS` | This is not the last fragment — see §D |
| 2 | `FRAGMENT_START` | This is the first fragment of a larger message |
| 3 | `ACK_REQUESTED` | Sender wants an acknowledgement (maps to `@qos reliable`) |
| 4–7 | reserved | Leave at `0` for forward compatibility |

With `ACK_REQUESTED` in the wire format, `@qos reliable` in the IDL finally has
somewhere real to live: the runtime sets this bit on send and waits for an ack-shaped
reply before handing the call back to the application.  `@qos best_effort` leaves the
bit clear.

---

## B. Generalize the Transport Contract for Addressed and Connectionless Media

### The gap

[`ISerialCommTransport`](include/serial_comm/core/serial_comm_transport.h) is shaped like
a UART: `write(data, len)` / `read(data, len, timeout_ms)` / `available()`, and `Config`
is full of UART-specific concepts (`baudrate`, `interbyte_chars`,
`auto_interbyte_timeout`).  This works perfectly for any **point-to-point byte stream**
(UART, USB CDC, RS-485, Bluetooth Classic SPP, TCP-as-client).  It does not work for
media that are inherently addressed and multi-peer — Wi-Fi UDP, ESP-NOW, BLE — where
every send needs to say *to whom* and every receive reports *from whom*.

### The central design choice: where does "peer address" live?

Two options:

1. *Push physical addresses (MAC / IP:port / BLE connection handle) up into the protocol
   as `src`/`dst`.*  Rejected: MAC addresses are 6 bytes, IP:port is 6 bytes, BLE
   handles are opaque integers — none fit a single byte, and mixing transport-specific
   address formats into the protocol header destroys portability.

2. *Keep `src`/`dst` as small, transport-agnostic logical node IDs, and let each
   transport maintain its own mapping from logical node ID to physical address.*
   **Recommended.**  A `UDPTransport` keeps a `node_id → IP:port` table, an
   `ESPNowTransport` keeps a `node_id → MAC[6]` table, a `BLETransport` keeps a
   `node_id → connection_handle` table.  `ISerialCommTransport` grows one optional,
   additive method:

```cpp
// Only addressed transports need to implement this; the default is ERR_NOT_SUPPORTED.
virtual errCode register_peer(uint8_t node_id, const void* physical_addr, size_t addr_len);
```

Stream transports (UART, SPP, TCP-client) never call it and keep working exactly as they
do today.  Addressed transports use it to build the lookup table that lets them turn a
logical `dst` from the protocol header into a physical address at send time.

This is also exactly what makes "Service discovery" (already listed in the Current Status
section and in `idf_component.yml` 0.3.x) stop being optional: with a wired point-to-
point link there is nothing to discover; with several wireless nodes, something has to
populate that `node_id → physical address` table.  A broadcast `Event` carrying
`LOCAL_NODE_ID` and the sender's physical address is the natural primitive — it is
already the right shape (one sender, many subscribers).

### Two existing pieces that already point this direction

* `ISerialCommTransport::Event::CONNECTED` / `DISCONNECTED` already exist in the events
  enum.  Wireless links need exactly this (association complete, link lost) where UART
  never does.  No new API addition is needed for connection lifecycle.
* `parse_next_packet()` / multi-packet-per-buffer support was built specifically for "non
  stream-based transport like Wi-Fi" (v0.1.2 changelog).  Datagrams from
  `recvfrom()` / `esp_now_register_recv_cb()` arrive as discrete packets in a buffer —
  already handled correctly without any change.

---

## C. Transport-by-Transport Notes, in Implementation Order

Each step reuses what the previous step proved, so the hardest problems (addressing,
fragmentation) are first encountered where they are unavoidable rather than up front.

---

### 1. Bluetooth Classic — SPP (Serial Port Profile)

**Why first:** SPP literally emulates a serial port (`esp_spp_api.h` / Bluedroid stack).
It is connection-oriented and byte-stream-shaped — a near drop-in
`ISerialCommTransport` that can use `UARTTransport`
([`include/serial_comm/transport/uart_serial_comm.h`](include/serial_comm/transport/uart_serial_comm.h))
as a structural template.  It exercises `Event::CONNECTED` / `DISCONNECTED` for the first
time (pairing and link loss) without introducing addressing or fragmentation.  If the
abstraction generalizes at all, it will be visible here with almost no new concepts.

**Key ESP-IDF surface:**

```c
esp_spp_register_callback(spp_callback, ...);
esp_spp_init(ESP_SPP_MODE_CB);      // or VFS mode for stream-style read/write
esp_spp_start_srv(SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, "SerialComm");
esp_spp_write(handle, len, data);   // or use the VFS fd path
```

**No protocol changes required.**  V1 over SPP works identically to V1 over UART.

---

### 2. Wi-Fi — TCP (Reliable Stream)

**Why second:** TCP is still connection-oriented and byte-stream-shaped — a connected
socket is a byte stream, so it slots into the existing `ISerialCommTransport` contract
with minimal friction.  The new problems are purely connection-lifecycle ones (server vs.
client role, multiple simultaneous peers, Wi-Fi drop/reconnect) handled by `esp_netif`
event callbacks and `Event::CONNECTED` / `DISCONNECTED`.

**Key ESP-IDF surface (lwIP BSD sockets):**

```c
// server side
int srv = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
bind(srv, ...);
listen(srv, backlog);
int peer_fd = accept(srv, &peer_addr, &addr_len);
send(peer_fd, data, len, 0);
recv(peer_fd, buf,  cap, 0);

// client side
int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
connect(fd, &server_addr, sizeof(server_addr));
```

**No protocol changes required.**  V1 or V2 over TCP works the same as over UART.
For server mode accepting multiple clients, `SerialComm::send()` needs to know which
socket to write to — this is the first place `send_to(node_id, packet)` becomes useful,
though a single-client TCP client transport can keep the existing `send(packet)` API
unchanged.

---

### 3. Wi-Fi — UDP and ESP-NOW (Addressed, Connectionless)

**Why together:** Both are connectionless and packet-oriented; both naturally support
broadcast; both require `src`/`dst` addressing (§A) and fragmentation (§D) for full-
sized payloads.  This pair is where all the hard architecture work from §A and §B gets
exercised for the first time.

#### Wi-Fi UDP

Natural fit for `Event` pub-sub: one publisher sends a UDP datagram to a broadcast/
multicast address; all subscribers receive it.  For unicast `Request`/`Mission`, the
transport's `node_id → IP:port` table (from `register_peer`) resolves the logical `dst`
to a socket address.

```c
int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
sendto(fd, data, len, 0, &peer_addr, sizeof(peer_addr));
recvfrom(fd, buf, cap, 0, &sender_addr, &addr_len);  // sender_addr feeds the SRC lookup
```

#### ESP-NOW

The most embedded-friendly of the wireless options: no IP stack, no connection setup,
native MAC-addressed point-to-point and broadcast (`FF:FF:FF:FF:FF:FF`), available
without a Wi-Fi access point.  Frame limit is **250 bytes** (`ESP_NOW_MAX_DATA_LEN`) —
smaller than the default `SerialCommConfig::MAX_PAYLOAD_SIZE` of 512 bytes, so §D
(fragmentation) is **mandatory** here for full-sized IDL payloads.

```c
esp_now_init();
esp_now_register_recv_cb(on_data_recv);  // fires with (src_mac, data, len)
esp_now_register_send_cb(on_data_sent);

// add a peer (maps to register_peer in the transport abstraction)
esp_now_peer_info_t peer = { .peer_addr = {0xFF,...}, .channel = 0, .encrypt = false };
esp_now_add_peer(&peer);

esp_now_send(dst_mac, data, len);        // len must be <= ESP_NOW_MAX_DATA_LEN
```

The transport's `register_peer(node_id, mac, 6)` call wraps `esp_now_add_peer`.  On
receive, `on_data_recv`'s `src_mac` parameter populates the sender side of the logical
address lookup so the packet's V2 `src` field can be set before handing off to the
parser.

---

### 4. BLE — GATT (Bluetooth Low Energy)

**Why last:** Most complex to implement; benefits from every prior step.  Connection-
oriented like SPP/TCP, but with negotiated and initially tiny MTUs — default ATT MTU is
23 bytes, yielding **20 bytes** of effective GATT payload; negotiated MTU (via
`esp_ble_gap_set_prefer_conn_params` / `BLE_ATT_MTU_MAX`) can reach up to **244 bytes**
of effective payload depending on the stack and remote device.  Fragmentation (§D) is
required for everything larger.

**Key ESP-IDF surface (Bluedroid GATT — NimBLE is a lighter-weight alternative):**

```c
// Server (peripheral)
esp_ble_gatts_register_callback(gatts_event_handler);
esp_ble_gatts_app_register(APP_ID);
// In handler: create service, add characteristic (for TX notify, RX write-without-response)

// Client (central)
esp_ble_gattc_register_callback(gattc_event_handler);
esp_ble_gattc_app_register(APP_ID);
// In handler: connect, discover services/characteristics, subscribe to notify, write

// MTU negotiation (call after connection)
esp_ble_gatt_set_local_mtu(247);
esp_ble_gattc_send_mtu_req(gattc_if, conn_id);
```

A practical GATT layout for SerialComm:

* **RX characteristic** (written by the remote without response) — receives raw protocol
  bytes (or fragments) from the peer.
* **TX characteristic** (notified by this device) — sends raw protocol bytes (or
  fragments) to the peer.

With `max_frame_size()` returning the negotiated MTU minus ATT overhead, the
fragmentation layer (§D) handles splitting automatically — the GATT transport just writes
or notifies one fragment per characteristic operation.

---

## D. Fragmentation — Required Once Any Small-MTU Transport Exists

### The numbers

| Transport | Effective payload per frame |
|---|---|
| UART / SPP / TCP | Unbounded (stream) |
| Wi-Fi UDP | ~1460 bytes (Ethernet MTU) |
| ESP-NOW | **250 bytes** (hard limit — `ESP_NOW_MAX_DATA_LEN`) |
| BLE (default MTU) | **20 bytes** |
| BLE (negotiated MTU 247) | **244 bytes** |

`SerialCommConfig::MAX_PAYLOAD_SIZE` defaults to **512 bytes** (`Kconfig.projbuild`).
Any IDL message larger than the active transport's frame limit cannot be sent as a single
frame — for example, `PlayMovement_Goal` with its `Pose[] poses` / `float32[] durations`
dynamic arrays will routinely exceed 250 bytes.

### A mechanism that reuses existing wire fields

`seq_id` already provides a correlation key per logical message (all fragments of the same
message get the same `seq_id`).  The `flags` byte from §A provides the `MORE_FRAGMENTS`
and `FRAGMENT_START` bits.  A fragment index can occupy one byte of the payload prefix
(fragment 0, 1, 2…) — small enough to be negligible.

Fragment assembly sits between `SerialCommProtocol` and the dispatcher:

1. **On send:** if `payload_len > transport->max_frame_size() - header_overhead`, split
   the payload into N chunks, assign all of them the same `seq_id`, set
   `FLAG_MORE_FRAGMENTS` on all but the last, and send each as a separate encoded packet.
2. **On receive:** buffer incoming fragments by `(src, seq_id)`.  When the packet without
   `FLAG_MORE_FRAGMENTS` arrives, concatenate all fragments in order and hand a single
   complete `SerialCommProtoPacket` to the dispatcher — which never needs to know
   fragmentation happened.

Add `virtual size_t max_frame_size() const` to `ISerialCommTransport` — stream
transports return `SIZE_MAX` and the fragmentation layer becomes a no-op, preserving
existing behavior exactly.

Temporary assembly buffers should be allocated from the heap (or a fixed pool) and
reclaimed on completion or on timeout (reusing `SerialCommWatchdogTimer` makes sense
here — the same inter-byte watchdog concept applies to stalled fragment sequences).

---

## E. Sequence and Discovery — Why This Order Matters

Each phase is a prerequisite for the next, and each is independently useful even if the
phases after it are never started:

* **§A alone** (V2 header) is useful on UART — it gives `@qos` / `@timeout` metadata
  somewhere real to live in the wire format, and gives `@id` per-source meaning for any
  multi-drop wired setup (RS-485, shared serial bus).
* **§B alone** (transport contract extension) is purely additive — stream transports that
  never call `register_peer()` keep working without change.
* **§C transport order** — SPP first proves the abstraction at near-zero conceptual cost;
  TCP second adds connection lifecycle without addressing; UDP/ESP-NOW third forces §A
  and §B to be exercised under real conditions; BLE last inherits from all of them.
* **§D fragmentation** only needs to exist once a transport with a constrained frame size
  exists — and by then §A has already provided the bits (`flags`) and correlation key
  (`seq_id`) it needs at no additional wire cost.

**Service discovery** (in Current Status → *In progress*; in `idf_component.yml`
0.3.x) ties all five phases together: once nodes have logical IDs (§A), multiple
transports exist (§C), and fragment reassembly works (§D), the missing piece is automatic
population of the `node_id → physical address` table (§B).  A discovery beacon is simply
a broadcast `Event` (already the right primitive) carrying `LOCAL_NODE_ID` and the
sender's physical address — received by all nodes on the medium, which then call
`register_peer()` to record the mapping.  No new concepts are needed: it is an
application of the primitives described in §A–§D.
