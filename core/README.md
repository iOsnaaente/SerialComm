# Protocol Layer

## Protocol infrastructure layer

Responsável por:

- parser;
- framing;
- CRC;
- seq_id;
- dispatcher;
- watchdog;
- serialization raw;
- transport orchestration.

Não conhece:

- services;
- topics;
- actions;
- payloads da aplicação.
- middleware/
- Semantic communication layer

Responsável por:

- services;
- topics;
- actions;
- transactions;
- typed callbacks;
- serializers;
- request/reply abstraction.
