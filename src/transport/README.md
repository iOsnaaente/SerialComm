# Transport Layer

## Hardware abstraction layer

Responsável por:

- UART;
- USB CDC;
- TCP;
- BLE;
- SPI;
- RS485.

Não conhece:

- packet;
- parser;
- services;
- topics.

Trabalha apenas com bytes em formato de Stream.

## TODO

Implementar um transporte que seja em quadros e não stream como a serial é hoje.
