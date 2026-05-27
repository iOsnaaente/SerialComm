# Middleware Layer

## Orquestra abstrações semânticas

Ele deve:

- registrar services;
- registrar topics;
- registrar actions;
- integrar com SerialComm;
- integrar com TransactionManager;
- fazer roteamento semântico;
- encapsular packets;
- encapsular seq_id;
- encapsular replies.

Ele abstrai o uso do SerialComm, Dispatcher e Parser do usuário.
