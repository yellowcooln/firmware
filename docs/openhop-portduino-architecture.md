# openHop Portduino radio backend

The Portduino backend runs the Meshtastic node stack above an openHop TCP
modem. MeshMonitor remains unchanged.

```text
MeshMonitor (unchanged)
        |
        | standard StreamAPI, TCP 4403
        v
Meshtastic core / OpenHopRadio
        |
        | openHop modem protocol, TCP 5055 by default
        v
Portduino OpenHopTcpSession -> openHop modem
```

## Boundaries

- Portduino `OpenHop` configuration owns the modem host, port (default `5055`),
  optional token, connect timeout, and reconnect backoff settings.
- `OpenHopProtocol` owns the byte-level framing: `0xAA` sync, command, little
  endian `u16` length, payload, and little endian CRC-16/CCITT-FALSE over
  command + length + payload. Its parser is bounded and handles fragmented,
  coalesced, corrupted, and resynchronized streams.
- `OpenHopTcpSession` owns one worker, the TCP lifecycle, optional `AUTH`,
  `PING`, `SET_CONFIG`, `RX_START`, serialized request/response exchanges,
  unsolicited `RX_PACKET` callbacks, reconnect, and configuration replay.
- `OpenHopRadio` adapts the session to `RadioInterface`, including Meshtastic
  air-packet serialization, RX metadata, CAD/LBT, TX completion, radio
  reconfiguration, queue ownership, and airtime accounting.

The token is retained for authentication and YAML round-tripping, but no
logging path prints it. Generated YAML is configuration output and should be
handled with the same filesystem permissions as the source configuration.

## Configuration

```yaml
Lora:
  Module: openHop

OpenHop:
  Host: 10.0.50.67
  Port: 5055
  Token: ""
  ConnectTimeoutMs: 5000
  ReconnectInitialMs: 1000
  ReconnectMaxMs: 30000
```

TCP sockets are implemented for native Portduino targets. Browser/WASM builds
must not instantiate the session until a browser transport exists.

The modem TCP server accepts one active client. Stop openHop Core, repeaters,
diagnostic clients, or stale daemon instances before starting meshtasticd.
Additional TCP handshakes may enter the listen backlog while the active client
owns the protocol slot, then time out or reset without receiving `PONG`.

MeshMonitor remains unchanged and continues to use the standard StreamAPI
endpoint above the Meshtastic core.
