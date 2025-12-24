# MAVLink Router Architecture Documentation

## Table of Contents
1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Core Components](#core-components)
4. [Message Routing Mechanism](#message-routing-mechanism)
5. [Class Diagrams](#class-diagrams)
6. [Message Flow](#message-flow)
7. [Key Features](#key-features)

---

## Overview

**MAVLink Router** is a message routing application that distributes MAVLink protocol messages between multiple endpoints (connections). It acts as a central hub that:

- Receives MAVLink messages from various sources (vehicles, ground stations, etc.)
- Routes messages to appropriate destinations based on target addressing
- Prevents message loops
- Filters messages based on configurable rules
- Supports multiple transport protocols: UART, UDP, and TCP
- Provides logging capabilities for flight stack data

### Key Concepts

- **Endpoint**: A connection point (UART, UDP, or TCP) that can send and receive MAVLink messages
- **System ID (sysid)**: Unique identifier for a MAVLink system (e.g., a vehicle)
- **Component ID (compid)**: Identifier for a component within a system (e.g., autopilot, camera)
- **Routing**: The process of determining which endpoints should receive a message based on its target address

---

## Architecture

The application follows an event-driven architecture using Linux `epoll` for efficient I/O multiplexing:

```
┌─────────────────────────────────────────────────────────────┐
│                      Mainloop (Singleton)                    │
│  - Event loop using epoll                                    │
│  - Manages all endpoints                                     │
│  - Routes messages between endpoints                         │
│  - Handles timeouts and TCP connections                     │
└─────────────────────────────────────────────────────────────┘
                            │
                            │ manages
                            ▼
        ┌───────────────────────────────────────┐
        │         Endpoint (Abstract)           │
        │  - Pollable (epoll integration)       │
        │  - Message filtering                  │
        │  - System/component tracking          │
        │  - Statistics                         │
        └───────────────────────────────────────┘
                    │
        ┌───────────┼───────────┬──────────────┐
        │           │           │                │
        ▼           ▼           ▼                ▼
    UartEndpoint UdpEndpoint TcpEndpoint   LogEndpoint
```

---

## Core Components

### 1. Mainloop (`mainloop.h` / `mainloop.cpp`)

The **Mainloop** is the central orchestrator of the application. It's implemented as a singleton and manages:

- **Event Loop**: Uses Linux `epoll` to monitor file descriptors for I/O events
- **Endpoint Management**: Maintains a list of all active endpoints
- **Message Routing**: Distributes messages from one endpoint to others
- **TCP Server**: Listens for incoming TCP connections
- **Timeout Management**: Handles periodic tasks (statistics, logging, etc.)
- **De-duplication**: Prevents duplicate messages from being routed

**Key Methods:**
- `loop()`: Main event loop that processes epoll events
- `route_msg()`: Routes a message to all appropriate endpoints
- `add_endpoints()`: Creates and registers endpoints from configuration
- `write_msg()`: Writes a message to an endpoint

### 2. Endpoint (`endpoint.h` / `endpoint.cpp`)

**Endpoint** is the abstract base class for all connection types. It provides:

- **Pollable Interface**: Inherits from `Pollable` for epoll integration
- **Message Parsing**: Parses MAVLink 1.0 and 2.0 packets
- **System Tracking**: Maintains a list of system/component IDs seen on this endpoint
- **Message Filtering**: Supports allow/block filters for incoming and outgoing messages
- **Statistics**: Tracks CRC errors, sequence drops, message counts

**Key Methods:**
- `handle_read()`: Called when data is available to read
- `read_msg()`: Parses a complete MAVLink message from the buffer
- `accept_msg()`: Determines if a message should be sent to this endpoint
- `write_msg()`: Writes a message to the endpoint (pure virtual)

**Message Acceptance Logic:**
1. **Reject** if message came from this endpoint (prevents loops)
2. **Filter** based on outgoing filters (message ID, source system/component)
3. **Accept** if:
   - Message is broadcast (target_sysid = 0 or -1)
   - Target system/component is known on this endpoint
   - Endpoint has sniffer sysid configured

### 3. UartEndpoint

Handles serial port (UART) connections:
- Opens and configures serial devices
- Supports multiple baudrates with automatic switching
- Handles flow control
- Used for telemetry radios and direct serial connections

### 4. UdpEndpoint

Handles UDP network connections with two modes:

**Client Mode:**
- Sends to a configured IP:port
- Receives from any source (learns remote address from first message)
- Supports broadcast/multicast addresses
- Switches back to broadcast after 5 seconds of inactivity

**Server Mode:**
- Listens on a configured IP:port
- Sends to the last received IP:port
- Used for accepting incoming connections

### 5. TcpEndpoint

Handles TCP network connections:
- Connects to a configured server (client mode)
- Supports automatic reconnection with configurable timeout
- Can become invalid when connection is lost
- Non-critical endpoints (don't cause router exit on error)

### 6. LogEndpoint

Special endpoint for logging flight stack data:
- Supports PX4 (`.ulg`) and ArduPilot (`.bin`) formats
- Can log telemetry data (`.tlog`)
- Auto-detects MAVLink dialect
- Manages log file rotation and cleanup

---

## Message Routing Mechanism
Client B accepts it (if target matches)

### Routing Rules

When a message is received on an endpoint, the following process occurs:

```
1. Message Received
   │
   ├─► Parse MAVLink packet (MAVLink 1.0 or 2.0)
   │
   ├─► Validate CRC
   │
   ├─► Extract source (sysid/compid) and target (sysid/compid)
   │
   ├─► Update endpoint's known systems list
   │
   ├─► Check de-duplication (if enabled)
   │
   ├─► Check incoming filters
   │
   └─► Route to Mainloop
```

### Mainloop Routing Logic

```cpp
void Mainloop::route_msg(struct buffer *buf)
{
    for (each endpoint) {
        switch (endpoint->accept_msg(buf)) {
            case Accepted:
                write_msg(endpoint, buf);
                break;
            case Filtered:
                // Message filtered out, skip
                break;
            case Rejected:
                // Message not for this endpoint
                break;
        }
    }
}
```

### Endpoint Acceptance Decision Tree

```
accept_msg(buffer)
│
├─► Is source sysid/compid known on this endpoint?
│   └─► YES → Reject (prevent loop)
│
├─► Check outgoing filters (MsgId, SrcSys, SrcComp)
│   └─► Filtered? → Return Filtered
│
├─► Is target_sysid broadcast (0 or -1)?
│   └─► YES → Accept
│
├─► Is target sysid/compid known on this endpoint?
│   └─► YES → Accept
│
├─► Is target sysid known AND compid broadcast (0 or -1)?
│   └─► YES → Accept
│
├─► Is this endpoint a sniffer (has sniffer_sysid)?
│   └─► YES → Accept
│
└─► Otherwise → Reject
```

### System/Component Tracking

Each endpoint maintains a list of system/component IDs (`_sys_comp_ids`) that have been seen on that endpoint. This list is updated whenever a message is received:

- When a message arrives, the source `sysid/compid` is added to the endpoint's list
- This list determines which messages should be routed to this endpoint
- Endpoints in the same **group** share their system lists (for redundant links)

### Broadcast Rules

MAVLink uses broadcast addressing:
- **System ID = 0**: Broadcast to all systems
- **Component ID = 0**: Broadcast to all components in a system
- **No target**: Message is broadcast

The router handles broadcasts by:
- Sending broadcast messages to all endpoints (except the source)
- Sending system broadcasts to endpoints that know that system
- Sending component broadcasts only if the system is known

---

## Class Diagrams

### Class Hierarchy

```
Pollable (Abstract)
│
│  + fd: int
│  + handle_read(): int (pure virtual)
│  + handle_canwrite(): bool (pure virtual)
│  + is_valid(): bool
│  + is_critical(): bool
│
└─── Endpoint (Abstract)
    │
    │  + rx_buf: buffer
    │  + tx_buf: buffer
    │  + fd: int (inherited)
    │
    │  # _type: string
    │  # _name: string
    │  # _sys_comp_ids: vector<uint16_t>
    │  # _allowed_outgoing_msg_ids: vector<uint32_t>
    │  # _blocked_outgoing_msg_ids: vector<uint32_t>
    │  # _allowed_incoming_msg_ids: vector<uint32_t>
    │  # _blocked_incoming_msg_ids: vector<uint32_t>
    │  # _stat: statistics struct
    │
    │  + handle_read(): int
    │  + handle_canwrite(): bool
    │  + read_msg(buffer*): int
    │  + accept_msg(buffer*): AcceptState
    │  + write_msg(buffer*): int (pure virtual)
    │  + flush_pending_msgs(): int (pure virtual)
    │  + has_sys_id(sysid): bool
    │  + has_sys_comp_id(sysid, compid): bool
    │  + filter_add_allowed_out_msg_id(msg_id)
    │  + filter_add_blocked_out_msg_id(msg_id)
    │  # _add_sys_comp_id(sysid, compid)
    │  # _check_crc(msg_entry): bool
    │
    ├─── UartEndpoint
    │    │
    │    # _baudrates: vector<uint32_t>
    │    # _current_baud_idx: size_t
    │    # _change_baud_timeout: Timeout*
    │    │
    │    + setup(UartEndpointConfig): bool
    │    + write_msg(buffer*): int
    │    + flush_pending_msgs(): int
    │    # open(path): bool
    │    # set_speed(baudrate): int
    │    # set_flow_control(enabled): int
    │
    ├─── UdpEndpoint
    │    │
    │    # sockaddr: sockaddr_in
    │    # sockaddr6: sockaddr_in6
    │    # is_ipv6: bool
    │    # nomessage_timeout: Timeout*
    │    │
    │    + setup(UdpEndpointConfig): bool
    │    + write_msg(buffer*): int
    │    + flush_pending_msgs(): int
    │    # open(ip, port, mode): bool
    │    # open_ipv4(ip, port, mode): int
    │    # open_ipv6(ip, port, mode): int
    │
    ├─── TcpEndpoint
    │    │
    │    # _ip: string
    │    # _port: unsigned long
    │    # _valid: bool
    │    # _retry_timeout: int
    │    # sockaddr: sockaddr_in
    │    # sockaddr6: sockaddr_in6
    │    │
    │    + setup(TcpEndpointConfig): bool
    │    + accept(listener_fd): int
    │    + reopen(): bool
    │    + close()
    │    + write_msg(buffer*): int
    │    + flush_pending_msgs(): int
    │    + is_valid(): bool (override)
    │    + is_critical(): bool (override)
    │    # open(ip, port): bool
    │    # _schedule_reconnect()
    │
    └─── LogEndpoint (Abstract)
         │
         # _config: LogOptions
         # _target_system_id: int
         # _file: int
         # _timeout: timeout structs
         │
         + start(): bool (pure virtual)
         + stop()
         + mark_unfinished_logs()
         # _get_logfile_extension(): const char* (pure virtual)
         # _logging_start_timeout(): bool (pure virtual)
         │
         ├─── BinLog (ArduPilot)
         └─── ULog (PX4)
```

### Mainloop Class

```
Mainloop (Singleton)
│
│  - epollfd: int
│  - g_endpoints: vector<shared_ptr<Endpoint>>
│  - g_tcp_fd: int
│  - _log_endpoint: shared_ptr<LogEndpoint>
│  - _timeouts: Timeout*
│  - _msg_dedup: Dedup
│  - _errors_aggregate: statistics
│
│  + init(): Mainloop& (static)
│  + get_instance(): Mainloop& (static)
│  + open(): int
│  + loop(): int
│  + route_msg(buffer*)
│  + add_endpoints(Configuration): bool
│  + write_msg(endpoint, buffer): int
│  + handle_tcp_connection()
│  + process_tcp_hangups()
│  + add_timeout(msec, callback, data): Timeout*
│  + del_timeout(Timeout*)
│  + dedup_check_msg(buffer): bool
│  # tcp_open(port): int
│  # _del_timeouts()
│  # _log_aggregate_timeout(data): bool
```

### Relationship Diagram

```
┌─────────────┐
│  Mainloop   │◄─────┐
│ (Singleton) │      │ manages
└──────┬──────┘      │
       │             │
       │ contains    │
       ▼             │
┌─────────────────┐  │
│  g_endpoints[]  │  │
│  (vector)       │  │
└──────┬──────────┘  │
       │             │
       │ points to   │
       ▼             │
┌─────────────────┐  │
│    Endpoint     │──┘
│   (Abstract)    │
└──────┬──────────┘
       │
       ├─── UartEndpoint
       ├─── UdpEndpoint
       ├─── TcpEndpoint
       └─── LogEndpoint
            ├─── BinLog
            └─── ULog

┌─────────────┐
│  Mainloop   │
└──────┬──────┘
       │ uses
       ▼
┌─────────────┐
│    Dedup    │
│  (optional) │
└─────────────┘

┌─────────────┐
│  Mainloop   │
└──────┬──────┘
       │ manages
       ▼
┌─────────────┐
│   Timeout   │───► Timeout ──► Timeout ──► ...
│  (linked list)
└─────────────┘
```

---

## Message Flow

### Flow: Vehicle → Endpoint

```
┌─────────┐
│ Vehicle │
│ (UART)  │
└────┬────┘
     │ MAVLink Message
     │ (sysid=100, compid=1)
     │ target: broadcast
     ▼
┌─────────────────┐
│ UartEndpoint     │
│ [fd=3]          │
└────┬────────────┘
     │ handle_read()
     │   ├─► read_msg()
     │   │   ├─► Parse MAVLink header
     │   │   ├─► Validate CRC
     │   │   ├─► Extract sysid/compid
     │   │   └─► Create buffer struct
     │   │
     │   ├─► allowed_by_dedup()? ✓
     │   ├─► allowed_by_incoming_filters()? ✓
     │   └─► _add_sys_comp_id(100, 1)
     │       └─► Add to _sys_comp_ids[]
     │
     │ route_msg(&buf)
     ▼
┌─────────────────┐
│   Mainloop      │
│   route_msg()   │
└────┬────────────┘
     │
     │ Iterate all endpoints
     │
     ├─► UartEndpoint [fd=3] (source)
     │   └─► accept_msg()
     │       └─► has_sys_comp_id(100,1)? YES
     │           └─► Reject (prevent loop)
     │
     ├─► UdpEndpoint [fd=4] (Ground Station)
     │   └─► accept_msg()
     │       ├─► has_sys_comp_id(100,1)? NO ✓
     │       ├─► Check filters? ✓
     │       ├─► target_sysid == 0? YES (broadcast)
     │       └─► Accept
     │           └─► write_msg()
     │               └─► sendto(udp_socket, ...)
     │
     ├─► TcpEndpoint [fd=5] (QGroundControl)
     │   └─► accept_msg()
     │       ├─► has_sys_comp_id(100,1)? NO ✓
     │       ├─► target_sysid == 0? YES (broadcast)
     │       └─► Accept
     │           └─► write_msg()
     │               └─► send(tcp_socket, ...)
     │
     └─► LogEndpoint [fd=6]
         └─► accept_msg()
             └─► Accept (logs all messages)
                 └─► write_msg()
                     └─► write(log_file, ...)
```

### Flow: Endpoint → Vehicle

```
┌──────────────┐
│ Ground       │
│ Station      │
│ (UDP)        │
└──────┬───────┘
       │ MAVLink Message
       │ (sysid=200, compid=1)
       │ target: sysid=100, compid=1
       ▼
┌─────────────────┐
│ UdpEndpoint     │
│ [fd=4]          │
└────┬────────────┘
     │ handle_read()
     │   ├─► read_msg()
     │   │   └─► Parse & validate
     │   │
     │   ├─► _add_sys_comp_id(200, 1)
     │   └─► route_msg(&buf)
     ▼
┌─────────────────┐
│   Mainloop      │
│   route_msg()   │
└────┬────────────┘
     │
     ├─► UdpEndpoint [fd=4] (source)
     │   └─► Reject (prevent loop)
     │
     ├─► UartEndpoint [fd=3] (Vehicle)
     │   └─► accept_msg()
     │       ├─► has_sys_comp_id(200,1)? NO ✓
     │       ├─► target_sysid == 100?
     │       ├─► has_sys_comp_id(100,1)? YES
     │       └─► Accept
     │           └─► write_msg()
     │               └─► write(uart_fd, ...)
     │
     └─► Other endpoints
         └─► Reject (target not known)
```

### Detailed Message Processing Pipeline

```
┌─────────────────────────────────────────────────────────────┐
│                    Message Processing Pipeline                │
└─────────────────────────────────────────────────────────────┘

1. I/O Event (epoll_wait)
   │
   ├─► EPOLLIN event on endpoint fd
   │
   ▼
2. Endpoint::handle_read()
   │
   ├─► read_msg(&buf)
   │   │
   │   ├─► Read raw bytes from fd
   │   │   └─► _read_msg() [UART: read(), UDP: recvfrom(), TCP: recv()]
   │   │
   │   ├─► Find MAVLink magic byte (STX)
   │   │   └─► Skip invalid data before magic byte
   │   │
   │   ├─► Parse header (MAVLink 1.0 or 2.0)
   │   │   ├─► Extract: msg_id, seq, sysid, compid, payload_len
   │   │   └─► Calculate expected packet size
   │   │
   │   ├─► Wait for complete packet (if partial)
   │   │
   │   ├─► Validate CRC
   │   │   └─► _check_crc()
   │   │       └─► Calculate CRC and compare
   │   │
   │   ├─► Extract target sysid/compid from payload
   │   │   └─► Use msg_entry->target_system_ofs
   │   │
   │   └─► Return buffer with parsed message info
   │
   ├─► allowed_by_dedup(&buf)
   │   └─► Mainloop::dedup_check_msg()
   │       └─► Dedup::check_packet()
   │           └─► Hash message, check if seen recently
   │
   ├─► allowed_by_incoming_filters(&buf)
   │   └─► Check AllowMsgIdIn, BlockMsgIdIn, etc.
   │
   ├─► _add_sys_comp_id(src_sysid, src_compid)
   │   └─► Add to _sys_comp_ids[] if not present
   │       └─► Also add to grouped endpoints
   │
   └─► Mainloop::route_msg(&buf)
       │
       ├─► For each endpoint in g_endpoints[]
       │   │
       │   ├─► endpoint->accept_msg(&buf)
       │   │   │
       │   │   ├─► Check if source is known (prevent loop)
       │   │   │   └─► Reject if yes
       │   │   │
       │   │   ├─► Check outgoing filters
       │   │   │   └─► Filtered if blocked/allowed rules match
       │   │   │
       │   │   ├─► Check broadcast (target_sysid == 0)
       │   │   │   └─► Accept if yes
       │   │   │
       │   │   ├─► Check if target sysid/compid is known
       │   │   │   └─► Accept if yes
       │   │   │
       │   │   ├─► Check if target sysid known, compid broadcast
       │   │   │   └─► Accept if yes
       │   │   │
       │   │   └─► Check sniffer sysid
       │   │       └─► Accept if endpoint has sniffer sysid
       │   │
       │   └─► If Accepted:
       │       └─► Mainloop::write_msg(endpoint, &buf)
       │           └─► endpoint->write_msg(&buf)
       │               │
       │               ├─► UartEndpoint: write(uart_fd, ...)
       │               ├─► UdpEndpoint: sendto(udp_socket, ...)
       │               ├─► TcpEndpoint: send(tcp_socket, ...)
       │               └─► LogEndpoint: write(log_file, ...)
       │
       └─► If all endpoints reject: count as "unknown message"
```

---

## Key Features

### 1. Message Filtering

Endpoints support filtering at two points:

**Incoming Filters** (applied before routing):
- `AllowMsgIdIn`: Only allow specific message IDs
- `BlockMsgIdIn`: Block specific message IDs
- `AllowSrcSysIn` / `BlockSrcSysIn`: Filter by source system ID
- `AllowSrcCompIn` / `BlockSrcCompIn`: Filter by source component ID

**Outgoing Filters** (applied after routing decision):
- `AllowMsgIdOut` / `BlockMsgIdOut`
- `AllowSrcSysOut` / `BlockSrcSysOut`
- `AllowSrcCompOut` / `BlockSrcCompOut`

### 2. Endpoint Groups

Endpoints can be grouped together to share system/component lists. This is useful for:
- **Redundant Links**: Multiple parallel data links (e.g., LTE + telemetry radio)
- **Load Balancing**: Multiple endpoints serving the same systems

When endpoints are grouped:
- They share `_sys_comp_ids` lists
- Messages from one endpoint in the group are not rejected by others (prevents routing rule #1 from blocking redundant links)

### 3. Message De-duplication

Optional feature to prevent duplicate messages:
- Uses hash-based deduplication
- Configurable time window (`DeduplicationPeriod`)
- Messages with identical content within the period are dropped
- Useful for redundant links or network retransmissions

### 4. Sniffer Mode

A special mode where an endpoint with a specific system ID receives **all** messages:
- Configured via `SnifferSysid`
- Useful for logging or monitoring all traffic
- Overrides normal routing rules

### 5. Statistics

Each endpoint tracks:
- **Received**: Total messages, CRC errors, sequence drops, bytes
- **Transmitted**: Total messages, bytes
- **Errors**: Messages to unknown endpoints

### 6. TCP Server

The router can act as a TCP server:
- Listens on a configurable port (default: 5760)
- Accepts dynamic client connections
- Each connection becomes a new `TcpEndpoint`
- Connections are removed when closed

### 7. Automatic Baudrate Switching (UART)

UART endpoints can try multiple baudrates:
- Configured as a list: `baud=115200,57600,38400`
- Automatically switches if no valid messages received
- Useful for autodetecting vehicle baudrate

---

## Configuration Example

```ini
[General]
TcpServerPort=5760
DebugLogLevel=info
DeduplicationPeriod=100
SnifferSysid=255

[UartEndpoint vehicle]
device=/dev/ttyUSB0
baud=115200,57600
FlowControl=false
AllowMsgIdOut=0,1,2,3

[UdpEndpoint gcs]
address=192.168.1.100
port=14550
mode=Client

[UdpEndpoint server]
address=0.0.0.0
port=24550
mode=Server

[TcpEndpoint qgc]
address=192.168.1.50
port=5760
RetryTimeout=5

[Log]
Log=/var/log/flight-stack
MavlinkDialect=Auto
LogTelemetry=true
```

---

## Summary

MAVLink Router is a sophisticated message routing system that:

1. **Centralizes** MAVLink communication through a single process
2. **Routes** messages intelligently based on target addressing
3. **Prevents** message loops by tracking message sources
4. **Filters** messages based on configurable rules
5. **Supports** multiple transport protocols seamlessly
6. **Logs** flight data for analysis
7. **Scales** to handle multiple vehicles and ground stations

The architecture is event-driven and efficient, using Linux epoll for high-performance I/O multiplexing. The routing logic ensures messages reach their intended destinations while preventing loops and unnecessary traffic.

